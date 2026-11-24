/*
 * motion_law.h
 *
 * 速度环 / 位置环的纯数学部分：钳位、速度设定值斜率限制、速度 PI（带抗饱和）、
 * 位置 P、位置跟随误差故障判定，以及编码器速度 PLL。
 *
 * 这里刻意不包含任何固件头文件（与 can_cyberbeast_codec.h 同一做法），host 侧
 * 测试 tools/test_motion_law.c 直接包含它 —— 测的就是固件跑的那份代码。
 *
 * 单位一律是**输出轴**量，与 torque_control() 的口径一致：
 *   pos_gain             [(rad/s)/rad]
 *   vel_gain             [A/(rad/s)]      控制输出是电机端相电流（安培域）
 *   vel_integrator_gain  [A/(rad/s)/s]
 *   accel_limit          [rad/s^2]        0 = 关闭
 *   pos_error_fault      [rad]            0 = 关闭
 *
 * 为什么在安培域而不在 N·m 域：CSP/CSV 帧里的限幅本来就是电流(A)，电流环、
 * field_weaken()、过流阈值也都在安培域，省掉每拍 2 次乘法 + 1 次除法，而且
 * 增益不依赖标定出来的 KT 是否准确。
 */

#ifndef MOTION_LAW_H
#define MOTION_LAW_H

#include <math.h>
#include <string.h>
#include <stdint.h>

#define ML_INT_DECAY        0.99f   /* 输出饱和时速度积分器的衰减因子（ODrive 同款） */
#define ML_POS_FAULT_TIME   0.1f    /* 位置跟随误差故障去抖时间（秒） */

/* 速度环/位置环的缺省 PLL 带宽。config.pll_bandwidth 为 0/非法时用它，
 * 与 params.json 的默认值保持一致。
 *
 * 为什么是 2000 而不是 ODrive 的 500：速度环的相位预算里，估计器的滞后
 * 是叠加在“积分器固有的 -90°”上的。临界阻尼 PLL 在 ω = ωn 处相位 -27°，
 * 在 ω = 3.3·ωn 处约 -58°。取 500 rad/s（80 Hz）时，200 Hz 的环路穿越点已经
 * 吃到 -58°，可用带宽被压到 120 Hz 左右；取 2000 rad/s（318 Hz）时 200 Hz 处
 * 只有 -13°。
 *
 * 噪声（2026-11-06 台架实测）不是带宽能解决的问题：编码器原始计数本身就有
 * σ≈6 count（16 bit，0.033°）的抖动、τ≈0.46 ms，折成速度噪声在 2000 带宽下
 * 就是 0.40 rad/s（电机侧，输出轴 0.044）。别用它换：实测同一 vel_gain 下把
 * 带宽降到 1000 会让阶跃超调从 6% 涨到 35~43%（反馈滞后比噪声有害得多），
 * 降到 500 直接失稳。详见 docs/DEV.md 的“编码器速度 PLL”。 */
#define ML_PLL_BW_DEFAULT   2000.0f

static inline float ml_clampf(float x, float lo, float hi)
{
	if (x < lo) { return lo; }
	if (x > hi) { return hi; }
	return x;
}

/* 速度设定值的斜率限制。accel <= 0 或目标非有限值时直接跟随 —— 后者保证一个
 * 意外的 NaN 指令不会把设定值永久锁在上一次的值上。 */
static inline float ml_slew(float current, float target, float accel, float dt)
{
	if (!(target == target)) { return current; }
	if (!(accel > 0.0f)) { return target; }
	float step = accel * dt;
	float delta = target - current;
	if (delta > step) { delta = step; }
	if (delta < -step) { delta = -step; }
	return current + delta;
}

/* 加速/减速分开的斜率限制（config.accel_limit / config.decel_limit）。
 * "在减速"的判据：目标与当前同号（或目标为 0，即往零收）且 |target| < |current|。
 * 反向跨越零点按加速处理（那是两段运动，用 accel 更保守）。
 * decel <= 0 时退回 accel，所以只配 accel 的老参数含义不变。 */
static inline float ml_slew2(float current, float target, float accel, float decel, float dt)
{
	if (!(target == target)) { return current; }
	float rate = accel;
	if ((decel > 0.0f) && ((target * current) >= 0.0f)
	    && (fabsf(target) < fabsf(current))) {
		rate = decel;
	}
	if (!(rate > 0.0f)) { return target; }
	float step = rate * dt;
	float delta = target - current;
	if (delta > step) { delta = step; }
	if (delta < -step) { delta = -step; }
	return current + delta;
}

/* 专用限值的选择规则（config.vel_ramp_rate / trap_accel_limit / trap_decel_limit）：
 * override > 0 用它，否则退回通用的 accel/decel_limit。0 / 负数 / NaN 都算"未设置"
 * —— 默认就是 0，所以没有配这两组新参数时行为与改动前完全一致。 */
static inline float ml_limit_or(float override, float fallback)
{
	return (override > 0.0f) ? override : fallback;
}

/* 梯形规划的巡航上限：显式值只在**更严格**时才生效（取 min）。这与 ODrive 一致
 * —— 它的 trap_traj.vel_limit 之后还会被 controller.vel_limit 再夹一次。
 * cfg <= 0 时直接用运行期限速（loop_lim），不会出现"轨迹要求的速度环路给不出"。 */
static inline float ml_trap_vel(float cfg, float loop_lim)
{
	if (!(cfg > 0.0f)) { return loop_lim; }
	return (cfg < loop_lim) ? cfg : loop_lim;
}

/* 位置指令的允许区间（输出轴 rad）：
 *   界内      -> [p_min, p_max]
 *   已在界外  -> 放宽到"当前实测位置"这一侧，即允许给一个**仍在界外、但朝界内**
 *                方向的目标（例：限位 0..10、当前 15，可以命令 12），但目标不能
 *                比当前位置更外（当前 12 时不能再命令 13）。
 * p_max <= p_min（软限位关闭）时返回 ±FLT_MAX，调用方原样通过。
 *
 * 通信层写 p_des 时用同一套界（foc.c 的 pos_cmd_lo/hi/limit）；控制层每拍用
 * ml_pos_cmd_clamp() 重算 —— 位置在动，写入口那一刻的界可能已经变了。 */
static inline void ml_pos_cmd_range(float p_meas, float p_min, float p_max,
                                    float *lo, float *hi)
{
	*lo = -3.40282347e+38f;
	*hi =  3.40282347e+38f;
	if (!(p_max > p_min)) { return; }
	*lo = p_min;
	*hi = p_max;
	if (p_meas > p_max) { *hi = p_meas; }			// 正限位外：目标不得比现在更外
	else if (p_meas < p_min) { *lo = p_meas; }		// 负限位外同理
}

static inline float ml_pos_cmd_clamp(float p, float p_meas, float p_min, float p_max)
{
	float lo, hi;
	ml_pos_cmd_range(p_meas, p_min, p_max, &lo, &hi);
	if (p < lo) { return lo; }
	if (p > hi) { return hi; }
	return p;
}

/* 实测位置是否已经在软限位之外。p_max <= p_min 表示软限位关闭（与
 * ml_pos_cmd_range() 同一判据），永远返回 0。切到 CSP 时用它决定要不要
 * 冻结位置参考（界外冻结 -> 不强制回限位，见 motion.c 的 pos_hold）。 */
static inline int ml_pos_outside(float p, float p_min, float p_max)
{
	if (!(p_max > p_min)) { return 0; }
	return (p > p_max) || (p < p_min);
}

/* 位置软限位的**单向门**：已经越过限位时只放行"往回收"的方向，不允许继续
 * 往外推；界内原样通过（两个方向都不限制）。
 *
 * 它作用在"正号 = 使位置增大"的任何指令上，所以速度环喂 `v_setpoint`
 * （rad/s）、阻抗/力矩环喂 `i_q_des`（A）都是同一个函数 —— 两处只用各自的
 * 单位，判据完全一样。用**实测位置**判方向（不是设定值），只丢越界方向的那
 * 一拍，指令本身（`v_des` / `t_ff`）不动：主站随时可以命令回转，回转方向
 * 永远放行。
 *
 * 为什么 CSV / 纯力矩需要它：CSV 没有位置目标，`p_des` 的钳位管不到速度指令；
 * 纯力矩（CST，kp = kd = 0）同样只看 `t_ff`，一个持续的值会把关节一路推出
 * `config.p_min/p_max`。MIT 有 kp，`p_des` 被钳在限位内，正常工作时这道门
 * 不动作（越过限位时弹簧本来就给反向力矩）。
 *
 * 判据与 ml_pos_cmd_range() 一致：p_max <= p_min 表示软限位
 * 关闭，原样通过。NaN 位置同样原样通过（交给下游的 NaN 保护）。 */
static inline float ml_pos_dir_gate(float cmd, float p_meas, float p_min, float p_max)
{
	if (!(p_max > p_min)) { return cmd; }		// 软限位关闭
	if (p_meas >= p_max) {						// 已到/越过正限位：只许回转（cmd <= 0）
		return (cmd > 0.0f) ? 0.0f : cmd;
	}
	if (p_meas <= p_min) {						// 已到/越过负限位：只许回转（cmd >= 0）
		return (cmd < 0.0f) ? 0.0f : cmd;
	}
	return cmd;
}

/* 速度 PI 带前馈：i_q = clamp(vel_gain*err + integ + i_ff)。
 * i_ff（摩擦/重力前馈）参与**限幅判断**，所以饱和时积分器不会因为"前馈把输出
 * 顶出去"而误以为是误差造成的、继续累积；不饱和时前馈不影响积分。 */
static inline float ml_vel_pi_ff(float vel_gain, float integrator_gain, float i_max,
                                 float *integ, float v_cmd, float v_meas, float i_ff,
                                 float dt)
{
	if (!(v_cmd == v_cmd) || !(v_meas == v_meas) || !(i_max > 0.0f) || !(dt > 0.0f)) {
		*integ = 0.0f;
		return 0.0f;
	}
	if (!(i_ff == i_ff)) { i_ff = 0.0f; }

	float err = v_cmd - v_meas;
	float unlim = vel_gain * err + (*integ) + i_ff;
	float iq = ml_clampf(unlim, -i_max, i_max);

	if (iq != unlim) {
		*integ *= ML_INT_DECAY;
	} else {
		*integ += integrator_gain * err * dt;
	}
	*integ = ml_clampf(*integ, -i_max, i_max);

	return iq;
}

/* 速度 PI：i_q = clamp(vel_gain*err + integ)。
 * 抗饱和用 ODrive 的做法：输出被限幅的那一拍积分器按 ML_INT_DECAY 衰减而不是
 * 继续累积，未限幅才正常积分。任何 NaN 输入都退化为 0 —— 绝不让 NaN 走进电流环。 */
static inline float ml_vel_pi(float vel_gain, float integrator_gain, float i_max,
                              float *integ, float v_cmd, float v_meas, float dt)
{
	return ml_vel_pi_ff(vel_gain, integrator_gain, i_max, integ, v_cmd, v_meas, 0.0f, dt);
}

/* 位置 P：速度指令 = pos_gain*位置误差 + 前馈，单位 (rad/s) */
static inline float ml_pos_p(float pos_gain, float pos_ff_gain, float pos_err, float v_ff)
{
	if (!(pos_err == pos_err)) { return 0.0f; }
	if (!(v_ff == v_ff)) { v_ff = 0.0f; }
	return pos_gain * pos_err + pos_ff_gain * v_ff;
}

/* 位置跟随误差故障：只在位置环**没有**被速度限幅饱和时计时，连续
 * ML_POS_FAULT_TIME 秒超过阈值才判定。饱和意味着“正在按速度上限追赶”，这时
 * 误差大是正常的（长距离移动、上电对齐），不能算故障；limit <= 0 关闭。
 * 返回 1 表示应当锁存故障。 */
static inline int ml_pos_fault(float limit, float *timer, float pos_err,
                               int saturated, float dt)
{
	if (!(limit > 0.0f) || saturated || !(pos_err == pos_err)
	    || !(fabsf(pos_err) > limit)) {
		*timer = 0.0f;
		return 0;
	}

	*timer += dt;
	return (*timer > ML_POS_FAULT_TIME) ? 1 : 0;
}

/* ---- 输入整形：位置轨迹发生器 ------------------------------------------
 *
 * POS_FILTER 与 TRAP_TRAJ 都输出**位置 + 速度**设定（输出轴 rad / rad/s），交给
 * 现有的位置环去跟踪，所以电流环与速度环一行都不用改。两种模式都可能在运动中
 * 被改目标，因此状态要从**上一次的输出**续起（而不是从 0），否则会跳。
 */

/* 梯形位置规划器：ODrive TrapezoidalTrajectory（trapTraj.cpp）的移植。
 * 按 vel/accel/decel 三个限值把"以当前速度从当前点走到目标"拆成
 * 加速 / 匀速 / 减速三段；行程不够时自动退化成三角形（无匀速段），
 * 起始速度已经高于巡航速度时会先减速（"double deceleration move"）。 */
typedef struct{
	float xi, xf, vi;		// 起点、终点、起始速度
	float ar, vr, dr;		// 实际使用的（带符号）加速度 / 巡航速度 / 减速度
	float ta, tv, td, tf;	// 加速 / 匀速 / 减速 / 总时长
	float y_accel;			// 加速段结束时的位置
	float t;				// 规划内时间
	float goal;				// 上次规划的目标（指令变了就重规划）
	uint8_t planned;		// 0 = 还没规划过（第一拍要从实测位置起步）
} ml_trap_t;

/* 二阶位置跟踪滤波（ODrive INPUT_MODE_POS_FILTER）的状态 */
typedef struct{
	float pos, vel;
	uint8_t seeded;
} ml_posf_t;

/* 与 ODrive 的 sign_hard 一致：-0.0 也算负号（否则 dx == 0 的边界会翻方向） */
static inline float ml_sign_hard(float v)
{
	return signbit(v) ? -1.0f : 1.0f;
}

/* 规划一条梯形轨迹。参数非法（vel/accel/decel 非正、输入含 NaN）时返回 0，
 * 调用方应退化成直接跟随。 */
static inline int ml_trap_plan(ml_trap_t *tr, float xf, float xi, float vi,
                               float vmax, float amax, float dmax)
{
	if (!(vmax > 0.0f) || !(amax > 0.0f) || !(dmax > 0.0f)) { return 0; }
	if (!(xf == xf) || !(xi == xi) || !(vi == vi)) { return 0; }

	float dx = xf - xi;
	float stop_dist = (vi*vi) / (2.0f*dmax);		// 当前速度下的最小制动距离
	float dxstop = copysignf(stop_dist, vi);
	float s = ml_sign_hard(dx - dxstop);			// 巡航速度的方向

	tr->ar = s*amax;
	tr->dr = -s*dmax;
	tr->vr = s*vmax;

	/* 起始速度已经比巡航速度快 → 这一段其实是减速 */
	if ((s*vi) > (s*tr->vr)) { tr->ar = -s*amax; }

	tr->ta = (tr->vr - vi) / tr->ar;
	tr->td = -tr->vr / tr->dr;
	float dxmin = 0.5f*tr->ta*(tr->vr + vi) + 0.5f*tr->td*tr->vr;

	if (s*dx < s*dxmin) {
		/* 行程不够：三角形曲线，去掉匀速段 */
		float den = tr->dr - tr->ar;
		float num = tr->dr*(vi*vi) + 2.0f*tr->ar*tr->dr*dx;
		tr->vr = s*sqrtf(fmaxf((den != 0.0f) ? (num/den) : 0.0f, 0.0f));
		tr->ta = fmaxf(0.0f, (tr->vr - vi) / tr->ar);
		tr->td = fmaxf(0.0f, -tr->vr / tr->dr);
		tr->tv = 0.0f;
	}
	else {
		tr->tv = (dx - dxmin) / tr->vr;
	}

	tr->tf = tr->ta + tr->tv + tr->td;
	tr->xi = xi;
	tr->xf = xf;
	tr->vi = vi;
	tr->y_accel = xi + vi*tr->ta + 0.5f*tr->ar*tr->ta*tr->ta;
	tr->t = 0.0f;
	tr->goal = xf;
	tr->planned = 1;
	return 1;
}

/* 按规划内时间 t 求 (位置, 速度, 加速度)；指针可为 NULL */
static inline void ml_trap_eval(const ml_trap_t *tr, float t,
                                float *pos, float *vel, float *acc)
{
	float y, yd, ydd;

	if (t < 0.0f) {								// 初始条件
		y = tr->xi;  yd = tr->vi;  ydd = 0.0f;
	}
	else if (t < tr->ta) {						// 加速
		y = tr->xi + tr->vi*t + 0.5f*tr->ar*t*t;
		yd = tr->vi + tr->ar*t;
		ydd = tr->ar;
	}
	else if (t < (tr->ta + tr->tv)) {			// 匀速
		y = tr->y_accel + tr->vr*(t - tr->ta);
		yd = tr->vr;
		ydd = 0.0f;
	}
	else if (t < tr->tf) {						// 减速（相对终点的反向时间）
		float td = t - tr->tf;
		y = tr->xf + 0.5f*tr->dr*td*td;
		yd = tr->dr*td;
		ydd = tr->dr;
	}
	else {										// 结束
		y = tr->xf;  yd = 0.0f;  ydd = 0.0f;
	}

	if (pos) { *pos = y; }
	if (vel) { *vel = yd; }
	if (acc) { *acc = ydd; }
}

/* 推进一拍（在位置环频率上调用）。目标变了会从**上一拍的输出**重新规划。
 * 返回 1 = 还在走，0 = 已到终点（此时 *pos 精确等于目标）。
 * 时间在结束时冻结在 tf，不会无限增长（float32 长时间累加会丢精度）。 */
static inline int ml_trap_step(ml_trap_t *tr, float dt, float goal,
                               float vmax, float amax, float dmax,
                               float *pos, float *vel)
{
	if (!tr->planned || !(goal == tr->goal)) {
		float xi = pos ? *pos : goal;
		float vi = vel ? *vel : 0.0f;
		if (!ml_trap_plan(tr, goal, xi, vi, vmax, amax, dmax)) {
			/* 参数非法：直接跟随目标，至少不能把关节卡在旧设定值上 */
			tr->planned = 0;
			if (pos) { *pos = goal; }
			if (vel) { *vel = 0.0f; }
			return 0;
		}
	}

	if (tr->t >= tr->tf) {
		if (pos) { *pos = tr->xf; }
		if (vel) { *vel = 0.0f; }
		return 0;
	}

	ml_trap_eval(tr, tr->t, pos, vel, 0);
	tr->t += dt;
	/* 时间冻结在 tf：长时间停在终点时 float32 也不会丢精度 */
	if (tr->t > tr->tf) { tr->t = tr->tf; }
	return 1;
}

/* 把滤波状态对齐到 (pos, vel)：进入 POS_FILTER 时从实测位置起步 */
static inline void ml_posf_reset(ml_posf_t *f, float pos, float vel)
{
	f->pos = pos;
	f->vel = vel;
	f->seeded = 1;
}

/* 二阶跟踪滤波一拍：ki = 2*bw，kp = ki²/4（临界阻尼，与 ODrive 相同）。
 * 离散稳定要求 kp*dt <= 0.5 ⇔ ki <= sqrt(2/dt)；超过就压低 ki（4 kHz 位置环上
 * bw 的实际上限约 45 rad/s），否则会自激。 */
static inline void ml_posf_step(ml_posf_t *f, float target_pos, float target_vel,
                                float bw, float dt, float *pos, float *vel)
{
	if (!(bw > 0.0f)) { bw = 10.0f; }
	float ki = 2.0f*bw;
	float kp = 0.25f*ki*ki;
	if ((dt > 0.0f) && ((kp*dt) > 0.5f)) {
		ki = sqrtf(2.0f/dt);
		kp = 0.25f*ki*ki;
	}
	if (!(target_pos == target_pos)) { target_pos = f->pos; }
	if (!(target_vel == target_vel)) { target_vel = 0.0f; }

	float dpos = target_pos - f->pos;
	float dvel = target_vel - f->vel;
	float acc = kp*dpos + ki*dvel;

	f->vel += acc*dt;
	f->pos += f->vel*dt;

	if (pos) { *pos = f->pos; }
	if (vel) { *vel = f->vel; }
}

/* 临界阻尼 PLL 增益：kp = 2*bw，ki = 0.25*kp^2（bw 单位 rad/s，与 ODrive 一致）。
 * dt*kp 必须 < 1 才谈得上离散稳定；这里额外留一倍余量，超了就把 kp 压到 0.5/dt。 */
static inline void ml_pll_gains(float bandwidth, float dt, float *kp, float *ki)
{
	if (!(bandwidth > 0.0f)) { bandwidth = ML_PLL_BW_DEFAULT; }

	float kp_value = 2.0f * bandwidth;
	if ((dt > 0.0f) && (kp_value * dt > 0.5f)) { kp_value = 0.5f / dt; }

	*kp = kp_value;
	*ki = 0.25f * kp_value * kp_value;
}

/* 编码器速度 PLL 的一拍（25 us）。输入是**单圈**计数（[0,CPR)），输出速度单位
 * counts/s。用单圈计数而不是多圈浮点位置：|count| 超过 2^24 后 float32 已经
 * 丢整数分辨率（65536*256 就超了），位置一大相位检测就变成台阶。
 *
 * 顺序与 ODrive 的 Encoder::update() 一致：先按当前速度预测，再用相位差同时
 * 修正位置和速度；速度小于一个增量时归零，避免静止时的抖动（零速吸附）。 */
static inline void ml_pll_step(float *pos_cpr, float *vel_counts, float count_cpr,
                               float cpr, float kp, float ki, float dt)
{
	float half = cpr * 0.5f;

	*pos_cpr += dt * (*vel_counts);

	float delta = count_cpr - (*pos_cpr);
	if (delta > half) { delta -= cpr; }
	else if (delta < -half) { delta += cpr; }

	*pos_cpr += dt * kp * delta;
	*vel_counts += dt * ki * delta;

	if (*pos_cpr >= cpr) { *pos_cpr -= cpr; }
	else if (*pos_cpr < 0.0f) { *pos_cpr += cpr; }

	if (fabsf(*vel_counts) < 0.5f * dt * ki) { *vel_counts = 0.0f; }
}

/* ---- 惯量标定用的流式最小二乘 ------------------------------------------
 *
 * 模型（输出侧）：J·dω/dt = KT·GR·i_q − c，其中 c 是重力 + 库仑摩擦（常值）。
 * 用**同一方向上的两个电流台阶**测两条加速度，拟合 a(i) 的斜率就能把 c 消掉：
 *
 *     J = KT·GR·(i_hi − i_lo) / (a_hi − a_lo)
 *
 * 比 ODrive 的 ±对称法更准：那里库仑摩擦的方向随运动反向，对称相加抵不掉，
 * 会把 J 高估（高估多少取决于摩擦占力矩的比例，台架上可以到 30%）。
 *
 * 拟合只累计 5 个和，不缓存样本（ISR 里跑，不能开数组）。 */

typedef struct{
	float n;		// 样本数
	float st, sw;		// Σt, Σω
	float stt, stw;		// Σt², Σt·ω
} ml_fit_t;

static inline void ml_fit_reset(ml_fit_t *fit)
{
	fit->n = 0.0f;
	fit->st = 0.0f;
	fit->sw = 0.0f;
	fit->stt = 0.0f;
	fit->stw = 0.0f;
}

static inline void ml_fit_add(ml_fit_t *fit, float t, float w)
{
	fit->n += 1.0f;
	fit->st += t;
	fit->sw += w;
	fit->stt += t * t;
	fit->stw += t * w;
}

/* 斜率 dω/dt；样本不足或分母退化时返回 0 */
static inline float ml_fit_slope(const ml_fit_t *fit)
{
	if (fit->n < 2.0f) { return 0.0f; }
	float den = fit->n * fit->stt - fit->st * fit->st;
	if (!(den > 1.0e-9f)) { return 0.0f; }
	return (fit->n * fit->stw - fit->st * fit->sw) / den;
}

/* 由两个电流台阶的加速度求惯量（输出端 kg·m²）。返回 1 = 成功。
 * 阈值故意宽（只要量级对就行）：标定值只是给增益一个起始点，最终还要按
 * 穿越频率和相位余量折中。 */
static inline int ml_inertia_from_slopes(float kt, float gr, float i_lo, float a_lo,
                                         float i_hi, float a_hi, float *j_out)
{
	if (!(kt > 0.0f) || !(gr > 0.0f)) { return 0; }
	if (!(i_hi > i_lo + 0.02f)) { return 0; }			/* 两个台阶差太小，噪声主导 */
	if (!(a_hi > a_lo + 0.5f)) { return 0; }			/* 加速度差太小，摩擦可能没突破 */

	float j = kt * gr * (i_hi - i_lo) / (a_hi - a_lo);
	if (!(j > 1.0e-6f) || !(j < 100.0f)) { return 0; }

	*j_out = j;
	return 1;
}

/* 由惯量算外环增益（`action.tune_gains`）。返回 1 = 成功。
 *
 *   vel_gain            = margin·J·ω_c/(KT·GR)      [A/(rad/s)]
 *   vel_integrator_gain = vel_gain·clamp(ω_c/25, 5, 30)   [A/(rad/s)/s]
 *   pos_gain            = ω_c/30                    [(rad/s)/rad]
 *
 * 这几个分母都是台架试出来的，不是教科书取值。线性串级设计会说
 * `vel_integrator_gain = vel_gain·ω_c/5`、`pos_gain = ω_c/5`，但实测：
 *   - 积分拐点开到 90 rad/s 就出现 stick-slip 极限环（速度误差反而涨到 13%），
 *     拐点压到 ~18 rad/s 才干净（ω_c/25）；
 *   - pos_gain 22.5（=ω_c/20）在 accel_limit 10 时 0.2 rad 阶跃过冲 29%、
 *     1 rad 阶跃过冲 29~49%；15（=ω_c/30）分别是 4% / 4%。
 *     ⚠ 位置环本身是阻尼良好的（把 accel_limit 放到 200，45 也只有 0.1% 过冲），
 *     过冲主要来自"速度设定值被斜率限制拖慢"——所以 accel_limit 太小也会
 *     表现为位置环过冲。
 * `margin` 只放大速度环：它补的是线性公式对**惯性主导**假设的低估（两块板
 * 实测都要 ~2.2~2.6），位置环不跟着放大更安全。 */
static inline int ml_tune_gains(float j, float kt, float gr, float omega_c, float margin,
                                float *vel_gain, float *vel_i_gain, float *pos_gain)
{
	if (!(j > 0.0f) || !(kt > 0.0f) || !(gr > 0.0f) || !(omega_c > 0.0f)) { return 0; }
	if (!(margin > 0.0f)) { margin = 1.0f; }

	float w_i = ml_clampf(omega_c / 25.0f, 5.0f, 30.0f);

	*vel_gain = margin * j * omega_c / (kt * gr);
	*vel_i_gain = *vel_gain * w_i;
	*pos_gain = omega_c / 30.0f;
	return 1;
}

/* ---- 自适应窗口最小二乘测速 ---------------------------------------------
 *
 * 为什么不用（只调低带宽的）PLL：16 位编码器静止时原始计数就有 σ≈4.3 count
 * 的抖动，二阶 PLL 的**速度**噪声 ∝ bw^1.5。台架实测 bw=3000 时静止速度
 * σ=0.053 rad/s（输出轴，约 0.5 RPM）、峰峰 2.9 RPM —— 客户看到的"静止 1 RPM
 * 波动"就是它。把带宽降下来能压噪声，但相位滞后会跟着涨，450 rad/s 的速度环
 * 立刻失稳（docs/DEV.md 有 1k/2k/4k 的自激频率实测）。
 *
 * 这里改成**位置的最小二乘斜率**：对最近 m 个"块平均计数"做直线拟合，斜率就是
 * 速度。噪声 σ_slope ≈ σ_blk·√12/(m^1.5·dt)，样本越多噪声掉得越快（PLL 是
 * 带宽换噪声，这里是窗口长度换噪声），而窗口长度可以随速度自适应：
 *   - 高速：m 小（0.5~1 ms），噪声已经远低于 0.1%，滞后可以忽略；
 *   - 低速：m 大（10~100 ms），把噪声按 1/m^1.5 压到目标值，代价是滞后增大
 *     —— 但低速时机械时间常数本来就大，外环按 ml_lag_gain_scale() 降带宽即可。
 * 静止时如果计数完全不动，斜率**精确为 0**（不像 PLL 会一直抖）。
 *
 * ml_ls_window：给定速度与块噪声，选满足相对噪声目标的最短窗口（块数）。
 * speed 单位 counts/s，dt_blk 单位 s。 */
static inline float ml_pow_two_thirds(float x)
{
	if (!(x > 1.0f)) { return 1.0f; }

	uint32_t u;
	memcpy(&u, &x, 4);
	int e = (int)((u >> 23) & 0xFFu) - 127;
	u = (u & 0x007FFFFFu) | 0x3F800000u;		/* 尾数还原到 [1,2) */
	float mant;
	memcpy(&mant, &u, 4);
	if (mant > 1.4142136f) { mant *= 0.5f; e += 1; }	/* 收窄到 [0.707, 1.414) */

	/* log2(m) = 2/ln2 * (t + t^3/3 + ...)，t = (m-1)/(m+1)，|t| <= 0.172：
	 * 取到 t^3 的误差 <1e-4。一次除法远小于 powf 的几百个周期。 */
	float t = (mant - 1.0f) / (mant + 1.0f);
	float t2 = t * t;
	float y = ((float)e + 2.8853901f * t * (1.0f + t2 * (1.0f / 3.0f))) * 0.66666667f;

	int n = (int)y;
	float f = y - (float)n;
	/* 2^f 的五阶泰勒（f 在 [0,1)，误差 <2e-4） */
	float p2 = 1.0f + f * (0.6931472f + f * (0.2402265f + f * (0.0555041f
	          + f * (0.0096181f + f * 0.0013334f))));
	uint32_t su = (uint32_t)(n + 127) << 23;
	float scale;
	memcpy(&scale, &su, 4);
	return p2 * scale;
}

/* ml_ls_window：给定速度与块噪声，选满足相对噪声目标的最短窗口（块数）。
 * speed 单位 counts/s，dt_blk 单位 s。
 * ⚠ x^(2/3) 用 ml_pow_two_thirds()（不用 powf：libm 在 40 kHz 中断里太贵，
 * 台架实测 1 RPM + dither 时 total_max 会被它顶到 5987/6000）。 */
static inline int ml_ls_window(float speed_abs, float dt_blk, float sigma_blk,
                               float rel, int m_max)
{
	if (!(rel > 0.0f)) { rel = 0.004f; }
	if (!(dt_blk > 0.0f)) { return m_max; }
	if (!(speed_abs > 1.0e-6f)) { return m_max; }

	/* m^1.5 = 3.464*sigma/(rel*v*dt)；need < 2^1.5 时直接取最小窗口 */
	float need = 3.4641016f * sigma_blk / (rel * speed_abs * dt_blk);
	if (need <= 2.8284271f) { return (m_max < 2) ? m_max : 2; }
	float m = ml_pow_two_thirds(need);
	int mi = (int)(m + 0.5f);
	if (mi < 2) { mi = 2; }
	if (mi > m_max) { mi = m_max; }
	return mi;
}

/* 滑动窗口最小二乘的增量状态（**整数**，避免浮点舍入随运行时间累积）。
 *
 * 对窗口内的**位置**样本 p_i（i = 年龄，0 = 最新）做直线拟合，速度 = -dp/dt。
 * 位置可以写成 p_i = p_0 − Σ_{k<i} y_k（y_k 是每块的计数增量），代进最小二乘
 * 的正规方程后，斜率只依赖三个和：
 *
 *   A = Σ y_k,  B = Σ k·y_k,  C = Σ k²·y_k        （k = 0…m−1）
 *   v = 12·Σu_k y_k / (dt·m(m²−1)),  u_k = (k+1)(m−1−k)/2
 *   Σu_k y_k = [(m−2)·B − C + (m−1)·A] / 2
 *
 * ⚠ 权重是**三角形**（两端小、中间大），不是线性的 —— 早期版本按"对增量本身
 * 做线性拟合"写，静止时看似正确（0），一转起来就恒为 0。这个等价形式还有一个
 * 好处：稳态时分子是满量程（不靠大数相消），float 转换不会丢精度。
 *
 * 每块 O(1) 更新（滑窗 + 必要时加长/缩短一块）：
 *   滑窗：B += A − m·y_out,  C += 2B + A − m²·y_out,  A += y_in − y_out
 *   加长：A += u, B += m·u, C += m²·u, m++
 *   缩短：m−−, A −= u, B −= m·u, C −= m²·u
 *
 * ⚠ 用 int64：C 在高速下会到 m³/3·y（本机 m=1023、6 kcount/块 时约 2e12），
 * int32 早就溢出了。 */
typedef struct{
	int m;
	int64_t a;
	int64_t b;
	int64_t c;
} ml_ls_t;

static inline void ml_ls_reset(ml_ls_t *s)
{
	s->m = 0;
	s->a = 0;
	s->b = 0;
	s->c = 0;
}

/* 窗口滑一块：y_in 是新样本，y_out 是离开窗口的样本（还没填满时传 0）*/
static inline void ml_ls_slide(ml_ls_t *s, int32_t y_in, int32_t y_out)
{
	const int64_t yo = y_out;
	const int64_t b_old = s->b;
	s->c += 2 * b_old + s->a - (int64_t)s->m * s->m * yo;
	s->b = b_old + s->a - (int64_t)s->m * yo;
	s->a += (int64_t)y_in - yo;
}

/* 加长一块：u 是"年龄 = m"的样本（紧邻当前窗口的那块）*/
static inline void ml_ls_grow(ml_ls_t *s, int32_t u)
{
	s->a += u;
	s->b += (int64_t)s->m * u;
	s->c += (int64_t)s->m * s->m * u;
	s->m++;
}

/* 缩短一块：u 是原窗口里最老的那块（年龄 = m-1）*/
static inline void ml_ls_shrink(ml_ls_t *s, int32_t u)
{
	s->m--;
	s->a -= u;
	s->b -= (int64_t)s->m * u;
	s->c -= (int64_t)s->m * s->m * u;
}

/* 斜率（块计数 / 秒）：v = 6·N / [dt·m(m²−1)]，N = (m−2)b − c + (m−1)a */
static inline float ml_ls_slope(const ml_ls_t *s, float dt_blk)
{
	const int m = s->m;
	if (m < 2) { return 0.0f; }
	float den = dt_blk * (float)m * (float)(m * m - 1) * (1.0f / 6.0f);
	if (!(den > 0.0f)) { return 0.0f; }
	int64_t num = (int64_t)(m - 2) * s->b - s->c + (int64_t)(m - 1) * s->a;
	return (float)num / den;
}

/* 窗口的等效相位滞后（秒）：窗口本身的群延迟 m·dt/2，加上块平均 dt/2，
 * 再加上外环 1 kHz 左右更新带来的零阶保持（按 0.5 ms 计） */
static inline float ml_ls_lag(int m, float dt_blk)
{
	return (0.5f * (float)m + 0.5f) * dt_blk + 0.0005f;
}

/* 按测速滞后限制外环穿越频率：滞后在 ω_c 处贡献 ω_c·T_lag 的相位，取
 * ≤ 0.35 rad（20°）作为预算。返回速度环增益的缩放系数（1 = 不降）。
 * 下限 0.02 只是防呆，低于它说明窗口太长、参数不合理。 */
static inline float ml_lag_gain_scale(float omega_c, float lag)
{
	if (!(lag > 0.0f) || !(omega_c > 0.0f)) { return 1.0f; }
	float g = (0.35f / lag) / omega_c;
	if (g > 1.0f) { g = 1.0f; }
	if (g < 0.02f) { g = 0.02f; }
	return g;
}

/* 摩擦前馈：i_q_ff = viscous·v + coulomb·sat(v/eps)。
 * 库仑项在 ±eps 内线性过渡而不是直接取符号 —— 速度设定值过零时电流跟着平滑
 * 过零，不会像 sign() 那样在 0 附近来回跳（那是可听的"咔哒"和额外的振动）。
 * 输出是**电机端相电流**（与速度环同一量纲），用输出轴速度作自变量。 */
static inline float ml_friction_ff(float coulomb, float viscous, float eps, float v)
{
	if (!(v == v)) { return 0.0f; }
	float ff = (viscous > 0.0f) ? (viscous * v) : 0.0f;
	if ((coulomb > 0.0f)) {
		if (!(eps > 0.0f)) {
			ff += (v > 0.0f) ? coulomb : ((v < 0.0f) ? -coulomb : 0.0f);
		} else {
			float x = v / eps;
			if (x > 1.0f) { x = 1.0f; }
			else if (x < -1.0f) { x = -1.0f; }
			ff += coulomb * x;
		}
	}
	return ff;
}

#endif /* MOTION_LAW_H */
