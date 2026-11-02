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
 * 只有 -13°，而量化噪声仍然小到可以忽略（17 位编码器、40 kHz：
 * 1 rev/s 时速度噪声约 0.05%，同条件下 19 点差分是 1.6%）。 */
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

/* 速度 PI：i_q = clamp(vel_gain*err + integ)。
 * 抗饱和用 ODrive 的做法：输出被限幅的那一拍积分器按 ML_INT_DECAY 衰减而不是
 * 继续累积，未限幅才正常积分。任何 NaN 输入都退化为 0 —— 绝不让 NaN 走进电流环。 */
static inline float ml_vel_pi(float vel_gain, float integrator_gain, float i_max,
                              float *integ, float v_cmd, float v_meas, float dt)
{
	if (!(v_cmd == v_cmd) || !(v_meas == v_meas) || !(i_max > 0.0f) || !(dt > 0.0f)) {
		*integ = 0.0f;
		return 0.0f;
	}

	float err = v_cmd - v_meas;
	float unlim = vel_gain * err + (*integ);
	float iq = ml_clampf(unlim, -i_max, i_max);

	if (iq != unlim) {
		*integ *= ML_INT_DECAY;
	} else {
		*integ += integrator_gain * err * dt;
	}
	*integ = ml_clampf(*integ, -i_max, i_max);

	return iq;
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
 *   pos_gain            = ω_c/20                    [(rad/s)/rad]
 *
 * 这两个 5 / 20 都是台架试出来的，不是教科书取值：线性串级设计会说
 * `vel_integrator_gain = vel_gain·ω_c/5`、`pos_gain = ω_c/5`，但这台
 * "小惯量 + 大摩擦 + 齿隙"的关节上：
 *   - 积分拐点开到 90 rad/s 就出现 stick-slip 极限环（速度误差反而涨到 13%），
 *     拐点压到 ~18 rad/s 才干净；
 *   - pos_gain 45 以上位置环就开始过冲（0.1 rad 阶跃过冲 33%~65%），
 *     22.5 时只有 1.3%，15 时 3%。
 * `margin` 只放大速度环：它补的是线性公式对**惯性主导**假设的低估（台架需要
 * ~2.6），位置环不跟着放大更安全。 */
static inline int ml_tune_gains(float j, float kt, float gr, float omega_c, float margin,
                                float *vel_gain, float *vel_i_gain, float *pos_gain)
{
	if (!(j > 0.0f) || !(kt > 0.0f) || !(gr > 0.0f) || !(omega_c > 0.0f)) { return 0; }
	if (!(margin > 0.0f)) { margin = 1.0f; }

	float w_i = ml_clampf(omega_c / 25.0f, 5.0f, 30.0f);

	*vel_gain = margin * j * omega_c / (kt * gr);
	*vel_i_gain = *vel_gain * w_i;
	*pos_gain = omega_c / 20.0f;
	return 1;
}

#endif /* MOTION_LAW_H */
