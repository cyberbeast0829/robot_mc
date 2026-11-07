/*
 * motion.c
 *
 * MOTOR_MODE 的控制律分发与串级外环（CSP = 位置环 + 速度环，CSV = 速度环）。
 *
 * 分工：
 *   - 电流环（commutate()）一行不动，仍在 40 kHz 的同一个中断里、同样的采样
 *     相位与预装载时序。外环只负责算出 i_q_des 的参考值。
 *   - 速度环 / 位置环按 config.vel_loop_prescaler、config.pos_loop_prescaler
 *     分频运行（默认 40 kHz / 5 = 8 kHz，再 / 2 = 4 kHz），相位锁在控制
 *     中断计数器上，不用额外的定时器。速度估计的滞后比这大一个数量级，
 *     所以分频不会降低外环可用带宽，反而把 ISR 平均负载降下来了。
 *   - CTRL_IMPEDANCE（MIT / legacy / 0x03 / 0x04）仍然每拍执行 torque_control()，
 *     行为与改动前逐字节一致。
 *
 * 实时性：全部在控制中断里跑，不 printf、不写 flash、不阻塞。
 */

#include "foc.h"
#include "motion_law.h"
#include "cogging.h"
#include "user_config.h"
#include "hw_config.h"
#include "position_sensor.h"

/* 惯量标定的时长/阈值（控制周期数，40 kHz）。都远小于 IWDG(~4 s) */
#define INERTIA_T(sec)			((uint16_t)((sec)/DT))
#define INERTIA_SETTLE_CYC		INERTIA_T(0.15f)	// 起始静置
#define INERTIA_BRAKE_MIN		INERTIA_T(0.020f)	// 刹车至少这么长（别被单点噪声提前放行）
#define INERTIA_BRAKE_CYC		INERTIA_T(0.15f)	// 刹车最长时间
#define INERTIA_BREAKAWAY_MAX	INERTIA_T(0.60f)	// 试探超时
#define INERTIA_BREAKAWAY_STEP	INERTIA_T(0.008f)	// 每 8 ms 加一档
#define INERTIA_TEST_CYC		INERTIA_T(0.060f)	// 单个台阶最长时长
#define INERTIA_SKIP_CYC		INERTIA_T(0.008f)	// 丢掉电流环建立时间
#define INERTIA_MIN_SAMPLES		20u					// 少于这个数认为拟合不可信
/* 下面的阈值必须明显高于静止时的 PLL 速度噪声，否则一上电就"检测到在动"，
 * 或者试探阶段一拍就误判成已突破摩擦。**实测**（2026-11-06，编码器原始计数
 * σ≈6 count，`pll_bandwidth 2000`）：静止时 |pll_velocity| 峰值 ±0.18 rad/s
 * （输出轴）、σ≈0.044 —— 所以阈值取 0.35 才有 2x 余量（带宽开 3000 时峰值
 * ±0.23 仍然够）。取 0.15 会让突破电流被噪声误判成最小档 0.05 A，随后两个
 * 测试台阶（1.5x/2.5x）只有 0.075/0.125 A，拟合出的加速度全是噪声
 * （status 6）。细节见 docs/DEV.md 的"编码器速度 PLL" */
#define INERTIA_V_IDLE			0.35f				// 认为关节静止：窗内最大速度低于它
#define INERTIA_V_BREAK			0.35f				// 判定"动了"的输出轴速度
#define INERTIA_V_STOP			0.25f				// 刹车到低于它就算停了（同样要高于噪声）
/* 速度硬上限（安全闸）。原来是 2.0，但本机 a≈150 rad/s² 时 13 ms 就撞上它，
 * 而前面还要跳过 8 ms 建立时间 ⇒ 每个台阶只剩 ~5 ms(200 点) 在拟合，齿隙/摩擦
 * 的瞬态全进来了（台架实测单次 J 离散 ±50%）。放到 5.0 后由位移上限 0.10 rad
 * 先触发，拟合窗口变成 ~26 ms，重复性明显变好（见 docs/DEV.md） */
#define INERTIA_V_CAP			5.0f				// 测试期间的速度硬上限（安全闸）
#define INERTIA_TRAVEL_CAP		0.10f				// 单个台阶的位移上限（rad，输出轴）
/* 试探步长。原来是 0.05 A，但本机突破电流只有 0.15~0.25 A ⇒ 只有 3~5 档可选，
 * 两台阶的 Δi 在 0.15~0.25 之间跳（±25% 直接进 J 的分母）。0.02 A 让 Δi 一致得多 */
#define INERTIA_I_STEP			0.02f				// 试探电流步长（A）
#define INERTIA_I_MAX_FRAC		0.5f				// 测试电流 ≤ 0.5·I_MAX
#define INERTIA_FIT_T			0.100f				// pos_error_fault 推荐值里用的恢复时间（s）

/* GR 可以在运行期改（config.gr），所以每拍重算；非法值退化为 1，
 * 免得除零把速度反馈变成 inf 灌进电流环 */
static float inv_gear_ratio(void)
{
	float gr = GR;
	return (gr > 0.0f) ? (1.0f / gr) : 1.0f;
}

static int prescaler(int value)
{
	if (value < 1) { return 1; }
	if (value > MOTION_PRESCALER_MAX) { return MOTION_PRESCALER_MAX; }
	return value;
}

/* config.input_mode 的取值清洗：越界或不支持的模式（MIX_CHANNELS / MIRROR /
 * TUNING）按 PASSTHROUGH 处理 —— 宁可"没整形"，也不能让关节因为一个无效参数
 * 彻底不动。 */
static uint8_t input_mode_now(void)
{
	int m = INPUT_MODE;
	if ((m < 0) || (m > INPUT_MODE_MIT)) { return INPUT_MODE_PASSTHROUGH; }
	if ((m == INPUT_MODE_MIX_CHANNELS) || (m == INPUT_MODE_MIRROR)
	    || (m == INPUT_MODE_TUNING)) {
		return INPUT_MODE_PASSTHROUGH;
	}
	return (uint8_t)m;
}

/* 位置跟随误差故障的推荐阈值：
 *   e_recover = ½·a_max·T²      T 秒内能把静止的关节拉回来的位移（a_max = KT·GR·I_MAX/J）
 *   e_limit   = ½·V_MAX/POS_GAIN 再大位置环就必然被速度限幅饱和，故障永不触发
 * 取较小值；惯量未标定时只用 e_limit。 */
static void update_pos_fault_suggest(ControllerStruct *controller)
{
	float v = V_MAX > 0.0f ? V_MAX : 1.0f;
	float pg = POS_GAIN > 0.0f ? POS_GAIN : 1.0f;
	float suggest = 0.5f * v / pg;

	if (INERTIA > 0.0f) {
		float accel_max = KT * GR * (I_MAX > 0.0f ? I_MAX : 0.0f) / INERTIA;
		float recover = 0.5f * accel_max * INERTIA_FIT_T * INERTIA_FIT_T;
		if (recover < suggest) { suggest = recover; }
	}
	controller->pos_fault_suggest = suggest;
}

/* 进入一个新的控制律时播种状态：不清的话，速度环积分、斜率限制器、位置前馈
 * 都会带着上一个模式的残留，表现是切换瞬间的一下顿挫或冲车。 */
static void motion_seed(ControllerStruct *controller, const EncoderStruct *encoder, float inv_gr)
{
	controller->vel_integ = 0.0f;
	controller->pos_fault_t = 0.0f;
	controller->pos_err = 0.0f;
	controller->vel_err = 0.0f;

	/* 从实测速度起步：CSV 的阶跃变成“从当前速度开始拉”，CSP 也不会因为
	 * 速度环从零速开始而先顿一下 */
	controller->v_setpoint = encoder->pll_velocity * inv_gr;
	controller->v_target = controller->v_setpoint;

	/* 位置前馈按帧差分，切换模式时丢弃上一帧的参考与时间戳 */
	controller->pos_ff = 0.0f;
	controller->pos_ff_last = controller->p_des;
	controller->pos_ff_cycle = controller->loop_count;

	/* 输入整形状态：POS_FILTER 从实测位置起步；TRAP_TRAJ 下一拍会从
	 * (当前设定点, 0) 重新规划（planned = 0 就是"还没规划"的标志） */
	controller->pos_filter.seeded = 0;
	controller->pos_trap.planned = 0;
	controller->pos_trap.t = 0.0f;
}

/* ---- 惯量标定 ---------------------------------------------------------
 *
 * 输出侧模型：J·dω/dt = KT·GR·i_q − c（c = 重力 + 库仑摩擦，常值）。
 * 同一个方向上打两个电流台阶、分别拟合加速度，斜率里 c 被消掉：
 *
 *     J = KT·GR·(i_hi − i_lo) / (a_hi − a_lo)
 *
 * 条件：门极已开（MOTOR_MODE）、关节能自由转动一段行程。测试期间
 * motion_control() 独占控制权，返回前把 i_q_des 设成 0。
 * 结果只写 RAM（config.inertia），**不保存 flash** —— 由用户决定是否 action.save。 */
static void inertia_step(ControllerStruct *controller, const EncoderStruct *encoder, float inv_gr)
{
	const float v_out = encoder->pll_velocity * inv_gr;
	const float i_cap = INERTIA_I_MAX_FRAC * I_MAX;

	controller->i_q_des = 0.0f;			// 除测试台阶外一律零电流
	controller->i_d_des = 0.0f;

	switch (controller->inertia_state) {
	case INERTIA_SETTLE:
		/* 等关节停稳。用窗内最大速度判断，单点会被 PLL 噪声骗到 */
		if (fabsf(v_out) > controller->inertia_v_peak) {
			controller->inertia_v_peak = fabsf(v_out);
		}
		if (controller->inertia_cycles < INERTIA_SETTLE_CYC) {
			controller->inertia_cycles++;
			break;
		}
		if (controller->inertia_v_peak > INERTIA_V_IDLE) {
			controller->inertia_run_code = 3;			// 关节本来就在动
			controller->inertia_state = INERTIA_FAIL;
			break;
		}
		controller->inertia_i = 0.0f;
		controller->inertia_i_break = 0.0f;
		controller->inertia_cycles = 0;
		controller->inertia_state = INERTIA_BREAKAWAY;
		break;

	case INERTIA_BREAKAWAY:
		/* 每 8 ms 加一档电流，直到关节开始动 —— 这一档就是"摩擦+重力"的
		 * 等效电流，用它决定两个测试台阶（必须都明显大于它） */
		controller->inertia_cycles++;
		controller->i_q_des = controller->inertia_i;
		if (fabsf(v_out) > INERTIA_V_BREAK) {
			controller->inertia_i_break = controller->inertia_i;
			controller->inertia_cycles = 0;
			controller->inertia_next = INERTIA_TEST_LO;
			controller->inertia_state = INERTIA_BRAKE;
			break;
		}
		if ((controller->inertia_cycles % INERTIA_BREAKAWAY_STEP) == 0) {
			controller->inertia_i += INERTIA_I_STEP;
		}
		if ((controller->inertia_cycles > INERTIA_BREAKAWAY_MAX)
		    || (controller->inertia_i > i_cap)) {
			controller->inertia_run_code = 4;			// 加到上限还不动：卡住或摩擦过大
			controller->inertia_state = INERTIA_FAIL;
		}
		break;

	case INERTIA_BRAKE:
		/* 主动刹车而不是零电流滑行：滑行会让关节多走很多行程，有限位的关节
		 * 直接就撞上去了。阻尼式的小电流把它拉停，最多等 INERTIA_BRAKE_CYC */
		controller->inertia_cycles++;
		{
			float brake = -v_out;
			if (brake > i_cap) { brake = i_cap; }
			if (brake < -i_cap) { brake = -i_cap; }
			controller->i_q_des = brake;
		}
		if ((controller->inertia_cycles >= INERTIA_BRAKE_MIN)
		    && ((fabsf(v_out) < INERTIA_V_STOP)
		        || (controller->inertia_cycles >= INERTIA_BRAKE_CYC))) {
			controller->inertia_cycles = 0;
			controller->inertia_pos_start = encoder->angle_multiturn * inv_gr;

			if (controller->inertia_next == INERTIA_TEST_LO) {
				float lo = 1.5f * controller->inertia_i_break;
				float hi = 2.5f * controller->inertia_i_break;
				if (lo < INERTIA_I_STEP) { lo = INERTIA_I_STEP; }
				if (hi > i_cap) { hi = i_cap; }
				if (lo > i_cap) { lo = i_cap; }
				controller->inertia_i_lo = lo;
				controller->inertia_i_hi = hi;
				ml_fit_reset(&controller->inertia_fit_lo);
				controller->inertia_i_sum_lo = 0.0f;
				controller->inertia_n_lo = 0;
			} else if (controller->inertia_next == INERTIA_TEST_HI) {
				ml_fit_reset(&controller->inertia_fit_hi);
				controller->inertia_i_sum_hi = 0.0f;
				controller->inertia_n_hi = 0;
			}
			controller->inertia_state = controller->inertia_next;
		}
		break;

	case INERTIA_TEST_LO:
	case INERTIA_TEST_HI: {
		const int hi_phase = (controller->inertia_state == INERTIA_TEST_HI);
		const float i_test = hi_phase ? controller->inertia_i_hi : controller->inertia_i_lo;
		const float t = (float)controller->inertia_cycles * DT;
		const float travel = fabsf(encoder->angle_multiturn * inv_gr - controller->inertia_pos_start);

		controller->inertia_cycles++;
		controller->i_q_des = i_test;

		/* 丢掉前 8 ms：电流环建立起力矩之前的数据点会把斜率拉平 */
		if (controller->inertia_cycles > INERTIA_SKIP_CYC) {
			if (hi_phase) {
				ml_fit_add(&controller->inertia_fit_hi, t, v_out);
				controller->inertia_i_sum_hi += controller->i_q;
				controller->inertia_n_hi++;
			} else {
				ml_fit_add(&controller->inertia_fit_lo, t, v_out);
				controller->inertia_i_sum_lo += controller->i_q;
				controller->inertia_n_lo++;
			}
		}

		/* 结束条件：时长到 / 走够行程（轻关节几毫秒就到） / 速度硬上限 */
		if ((controller->inertia_cycles >= INERTIA_TEST_CYC)
		    || (travel > INERTIA_TRAVEL_CAP)
		    || (fabsf(v_out) > INERTIA_V_CAP)) {
			controller->inertia_cycles = 0;
			controller->inertia_next = hi_phase ? INERTIA_DONE : INERTIA_TEST_HI;
			controller->inertia_state = INERTIA_BRAKE;
		}
		break;
	}

	case INERTIA_DONE:
		if ((controller->inertia_n_lo < INERTIA_MIN_SAMPLES)
		    || (controller->inertia_n_hi < INERTIA_MIN_SAMPLES)) {
			controller->inertia_run_code = 5;			// 台阶太短，采不到足够的点
			controller->inertia_state = INERTIA_FAIL;
			break;
		}
		controller->inertia_acc_lo = ml_fit_slope(&controller->inertia_fit_lo);
		controller->inertia_acc_hi = ml_fit_slope(&controller->inertia_fit_hi);

		float i_lo = controller->inertia_i_sum_lo / (float)controller->inertia_n_lo;
		float i_hi = controller->inertia_i_sum_hi / (float)controller->inertia_n_hi;
		if (ml_inertia_from_slopes(KT, GR, i_lo, controller->inertia_acc_lo,
		                           i_hi, controller->inertia_acc_hi,
		                           &controller->inertia_meas)) {
			INERTIA = controller->inertia_meas;	// RAM only；不写 flash
			update_pos_fault_suggest(controller);
			controller->inertia_run_code = 2;
		} else {
			controller->inertia_run_code = 6;			// 激励不足 / 斜率不可信
		}
		controller->inertia_state = INERTIA_IDLE;
		controller->i_q_ref = 0.0f;
		break;

	case INERTIA_FAIL:
	default:
		controller->i_q_ref = 0.0f;
		controller->inertia_state = INERTIA_IDLE;
		break;
	}
}

/* action.tune_gains：用已标定的惯量算外环增益并写进 RAM（不落盘，保存仍由
 * action.save 决定）。钳位值必须与 params.json 里对应参数的 min/max 一致 ——
 * 这里是直接写 __float_reg，绕过了 endpoint 层的限幅。 */
void motion_tune_gains(ControllerStruct *controller)
{
	float vg, vig, pg;

	if (!ml_tune_gains(INERTIA, KT, GR, OMEGA_C, TUNE_MARGIN, &vg, &vig, &pg)) {
		return;
	}
	VEL_GAIN = ml_clampf(vg, 0.0f, 2000.0f);
	VEL_INTEGRATOR_GAIN = ml_clampf(vig, 0.0f, 200000.0f);
	POS_GAIN = ml_clampf(pg, 0.0f, 1000.0f);
	update_pos_fault_suggest(controller);
}

void motion_control(ControllerStruct *controller, EncoderStruct *encoder)
{
	int vel_pre = prescaler(VEL_LOOP_PRESCALER);
	int pos_pre = prescaler(POS_LOOP_PRESCALER);

	/* 齿槽补偿标定独占控制权（与惯量标定同构）：它自己产生 i_q_des，
	 * 不走任何控制律，也不参与外环分频 */
	if (cogging_cal_step(controller, encoder)) {
		return;
	}

	/* 惯量标定期间独占控制权：它会自己产生 i_q_des，不走任何控制律 */
	if (controller->inertia_req && (controller->inertia_state == INERTIA_IDLE)) {
		controller->inertia_req = 0;
		controller->inertia_state = INERTIA_SETTLE;
		controller->inertia_run_code = 1;
		controller->inertia_cycles = 0;
		controller->inertia_i = 0.0f;
		controller->inertia_v_peak = 0.0f;
		controller->inertia_meas = 0.0f;
	}
	if (controller->inertia_state != INERTIA_IDLE) {
		inertia_step(controller, encoder, inv_gear_ratio());
		return;
	}

	if (controller->ctrl_mode != controller->ctrl_mode_prev) {
		motion_seed(controller, encoder, inv_gear_ratio());
		/* 让外环在下一拍就更新，不用等满一个预分频周期 */
		controller->vel_prescale_cnt = (uint16_t)(vel_pre - 1);
		controller->pos_prescale_cnt = (uint16_t)(pos_pre - 1);
	}
	controller->ctrl_mode_prev = controller->ctrl_mode;

	/* 输入整形模式变了也要重新播种：TRAP_TRAJ/POS_FILTER 的状态属于上一个模式，
	 * 带着走会从旧的设定点突然跳过去 */
	const uint8_t in_mode = input_mode_now();
	if (in_mode != controller->input_mode_prev) {
		controller->pos_filter.seeded = 0;
		controller->pos_trap.planned = 0;
		controller->pos_trap.t = 0.0f;
		controller->input_mode_prev = in_mode;
	}

	if (controller->ctrl_mode == CTRL_IMPEDANCE) {
		/* 现有阻抗律：每拍都算，与改动前完全一致 */
		torque_control(controller);
		controller->i_q_ref = controller->i_q_des;	// 保持镜像，切模式时无扰
		controller->vel_update = 0;
		controller->pos_update = 0;
		return;
	}

	/* 分频，嵌套执行（与 ODrive 的 vel/pos prescaler 同构）：
	 * 位置环只在速度环执行的那一拍里执行，两者相位天然对齐 */
	controller->vel_update = 0;
	controller->pos_update = 0;
	if (++controller->vel_prescale_cnt >= (uint16_t)vel_pre) {
		controller->vel_prescale_cnt = 0;
		controller->vel_update = 1;
		if (++controller->pos_prescale_cnt >= (uint16_t)pos_pre) {
			controller->pos_prescale_cnt = 0;
			controller->pos_update = 1;
		}
	}

	if (controller->vel_update) {
		const float inv_gr = inv_gear_ratio();
		const float dt_v = (float)vel_pre * DT;		// 速度环周期
		const float dt_p = dt_v * (float)pos_pre;	// 位置环周期
		/* 外环用 PLL 速度（平滑、滞后可控），不用 encoder.velocity 的
		 * 475 us 差分 —— 那条路径留给 MIT 与 legacy 遥测，一行没改 */
		const float v_meas = encoder->pll_velocity * inv_gr;
		/* 速度限幅来自帧又受 config.v_max 约束；万一被写成 0/负（旧配置或
		 * 手误），退回 v_max，不要变成“速度指令恒为 0”把关节放空 */
		float v_lim = controller->v_limit;
		if (!(v_lim > 0.0f)) { v_lim = V_MAX; }

		if (in_mode == INPUT_MODE_INACTIVE) {
			/* 忽略输入：设定值保持不变（ODrive 的 INPUT_MODE_INACTIVE 同义）。
			 * 仍跑速度环，所以关节会稳在当前设定点上，而不是卸力 */
			controller->pos_err = 0.0f;
			controller->pos_fault_t = 0.0f;
		} else if ((controller->ctrl_mode == CTRL_POSITION) && controller->pos_update) {
			float p_meas = encoder->angle_multiturn * inv_gr;
			float p_des = controller->p_des;
			/* P_MAX/P_MIN 配反了就不限幅（默认 ±12.5 rad 是给整机留的软限位） */
			if (P_MAX > P_MIN) { p_des = ml_clampf(p_des, P_MIN, P_MAX); }

			/* 输入整形：POS_FILTER / TRAP_TRAJ 给出自己的 (位置, 速度) 设定，
			 * 速度同时作为位置环前馈取代帧间差分的 pos_ff（两者叠加会过冲） */
			float p_track = p_des;
			float v_ff = controller->pos_ff;
			if (in_mode == INPUT_MODE_TRAP_TRAJ) {
				/* 轨迹用自己的一组限值：只配了 trap_* 就完全不碰通用的 v_max /
				 * accel_limit，可以"轨迹慢速爬 + 手动高速点动"共存。
				 * 三项都默认 0 ⇒ 退回通用值，行为与改动前一致。 */
				(void)ml_trap_step(&controller->pos_trap, dt_p, p_des,
				                   ml_trap_vel(TRAP_VEL_LIMIT, v_lim),
				                   ml_limit_or(TRAP_ACCEL_LIMIT, ACCEL_LIMIT),
				                   ml_limit_or(TRAP_DECEL_LIMIT, DECEL_LIMIT),
				                   &p_track, &v_ff);
			} else if (in_mode == INPUT_MODE_POS_FILTER) {
				if (!controller->pos_filter.seeded) {
					ml_posf_reset(&controller->pos_filter, p_meas, 0.0f);
				}
				ml_posf_step(&controller->pos_filter, p_des, 0.0f, INPUT_BW, dt_p,
				             &p_track, &v_ff);
			}

			float pos_err = p_track - p_meas;
			float v_cmd = ml_pos_p(POS_GAIN, POS_FF_GAIN, pos_err, v_ff);

			controller->pos_err = pos_err;
			/* config.enable_vel_limit 关掉时不再夹取（帧内的 vel_limit 也不生效） */
			controller->v_target = VEL_LIMIT_EN ? ml_clampf(v_cmd, -v_lim, v_lim) : v_cmd;

			/* 被速度限幅 = 正在按上限追赶，这时误差大是正常的；
			 * 只有“没饱和却一直追不上”才当成跟随故障 */
			if (ml_pos_fault(POS_ERROR_FAULT, &controller->pos_fault_t, pos_err,
			                 VEL_LIMIT_EN && (v_cmd != controller->v_target), dt_p)) {
				controller->fault |= FAULT_POS;
			}
		} else if (controller->ctrl_mode == CTRL_VELOCITY) {
			controller->pos_err = 0.0f;
			controller->pos_fault_t = 0.0f;
			controller->v_target = VEL_LIMIT_EN
			                       ? ml_clampf(controller->v_des, V_MIN, V_MAX)
			                       : controller->v_des;
		}

		/* 速度设定值整形：只有 VEL_RAMP 走斜率限制。PASSTHROUGH / MIT 直通，
		 * POS_FILTER / TRAP_TRAJ 的平滑已经由轨迹给出（位置环输出就是速度指令） */
		if (in_mode != INPUT_MODE_INACTIVE) {
			/* config.vel_ramp_rate > 0 时加速与减速都用它（ODrive 的 vel_ramp_rate
			 * 就是对称速率）；= 0（默认）时用 accel_limit / decel_limit */
			controller->v_setpoint = (in_mode == INPUT_MODE_VEL_RAMP)
			                         ? ml_slew2(controller->v_setpoint, controller->v_target,
			                                    ml_limit_or(VEL_RAMP_RATE, ACCEL_LIMIT),
			                                    ml_limit_or(VEL_RAMP_RATE, DECEL_LIMIT), dt_v)
			                         : controller->v_target;
		}
		controller->vel_err = controller->v_setpoint - v_meas;
		controller->i_q_ref = ml_vel_pi(VEL_GAIN, VEL_INTEGRATOR_GAIN,
		                                controller->i_cmd_max, &controller->vel_integ,
		                                controller->v_setpoint, v_meas, dt_v);
		/* 推荐阈值随 pos_gain / inertia / i_max 变，跟着算一次（十几条浮点运算） */
		update_pos_fault_suggest(controller);
	}

	/* field_weaken() 每拍都会就地改写 i_q_des（i_q_des += fw_int）。分频等待
	 * 期间必须从外环保持值重新赋值，否则它会一拍接一拍地累加下去 */
	controller->i_q_des = controller->i_q_ref;
	controller->i_d_des = 0.0f;
}
