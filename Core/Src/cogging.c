/*
 * cogging.c
 *
 * 齿槽转矩补偿：运行时前馈（cogging_apply）+ 标定状态机（cogging_cal_step）。
 *
 * 标定方法（与 ODrive 的 anticogging 同源，但改成"正反两遍 + 去均值 + 去趋势"）：
 *   把转子逐个表点**保持住**，等位置误差收敛后读速度环积分器。稳态下
 *   v_meas == v_cmd == pos_gain·pos_err，而位置为常值时唯一能让误差为 0 的
 *   就是"维持该位置所需的 i_q"，也就是齿槽力矩（叠加重力/摩擦的直流分量）。
 *   正转、反转各扫一遍取平均：保持位置时静摩擦的方向由**接近方向**决定，
 *   两遍符号相反，平均就把它抵消；剩下的直流分量（平均摩擦）整体减掉，重力/
 *   外载则靠多项式去趋势拿掉（它对两遍是同一个方向，见 cog_detrend_poly）。
 *   所以运行时表里只有"随转子位置周期变化"的部分。
 *
 * 为什么用转子机械周期做表：齿槽来自转子磁钢与定子齿，周期是
 * 1/LCM(槽数, 2·极对数) 个机械圈，比输出轴周期短 GR 倍。表索引直接用编码器
 * 原始计数（count，不受 config.m_zero 影响），所以 action.zero 不会让表失效。
 *
 * 实时性：cogging_apply() 只做一次 4 点三次插值（约 30 个周期）；状态机每拍
 * 几条浮点运算。合并（约 10 KB 的数组遍历）、质检、打印都在主循环
 * cogging_publish_result()。
 */

#include <math.h>
#include "cogging.h"
#include "cogging_law.h"
#include "motion_law.h"
#include "user_config.h"
#include "hw_config.h"
#include "math_ops.h"

/* COG_N 必须整除编码器一圈的计数（索引直接用 count % ENC_CPR） */
_Static_assert((ENC_CPR % COG_N) == 0, "COG_N 必须整除 ENC_CPR");

#define COG_STEP		(ENC_CPR / COG_N)		// 每个表点对应的编码器计数
#define COG_CYC(sec)	((uint32_t)((sec) / DT))

/* 起始静置：等关节停稳再开始扫（零电流）。超过超时也照样开始 ——
 * 第一个点的闭环自己会把它拉住 */
#define COG_START_SETTLE_CYC	COG_CYC(0.15f)
#define COG_START_TIMEOUT_CYC	COG_CYC(2.0f)
/* 认为"静止"的输出轴速度阈值。必须高于静止时 PLL 速度的噪声峰值
 * （台架实测 |pll_velocity| 峰值 ±0.18 rad/s，见 motion.c 的说明） */
#define COG_V_IDLE		0.35f
/* 单个点的超时：0.5 s 还没收敛就记成坏点跳过，不让一个点卡住整轮标定 */
#define COG_POINT_TIMEOUT_CYC	COG_CYC(0.5f)
/* sweep 法：换向后先丢掉这么多周期的过渡（加速 + 速度环建立） */
#define COG_SWEEP_SKIP_CYC		COG_CYC(0.25f)
/* sweep 法：一格至少要累到这么多个样本才算有效（4 s/圈时约 156 个） */
#define COG_MIN_SAMPLES			8u
/* sweep 法：分箱前先二阶低通掉电流测量。台架实测噪声集中在速度环穿越频率
 * （本机约 100 Hz）附近，幅值能到 0.1 A，正好落在表的高空间频率上；两遍平均
 * 也压不掉（每格只有几毫秒）。齿槽基波只有 5~30 Hz，两级 90 Hz 低通对它是
 * 0.9 的增益、对 100 Hz 振铃是 0.3，性价比最高。低通引入的相位滞后在正反两遍
 * 里符号相反，平均时自动抵消。 */
#define COG_IQ_LP_A				(2.0f * 3.14159265f * 90.0f * DT)
/* settle_ms 至少给这么多，避免用户把 cog_settle_ms 写成 0 时"判据一拍就过" */
#define COG_SETTLE_MIN_CYC		COG_CYC(0.005f)
/* 表值上限：运行时由 config.cogging_max 再钳一次，这里存的是**实测**值 */
#define COG_TABLE_LIMIT		COG_MAX_A

CoggingCalStruct cogging_cal;
int16_t COGGING_LUT[COG_N];
/* 合并结果先写这里，质检通过才拷进 COGGING_LUT —— 标定失败时不能把
 * 上一张还能用的表毁掉 */
static int16_t cog_new_lut[COG_N];
/* 标定两遍的原始副本，只给诊断端点看 */
static float cog_dbg1[COG_N];
static float cog_dbg2[COG_N];
/* sweep 法分箱前的二阶低通状态 */
static float cog_iq_lp1, cog_iq_lp2;

static uint32_t settle_cycles(void)
{
	int ms = COG_SETTLE_MS;
	uint32_t c = (ms > 0) ? ((uint32_t)ms * (uint32_t)(1.0f / DT) / 1000u) : 0u;
	return (c < COG_SETTLE_MIN_CYC) ? COG_SETTLE_MIN_CYC : c;
}

static uint16_t avg_cycles(void)
{
	int n = COG_AVG;
	if (n < 1) { return 1; }
	if (n > 400) { return 400; }
	return (uint16_t)n;
}

/* sweep 法：转一个转子机械圈用多少秒（config.cogging_sweep_ms，钳 0.5~20 s） */
static float sweep_seconds(void)
{
	float s = (float)COG_SWEEP_MS * 0.001f;
	if (!(s >= 0.5f)) { s = 0.5f; }
	if (s > 20.0f) { s = 20.0f; }
	return s;
}

/* 一个表点的目标位置（输出轴 rad）。表点网格在编码器原始计数上：
 * count + M_ZERO ≡ 0 (mod COG_STEP)。 */
static void cog_begin_point(uint8_t pass, uint16_t index)
{
	int32_t target_abs = cogging_cal.start_abs + (int32_t)index * (int32_t)COG_STEP;
	float scale = TWO_PI_F / ((float)ENC_CPR * cogging_cal.gr);

	cogging_cal.p_tgt = (float)target_abs * scale;
	cogging_cal.index = index;
	cogging_cal.pass = pass;
	cogging_cal.t_point = 0;
	cogging_cal.settle_cnt = 0;
	cogging_cal.sum = 0.0f;
	cogging_cal.n = 0;
	cogging_cal.state = COG_CAL_MOVE;
}

static void cog_finish(void)
{
	cogging_cal.state = COG_CAL_IDLE;
	cogging_cal.data_ready = 1;		// 合并/质检交给主循环
}

/* sweep 法的一次单向扫描：rate 是输出轴位置斜坡（rad/s，带方向） */
static void cog_sweep_begin(int direction)
{
	cogging_cal.rate = (float)direction * (TWO_PI_F / cogging_cal.gr) / sweep_seconds();
	cogging_cal.t_point = 0;
	cogging_cal.settle_cnt = (uint16_t)COG_SWEEP_SKIP_CYC;	// 先丢一段过渡
	cogging_cal.state = COG_CAL_SWEEP;
}

static void cog_next_point(void)
{
	if (cogging_cal.pass == 0) {
		if ((uint16_t)(cogging_cal.index + 1u) >= (uint16_t)COG_N) {
			/* 正转扫完，接着反转：第一个反转点就是当前位置 */
			cog_begin_point(1, (uint16_t)(COG_N - 1));
		} else {
			cog_begin_point(0, (uint16_t)(cogging_cal.index + 1u));
		}
		return;
	}
	if (cogging_cal.index == 0) {
		cog_finish();
		return;
	}
	cog_begin_point(1, (uint16_t)(cogging_cal.index - 1u));
}

/* 标定期间的位置保持环（定义在下面） */
static float cog_hold(ControllerStruct *controller, float p_meas, float v_meas);

/* sweep 法的一拍：位置斜坡 + 分箱累积速度环输出（含 P 与 I，即"维持这个速度
 * 所需的全部 i_q"）。匀速时动摩擦是常值（去均值时去掉）、方向相反（两遍平均
 * 抵消），而齿槽随位置变化，正好留在表里。
 * 返回本拍的 i_q 指令。 */
static float cog_sweep_sample(ControllerStruct *controller, const EncoderStruct *encoder,
                              float p_meas, float v_meas)
{
	cogging_cal.p_tgt += cogging_cal.rate * DT;
	float iq = cog_hold(controller, p_meas, v_meas);

	/* 二阶低通（两级一阶）：抑制速度环穿越频率附近的振铃，见 COG_IQ_LP_A */
	cog_iq_lp1 += COG_IQ_LP_A * (iq - cog_iq_lp1);
	cog_iq_lp2 += COG_IQ_LP_A * (cog_iq_lp1 - cog_iq_lp2);

	if (cogging_cal.settle_cnt > 0) {
		cogging_cal.settle_cnt--;
	} else {
		/* 四舍五入到最近的格，与运行时查表 / hold 存表同一套约定 */
		uint16_t bin = (uint16_t)cog_bin_of_count(encoder->count, (int32_t)ENC_CPR, COG_N);
		if (cogging_cal.pass == 0) {
			cogging_cal.pass1[bin] += cog_iq_lp2;
			if (cogging_cal.cnt1[bin] < 0xFFFFu) { cogging_cal.cnt1[bin]++; }
		} else {
			cogging_cal.pass2[bin] += cog_iq_lp2;
			if (cogging_cal.cnt2[bin] < 0xFFFFu) { cogging_cal.cnt2[bin]++; }
		}
	}

	cogging_cal.t_point++;
	cogging_cal.done = (uint16_t)((uint16_t)cogging_cal.pass * (uint16_t)COG_N
	                    + (uint16_t)(((uint32_t)cogging_cal.t_point * (uint32_t)COG_N)
	                                 / cogging_cal.t_rev));
	if (cogging_cal.t_point >= cogging_cal.t_rev) {
		if (cogging_cal.pass == 0) {
			cogging_cal.pass = 1;
			cogging_cal.done = (uint16_t)COG_N;
			cog_sweep_begin(-1);
		} else {
			cogging_cal.done = (uint16_t)(2 * COG_N);
			cog_finish();
		}
	}
	return iq;
}

static void cog_record(float value, uint8_t ok)
{
	/* 存到**物理格**：第 index 个点对应 (start_bin + index) mod COG_N —— 必须与
	 * 运行时按绝对计数查表的约定一致（见 cog_bin_of_count） */
	uint16_t i = (uint16_t)(((uint32_t)cogging_cal.start_bin + (uint32_t)cogging_cal.index)
	                        % (uint32_t)COG_N);
	if (cogging_cal.pass == 0) {
		cogging_cal.pass1[i] = value;
		cogging_cal.ok1[i] = ok;
	} else {
		cogging_cal.pass2[i] = value;
		cogging_cal.ok2[i] = ok;
	}
	cogging_cal.done++;
	cog_next_point();
}

/* 标定期间的位置保持环：直接用外环同一套律（config.pos_gain / vel_gain /
 * vel_integrator_gain），所以"用户调好的外环"就是标定用的外环 —— 标定结果与
 * 实际运行条件一致。返回本拍的 i_q 指令。 */
static float cog_hold(ControllerStruct *controller, float p_meas, float v_meas)
{
	float pos_err = cogging_cal.p_tgt - p_meas;
	float v_cmd = ml_pos_p(POS_GAIN, 0.0f, pos_err, 0.0f);
	float vlim = (V_MAX > 0.0f) ? V_MAX : 1.0f;
	float i_lim = (I_MAX > 0.0f) ? I_MAX : 1.0f;

	v_cmd = ml_clampf(v_cmd, -vlim, vlim);
	controller->i_cmd_max = i_lim;
	float iq = ml_vel_pi(VEL_GAIN, VEL_INTEGRATOR_GAIN, i_lim, &controller->vel_integ,
	                     v_cmd, v_meas, DT);

	controller->pos_err = pos_err;
	controller->vel_err = v_cmd - v_meas;
	controller->v_setpoint = v_cmd;
	controller->v_target = v_cmd;
	return iq;
}

int cogging_cal_step(ControllerStruct *controller, const EncoderStruct *encoder)
{
	float iq = 0.0f;

	if (cogging_cal.state == COG_CAL_IDLE) {
		if (!controller->cogging_req) { return 0; }
		controller->cogging_req = 0;

		if (!encoder->first_sample) {
			cogging_cal.status = COG_ERR_ABORT;		// 编码器还没有有效帧
			return 0;
		}
		if (!(GR > 0.0f) || !(POS_GAIN > 0.0f) || !(VEL_GAIN > 0.0f)
		    || !(VEL_INTEGRATOR_GAIN > 0.0f)) {
			cogging_cal.status = COG_ERR_CONFIG;
			return 0;
		}

		cogging_cal.gr = GR;
		cogging_cal.method = (COG_METHOD == COG_METHOD_SWEEP) ? COG_METHOD_SWEEP : COG_METHOD_HOLD;
		cogging_cal.pass = 0;
		cogging_cal.index = 0;
		cogging_cal.done = 0;
		cogging_cal.bad = 0;
		cogging_cal.amp = 0.0f;
		cogging_cal.mismatch = 0.0f;
		cogging_cal.sum = 0.0f;
		cogging_cal.n = 0;
		cogging_cal.settle_cnt = 0;
		cogging_cal.t_point = 0;
		cogging_cal.data_ready = 0;
		cogging_cal.t_rev = (uint32_t)COG_SWEEP_SKIP_CYC
		                  + (uint32_t)(sweep_seconds() / DT);
		for (int i = 0; i < COG_N; i++) {
			cogging_cal.ok1[i] = 0;
			cogging_cal.ok2[i] = 0;
			cogging_cal.pass1[i] = 0.0f;
			cogging_cal.pass2[i] = 0.0f;
			cogging_cal.cnt1[i] = 0;
			cogging_cal.cnt2[i] = 0;
		}

		/* 表点网格：绝对计数 = k·COG_STEP - M_ZERO。取不高于当前位置的那个点，
		 * 这样第一个目标最多往回走一个表距。
		 * start_bin 是第 0 个点在**表里**的索引（运行时按绝对计数查表）：
		 * 第 i 个点的物理格是 (start_bin + i) mod COG_N，不是 i —— hold 法存表
		 * 必须用它，否则整张表会被旋转一个随机的 k0 格。 */
		int64_t k = (int64_t)encoder->count_buff[0] + (int64_t)M_ZERO;
		if (k < 0) { k -= (int64_t)(COG_STEP - 1); }		// 负数也要向下取整
		k /= (int64_t)COG_STEP;
		cogging_cal.start_abs = (int32_t)(k * (int64_t)COG_STEP) - M_ZERO;
		int32_t kb = (int32_t)(k % (int64_t)COG_N);
		if (kb < 0) { kb += COG_N; }
		cogging_cal.start_bin = (uint16_t)kb;

		controller->vel_integ = 0.0f;
		cog_iq_lp1 = 0.0f;
		cog_iq_lp2 = 0.0f;
		cogging_cal.status = COG_STATUS_RUNNING;
		cogging_cal.state = COG_CAL_SETTLE;
	}

	const float inv_gr = 1.0f / cogging_cal.gr;
	const float p_meas = encoder->angle_multiturn * inv_gr;
	const float v_meas = encoder->pll_velocity * inv_gr;

	switch (cogging_cal.state) {
	case COG_CAL_SETTLE:
		controller->vel_integ = 0.0f;
		if (fabsf(v_meas) < COG_V_IDLE) {
			if (cogging_cal.settle_cnt < 0xFFFFu) { cogging_cal.settle_cnt++; }
		} else {
			cogging_cal.settle_cnt = 0;
		}
		cogging_cal.t_point++;
		if ((cogging_cal.settle_cnt >= COG_START_SETTLE_CYC)
		    || (cogging_cal.t_point >= COG_START_TIMEOUT_CYC)) {
			if (cogging_cal.method == COG_METHOD_SWEEP) {
				/* 从当前位置起步，正向匀速扫一圈再反向扫回来 */
				cogging_cal.p_tgt = p_meas;
				cogging_cal.pass = 0;
				cogging_cal.done = 0;
				cog_sweep_begin(+1);
			} else {
				cog_begin_point(0, 0);
			}
		}
		break;

	case COG_CAL_SWEEP:
		iq = cog_sweep_sample(controller, encoder, p_meas, v_meas);
		break;

	case COG_CAL_MOVE:
		iq = cog_hold(controller, p_meas, v_meas);
		if (fabsf(controller->pos_err) <= COG_POS_THR) {
			if (cogging_cal.settle_cnt < 0xFFFFu) { cogging_cal.settle_cnt++; }
		} else {
			cogging_cal.settle_cnt = 0;
		}
		cogging_cal.t_point++;
		if (cogging_cal.t_point >= COG_POINT_TIMEOUT_CYC) {
			cog_record(0.0f, 0);			// 没收敛：记坏点，继续下一个
		} else if (cogging_cal.settle_cnt >= settle_cycles()) {
			cogging_cal.sum = 0.0f;
			cogging_cal.n = 0;
			cogging_cal.state = COG_CAL_SAMPLE;
		}
		break;

	case COG_CAL_SAMPLE:
		iq = cog_hold(controller, p_meas, v_meas);
		if (fabsf(controller->pos_err) > 2.0f * COG_POS_THR) {
			/* 采样窗内被推走了（外力/齿隙跳变）：丢掉这半段，回去重新收敛。
			 * 阈值取 2 倍，避免在噪声边界上反复跳状态 */
			cogging_cal.settle_cnt = 0;
			cogging_cal.sum = 0.0f;
			cogging_cal.n = 0;
			cogging_cal.state = COG_CAL_MOVE;
			break;
		}
		cogging_cal.sum += controller->vel_integ;
		cogging_cal.n++;
		if (cogging_cal.n >= avg_cycles()) {
			cog_record(cogging_cal.sum / (float)cogging_cal.n, 1);
		}
		break;

	default:
		cogging_cal.state = COG_CAL_IDLE;
		break;
	}

	if (cogging_cal.state == COG_CAL_IDLE) {
		/* 收尾（或中途出错）：不能带着保持电流回到正常控制 */
		iq = 0.0f;
		controller->vel_integ = 0.0f;
	}

	controller->i_q_des = iq;
	controller->i_q_ref = iq;
	controller->i_d_des = 0.0f;
	controller->vel_update = 0;
	controller->pos_update = 0;
	return 1;
}

void cogging_apply(ControllerStruct *controller, const EncoderStruct *encoder)
{
	controller->cog_ff = 0.0f;
	if (!COGGING_EN || !COGGING_VALID) { return; }
	if (cogging_cal.state != COG_CAL_IDLE) { return; }		// 标定中不叠加
	if (!(COG_GAIN > 0.0f)) { return; }

	float lim = COG_MAX;
	if (!(lim > 0.0f)) { return; }							// 0 = 关闭（双保险）

	float frac = cog_frac_of_count(encoder->count, (int32_t)ENC_CPR);
	float ff = cog_eval(COGGING_LUT, COG_N, frac) * COG_GAIN;
	if (ff > lim) { ff = lim; }
	if (ff < -lim) { ff = -lim; }

	controller->cog_ff = ff;
	controller->i_q_des += ff;
}

void cogging_abort(ControllerStruct *controller)
{
	if (cogging_cal.state != COG_CAL_IDLE) {
		cogging_cal.state = COG_CAL_IDLE;
		cogging_cal.data_ready = 0;
		cogging_cal.status = COG_ERR_ABORT;
	}
	controller->cogging_req = 0;
}

void cogging_reset(ControllerStruct *controller)
{
	for (int i = 0; i < COG_N; i++) { COGGING_LUT[i] = 0; }
	COGGING_VALID = 0;
	controller->cog_ff = 0.0f;
	cogging_cal.amp = 0.0f;
	cogging_cal.mismatch = 0.0f;
	cogging_cal.bad = 0;
}

void cogging_publish_result(void)
{
	int detrend = (COG_DETREND < 0) ? 0 : (COG_DETREND > 6 ? 6 : COG_DETREND);

	if (cogging_cal.method == COG_METHOD_SWEEP) {
		/* sweep 的 pass1/pass2 是累积和，先平均成每格的值 */
		for (int i = 0; i < COG_N; i++) {
			cogging_cal.ok1[i] = (cogging_cal.cnt1[i] >= COG_MIN_SAMPLES);
			if (cogging_cal.ok1[i]) {
				cogging_cal.pass1[i] /= (float)cogging_cal.cnt1[i];
			}
			cogging_cal.ok2[i] = (cogging_cal.cnt2[i] >= COG_MIN_SAMPLES);
			if (cogging_cal.ok2[i]) {
				cogging_cal.pass2[i] /= (float)cogging_cal.cnt2[i];
			}
		}
	}

	/* 留一份原始副本（诊断端点 telemetry.cogging_pass1/2） */
	for (int i = 0; i < COG_N; i++) {
		cog_dbg1[i] = cogging_cal.pass1[i];
		cog_dbg2[i] = cogging_cal.pass2[i];
	}

	/* 每一遍各去一次均值 + 去趋势：两遍的摩擦方向相反，减去各自的直流分量后
	 * 平均，迟滞项才会真正抵消；重力/外载是慢变趋势，两遍都一样，
	 * 只能靠多项式去趋势拿掉（原因见 cogging_law.h 的 cog_detrend_poly） */
	cog_mean_remove(cogging_cal.pass1, cogging_cal.ok1, COG_N);
	cog_mean_remove(cogging_cal.pass2, cogging_cal.ok2, COG_N);
	cog_detrend_poly(cogging_cal.pass1, COG_N, detrend);
	cog_detrend_poly(cogging_cal.pass2, COG_N, detrend);
	cogging_cal.mismatch = cog_rms_diff(cogging_cal.pass1, cogging_cal.ok1,
	                                    cogging_cal.pass2, cogging_cal.ok2, COG_N);

	cogging_cal.bad = (uint16_t)cog_combine(cogging_cal.pass1, cogging_cal.ok1,
	                                        cogging_cal.pass2, cogging_cal.ok2,
	                                        COG_N, cog_new_lut, COG_TABLE_LIMIT, detrend);
	cogging_cal.amp = cog_rms_i16(cog_new_lut, COG_N);

	if (cogging_cal.bad > (uint16_t)(COG_N / 4)) {
		cogging_cal.status = COG_ERR_POINTS;
		COGGING_VALID = 0;
	} else if (cogging_cal.mismatch > 0.5f * cogging_cal.amp + 0.005f) {
		/* 正反差异与信号同量级：多半是齿隙/摩擦太大、增益不对，或者标定期间
		 * 有外力。宁可不生效，也不要往电流环里灌一张错表；旧表保持原样，
		 * 但先停用（它对应的可能是换过的编码器） */
		cogging_cal.status = COG_ERR_MATCH;
		COGGING_VALID = 0;
	} else {
		for (int i = 0; i < COG_N; i++) { COGGING_LUT[i] = cog_new_lut[i]; }
		cogging_cal.status = COG_STATUS_OK;
		COGGING_VALID = 1;
	}
}

int8_t cogging_status(void)
{
	return cogging_cal.status;
}

int32_t cogging_progress(void)
{
	return (int32_t)cogging_cal.done;
}

int32_t cogging_progress_total(void)
{
	return 2 * COG_N;
}

float cogging_amplitude(void)
{
	/* 上电后表是从 flash 读进来的，标定过程没有跑过，这里按需算一次，
	 * 否则 telemetry 会显示 0（表其实是有效的） */
	if ((cogging_cal.amp <= 0.0f) && COGGING_VALID) {
		cogging_cal.amp = cog_rms_i16(COGGING_LUT, COG_N);
	}
	return cogging_cal.amp;
}

float cogging_mismatch(void)
{
	return cogging_cal.mismatch;
}

uint16_t cogging_bad_points(void)
{
	return cogging_cal.bad;
}

float cogging_index_value(int32_t idx)
{
	if ((idx < 0) || (idx >= COG_N)) { return 0.0f; }
	return cog_dequant(COGGING_LUT[idx]);
}

float cogging_pass1_value(int32_t idx)
{
	if ((idx < 0) || (idx >= COG_N)) { return 0.0f; }
	return cog_dbg1[idx];
}

float cogging_pass2_value(int32_t idx)
{
	if ((idx < 0) || (idx >= COG_N)) { return 0.0f; }
	return cog_dbg2[idx];
}
