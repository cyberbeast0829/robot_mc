/*
 * dual_gear.h
 *
 * 齿轮式双编（韦尼尔 / Vernier 双齿轮）的纯数学部分：用主、副两路单圈绝对
 * 编码器的一次读数，解算主编码器（转子轴）相对某个参考位置的绝对多圈位置。
 * 目的是断电后恢复减速机输出轴位置 —— 主编码器只知道自己转到了单圈内的哪个
 * 角度，转了第几圈只能靠 flash 记忆；双齿轮用两路互质齿数的齿轮把圈数也测出来。
 *
 * 这里刻意不包含任何固件头文件（与 motion_law.h / can_cyberbeast_codec.h 同一
 * 做法），host 测试 tools/test_dual_gear.c 直接包含它 —— 测的就是固件跑的那份代码。
 *
 * 原理：
 *   主编码器装在转子轴上，单圈读数 f = frac(θ + φf)，CPR = first_cpr；
 *   副编码器装在一个与转子齿轮啮合的齿轮上（齿数 T1、T2），单圈读数
 *       s = frac(θ·T1/T2 + φs)
 *   令 Δ = (s − f) mod 1，则
 *       Δ = frac(k·θ / T2 + const),   k = T1 − T2
 *   Δ 只随 θ 缓慢变化（每转子一圈变化 |k|/T2），所以**两次读数的 Δ 之差**给出
 *   转子转过的圈数（模 T2），而单圈内的小数部分继续用主编码器自己的计数
 *   （那一路是精确的，韦尼尔只负责"第几圈"）。
 *
 *   读数方向：如果副编码器的读数随正转变小（磁铁反装，或齿轮外啮合让副轴反转），
 *   上面那条关系里的 θ 就带了负号，Δ 对 θ 的斜率会变成 −(T1+T2)/T2 —— 转子每转
 *   一圈相位跑两圈多，圈数的无歧义窗口缩到 ±T2/(2(T1+T2))（34/33 只有 ±0.25 圈，
 *   等于没用）。正确做法是先把读数镜像成 s' = 1 − s（ODrive 的
 *   `sec_enc_pos_abs_ = cpr - raw` 就是这件事），镜像后斜率又回到 +T1/T2，
 *   窗口恢复成 ±T2/2。参数里对应 `reversed`。
 *
 * 为什么不需要"旋转找重合点"（ODrive 的 59e2ad0dc）：Δ 的绝对值里含一个与安装
 * 位置有关的常数，单凭一次读数无法确定 θ（而且镜像/偏置都会改变它）。所以这里把
 * "标定时刻的那一次读数"存成参考（相位 + 当时的圈数计数），上电解算的是相对参考
 * 的圈数差 —— 参考本身就是零点的定义，常数项自然消掉，不需要电机转任何角度。
 *
 * 约束：k = T1 - T2 不能为 0（否则两路信息退化成一路）。无歧义窗口是
 * ±T2/(2|k|) 转子圈 —— 断电期间转子移动超过它就会差整整一个相位周期 T2/|k|。
 * 所以齿数要**接近但不等**：34/33 给 ±16.5 圈，10/9 只给 ±4.5 圈。齿数成比例
 * 放大（10/9 = 20/18 = 30/27）行为完全相同（窗口只取决于 k/T2），现场填真实的
 * 物理齿数即可。gcd(|k|, T2) 是否为 1 不影响本模块的"最近候选"解码。
 */

#ifndef DUAL_GEAR_H
#define DUAL_GEAR_H

#include <math.h>
#include <stdint.h>

typedef struct{
	int32_t t_first;		/* T1：转子（主编码器）侧齿轮齿数 */
	int32_t t_sec;			/* T2：副编码器侧齿轮齿数 */
	int32_t first_cpr;		/* 主编码器每圈计数 */
	int32_t sec_cpr;		/* 副编码器每圈计数 */
	uint8_t reversed;		/* 1 = 副编码器读数随正转变小，解算前先镜像（s = 1 - s） */
} DualGearParams;

static inline int32_t dg_gcd(int32_t a, int32_t b)
{
	if(a < 0){a = -a;}
	if(b < 0){b = -b;}
	while(b != 0){
		int32_t t = a % b;
		a = b;
		b = t;
	}
	return a;
}

static inline int32_t dg_mod(int32_t a, int32_t m)
{
	int32_t r = a % m;
	return r < 0 ? r + m : r;
}

/* 韦尼尔相位对转子圈数的斜率 k（带符号）：Δ 每转子圈变化 k/T2 */
static inline int32_t dg_k(const DualGearParams * p)
{
	return p->t_first - p->t_sec;
}

static inline int32_t dg_k_abs(const DualGearParams * p)
{
	int32_t k = dg_k(p);
	return k < 0 ? -k : k;
}

/* 韦尼尔周期（转子圈）= T2 / gcd(|k|, T2)；0 = 参数非法。
 * 注意这个「整周期」只在按查表法解码（ODrive 的 sec_enc_zero_table）时才有意义；
 * 本模块用"最近的候选"解码（见 dg_turn_delta），真正的量程是 T2/(2|k|)，与 gcd 无关 */
static inline int32_t dg_period_turns(const DualGearParams * p)
{
	int32_t ak = dg_k_abs(p);
	if(ak <= 0 || p->t_sec <= 0){return 0;}
	return p->t_sec / dg_gcd(ak, p->t_sec);
}

/* 无歧义窗口（转子圈）：±T2/(2|k|)。断电期间转子移动超过它就会差整整一个
 * 相位周期 T2/|k|（34/33 时是 ±16.5 / 33 圈，10/9 时是 ±4.5 / 9 圈） */
static inline float dg_window_turns(const DualGearParams * p)
{
	int32_t ak = dg_k_abs(p);
	if(ak <= 0){return 0.0f;}
	return (float)p->t_sec / (2.0f * (float)ak);
}

static inline int dg_params_ok(const DualGearParams * p)
{
	if(p->t_first <= 0 || p->t_sec <= 0){return 0;}
	if(p->t_first == p->t_sec){return 0;}			/* k = 0：两路信息退化成一路 */
	if(p->first_cpr <= 0 || p->sec_cpr <= 0){return 0;}
	return 1;
}

/* 取小数部分 [0,1)。写成这样是为了把浮点舍入落回 1.0 的情况也压回 0 */
static inline float dg_wrap01(float x)
{
	x -= floorf(x);
	if(!(x >= 0.0f) || x >= 1.0f){x = 0.0f;}		/* 含 NaN：当作 0，别把脏值传下去 */
	return x;
}

/* 韦尼尔相位 Δ = (s - f) mod 1，两路原始单圈计数进来（reversed 时先镜像副路） */
static inline float dg_phase(int32_t first_raw, int32_t sec_raw, const DualGearParams * p)
{
	int32_t s_cnt = p->reversed ? (p->sec_cpr - sec_raw) : sec_raw;
	float f = (float)first_raw / (float)p->first_cpr;
	float s = (float)s_cnt / (float)p->sec_cpr;
	return dg_wrap01(s - f);
}

/* 相位差 → 转子圈数差（带符号，已折到最接近 0 的那一支）
 *
 * d = frac(k·Δθ/T2)，于是 |k|·Δθ/T2 = d + m（m 为整数），候选
 * Δθ = ±(d + m)·T2/|k|（m = 0 … |k|-1，k < 0 时整体取负），相邻候选相差 T2/|k|。
 * 取幅值最小的一支：判据是"停机点离参考不到半个相位周期"，这与 ODrive 分支里
 * |R - Z0| < GR/2 的机械假设一致；量程 ±T2/(2|k|)（见 dg_window_turns）。 */
static inline float dg_turn_delta(const DualGearParams * p, float phase_now, float phase_ref)
{
	int32_t ak = dg_k_abs(p);
	float t2 = (float)p->t_sec;
	float d = dg_wrap01(phase_now - phase_ref);
	float best = 0.0f;
	float best_abs = 3.0e38f;

	if(ak <= 0){return 0.0f;}
	for(int32_t m = 0; m < ak; m++){
		float th = (d + (float)m) * t2 / (float)ak;	/* [0, T2) */
		if(th > t2 * 0.5f){th -= t2;}			/* 折到 (-T2/2, T2/2] */
		if(fabsf(th) < best_abs){
			best_abs = fabsf(th);
			best = th;
		}
	}
	return (dg_k(p) < 0) ? -best : best;
}

/* 上电解算：返回应当写进多圈回绕计数（encoder->turns）的整数值。
 *
 *   phase_ref     标定时刻的韦尼尔相位（config.gear_zero_phase）
 *   count_ref     标定时刻的多圈计数 turns*CPR + count_wrapped（config.gear_zero_count）
 *   count_wrapped 当前主编码器相对机械零点的单圈计数
 *   abs_turns_out 可选：解算出的绝对转子圈数（相对标定参考，含小数）
 *
 * 整数圈数来自韦尼尔，小数部分仍然来自主编码器 —— 韦尼尔的小数精度受齿隙/偏心
 * 影响（约千分之一圈），而主编码器单圈计数是精确的，两者结合后上电位置与断电前
 * 只差 ±1 个 count（边界处才可能出现整整一圈的歧义，这是方案固有的）。 */
static inline int32_t dg_restore_turns(const DualGearParams * p,
                                       int32_t first_raw, int32_t sec_raw,
                                       float phase_ref, int32_t count_ref, int32_t count_wrapped,
                                       float * abs_turns_out)
{
	int32_t wrapped_ref = dg_mod(count_ref, p->first_cpr);
	int32_t turns_ref = (count_ref - wrapped_ref) / p->first_cpr;
	float frac_d = (float)(count_wrapped - wrapped_ref) / (float)p->first_cpr;
	float dtheta = dg_turn_delta(p, dg_phase(first_raw, sec_raw, p), phase_ref);
	float n = dtheta - frac_d;

	if(abs_turns_out){
		*abs_turns_out = (float)turns_ref + dtheta;
	}
	return turns_ref + (int32_t)lroundf(n);
}

#endif /* DUAL_GEAR_H */
