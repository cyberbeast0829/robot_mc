/*
 * mtpa_law.h
 *
 * MTPA（Maximum Torque Per Ampere，每安培最大转矩）的纯数学部分。
 *
 * 与 motion_law.h / cogging_law.h 同一做法：这里不包含任何固件头文件，
 * host 侧测试 tools/test_mtpa.c 直接包含它 —— 测的就是固件跑的那份代码。
 *
 * ---- 模型（与 foc.c 的 dq0()/abc() 同一套幅值不变约定）----
 *
 *     τ = 1.5·p·(λ·Iq + (Ld − Lq)·Id·Iq)          τ 是电机端 N·m
 *     KT = 1.5·p·λ                                本仓库标定给出的转矩常数
 *     ΔL = Ld − Lq                                凸极电机为负（q 轴电感更大）
 *
 * 本文件里出现的 "iq" 有两个口径，别混：
 *   - iq_cmd：**SPM 等价的 q 轴电流**，即 τ = KT·iq_cmd。这是控制律
 *     （torque_control / 速度环 / 位置环）输出的量，它们并不知道凸极存在。
 *   - 返回值 .iq：MTPA 拆开之后的真实 q 轴电流，与 iq_cmd 表达**同一个转矩**。
 *
 * ---- 为什么不用"轨迹 + 一次代入"（ODrive 4680 的写法）----
 *
 * 轨迹上的 MTPA 条件（∂τ/∂Id = 0，Id²+Iq² = const）是
 *
 *     ΔL·Id² + λ·Id − ΔL·Iq² = 0
 *
 * 把 Id 解出来代回去、再按 τ 重算 Iq，是一个固定点迭代。凸极弱时一次就收敛
 * （本机台架、ξ<0.2 的电机误差 < 1e-4），但强凸极（ξ = Imax|ΔL|/λ 到 3）时
 * 会来回震荡：实测一次迭代把 Id 从 −134 A 跳到 −17.5 A，算出的转矩比指令大
 * 25%。所以这里换成**以幅值参数化的 Newton 求解**，与凸极强度无关。
 *
 * ---- 求解（记 k = λ/|ΔL|，u = |Id|，y = Iq）----
 *
 * 轨迹条件等价于  Iq² = u² + k·u，于是
 *     τ/(1.5p) = (λ + |ΔL|u)·y ,   幅度² = 2u² + k·u
 * 消掉 u 得到**一元单调方程**（g 严格增，Newton 二次收敛）：
 *
 *     y·(k + √(k² + 4y²)) = 2·k·a          a = |iq_cmd|
 *
 * 解出 y 后 u = (√(k²+4y²) − k)/2。电流圆上的最优点（τ 取最大）同理：
 * u_c = (√(k² + 8Imax²) − k)/4，它的转矩与 λ·a 直接比就知道是否饱和。
 * 保转矩的等价说法：kt_eff = λ + ΔL·Id 就是含磁阻分量的等效磁链
 * （×1.5×PPAIRS 即等效 Kt，电机端 N·m/A）。
 *
 * ---- 安全不变式（host 测试逐条覆盖）----
 *   1. |Id| ≤ id_frac·Imax，|I| ≤ Imax；
 *   2. kt_eff ≥ λ（MTPA 只会让每安培的转矩变大），否则整条路径退回 SPM；
 *   3. ΔL 符号写反 / Ld、Lq 填错 / NaN、未标定 ⇒ 逐位退化为 {0, iq_cmd}，
 *      也就是"关掉 MTPA"的行为；
 *   4. 转矩对 iq_cmd 单调、在饱和判据处连续（不会突然多给一截转矩）。
 *
 * 收益（电流极限处的转矩增益）≈ ξ²/2 − ξ⁴/2，ξ = Imax·|ΔL|/λ：
 *   ξ=0.04（本机台架，λ=1.885 mWb、ΔL=−22.5 µH、3 A）→ +0.13%（噪声里）
 *   ξ=0.5                                                  → +10%
 *   ξ=3.0                                                  → +124%
 */

#ifndef MTPA_LAW_H
#define MTPA_LAW_H

#include <math.h>

/* 凸极指数 ξ 低于它就视为 SPM：预测增益 < ξ²/2 = 0.02%，不值得每拍查表
 * （mtpa_service() 用它在主循环里挡掉 SPM 电机） */
#define ML_MTPA_SALIENCY_MIN	0.02f

/* Newton 迭代：相对步长小于它就算收敛（4 次上限，实际弱凸极 2 次就停） */
#define ML_MTPA_NEWTON_TOL		1.0e-3f
#define ML_MTPA_NEWTON_MAX		4

/* MTPA 拆解结果：同一个转矩下的 (Id, Iq)，单位 A（电机端、幅值不变 dq） */
typedef struct{
	float id;
	float iq;
} ml_mtpa_t;

/* 含磁阻分量的等效磁链（V·s/rad）；×1.5×PPAIRS 即等效 Kt（电机端 N·m/A） */
static inline float ml_mtpa_lambda_eff(float lambda, float dL, float id)
{
	return lambda + dL*id;
}

/* 凸极指数 ξ = Imax·|ΔL|/λ。λ≤0（未标定）或 Imax≤0 时返回 0 */
static inline float ml_mtpa_saliency(float i_max, float lambda, float dL)
{
	if(!(lambda > 0.0f) || !(i_max > 0.0f)){ return 0.0f; }
	return i_max*fabsf(dL)/lambda;
}

/* 轨迹上给定 q 轴电流时的最优 d 轴电流（ΔL<0 时为负）。
 * 有理化形式：Iq 很小时 (−λ+√(λ²+q²)) 会灾难性抵消，这里一个除法就稳。
 * 只给测试/诊断用，控制路径走 ml_mtpa_split（它解的是同一个条件） */
static inline float ml_mtpa_id_of_iq(float iq, float lambda, float dL)
{
	float q = 2.0f*dL*iq;
	float s = sqrtf(lambda*lambda + q*q);
	return q*q/(2.0f*dL*(s + lambda));
}

/* 电流圆上（|I| = i_max、转矩取最大）的 MTPA 最优点 d 轴电流的幅值（正）。
 * 由 2u² + k·u = Imax² 解出 u = (√(k²+8Imax²) − k)/4 */
static inline float ml_mtpa_u_at_limit(float i_max, float lambda, float dL, float id_frac)
{
	float absdL, k, u, lim;
	if(!(lambda > 0.0f) || !(i_max > 0.0f) || !(dL != 0.0f)){ return 0.0f; }
	absdL = fabsf(dL);
	k = lambda/absdL;
	u = 0.25f*(sqrtf(k*k + 8.0f*i_max*i_max) - k);
	if(!(u > 0.0f)){ return 0.0f; }
	lim = id_frac*i_max;
	if(u > lim){ u = lim; }
	return u;
}

/* 电流圆上（|I| = i_max、转矩取最大）的 MTPA 最优点 d 轴电流（ΔL<0 时为负） */
static inline float ml_mtpa_id_at_limit(float i_max, float lambda, float dL)
{
	float s = sqrtf(lambda*lambda + 8.0f*dL*dL*i_max*i_max);
	return 2.0f*dL*i_max*i_max/(s + lambda);
}

/* 这台机器此刻能交付的**最大 q 轴指令**，用"SPM 等价 q 电流"表示（≥ i_max）：
 * 电流圆上的 MTPA 最优点折回 SPM 口径。`torque_control()` 用它当钳位上限，
 * 于是饱和时能多出 g = 该值/i_max 的转矩（MTPA 的峰值转矩收益）。
 * MTPA 不可用/参数异常时返回 i_max（= 旧行为）。 */
static inline float ml_mtpa_iq_cmd_at_limit(float lambda, float dL, float i_max, float id_frac)
{
	float u, y, kt;
	if(!(lambda > 0.0f) || !(i_max > 0.0f) || !(dL != 0.0f) || !(id_frac > 0.0f)){
		return i_max;
	}
	u = ml_mtpa_u_at_limit(i_max, lambda, dL, id_frac);	/* 含 id_frac 钳位 */
	if(!(u > 0.0f)){ return i_max; }
	{
		float q2 = i_max*i_max - u*u;
		if(!(q2 > 0.0f)){ return i_max; }
		y = sqrtf(q2);									/* Iq = √(Imax² − u²) */
	}
	kt = lambda + fabsf(dL)*u;							/* 等效磁链 */
	if(!(kt > lambda)){ return i_max; }
	return y*(kt/lambda);								/* 折回 SPM 等价 q 电流 */
}

/* 在电流极限处、MTPA 相对 SPM 的转矩增益（≥1；不可用时 = 1）。
 * 只给遥测/选型用（telemetry.mtpa_gain）以及 `torque_control()` 的钳位系数，
 * 都在主循环/每条指令里做一次，不进 40 kHz 中断 */
static inline float ml_mtpa_gain_at_limit(float lambda, float dL, float i_max, float id_frac)
{
	if(!(i_max > 0.0f)){ return 1.0f; }
	return ml_mtpa_iq_cmd_at_limit(lambda, dL, i_max, id_frac)/i_max;
}

/* 把"SPM 等价 q 轴电流"拆成 MTPA 的 (Id, Iq)。
 *
 *   iq_cmd  : 控制律要的 q 轴电流（τ = KT·iq_cmd），A
 *   lambda  : 永磁磁链 λ = KT/(1.5·PPAIRS)，Wb
 *   dL      : Ld − Lq，H（凸极电机为负）
 *   i_max   : 总电流上限 |I| ≤ i_max，A
 *   id_frac : |Id| ≤ id_frac·i_max（防坏参数把 d 轴推大；0 = 不做 MTPA）
 *
 * 返回值与 iq_cmd 表达同一个转矩；iq_cmd 超过电流圆能给出的最大转矩时，
 * 输出圆上的 MTPA 最优点（也就是这台机器此刻的峰值转矩）。
 * 任何不可用/可疑的情况都返回 {0, iq_cmd}（= 关掉 MTPA）。
 */
static inline ml_mtpa_t ml_mtpa_split(float iq_cmd, float lambda, float dL,
                                      float i_max, float id_frac)
{
	ml_mtpa_t out;
	float absdL, k, a, lim, u, y, s, target;
	int i;

	out.id = 0.0f;
	out.iq = iq_cmd;
	if(!(lambda > 0.0f) || !(i_max > 0.0f) || !(id_frac > 0.0f)){ return out; }
	if(!isfinite(iq_cmd)){ out.iq = 0.0f; return out; }		/* NaN/Inf 不进控制环 */
	if(!isfinite(dL) || !(dL != 0.0f)){ return out; }

	absdL = fabsf(dL);
	k = lambda/absdL;			/* λ/|ΔL|，单位 A */
	a = fabsf(iq_cmd);

	/* ---- 电流圆饱和 ----
	 * |I| ≤ a 恒成立（见文件头），所以 a < i_max 时不可能饱和，省掉两次开方。
	 * 饱和判据在**转矩域**比：λ·a 对圆上的最大转矩 —— 两个分支在这里连续。
	 * 这里的 cap 与 ml_mtpa_iq_cmd_at_limit() 必须逐位一致（torque_control 用它
	 * 当钳位上限，host 测试 test_iq_cmd_at_limit 守着）。 */
	if(a >= i_max){
		float uc = 0.25f*(sqrtf(k*k + 8.0f*i_max*i_max) - k);
		if(uc > 0.0f){
			float yc, tau;
			lim = id_frac*i_max;
			if(uc > lim){ uc = lim; }
			{
				float q2 = i_max*i_max - uc*uc;
				yc = (q2 > 0.0f) ? sqrtf(q2) : 0.0f;
			}
			tau = (lambda + absdL*uc)*yc;
			if(lambda*a >= tau){
				out.id = (dL < 0.0f) ? -uc : uc;
				out.iq = (iq_cmd < 0.0f) ? -yc : yc;
				return out;
			}
		}
	}

	/* ---- 轨迹：Newton 解 y·(k + √(k²+4y²)) = 2·k·a ---- */
	target = 2.0f*k*a;
	y = a;
	s = sqrtf(k*k + 4.0f*y*y);
	for(i = 0; i < ML_MTPA_NEWTON_MAX; i++){
		float g = y*(k + s);
		float gp, yn;

		if(g == target){ break; }
		gp = k + s + 4.0f*y*y/s;			/* g'(y) > 0 */
		if(!(gp > 0.0f)){ break; }
		yn = y - (g - target)/gp;
		if(!(yn > 0.0f)){ break; }
		s = sqrtf(k*k + 4.0f*yn*yn);
		if(fabsf(yn - y) <= ML_MTPA_NEWTON_TOL*y){ y = yn; break; }
		y = yn;
	}

	/* |Id| = (√(k²+4y²) − k)/2，同样有理化：小电流时两个 40 量级的数相减会
	 * 直接归零（1e-6 A 的指令下实测 u = 0），而实际应有 u ∝ Iq² */
	u = 2.0f*y*y/(s + k);
	if(!(u > 0.0f)){ return out; }

	lim = id_frac*i_max;
	if(u > lim){
		/* 极端凸极 / 过紧的 id_frac（正常参数下进不来）：夹住 Id，再用保转矩的
		 * Iq 收口。夹过 Id 之后 |I|² = u² + y² 不再自动 ≤ Imax²，再用
		 * √(Imax²−u²) 兜一次，不变式 1 才是无条件成立的 */
		float q2;

		u = lim;
		y = a*(lambda/(lambda + absdL*u));
		q2 = i_max*i_max - u*u;
		q2 = (q2 > 0.0f) ? sqrtf(q2) : 0.0f;
		if(y > q2){ y = q2; }
	}
	if(!(y > 0.0f)){ return out; }

	out.id = (dL < 0.0f) ? -u : u;
	out.iq = (iq_cmd < 0.0f) ? -y : y;

	/* 最后一道安全阀：等效磁链必须变大，否则整条路径退回 SPM */
	if(!(ml_mtpa_lambda_eff(lambda, dL, out.id) >= lambda)){
		out.id = 0.0f;
		out.iq = iq_cmd;
	}
	return out;
}

#endif /* MTPA_LAW_H */
