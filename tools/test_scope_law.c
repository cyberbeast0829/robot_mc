/*
 * test_scope_law.c
 *
 * 电流环扫频纯数学层（scope_law.h）的 host 侧回归测试，不需要目标板。
 *
 * 覆盖：
 *   - 定点量化/反量化（含饱和与 NaN）
 *   - 锁相解调：已知相量的幅度/相位/直流/残差，以及"二次谐波只落在残差里"
 *   - 闭环→开环反算：一阶闭环 T=1/(1+s/ωc) 应该还原出 L=ωc/s
 *   - 穿越频率/相位裕度：解析的 L=(ωc/jω)·e^{-jωTd}，与闭式解对照
 *     这组数字就是 scope 测 i_bw 极限时的判据，公式和符号错一格结果就全反
 *   - 阶跃指标：一阶上升的 t90、二阶过冲
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "scope_law.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

/* 锁相输入：x[n] = dc + A·cosθn + B·sinθn + harm·cos(2θn)，θn = 2πkn/N */
static void synth(int n, int k, float dc, float a, float b, float harm,
                  float *s_c, float *s_s, float *s_0, float *s_2)
{
	*s_c = *s_s = *s_0 = *s_2 = 0.0f;
	for (int i = 0; i < n; i++) {
		float th = 6.2831853f * (float)k * (float)i / (float)n;
		float x = dc + a * cosf(th) + b * sinf(th) + harm * cosf(2.0f * th);
		*s_c += x * cosf(th);
		*s_s += x * sinf(th);
		*s_0 += x;
		*s_2 += x * x;
	}
}

static void test_quant(void)
{
	CHECK(sl_quant(0.5f, SL_SCALE_A) == 500);
	CHECK_NEAR(sl_dequant(sl_quant(-1.25f, SL_SCALE_A), SL_SCALE_A), -1.25f, 1.0f / SL_SCALE_A);
	CHECK(sl_quant(100.0f, SL_SCALE_A) == 32767);			/* 饱和，不能溢出 int16 */
	CHECK(sl_quant(-100.0f, SL_SCALE_A) == (int16_t)-32768);
	CHECK(sl_quant(NAN, SL_SCALE_A) == 0);					/* NaN 绝不进缓冲 */
}

static void test_lockin(void)
{
	/* 3 个周期的 2048 点窗口，参考量 X = A - jB */
	const int n = 2048, k = 3;
	float sc, ss, s0, s2;
	synth(n, k, 1.0f, 0.25f, -0.10f, 0.0f, &sc, &ss, &s0, &s2);

	sl_phasor_t p = sl_lockin(n, sc, ss, s0, s2);
	CHECK_NEAR(p.re, 0.25f, 1e-4f);
	CHECK_NEAR(p.im, 0.10f, 1e-4f);						/* im = -B */
	CHECK_NEAR(p.amp, sqrtf(0.25f * 0.25f + 0.10f * 0.10f), 1e-4f);
	CHECK_NEAR(p.dc, 1.0f, 1e-4f);
	CHECK(p.resid < 1e-4f);

	/* 二次谐波不会污染基波，但必须出现在残差里（残差是"这条曲线可不可信"的指标）*/
	synth(n, k, 0.0f, 0.25f, 0.0f, 0.05f, &sc, &ss, &s0, &s2);
	p = sl_lockin(n, sc, ss, s0, s2);
	CHECK_NEAR(p.amp, 0.25f, 1e-4f);
	CHECK_NEAR(p.resid, 0.05f / sqrtf(2.0f), 1e-4f);

	/* n = 0 不能除零 */
	p = sl_lockin(0, 1.0f, 1.0f, 1.0f, 1.0f);
	CHECK(p.amp == 0.0f && p.dc == 0.0f);
}

static void test_closed_to_open(void)
{
	/* 一阶闭环 T = 1/(1+jω/ωc) ⇒ 开环 L = ωc/(jω) */
	const float wc = 6.2831853f * 1000.0f;				/* 1 kHz */
	const float freq[3] = {100.0f, 1000.0f, 3000.0f};
	for (int i = 0; i < 3; i++) {
		float w = 6.2831853f * freq[i];
		/* T = 1/(1 + j w/wc) */
		float d = 1.0f + (w / wc) * (w / wc);
		sl_cplx_t t = sl_cplx_make(1.0f / d, -(w / wc) / d);
		sl_cplx_t l = sl_closed_to_open(t);
		CHECK_NEAR(sl_cplx_abs(l), wc / w, 1e-2f * (wc / w));
		CHECK_NEAR(sl_cplx_arg_deg(l), -90.0f, 0.1f);
	}

	/* |1-T| ≈ 0（低频，闭环增益 ≈ 1）时不能返回 inf */
	sl_cplx_t l = sl_closed_to_open(sl_cplx_make(1.0f, 0.0f));
	CHECK(isfinite(l.re) && isfinite(l.im));
}

static void test_margins(void)
{
	/* 解析的电流环：L(jω) = (ωc/jω)·e^{-jωTd}，ωc = 2π·1000、Td = 50 µs（2 拍）
	 * ⇒ 穿越 1000 Hz、PM = 90° − ωc·Td = 72° */
	const float fc0 = 1000.0f, td = 50e-6f;
	enum { NP = 32 };
	float f[NP], mag[NP], ph[NP];
	for (int i = 0; i < NP; i++) {
		f[i] = 20.0f * powf(10.0f, (float)i / 12.0f);	/* 20 Hz .. 7.9 kHz，1/6 十倍频 */
		float w = 6.2831853f * f[i];
		float wc = 6.2831853f * fc0;
		mag[i] = wc / w;
		ph[i] = -90.0f - (w * td) * (180.0f / SL_PI);
	}

	float fc, pm, gm;
	int ok = sl_margins(f, mag, ph, NP, &fc, &pm, &gm);
	CHECK(ok);
	CHECK_NEAR(fc, fc0, 1.0f);
	CHECK_NEAR(pm, 72.0f, 0.5f);
	/* 相位 -180° 在 5 kHz（ω·Td = π/2），那里 |L| = 0.2 ⇒ GM = 14 dB */
	CHECK_NEAR(gm, 20.0f * log10f(5.0f), 0.3f);

	/* 没有穿越（整段都在 0 dB 以下）时应返回 0 */
	float lowf[2] = {10.0f, 100.0f}, lowm[2] = {0.1f, 0.01f}, lowp[2] = {-90.0f, -90.0f};
	CHECK(sl_margins(lowf, lowm, lowp, 2, &fc, &pm, &gm) == 0);
}

/* 相位卷绕：atan2 给的是 (-180, +180]，实机数据就是这样。不先解卷绕的话
 * -180° 穿越永远找不到、跨卷绕点插值会把 PM 算成 180~350°（真值可能是负的）*/
static float wrap180(float a)
{
	a = fmodf(a, 360.0f);
	if (a > 180.0f) { a -= 360.0f; }
	if (a < -180.0f) { a += 360.0f; }
	return a;
}

static void build_model(float fc0, float td, float *f, float *mag, float *ph, int n)
{
	for (int i = 0; i < n; i++) {
		float w = 6.2831853f * f[i];
		float wc = 6.2831853f * fc0;
		mag[i] = wc / w;
		ph[i] = -90.0f - (w * td) * (180.0f / SL_PI);
	}
}

static void test_margins_wrapped(void)
{
	/* 网格要够密：相邻频点的相位变化必须 < 180°，否则任何解卷绕都无法区分
	 * "真的跳了一大截"和"跨了卷绕"。1/10 十倍频、到 20 kHz 时 Δphase ≈ 0.00466·f
	 * = 93°，安全（真实扫频到 5 kHz、1/6 十倍频时只有 34°）*/
	enum { NP = 32 };
	float f[NP], mag[NP], ph[NP], wph[NP];
	for (int i = 0; i < NP; i++) { f[i] = 20.0f * powf(10.0f, (float)i / 10.0f); }

	build_model(1000.0f, 50e-6f, f, mag, ph, NP);
	float fc1, pm1, gm1;
	sl_margins(f, mag, ph, NP, &fc1, &pm1, &gm1);
	for (int i = 0; i < NP; i++) { wph[i] = wrap180(ph[i]); }

	float fc2, pm2, gm2;
	int ok = sl_margins(f, mag, wph, NP, &fc2, &pm2, &gm2);
	CHECK(ok);
	CHECK_NEAR(fc2, fc1, 1e-3f * fc1);
	CHECK_NEAR(pm2, pm1, 0.05f);
	CHECK_NEAR(gm2, gm1, 0.05f);

	/* 已经失稳：PM 必须是负数（真值 -10°），不能报成 350° */
	build_model(5555.0f, 50e-6f, f, mag, ph, NP);
	for (int i = 0; i < NP; i++) { wph[i] = wrap180(ph[i]); }
	sl_margins(f, mag, wph, NP, &fc2, &pm2, &gm2);
	CHECK(pm2 < 0.0f);
	CHECK_NEAR(pm2, -10.0f, 1.0f);
	CHECK(fabsf(fc2 - 5555.0f) < 0.01f * 5555.0f);
}

static void test_step(void)
{
	/* 一阶：x(t) = 1 - e^{-t/τ}，t90 = τ·ln10、无超调 */
	const float dt = 25e-6f, tau = 0.5e-3f;
	enum { N = 512, PRE = 32 };
	float x[N];
	for (int i = 0; i < N; i++) {
		x[i] = (i < PRE) ? 0.0f : (1.0f - expf(-(float)(i - PRE) * dt / tau));
	}
	float peak, ovs, t90, settle;
	float span = sl_step_metrics(x, N, dt, PRE, &peak, &ovs, &t90, &settle);
	CHECK_NEAR(span, 1.0f, 1e-3f);
	CHECK(ovs == 0.0f);
	CHECK_NEAR(t90, tau * logf(10.0f), 2.0f * dt);
	CHECK(settle > 3.0f * tau);							/* 2% 带 ≈ 3.9τ */

	/* 二阶（ζ=0.456 的 20% 过冲）：峰值应被判成 1.2 左右 */
	const float wn = 2.0f * 6.2831853f * 400.0f;
	const float zeta = 0.456f;
	const float wd = wn * sqrtf(1.0f - zeta * zeta);
	for (int i = 0; i < N; i++) {
		float t = (float)(i - PRE) * dt;
		x[i] = (i < PRE) ? 0.0f
		     : 1.0f - expf(-zeta * wn * t) * (cosf(wd * t) + (zeta * wn / wd) * sinf(wd * t));
	}
	sl_step_metrics(x, N, dt, PRE, &peak, &ovs, &t90, &settle);
	CHECK_NEAR(ovs, 20.0f, 2.0f);
	CHECK(t90 > 0.0f);

	/* 噪声/纹波：本机实测 1 A 偏置下的电流噪声 std ≈ 12 mA、0.5 A 阶跃。
	 * 未做平滑时"最后一次离开 2% 带"会被噪声一路推到最后（台架读到 23.8 ms），
	 * 平滑 + 连续 hold 之后必须回到几毫秒以内 */
	srand(12345);
	for (int i = 0; i < N; i++) {
		float t = (float)(i - PRE) * dt;
		float y = (i < PRE) ? 0.0f : 0.5f * (1.0f - expf(-t / 0.4e-3f));
		float noise = 0.012f * sqrtf(-2.0f * logf(1.0f - (float)rand() / (float)RAND_MAX))
		            * cosf(6.2831853f * (float)rand() / (float)RAND_MAX);
		x[i] = y + noise;
	}
	sl_step_metrics(x, N, dt, PRE, &peak, &ovs, &t90, &settle);
	CHECK(settle > 0.0f);
	CHECK(settle < 6.0e-3f);					/* 真实值 ~2.5 ms；噪声不该把它推到窗口末尾 */
	CHECK_NEAR(ovs, 0.0f, 25.0f);				/* 一阶上升 + 噪声：超调必须远小于 25% */
}

int main(void)
{
	test_quant();
	test_lockin();
	test_closed_to_open();
	test_margins();
	test_margins_wrapped();
	test_step();

	if (failures == 0) {
		printf("test_scope_law: all tests passed\n");
		return 0;
	}
	printf("test_scope_law: %d failure(s)\n", failures);
	return 1;
}
