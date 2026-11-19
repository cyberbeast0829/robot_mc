/*
 * scope_law.h
 *
 * 电流环扫频（scope）的纯数学层：锁相解调、闭环→开环反算、穿越频率/相位裕度、
 * 阶跃指标。不引用任何固件头文件，host 可测（tools/test_scope_law.c）。
 *
 * 记号：激励为 u(t) = bias + amp·cos(θ(t))，θ 由 32 位相位累加器给出。
 * 一个累积窗取 N = 2^m 个控制周期、频点 k，则 θ 每拍前进 k·2π/N，窗口正好 k
 * 个周期 —— 离散求和没有泄漏，锁相解调的结果与相位起点无关。
 *
 * 复数约定：x = Re[X·e^{jθ}]，所以 X = A - jB，其中
 *   A = 2Σx·cosθ/N，B = 2Σx·sinθ/N（X 的幅度就是基波峰值）。
 * 闭环 T = X_fb/X_cmd；开环 L = T/(1-T)（测闭环反算开环，不需要开环注入）。
 */

#ifndef INC_SCOPE_LAW_H_
#define INC_SCOPE_LAW_H_

#include <stdint.h>
#include <math.h>

#define SL_PI 3.14159265358979f

/* 一个频点的锁相解调结果 */
typedef struct{
	float re, im;		/* 基波相量 X = re + j·im，含义是 x = Re[X·e^{jθ}]
						 * （即 re = A、im = -B，见 sl_lockin()） */
	float amp;			/* 基波峰值 */
	float dc;			/* 直流分量 */
	float resid;		/* 去掉直流与基波后的剩余 RMS（纹波/噪声/非线性） */
}sl_phasor_t;

/* 复数的基本运算（都是 static inline，host 与固件共用同一份实现） */
typedef struct{float re, im;}sl_cplx_t;

static inline sl_cplx_t sl_cplx_make(float re, float im)
{
	sl_cplx_t z; z.re = re; z.im = im; return z;
}

static inline sl_cplx_t sl_cplx_div(sl_cplx_t a, sl_cplx_t b)
{
	float d = b.re*b.re + b.im*b.im;
	if(!(d > 0.0f)){ return sl_cplx_make(0.0f, 0.0f); }
	return sl_cplx_make((a.re*b.re + a.im*b.im)/d, (a.im*b.re - a.re*b.im)/d);
}

static inline sl_cplx_t sl_cplx_sub(sl_cplx_t a, sl_cplx_t b)
{
	return sl_cplx_make(a.re - b.re, a.im - b.im);
}

static inline float sl_cplx_abs(sl_cplx_t a)
{
	return sqrtf(a.re*a.re + a.im*a.im);
}

/* 相角（度）。atan2f 很贵，只允许在主循环/上位机调用，不要在 40 kHz 中断里用 */
static inline float sl_cplx_arg_deg(sl_cplx_t a)
{
	return atan2f(a.im, a.re)*(180.0f/SL_PI);
}

/*
 * 锁相解调。入参是已经累加好的和（全部由中断逐点累加，避免在中断里做除法）：
 *   s_c = Σ x[n]·cosθ, s_s = Σ x[n]·sinθ, s_0 = Σ x[n], s_2 = Σ x[n]²
 * n 必须是激励周期的整数倍（scope.c 保证）。
 */
static inline sl_phasor_t sl_lockin(int n, float s_c, float s_s, float s_0, float s_2)
{
	sl_phasor_t p;
	p.re = 0.0f; p.im = 0.0f; p.amp = 0.0f; p.dc = 0.0f; p.resid = 0.0f;
	if(n <= 0){ return p; }
	const float inv = 1.0f/(float)n;
	const float a = 2.0f*s_c*inv;		/* A：cos 分量 */
	const float b = 2.0f*s_s*inv;		/* B：sin 分量 */
	p.re = a;
	p.im = -b;							/* x = Re[(A - jB)·e^{jθ}] */
	p.amp = sqrtf(a*a + b*b);
	p.dc = s_0*inv;
	float rms2 = s_2*inv - p.dc*p.dc - 0.5f*(a*a + b*b);
	if(!(rms2 > 0.0f)){ rms2 = 0.0f; }	/* 数值噪声可能让它略小于 0 */
	p.resid = sqrtf(rms2);
	return p;
}

/* 闭环 → 开环：L = T/(1-T)。T 用相量表示（|T|、相角）时先还原成复数。 */
static inline sl_cplx_t sl_closed_to_open(sl_cplx_t t)
{
	sl_cplx_t one = sl_cplx_make(1.0f, 0.0f);
	sl_cplx_t d = sl_cplx_sub(one, t);
	/* |1-T| 太小说明闭环增益 ≈ 1（低频段），此时 L 很大，直接除会溢出：
	 * 返回一个有限的大值，让上位机的曲线断在那里而不是变成 inf */
	float ad = sl_cplx_abs(d);
	if(ad < 1.0e-4f){
		float big = sl_cplx_abs(t)/1.0e-4f;
		return sl_cplx_make(big, 0.0f);
	}
	return sl_cplx_div(t, d);
}

/*
 * 从一张 (f, |L|, ∠L[deg]) 表里读穿越频率、相位裕度、增益裕度。
 * f[] 必须递增；返回 0 表示没有找到（例如整段都 < 0 dB，或相位从未到 -180°）。
 *   fc     |L| = 1 的频率（对数插值）
 *   pm     PM = 180 + ∠L(fc)
 *   gm_db  GM = -20·log10(|L|)（∠L = -180° 处）；找不到穿越点时置 0 并返回 1
 */
/* sl_margins() 内部解卷绕用的点数上限（台架一次扫频 20~30 个点，64 够用） */
#define SL_MAX_POINTS 64

static inline int sl_margins(const float *f, const float *mag, const float *phase_deg, int n,
                             float *fc, float *pm, float *gm_db)
{
	*fc = 0.0f; *pm = 0.0f; *gm_db = 0.0f;
	if((n < 2) || (f == 0) || (mag == 0) || (phase_deg == 0) || (n > SL_MAX_POINTS)){ return 0; }

	/* 相位先解卷绕：atan2 给的是 (-180, +180]，直接拿它找 -180° 穿越几乎永远
	 * 找不到，跨卷绕点插值还会把 PM 算成 180~350°（真值其实是负的、已经失稳）。
	 * 判据：相邻点跳变超过 180° 就说明跨了卷绕 */
	float ph[SL_MAX_POINTS];
	float off = 0.0f;
	ph[0] = phase_deg[0];
	for(int i = 1; i < n; i++){
		const float d = phase_deg[i] - phase_deg[i - 1];
		if(d > 180.0f){ off -= 360.0f; }
		else if(d < -180.0f){ off += 360.0f; }
		ph[i] = phase_deg[i] + off;
	}

	int found = 0;
	for(int i = 0; i + 1 < n; i++){
		float m0 = mag[i], m1 = mag[i + 1];
		if(!(isfinite(m0) && isfinite(m1))){ continue; }
		if(!((m0 > 0.0f) && (m1 > 0.0f))){ continue; }
		if(((m0 - 1.0f) == 0.0f) || ((m0 - 1.0f)*(m1 - 1.0f) < 0.0f)){
			/* 在 log|L| 与 log f 上插值（Bode 图的标准做法）：积分器 1/f 在这套
			 * 坐标里是直线，穿越频率因此是精确的；在线性幅值上插值会有 5% 量级的偏差 */
			float l0 = logf(m0), l1 = logf(m1);
			float t = 0.0f;
			if(l0 != l1){ t = l0/(l0 - l1); }
			if(!(t >= 0.0f)){ t = 0.0f; }
			if(t > 1.0f){ t = 1.0f; }
			float lf = logf(f[i]) + t*(logf(f[i + 1]) - logf(f[i]));
			*fc = expf(lf);
			*pm = 180.0f + ph[i] + t*(ph[i + 1] - ph[i]);
			found = 1;
			break;
		}
	}

	/* 增益裕度：相位第一次穿过 -180° 的地方（-360° 等更高次穿越不处理） */
	for(int i = 0; i + 1 < n; i++){
		float p0 = ph[i], p1 = ph[i + 1];
		if(!(isfinite(p0) && isfinite(p1))){ continue; }
		if((p0 > -180.0f) && (p1 <= -180.0f)){
			float t = 0.0f;
			if(p0 != p1){ t = (p0 + 180.0f)/(p0 - p1); }
			float db0 = (mag[i] > 1.0e-6f) ? (20.0f*log10f(mag[i])) : -120.0f;
			float db1 = (mag[i + 1] > 1.0e-6f) ? (20.0f*log10f(mag[i + 1])) : -120.0f;
			*gm_db = -(db0 + t*(db1 - db0));
			if(*gm_db > 60.0f){ *gm_db = 60.0f; }
			break;
		}
	}
	return found;
}

/*
 * 阶跃指标。x[] 是实测响应（A），dt 是采样间隔（s），前 pre 个点是阶跃前的基线。
 * 基线/终值取测量值而不是标称值：稳态误差本身也是要看的量。
 *   peak       最大值（漂移方向意义上的极值）
 *   overshoot  超调百分比（相对阶跃幅度；无超调为 0）
 *   t90        从阶跃点（第 pre 个点）起到达 90% 的时间（s），找不到返回 -1
 *   settle     |x - 终值| 最后一次超出 2% 幅度的时刻（s），且此后必须一直留在带内
 *
 * ⚠ 三个点都先在 3 抽头滑动平均上算：电流纹波 + ADC 噪声（本机 std ≈ 12 mA）
 * 和 0.5 A 阶跃的 2% 带（10 mA）是同一量级，直接取单点最大值/最小值会把纹波峰
 * 当成"超调"、把噪声当成"没稳定"（台架实测：未平滑的 settle 会读到整个窗口的
 * 末尾 23.8 ms，平滑后是 2.6 ms）。t90 在平滑信号上只差半个抽头。
 */
static inline float sl_ma3(const float *x, int n, int i)
{
	float a = x[(i > 0) ? (i - 1) : 0];
	float b = x[i];
	float c = x[(i < (n - 1)) ? (i + 1) : (n - 1)];
	return (a + b + c)*(1.0f/3.0f);
}

/* 判定"已稳定"用更长的滑动窗口：3 抽头平滑后的残差仍有 ±5 mA 的环路纹波，
 * 和 0.5 A 阶跃的 2% 带（10 mA）同量级，用瞬时值判会一直"出带"。0.8 ms 的
 * 箱式平均把环路纹波压掉一个数量级，同时 4 kHz 以下的稳态信息都还在 */
#define SL_SETTLE_WIN 33
/* 平滑后必须连续在 2% 带内这么多拍，才算"已稳定"（1 ms @ 40 kHz） */
#define SL_SETTLE_HOLD 40

static inline float sl_step_metrics(const float *x, int n, float dt, int pre,
                                    float *peak, float *overshoot, float *t90, float *settle)
{
	*peak = 0.0f; *overshoot = 0.0f; *t90 = -1.0f; *settle = -1.0f;
	if((x == 0) || (n < 8) || (pre < 1) || (pre >= n)){ return 0.0f; }

	float base = 0.0f;
	for(int i = 0; i < pre; i++){ base += sl_ma3(x, n, i); }
	base /= (float)pre;

	const int tail = (n - pre)/4;
	if(tail < 1){ return 0.0f; }
	float fin = 0.0f;
	for(int i = n - tail; i < n; i++){ fin += sl_ma3(x, n, i); }
	fin /= (float)tail;

	const float span = fin - base;
	const float dir = (span >= 0.0f) ? 1.0f : -1.0f;
	const float abs_span = fabsf(span);

	float pk = base;
	for(int i = pre; i < n; i++){
		float v = sl_ma3(x, n, i);
		if(dir*(v - pk) > 0.0f){ pk = v; }
	}
	*peak = pk;
	if(abs_span <= 1.0e-6f){ return span; }
	float ov = dir*(pk - fin)/abs_span;
	*overshoot = (ov > 0.0f) ? (100.0f*ov) : 0.0f;

	for(int i = pre; i < n; i++){
		if(dir*(sl_ma3(x, n, i) - base) >= 0.9f*abs_span){ *t90 = (float)(i - pre)*dt; break; }
	}

	/* 最后一次离开 2% 带的时刻（在 SL_SETTLE_WIN 箱式平均上判）；此后必须已经
	 * 连续在带内 SL_SETTLE_HOLD 拍，否则判"没稳定" */
	float ring[SL_SETTLE_WIN];
	float sum = 0.0f;
	int rp = 0, cnt = 0;
	int last_out = -1;
	for(int i = 0; i < n; i++){
		const float v = sl_ma3(x, n, i);
		sum += v;
		if(cnt < SL_SETTLE_WIN){ cnt++; }
		else{ sum -= ring[rp]; }
		ring[rp] = v;
		rp++; if(rp >= SL_SETTLE_WIN){ rp = 0; }
		if((i < pre) || (cnt < SL_SETTLE_WIN)){ continue; }
		if(fabsf(sum/(float)cnt - fin) > 0.02f*abs_span){ last_out = i; }
	}
	if((last_out >= 0) && ((n - 1 - last_out) >= SL_SETTLE_HOLD)){
		*settle = (float)(last_out + 1 - pre)*dt;
	}
	return span;
}

/* 定标：捕获缓冲用 int16，1 A = SCOPE_SCALE，1 V = SCOPE_VSCALE */
#define SL_SCALE_A		1000.0f
#define SL_SCALE_V		256.0f
#define SL_SCALE_THETA	4096.0f		/* 1 rad = 4096 counts（±8 rad） */

static inline int16_t sl_quant(float x, float scale)
{
	if(!(x == x)){ return 0; }					/* NaN */
	float c = x*scale;
	if(c > 32767.0f){ c = 32767.0f; }
	if(c < -32768.0f){ c = -32768.0f; }
	return (int16_t)((c >= 0.0f) ? (c + 0.5f) : (c - 0.5f));
}

static inline float sl_dequant(int16_t v, float scale)
{
	return (float)v/scale;
}

#endif /* INC_SCOPE_LAW_H_ */
