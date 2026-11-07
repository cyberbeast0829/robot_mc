/*
 * cogging_law.h
 *
 * 齿槽转矩补偿的纯数学层（不引用任何固件头文件，host 可测：
 * tools/test_cogging.c）。
 *
 * 表是**一个转子机械周期**（= 编码器一整圈）内的 i_q 电流，int16 定点存储：
 *   COG_SCALE = 4096 counts/A  ->  ±8 A 量程、0.244 mA 分辨率
 * 用转子（而不是输出轴）做周期：齿槽来自转子磁钢与定子齿的相互作用，它的基波
 * 周期是 1/LCM(槽数, 2·极对数) 个机械周期，比输出轴周期短 GR 倍 —— 用输出轴
 * 做表要么把 GR 倍的高次谐波丢掉，要么表长乘 GR。
 *
 * 取表用 Catmull-Rom 三次插值（4 点）。12 槽/14 极对的本机齿槽基波是 84 次/转，
 * COG_N=1024 时只有 12 个点/周期，线性插值会削掉 3~10% 的幅值，三次插值基本
 * 没有这个误差，代价只有 4 次乘加。
 */

#ifndef INC_COGGING_LAW_H_
#define INC_COGGING_LAW_H_

#include <stdint.h>
#include <math.h>

#define COG_N		1024		// 一个转子机械周期内的表点数（必须整除 ENC_CPR，见 cogging.c）
#define COG_SCALE	4096.0f		// 1 A = 4096 counts
#define COG_MAX_A	7.99f		// int16 能表示的上限（留一点余量）

static inline float cog_dequant(int16_t v)
{
	return (float)v * (1.0f / COG_SCALE);
}

static inline int16_t cog_quant(float amps)
{
	if (!(amps == amps)) { return 0; }			// NaN
	float c = amps * COG_SCALE;
	if (c > 32767.0f) { c = 32767.0f; }
	if (c < -32768.0f) { c = -32768.0f; }
	return (int16_t)((c >= 0.0f) ? (c + 0.5f) : (c - 0.5f));
}

/* 位置（圈，0..1）取表值。lut 允许环形回绕，所以不需要哨兵元素。 */
static inline float cog_eval(const int16_t *lut, int n, float frac)
{
	if (!(frac >= 0.0f) || (frac >= 1.0f)) { frac = 0.0f; }	// NaN / 越界都不读表外
	float x = frac * (float)n;
	int i1 = (int)x;
	if (i1 >= n) { i1 = n - 1; if (i1 < 0) { return 0.0f; } }
	float t = x - (float)i1;
	int i0 = (i1 == 0) ? (n - 1) : (i1 - 1);
	int i2 = (i1 + 1 >= n) ? 0 : (i1 + 1);
	int i3 = (i2 + 1 >= n) ? 0 : (i2 + 1);

	float y0 = cog_dequant(lut[i0]);
	float y1 = cog_dequant(lut[i1]);
	float y2 = cog_dequant(lut[i2]);
	float y3 = cog_dequant(lut[i3]);

	/* Catmull-Rom: p(t) = y1 + c1 t + c2 t^2 + c3 t^3 */
	float c1 = 0.5f * (y2 - y0);
	float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
	float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
	return ((c3 * t + c2) * t + c1) * t + y1;
}

/* 绝对计数 -> 圈内的相位 0..1。count 可以是任意符号（按 cpr 取模）。 */
static inline float cog_frac_of_count(int32_t count, int32_t cpr)
{
	int32_t w = count % cpr;
	if (w < 0) { w += cpr; }
	return (float)w * (1.0f / (float)cpr);
}

/* 绝对计数 -> 表索引（0..n-1），四舍五入到最近的一格。
 *
 * **运行时查表、sweep 分箱、hold 存表必须用同一套约定**：LUT[i] 表示
 * count ≡ i·cpr/n (mod cpr) 处的值（cog_eval() 在 frac = i/n 时正好取 LUT[i]）。
 * 用 floor 分箱会让整张表偏半格（本机 22 阶上是 4° 的相位误差），hold 法如果
 * 直接用"从起点数起的点号"当索引，表还会被整体旋转一个随机的 k0 格。
 * w 接近 cpr 时四舍五入会进位到 n，这里折回 0。 */
static inline int cog_bin_of_count(int32_t count, int32_t cpr, int n)
{
	if ((cpr <= 0) || (n <= 0)) { return 0; }
	int32_t w = count % cpr;
	if (w < 0) { w += cpr; }
	int32_t b = (int32_t)(((int64_t)w * (int64_t)n + (int64_t)cpr / 2) / (int64_t)cpr);
	if (b >= n) { b = 0; }
	if (b < 0) { b = 0; }
	return (int)b;
}

/* 只对有效点求均值再减掉（重力/摩擦的直流分量）。没有有效点时清零。 */
static inline void cog_mean_remove(float *v, const uint8_t *ok, int n)
{
	float sum = 0.0f;
	int cnt = 0;
	for (int i = 0; i < n; i++) {
		if (ok[i]) { sum += v[i]; cnt++; }
	}
	if (cnt == 0) {
		for (int i = 0; i < n; i++) { v[i] = 0.0f; }
		return;
	}
	float mean = sum / (float)cnt;
	for (int i = 0; i < n; i++) {
		if (ok[i]) { v[i] -= mean; }
	}
}

/* 两遍扫描的差异（RMS，A）。只有两遍都有效的点参与，用来判断重复性。 */
static inline float cog_rms_diff(const float *a, const uint8_t *oka,
                                 const float *b, const uint8_t *okb, int n)
{
	float sum = 0.0f;
	int cnt = 0;
	for (int i = 0; i < n; i++) {
		if (oka[i] && okb[i]) {
			float d = a[i] - b[i];
			sum += d * d;
			cnt++;
		}
	}
	if (cnt == 0) { return 0.0f; }
	return sqrtf(sum / (float)cnt);
}

/* 表值的 RMS（A） */
static inline float cog_rms_i16(const int16_t *lut, int n)
{
	if (n <= 0) { return 0.0f; }
	float sum = 0.0f;
	for (int i = 0; i < n; i++) {
		float v = cog_dequant(lut[i]);
		sum += v * v;
	}
	return sqrtf(sum / (float)n);
}

/* 有效点的均值（cnt 返回参与的点数） */
static inline float cog_valid_mean(const float *v, const uint8_t *ok, int n, int *cnt)
{
	float sum = 0.0f;
	int c = 0;
	for (int i = 0; i < n; i++) {
		if (ok[i]) { sum += v[i]; c++; }
	}
	if (cnt) { *cnt = c; }
	return (c > 0) ? (sum / (float)c) : 0.0f;
}

/* 去掉一个 degree 阶多项式趋势（0 = 关闭），只用于主循环里的离线后处理。
 *
 * 为什么需要：齿槽标定测到的是"维持位置所需的总电流"，里面有重力/外载力矩，
 * 正反两遍抵消不了它（它对两遍是同一个方向）。重力作为转子位置的函数在一个
 * 转子机械圈里是**慢变趋势**：本机 GR=9，一个转子圈只走 1/9 输出圈（约 40°），
 * 这段余弦在这一圈里几乎就是一条三次曲线。而齿槽本身是 84/168… 次/转，
 * 三次多项式表达不出来，所以不会被误删。
 * 拟合在归一化坐标 x∈[-1,1] 上做（法方程条件数好），float 足够。
 * ⚠ 直接驱动（GR=1）时一个转子圈正好走完一个输出圈，重力是完整的一次谐波，
 * 应该用更高的阶数（见 config.cogging_detrend 的说明）。
 */
static inline void cog_detrend_poly(float *v, int n, int degree)
{
	if ((degree <= 0) || (n <= degree + 1)) { return; }
	if (degree > 6) { degree = 6; }

	const int m = degree + 1;
	float A[7][8];
	for (int r = 0; r < m; r++) {
		for (int c = 0; c <= m; c++) { A[r][c] = 0.0f; }
	}

	for (int i = 0; i < n; i++) {
		float x = (2.0f * (float)i - (float)(n - 1)) / (float)(n - 1);
		float xp[13];
		xp[0] = 1.0f;
		for (int k = 1; k < 2 * degree + 1; k++) { xp[k] = xp[k - 1] * x; }
		for (int r = 0; r < m; r++) {
			for (int c = 0; c < m; c++) { A[r][c] += xp[r + c]; }
			A[r][m] += xp[r] * v[i];
		}
	}

	/* 高斯消元（列主元） */
	for (int c = 0; c < m; c++) {
		int piv = c;
		for (int r = c + 1; r < m; r++) {
			if (fabsf(A[r][c]) > fabsf(A[piv][c])) { piv = r; }
		}
		if (fabsf(A[piv][c]) < 1e-12f) { return; }
		if (piv != c) {
			for (int k = c; k <= m; k++) {
				float t = A[c][k]; A[c][k] = A[piv][k]; A[piv][k] = t;
			}
		}
		float d = A[c][c];
		for (int k = c; k <= m; k++) { A[c][k] /= d; }
		for (int r = 0; r < m; r++) {
			if (r == c) { continue; }
			float f = A[r][c];
			if (f == 0.0f) { continue; }
			for (int k = c; k <= m; k++) { A[r][k] -= f * A[c][k]; }
		}
	}

	for (int i = 0; i < n; i++) {
		float x = (2.0f * (float)i - (float)(n - 1)) / (float)(n - 1);
		float p = 0.0f, xk = 1.0f;
		for (int k = 0; k < m; k++) {
			p += A[k][m] * xk;
			xk *= x;
		}
		v[i] -= p;
	}
}

/*
 * 合并正反两遍扫描并量化成表。
 *   v / ok      正转那一遍（**会被就地改写**成合并结果）
 *   b / okb     反转那一遍（只读）
 *   detrend     去掉 degree 阶多项式趋势（0 = 不去，见 cog_detrend_poly）
 * 两遍各自先减掉自己的均值（两遍的静摩擦方向相反，减完再平均迟滞项才抵消；
 * 重力/平均摩擦也一起去掉），然后：
 *   两遍都有效的点取平均，只有一遍有效就用那一遍，两遍都无效的点用相邻有效点
 *   线性插值补上（齿槽是连续函数，个别点没收敛不该让整张表作废）。
 * 最后整体去均值（+ 可选的去趋势）并量化。返回值是**两遍都无效**的点数。
 * 对已经去过均值的输入是幂等的（再减一次 0）。
 */
static inline int cog_combine(float *v, uint8_t *ok,
                              const float *b, const uint8_t *okb,
                              int n, int16_t *out, float limit, int detrend)
{
	int bad = 0;
	float ma = cog_valid_mean(v, ok, n, NULL);
	float mb = cog_valid_mean(b, okb, n, NULL);

	for (int i = 0; i < n; i++) {
		if (ok[i] && okb[i]) {
			v[i] = 0.5f * ((v[i] - ma) + (b[i] - mb));
		} else if (okb[i]) {
			v[i] = b[i] - mb;
			ok[i] = 1;
		} else if (ok[i]) {
			v[i] -= ma;
		} else {
			v[i] = 0.0f;
			bad++;
		}
	}

	if (bad == n) {
		for (int i = 0; i < n; i++) { out[i] = 0; }
		return bad;
	}

	/* 环形插值补洞：往两边找最近的有效点 */
	for (int i = 0; i < n; i++) {
		if (ok[i]) { continue; }
		int p = i, q = i, dp = 0, dq = 0;
		do { p = (p == 0) ? (n - 1) : (p - 1); dp++; } while (!ok[p] && (dp < n));
		do { q = (q + 1 == n) ? 0 : (q + 1); dq++; } while (!ok[q] && (dq < n));
		if ((dp + dq) >= n) {
			v[i] = 0.0f;
		} else {
			v[i] = v[p] + (v[q] - v[p]) * ((float)dp / (float)(dp + dq));
		}
		ok[i] = 1;			// 补过的点也参与最后的去均值 / 量化
	}

	cog_mean_remove(v, ok, n);		/* 补洞后 ok 全为 1，等价于整体去均值 */
	cog_detrend_poly(v, n, detrend);

	if (limit > COG_MAX_A) { limit = COG_MAX_A; }
	for (int i = 0; i < n; i++) {
		float x = v[i];
		if (x > limit) { x = limit; }
		if (x < -limit) { x = -limit; }
		out[i] = cog_quant(x);
	}
	return bad;
}

#endif /* INC_COGGING_LAW_H_ */
