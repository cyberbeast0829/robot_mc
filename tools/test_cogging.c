/*
 * test_cogging.c
 *
 * 齿槽补偿纯数学层（cogging_law.h）的 host 侧回归测试，不需要目标板。
 *
 * 覆盖：
 *   - 定点量化/反量化的往返与饱和
 *   - 绝对计数 -> 圈内相位 / 表索引（含负数、回绕、四舍五入与 hold 法的旋转性质）
 *   - Catmull-Rom 插值：84 次/转的合成齿槽波形在 1024 点表上的重构误差
 *   - 表首尾环形连续
 *   - 正反两遍合并：取平均、抵消迟滞、坏点插值、去均值
 *   - 去趋势：重力/外载的慢变趋势被删掉，齿槽不受影响
 *   - RMS / 两遍差异
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "cogging_law.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

/* 合成的齿槽波形：幅值 0.4 A、84 次/转（12 槽 / 14 极对示例） */
static float cog_wave(float frac)
{
    return 0.4f * sinf(6.2831853f * 84.0f * frac)
         + 0.10f * sinf(6.2831853f * 168.0f * frac + 0.7f);
}

static void fill_wave(int16_t *lut, int n)
{
    for (int i = 0; i < n; i++) {
        lut[i] = cog_quant(cog_wave((float)i / (float)n));
    }
}

static void test_quant(void)
{
    CHECK_NEAR(cog_dequant(cog_quant(0.0f)), 0.0f, 1e-6f);
    CHECK_NEAR(cog_dequant(cog_quant(0.25f)), 0.25f, 1.0f / COG_SCALE);
    CHECK_NEAR(cog_dequant(cog_quant(-1.5f)), -1.5f, 1.0f / COG_SCALE);

    /* 饱和：不能让 int16 溢出 */
    CHECK(cog_quant(100.0f) == 32767);
    CHECK(cog_quant(-100.0f) == (int16_t)-32768);
    /* NaN 退化为 0（绝不能把 NaN 存进表） */
    CHECK(cog_quant(NAN) == 0);

    /* 一个表点的量化误差小于 0.5 LSB */
    CHECK_NEAR(cog_dequant(cog_quant(1.2345f)), 1.2345f, 0.5f / COG_SCALE);
}

static void test_frac(void)
{
    CHECK_NEAR(cog_frac_of_count(0, 65536), 0.0f, 0.0f);
    CHECK_NEAR(cog_frac_of_count(32768, 65536), 0.5f, 0.0f);
    CHECK_NEAR(cog_frac_of_count(65535, 65536), 65535.0f / 65536.0f, 1e-6f);
    /* 回绕与负数都要归一化到 [0,1) */
    CHECK_NEAR(cog_frac_of_count(65536, 65536), 0.0f, 0.0f);
    CHECK_NEAR(cog_frac_of_count(-1, 65536), 65535.0f / 65536.0f, 1e-6f);
    CHECK_NEAR(cog_frac_of_count(-65537, 65536), 65535.0f / 65536.0f, 1e-6f);
    CHECK_NEAR(cog_frac_of_count(3 * 65536 + 16384, 65536), 0.25f, 0.0f);
}

static void test_bin(void)
{
    const int n = COG_N;
    const int step = 65536 / n;

    /* 表点正好落在格心上 */
    for (int i = 0; i < n; i += 37) {
        CHECK(cog_bin_of_count(i * step, 65536, n) == i);
    }
    /* 四舍五入：半格以内归前一格，达到半格归后一格 */
    CHECK(cog_bin_of_count(step / 2 - 1, 65536, n) == 0);
    CHECK(cog_bin_of_count(step / 2, 65536, n) == 1);
    CHECK(cog_bin_of_count(n * step - 1, 65536, n) == 0);          /* 尾部折回 0 */
    CHECK(cog_bin_of_count(n * step - step / 2 - 1, 65536, n) == n - 1);
    /* 负数与多圈 */
    CHECK(cog_bin_of_count(-step, 65536, n) == n - 1);
    CHECK(cog_bin_of_count(-1, 65536, n) == 0);
    CHECK(cog_bin_of_count(65536 + step, 65536, n) == 1);

    /* 与相位版本一致（运行时两者必须指向同一格） */
    for (int c = -70000; c < 140000; c += 97) {
        int b = cog_bin_of_count(c, 65536, n);
        int expect = (int)(cog_frac_of_count(c, 65536) * (float)n + 0.5f) % n;
        CHECK(b == expect);
    }

    /* hold 法的关键性质：从任意计数 c0 起步、第 i 个点在 c0 + i·step，
     * 它的格号必须是 (bin(c0) + i) mod n —— 直接用点号 i 当索引会让整张表
     * 被旋转一个随机的 k0 格（曾经的真 bug） */
    const int c0 = 3000;                        /* 故意选在格与格之间 */
    int base = cog_bin_of_count(c0, 65536, n);
    for (int i = 0; i < n; i += 53) {
        CHECK(cog_bin_of_count(c0 + i * step, 65536, n) == ((base + i) % n));
        /* 但 c0 本身不是格心，所以 grid 起点要取整到格心才算准 */
        CHECK(cog_bin_of_count((base + i) % n * step, 65536, n) == ((base + i) % n));
    }
    CHECK(base != 0);                           /* 确保这个测试真的覆盖了旋转 */
}

static void test_eval(void)
{
    int16_t lut[COG_N];
    fill_wave(lut, COG_N);

    /* 84 次/转在 1024 点表上是 12.2 点/周期：三次插值的幅值/相位误差应在 1% 内
     * （线性插值在这里会差 3% 以上，这就是用 Catmull-Rom 的理由） */
    float peak = 0.0f, max_err = 0.0f;
    for (int k = 0; k < 20000; k++) {
        float frac = (float)k / 20000.0f;
        float ref = cog_wave(frac);
        float got = cog_eval(lut, COG_N, frac);
        float e = fabsf(got - ref);
        if (e > max_err) { max_err = e; }
        if (fabsf(ref) > peak) { peak = fabsf(ref); }
    }
    CHECK(peak > 0.35f);
    CHECK(max_err < 0.01f * 1.0f);          /* 绝对误差 < 10 mA */

    /* 首尾连续：0 与 1-eps 的差值跟相邻两点的差值同量级 */
    float a = cog_eval(lut, COG_N, 0.0f);
    float b = cog_eval(lut, COG_N, 1.0f / (float)COG_N);
    float c = cog_eval(lut, COG_N, 1.0f - 1e-4f);
    CHECK(fabsf(a - b) < 0.25f);
    CHECK(fabsf(c - cog_wave(1.0f - 1e-4f)) < 0.02f);

    /* 越界/NaN 输入不能读出表外 */
    CHECK(cog_eval(lut, COG_N, -1.0f) == cog_eval(lut, COG_N, 0.0f));
    CHECK(cog_eval(lut, COG_N, 2.0f) == cog_eval(lut, COG_N, 0.0f));
    CHECK(cog_eval(lut, COG_N, NAN) == cog_eval(lut, COG_N, 0.0f));
}

static void test_mean_remove(void)
{
    float v[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t ok[8] = {1, 1, 0, 1, 1, 1, 1, 1};       /* 第 2 点无效：不参与均值 */
    const float mean = 33.0f / 7.0f;                /* (1+2+4+5+6+7+8) / 7 */
    cog_mean_remove(v, ok, 8);
    CHECK_NEAR(v[0], 1.0f - mean, 1e-5f);
    CHECK_NEAR(v[7], 8.0f - mean, 1e-5f);
    CHECK_NEAR(v[2], 3.0f, 1e-6f);                  /* 无效点原样 */

    /* 全无效：清零，不留 NaN/旧值 */
    uint8_t none[4] = {0, 0, 0, 0};
    float w[4] = {9, 9, 9, 9};
    cog_mean_remove(w, none, 4);
    for (int i = 0; i < 4; i++) { CHECK(w[i] == 0.0f); }
}

static void test_rms(void)
{
    int16_t lut[4] = {4096, -4096, 4096, -4096};    /* ±1 A */
    CHECK_NEAR(cog_rms_i16(lut, 4), 1.0f, 1e-3f);

    float a[4] = {1, 2, 3, 4};
    float b[4] = {1.5f, 2.5f, 3.5f, 4.5f};
    uint8_t oka[4] = {1, 1, 1, 0};
    uint8_t okb[4] = {1, 1, 1, 1};
    CHECK_NEAR(cog_rms_diff(a, oka, b, okb, 4), 0.5f, 1e-6f);   /* 只有前 3 点参与 */
}

static void test_combine(void)
{
    /* 两遍 = 真值 ∓ 摩擦偏置（0.20 A），其中各有一个坏点 */
    float a[COG_N], b[COG_N];
    uint8_t oka[COG_N], okb[COG_N];
    int16_t out[COG_N];

    for (int i = 0; i < COG_N; i++) {
        float truth = cog_wave((float)i / (float)COG_N);
        a[i] = truth + 0.20f;
        b[i] = truth - 0.20f;
        oka[i] = 1;
        okb[i] = 1;
    }
    oka[100] = 0;                   /* 正转第 100 点没收敛 */
    okb[500] = 0;                   /* 反转第 500 点没收敛 */
    oka[900] = 0;
    okb[900] = 0;                   /* 两遍都没收敛：必须靠插值补上 */

    int bad = cog_combine(a, oka, b, okb, COG_N, out, 8.0f, 0);
    CHECK(bad == 1);

    float max_err = 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < COG_N; i++) {
        float got = cog_dequant(out[i]);
        sum += got;
        if ((i != 900) && (fabsf(got - cog_wave((float)i / (float)COG_N)) > max_err)) {
            max_err = fabsf(got - cog_wave((float)i / (float)COG_N));
        }
    }
    CHECK(max_err < 0.01f);                 /* 两遍平均后偏置抵消 */
    CHECK_NEAR(sum / (float)COG_N, 0.0f, 1e-3f);   /* 去均值 */

    /* 坏点也要落在邻点的合理范围内（不能是 0 或满量程） */
    CHECK(fabsf(cog_dequant(out[900])) < 0.6f);

    /* 全部无效：清零而不是留垃圾 */
    memset(oka, 0, sizeof(oka));
    memset(okb, 0, sizeof(okb));
    bad = cog_combine(a, oka, b, okb, COG_N, out, 8.0f, 0);
    CHECK(bad == COG_N);
    for (int i = 0; i < COG_N; i++) { CHECK(out[i] == 0); }
}

static void test_combine_limit(void)
{
    /* 前馈钳位：写到表里的值不能超过 limit */
    float a[16], b[16];
    uint8_t oka[16], okb[16];
    int16_t out[16];
    for (int i = 0; i < 16; i++) {
        a[i] = (i % 2) ? -5.0f : 5.0f;
        b[i] = a[i];
        oka[i] = 1;
        okb[i] = 1;
    }
    cog_combine(a, oka, b, okb, 16, out, 2.0f, 0);
    for (int i = 0; i < 16; i++) {
        CHECK(fabsf(cog_dequant(out[i])) <= 2.0f + 1e-4f);
    }
}

static void test_detrend(void)
{
    /* 齿槽 + 重力趋势（本机 GR=9：一个转子圈只走 1/9 输出圈）。
     * 三次多项式去趋势后应该几乎只剩齿槽本身 */
    float v[COG_N];
    const float g_amp = 4.0f;                 /* 重力项是齿槽的 10 倍 */
    const float turn = 6.28318530718f / 9.0f; /* 一个转子圈对应的输出轴角度 */
    for (int i = 0; i < COG_N; i++) {
        float x = (float)i / (float)COG_N;
        v[i] = cog_wave(x) + g_amp * cosf(0.3f + turn * x) + 2.5f;
    }
    uint8_t ok[COG_N];
    for (int i = 0; i < COG_N; i++) { ok[i] = 1; }
    cog_mean_remove(v, ok, COG_N);
    cog_detrend_poly(v, COG_N, 3);

    float max_err = 0.0f;
    for (int i = 0; i < COG_N; i++) {
        float e = fabsf(v[i] - cog_wave((float)i / (float)COG_N));
        if (e > max_err) { max_err = e; }
    }
    CHECK(max_err < 0.03f);                   /* 三次拟合的残差 ~ 四次项量级 */

    /* 0 = 不处理：重力留在里面（确认去趋势真的起作用） */
    float w[COG_N];
    for (int i = 0; i < COG_N; i++) {
        float x = (float)i / (float)COG_N;
        w[i] = cog_wave(x) + g_amp * cosf(0.3f + turn * x) + 2.5f;
    }
    cog_mean_remove(w, ok, COG_N);
    cog_detrend_poly(w, COG_N, 0);
    float rms = 0.0f;
    for (int i = 0; i < COG_N; i++) { rms += w[i] * w[i]; }
    /* 去均值后重力项剩下的是它在这 40° 里的变化量（约 ±0.4 A），与齿槽叠加
     * 后 RMS ≈ 0.57；纯齿槽只有 0.29 */
    CHECK(sqrtf(rms / (float)COG_N) > 0.4f);

    /* 齿槽本身不能被去趋势吃掉：无重力时三次去趋势后幅值基本不变 */
    float u[COG_N];
    for (int i = 0; i < COG_N; i++) { u[i] = cog_wave((float)i / (float)COG_N); }
    float before = 0.0f;
    for (int i = 0; i < COG_N; i++) { before += u[i] * u[i]; }
    cog_detrend_poly(u, COG_N, 3);
    float after = 0.0f;
    for (int i = 0; i < COG_N; i++) { after += u[i] * u[i]; }
    CHECK(fabsf(after - before) < 0.01f * before);

    /* 三次多项式本身应被完全去掉（拟合/消元的正确性） */
    float q[COG_N];
    for (int i = 0; i < COG_N; i++) {
        float x = (2.0f * (float)i - (float)(COG_N - 1)) / (float)(COG_N - 1);
        q[i] = 1.0f + 2.0f * x - 3.0f * x * x + 0.5f * x * x * x;
    }
    cog_detrend_poly(q, COG_N, 3);
    for (int i = 0; i < COG_N; i++) { CHECK(fabsf(q[i]) < 1e-3f); }
}

int main(void)
{
    test_quant();
    test_frac();
    test_bin();
    test_eval();
    test_mean_remove();
    test_rms();
    test_combine();
    test_combine_limit();
    test_detrend();

    if (failures == 0) {
        printf("test_cogging: all tests passed\n");
        return 0;
    }
    printf("test_cogging: %d failure(s)\n", failures);
    return 1;
}
