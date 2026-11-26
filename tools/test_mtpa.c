/*
 * test_mtpa.c
 *
 * MTPA 纯数学（Core/Inc/mtpa_law.h）的 host 侧回归测试，不需要目标板。
 *
 * 覆盖：
 *   - 退化输入（ΔL=0、λ≤0、Imax≤0、id_frac=0、NaN/Inf）逐位退回 SPM
 *   - 符号：凸极电机（ΔL<0）无论转矩正负都应给出 Id<0
 *   - 保转矩：拆解前后 1.5p(λIq + ΔL·IdIq) 不变
 *   - 电流圆：Id²+Iq² ≤ Imax²，|Id| ≤ id_frac·Imax
 *   - 安全不变式：kt_eff = λ + ΔL·Id ≥ λ（MTPA 只会让每安培转矩变大）
 *   - 与电流圆上的**暴力扫描**比：饱和时输出的就是最大转矩点（Id 误差 < 2%）
 *   - 饱和判据的连续性（cap 前后 τ 不跳变）
 *   - 小电流的数值稳定性（有理化形式，无灾难性抵消）
 *   - 转矩对 iq_cmd 单调
 *   - ml_mtpa_gain_at_limit 与暴力扫描一致、且恒 ≥ 1
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "mtpa_law.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

/* 电机端转矩（p = 1，只用来比较，比例任意） */
static float tau_of(float lambda, float dL, float id, float iq)
{
    return lambda*iq + dL*id*iq;
}

/* 电流圆上扫 Id ∈ [-I, 0]，返回扫到的最大转矩及其 Id */
static void scan_max(float lambda, float dL, float I, float *tau_max, float *id_best)
{
    const int n = 400000;
    float best = -1e30f, best_id = 0.0f;
    for (int i = 0; i <= n; i++) {
        float id = -I * (float)i / (float)n;
        float q2 = I*I - id*id;
        float iq = (q2 > 0.0f) ? sqrtf(q2) : 0.0f;
        float t = tau_of(lambda, dL, id, iq);
        if (t > best) { best = t; best_id = id; }
    }
    *tau_max = best;
    *id_best = best_id;
}

/* 三组参数：本机台架（弱凸极）、中等凸极、强凸极 */
static const struct { float lambda, dL, I; const char *name; } motors[] = {
    { 0.001885f, -0.0000225f,  3.0f, "bench 11pp 22.8/45.3uH"    },
    { 0.008000f, -0.0002000f, 20.0f, "mid-saliency (xi=0.5)"     },
    { 0.010000f, -0.0003000f,100.0f, "strong-saliency (xi=3)"    },
    { 0.031100f, -0.0001000f, 50.0f, "high-flux, mild saliency"  },
};

static void test_degenerate(void)
{
    const float lam = 0.008f, dL = -0.0002f, I = 20.0f;
    ml_mtpa_t r;

    /* ΔL = 0：SPM，必须逐位恒等 */
    r = ml_mtpa_split(7.0f, lam, 0.0f, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);

    /* λ ≤ 0（未标定） */
    r = ml_mtpa_split(7.0f, 0.0f, dL, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);
    r = ml_mtpa_split(7.0f, -1.0f, dL, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);

    /* 电流上限/允许的 Id 比例为 0 */
    r = ml_mtpa_split(7.0f, lam, dL, 0.0f, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);
    r = ml_mtpa_split(7.0f, lam, dL, I, 0.0f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);

    /* NaN / Inf 输入：不能把 NaN 传进控制环 */
    r = ml_mtpa_split(NAN, lam, dL, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 0.0f);
    r = ml_mtpa_split(7.0f, lam, NAN, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);
    r = ml_mtpa_split(INFINITY, lam, dL, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 0.0f);
    /* dL = NaN 时 isfinite 挡住；dL 为 Inf 同理 */
    r = ml_mtpa_split(7.0f, lam, INFINITY, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 7.0f);
}

static void test_sign_and_torque(void)
{
    for (unsigned m = 0; m < sizeof(motors)/sizeof(motors[0]); m++) {
        const float lam = motors[m].lambda;
        const float dL  = motors[m].dL;
        const float I   = motors[m].I;
        const float iqs[] = { 0.1f, 1.0f, 0.5f, 0.95f };

        for (unsigned k = 0; k < sizeof(iqs)/sizeof(iqs[0]); k++) {
            const float iq_cmd = iqs[k]*I*(m == 2 ? 1.0f : 1.0f);
            for (int s = 0; s < 2; s++) {
                const float cmd = (s == 0) ? iq_cmd : -iq_cmd;
                ml_mtpa_t r = ml_mtpa_split(cmd, lam, dL, I, 0.9f);
                float tau_cmd = tau_of(lam, dL, 0.0f, cmd);
                float tau_out = tau_of(lam, dL, r.id, r.iq);

                /* 凸极电机：两个转向都应该是 Id < 0（磁阻转矩只与 Id·Iq 的积有关） */
                CHECK(r.id < 0.0f);
                /* 严格同号 */
                CHECK((cmd > 0.0f) ? (r.iq > 0.0f) : (r.iq < 0.0f));
                /* 保转矩（轨迹区，未饱和） */
                CHECK_NEAR(tau_out, tau_cmd, 1e-4f*fabsf(tau_cmd) + 1e-9f);
                /* 安全不变式：每安培转矩只会变大 */
                CHECK(ml_mtpa_lambda_eff(lam, dL, r.id) >= lam);
                /* 电流圆 */
                CHECK(r.id*r.id + r.iq*r.iq <= I*I*(1.0f + 1e-5f));
                CHECK(fabsf(r.id) <= 0.9f*I*(1.0f + 1e-5f));
            }
        }
    }
}

static void test_zero_and_tiny(void)
{
    const float lam = 0.008f, dL = -0.0002f, I = 20.0f;
    ml_mtpa_t r;

    r = ml_mtpa_split(0.0f, lam, dL, I, 0.9f);
    CHECK(r.id == 0.0f && r.iq == 0.0f);

    /* 小电流：有理化形式不能因抵消而丢精度（Id ∝ Iq²，此时应 ≈ 0） */
    const float tiny[] = { 1e-6f, 1e-4f, 1e-3f, 1e-2f };
    for (unsigned k = 0; k < sizeof(tiny)/sizeof(tiny[0]); k++) {
        r = ml_mtpa_split(tiny[k], lam, dL, I, 0.9f);
        CHECK(isfinite(r.id) && isfinite(r.iq));
        CHECK_NEAR(r.iq, tiny[k], 1e-6f*tiny[k] + 1e-12f);
        /* Id = 2ΔL·Iq²/(λ+√(λ²+4ΔL²Iq²)) → 量级 (2|ΔL|/2λ)·Iq² */
        {
            float expect = (dL/lam)*tiny[k]*tiny[k];   /* = 2ΔL·Iq²/(2λ) */
            CHECK(r.id < 0.0f);
            CHECK_NEAR(r.id, expect, 1e-3f*fabsf(expect) + 1e-15f);
        }
    }
}

static void test_circle_optimum(void)
{
    for (unsigned m = 0; m < sizeof(motors)/sizeof(motors[0]); m++) {
        const float lam = motors[m].lambda;
        const float dL  = motors[m].dL;
        const float I   = motors[m].I;
        float tau_scan, id_scan;
        ml_mtpa_t r;
        float tau_out;

        scan_max(lam, dL, I, &tau_scan, &id_scan);

        /* iq_cmd 远超电流圆：输出应是圆上的 MTPA 最优点 */
        r = ml_mtpa_split(10.0f*I, lam, dL, I, 0.95f);
        tau_out = tau_of(lam, dL, r.id, r.iq);

        /* Id 与暴力扫描一致（网格 1/400000 → 允许 2%） */
        CHECK(r.id < 0.0f);
        CHECK_NEAR(r.id, id_scan, 0.02f*fabsf(id_scan));
        /* 转矩不低于扫描值的 (1 - 1e-3) */
        CHECK(tau_out >= tau_scan*(1.0f - 1e-3f));
        /* 必须**明显优于** SPM（同电流） */
        CHECK(tau_out > tau_of(lam, dL, 0.0f, I)*(1.0f + 1e-4f));
        /* 电流圆 */
        CHECK_NEAR(r.id*r.id + r.iq*r.iq, I*I, 1e-4f*I*I);
    }
}

static void test_gain_helper(void)
{
    for (unsigned m = 0; m < sizeof(motors)/sizeof(motors[0]); m++) {
        const float lam = motors[m].lambda;
        const float dL  = motors[m].dL;
        const float I   = motors[m].I;
        float tau_scan, id_scan;
        float g = ml_mtpa_gain_at_limit(lam, dL, I, 0.95f);
        float g_scan;

        scan_max(lam, dL, I, &tau_scan, &id_scan);
        g_scan = tau_scan / tau_of(lam, dL, 0.0f, I);

        CHECK(g >= 1.0f);
        CHECK_NEAR(g, g_scan, 0.01f*g_scan);
    }
    /* 退化输入恒 = 1 */
    CHECK(ml_mtpa_gain_at_limit(0.0f, -0.0002f, 20.0f, 0.9f) == 1.0f);
    CHECK(ml_mtpa_gain_at_limit(0.008f, 0.0f, 20.0f, 0.9f) == 1.0f);
    CHECK(ml_mtpa_gain_at_limit(0.008f, -0.0002f, 0.0f, 0.9f) == 1.0f);
    CHECK(ml_mtpa_gain_at_limit(0.008f, -0.0002f, 20.0f, 0.0f) == 1.0f);
    /* 本机台架：增益应该在 1.001 附近（≈ +0.13%），用来确认量级没算错 */
    {
        float g = ml_mtpa_gain_at_limit(0.001885f, -0.0000225f, 3.0f, 0.95f);
        CHECK(g > 1.0005f && g < 1.005f);
    }
}

/* 饱和边界两侧：转矩与电流必须连续（不能出现"多给一截转矩"的阶跃） */
static void test_saturation_continuity(void)
{
    const float lam = 0.008f, dL = -0.0002f, I = 20.0f, frac = 0.95f;

    /* 找 cap：二分出"轨迹点刚好落在电流圆上"的 iq_cmd */
    float lo = 0.0f, hi = 3.0f*I;
    for (int k = 0; k < 60; k++) {
        float mid = 0.5f*(lo + hi);
        ml_mtpa_t r = ml_mtpa_split(mid, lam, dL, I, frac);
        float mag2 = r.id*r.id + r.iq*r.iq;
        if (mag2 < I*I*(1.0f - 1e-6f)) { lo = mid; } else { hi = mid; }
    }
    {
        ml_mtpa_t a = ml_mtpa_split(lo*0.999f, lam, dL, I, frac);
        ml_mtpa_t b = ml_mtpa_split(hi*1.001f, lam, dL, I, frac);
        float ta = tau_of(lam, dL, a.id, a.iq);
        float tb = tau_of(lam, dL, b.id, b.iq);
        CHECK_NEAR(ta, tb, 0.005f*ta);
        CHECK_NEAR(a.id, b.id, 0.02f*fabsf(a.id));
        CHECK_NEAR(a.iq, b.iq, 0.02f*fabsf(a.iq));
    }
}

/* 转矩对指令单调（饱和前严格增，饱和后持平） */
static void test_monotonic(void)
{
    const float lam = 0.010f, dL = -0.0003f, I = 100.0f;
    float prev = -1e30f;
    for (int k = 1; k <= 200; k++) {
        float cmd = I*1.3f*(float)k/200.0f;
        ml_mtpa_t r = ml_mtpa_split(cmd, lam, dL, I, 0.95f);
        float t = tau_of(lam, dL, r.id, r.iq);
        CHECK(t >= prev - 1e-6f*fabsf(prev));
        prev = t;
    }
}

static void test_live_limit(void)
{
    const int N = 49;
    const float frac = 0.9f;

    /* 温度降流/转矩限幅后，foc.c 的 commutate() 会把**实际** i_max 换成
     * i_q_peak = ml_mtpa_iq_cmd_at_limit(..., live, ...)（表仍锚在 Imax），
     * mtpa_apply() 再把指令钳到 i_q_peak。这里逐条验支撑这个做法的事实：
     *   1. 表查到的点落在这个**小**圆上（|I| = live）；
     *   2. 同电流下 MTPA 不劣于 SPM —— 收口写错（仍按 Imax 取 Id）时，
     *      强凸极 + 重度降流会掉到 0.72×，比不开 MTPA 还差；
     *   3. |Id| 仍受 id_frac 约束。 */
    for (unsigned m = 0; m < sizeof(motors)/sizeof(motors[0]); m++) {
        const float lam = motors[m].lambda, dL = motors[m].dL, I = motors[m].I;
        float tab_id[N], tab_iq[N];
        const float a_max = I*ml_mtpa_gain_at_limit(lam, dL, I, frac);
        const float live_fracs[] = { 0.15f, 0.30f, 0.50f, 0.80f, 1.00f };

        CHECK(a_max >= I);
        for (int k = 0; k < N; k++) {
            ml_mtpa_t r = ml_mtpa_split(a_max*(float)k/(float)(N-1), lam, dL, I, frac);
            tab_id[k] = r.id;
            tab_iq[k] = fabsf(r.iq);
        }

        for (unsigned j = 0; j < sizeof(live_fracs)/sizeof(live_fracs[0]); j++) {
            const float live = I*live_fracs[j];
            const float cap = ml_mtpa_iq_cmd_at_limit(lam, dL, live, frac);
            const float cmd = cap;              /* 饱和指令被钳到 cap */
            float x, f, id, iq;
            int i, i1;

            /* g 恒 ≥ 1：同电流下 MTPA 不会比 SPM 差（这是整个功能的立身之本） */
            CHECK(ml_mtpa_gain_at_limit(lam, dL, live, frac) >= 1.0f);
            CHECK(cap >= live*(1.0f - 1e-5f));

            /* 查表：与 foc.c 的 mtpa_apply() 同一套参数化与线性插值 */
            x = cmd*(float)(N-1)/a_max;
            if (x > (float)(N-1)) { x = (float)(N-1); }
            i = (int)x;
            f = x - (float)i;
            i1 = (i < N-1) ? i+1 : N-1;
            id = tab_id[i] + f*(tab_id[i1]-tab_id[i]);
            iq = tab_iq[i] + f*(tab_iq[i1]-tab_iq[i]);

            /* field_weaken()：保转矩重算 Iq，再按小圆收口 */
            {
                const float kt = lam + dL*id;
                const float q2 = live*live - id*id;
                const float qmax = (q2 > 0.0f) ? sqrtf(q2) : 0.0f;

                CHECK(kt >= lam);
                iq = cmd*(lam/kt);
                if (iq > qmax) { iq = qmax; }
            }

            /* 1. 在小圆上：插值误差 ≤ 1% */
            CHECK_NEAR(sqrtf(id*id + iq*iq), live, 0.01f*live);
            /* 2. 不劣于 SPM */
            CHECK(tau_of(lam, dL, id, iq) >= tau_of(lam, dL, 0.0f, live)*(1.0f - 1e-4f));
            /* 3. |Id| ≤ id_frac·live */
            CHECK(fabsf(id) <= frac*live*(1.0f + 2e-3f));
        }
    }
}

/* 凸极指数与实际增益的关系（文档里的收益表就是它） */
static void test_saliency_index(void)
{
    CHECK_NEAR(ml_mtpa_saliency(3.0f, 0.001885f, -0.0000225f), 0.0358f, 0.001f);
    CHECK_NEAR(ml_mtpa_saliency(100.0f, 0.010f, -0.0003f), 3.0f, 0.01f);
    CHECK(ml_mtpa_saliency(0.0f, 0.01f, -0.0003f) == 0.0f);
    CHECK(ml_mtpa_saliency(10.0f, 0.0f, -0.0003f) == 0.0f);

    /* ξ 很小时增益 ≈ 1 + ξ²/2 − ξ⁴/2 */
    {
        float xi = ml_mtpa_saliency(3.0f, 0.001885f, -0.0000225f);
        float g  = ml_mtpa_gain_at_limit(0.001885f, -0.0000225f, 3.0f, 0.95f);
        float approx = 1.0f + xi*xi/2.0f - xi*xi*xi*xi/2.0f;
        CHECK_NEAR(g, approx, 0.1f*(approx - 1.0f) + 1e-7f);
    }
}

/* 正凸极（Ld > Lq，少数电机）：公式应给出 Id > 0 且同样提高每安培转矩 */
static void test_positive_saliency(void)
{
    const float lam = 0.010f, dL = +0.0002f, I = 50.0f;
    ml_mtpa_t r = ml_mtpa_split(0.5f*I, lam, dL, I, 0.9f);
    CHECK(r.id > 0.0f);
    CHECK(ml_mtpa_lambda_eff(lam, dL, r.id) > lam);
    CHECK_NEAR(tau_of(lam, dL, r.id, r.iq), tau_of(lam, dL, 0.0f, 0.5f*I),
               1e-4f*fabsf(tau_of(lam, dL, 0.0f, 0.5f*I)));
}

/* 查表分辨率：与 foc.c 的 mtpa_service 同一套参数化（N 点、[0, I_MAX·g]、
 * split 用 I_MAX 作电流上限 —— 顶端落入饱和分支），插值结果与精确解比。
 * 中断里只做插值，这一步保证"省 650 拍"不是拿精度换的 */
static void test_table_resolution(void)
{
    enum { N = 49 };        /* 必须与 foc.c 的 MTPA_TAB_N 一致 */
    static float tid[N], tiq[N];

    for (unsigned m = 0; m < sizeof(motors)/sizeof(motors[0]); m++) {
        const float lam = motors[m].lambda;
        const float dL  = motors[m].dL;
        const float Imax = motors[m].I;
        const float frac = 0.9f;
        /* 与 mtpa_service() 完全一致 */
        const float a_max = Imax*ml_mtpa_gain_at_limit(lam, dL, Imax, frac);
        float inv_da = (float)(N - 1)/a_max;
        float worst_abs = 0.0f, worst_rel = 0.0f, worst_id = 0.0f;

        for (int k = 0; k < N; k++) {
            float a = (float)k*a_max/(float)(N - 1);
            ml_mtpa_t r = ml_mtpa_split(a, lam, dL, Imax, frac);
            tid[k] = r.id;
            tiq[k] = fabsf(r.iq);
        }
        for (int n = 0; n <= 400; n++) {
            float cmd = a_max*(float)n/400.0f;
            float x = cmd*inv_da;
            if (!(x <= (float)(N - 1))) { x = (float)(N - 1); }
            int i = (int)x;
            float f = x - (float)i;
            int i1 = (i < N - 1) ? (i + 1) : (N - 1);
            float id = tid[i] + f*(tid[i1] - tid[i]);
            float iq = tiq[i] + f*(tiq[i1] - tiq[i]);
            ml_mtpa_t ref = ml_mtpa_split(cmd, lam, dL, Imax, frac);
            float tau_tab = tau_of(lam, dL, id, iq);
            float tau_ref = tau_of(lam, dL, ref.id, ref.iq);
            /* 误差判据：绝对误差相对**满量程转矩**（λ·Imax）< 0.1%；
             * 指令 ≥ 10% Imax 时再要求相对误差 < 1%（小指令处的相对误差
             * 天然更大，但绝对量可以忽略） */
            float e_abs = fabsf(tau_tab - tau_ref)/(lam*Imax);
            float e_rel = fabsf(tau_tab - tau_ref)/(fabsf(tau_ref) + 1e-12f);
            float e_id  = fabsf(id - ref.id);
            if (e_abs > worst_abs) { worst_abs = e_abs; }
            if ((cmd >= 0.1f*Imax) && (e_rel > worst_rel)) { worst_rel = e_rel; }
            if (e_id  > worst_id)  { worst_id  = e_id; }
        }
        CHECK(worst_abs < 1e-3f);
        CHECK(worst_rel < 1e-2f);
        CHECK(worst_id < 2e-3f*Imax);
    }
}

/* torque_control() 的钳位上限：电流圆上还能交付的最大 iq_cmd（SPM 等价）。
 * 必须与 ml_mtpa_split() 的饱和分支一致 —— 否则指令放到上限时不在电流圆上 */
static void test_iq_cmd_at_limit(void)
{
    for (unsigned m = 0; m < sizeof(motors)/sizeof(motors[0]); m++) {
        const float lam = motors[m].lambda;
        const float dL  = motors[m].dL;
        const float I   = motors[m].I;
        const float fracs[] = { 0.95f, 0.9f, 0.5f };

        for (unsigned q = 0; q < sizeof(fracs)/sizeof(fracs[0]); q++) {
            const float frac = fracs[q];
            float cap = ml_mtpa_iq_cmd_at_limit(lam, dL, I, frac);
            ml_mtpa_t r, r2;

            CHECK(cap >= I);                       /* 只会变大 */
            CHECK_NEAR(ml_mtpa_gain_at_limit(lam, dL, I, frac), cap/I, 1e-6f*(cap/I));

            /* 在 cap 处：正好落在电流圆上，转矩 = λ·cap（与 SPM 口径一致） */
            r = ml_mtpa_split(cap, lam, dL, I, frac);
            CHECK_NEAR(r.id*r.id + r.iq*r.iq, I*I, 1e-4f*I*I);
            CHECK_NEAR(tau_of(lam, dL, r.id, r.iq), lam*cap, 1e-4f*lam*cap);
            /* |Id| ≤ frac·Imax */
            CHECK(fabsf(r.id) <= frac*I*(1.0f + 1e-5f));

            /* 略小于 cap：仍在圆内（不会因为浮点边界跳出） */
            r2 = ml_mtpa_split(0.995f*cap, lam, dL, I, frac);
            CHECK(r2.id*r2.id + r2.iq*r2.iq <= I*I*(1.0f + 1e-5f));
            /* 略大于 cap：饱和分支恒定（单调、不跳变） */
            {
                ml_mtpa_t r3 = ml_mtpa_split(1.2f*cap, lam, dL, I, frac);
                CHECK_NEAR(r3.id, r.id, 1e-5f*fabsf(r.id) + 1e-9f);
                CHECK_NEAR(r3.iq, r.iq, 1e-5f*fabsf(r.iq) + 1e-9f);
            }
        }
    }
    /* 退化输入 = i_max（旧行为） */
    CHECK(ml_mtpa_iq_cmd_at_limit(0.0f, -0.0002f, 20.0f, 0.9f) == 20.0f);
    CHECK(ml_mtpa_iq_cmd_at_limit(0.008f, 0.0f, 20.0f, 0.9f) == 20.0f);
    CHECK(ml_mtpa_iq_cmd_at_limit(0.008f, -0.0002f, 0.0f, 0.9f) == 0.0f);
    CHECK(ml_mtpa_iq_cmd_at_limit(0.008f, -0.0002f, 20.0f, 0.0f) == 20.0f);
}

int main(void)
{
    test_degenerate();
    test_sign_and_torque();
    test_zero_and_tiny();
    test_circle_optimum();
    test_gain_helper();
    test_saturation_continuity();
    test_monotonic();
    test_saliency_index();
    test_positive_saliency();
    test_live_limit();
    test_table_resolution();
    test_iq_cmd_at_limit();

    if (failures) {
        printf("test_mtpa: %d FAILURE(S)\n", failures);
        return 1;
    }
    printf("test_mtpa: OK\n");
    return 0;
}
