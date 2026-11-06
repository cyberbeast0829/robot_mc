/*
 * test_motion_law.c
 *
 * 速度环 / 位置环纯数学部分的 host 侧回归测试（不需要目标板）。
 *
 * 覆盖：
 *   - 钳位与速度设定值斜率限制（NaN 目标不能把设定值锁死）
 *   - 速度 PI：零误差、纯 P、限幅、抗饱和（饱和期间积分不累积）、NaN 输入
 *   - 位置 P
 *   - 位置跟随误差故障：阈值、去抖时间、饱和时不判定、0 = 关闭
 *   - PLL 增益的离散稳定性钳位
 *   - PLL 收敛（匀速、静止零速吸附、单圈回绕）
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "motion_law.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

#define DT25 0.000025f

static void test_clamp(void)
{
    CHECK_NEAR(ml_clampf(1.0f, -2.0f, 2.0f), 1.0f, 0.0f);
    CHECK_NEAR(ml_clampf(-3.0f, -2.0f, 2.0f), -2.0f, 0.0f);
    CHECK_NEAR(ml_clampf(3.0f, -2.0f, 2.0f), 2.0f, 0.0f);
}

static void test_slew(void)
{
    /* accel 关闭：直接跟随 */
    CHECK_NEAR(ml_slew(0.0f, 5.0f, 0.0f, DT25), 5.0f, 0.0f);

    /* accel = 1000 rad/s^2，一拍 25 us 只能走 0.025 rad/s */
    float v = 0.0f;
    int steps = 0;
    while (v < 1.0f && steps < 1000) { v = ml_slew(v, 1.0f, 1000.0f, DT25); steps++; }
    CHECK_NEAR(v, 1.0f, 1e-5f);
    CHECK(steps >= 40 && steps <= 41);  /* 1.0 / 0.025 = 40 拍（float32 累积误差 ±1 拍） */

    /* 反方向同样受限 */
    CHECK_NEAR(ml_slew(0.0f, -1.0f, 1000.0f, DT25), -0.025f, 1e-6f);

    /* NaN 目标：保持当前值，不把 NaN 传下去 */
    CHECK_NEAR(ml_slew(0.5f, NAN, 1000.0f, DT25), 0.5f, 0.0f);
    CHECK_NEAR(ml_slew(0.5f, NAN, 0.0f, DT25), 0.5f, 0.0f);
}

static void test_vel_pi(void)
{
    float integ = 0.0f;

    /* 零误差、无积分 → 0 */
    CHECK_NEAR(ml_vel_pi(5.0f, 25.0f, 4.0f, &integ, 0.0f, 0.0f, 0.0002f), 0.0f, 0.0f);

    /* 纯 P：err = 0.1 rad/s，gain = 5 A/(rad/s) → 0.5 A */
    integ = 0.0f;
    CHECK_NEAR(ml_vel_pi(5.0f, 0.0f, 4.0f, &integ, 0.1f, 0.0f, 0.0002f), 0.5f, 1e-6f);

    /* 限幅：err 很大 → 输出正好等于 i_max */
    integ = 0.0f;
    CHECK_NEAR(ml_vel_pi(5.0f, 25.0f, 1.0f, &integ, 100.0f, 0.0f, 0.0002f), 1.0f, 0.0f);
    CHECK(fabsf(integ) <= 1.0f + 1e-6f);

    /* 抗饱和：持续饱和 5000 拍后，积分器不得远离 ±i_max，
     * 且误差反向时输出要能很快离开饱和（不会因为积分顶住而迟钝） */
    integ = 0.0f;
    for (int i = 0; i < 5000; i++) {
        (void)ml_vel_pi(5.0f, 25.0f, 1.0f, &integ, 100.0f, 0.0f, 0.0002f);
    }
    CHECK(fabsf(integ) <= 1.0f + 1e-6f);

    float iq = 1.0f;
    int steps = 0;
    while (iq >= 1.0f && steps < 1000) {
        iq = ml_vel_pi(5.0f, 25.0f, 1.0f, &integ, -5.0f, 0.0f, 0.0002f);
        steps++;
    }
    CHECK(steps < 20);                  /* 0.99^n 衰减：约 3 拍就足够了 */

    /* NaN 输入 → 0 输出并清积分（绝不让 NaN 进电流环） */
    integ = 0.7f;
    CHECK_NEAR(ml_vel_pi(5.0f, 25.0f, 4.0f, &integ, NAN, 0.0f, 0.0002f), 0.0f, 0.0f);
    CHECK_NEAR(integ, 0.0f, 0.0f);
    integ = 0.7f;
    CHECK_NEAR(ml_vel_pi(5.0f, 25.0f, 4.0f, &integ, 0.0f, NAN, 0.0002f), 0.0f, 0.0f);
    CHECK_NEAR(integ, 0.0f, 0.0f);

    /* 电流限幅为 0（客户端显式要求零电流）→ 输出 0、积分清掉 */
    integ = 0.7f;
    CHECK_NEAR(ml_vel_pi(5.0f, 25.0f, 0.0f, &integ, 1.0f, 0.0f, 0.0002f), 0.0f, 0.0f);
    CHECK_NEAR(integ, 0.0f, 0.0f);

    /* 长稳态后积分应当逼近负载转矩对应的电流 */
    integ = 0.0f;
    for (int i = 0; i < 2000; i++) {
        (void)ml_vel_pi(5.0f, 25.0f, 4.0f, &integ, 0.0f, -0.01f, 0.0002f);
    }
    CHECK(integ > 0.0f);
    CHECK(integ < 4.0f);
}

static void test_pos_p(void)
{
    CHECK_NEAR(ml_pos_p(15.0f, 1.0f, 0.1f, 0.0f), 1.5f, 1e-6f);       /* 0.1 rad → 1.5 rad/s */
    CHECK_NEAR(ml_pos_p(15.0f, 1.0f, 0.0f, 2.0f), 2.0f, 1e-6f);        /* 纯前馈 */
    CHECK_NEAR(ml_pos_p(15.0f, 0.0f, 0.1f, 2.0f), 1.5f, 1e-6f);        /* 前馈关闭 */
    CHECK_NEAR(ml_pos_p(15.0f, 1.0f, NAN, 0.0f), 0.0f, 0.0f);          /* NaN 误差 → 0 */
    CHECK_NEAR(ml_pos_p(15.0f, 1.0f, 0.0f, NAN), 0.0f, 0.0f);          /* NaN 前馈 → 0 */
}

static void test_pos_fault(void)
{
    float timer = 0.0f;

    /* 阈值 0.5 rad，位置环周期 250 us：0.1 s 需要 400 拍（float32 累积，留一拍的余量） */
    int latched = 0;
    for (int i = 0; i < 399; i++) { latched |= ml_pos_fault(0.5f, &timer, 1.0f, 0, 0.00025f); }
    CHECK(latched == 0);
    for (int i = 0; i < 5; i++) { latched |= ml_pos_fault(0.5f, &timer, 1.0f, 0, 0.00025f); }
    CHECK(latched == 1);

    /* 误差回到阈值内 → 计时清零 */
    timer = 0.0f;
    for (int i = 0; i < 300; i++) { (void)ml_pos_fault(0.5f, &timer, 1.0f, 0, 0.00025f); }
    CHECK(timer > 0.05f);
    (void)ml_pos_fault(0.5f, &timer, 0.1f, 0, 0.00025f);
    CHECK_NEAR(timer, 0.0f, 0.0f);

    /* 速度限幅饱和（正在追赶）时不计时：长距离移动 / 上电对齐不能算故障 */
    timer = 0.0f;
    for (int i = 0; i < 2000; i++) {
        CHECK(ml_pos_fault(0.5f, &timer, 10.0f, 1, 0.00025f) == 0);
    }
    CHECK_NEAR(timer, 0.0f, 0.0f);

    /* 0 = 关闭 */
    timer = 0.0f;
    for (int i = 0; i < 5000; i++) { CHECK(ml_pos_fault(0.0f, &timer, 100.0f, 0, 0.00025f) == 0); }

    /* NaN 误差：不计时（不能因为未初始化就锁故障） */
    timer = 0.0f;
    CHECK(ml_pos_fault(0.5f, &timer, NAN, 0, 0.00025f) == 0);
    CHECK_NEAR(timer, 0.0f, 0.0f);
}

static void test_pll_gains(void)
{
    float kp, ki;

    ml_pll_gains(500.0f, DT25, &kp, &ki);
    CHECK_NEAR(kp, 1000.0f, 1e-3f);            /* kp = 2 * bw */
    CHECK_NEAR(ki, 250000.0f, 1.0f);           /* ki = 0.25 * kp^2 */
    CHECK(kp * DT25 < 1.0f);

    /* 非法带宽退化为默认值 */
    ml_pll_gains(0.0f, DT25, &kp, &ki);
    CHECK_NEAR(kp, 2.0f * ML_PLL_BW_DEFAULT, 1e-3f);

    /* 带宽过大时把 kp 压到 0.5/dt，保证离散稳定 */
    ml_pll_gains(1.0e6f, DT25, &kp, &ki);
    CHECK_NEAR(kp, 0.5f / DT25, 1.0f);
}

static void test_pll_converge(void)
{
    const float cpr = 65536.0f;
    const float speed_counts = 65536.0f;        /* 1 转/秒 */
    float pos = 0.0f, vel = 0.0f, kp, ki;

    ml_pll_gains(500.0f, DT25, &kp, &ki);

    /* 匀速：每拍前进 speed_counts * dt 个计数 */
    for (int i = 0; i < 4000; i++) {
        pos += 0.0f;                             /* 无效，仅为可读性 */
        float count = fmodf(speed_counts * DT25 * (float)(i + 1), cpr);
        ml_pll_step(&pos, &vel, count, cpr, kp, ki, DT25);
    }
    CHECK_NEAR(vel, speed_counts, speed_counts * 0.02f);

    /* 静止：速度必须归零（零速吸附），位置跟住不变的计数 */
    for (int i = 0; i < 4000; i++) {
        float count = fmodf(speed_counts * DT25 * 4000.0f, cpr);
        ml_pll_step(&pos, &vel, count, cpr, kp, ki, DT25);
    }
    CHECK_NEAR(vel, 0.0f, 1e-6f);

    /* 单圈回绕：从 CPR-1 跨到 0 不能产生巨大的相位差 */
    pos = cpr - 1.0f;
    vel = 0.0f;
    for (int i = 0; i < 100; i++) {
        ml_pll_step(&pos, &vel, 0.0f, cpr, kp, ki, DT25);
    }
    CHECK(fabsf(vel) < speed_counts * 0.5f);

    /* 正反转对称 */
    pos = 0.0f;
    vel = 0.0f;
    for (int i = 0; i < 4000; i++) {
        float count = fmodf(cpr + speed_counts * DT25 * (float)(i + 1) * -1.0f, cpr);
        if (count < 0.0f) { count += cpr; }
        ml_pll_step(&pos, &vel, count, cpr, kp, ki, DT25);
    }
    CHECK_NEAR(vel, -speed_counts, speed_counts * 0.02f);
}

static void test_fit_and_inertia(void)
{
    /* 直线 v = a*t + v0 的斜率必须精确复现（这是惯量标定的核心） */
    ml_fit_t fit;
    ml_fit_reset(&fit);
    const float a_true = 45.4f, v0 = -0.3f, dt = 1.0f / 40000.0f;
    for (int i = 0; i < 2000; i++) {
        float t = (float)i * dt;
        ml_fit_add(&fit, t, a_true * t + v0);
    }
    CHECK_NEAR(ml_fit_slope(&fit), a_true, a_true * 0.001f);

    /* 样本不足 → 0，不能让调用方拿到垃圾斜率 */
    ml_fit_reset(&fit);
    ml_fit_add(&fit, 0.0f, 1.0f);
    CHECK_NEAR(ml_fit_slope(&fit), 0.0f, 0.0f);

    /* 惯量反演：J·a = KT·GR·i - c，用两个台阶消掉 c（重力+库仑摩擦）。
     * 台阶电流按"突破电流"自适应放大，所以重关节用大电流（同固件里的做法）。 */
    const float kt = 0.12317f, gr = 7.75f;
    {
        struct { float j, c, i_lo, i_hi; } cases[] = {
            {0.006f, 0.30f, 0.6f, 1.2f},      /* 轻关节：摩擦占 1/2 个台阶 */
            {0.5f,   1.00f, 2.0f, 4.0f},      /* 中等 */
            {3.0f,   1.50f, 4.5f, 7.5f},      /* 重关节：电流也要大 */
        };
        for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
            float a_lo = (kt * gr * cases[k].i_lo - cases[k].c) / cases[k].j;
            float a_hi = (kt * gr * cases[k].i_hi - cases[k].c) / cases[k].j;
            float j = 0.0f;
            CHECK(ml_inertia_from_slopes(kt, gr, cases[k].i_lo, a_lo,
                                         cases[k].i_hi, a_hi, &j) == 1);
            CHECK_NEAR(j, cases[k].j, cases[k].j * 0.001f);
        }
    }

    /* 拒收：台阶电流差太小 / 加速度差太小（摩擦没突破）/ KT 或 GR 非法 */
    float j = 0.0f;
    CHECK(ml_inertia_from_slopes(kt, gr, 1.0f, 10.0f, 1.01f, 90.0f, &j) == 0);
    CHECK(ml_inertia_from_slopes(kt, gr, 0.6f, 45.0f, 1.2f, 45.3f, &j) == 0);
    CHECK(ml_inertia_from_slopes(0.0f, gr, 0.6f, 45.0f, 1.2f, 140.0f, &j) == 0);
    CHECK(ml_inertia_from_slopes(kt, 0.0f, 0.6f, 45.0f, 1.2f, 140.0f, &j) == 0);
}

static void test_tune_gains(void)
{
    /* 台架实测参数：J = 0.0026 kg·m²、KT = 0.1255、GR = 7.75 */
    const float j = 0.00261389f, kt = 0.125466f, gr = 7.75f;
    float vg = 0.0f, vig = 0.0f, pg = 0.0f;

    CHECK(ml_tune_gains(j, kt, gr, 450.0f, 1.0f, &vg, &vig, &pg) == 1);
    CHECK_NEAR(vg, j * 450.0f / (kt * gr), 1e-6f);          /* ≈ 1.05 A/(rad/s) */
    CHECK_NEAR(vig, vg * 18.0f, vg * 1e-5f);                /* ω_i = 450/25 = 18 */
    CHECK_NEAR(pg, 15.0f, 1e-6f);                           /* ω_c/30（台架实测 22.5 会过冲） */

    /* 积分拐点夹在 [5, 30] rad/s：ω_c 很小时不跟着掉，很大时不跟着涨 */
    float vg4 = 0.0f, vig4 = 0.0f, pg4 = 0.0f;
    CHECK(ml_tune_gains(j, kt, gr, 50.0f, 1.0f, &vg4, &vig4, &pg4) == 1);
    CHECK_NEAR(vig4, vg4 * 5.0f, vg4 * 1e-5f);
    CHECK_NEAR(pg4, 1.6666666f, 1e-4f);
    CHECK(ml_tune_gains(j, kt, gr, 1200.0f, 1.0f, &vg4, &vig4, &pg4) == 1);
    CHECK_NEAR(vig4, vg4 * 30.0f, vg4 * 1e-5f);
    CHECK_NEAR(pg4, 40.0f, 1e-6f);

    /* margin 只放大速度环，位置环不受影响 */
    float vg2 = 0.0f, vig2 = 0.0f, pg2 = 0.0f;
    CHECK(ml_tune_gains(j, kt, gr, 450.0f, 2.6f, &vg2, &vig2, &pg2) == 1);
    CHECK_NEAR(vg2, vg * 2.6f, vg2 * 1e-5f);
    CHECK_NEAR(vig2, vig * 2.6f, vig2 * 1e-5f);
    CHECK_NEAR(pg2, pg, 1e-6f);

    /* margin 非法按 1 处理，不返回失败 */
    float vg3 = 0.0f;
    CHECK(ml_tune_gains(j, kt, gr, 450.0f, 0.0f, &vg3, &vig, &pg) == 1);
    CHECK_NEAR(vg3, vg, 1e-6f);

    /* 拒收：惯量未标定 / KT/GR/ω_c 非法 */
    CHECK(ml_tune_gains(0.0f, kt, gr, 450.0f, 1.0f, &vg, &vig, &pg) == 0);
    CHECK(ml_tune_gains(j, 0.0f, gr, 450.0f, 1.0f, &vg, &vig, &pg) == 0);
    CHECK(ml_tune_gains(j, kt, 0.0f, 450.0f, 1.0f, &vg, &vig, &pg) == 0);
    CHECK(ml_tune_gains(j, kt, gr, 0.0f, 1.0f, &vg, &vig, &pg) == 0);
}

/* ---- 输入整形：加速/减速分开的斜率限制 ---- */
static void test_slew2(void)
{
    /* 加速用 accel：0 → 1，accel 1000、dt 25 us ⇒ 每拍 0.025 */
    CHECK_NEAR(ml_slew2(0.0f, 1.0f, 1000.0f, 100.0f, DT25), 0.025f, 1e-6f);

    /* 减速用 decel：1 → 0（目标为 0 也算减速），同号收缩 ⇒ 每拍 0.0025 */
    CHECK_NEAR(ml_slew2(1.0f, 0.0f, 1000.0f, 100.0f, DT25), 0.9975f, 1e-6f);
    /* 同号收缩同样走 decel */
    CHECK_NEAR(ml_slew2(1.0f, 0.2f, 1000.0f, 100.0f, DT25), 0.9975f, 1e-6f);

    /* 反向（目标与当前异号）算加速，不受 decel 限制 */
    CHECK_NEAR(ml_slew2(-1.0f, 1.0f, 1000.0f, 100.0f, DT25), -0.975f, 1e-6f);

    /* |target| > |current| 且同号 → 加速 */
    CHECK_NEAR(ml_slew2(0.5f, 1.0f, 1000.0f, 100.0f, DT25), 0.525f, 1e-6f);

    /* decel = 0 ⇒ 退回 accel（老参数含义不变） */
    CHECK_NEAR(ml_slew2(1.0f, 0.0f, 1000.0f, 0.0f, DT25), 0.975f, 1e-6f);

    /* NaN 目标保持当前值 */
    CHECK_NEAR(ml_slew2(0.5f, NAN, 1000.0f, 100.0f, DT25), 0.5f, 0.0f);
}

/* ---- TRAP_TRAJ：规划与求值 ---- */
static void test_trap_plan_eval(void)
{
    ml_trap_t tr;
    float p, v, a;

    /* 长行程（梯形）：0 → 10 rad，vmax 5，a = d = 10
     * 加速 0.5 s 走 1.25 rad，减速同样，匀速 7.5 rad / 5 = 1.5 s */
    memset(&tr, 0, sizeof(tr));
    CHECK(ml_trap_plan(&tr, 10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 10.0f) == 1);
    CHECK_NEAR(tr.ta, 0.5f, 1e-4f);
    CHECK_NEAR(tr.tv, 1.5f, 1e-4f);
    CHECK_NEAR(tr.td, 0.5f, 1e-4f);
    CHECK_NEAR(tr.tf, 2.5f, 1e-4f);

    ml_trap_eval(&tr, 0.0f, &p, &v, &a);
    CHECK_NEAR(p, 0.0f, 1e-5f);
    CHECK_NEAR(a, 10.0f, 1e-5f);
    ml_trap_eval(&tr, 0.5f, &p, &v, &a);       /* 加速段结束：1.25 rad, 5 rad/s */
    CHECK_NEAR(p, 1.25f, 1e-4f);
    CHECK_NEAR(v, 5.0f, 1e-4f);
    ml_trap_eval(&tr, 1.0f, &p, &v, &a);       /* 匀速中：5.0 + 5*0.5 = 3.75 */
    CHECK_NEAR(p, 3.75f, 1e-4f);
    CHECK_NEAR(v, 5.0f, 1e-4f);
    CHECK_NEAR(a, 0.0f, 1e-6f);
    ml_trap_eval(&tr, 2.5f, &p, &v, &a);       /* 结束 */
    CHECK_NEAR(p, 10.0f, 1e-4f);
    CHECK_NEAR(v, 0.0f, 1e-6f);

    /* 短行程：0 → 0.5 rad 到不了 5 rad/s ⇒ 三角形，没有匀速段 */
    memset(&tr, 0, sizeof(tr));
    CHECK(ml_trap_plan(&tr, 0.5f, 0.0f, 0.0f, 5.0f, 10.0f, 10.0f) == 1);
    CHECK_NEAR(tr.tv, 0.0f, 0.0f);
    CHECK(tr.vr < 5.0f && tr.vr > 0.0f);
    ml_trap_eval(&tr, tr.tf, &p, &v, &a);
    CHECK_NEAR(p, 0.5f, 1e-4f);
    CHECK_NEAR(v, 0.0f, 1e-5f);

    /* 反向同样对称 */
    memset(&tr, 0, sizeof(tr));
    CHECK(ml_trap_plan(&tr, -10.0f, 0.0f, 0.0f, 5.0f, 10.0f, 10.0f) == 1);
    ml_trap_eval(&tr, tr.tf, &p, &v, &a);
    CHECK_NEAR(p, -10.0f, 1e-4f);
    CHECK_NEAR(v, 0.0f, 1e-5f);

    /* 起始速度已高于巡航速度：先减速（运动方向为负，所以要正的加速度） */
    memset(&tr, 0, sizeof(tr));
    CHECK(ml_trap_plan(&tr, 0.0f, 10.0f, -5.0f, 2.0f, 10.0f, 10.0f) == 1);
    CHECK(tr.ar > 0.0f);
    ml_trap_eval(&tr, tr.tf, &p, &v, &a);
    CHECK_NEAR(p, 0.0f, 1e-4f);
    CHECK_NEAR(v, 0.0f, 1e-5f);

    /* 非法限值一律拒绝 */
    memset(&tr, 0, sizeof(tr));
    CHECK(ml_trap_plan(&tr, 1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f) == 0);
    CHECK(ml_trap_plan(&tr, 1.0f, 0.0f, 0.0f, 5.0f, 0.0f, 10.0f) == 0);
    CHECK(ml_trap_plan(&tr, 1.0f, 0.0f, 0.0f, 5.0f, 10.0f, 0.0f) == 0);
}

/* ---- TRAP_TRAJ：按拍推进 ---- */
static void test_trap_step(void)
{
    ml_trap_t tr;
    float p = 0.0f, v = 0.0f;
    const float dt = 1.0f / 4000.0f;
    int steps = 0;
    int moving = 1;
    float vmax_seen = 0.0f;

    memset(&tr, 0, sizeof(tr));
    /* 0 → 2 rad，1 rad/s 巡航，10 rad/s² 加减速：约 2.1 s */
    while (moving && (steps < 40000)) {
        moving = ml_trap_step(&tr, dt, 2.0f, 1.0f, 10.0f, 10.0f, &p, &v);
        if (fabsf(v) > vmax_seen) { vmax_seen = fabsf(v); }
        steps++;
    }
    CHECK_NEAR(p, 2.0f, 1e-5f);                 /* 终点精确 */
    CHECK_NEAR(v, 0.0f, 1e-6f);
    CHECK(vmax_seen <= 1.0f + 1e-4f);           /* 不超巡航速度 */
    CHECK(steps > 8000 && steps < 9000);        /* ≈ 2.1 s * 4000 */

    /* 到位后再调用：停在终点，返回 0，位置不漂 */
    moving = ml_trap_step(&tr, dt, 2.0f, 1.0f, 10.0f, 10.0f, &p, &v);
    CHECK(moving == 0);
    CHECK_NEAR(p, 2.0f, 1e-5f);
    CHECK_NEAR(tr.t, tr.tf, 0.0f);              /* 时间冻结，不会无限增长 */

    /* 目标中途改变：从当前点续规划，不跳变 */
    memset(&tr, 0, sizeof(tr));
    p = 0.0f; v = 0.0f;
    for (int i = 0; i < 100; i++) { ml_trap_step(&tr, dt, 5.0f, 1.0f, 10.0f, 10.0f, &p, &v); }
    float p_before = p;
    CHECK(p > 0.0f && p < 1.0f);
    ml_trap_step(&tr, dt, -1.0f, 1.0f, 10.0f, 10.0f, &p, &v);   /* 立刻反向 */
    CHECK_NEAR(p, p_before, 0.02f);             /* 一拍内不跳 */
    CHECK(tr.xf == -1.0f);

    /* 非法的 vel/accel：退化成直接跟随目标，不卡死 */
    memset(&tr, 0, sizeof(tr));
    p = 0.0f; v = 9.0f;
    moving = ml_trap_step(&tr, dt, 3.0f, 0.0f, 0.0f, 0.0f, &p, &v);
    CHECK(moving == 0);
    CHECK_NEAR(p, 3.0f, 0.0f);
    CHECK_NEAR(v, 0.0f, 0.0f);
}

/* ---- POS_FILTER：二阶跟踪滤波 ---- */
static void test_posf(void)
{
    ml_posf_t f;
    float p = 0.0f, v = 0.0f;
    const float dt = 1.0f / 4000.0f;

    memset(&f, 0, sizeof(f));
    ml_posf_reset(&f, 0.0f, 0.0f);

    /* 目标阶跃 1 rad：位置单调逼近、速度先升后落、稳态无超调 */
    float p_prev = 0.0f;
    int converged = -1;
    for (int i = 0; i < 20000; i++) {
        ml_posf_step(&f, 1.0f, 0.0f, 10.0f, dt, &p, &v);
        CHECK(p >= p_prev - 1e-6f);             /* 临界阻尼不应过冲 */
        p_prev = p;
        if ((converged < 0) && (fabsf(p - 1.0f) < 1e-3f)) { converged = i; }
    }
    CHECK_NEAR(p, 1.0f, 1e-3f);
    CHECK_NEAR(v, 0.0f, 1e-3f);
    CHECK(converged > 0 && converged < 8000);   /* bw = 10 rad/s ⇒ 约 0.5 s 收敛 */

    /* 带宽过大时自动压低增益，不能发散 */
    memset(&f, 0, sizeof(f));
    ml_posf_reset(&f, 0.0f, 0.0f);
    for (int i = 0; i < 20000; i++) { ml_posf_step(&f, 1.0f, 0.0f, 100000.0f, dt, &p, &v); }
    CHECK(isfinite(p) && fabsf(p - 1.0f) < 0.05f);

    /* NaN 目标：保持当前值 */
    memset(&f, 0, sizeof(f));
    ml_posf_reset(&f, 0.25f, 0.0f);
    ml_posf_step(&f, NAN, 0.0f, 10.0f, dt, &p, &v);
    CHECK_NEAR(p, 0.25f, 1e-6f);
}

/* ---- 专用限值与轨迹上限的选择规则 ---- */
static void test_limits(void)
{
    /* ml_limit_or：>0 用覆盖值，0/负数/NaN 都退回 fallback */
    CHECK_NEAR(ml_limit_or(15.0f, 50.0f), 15.0f, 0.0f);
    CHECK_NEAR(ml_limit_or(0.0f, 50.0f), 50.0f, 0.0f);
    CHECK_NEAR(ml_limit_or(-3.0f, 50.0f), 50.0f, 0.0f);
    CHECK_NEAR(ml_limit_or(NAN, 50.0f), 50.0f, 0.0f);

    /* ml_trap_vel：未配置用运行期限速；配置更严格时取配置；更宽松时仍取运行期 */
    CHECK_NEAR(ml_trap_vel(0.0f, 65.0f), 65.0f, 0.0f);
    CHECK_NEAR(ml_trap_vel(5.0f, 65.0f), 5.0f, 0.0f);
    CHECK_NEAR(ml_trap_vel(100.0f, 65.0f), 65.0f, 0.0f);
    CHECK_NEAR(ml_trap_vel(-1.0f, 65.0f), 65.0f, 0.0f);

    /* VEL_RAMP 的两种配置：只配 accel/decel（默认）与配了 vel_ramp_rate（对称） */
    const float dt = 1.0f / 8000.0f;
    float v = 0.0f;
    v = ml_slew2(v, 10.0f, ml_limit_or(0.0f, 50.0f), ml_limit_or(0.0f, 50.0f), dt);
    CHECK_NEAR(v, 50.0f * dt, 1e-6f);                 /* 50 rad/s² */
    v = 0.0f;
    v = ml_slew2(v, 10.0f, ml_limit_or(15.0f, 50.0f), ml_limit_or(15.0f, 50.0f), dt);
    CHECK_NEAR(v, 15.0f * dt, 1e-6f);                 /* 15 rad/s²，加速与减速对称 */

    /* 减速同样走 vel_ramp_rate */
    v = ml_slew2(1.0f, 0.0f, ml_limit_or(15.0f, 50.0f), ml_limit_or(15.0f, 50.0f), dt);
    CHECK_NEAR(v, 1.0f - 15.0f * dt, 1e-6f);
}

int main(void)
{
    test_clamp();
    test_slew();
    test_vel_pi();
    test_pos_p();
    test_pos_fault();
    test_pll_gains();
    test_pll_converge();
    test_fit_and_inertia();
    test_tune_gains();
    test_slew2();
    test_limits();
    test_trap_plan_eval();
    test_trap_step();
    test_posf();

    if (failures == 0) {
        printf("test_motion_law: all tests passed\n");
        return 0;
    }
    printf("test_motion_law: %d failure(s)\n", failures);
    return 1;
}
