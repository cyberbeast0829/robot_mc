/*
 * test_dual_gear.c
 *
 * 齿轮式双编（韦尼尔双齿轮）解算的 host 侧回归测试（不需要目标板）。
 *
 * 覆盖：
 *   - 参数合法性（齿数相同 / k = 2 也合法 / 齿数 0 / 量程 dg_window_turns）
 *   - 相位 Δ 对安装偏置不变、每个相位周期回到原值
 *   - 上电解算：k = +1 / -1 / >1、副编码器读数反向（镜像），参考前后 ± 半个
 *     相位周期内都能还原出正确的圈数（误差 < 1 count）
 *   - 副编码器 ±2 count 噪声下仍只允许整圈级歧义（不许出现系统性的半圈错）
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "dual_gear.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

#define CPR 65536

static DualGearParams mk(int32_t t1, int32_t t2, uint8_t reversed)
{
    DualGearParams p;
    p.t_first = t1;
    p.t_sec = t2;
    p.first_cpr = CPR;
    p.sec_cpr = CPR;
    p.reversed = reversed;
    return p;
}

/* 机械模型：给定转子相对某个绝对原点的圈数 theta，返回两路编码器的单圈读数。
 * off_f / off_s 是两路编码器各自的安装偏置（等效于齿轮/磁铁装在哪）。 */
#define OFF_F 0.317f
#define OFF_S 0.731f

static void mech(const DualGearParams *p, float theta, int32_t *f_raw, int32_t *s_raw)
{
    float dir = p->reversed ? -1.0f : 1.0f;   /* 副编码器读数随正转变小 = 反向 */
    float f = dg_wrap01(theta + OFF_F);
    float s = dg_wrap01(dir * theta * (float)p->t_first / (float)p->t_sec + OFF_S);
    *f_raw = (int32_t)lroundf(f * (float)p->first_cpr) % p->first_cpr;
    *s_raw = (int32_t)lroundf(s * (float)p->sec_cpr) % p->sec_cpr;
}

static void test_params(void)
{
    DualGearParams p;

    p = mk(34, 33, 0);
    CHECK(dg_params_ok(&p));
    CHECK(dg_period_turns(&p) == 33);

    p = mk(33, 34, 0);
    CHECK(dg_params_ok(&p));
    CHECK(dg_k(&p) == -1);
    CHECK(dg_period_turns(&p) == 34);

    p = mk(34, 34, 0);
    CHECK(!dg_params_ok(&p));               /* k = 0 */

    /* k = 2 也合法：限制量程的是 k/T2（见 dg_window_turns），不是 gcd */
    p = mk(34, 32, 0);
    CHECK(dg_params_ok(&p));
    CHECK_NEAR(dg_window_turns(&p), 8.0f, 1e-6f);

    p = mk(35, 32, 0);
    CHECK(dg_params_ok(&p));                /* k = 3, gcd(3,32) = 1 */
    CHECK(dg_period_turns(&p) == 32);

    p = mk(0, 33, 0);
    CHECK(!dg_params_ok(&p));

    /* reversed 只是镜像读数，合法性与齿数无关 */
    p = mk(34, 33, 1);
    CHECK(dg_params_ok(&p));
    CHECK(dg_k(&p) == 1);

    p = mk(34, 33, 0);
    CHECK(dg_params_ok(&p));
    CHECK(dg_period_turns(&p) == 33);
}

static void test_phase(void)
{
    DualGearParams p = mk(34, 33, 0);
    int32_t f0, s0, f1, s1;

    /* Δ 与 theta 的绝对原点无关：同一个物理位置的两种表示必须一致 */
    mech(&p, 3.25f, &f0, &s0);
    mech(&p, 3.25f + 33.0f, &f1, &s1);       /* 一个韦尼尔周期之后 */
    CHECK(fabsf(dg_phase(f0, s0, &p) - dg_phase(f1, s1, &p)) < 1.0f / CPR);

    /* Δ 必须落在 [0,1) */
    for(int i = 0; i < 200; i++){
        float th = -20.0f + 0.13f * (float)i;
        mech(&p, th, &f0, &s0);
        float ph = dg_phase(f0, s0, &p);
        CHECK(ph >= 0.0f && ph < 1.0f);
    }

    /* 单圈内 Δ 的变化率就是 k/T2：转一圈应当变化 1/33 */
    mech(&p, 5.0f, &f0, &s0);
    mech(&p, 6.0f, &f1, &s1);
    float d = dg_phase(f1, s1, &p) - dg_phase(f0, s0, &p);
    d = dg_wrap01(d);
    CHECK(fabsf(d - 1.0f / 33.0f) < 5.0f / CPR);
}

/* 一次完整的"标定 → 断电 → 上电"流程：参考点取 theta_ref，上电时转子在
 * theta_ref + delta，要求还原出的位置与断电前连续。 */
static void roundtrip(const DualGearParams *p, float theta_ref, float delta,
                      int32_t turns_ref, int32_t noise_counts)
{
    int32_t f_ref, s_ref, f_now, s_now;

    mech(p, theta_ref, &f_ref, &s_ref);
    float phase_ref = dg_phase(f_ref, s_ref, p);
    int32_t count_ref = turns_ref * p->first_cpr + f_ref;    /* = turns*CPR + count_wrapped */

    mech(p, theta_ref + delta, &f_now, &s_now);
    s_now += noise_counts;

    float abs_turns = 0.0f;
    int32_t turns = dg_restore_turns(p, f_now, s_now, phase_ref, count_ref, f_now, &abs_turns);

    /* 上电后的位置（多圈，单位：圈）= turns + 小数部分，必须等于参考位置 + delta */
    float q_got = (float)turns + (float)f_now / (float)p->first_cpr;
    float q_ref = (float)count_ref / (float)p->first_cpr;
    float q_true = q_ref + delta;
    float err = fabsf(q_got - q_true);

    if(noise_counts == 0){
        if(err > 1.5f / (float)p->first_cpr){
            printf("FAIL %s: teeth %d/%d rev %d ref %.3f delta %.3f err %.6f turns\n",
                   __func__, p->t_first, p->t_sec, p->reversed, theta_ref, delta, err);
            failures++;
        }
        /* abs_turns（绝对圈数，与 turns 同一坐标系）= turns_ref + delta */
        if(fabsf(abs_turns - ((float)turns_ref + delta)) > 1.0f / 32.0f){
            printf("FAIL %s: abs_turns %.4f != %.4f\n", __func__, abs_turns, (float)turns_ref + delta);
            failures++;
        }
    }
    else{
        /* 有噪声时只允许"整整一圈"的歧义，绝不允许半圈/系统性的比例错 */
        if(err > 1.0f + 1.5f / (float)p->first_cpr){
            printf("FAIL %s(noise %d): teeth %d/%d rev %d ref %.3f delta %.3f err %.6f turns\n",
                   __func__, noise_counts, p->t_first, p->t_sec, p->reversed, theta_ref, delta, err);
            failures++;
        }
    }
}

static void test_roundtrip(void)
{
    DualGearParams p;

    /* k = +1：无歧义窗口 ±16.5 圈 */
    p = mk(34, 33, 0);
    for(int i = -16; i <= 16; i++){
        roundtrip(&p, 7.4f, (float)i + 0.37f, 0, 0);
        roundtrip(&p, -2.9f, (float)i + 0.11f, -5, 0);
    }
    roundtrip(&p, 1.0f, 0.0f, 3, 0);                    /* 正好在参考点 */

    /* k = -1（副齿轮齿数更大） */
    p = mk(33, 34, 0);
    for(int i = -16; i <= 16; i++){
        roundtrip(&p, 3.1f, (float)i + 0.17f, 2, 0);
    }

    /* k = 3：窗口 ±5.3 圈 */
    p = mk(35, 32, 0);
    for(int i = -5; i <= 4; i++){
        roundtrip(&p, 4.2f, (float)i + 0.23f, 0, 0);
    }

    /* 外啮合：k = -(34+33) = -67，窗口 ±0.246 圈（只用于验证方向） */
    p = mk(34, 33, 1);
    for(int i = -24; i <= 24; i++){
        roundtrip(&p, 1.7f, (float)i * 0.01f + 0.005f, 0, 0);
    }

    /* 副编码器噪声：±2 count 不应造成半圈级的系统误差 */
    p = mk(34, 33, 0);
    for(int i = -16; i <= 16; i++){
        roundtrip(&p, 7.4f, (float)i + 0.37f, 0, 2);
        roundtrip(&p, 7.4f, (float)i + 0.37f, 0, -2);
    }
}

/* 参考点附近越界一点点（半个周期之外）会落到另一支 —— 这是方案固有的歧义，
 * 这里只锁定"刚好在窗口边缘之内"的行为，防止有人把折叠写反 */
static void test_window_edge(void)
{
    DualGearParams p = mk(34, 33, 0);
    roundtrip(&p, 0.0f, 16.4f, 0, 0);
    roundtrip(&p, 0.0f, -16.4f, 0, 0);
    roundtrip(&p, 0.0f, 1.0f / 33.0f, 0, 0);            /* 细分同相点 */
}

int main(void)
{
    test_params();
    test_phase();
    test_roundtrip();
    test_window_edge();

    if(failures){
        printf("test_dual_gear: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_dual_gear: OK\n");
    return 0;
}
