/*
 * test_phase_current.c
 *
 * 低边分流电流重建（phase_current_law.h）的 host 侧回归测试，不需要目标板。
 *
 * 覆盖：
 *   - 低边窗口 ↔ CCR 的换算：PWM1 下低边是 CHx 的互补（CNT > CCR 才导通），
 *     峰值处的窗口 = AR - CCR —— 与 INVERT_DTC 无关（它只决定 CCR 怎么由 duty 算）
 *   - 选相：三相都够不重建；只重建窗口最小（CCR 最大）的那一相；相等时取小下标
 *   - 重建值：i_x = -i_y - i_z，另外两相不动，重建后三相之和为零
 *   - 本机的实际量级（AR = 3000、DTC_MAX = 0.9、阈值 15%）：满调制时 duty 最小的
 *     那一相窗口只剩 ~50 tick，判据必须选中它；旧上限 0.7 时它被钳到 0，更糟。
 *     改 DTC_MAX / I_SAMPLE_MIN_LOW_FRAC 时必须回来复查这里
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "phase_current_law.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

/* 本机：ATIM1->AR = 3000（40 kHz 中心对齐），I_SAMPLE_MIN_LOW_FRAC = 0.15 */
#define AR			3000u
#define MIN_WIN		((uint32_t)(0.15f*(float)AR + 0.5f))	/* 450 */

/* duty → set_dtc() 写入的比较值（INVERT_DTC = 1） */
static uint32_t ccr_of(float dtc)
{
	if(dtc < 0.0f){ dtc = 0.0f; }
	if(dtc > 1.0f){ dtc = 1.0f; }
	return (uint32_t)((float)AR*(1.0f - dtc) + 0.5f);
}

static void test_low_side_ticks(void)
{
	/* PWM1：低边是 CHx 的互补，只在 CNT > CCR 时导通 ⇒ 窗口 = AR - CCR。
	 * 本机 INVERT_DTC = 1 ⇒ CCR = AR*(1-dtc) ⇒ 窗口 ∝ dtc。 */
	CHECK(pcl_low_side_ticks(AR, 0) == AR);					/* CCR = 0：整周期低边 */
	CHECK(pcl_low_side_ticks(AR, AR) == 0);					/* CCR = AR：低边完全不导通 */
	CHECK(pcl_low_side_ticks(AR, ccr_of(0.90f)) == 2700u);	/* duty 0.9 → CCR 300 */
	CHECK(pcl_low_side_ticks(AR, ccr_of(0.00f)) == 0u);		/* duty 0   → CCR 3000 */
	/* 不做钳位：set_dtc() 保证 CCR ∈ [0, AR]；万一越界，窗口只会算得很大 →
	 * 判据不触发（退化成三相都用实测值），不会误重建 */
	CHECK(pcl_low_side_ticks(AR, AR + 12345u) > MIN_WIN);
}

static void test_pick(void)
{
	/* 三相都够：不重建 */
	CHECK(pcl_worst_phase(2000, 2000, 2000, MIN_WIN) == PCL_NONE);
	/* 边界：正好等于阈值算够 */
	CHECK(pcl_worst_phase(MIN_WIN, 2000, 2000, MIN_WIN) == PCL_NONE);
	CHECK(pcl_worst_phase(MIN_WIN - 1u, 2000, 2000, MIN_WIN) == 0);
	/* 任一相不够，就选它 */
	CHECK(pcl_worst_phase(2000, 10, 2000, MIN_WIN) == 1);
	CHECK(pcl_worst_phase(2000, 2000, 10, MIN_WIN) == 2);
	/* 两相都不够时仍只重建最差的那一相（KCL 只有一条约束），相等取小下标 */
	CHECK(pcl_worst_phase(300, 300, 2700, MIN_WIN) == 0);
	CHECK(pcl_worst_phase(300, 200, 2700, MIN_WIN) == 1);
}

static void test_reconstruct(void)
{
	/* 三相瞬时电流之和为零；a 相读数被振铃污染（7.0 而不是 4.0） */
	float ia = 7.0f, ib = -1.5f, ic = -2.5f;

	pcl_reconstruct(&ia, &ib, &ic, 0);
	CHECK(ia == 4.0f);
	CHECK(ib == -1.5f);
	CHECK(ic == -2.5f);

	ia = 7.0f; ib = 9.0f; ic = -2.5f;
	pcl_reconstruct(&ia, &ib, &ic, 1);
	CHECK(ia == 7.0f);
	CHECK(ib == -4.5f);
	CHECK(ic == -2.5f);

	ia = 7.0f; ib = -1.5f; ic = 9.0f;
	pcl_reconstruct(&ia, &ib, &ic, 2);
	CHECK(ic == -5.5f);

	/* PCL_NONE（和任何非法下标）不许动数据 */
	ia = 7.0f; ib = -1.5f; ic = 9.0f;
	pcl_reconstruct(&ia, &ib, &ic, PCL_NONE);
	pcl_reconstruct(&ia, &ib, &ic, 7);
	CHECK(ia == 7.0f);
	CHECK(ib == -1.5f);
	CHECK(ic == 9.0f);

	/* 任意（非精确）输入经过重建后三相之和必须为零 —— 这正是 dq0 的前提 */
	{
		float a = 1.2345f, b = -6.789f, c = 5.5545f;
		pcl_reconstruct(&a, &b, &c, 2);
		CHECK(fabsf(a + b + c) < 1e-5f);
	}
}

static void test_hw_scenario(void)
{
	/* 本机 DTC_MAX = 0.9、满调制时的一拍：control_svm 的 min-max 注入使三相 duty
	 * 形如 {0.883, 0.45, 0.017}。INVERT_DTC = 1 ⇒ 窗口 = AR*dtc，所以 **duty
	 * 最小的那一相** 只剩 51 tick ≈ 0.2 us（低边几乎不导通），必须由另外两相重建 */
	uint32_t w0 = pcl_low_side_ticks(AR, ccr_of(0.883f));
	uint32_t w1 = pcl_low_side_ticks(AR, ccr_of(0.450f));
	uint32_t w2 = pcl_low_side_ticks(AR, ccr_of(0.017f));
	CHECK(w2 == 51u);
	CHECK(w0 > MIN_WIN);
	CHECK(pcl_worst_phase(w0, w1, w2, MIN_WIN) == 2);

	/* 边界：窗口正好等于阈值（duty = 0.15）时不重建，再低一点就重建 */
	w2 = pcl_low_side_ticks(AR, ccr_of(0.150f));
	CHECK(w2 == MIN_WIN);
	CHECK(pcl_worst_phase(w0, w1, w2, MIN_WIN) == PCL_NONE);
	w2 = pcl_low_side_ticks(AR, ccr_of(0.140f));
	CHECK(pcl_worst_phase(w0, w1, w2, MIN_WIN) == 2);

	/* 旧上限 DTC_MAX = 0.7 时最低那一相被 DTC_MIN 钳到 0（窗口 0 tick，低边完全
	 * 不导通）—— 说明这个坑在提上限之前就存在，判据把两者都覆盖了 */
	w0 = pcl_low_side_ticks(AR, ccr_of(0.700f));
	w1 = pcl_low_side_ticks(AR, ccr_of(0.350f));
	w2 = pcl_low_side_ticks(AR, ccr_of(0.000f));
	CHECK(pcl_worst_phase(w0, w1, w2, MIN_WIN) == 2);

	/* 零电压/轻载：三相几乎相同，都不重建 */
	w0 = pcl_low_side_ticks(AR, ccr_of(0.450f));
	CHECK(pcl_worst_phase(w0, w1, w0, MIN_WIN) == PCL_NONE);
}

int main(void)
{
	test_low_side_ticks();
	test_pick();
	test_reconstruct();
	test_hw_scenario();

	if (failures) {
		printf("test_phase_current: %d failure(s)\n", failures);
		return 1;
	}
	printf("test_phase_current: OK\n");
	return 0;
}
