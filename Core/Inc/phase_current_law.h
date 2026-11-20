/*
 * phase_current_law.h
 *
 * 三相低边分流电流采样：挑出"低边导通窗口太短"的那一相，用另外两相按
 * KCL 重建它的电流。纯数学（不引用任何固件头文件），host 可测
 * （tools/test_phase_current.c）。对应 ODrive board.cpp 的
 * fetch_and_reset_adcs()：三相窗口都够就用三个实测值，否则丢掉窗口最小的
 * 那一相，用另外两相补。
 *
 * 为什么需要：低边分流电阻只在低边 MOSFET 导通时才有相电流流过，而三路注入
 * ADC 的触发点固定在 PWM 峰值（中心对齐，CNT = AR），峰值处三相低边同时导通。
 * PWM1 + 高有效时 CHx 在 CNT < CCR 期间有效，低边是它的互补，所以一相的低边
 * 只在 CNT > CCR 的区段导通（`tim.c` 里"CNT=0 时三相低边全关"就是这个意思，
 * 采样点也因此被挪到峰值）。于是：
 *
 *     低边导通窗口 = AR - CCR   （"最近一次开关沿 CNT=CCR → 采样点"同样这么长）
 *
 * 窗口与 INVERT_DTC 无关 —— 极性只决定 CCR 怎么由 duty 算出来（set_dtc()），
 * 窗口本身只由 CCR 决定。本机 INVERT_DTC = 1 ⇒ CCR = AR*(1-dtc) ⇒ 窗口 ∝ dtc：
 * **duty 最小的那一相**（min-max 注入把它钉在母线正端）低边几乎不导通，它的
 * 分流读数不可信。⚠ 别照搬 ODrive 的"CCR 最小就丢"：他们用 PWM mode 2（与
 * PWM1 互补），结论正好反过来。
 *
 * 某一相窗口只剩零点几微秒时开关振铃还没衰完、ADC 采样保持也没建立，读数不可
 * 信 —— 而且误差会以"三相之和不为零"的形式灌进 dq0 电流环。
 *
 * 三相瞬时电流之和恒为零（星形接法、无中线），所以另外两相仍然可信时，丢掉
 * 的那一相可以精确补回来：i_x = -i_y - i_z。补回来的值与两路实测值同一采样
 * 时刻、同一量纲，不引入相位滞后。
 *
 * 只有一条 KCL 约束：如果两相窗口都不够（极端调制），也只能补一相，另一相仍
 * 用实测值 —— 补最差的那一相是收益最大的选择。
 */

#ifndef INC_PHASE_CURRENT_LAW_H_
#define INC_PHASE_CURRENT_LAW_H_

#include <stdint.h>

#define PCL_NONE	(-1)	/* 三相窗口都够，不需要重建 */

/* 一相的低边导通 tick 数（见上面：CNT > CCR 的那一段）。ar = TIM_PWM->AR，
 * ccr = TIM_PWM->CCDATx 的本拍值。set_dtc() 保证 CCR ∈ [0, AR]，所以不做钳位；
 * 万一 CCR 是垃圾值，窗口只会算得很大、判据不触发（退化成三相都用实测值），
 * 不会误重建。 */
static inline uint32_t pcl_low_side_ticks(uint32_t ar, uint32_t ccr)
{
	return ar - ccr;
}

/* 返回低边窗口最小的那一相（0 = a/leg1，1 = b/leg2，2 = c/leg3，即 CCR 最大的
 * 那一相）；三相窗口都 >= min_win 时返回 PCL_NONE。相等时取下标小的，结果确定。
 * 常见路径（三相都够）只有三次比较，选相只在真需要重建时才跑。 */
static inline int pcl_worst_phase(uint32_t win_a, uint32_t win_b, uint32_t win_c,
                                  uint32_t min_win)
{
	if((win_a >= min_win) && (win_b >= min_win) && (win_c >= min_win)){
		return PCL_NONE;
	}

	int idx = 0;
	uint32_t worst = win_a;
	if(win_b < worst){ worst = win_b; idx = 1; }
	if(win_c < worst){ idx = 2; }
	return idx;
}

/* 用另外两相重建 idx 那一相（idx = PCL_NONE 时不动）。 */
static inline void pcl_reconstruct(float *ia, float *ib, float *ic, int idx)
{
	switch(idx){
	case 0: *ia = -*ib - *ic; break;
	case 1: *ib = -*ia - *ic; break;
	case 2: *ic = -*ia - *ib; break;
	default: break;
	}
}

#endif /* INC_PHASE_CURRENT_LAW_H_ */
