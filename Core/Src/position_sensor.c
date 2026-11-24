/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "motion_law.h"
#include "hw_config.h"
#include "user_config.h"
#include "timing.h"
#include "dual_gear.h"
#include "structs.h"

uint16_t spi_tx_words[2], spi_rx_words[2];

/* Poll iterations the DMA waits spin before giving up.  An iteration costs about
 * 20 cycles (the flag read is a function call), so this is ~2000 cycles: about 4x
 * the 521-cycle frame, so it cannot fire on a healthy encoder that happens to be
 * harvested a little early, yet far below the 6000-cycle (25 us) slot, so a dead
 * encoder - or a misconfigured DMA - cannot stall the interrupt or the boot,
 * which is what it used to do (infinite spin -> watchdog reset loop) */
#define PS_WAIT_ITERS 100

/* ---- 热路径上的寄存器直读/直写 -------------------------------------------
 * 编码器采样在 40 kHz 中断里，每拍要碰 5 次 SDK 辅助函数（DMA_GetFlagStatus /
 * DMA_ClearFlag / GPIO_WriteBits ×2）；它们都只是读一个寄存器或写一个掩码。
 * 这里展开成同样的寄存器操作（CS 由 GPIOx->PBC/PBSC 置位/复位，TC 标志读
 * INTSTS / 写 INTCLR），语义与 SDK 完全相同。
 * 注意：这一段的收益很小（几十个周期）；ps_finish_sample() 那段实测的 −32%
 * 主要来自局部变量化和 count_buff 环形化（见那里的注释）。 */
static inline int ps_tc1_set(void){ return (DMA2->INTSTS & DMA_FLAG_TC1) != 0u; }
static inline int ps_tc2_set(void){ return (DMA2->INTSTS & DMA_FLAG_TC2) != 0u; }
static inline void ps_clear_tc(void){ DMA2->INTCLR = DMA_FLAG_TC1 | DMA_FLAG_TC2; }
static inline void ps_cs_low(void){ ENC_SPI_NSS_GPIO_Port->PBC = ENC_SPI_NSS_Pin; }
static inline void ps_cs_high(void){ ENC_SPI_NSS_GPIO_Port->PBSC = ENC_SPI_NSS_Pin; }

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	spi_tx_words[0] = ENC_READ_WORD;
	spi_tx_words[1] = 0x0000;
	for(int i = 0; i<n; i++){
		/* The DMA counter counts data units, and the channels are configured for
		 * 16-bit (HALFWORD) transfers, so one 16-bit encoder frame is 1 - not 2 */
		ps_clear_tc();
		ps_cs_low(); // CS low
		SPI_Transmit_DMA((uint8_t *)spi_tx_words, 1);
		SPI_Receive_DMA((uint8_t *)spi_rx_words, 1);
		/* Bounded like ps_finish_sample: if the flags never arrive (DMA/SPI
		 * misconfigured, see MX_SPI1_Init) this used to spin forever right here,
		 * before the FSM ever starts - and with DBG_IWDG_STOP set even the
		 * watchdog can't rescue it under the debugger */
		uint32_t wait = PS_WAIT_ITERS;
		while(!ps_tc2_set() && (--wait != 0)){;}
		while(!ps_tc1_set() && (--wait != 0)){;}
		ps_cs_high(); // CS high
		SPI_StopTransmit_DMA();
		SPI_StopReceive_DMA();
		if(wait == 0){encoder->lost_frames++;}	// let telemetry show that the encoder never answered during warmup
	}
}

/* The encoder read is split in two so that the SPI transfer is not dead time.
 * A frame is a fixed 16 bits at 7.5 MHz - 2.13 us, 521 cycles measured on the
 * wire - and no CPU-side trick makes it shorter (10 MHz is the encoder's limit
 * and this SPI can only divide its 120 MHz APB2 clock by powers of two, so
 * 7.5 MHz is as fast as it goes).  What can be changed is whether the CPU waits
 * for it: the commutation interrupt starts the frame on entry, does the work
 * that does not need it (the ADC read and the fault check), and only then
 * collects the result.  The two halves are ~500 cycles apart in the ISR, and
 * the frame needs 521, so the wait is normally already over.
 *
 * ps_sample() is the two halves back to back, for a caller that wants the value
 * in the same breath. */

/* Poll iterations ps_finish_sample() waits before calling the frame lost (the
 * value and its rationale live with the definition above) */

void ps_start_read(void){
	/* The transfer-complete flags have to be cleared before the channels are
	 * enabled, or the previous frame's flag would read as this one completing */
	ps_clear_tc();
	ps_cs_low(); // CS low
	SPI_Transmit_DMA((uint8_t *)spi_tx_words, 1);
	SPI_Receive_DMA((uint8_t *)spi_rx_words, 1);
}

int ps_finish_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt.  Returns 0 if the frame never arrived, in which
	 * case nothing is updated and the previous sample is held */

	/* Only the RX flag is waited on: its DMA request comes from the last received
	 * bit, which cannot happen before the TX half has gone out */
	ISR_TIMING_SPAN_BEGIN(t_enc_wait);
	uint32_t wait = PS_WAIT_ITERS;
	while(!ps_tc1_set() && (--wait != 0)){;}
	ISR_TIMING_SPAN_END(ISR_EXT_ENC_WAIT, t_enc_wait);

	ps_cs_high(); // CS high
	SPI_StopTransmit_DMA();
	SPI_StopReceive_DMA();

	if(wait == 0){
		/* No frame came back.  Hold the last good sample (raw/count/angle/
		 * velocity are all left alone) instead of feeding the control loop a stale
		 * register read, and count it so a broken encoder shows up instead of
		 * silently freezing the position.  Reading the status then the data
		 * register clears RXNE/overrun, so a late frame cannot also swallow the
		 * next one */
		encoder->lost_frames++;
		(void)SPI1->STS;
		(void)SPI1->DAT;
		return 0;
	}
	/* 整段用局部变量跑完再写回：40 kHz 下每拍这一小段要做三十多次 SRAM 读写，
	 * 而 cortex-m4 每条 LDR/STR 都要走一次总线。数值与逐字段版本完全相同：
	 * 同样的运算顺序，且 ENC_CPR 是 2 的幂，乘 INV_CPR 与除 (float)ENC_CPR
	 * 逐位等价（见 docs/DEV.md "中断预算"）。 */
	const int raw = spi_rx_words[0];
	encoder->raw = raw;

	encoder->old_angle = encoder->angle_singleturn;

	/* Linearization: 128-entry table, linear interpolation inside one bin */
	const int idx    = raw >> 9;							// 0..127
	const int off_1  = encoder->offset_lut[idx];			// lookup table lower entry
	const int off_2  = encoder->offset_lut[(idx + 1) & 127];	// 128 is a power of two
	const int off_interp = off_1 + ((off_2 - off_1)*(raw & 511) >> 9);
	const int count = raw + off_interp;
	encoder->count = count;

	/* Single-turn count relative to the zero position, wrapped to [0, ENC_CPR):
	 * the mask is the same positive residue that "% + if(<0)" produced */
	const int count_wrapped = (count - M_ZERO) & (ENC_CPR - 1);
	encoder->count_wrapped = count_wrapped;
	const float angle_singleturn = TWO_PI_F*(float)count_wrapped*INV_CPR;
	encoder->angle_singleturn = angle_singleturn;

	const float ppairs = encoder->ppairs;
	float elec_angle = ppairs*(float)(count - E_ZERO)*INV_CPR;
	const int int_angle = (int)elec_angle;
	elec_angle = TWO_PI_F*(elec_angle - (float)int_angle);
	if(elec_angle < 0.0f){elec_angle += TWO_PI_F;}	// Add 2*pi to negative numbers
	encoder->elec_angle = elec_angle;

	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	int first_sample = !encoder->first_sample;
	if(first_sample){
		/* On the first sample, pick the turn count so the multi-turn angle starts
		 * within (-3*pi/2, pi/2] of the zero position, like the old mbed firmware.
		 * angle_singleturn is in [0, 2*pi) here, so it can never be negative */
		encoder->turns = 0;
		if(encoder->angle_singleturn > PI_OVER_2_F){encoder->turns = -1;}
		encoder->first_sample = 1;
	}
	/* 齿轮式双编上电恢复（ps_dual_gear_restore）：整圈数由副编码器解出来，覆盖
	 * 上面 first_sample 给的 0/-1。速度历史一并填满，否则恢复那一刻会出现一个
	 * 跟圈数差成正比的假速度尖峰 */
	if(encoder->abs_turns_valid){
		encoder->turns = encoder->abs_turns;
		encoder->abs_turns_valid = 0;
		encoder->first_sample = 1;
		first_sample = 1;
	}

	/* Multi-turn position, exact in integer counts (good for +/-32768 turns
	 * before overflowing), and in radians for the rest of the firmware.
	 * count_buff 改成环形：新样本写 head，19 拍的历史只花一次写（原来的
	 * 19 字搬移约等于这个函数 15% 的访存）。count_multi 就是原来
	 * count_buff[0] 的值，所以 cogging.c 照旧按它对表。 */
	const int count_multi = count_wrapped + ENC_CPR*encoder->turns;
	encoder->count_multi = count_multi;
	encoder->angle_multiturn = TWO_PI_F*(float)count_multi*INV_CPR;

	uint8_t head = encoder->count_head;
	if(++head >= N_POS_SAMPLES){head = 0;}
	encoder->count_head = head;
	encoder->count_buff[head] = count_multi;
	if(first_sample){	// fill the buffer on the first sample so velocity starts at zero, not with a spike
		for(int i = 0; i<N_POS_SAMPLES; i++){encoder->count_buff[i] = count_multi;}
	}
	uint8_t oldest = head + 1;			// head 的前一格 = N-1 拍之前的样本
	if(oldest >= N_POS_SAMPLES){oldest = 0;}

	/* Velocity: finite difference of the integer counts.  Differencing counts
	 * instead of the float angles keeps velocity resolution independent of how
	 * far the multi-turn position is from zero */
	/* (Tried a moving least-squares fit here; it wasn't any better than the simple difference) */
	encoder->velocity_diff = TWO_PI_F*((float)(count_multi - encoder->count_buff[oldest]))
			/((float)ENC_CPR*dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = ppairs*encoder->velocity;

	return 1;
}

/* ---------------------------------------------------------------------------
 * 自适应窗口最小二乘测速。每 25 us 调用一次（只在编码器帧有效时）。
 *
 * 每 VEL_BLK_DECIM 拍把"单圈计数的增量"累加成一个块（块内求和压掉 √10 的
 * 量化噪声），然后对最近 m 块做直线拟合，斜率就是速度。m 由运动速度决定
 * （ml_ls_window：把斜率噪声压到 config.vel_est_noise 的相对目标），块数通过
 * 两个滑动和增量维护，所以**每块只花 O(1)**，不需要每块重算 m 次乘加：
 *
 *   A   = Σ y[j]                （j = 0 最新）
 *   B   = Σ j·y[j]
 *   滑窗：B ← B + A_old − m·y_out,  A ← A_old + y_in − y_out
 *   加长：A += y[m],   B += m·y[m]           （新样本的年龄是 m）
 *   缩短：A −= y[m−1], B −= (m−1)·y[m−1]
 *   斜率 = ((m−1)/2·A − B) / (dt·m·(m²−1)/12)
 *
 * 窗口长度变化时只改一两块（VEL_M_STEP），避免 m 阶重算落到一个中断周期里。
 * ------------------------------------------------------------------------- */
#define VEL_M_STEP		8			/* 每块最多增减的窗口块数 */
/* 块平均后的计数噪声：原始计数 σ≈4.3 count（静止实测），块内 10 个样本求和
 * 平均 → 4.3/√10 ≈ 1.36。写成常数避免在中断里调 sqrtf */
#define VEL_BLK_SIGMA	1.36f

static void vel_push(EncoderStruct *encoder, int32_t y)
{
	if(++encoder->vel_head >= VEL_BLK_MAX){ encoder->vel_head = 0; }
	encoder->vel_blk[encoder->vel_head] = y;
	if(encoder->vel_fill < VEL_BLK_MAX){ encoder->vel_fill++; }
}

/* 环形缓冲里"年龄 = age"的样本（0 = 最新） */
static int32_t vel_at(const EncoderStruct *encoder, int age)
{
	int idx = (int)encoder->vel_head - age;
	if(idx < 0){ idx += VEL_BLK_MAX; }
	return encoder->vel_blk[idx];
}

/* 一套窗口滑一拍 + 朝目标长度调整（两套窗口共用同一个环形缓冲） */
static void vel_fit_step(const EncoderStruct *encoder, ml_ls_t *s, int32_t y, int m_t)
{
	/* 1) 窗口滑一拍。窗口还没盖满历史时不移出样本，而是顺势长一块。
	 * ⚠ m<2 时 m<fill 不代表"有样本移出窗口"（那会把唯一的老样本丢掉），
	 * 必须先按"0 → 1 → 2"补齐，所以这个分支要放在最前面 */
	if(s->m < 2){
		ml_ls_slide(s, y, 0);
		s->m++;
	}
	else if(s->m < (int)encoder->vel_fill){
		ml_ls_slide(s, y, vel_at(encoder, s->m));	/* 年龄 m 的样本移出窗口 */
	}
	else{
		ml_ls_slide(s, y, 0);
		if(s->m < (VEL_BLK_MAX - 1)){ s->m++; }		/* 还没盖满：窗口长一块 */
	}

	/* 2) 窗口长度朝目标调整，每块最多 VEL_M_STEP 块；不能超过已有历史 */
	int step = m_t - s->m;
	if(step > VEL_M_STEP){ step = VEL_M_STEP; }
	if(step < -VEL_M_STEP){ step = -VEL_M_STEP; }
	while((step > 0) && (s->m < (int)encoder->vel_fill) && (s->m < (VEL_BLK_MAX - 1))){
		ml_ls_grow(s, vel_at(encoder, s->m));		/* 年龄 m：窗口外的下一块 */
		step--;
	}
	while((step < 0) && (s->m > 2)){
		ml_ls_shrink(s, vel_at(encoder, s->m - 1));	/* 去掉最老的一块 */
		step++;
	}
}

void ps_update_velocity(EncoderStruct *encoder, float dt)
{
	const int cpr = ENC_CPR;

	int dw = (int)encoder->count_wrapped - (int)encoder->vel_prev_w;
	if(dw > (cpr/2)){ dw -= cpr; }
	else if(dw < -(cpr/2)){ dw += cpr; }
	encoder->vel_prev_w = encoder->count_wrapped;
	encoder->vel_acc += dw;

	if(++encoder->vel_blk_cnt < VEL_BLK_DECIM){ return; }
	encoder->vel_blk_cnt = 0;

	const int32_t y = encoder->vel_acc;
	encoder->vel_acc = 0;
	vel_push(encoder, y);

	const float dt_blk = (float)VEL_BLK_DECIM*dt;

	if(encoder->vel_fill < 2u){
		ml_ls_reset(&encoder->vel_fit);
		ml_ls_reset(&encoder->vel_fit_fast);
		encoder->vel_ls_counts = 0.0f;
		encoder->vel_fast_counts = 0.0f;
		encoder->velocity = 0.0f;
		encoder->velocity_fast = 0.0f;
		encoder->elec_velocity = 0.0f;
		encoder->vel_lag = ml_ls_lag(VEL_BLK_MAX, dt_blk);
		encoder->vel_fast_lag = encoder->vel_lag;
		return;
	}

	/* 窗口选择用的"速度提示"：测量值本身 + **指令值**。
	 * 只用测量值会掉进陷阱：静止时窗口取最长（滞后最大）→ 增益被压到最低 →
	 * 加上一个速度指令也推不动 → 测量值一直是 0，窗口永远缩不回来。
	 * 指令值（输出轴 rad/s）换算成电机端 counts/s 后取较大者即可破环。 */
	float hint = fabsf(encoder->vel_ls_counts);
	{
		const float gr = (GR > 0.0f) ? GR : 1.0f;
		const float v_cmd = fabsf(controller.v_setpoint)*gr*((float)cpr/TWO_PI_F);
		if((v_cmd == v_cmd) && (v_cmd > hint)){ hint = v_cmd; }
	}

	/* 遥测/测量窗口：噪声目标严（config.vel_est_noise），滞后大也无所谓 */
	int m_precise = ml_ls_window(hint, dt_blk, VEL_BLK_SIGMA, VEL_EST_NOISE, VEL_BLK_MAX - 1);
	/* 速度环窗口：噪声目标宽（config.vel_ctrl_noise），换更小的滞后 */
	int m_fast = ml_ls_window(hint, dt_blk, VEL_BLK_SIGMA, VEL_CTRL_NOISE, VEL_BLK_MAX - 1);
	if(m_precise > (int)encoder->vel_fill){ m_precise = (int)encoder->vel_fill; }
	if(m_fast > (int)encoder->vel_fill){ m_fast = (int)encoder->vel_fill; }
	if(m_precise < 2){ m_precise = 2; }
	if(m_fast < 2){ m_fast = 2; }

	vel_fit_step(encoder, &encoder->vel_fit, y, m_precise);
	vel_fit_step(encoder, &encoder->vel_fit_fast, y, m_fast);

	encoder->vel_ls_counts = ml_ls_slope(&encoder->vel_fit, dt_blk);
	encoder->vel_fast_counts = ml_ls_slope(&encoder->vel_fit_fast, dt_blk);
	const float rad_per_count = TWO_PI_F/(float)cpr;
	encoder->velocity = encoder->vel_ls_counts*rad_per_count;
	encoder->velocity_fast = encoder->vel_fast_counts*rad_per_count;
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;
	encoder->vel_lag = ml_ls_lag(encoder->vel_fit.m, dt_blk);
	encoder->vel_fast_lag = ml_ls_lag(encoder->vel_fit_fast.m, dt_blk);
}

void ps_update_pll(EncoderStruct * encoder, float dt){
	/* 自适应窗口最小二乘测速：encoder->velocity / elec_velocity 由它更新。
	 * 速度环、位置环与遥测都用这条（低噪声）；MIT 阻尼走 velocity_diff（要短滞后），
	 * PLL 只给齿槽标定内环。 */
	ps_update_velocity(encoder, dt);

	/* 编码器速度 PLL（临界阻尼，见 motion_law.h）。只在本周期的帧有效时调用
	 * —— 丢帧还继续预测会让估计值一直漂。 */
	float kp, ki;
	ml_pll_gains(PLL_BW, dt, &kp, &ki);

	if(!encoder->pll_seeded){
		/* 第一个有效样本直接对齐。不能让它从 0（或上一次的旧计数）追上来：
		 * 半圈的相位差会经 ki 变成几十 rad/s 的速度尖峰，再进电流环就是一
		 * 下冲车 */
		encoder->pll_pos = (float)encoder->count_wrapped;
		encoder->pll_vel_counts = 0.0f;
		encoder->pll_seeded = 1;
	}

	ml_pll_step(&encoder->pll_pos, &encoder->pll_vel_counts,
	            (float)encoder->count_wrapped, (float)ENC_CPR, kp, ki, dt);
	encoder->pll_velocity = encoder->pll_vel_counts * (TWO_PI_F/((float)ENC_CPR));
}

void ps_reset_pll(EncoderStruct * encoder){
	/* 进电机模式时重新播种（下一帧有效样本直接对齐），速度先清零 */
	encoder->pll_seeded = 0;
	encoder->pll_vel_counts = 0.0f;
	encoder->pll_velocity = 0.0f;

	/* 最小二乘测速同样清空：窗口里的旧计数属于上一个运行段，带过来会在
	 * 头几块里算出一个假速度 */
	encoder->vel_prev_w = encoder->count_wrapped;
	encoder->vel_acc = 0;
	encoder->vel_blk_cnt = 0;
	encoder->vel_fill = 0;
	encoder->vel_head = 0;
	ml_ls_reset(&encoder->vel_fit);
	ml_ls_reset(&encoder->vel_fit_fast);
	encoder->vel_ls_counts = 0.0f;
	encoder->vel_fast_counts = 0.0f;
	encoder->vel_lag = ml_ls_lag(VEL_BLK_MAX, (float)VEL_BLK_DECIM*DT);
	encoder->vel_fast_lag = encoder->vel_lag;
	encoder->velocity = 0.0f;
	encoder->velocity_fast = 0.0f;
	encoder->elec_velocity = 0.0f;
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* Both halves back to back, for a caller that wants the value in the same
	 * breath.  The commutation interrupt calls them apart, so the transfer runs
	 * while the ADC is read and the faults are checked - see the note above */
	ps_start_read();
	ps_finish_sample(encoder, dt);
}

void ps_print(EncoderStruct * encoder, int dt_ms){
	myprintf("Raw: %d", encoder->raw);
	myprintf("   Linearized Count: %d", encoder->count);
	myprintf("   Single Turn: %f", encoder->angle_singleturn);
	myprintf("   Multiturn: %f", encoder->angle_multiturn);
	myprintf("   Electrical: %f", encoder->elec_angle);
	myprintf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}

/* ===========================================================================
 * 齿轮式双编：断电记忆减速机输出轴位置
 *
 * 副编码器装在一个与转子齿轮啮合、齿数不同的齿轮上（齿数由 config.gear_* 给），
 * 两路单圈绝对值合起来能解出转子转到了第几圈 —— 数学在 dual_gear.h，
 * 这里只负责 SPI 读数、配置读写和把结果接到多圈计数上。
 *
 * 为什么能在不转电机的情况下标定：韦尼尔相位 Δ 里含一个安装常数，单次读数解不出
 * 绝对圈数，所以标定时把"当前这一刻"整体定义成参考（相位 + 当时的多圈计数），
 * 上电时算的是相对参考的圈数差。参考本身即零点，常数项自动消掉。
 * =========================================================================== */

static DualGearParams ps_dg_params(void){
	DualGearParams p;
	p.t_first = GEAR_TEETH_FIRST;
	p.t_sec = GEAR_TEETH_SEC;
	p.first_cpr = ENC_CPR;
	p.sec_cpr = SEC_ENC_CPR;
	p.reversed = GEAR_SEC_REVERSED ? 1 : 0;
	return p;
}

/* 副编码器单帧读取。调用者必须已经挡住控制中断（见 ps_sec_read） */
static int ps_sec_read_locked(uint16_t * raw_out){
	uint16_t tx = SEC_ENC_READ_WORD;
	uint16_t rx = 0xFFFF;
	DMA_ClearFlag(DMA_FLAG_TC1, DMA2);
	DMA_ClearFlag(DMA_FLAG_TC2, DMA2);
	GPIO_WriteBits(SEC_ENC_CS, Bit_RESET); // CS low
	SPI_Transmit_DMA((uint8_t *)&tx, 1);
	SPI_Receive_DMA((uint8_t *)&rx, 1);
	uint32_t wait = PS_WAIT_ITERS;
	while((DMA_GetFlagStatus(DMA_FLAG_TC1, DMA2) == RESET) && (--wait != 0)){;}
	GPIO_WriteBits(SEC_ENC_CS, Bit_SET); // CS high
	SPI_StopTransmit_DMA();
	SPI_StopReceive_DMA();
	if(wait == 0){
		/* 和主编码器一样有界等待：副编码器没插/坏了不能把启动卡死。
		 * 读状态再读数据清掉 RXNE/OVR，迟到的帧不会吞掉下一帧 */
		(void)SPI1->STS;
		(void)SPI1->DAT;
		return 0;
	}
	/* 0x0000 / 0xFFFF 视为"没有芯片应答"：没贴片时 MISO 被上/下拉就是这个值，
	 * 而真实读数正好落在 0/65535 的概率只有 1/65536。没有这条保护，一块没装
	 * 第二颗编码器的板子会拿一个固定值算出"相位"，然后一本正经地在两次上电之间
	 * 记错位置。台架上用 telemetry.sec_enc_raw 确认读数随转子变化 */
	if(rx == 0x0000 || rx == 0xFFFF){
		return 0;
	}
	*raw_out = rx;
	return 1;
}

int ps_sec_read(uint16_t * raw_out){
	int ok;
	/* 副编码器和主编码器共用 SPI1 + DMA2（控制中断里每 25 us 用一次），
	 * 所以读之前必须挡住控制中断；只关 ATIM1，USB/串口照常响应。
	 * 一帧 2.13 us + 有界等待，调用点都在门极关闭时（上电/标定/诊断） */
	NVIC_DisableIRQ(PWM_ISR);
	ok = ps_sec_read_locked(raw_out);
	NVIC_EnableIRQ(PWM_ISR);
	if(!ok){sec_encoder.lost_frames++;}
	return ok;
}

int ps_sec_probe(void){
	/* 副编码器是否存在：连读两帧，两帧都拿到有效值才算"在"。
	 * 没贴片/没接线时 MISO 是固定电平（被 ps_sec_read_locked 判成无应答），
	 * 少一次读也不会误判成"在线" */
	uint16_t a = 0, b = 0;
	int ok = ps_sec_read(&a) && ps_sec_read(&b);
	sec_encoder.present = ok ? 1u : 0u;
	if(ok){sec_encoder.raw = (int)b;}
	return (int)sec_encoder.present;
}

int ps_dual_gear_config_ok(void){
	DualGearParams p = ps_dg_params();
	return dg_params_ok(&p);
}

/* 同一时刻的读数对：屏蔽一次控制中断，把副编码器帧和主编码器的当前单圈值一起取走。
 * 门极关闭时转子静止，主编码器"最多陈旧 25 us"不影响结果 */
static int ps_dg_snapshot(EncoderStruct * encoder, uint16_t * sec_raw, int * first_raw, int * wrapped){
	int ok;
	NVIC_DisableIRQ(PWM_ISR);
	ok = ps_sec_read_locked(sec_raw);
	*first_raw = encoder->raw;
	*wrapped = encoder->count_wrapped;
	NVIC_EnableIRQ(PWM_ISR);
	if(!ok){sec_encoder.lost_frames++;}
	return ok;
}

int ps_dual_gear_read(EncoderStruct * encoder){
	DualGearParams p = ps_dg_params();
	uint16_t sec_raw = 0;
	int first_raw = 0, wrapped = 0;
	int32_t turns;

	if(!dg_params_ok(&p)){return -1;}
	if(!GEAR_REF_VALID){return -2;}
	if(!ps_dg_snapshot(encoder, &sec_raw, &first_raw, &wrapped)){sec_encoder.present = 0; return -3;}
	sec_encoder.present = 1;

	sec_encoder.raw = (int)sec_raw;
	sec_encoder.phase = dg_phase(first_raw, (int32_t)sec_raw, &p);
	turns = dg_restore_turns(&p, first_raw, (int32_t)sec_raw,
	                         GEAR_ZERO_PHASE, GEAR_ZERO_COUNT, wrapped, NULL);
	sec_encoder.abs_turns = (float)turns + (float)wrapped / (float)ENC_CPR;
	return 0;
}

int ps_dual_gear_capture(EncoderStruct * encoder){
	DualGearParams p = ps_dg_params();
	uint16_t sec_raw = 0;
	int first_raw = 0, wrapped = 0;

	if(!dg_params_ok(&p)){return -1;}
	/* GEAR_ZERO_COUNT 是 turns*ENC_CPR + count_wrapped，要放进 int32 —— 多圈计数
	 * 本身只支持 ±32768 圈，超出范围说明运行状态不正常，宁可拒绝也不要溢出 */
	if((encoder->turns > 30000) || (encoder->turns < -30000)){return -1;}
	if(!ps_dg_snapshot(encoder, &sec_raw, &first_raw, &wrapped)){sec_encoder.present = 0; return -3;}
	sec_encoder.present = 1;

	sec_encoder.raw = (int)sec_raw;
	sec_encoder.phase = dg_phase(first_raw, (int32_t)sec_raw, &p);
	/* 参考 = 当前这一刻：相位 + 当时的 turns*CPR + count_wrapped。
	 * 之后上电只要解出相对这一刻的圈数差就能接上（见 dg_restore_turns） */
	GEAR_ZERO_PHASE = sec_encoder.phase;
	GEAR_ZERO_COUNT = (int32_t)encoder->turns * ENC_CPR + encoder->count_wrapped;
	GEAR_REF_VALID = 1;
	sec_encoder.abs_turns = (float)encoder->turns + (float)encoder->count_wrapped / (float)ENC_CPR;
	return 0;
}

int ps_dual_gear_restore(EncoderStruct * encoder){
	DualGearParams p;
	uint16_t sec_raw = 0;
	float abs_turns = 0.0f;
	int32_t turns;

	if(!GEAR_REF_VALID){return 1;}			/* 没标定过：保持原行为（上电位置从 0 附近开始） */
	p = ps_dg_params();
	if(!dg_params_ok(&p)){return -1;}
	/* 参考计数是 turns*ENC_CPR + count_wrapped。多圈计数本身只支持 ±32768 圈，
	 * 所以超出这个范围说明这个参数被手改坏了（或 flash 里是别的格式）——
	 * 宁可当作没有参考，也不要拿它去写一个溢出的 turns */
	if((GEAR_ZERO_COUNT > 30000 * (int32_t)ENC_CPR) || (GEAR_ZERO_COUNT < -30000 * (int32_t)ENC_CPR)){return -1;}

	/* 只在 main() 里、控制中断使能之前调用（那时 SPI/DMA 没人和你抢）。
	 * 主编码器至少要有一帧有效数据，count_wrapped 才有意义 */
	if(!encoder->first_sample){ps_sample(encoder, DT);}
	if(!ps_sec_read(&sec_raw)){sec_encoder.present = 0; return -3;}
	sec_encoder.present = 1;

	sec_encoder.raw = (int)sec_raw;
	sec_encoder.phase = dg_phase(encoder->raw, (int32_t)sec_raw, &p);
	turns = dg_restore_turns(&p, encoder->raw, (int32_t)sec_raw,
	                         GEAR_ZERO_PHASE, GEAR_ZERO_COUNT, encoder->count_wrapped, &abs_turns);

	/* 不直接改 turns：下一帧的 first_sample 逻辑会把它清零。交给 ps_finish_sample
	 * 在算完本帧角度之后覆盖，并把速度历史一次填满（无尖峰） */
	encoder->abs_turns = (int)turns;
	encoder->abs_turns_valid = 1;
	sec_encoder.abs_turns = (float)turns + (float)encoder->count_wrapped / (float)ENC_CPR;
	return 0;
}

void ps_set_m_zero(int32_t v){
	/* 机械零点同时定义两件事：主编码器单圈计数（count_wrapped）的 0 点，以及双编
	 * 参考里 count_wrapped 的坐标系。action.zero 的语义是"当前位置就是 0"，所以
	 * 除了换 M_ZERO，还得把 turns（自由计数的整圈数）一起归零 —— 否则清零后的
	 * 读数会留在整圈上（台架：在 +1.43 转处清零读到 +1.00 转），用户没法判断是
	 * "清零没生效"还是"位置本来就那样"。
	 *
	 * 新坐标下当前位置 = (count - v) mod CPR，旧坐标下是 turns*CPR + count_wrapped，
	 * 两者之差就是双编参考要平移的量（参考本身就是这个坐标系里的绝对位置）。
	 * 整段挡住控制中断：ISR 每 25 us 读这些变量，不能让它看到半更新的状态；同时
	 * 把 old_angle 设成新角度，免得下一帧的 rollover 检测把这次跳变当成转子转了
	 * 一整圈（那又会差一圈）。坐标换算是纯算术，全程不碰 SPI。 */
	int32_t turns, old_counts, new_counts;
	int32_t count_now;
	float angle;

	NVIC_DisableIRQ(PWM_ISR);
	turns = comm_encoder.turns;
	count_now = comm_encoder.count;
	old_counts = turns*(int32_t)ENC_CPR + comm_encoder.count_wrapped;
	new_counts = dg_mod(count_now - v, (int32_t)ENC_CPR);
	angle = TWO_PI_F*((float)new_counts)/((float)ENC_CPR);

	M_ZERO = v;
	comm_encoder.m_zero = v;
	comm_encoder.turns = 0;
	comm_encoder.count_wrapped = new_counts;
	comm_encoder.angle_singleturn = angle;
	comm_encoder.angle_multiturn = angle;
	comm_encoder.old_angle = angle;
	for(int i = 0; i < N_POS_SAMPLES; i++){comm_encoder.count_buff[i] = new_counts;}
	comm_encoder.count_head = 0;
	comm_encoder.count_multi = new_counts;
	ps_reset_pll(&comm_encoder);		/* 速度别从这次跳变里算出来 */

	if(turns > 30000 || turns < -30000){
		GEAR_REF_VALID = 0;		/* 多圈计数装不下这个位移，参考作废（重新 capture） */
		GEAR_ZERO_COUNT = 0;
	}else{
		GEAR_ZERO_COUNT -= old_counts - new_counts;
	}
	NVIC_EnableIRQ(PWM_ISR);

	/* 这里刻意**不**顺手重取双编参考（`ps_dual_gear_capture()`）：参考是"相位 + 多圈
	 * 计数"成对的东西，清零这一刻转子未必静止（台架上滑行/回差都可能），在这种时刻
	 * 采一次参考有过整整差一圈的记录。参考重取是独立动作，要求转子静止、门极关闭，
	 * 由 action.gear_capture 显式触发；代价是清零后记忆窗口不再以零点为中心
	 * （窗口中心 = gear_zero_count/CPR），需要居中时再跑一次 capture。 */
}

