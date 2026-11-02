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

uint16_t spi_tx_words[2], spi_rx_words[2];

/* Poll iterations the DMA waits spin before giving up.  An iteration costs about
 * 20 cycles (the flag read is a function call), so this is ~2000 cycles: about 4x
 * the 521-cycle frame, so it cannot fire on a healthy encoder that happens to be
 * harvested a little early, yet far below the 6000-cycle (25 us) slot, so a dead
 * encoder - or a misconfigured DMA - cannot stall the interrupt or the boot,
 * which is what it used to do (infinite spin -> watchdog reset loop) */
#define PS_WAIT_ITERS 100

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	spi_tx_words[0] = ENC_READ_WORD;
	spi_tx_words[1] = 0x0000;
	for(int i = 0; i<n; i++){
		/* The DMA counter counts data units, and the channels are configured for
		 * 16-bit (HALFWORD) transfers, so one 16-bit encoder frame is 1 - not 2 */
		DMA_ClearFlag(DMA_FLAG_TC1, DMA2);
		DMA_ClearFlag(DMA_FLAG_TC2, DMA2);
		GPIO_WriteBits(ENC_CS, Bit_RESET); // CS low
		SPI_Transmit_DMA((uint8_t *)spi_tx_words, 1);
		SPI_Receive_DMA((uint8_t *)spi_rx_words, 1);
		/* Bounded like ps_finish_sample: if the flags never arrive (DMA/SPI
		 * misconfigured, see MX_SPI1_Init) this used to spin forever right here,
		 * before the FSM ever starts - and with DBG_IWDG_STOP set even the
		 * watchdog can't rescue it under the debugger */
		uint32_t wait = PS_WAIT_ITERS;
		while((DMA_GetFlagStatus(DMA_FLAG_TC2, DMA2) == RESET) && (--wait != 0)){;}
		while((DMA_GetFlagStatus(DMA_FLAG_TC1, DMA2) == RESET) && (--wait != 0)){;}
		GPIO_WriteBits(ENC_CS, Bit_SET); // CS high
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
	DMA_ClearFlag(DMA_FLAG_TC1, DMA2);
	DMA_ClearFlag(DMA_FLAG_TC2, DMA2);
	GPIO_WriteBits(ENC_CS, Bit_RESET); // CS low
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
	while((DMA_GetFlagStatus(DMA_FLAG_TC1, DMA2) == RESET) && (--wait != 0)){;}
	ISR_TIMING_SPAN_END(ISR_EXT_ENC_WAIT, t_enc_wait);

	GPIO_WriteBits(ENC_CS, Bit_SET); // CS high
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
	encoder->raw = spi_rx_words[0];

	/* Shift around previous samples.  The buffer holds multi-turn positions in
	 * integer counts, so velocity resolution doesn't degrade as the float
	 * multi-turn angle gets large */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Single-turn count relative to the zero position, wrapped to [0, ENC_CPR) */
	int count_wrapped = (encoder->count - M_ZERO) % ENC_CPR;
	if(count_wrapped < 0){count_wrapped += ENC_CPR;}
	encoder->count_wrapped = count_wrapped;
	encoder->angle_singleturn = TWO_PI_F*((float)count_wrapped)/((float)ENC_CPR);

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers

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

	/* Multi-turn position, exact in integer counts (good for +/-32768 turns
	 * before overflowing), and in radians for the rest of the firmware */
	encoder->count_buff[0] = count_wrapped + ENC_CPR*encoder->turns;
	encoder->angle_multiturn = TWO_PI_F*((float)encoder->count_buff[0])/((float)ENC_CPR);
	if(first_sample){	// fill the buffer on the first sample so velocity starts at zero, not with a spike
		for(int i = 1; i<N_POS_SAMPLES; i++){encoder->count_buff[i] = encoder->count_buff[0];}
	}

	/* Velocity: finite difference of the integer counts.  Differencing counts
	 * instead of the float angles keeps velocity resolution independent of how
	 * far the multi-turn position is from zero */
	/* (Tried a moving least-squares fit here; it wasn't any better than the simple difference) */
	encoder->velocity = TWO_PI_F*((float)(encoder->count_buff[0] - encoder->count_buff[N_POS_SAMPLES-1]))
			/((float)ENC_CPR*dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

	return 1;
}

void ps_update_pll(EncoderStruct * encoder, float dt){
	/* 编码器速度 PLL（临界阻尼，见 motion_law.h）。速度环/位置环用它当反馈：
	 * 比下面那条 475 us 整数差分平滑，且带宽可调。只在本周期的帧有效时调用 ——
	 * 丢帧还继续预测会让估计值一直漂。MIT 与 legacy 遥测仍走 encoder->velocity。 */
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
