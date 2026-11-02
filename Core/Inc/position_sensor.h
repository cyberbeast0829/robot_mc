/*
 * position_sensor.h
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_


//#include "structs.h"
#include "spi.h"
#include <stdint.h>

#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128

typedef struct{
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	float angle_singleturn, old_angle, angle_multiturn, elec_angle, velocity, elec_velocity, ppairs;
	int raw, count, turns;
	int count_buff[N_POS_SAMPLES];		// multi-turn position history in integer counts, for the velocity calculation
	int count_wrapped;					// single-turn count in [0, ENC_CPR), input of the velocity PLL
	int m_zero, e_zero;
	int offset_lut[N_LUT];
	uint8_t first_sample;
	int lost_frames;		// frames that never came back (encoder unplugged/dead) - see ps_finish_sample()
	/* 速度 PLL（见 ps_update_pll）。只有速度环/位置环用它；MIT 与 legacy 遥测
	 * 仍然用上面 velocity 的整数差分，那条路径不经过这里 */
	float pll_pos;			// PLL 位置，单圈计数 [0, ENC_CPR)
	float pll_vel_counts;	// PLL 速度，counts/s
	float pll_velocity;		// PLL 速度，电机端 rad/s
	uint8_t pll_seeded;		// 0 = 还没用有效样本播种，下一帧直接对齐
} EncoderStruct;


void ps_warmup(EncoderStruct * encoder, int n);
void ps_start_read(void);
int ps_finish_sample(EncoderStruct * encoder, float dt);
void ps_sample(EncoderStruct * encoder, float dt);
void ps_update_pll(EncoderStruct * encoder, float dt);
void ps_reset_pll(EncoderStruct * encoder);
void ps_print(EncoderStruct * encoder, int dt_ms);

#endif /* INC_POSITION_SENSOR_H_ */
