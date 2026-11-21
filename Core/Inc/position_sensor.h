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
	int count_buff[N_POS_SAMPLES];		// multi-turn position history (ring, newest at count_head), for the velocity calculation
	int count_multi;					// newest multi-turn count (= the old count_buff[0]); cogging uses this as its table index
	uint8_t count_head;					// ring head: slot holding the newest sample
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
	/* 齿轮式双编上电恢复（见 ps_dual_gear_restore）：下一帧用它覆盖 turns，
	 * 然后自动清 abs_turns_valid —— 不能在主循环里直接改 turns，
	 * 因为第一帧的 first_sample 逻辑会把它清零 */
	int abs_turns;
	uint8_t abs_turns_valid;
} EncoderStruct;

/* 齿轮式双编的副编码器（输出轴侧）。只在主循环里更新，不是实时量 */
typedef struct{
	int raw;				// 最近一次原始读数
	float phase;			// 最近一次韦尼尔相位 Δ ∈ [0,1)
	float abs_turns;		// 最近一次解算出的绝对转子圈数（与 turns 同一坐标系，含小数）
	int lost_frames;		// 读出失败的次数
	uint8_t present;		// 最近一次探测的结果：1 = 副编码器在线
} SecEncoderStruct;


void ps_warmup(EncoderStruct * encoder, int n);
void ps_start_read(void);
int ps_finish_sample(EncoderStruct * encoder, float dt);
void ps_sample(EncoderStruct * encoder, float dt);
void ps_update_pll(EncoderStruct * encoder, float dt);
void ps_reset_pll(EncoderStruct * encoder);
void ps_print(EncoderStruct * encoder, int dt_ms);

/* ---- 齿轮式双编（断电记忆输出轴位置） ----
 * 数学部分在 dual_gear.h（host 可测）；这里是硬件侧：SPI 读数与配置/持久化。
 * 三个动作都只在主循环、门极关闭时调用。返回 0 = 成功，负数 = 失败原因。 */
int ps_sec_read(uint16_t * raw_out);				// 读一次副编码器（内部屏蔽控制中断）
int ps_sec_probe(void);								// 检测副编码器是否在线（连读两帧），更新 sec_encoder.present
int ps_dual_gear_config_ok(void);					// 齿数/方向配置是否可用（等于 dg_params_ok）
int ps_dual_gear_restore(EncoderStruct * encoder);	// 上电：解算并恢复绝对多圈位置
int ps_dual_gear_capture(EncoderStruct * encoder);	// 标定：把当前位置定为相位/计数参考
int ps_dual_gear_read(EncoderStruct * encoder);		// 诊断：读一次并刷新遥测（不写 flash）
void ps_set_m_zero(int32_t v);						// 改机械零点：当前位置的读数归零，双编参考同步平移
													// （v 就是当前计数时顺手重取参考，让记忆窗口重新以零点为中心）

#endif /* INC_POSITION_SENSOR_H_ */
