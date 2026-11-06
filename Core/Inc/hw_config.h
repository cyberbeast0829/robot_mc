#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define SPI_SCK_Pin                     GPIO_PIN_3
#define SPI_SCK_GPIO_Port               GPIOB
#define SPI_MISO_Pin                    GPIO_PIN_4
#define SPI_MISO_GPIO_Port              GPIOB
#define SPI_MOSI_Pin                    GPIO_PIN_5
#define SPI_MOSI_GPIO_Port              GPIOB
#define ENC_SPI_NSS_Pin                 GPIO_PIN_15
#define ENC_SPI_NSS_GPIO_Port           GPIOA
#define SEC_ENC_SPI_NSS_Pin             GPIO_PIN_1
#define SEC_ENC_SPI_NSS_GPIO_Port       GPIOD

#define USB_DM_Pin                      GPIO_PIN_11
#define USB_DP_Pin                      GPIO_PIN_12
#define USB_GPIO_Port                   GPIOA

#define GHS_U_Pin                       GPIO_PIN_8
#define GHS_V_Pin                       GPIO_PIN_9
#define GHS_W_Pin                       GPIO_PIN_10
#define GHS_GPIO_Port                   GPIOA
#define GLS_U_Pin                       GPIO_PIN_13
#define GLS_U_GPIO_Port                 GPIOB
#define GLS_V_Pin                       GPIO_PIN_14
#define GLS_W_Pin                       GPIO_PIN_15
#define GLS_VW_GPIO_Port                GPIOB

#define AUX_H_Pin                       GPIO_PIN_0
#define AUX_L_Pin                       GPIO_PIN_1
#define AUX_GPIO_Port                   GPIOC

#define VBUS_AD_Pin                     GPIO_PIN_7
#define VBUS_AD_GPIO_Port               GPIOE
#define MOS_TEMP_Pin                    GPIO_PIN_7
#define MOS_TEMP_GPIO_Port              GPIOA
#define MOT_TEMP_Pin                    GPIO_PIN_8
#define MOT_TEMP_GPIO_Port              GPIOE

#define IU_OP1P_Pin                     GPIO_PIN_1
#define IU_OP1N_Pin                     GPIO_PIN_2
#define IU_OP1_GPIO_Port                GPIOA
#define IV_OP2P_Pin                     GPIO_PIN_4
#define IV_OP2N_Pin                     GPIO_PIN_3
#define IV_OP2_GPIO_Port                GPIOA
#define IW_OP3P_Pin                     GPIO_PIN_6
#define IW_OP3N_Pin                     GPIO_PIN_5
#define IW_OP3_GPIO_Port                GPIOA

#define IV_OP3OUT_COMP_Pin              GPIO_PIN_11
#define IV_OP3OUT_COMP_GPIO_Port        GPIOB

#define CAN_TX_Pin                      GPIO_PIN_6
#define CAN_TX_GPIO_Port                GPIOB
#define CAN_RX_Pin                      GPIO_PIN_7
#define CAN_RX_GPIO_Port                GPIOB

#define USART_TX_Pin                    GPIO_PIN_10
#define USART_TX_GPIO_Port              GPIOB
#define USART_RX_Pin                    GPIO_PIN_11
#define USART_RX_GPIO_Port              GPIOB
#define USART_EN_Pin                    GPIO_PIN_2
#define USART_EN_GPIO_Port              GPIOD

#define EXT_SDA_RX_Pin                  GPIO_PIN_7
#define EXT_SCL_TX_Pin                  GPIO_PIN_6
#define EXT_I2C_USART_GPIO_Port         GPIOB

#define DIR_IN_Pin                      GPIO_PIN_14
#define DIR_IN_GPIO_Port                GPIOC
#define RM_EN_Pin                       GPIO_PIN_14
#define RM_EN_GPIO_Port                 GPIOB

#define MUX1_Pin                        GPIO_PIN_15
#define MUX1_GPIO_Port                  GPIOB
#define MUX2_Pin                        GPIO_PIN_13
#define MUX2_GPIO_Port                  GPIOC

#define PWM_IN_OUT_Pin                  GPIO_PIN_0
#define PWM_IN_OUT_GPIO_Port            GPIOA

#define GPIO1_Pin                       GPIO_PIN_15
#define GPIO1_GPIO_Port                 GPIOC
#define GPIO2_Pin                       GPIO_PIN_14
#define GPIO2_GPIO_Port                 GPIOC

#define LED_Pin                         GPIO_PIN_6
#define LED_GPIO_Port                   GPIOE

/* Timer and PWM */
#define TIM_PWM			ATIM1				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal W timer channel
#define INVERT_DTC		1					// PWM inverting (1) or non-inverting (0)

/* ISRs */
#define PWM_ISR			ATIM1_UP_IRQn	// PWM Timer ISR
#define CAN_ISR			FDCAN2_INT0_IRQn		// CAN Receive ISR
#define USB_ISR			USB_FS_LP_IRQn	// USB ISR
#define USB_WKUP_ISR	USB_FS_WKUP_IRQn	// USB Wakeup ISR
#define USART_ISR		USART3_IRQn		// USART ISR

/* ADC
 *
 * NOTE: the old ST/DRV8323 handle macros (ADC_CH_IA = hadc1, ...) were removed -
 * they resolved to symbols that do not exist on this port and were never used.
 * The ADC instances are hard-coded in adc.c / foc.c (ADC1 = phase A, ADC2 =
 * phase B + VBUS). */

/* Gate drive
 *
 * NOTE: this hardware has no DRV8323.  There is no enable pin, no DRV SPI and
 * no CS (the old ENABLE_PIN/DRV_SPI/DRV_CS macros were removed).  The power
 * stage is enabled/disabled with the ATIM1 break/MOE bits - see
 * fsm_enter_state()/fsm_exit_state()/fault_shutdown(). */

/* SPI encoder (the commutation encoder is on SPI1 + DMA2, see spi.c/position_sensor.c) */
#define ENC_CS			ENC_SPI_NSS_GPIO_Port, ENC_SPI_NSS_Pin	// Encoder SPI CS pin
#define ENC_CPR			65536				// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x0000				// Encoder read command
/* 齿轮式双编的第二路编码器（输出轴侧）：同一颗 KTH7823、同一个 SPI1，只是 NSS
 * 独立（PD1）。帧格式/分辨率与主编码器相同，所以 CPR 和读命令直接复用。
 * 只在主循环里读（上电恢复、标定、gear_read），不进 40 kHz 中断 —— 见
 * position_sensor.c 的 ps_sec_read()。 */
#define SEC_ENC_CS		SEC_ENC_SPI_NSS_GPIO_Port, SEC_ENC_SPI_NSS_Pin
#define SEC_ENC_CPR		ENC_CPR
#define SEC_ENC_READ_WORD	ENC_READ_WORD

/* Misc. GPIO */
#define LED         	LED_GPIO_Port, LED_Pin	// LED Pin

/* CAN (the FDCAN2_InitType is defined in can.c; the old CAN_H = hcan1 alias was
 * removed - it was an ST-hal leftover and was never used) */

/* Other hardware-related constants */
/* Phase-current sense chain: low-side shunt -> on-chip PGA (differential) ->
 * 12-bit differential ADC.  I = (ADC count) * I_SCALE, with
 *     I_SCALE = (ADC_DIFF_SPAN / 2^12) / (R_SHUNT * PGA_GAIN)
 * ADC_DIFF_SPAN is the PGA output swing that the 4096 codes cover.  The PGA
 * output range is +-VREF/2 (3.3 V differential, code 2048 = 0 A), so one count
 * is ADC_DIFF_SPAN/4096 at the ADC and the measurable range is +-2048 counts:
 *     +-2048 * 0.0134277 A = +-27.5 A at gain 12 (was +-20.6 A at gain 16)
 * Do NOT additionally halve ADC_DIFF_SPAN - that makes the ADC saturate at
 * half the PGA range (it did, once: 0.0067139 A/count instead of 0.0134277).
 *
 * The three low-side shunts are 5 mOhm on this board, not 0.5 mOhm (the old
 * 0.050354 A/count coefficient assumed 0.5 mOhm and over-reported by 10x).
 *
 * PGA_GAIN is the ACTUAL differential gain.  It is derived from the PGA gain
 * field written in pga.c (PGA_CTRL_CH1GAIN_<n>_DIFF_<2n>); the actual gain is
 * n/2, and the field code is n's index in the gain table:
 *     code:      0    1    2    3    4     5     6     7
 *     <n>:       1    2    4    8   12    16    24    32
 *     actual:  0.5    1    2    4    6     8    12    16
 * code 7 (32_DIFF_64) is the old board -> I_SCALE = 0.01007080078125 A/count;
 * code 6 (24_DIFF_48) is this board      -> I_SCALE = 0.013427734375 A/count.
 * Changing the gain means editing pga.c AND PGA_GAIN_CODE here - pga.c has a
 * _Static_assert on PGA_GAIN_CODE so the two cannot drift apart.
 * Verify the absolute gain against a measured phase current before raising
 * the board current limit. */
#define ADC_VREF			3.3f			// ADC reference (V)
#define ADC_DIFF_SPAN		(ADC_VREF)		// differential span of the 4096 codes (PGA output +-VREF/2)
#define ADC_FULL_SCALE		4096.0f			// 12-bit converter
#define R_SHUNT				0.005f			// Low-side phase shunt (ohm)
#define PGA_GAIN_CODE		6				// Gain field: 6 = PGA_CTRL_CH1GAIN_24_DIFF_48
/* gain-table lookup: code -> <n> (see the table above) */
#define PGA_GAIN_N(code)	((code) == 0 ? 1 : (code) == 1 ? 2 : (code) == 2 ? 4 : \
							 (code) == 3 ? 8 : (code) == 4 ? 12 : (code) == 5 ? 16 : \
							 (code) == 6 ? 24 : 32)
#define PGA_GAIN			(PGA_GAIN_N(PGA_GAIN_CODE) / 2.0f)	// actual differential gain
#define I_SCALE				((ADC_DIFF_SPAN / ADC_FULL_SCALE) / (R_SHUNT * PGA_GAIN))
// V_SCALE = (3.3 * VBUS_S_DIVIDER) / (2^12) 其中 VBUS_S_DIVIDER = 93.91 / 3.01
#define V_SCALE 			0.02513618342504f    // Bus volts per A/D Count
#define DTC_MAX 			0.7f          	// Max duty cycle (calibration converts its voltages with CAL_V_PER_CMD - see calibration.c)
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
#define DT					.000025f		// Loop period
/* 硬件版本（system.hw_version_*；CYBERBEAST 的 QUERY_DEVICE_INFO 也报这组值）。
 * CBL89 板。换板/改版必须同步这三行 —— 上位机靠它区分引脚与量程。 */
#define HW_VERSION_MAJOR	4
#define HW_VERSION_MINOR	2
#define HW_VERSION_VARIANT	89
/* dq 电压指令 → 物理伏特：svm() 把 dq 电压按 v_scale = v_bus*(DTC_MAX-DTC_MIN)*sqrt(1/3)
 * 归一化，所以 1 个指令单位 = 0.5/((DTC_MAX-DTC_MIN)*sqrt(1/3)) 伏。
 * 标定（calibration.c 的 CAL_V_PER_CMD）和母线功率估算（foc.c 的 i_bus）共用它，
 * 改 svm()/DTC_MAX/DTC_MIN 时必须同步（sqrt(1/3) 与 math_ops.h 的 SQRT1_3 同值） */
#define V_PER_CMD			(0.5f/((DTC_MAX-DTC_MIN)*0.57735026919f))
/* 外环分频上限。速度环最低 40 kHz/100 = 400 Hz，仍远高于速度估计的可用带宽；
 * 再低就只是把相位裕度白白送掉 */
#define MOTION_PRESCALER_MAX	100
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization
#define V_BUS_MAX			60.0f			// max drive voltage - torque command is cut above this (soft limit).
											// The hard overvoltage / undervoltage limits are configuration
											// parameters (config.vbus_ov_trip, default 67 V; config.vbus_uv_trip,
											// default 0 = off) - see check_faults()

/* Conservative limit for the 5 mOhm board.  The PGA output range (+-VREF/2)
 * saturates 2048 counts from zero, i.e. at +-27.5 A with the gain above, and a
 * saturated reading can never exceed the OC threshold - so
 * I_OC_FAULT_FACTOR * I_MAX_LIMIT must stay below that (4 A -> 6 A trip =
 * ~447 counts, 10 A -> 15 A trip = ~1117 counts, both well inside).  Raising
 * this requires checking both the analogue sense range and the power-stage /
 * shunt ratings. */
#define I_MAX_LIMIT			10.0f			// Hard upper bound for I_MAX (A)
#define I_OC_FAULT_FACTOR	1.5f			// Overcurrent latch threshold = factor * I_MAX

/* Current controller
 *
 * k_d/k_q/ki_d/ki_q 不再用这里的常量，而是由标定出来的 R_PHASE / L_D / L_Q 与
 * config.i_bw 推导（foc.c 的 current_loop_retune()，极点对消），这样 i_bw 就是
 * 真正的穿越频率、且与电机无关。下面三个只是**未标定时的兜底**（出厂 R_PHASE=0），
 * 等价于"假设 L≈24 µH、R/L≈1800 rad/s"，保留是为了第一次上电/没跑标定时行为不变。 */
#define K_SCALE 0.00012f             // 兜底：k = K_SCALE*I_BW（假设 L≈24 µH）
#define KI_D 0.045f                // 兜底：PI 零点，rad/sample（假设 R/L≈1800 rad/s）
#define KI_Q 0.045f                // 同上（q 轴）
/* 推导出的 P 增益上限（指令 V/A）。防止 l_d/l_q 被写成量级错误的值时增益大 100
 * 倍直接失稳：本机（45 µH @ 1 kHz）只需 0.23，10 mH 级电机才用得到 10 */
#define I_LOOP_K_MAX		10.0f
#define OVERMODULATION 1.0f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.1f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter


#endif
