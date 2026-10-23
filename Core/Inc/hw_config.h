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
#define SEC_ENC_SPI_NSS_Pin             GPIO_PIN_10
#define SEC_ENC_SPI_NSS_GPIO_Port       GPIOC

#define USB_DM_Pin                      GPIO_PIN_11
#define USB_DP_Pin                      GPIO_PIN_12
#define USB_GPIO_Port                   GPIOA

#define GHS_U_Pin                       GPIO_PIN_8
#define GHS_V_Pin                       GPIO_PIN_9
#define GHS_W_Pin                       GPIO_PIN_10
#define GHS_GPIO_Port                   GPIOA
#define GLS_U_Pin                       GPIO_PIN_7
#define GLS_U_GPIO_Port                 GPIOA
#define GLS_V_Pin                       GPIO_PIN_0
#define GLS_W_Pin                       GPIO_PIN_1
#define GLS_VW_GPIO_Port                GPIOB

#define AUX_H_Pin                       GPIO_PIN_8
#define AUX_L_Pin                       GPIO_PIN_9
#define AUX_GPIO_Port                   GPIOB

#define VBUS_AD_Pin                     GPIO_PIN_2
#define VBUS_AD_GPIO_Port               GPIOB
#define MOS_TEMP_Pin                    GPIO_PIN_15
#define MOS_TEMP_GPIO_Port              GPIOB
#define MOT_TEMP_Pin                    GPIO_PIN_4
#define MOT_TEMP_GPIO_Port              GPIOC

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

#define CAN_TX_Pin                      GPIO_PIN_13
#define CAN_TX_GPIO_Port                GPIOB
#define CAN_RX_Pin                      GPIO_PIN_12
#define CAN_RX_GPIO_Port                GPIOB

#define USART_TX_Pin                    GPIO_PIN_6
#define USART_TX_GPIO_Port              GPIOB
#define USART_RX_Pin                    GPIO_PIN_7
#define USART_RX_GPIO_Port              GPIOB
#define USART_EN_Pin                    GPIO_PIN_11
#define USART_EN_GPIO_Port              GPIOC

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
#define LED_GPIO_Port                   GPIOC

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

/* Misc. GPIO */
#define LED         	LED_GPIO_Port, LED_Pin	// LED Pin

/* CAN (the FDCAN2_InitType is defined in can.c; the old CAN_H = hcan1 alias was
 * removed - it was an ST-hal leftover and was never used) */

/* Other hardware-related constants */
/* The three low-side shunts are 5 mOhm (confirmed on this board), not
 * 0.5 mOhm.  With the same ADC/PGA configuration the old 0.050354 A/count
 * coefficient over-reported current by 10x.  Keep the original conversion
 * convention, corrected for the actual shunts; verify absolute gain against
 * a phase-current measurement before increasing the board current limit. */
#define I_SCALE 			0.005035400390625f  // Nominal amps per A/D count with 5 mOhm shunts
// V_SCALE = (3.3 * VBUS_S_DIVIDER) / (2^12) 其中 VBUS_S_DIVIDER = 93.91 / 3.01
#define V_SCALE 			0.02513618342504f    // Bus volts per A/D Count
#define DTC_MAX 			0.7f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
#define DT					.000025f		// Loop period
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization
#define V_BUS_MAX			60.0f			// max drive voltage - torque command is cut above this (soft limit)
#define V_BUS_FAULT			67.0f			// hard overvoltage limit - latches a fault and disables the gate drive

/* Conservative limit for the 5 mOhm board.  At I_MAX=4 A the 1.5x OC trip
 * is 6 A, about 1192 ADC counts from zero with I_SCALE above, leaving room
 * around the measured midscale offsets.  The previous 40 A limit no longer
 * fits the corrected measurement range.  Raising this requires checking
 * both the analogue sense range and the power-stage/shunt ratings. */
#define I_MAX_LIMIT			4.0f			// Hard upper bound for I_MAX (A)
#define I_OC_FAULT_FACTOR	1.5f			// Overcurrent latch threshold = factor * I_MAX

/* Current controller */
#define L_D     95e-6f				// D axis inductance
#define L_Q     124e-6f				// Q axis inductance
#define K_D     .1f                    // Loop gain,  Volts/Amp
#define K_Q     .1f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.00012f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.045f                // PI zero, in radians per sample
#define KI_Q 0.045f                // PI zero, in radians per sample
#define OVERMODULATION 1.0f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.1f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter


#endif
