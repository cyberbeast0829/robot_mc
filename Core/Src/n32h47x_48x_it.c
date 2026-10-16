/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "n32h47x_48x_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "structs.h"
#include "math_ops.h"
#include "usart.h"
#include "fsm.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "foc.h"
#include "can.h"
#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"
#include "usb_istr.h"
#include "timing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* Get bytes of data according to the DLC value */
static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void fault_shutdown(void){
	/* Called from fault handlers and Error_Handler: get the power stage into a
	 * safe state using only register writes.  The gate drive is disabled at the
	 * ATIM1 break / main-output-enable bits (never SPI - the bus may be wedged or
	 * the fault may have happened mid-transaction), which turns off all six FETs.
	 * The watchdog will eventually reset the chip. */
	ATIM1->BKDT &= ~(TIM_BKDT_AOEN | TIM_BKDT_MOEN);
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_InitType hcan1;
extern USART_InitType huart2;
extern uint32_t _reboot_cookie;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  fault_shutdown();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  fault_shutdown();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  fault_shutdown();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  fault_shutdown();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  fault_shutdown();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void ATIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	//HAL_GPIO_WritePin(LED, GPIO_PIN_SET );	// Useful for timing
  TIM_ClrIntPendingBit(ATIM1, TIM_INT_UPDATE);

	/* Timing instrumentation (see timing.h).  ISR_TIMING_ENABLE = 0 compiles it
	 * out; the marks have to stay in place so the stages stay in step with the
	 * ISR_SEG_* indices.  This is also what catches the handler overrunning its
	 * 25 us slot - period_max is then a multiple of isr_period (6000 cycles) */
	ISR_TIMING_BEGIN();

	/* Start the encoder frame before anything else.  It needs 16 bits at 7.5 MHz
	 * = 2.13 us (521 cycles) on the wire and nothing below needs the result, so
	 * the transfer runs while the ADCs are read and the faults are checked
	 * instead of the CPU sitting out the wait.  ps_finish_sample() collects it
	 * below - see the note in position_sensor.c */
	ps_start_read();

	/* Kick the watchdog - if this loop ever stops running, the chip resets */
	IWDG_ReloadKey();

	/* Sample ADCs */
	analog_sample(&controller);
	ISR_TIMING_MARK(ISR_SEG_ADC);

	/* Check for latched faults (overvoltage, overcurrent).  Position independent,
	 * so it belongs inside the encoder frame rather than after it */
	check_faults(&controller);
	ISR_TIMING_MARK(ISR_SEG_FAULTS);

	/* Collect the encoder frame started above (the marks are in execution order,
	 * so FAULTS now precedes ENC - read the segments by name) */
	ps_finish_sample(&comm_encoder, DT);
	ISR_TIMING_MARK(ISR_SEG_ENC);

	/* Run Finite State Machine */
	run_fsm(&state);
	ISR_TIMING_MARK(ISR_SEG_FSM);

	/* Check for CAN messages */
	can_tx_rx();
	ISR_TIMING_MARK(ISR_SEG_CAN);

	ISR_TIMING_END();

	/* increment loop count */
	controller.loop_count++;
	//HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* Overrun first.  The receive register is overwritten (and ORE latches,
   * blocking all further reception) whenever this handler is late - which
   * happens routinely while flash is erased, because the CPU stalls for the
   * whole page erase.  Without clearing ORE the serial console would die for
   * good the first time the user types during a preference save.
   * Reading STS above then DAT clears it; the byte itself is dropped because
   * its position in the stream is no longer known. */
  if (SET == USART_GetFlagStatus(USART3, USART_FLAG_OREF)) {
    (void)USART_ReceiveData(USART3);
    USART_ClrFlag(USART3, USART_FLAG_OREF);
  }
  if (SET == USART_GetIntStatus(USART3, USART_INT_RXDNE)) {
    USART_ClrFlag(USART3, USART_FLAG_RXDNE);
    char c = USART_ReceiveData(USART3);
    update_fsm(&state, c);
  }
}

void USB_FS_LP_IRQHandler(void)
{
  USB_Istr();
}

/**
 * @brief  This function handles USB WakeUp interrupt request.
 */
void USB_FS_WKUP_IRQHandler(void)
{
  EXTI_ClrITPendBit(EXTI_LINE18);
}


/* USER CODE BEGIN 1 */

void can_tx_rx(void){

	if(FDCAN_GetRxFifoFillLevel(FDCAN2, FDCAN_RX_FIFO0) <= 0){
		return;		// no message pending
	}

  GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	FDCAN_GetRxMsg(FDCAN2, FDCAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN

	/* The 11-bit CAN ID is (opcode << 7) | node_id - see the table in can.h.
	 * The hardware filter only accepts this node's ID, so just dispatch on
	 * the opcode.  Malformed frames (wrong length, NaN floats) are ignored */
	uint32_t opcode = CAN_OPCODE(can_rx.rx_header.ID);
	uint32_t dlc = DLCtoBytes[can_rx.rx_header.DataLength >> FDCAN_ELEMENT_DLC_OFFSET];
	int send_telemetry_reply = 0;
	float f_a, f_b;

	switch(opcode){

		case CAN_OP_LEGACY:
			if(dlc == 8){
				/* Legacy protocol: all 8-byte frames.  Reply with the packed telemetry */
				pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, controller.v_bus_filt);	// Pack response
        FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);	// Send response

				/* Check for special Commands: bytes 0-6 all 0xFF, command in byte 7 */
				int special = (can_rx.data[0] & can_rx.data[1] & can_rx.data[2] & can_rx.data[3] & can_rx.data[4] & can_rx.data[5] & can_rx.data[6]) == 0xFF;
				if(special && (can_rx.data[7] == 0xFC)){
					update_fsm(&state, MOTOR_CMD);
				}
				else if(special && (can_rx.data[7] == 0xFD)){
					update_fsm(&state, MENU_CMD);
				}
				else if(special && (can_rx.data[7] == 0xFE)){
					update_fsm(&state, ZERO_CMD);
				}
        else if (special && (can_rx.data[7] == 0xFF)){
          __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
          _reboot_cookie = 0xDEADBEEF;
          NVIC_SystemReset();
        }
				else{
					unpack_cmd(can_rx, controller.commands);	// Unpack commands
					controller.timeout = 0;						// Reset timeout counter
				}
			}
			else if((dlc == 2) && (can_rx.data[0] == CAN_LEGACY_READ_VAR)){
				/* Legacy single float read */
				pack_float_reply(&can_tx, can_rx.data[1], can_read_variable(can_rx.data[1]));
				FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
			}
			else if((dlc == 3) && (can_rx.data[0] == CAN_LEGACY_READ_VAR)){
				/* Legacy float pair read */
				pack_float_pair_reply(&can_tx, can_read_variable(can_rx.data[1]), can_read_variable(can_rx.data[2]));
				FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
			}
			break;

		case CAN_OP_SET_PV:
			if((dlc == 8) && can_unpack_two_floats(&can_rx, &f_a, &f_b)){
				controller.p_des = fast_fmaxf(fast_fminf(f_a, P_MAX), P_MIN);
				controller.v_des = fast_fmaxf(fast_fminf(f_b, V_MAX), V_MIN);
				controller.timeout = 0;
				send_telemetry_reply = 1;
			}
			break;

		case CAN_OP_SET_GAINS:
			if((dlc == 8) && can_unpack_two_floats(&can_rx, &f_a, &f_b)){
				controller.kp = fast_fmaxf(fast_fminf(f_a, KP_MAX), KP_MIN);
				controller.kd = fast_fmaxf(fast_fminf(f_b, KD_MAX), KD_MIN);
				controller.timeout = 0;
				send_telemetry_reply = 1;
			}
			break;

		case CAN_OP_SET_TFF:
			if(dlc == 4){
				memcpy(&f_a, &can_rx.data[0], sizeof(float));
				if(!isnan(f_a)){
					controller.t_ff = fast_fmaxf(fast_fminf(f_a, I_MAX*KT*GR), -I_MAX*KT*GR);
					controller.timeout = 0;
					send_telemetry_reply = 1;
				}
			}
			break;

		case CAN_OP_SET_REPLY_FORMAT:
			/* Select which two variables come back in telemetry replies.
			 * Stored in RAM - re-send after a power cycle.  Doesn't reset the
			 * command timeout.  The reply acts as an ack, in the new format */
			if(dlc == 2){
				can_tx.reply_var_a = can_rx.data[0];
				can_tx.reply_var_b = can_rx.data[1];
				send_telemetry_reply = 1;
			}
			break;

		case CAN_OP_READ_VARS:
			/* One-shot read of one or two variables.  Doesn't change the
			 * configured reply format, and doesn't reset the command timeout -
			 * telemetry polling shouldn't keep a stale command alive */
			if(dlc == 2){
				pack_telemetry_reply(&can_tx, can_rx.data[0], can_rx.data[1]);
				FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
			}
			else if(dlc == 1){
				pack_telemetry_reply(&can_tx, can_rx.data[0], 0xFF);	// second slot reads back NaN
        FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
			}
			break;

		case CAN_OP_CONTROL:
			if(dlc == 1){
				if(can_rx.data[0] == CAN_CTRL_MOTOR_MODE){update_fsm(&state, MOTOR_CMD);}
				else if(can_rx.data[0] == CAN_CTRL_MENU_MODE){update_fsm(&state, MENU_CMD);}
				else if(can_rx.data[0] == CAN_CTRL_ZERO){update_fsm(&state, ZERO_CMD);}
			}
			break;

		/* Unknown opcodes are ignored */
	}

	if(send_telemetry_reply){
		pack_telemetry_reply(&can_tx, can_tx.reply_var_a, can_tx.reply_var_b);
		FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
	}

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
