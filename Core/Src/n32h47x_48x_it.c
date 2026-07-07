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
#include "structs.h"
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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
	 * safe state using only register writes.  The DRV8323 enable pin is pulled
	 * low by GPIO (not SPI - the SPI bus may be wedged or the fault may have
	 * happened mid-transaction), which puts it to sleep and turns off all six
	 * FETs.  PWM outputs are also disabled at the timer.  The watchdog will
	 * eventually reset the chip. */
	HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_RESET);
	TIM1->BDTR &= ~TIM_BDTR_MOE;
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

	/* Kick the watchdog - if this loop ever stops running, the chip resets */
	IWDG->KR = 0xAAAA;

	/* Sample ADCs */
	analog_sample(&controller);

	/* Sample position sensor */
	ps_sample(&comm_encoder, DT);

	/* Run Finite State Machine */
	run_fsm(&state);

	/* Check for CAN messages */
	can_tx_rx();

	/* increment loop count */
	controller.loop_count++;
	//HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );

}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART3_IRQHandler(void)
{
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

	if(FDCAN_GetRxFifoFillLevel(FDCAN2, FDCAN_RX_FIFO0) > 0){
    GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		FDCAN_GetRxMsg(FDCAN2, FDCAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN

		if(can_rx.rx_header.DLC == 8){
			/* Legacy protocol: all 8-byte frames.  Reply with the standard packed telemetry */
			pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, controller.v_bus_filt);	// Pack response
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
		else if((can_rx.rx_header.DLC == 2) && (can_rx.data[0] == CAN_OP_READ_VAR)){
			/* Extended protocol: float variable read.  Replies with the requested
			 * variable as a real float32.  Deliberately does NOT reset the command
			 * timeout - telemetry polling shouldn't keep a stale command alive */
			pack_float_reply(&can_tx, can_rx.data[1], can_read_variable(can_rx.data[1]));
      FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
		}
		else if((can_rx.rx_header.DLC == 3) && (can_rx.data[0] == CAN_OP_READ_VAR)){
			/* Float variable pair read: two variables in one 8-byte reply */
			pack_float_pair_reply(&can_tx, can_read_variable(can_rx.data[1]), can_read_variable(can_rx.data[2]));
      FDCAN_AddMsgToTxFifoQ(FDCAN2, &can_tx.tx_header, can_tx.data);
		}
		/* Frames with any other length/opcode are ignored */
	}

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
