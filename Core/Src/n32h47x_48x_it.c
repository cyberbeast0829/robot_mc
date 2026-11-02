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
	if(ps_finish_sample(&comm_encoder, DT)){
		/* 速度 PLL 只喂速度环/位置环，丢帧时不预测（见 ps_update_pll） */
		ps_update_pll(&comm_encoder, DT);
	}
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
	/* 25 us 时基：协议模块内部折算出 1 ms（心跳、超时） */
	can_proto_tick();

	/* 每个控制周期最多取两帧。总线上最多约 2 帧/ms，处理速度远高于此；
	 * 限制数量是为了不让大量参数帧挤占 FOC 的 25 us 时隙。 */
	can_frame_t frame;
	for(uint8_t i = 0; i < 2u; i++){
		if(!can_rx_fetch(&frame)){break;}
		GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		can_proto_rx(&frame);
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
