/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  GPIO_InitType GPIO_InitStructure;
  TIM_TimeBaseInitType TIM_TimeBaseStructure;
  OCInitType TIM_OCInitStructure;
  TIM_BDTRInitType TIM_BDTRInitStructure;
  NVIC_InitType NVIC_InitStructure; 

  // Enable peripheral clocks
  RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_ATIM1, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_GPIOB, ENABLE);

  // PA8 --> GHS_U
  // PA9 --> GHS_V
  // PA10 --> GHS_W
  GPIO_InitStruct(&GPIO_InitStructure);
  GPIO_InitStructure.Pin        = GHS_U_Pin | GHS_V_Pin | GHS_W_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
  GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_ATIM1_CH1_PA8; // GPIO_AF_ATIM1_CH2_PA9/GPIO_AF_ATIM1_CH3_PA10
  GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
  GPIO_InitPeripheral(GHS_GPIO_Port, &GPIO_InitStructure);

  // PA7 --> GLS_U
  // PB0 --> GLS_V
  // PB1 --> GLS_W
  GPIO_InitStructure.Pin = GLS_U_Pin;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_ATIM1_CH1N_PA7;
  GPIO_InitPeripheral(GLS_U_GPIO_Port, &GPIO_InitStructure);
  GPIO_InitStructure.Pin = GLS_V_Pin | GLS_W_Pin;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_ATIM1_CH2N_PB0; // GPIO_AF_ATIM1_CH3N_PB1
  GPIO_InitPeripheral(GLS_VW_GPIO_Port, &GPIO_InitStructure);

  TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.Prescaler = 0;
  TIM_TimeBaseStructure.CounterMode = TIM_CNT_MODE_CENTER_ALIGN1;
  TIM_TimeBaseStructure.Period    = 3000; // HCLK = 240M, 240M/3000/2 = 40KHz PWM
  TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;
  TIM_TimeBaseStructure.RepetCnt  = 1;
  TIM_InitTimeBase(ATIM1, &TIM_TimeBaseStructure);

  // 使用SYSCLK作为ATIM的时钟源
  RCC_ConfigATimClk(RCC_ATIM_CLKSRC_SYSCLK);

  TIM_InitOcStruct(&TIM_OCInitStructure);
  TIM_OCInitStructure.OCMode       = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
  TIM_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
  TIM_OCInitStructure.Pulse        = 0;
  TIM_OCInitStructure.OCPolarity   = TIM_OC_POLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity  = TIM_OCN_POLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState  = TIM_OC_IDLE_STATE_RESET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCN_IDLE_STATE_RESET;

  TIM_InitOc1(ATIM1, &TIM_OCInitStructure);
  TIM_ConfigOc1Fast(ATIM1, TIM_OC_FAST_ENABLE);
  TIM_ConfigOc1Preload(ATIM1, TIM_OC_PRE_LOAD_ENABLE);
  TIM_InitOc2(ATIM1, &TIM_OCInitStructure);
  TIM_ConfigOc2Fast(ATIM1, TIM_OC_FAST_ENABLE);
  TIM_ConfigOc2Preload(ATIM1, TIM_OC_PRE_LOAD_ENABLE);
  TIM_InitOc3(ATIM1, &TIM_OCInitStructure);
  TIM_ConfigOc3Fast(ATIM1, TIM_OC_FAST_ENABLE);
  TIM_ConfigOc3Preload(ATIM1, TIM_OC_PRE_LOAD_ENABLE);

  // No master slave mode
  TIM_SelectOutputTrig(ATIM1, TIM_TRGO_SRC_UPDATE);
  TIM_SelectMasterSlaveMode(ATIM1, TIM_MASTER_SLAVE_MODE_DISABLE);

  TIM_InitBkdtStruct(&TIM_BDTRInitStructure);
  TIM_BDTRInitStructure.OSSRState       = TIM_OSSR_STATE_DISABLE;
  TIM_BDTRInitStructure.OSSIState       = TIM_OSSI_STATE_DISABLE;
  TIM_BDTRInitStructure.LOCKLevel       = TIM_LOCK_LEVEL_OFF;
  TIM_BDTRInitStructure.DeadTime        = 20;
  TIM_BDTRInitStructure.Break           = TIM_BREAK_IN_ENABLE;
  TIM_BDTRInitStructure.BreakPolarity   = TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_DISABLE;
  TIM_BDTRInitStructure.Bidirection     = TIM_BREAK_BID_DISABLE;

  TIM_ConfigBkdt(ATIM1, &TIM_BDTRInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel                   = ATIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	TIM_EnableCtrlPwmOutputs(ATIM1, DISABLE);
	/* Prime the INTERNAL repetition counter with 0, then leave its reload
	 * value at 1 without another software update.  The first overflow (CNT=AR)
	 * then generates UEV and reloads 1; subsequent UEVs stay at CNT=AR, once
	 * per full 40 kHz PWM cycle.  Loading 1 with UG before starting instead
	 * puts UEV/TRGO at CNT=0, where PWM1 has all low-side switches off.
	 * UEV also transfers the PWM compare preloads and triggers ADC1/2/3.
	 * Do not issue another software update after this sequence. */
	ATIM1->REPCNT = 0;
	TIM_GenerateEvent(ATIM1, TIM_EVT_SRC_UPDATE);
	ATIM1->REPCNT = 1;
	TIM_ClrIntPendingBit(ATIM1, TIM_INT_UPDATE);
	TIM_Enable(ATIM1, ENABLE);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
