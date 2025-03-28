/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitType GPIO_InitStructure;

  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOB | RCC_AHB_PERIPHEN_GPIOC, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

  GPIO_InitStruct(&GPIO_InitStructure);

  GPIO_InitStructure.Pin        = MUX1_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
  GPIO_InitStructure.GPIO_Current = GPIO_DC_2mA;
  GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SR_SLOW_SLEW;
  GPIO_InitPeripheral(MUX1_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin        = MUX2_Pin;
  GPIO_InitPeripheral(MUX2_GPIO_Port, &GPIO_InitStructure);

  GPIO_ResetBits(MUX1_GPIO_Port, MUX1_Pin);
  GPIO_SetBits(MUX2_GPIO_Port, MUX2_Pin);

  GPIO_InitStructure.Pin        = LED_Pin;
  GPIO_InitPeripheral(LED_GPIO_Port, &GPIO_InitStructure);

  GPIO_ResetBits(LED_GPIO_Port, LED_Pin);

  GPIO_InitStructure.Pin        = RM_EN_Pin;
  GPIO_InitPeripheral(RM_EN_GPIO_Port, &GPIO_InitStructure);

  GPIO_SetBits(RM_EN_GPIO_Port, RM_EN_Pin);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
