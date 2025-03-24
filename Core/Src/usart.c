/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

USART_InitType huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{
  GPIO_InitType GPIO_InitStructure;
  NVIC_InitType NVIC_InitStructure;  

  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOB | RCC_AHB_PERIPHEN_GPIOC, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
  RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE);

  GPIO_InitStructure.Pin        = USART_TX_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
  GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_USART3_TX;
  GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
  GPIO_InitPeripheral(USART_TX_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin        = USART_RX_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_USART3_RX;
  GPIO_InitPeripheral(USART_RX_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin        = USART_EN_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitPeripheral(USART_EN_GPIO_Port, &GPIO_InitStructure);

  huart2.BaudRate           = 115200;
  huart2.Parity             = USART_PE_NO;
  huart2.WordLength         = USART_WL_8B;
  huart2.StopBits            = USART_STPB_1;
  huart2.HardwareFlowControl = USART_HFCTRL_NONE;
  huart2.Mode                = USART_MODE_RX | USART_MODE_TX;
  USART_Init(USART3, &huart2);

  NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Enable(USART3, ENABLE);
}

/* USER CODE BEGIN 1 */
#include <stdio.h>
int fputc(int ch, FILE *stream) {
  USART_SendData(USART3, ch);
  while (USART_GetFlagStatus(USART3, USART_FLAG_TXC) == RESET);
  return ch;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
