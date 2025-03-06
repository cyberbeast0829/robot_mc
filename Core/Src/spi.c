/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_InitType hspi1 = {
  .DataDirection         = SPI_DIR_DOUBLELINE_FULLDUPLEX,
  .SpiMode               = SPI_MODE_MASTER,
  .DataLen               = SPI_DATA_SIZE_16BITS,
  .CLKPOL                = SPI_CLKPOL_LOW,
  .CLKPHA                = SPI_CLKPHA_FIRST_EDGE,
  .NSS                   = SPI_NSS_SOFT,
  .BaudRatePres          = SPI_BR_PRESCALER_4,
  .FirstBit              = SPI_FB_MSB,
  .CRCPoly               = 10,
};

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  GPIO_InitType GPIO_InitStructure;
  NVIC_InitType NVIC_InitStructure;  

  // Enable peripheral clocks
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_GPIOB, ENABLE);

  // 将JTAG禁用掉，否则PA15，PB3，PB4将无法正常用于SPI1
  GPIO_ConfigPinRemap(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE0, GPIO_RMP_SWJ_SWD);

  // PB3 --> SPI_SCK
  // PB4 --> SPI_MISO
  // PB5 --> SPI_MOSI
  GPIO_InitStructure.Pin        = SPI_SCK_Pin | SPI_MISO_Pin | SPI_MOSI_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
  GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
  GPIO_InitStructure.GPIO_Alternate  = GPIO_AF_SPI1_SCK_PB3; // GPIO_AF_SPI1_MISO_PB4/GPIO_AF_SPI1_MOSI_PB5
  GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
  GPIO_InitPeripheral(SPI_SCK_GPIO_Port, &GPIO_InitStructure);

  // PA15 --> ENC_NSS
  GPIO_InitStructure.Pin        = ENC_SPI_NSS_Pin;
  GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitPeripheral(ENC_SPI_NSS_GPIO_Port, &GPIO_InitStructure);
  GPIO_WriteBits(ENC_SPI_NSS_GPIO_Port, ENC_SPI_NSS_Pin, Bit_SET);

  // PC10 --> SEC_ENC_NSS
  GPIO_InitStructure.Pin        = SEC_ENC_SPI_NSS_Pin;
  GPIO_InitPeripheral(SEC_ENC_SPI_NSS_GPIO_Port, &GPIO_InitStructure);
  GPIO_WriteBits(SEC_ENC_SPI_NSS_GPIO_Port, SEC_ENC_SPI_NSS_Pin, Bit_SET);

  SPI_Init(SPI1, &hspi1);

  SPI_SSOutputEnable(SPI1, DISABLE);

  SPI_Enable(SPI1, ENABLE);

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
