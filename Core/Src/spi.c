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
  /* KTH7823 SPI 手册第 5 节：通信模式为 mode3（CPOL = 1，CPHA = 1）。
   * 之前是 mode0（SCK 空闲低、第一个边沿采样），编码器的 MISO 是在 SCK 下降沿
   * 更新的，用 mode0 会在"采样沿 = 数据更新沿"上抢跑，抓到的 16 位帧会随机
   * 整体移一位（表现为高位跳变）。改成 mode3 后采样沿（上升沿）落在数据位中间 */
  .CLKPOL                = SPI_CLKPOL_HIGH,
  .CLKPHA                = SPI_CLKPHA_SECOND_EDGE,
  .NSS                   = SPI_NSS_SOFT,
  .BaudRatePres          = SPI_BR_PRESCALER_16,
  .FirstBit              = SPI_FB_MSB,
  .CRCPoly               = 10,
};

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  GPIO_InitType GPIO_InitStructure;
  DMA_InitType DMA_InitStructure;
  // NVIC_InitType NVIC_InitStructure;   // CAN RX is polled from the commutation ISR, no SPI interrupt is used

  // Enable peripheral clocks
  RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_DMA2, ENABLE);
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

  DMA_DeInit(DMA2_CH2);
  DMA_InitStructure.PeriphAddr     = (uint32_t)&SPI1->DAT;
  DMA_InitStructure.MemAddr        = (uint32_t)0; // 会被应用赋值
  DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
  DMA_InitStructure.BufSize        = 0; // 会被应用赋值
  DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
  DMA_InitStructure.MemoryInc      = DMA_MEM_INC_ENABLE;
  DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;
  DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_HALFWORD;
  DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
  DMA_InitStructure.Priority       = DMA_PRIORITY_MEDIUM;
  DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
  DMA_Init(DMA2_CH2, &DMA_InitStructure);
  DMA_RequestRemap(DMA_REMAP_SPI1_TX, DMA2_CH2, ENABLE);

  DMA_DeInit(DMA2_CH1);
  DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
  DMA_Init(DMA2_CH1, &DMA_InitStructure);
  DMA_RequestRemap(DMA_REMAP_SPI1_RX, DMA2_CH1, ENABLE);

  DMA_EnableChannel(DMA2_CH2, DISABLE);
  DMA_EnableChannel(DMA2_CH1, DISABLE);

  SPI_Init(SPI1, &hspi1);

  SPI_SSOutputEnable(SPI1, DISABLE);

  SPI_I2S_EnableDma(SPI1, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, ENABLE);  

  SPI_Enable(SPI1, ENABLE);

}

void SPI_Transmit_DMA(uint8_t *txBuf, uint16_t bufLen)
{
  DMA_SetCurrDataCounter(DMA2_CH2, bufLen);

  DMA2_CH2->MADDR = (uint32_t)txBuf;
  DMA_EnableChannel(DMA2_CH2, ENABLE);
}

void SPI_Receive_DMA(uint8_t *rxBuf, uint16_t bufLen)
{
  DMA_SetCurrDataCounter(DMA2_CH1, bufLen);

  DMA2_CH1->MADDR = (uint32_t)rxBuf;
  DMA_EnableChannel(DMA2_CH1, ENABLE);
  // SPI_I2S_EnableInt(SPI1, SPI_I2S_INT_RNE, ENABLE);
  // DMA_ConfigInt(DMA2_CH1, DMA_INT_TXC, ENABLE);
}

void SPI_StopTransmit_DMA()
{
  DMA_EnableChannel(DMA2_CH2, DISABLE);
}

void SPI_StopReceive_DMA()
{
  // SPI_I2S_EnableInt(SPI1, SPI_I2S_INT_RNE, DISABLE);
  // DMA_ConfigInt(DMA2_CH1, DMA_INT_TXC, DISABLE);
  DMA_EnableChannel(DMA2_CH1, DISABLE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
