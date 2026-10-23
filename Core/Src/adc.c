/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"


/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_InitType ADC_InitStructure;
  // DMA_InitType DMA_InitStructure;

  // RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_DMA1, ENABLE);
  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_ADC1, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

  ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV8);
  RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV24);

  ADC_InitStruct(&ADC_InitStructure);
  ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
	/* ADC1 only uses the injected group.  The unused regular group still
	 * needs a valid SDK sequence length (1..16); zero encodes 16 channels. */
	ADC_InitStructure.MultiChEn      = DISABLE;
  ADC_InitStructure.ContinueConvEn = DISABLE;
  ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
  ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
	ADC_InitStructure.ChsNumber      = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_ConfigInjectedChannel(ADC1, ADC_CH_1, 1, ADC_SAMP_TIME_CYCLES_7_5);
	/* Enable ADC1 channel 1 positive end connecting to output of PGA1 positive end. */
	ADC_EnableCH1PositiveEndConnetPGA_P(ADC1, ENABLE);
	/* Enable ADC1 channel 1 negtive end connecting to output of PGA1 negtive end. */
	ADC_EnableCH1NegtiveEndConnetPGA_N(ADC1, ENABLE);
	/* Enable ADC1 channel 1 work on differential mode. */
	ADC_SetChannelSingleDiff(ADC1, ADC_CH_1, ADC_DIFFERENTIAL_ENDED);
  
  ADC_ConfigInjectedSequencerLength(ADC1, 1);
  ADC_ConfigExternalTrigInjectedConv(ADC1, ADC_EXT_TRIG_INJ_CONV_ATIM1_TRGO);
  ADC_EnableAutoInjectedConv(ADC1, DISABLE);
  ADC_EnableInjectedDiscMode(ADC1, DISABLE);

  ADC_Enable(ADC1, DISABLE);
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  GPIO_InitType GPIO_InitStructure;
  ADC_InitType ADC_InitStructure;
  DMA_InitType DMA_InitStructure;

  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_ADC2 | RCC_AHB_PERIPHEN_GPIOB | RCC_AHB_PERIPHEN_GPIOC, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
  RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_DMA1, ENABLE);

  GPIO_InitStructure.Pin       = VBUS_AD_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
  GPIO_InitPeripheral(VBUS_AD_GPIO_Port, &GPIO_InitStructure); 

  GPIO_InitStructure.Pin       = MOS_TEMP_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
  GPIO_InitPeripheral(MOS_TEMP_GPIO_Port, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin       = MOT_TEMP_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
  GPIO_InitPeripheral(MOT_TEMP_GPIO_Port, &GPIO_InitStructure);

  DMA_StructInit(&DMA_InitStructure);
	DMA_DeInit(DMA1_CH1);
	DMA_InitStructure.PeriphAddr = (uint32_t)&ADC2->DAT;
	DMA_InitStructure.MemAddr = 0;
	DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;
	DMA_InitStructure.BufSize = 0;
	DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStructure.MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;
	DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_HALFWORD;
	DMA_InitStructure.CircularMode = DMA_MODE_CIRCULAR;
	DMA_InitStructure.Priority = DMA_PRIORITY_LOW;
	DMA_InitStructure.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(DMA1_CH1, &DMA_InitStructure);
	DMA_RequestRemap(DMA_REMAP_ADC2, DMA1_CH1, ENABLE);
	DMA_EnableChannel(DMA1_CH1, DISABLE);

  ADC_InitStruct(&ADC_InitStructure);
  ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
  ADC_InitStructure.MultiChEn      = ENABLE;
  ADC_InitStructure.ContinueConvEn = ENABLE;
  ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_REG_CONV_ATIM1_TRGO;
  ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
  ADC_InitStructure.ChsNumber      = 3;
  ADC_Init(ADC2, &ADC_InitStructure);

  ADC_ConfigRegularChannel(ADC2, ADC_CH_12, 1, ADC_SAMP_TIME_CYCLES_71_5);
  ADC_ConfigRegularChannel(ADC2, ADC_CH_15, 2, ADC_SAMP_TIME_CYCLES_71_5);
  ADC_ConfigRegularChannel(ADC2, ADC_CH_5, 3, ADC_SAMP_TIME_CYCLES_71_5);
  ADC_EnableDiscMode(ADC2, DISABLE);
  ADC_SetRegularTriggerEdge(ADC2, ADC_REG_TRIG_EXT_FALLING);
  // ADC_SetDMATransferMode(ADC2, ADC_MULTI_REG_DMA_EACH_ADC);

  ADC_ConfigInjectedChannel(ADC2, ADC_CH_1, 1, ADC_SAMP_TIME_CYCLES_7_5);
	/* Enable ADC2 channel 1 positive end connecting to output of PGA1 positive end. */
	ADC_EnableCH1PositiveEndConnetPGA_P(ADC2, ENABLE);
	/* Enable ADC2 channel 1 negtive end connecting to output of PGA1 negtive end. */
	ADC_EnableCH1NegtiveEndConnetPGA_N(ADC2, ENABLE);
	/* Enable ADC2 channel 1 work on differential mode. */
	ADC_SetChannelSingleDiff(ADC2, ADC_CH_1, ADC_DIFFERENTIAL_ENDED);

  ADC_ConfigInjectedSequencerLength(ADC2, 1);
  ADC_ConfigExternalTrigInjectedConv(ADC2, ADC_EXT_TRIG_INJ_CONV_ATIM1_TRGO);
  ADC_EnableAutoInjectedConv(ADC2, DISABLE);
  ADC_EnableInjectedDiscMode(ADC2, DISABLE);

  ADC_Enable(ADC2, DISABLE);
}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_InitType ADC_InitStructure;

  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_ADC3, ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

  ADC_InitStruct(&ADC_InitStructure);
  ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
	/* ADC3 only uses the injected group, like ADC1. */
	ADC_InitStructure.MultiChEn      = DISABLE;
	ADC_InitStructure.ContinueConvEn = DISABLE;
  ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
  ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
	ADC_InitStructure.ChsNumber      = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  ADC_ConfigInjectedChannel(ADC3, ADC_CH_1, 1, ADC_SAMP_TIME_CYCLES_7_5);
	/* Enable ADC3 channel 1 positive end connecting to output of PGA1 positive end. */
	ADC_EnableCH1PositiveEndConnetPGA_P(ADC3, ENABLE);
	/* Enable ADC3 channel 1 negtive end connecting to output of PGA1 negtive end. */
	ADC_EnableCH1NegtiveEndConnetPGA_N(ADC3, ENABLE);
	/* Enable ADC3 channel 1 work on differential mode. */
	ADC_SetChannelSingleDiff(ADC3, ADC_CH_1, ADC_DIFFERENTIAL_ENDED);

  ADC_ConfigInjectedSequencerLength(ADC3, 1);
  ADC_ConfigExternalTrigInjectedConv(ADC3, ADC_EXT_TRIG_INJ_CONV_ATIM1_TRGO);
  ADC_EnableAutoInjectedConv(ADC3, DISABLE);
  ADC_EnableInjectedDiscMode(ADC3, DISABLE);

  ADC_Enable(ADC3, DISABLE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
