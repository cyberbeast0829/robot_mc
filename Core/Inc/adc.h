/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* 注入组结束标志的直读/直清（analog_sample() 每拍 6 次）。
 * ADC_GetFlagStatus()/ADC_ClearFlag() 是跨 TU 调用，而它们只是读 STS / 往 STS
 * 写掩码（写 1 清除，SDK 的 ADC_ClearFlag 也就是
 * `ADCx->STS = (flag & ADC_STS_ALL)`）。这里展开成同一组寄存器操作，判据与返回
 * 值完全相同（SET 当且仅当 STS 的对应位非 0）。 */
static inline int adc_jendc_set(ADC_Module * adc){ return (adc->STS & ADC_FLAG_JENDC) != 0u; }
static inline void adc_jendc_clear(ADC_Module * adc){ adc->STS = ADC_FLAG_JENDC; }

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
