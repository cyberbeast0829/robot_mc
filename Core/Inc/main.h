/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "n32h47x_48x.h"
#include "n32h47x_48x_conf.h"
#include "hw_config.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* 固件版本。**整数三元组是唯一真源**：endpoint 的 system.fw_version_*、
 * CYBERBEAST 的 QUERY_DEVICE_INFO、启动横幅、legacy CAN 的版本变量都由它派生，
 * 不再单独维护一个 float 版本号（旧的 VERSION_NUM / system.version 已删除）。
 * 发版时只改这里 + docs/README.md 的版本矩阵 + 发布说明。 */
#define FW_VERSION_MAJOR		1
#define FW_VERSION_MINOR		0
#define FW_VERSION_REVISION		5
/* 1 = 非正式发布（带 -dirty / -pre 后缀的构建）；正式版必须为 0。
 * 对应 ODrive 的 fw_version_unreleased，由 system.fw_version_unreleased 上报。 */
#define FW_VERSION_UNRELEASED	0
/* minor 占百分位（minor < 100）：legacy CAN 的版本变量是 float32，编码成
 * major + 0.01*minor（本版 2.3.0 → 2.03，读作"V2.03"）。用 0.1 的话 minor 只占
 * 十分位，minor ≥ 10 就会进位吃掉整数部分（2.10 → 3.0），所以只能用 0.01。
 * 修订号不进 float（float32 只有 ~7 位有效数字，塞三段会失真），需要时读
 * system.fw_version_revision。 */
_Static_assert(FW_VERSION_MINOR < 100, "legacy CAN 的 float 版本号只能容纳两位 minor");
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void myprintf(const char *fmt, ...);
void console_write_raw(const uint8_t *buf, uint32_t len);
void Error_Handler(void);
/* 芯片 96 位唯一 ID 的低 64 位（system.serial_number / CYBERBEAST 设备信息帧） */
uint64_t board_serial_number(void);

/* USER CODE BEGIN EFP */
void fault_shutdown(void);		// gate drive off + PWM outputs off, register writes only
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// #define B1_Pin GPIO_PIN_13
// #define B1_GPIO_Port GPIOC
// #define USART_TX_Pin GPIO_PIN_2
// #define USART_TX_GPIO_Port GPIOA
// #define USART_RX_Pin GPIO_PIN_3
// #define USART_RX_GPIO_Port GPIOA
// #define TMS_Pin GPIO_PIN_13
// #define TMS_GPIO_Port GPIOA
// #define TCK_Pin GPIO_PIN_14
// #define TCK_GPIO_Port GPIOA
// #define SWO_Pin GPIO_PIN_3
// #define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
