/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "n32h47x_48x_flash.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 2.0f


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;

/* init but don't allocate calibration arrays */
int *error_array = NULL;
int *lut_array = NULL;

uint8_t Serial2RxBuffer[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  // 设置NVIC的priority方式
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  // MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  // MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  /* Load settings from flash */
  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);

  /* Sanitize configs in case flash is empty*/
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=0;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}
  if(isnan(GR) || GR==-1){GR = 1.0f;}
  if(isnan(KT) || KT==-1){KT = 1.0f;}
  if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
  if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
  if(isnan(P_MAX)){P_MAX = 12.5f;}
  if(isnan(P_MIN)){P_MIN = -12.5f;}
  if(isnan(V_MAX)){V_MAX = 65.0f;}
  if(isnan(V_MIN)){V_MIN = -65.0f;}

  printf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);

  /* Controller Setup */
  if(PHASE_ORDER){							// Timer channel to phase mapping

  }
  else{

  }

  init_controller_params(&controller);

  /* calibration "encoder" zeroing */
  memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));

  /* commutation encoder setup */
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;

  if(EN_ENC_LINEARIZATION){memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));}	// Copy the linearization lookup table
  else{memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));}
  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on

  //for(int i = 0; i<128; i++){printf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs */
  ADC_Enable(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) == RESET);
  ADC_CalibrationOperation(ADC1, ADC_CALIBRATION_SINGLE_MODE);
  while (ADC_GetCalibrationStatus(ADC1, ADC_CALIBRATION_SINGLE_MODE));
  ADC_Enable(ADC2, ENABLE);
  while(ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY) == RESET);
  ADC_CalibrationOperation(ADC2, ADC_CALIBRATION_SINGLE_MODE);
  while (ADC_GetCalibrationStatus(ADC2, ADC_CALIBRATION_SINGLE_MODE));

  /* DRV8323 setup */
  GPIO_WriteBits(DRV_CS, Bit_SET ); 	// CS high
  // GPIO_WriteBits(ENABLE_PIN, Bit_SET );
  // HAL_Delay(1);
  //drv_calibrate(drv);
  // HAL_Delay(1);
  // drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  // HAL_Delay(1);
  // int CSA_GAIN;
  // if(I_MAX <= 40.0f){CSA_GAIN = CSA_GAIN_40;}	// Up to 40A use 40X amplifier gain
  // else{CSA_GAIN = CSA_GAIN_20;}					// From 40-60A use 20X amplifier gain.  (Make this generic in the future)
  // drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_0_25);
  // HAL_Delay(1);
  // drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_25);
  // HAL_Delay(1);
  zero_current(&controller);
  // HAL_Delay(1);
  // drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_0_45);
  // HAL_Delay(1);
  // drv_disable_gd(drv);
  // HAL_Delay(1);
  //drv_enable_gd(drv);   */
  printf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset);

  /* Turn on PWM */
  ATIM1->CCDAT1 = ATIM1->AR / 2;
  ATIM1->CCDAT2 = ATIM1->AR / 2;
  ATIM1->CCDAT3 = ATIM1->AR / 2;
  ATIM1->CCEN |= TIM_CCEN_CC1EN;
  ATIM1->CCEN |= TIM_CCEN_CC1NEN;
  ATIM1->CCEN |= TIM_CCEN_CC2EN;
  ATIM1->CCEN |= TIM_CCEN_CC2NEN;
  ATIM1->CCEN |= TIM_CCEN_CC3EN;
  ATIM1->CCEN |= TIM_CCEN_CC3NEN;

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  FDCAN_Start(FDCAN2); //start CAN
  //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can interrupt

  /* Set Interrupt Priorities */
  NVIC_SetPriority(PWM_ISR, 0x0); // commutation > communication
  NVIC_SetPriority(CAN_ISR, 0x01);

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;


  /* Turn on interrupts */
  USART_ConfigInt(USART3, USART_INT_RXDNE, ENABLE);
  USART_Enable(USART3, ENABLE);

  TIM_ClearFlag(ATIM1, TIM_FLAG_UPDATE);
  TIM_ClrIntPendingBit(ATIM1, TIM_INT_UPDATE);
  TIM_ConfigInt(ATIM1, TIM_INT_UPDATE, ENABLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // HAL_Delay(100);
	  // drv_print_faults(drv);
	 // if(state.state==MOTOR_MODE){
	  	  //printf("%.2f %.2f %.2f %.2f %.2f\r\n", controller.p_des, controller.v_des, controller.kp, controller.kd, controller.t_ff);
	  //}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  ErrorStatus HSEStartUpStatus;

  RCC_DeInit();
  RCC_ConfigLse(RCC_LSE_DISABLE);
  RCC_ConfigHse(RCC_HSE_ENABLE);

  HSEStartUpStatus = RCC_WaitHseStable();

  if (HSEStartUpStatus == SUCCESS) {
    // FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);
    // FLASH_SetLatency(FLASH_LATENCY_4);

    // HCLK = SYSCLK 240M
    RCC_ConfigHclk(RCC_SYSCLK_DIV1);
    // PCLK2 = HCLK/2 120M (APB2)
    RCC_ConfigPclk2(RCC_HCLK_DIV2);
    // PCLK1 = HCLK/4 60M (APB1)
    RCC_ConfigPclk1(RCC_HCLK_DIV4);
    // 24M HSE -> /4 x40 /1 -> 240M PLLCLK
    RCC_ConfigPll(RCC_PLL_SRC_HSE, RCC_PLL_PRE_4, RCC_PLL_MUL_40, RCC_PLLOUT_DIV_1);
    RCC_EnablePll(ENABLE);
    // wait till PLL is ready
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDF) == RESET);
    // SYSCLK = PLLCLK select PLL as system clock source
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLL);
    // wait till PLL is used as system clock source
    while (RCC_GetSysclkSrc() != RCC_CFG_SCLKSTS_PLL);
  }
  else {
    _Error_Handler(__FILE__, __LINE__);
  }

  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_Config(SystemCoreClock / 1000);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
