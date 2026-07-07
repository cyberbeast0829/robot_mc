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
#include "pga.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "n32h47x_48x_flash.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "fsm.h"
#include "foc.h"
#include "math_ops.h"
#include "calibration.h"
#include "usb.h"
#include "usb_pwr.h"
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
FSMStruct state;
EncoderStruct comm_encoder;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;
CANRxMessage can_rx;

uint8_t Serial2RxBuffer[1];

// 参见https://www.eet-china.com/mp/a269236.html “Keil、IAR、CubeIDE中变量不被初始化方法”
// 下述方法仅限于用Keil+AC6
uint32_t _reboot_cookie __attribute__ ((section(".bss.NO_INIT")));
extern bool packet_sent;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stdint.h>

// int isnan(double x) {
//     union {
//         uint64_t u;
//         double f;
//     } ieee754;
//     ieee754.f = x;
//     return ((ieee754.u & 0x7FF0000000000000) == 0x7FF0000000000000) &&
//            ((ieee754.u & 0x000FFFFFFFFFFFFF) != 0);
// }

/* Internal SRAM config fo MCU */
#define SRAM_BASE_ADDR          (0x20000000)
#define SRAM_SIZE               (0x24000)

/* Constant for BOOT */
#define SRAM_VECTOR_WORD_SIZE   (128)
#define SRAM_VECTOR_ADDR        (SRAM_BASE_ADDR+SRAM_SIZE-0x200)
uint32_t BootAddr = 0x1FFF01a1,
         SPAddr = 0x200024a8,
         _mainAddr = 0x1FFF0189;   // BOOT V1.0 Version

typedef enum
{
    CMD_CR_SUCCESS  = 0x00,
    CMD_CR_FAILED   = 0x01,
}CMD_RETURN_CR;

CMD_RETURN_CR     (*Get_NVR)(uint32_t addr, uint8_t* data, uint32_t len) = (CMD_RETURN_CR (*)(uint32_t ,uint8_t*,uint32_t))0x1FFF02A5;

/* The pointer of function */
typedef void (*pFunction)(void);

static ErrorStatus SetSysClockToHSI(void)
{
    uint32_t timeout_value = 0xFFFFFFFF; 
    ErrorStatus ClockStatus;

    /* It is necessary to initialize the RCC peripheral to the reset state.*/
    RCC_DeInit();
    
    /* Enable HSI   */
    RCC_EnableHsi(ENABLE);
    /* Wait till HSI is ready */
    ClockStatus = RCC_WaitHsiStable();

    if (ClockStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);

        /* Flash 0 wait state */
        FLASH_SetLatency(FLASH_LATENCY_0);

        /* HCLK = SYSCLK */
        RCC_ConfigHclk(RCC_SYSCLK_DIV1);

        /* PCLK2 = HCLK */
        RCC_ConfigPclk2(RCC_HCLK_DIV1);

        /* PCLK1 = HCLK */
        RCC_ConfigPclk1(RCC_HCLK_DIV1);
        
        /* Restore HSI to PLL frequency division as default value */
        RCC->PLLCTRL |= 0x00040000;

        /* Select HSI as system clock source */
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_HSI);
           
        /* Wait till HSI is used as system clock source */
        while (RCC_GetSysclkSrc() != RCC_CFG_SCLKSTS_HSI)
        {
            if ((timeout_value--) == 0)
            {
                return ERROR;
            }
        }
    }
    else
    {
        /* HSI fails  */
        return ERROR;
    }
    return SUCCESS;
}

static void Jump_To_BOOT(void)
{
    uint8_t value1[8] = {0x00},value2 = 8;
    uint32_t i,*pVec;
    
    /* Init vector */
    pVec = (uint32_t *)SRAM_VECTOR_ADDR;
    for(i=0;i<SRAM_VECTOR_WORD_SIZE;i++)
        pVec[i] = 0;
    
    /* clean up ICache*/
    FLASH_iCacheCmd(DISABLE);
    FLASH_iCacheRST();
    
    /* Get BOOT Version */
    Get_NVR(0x1FFF37A8,value1,value2);
    
    if((value1[0] & 0xFF) == 0x11)    //BOOT V1.1 Version
    {
        SPAddr = 0x200024C0;
        pVec[SysTick_IRQn+16]           = 0x1FFF2F15;
        pVec[USART3_IRQn+16]            = 0x1FFF2F95;
        pVec[USB_FS_LP_IRQn+16]         = 0x1FFF3095;
    }
    else                           //BOOT V1.0 Version
    {
        pVec[SysTick_IRQn+16]           = 0x1FFF2d31;
        pVec[USART3_IRQn+16]            = 0x1FFF2db1;
        pVec[USB_FS_LP_IRQn+16]         = 0x1FFF2eb1;
    }
    /* Disable all interrupt */
    __disable_irq();
    
    /* Config IWDG */
    IWDG_ReloadKey();
    IWDG_WriteConfig(IWDG_WRITE_DISABLE);
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV256);
       
    /* Config system clock with HSI */ 
    SetSysClockToHSI();
    
    /* Set JumpBoot addr */
    pFunction JumpBoot = (pFunction)BootAddr;
    
    /* Initalize Stack Pointer */
    __set_MSP(SPAddr);
    
    /* Enable interrupt */
    __enable_irq();
    
    /* Initialize vector table */
    SCB->VTOR = SRAM_VECTOR_ADDR;

    /* Jump to BOOT */
    JumpBoot();
}

/**
 * @brief Carries out early startup tasks that need to run before any static
 * initializers.
 * This function gets called from the startup assembly code.
 */
void early_start_checks(void) {
  if(_reboot_cookie == 0xDEADFE75) {
      /* The STM DFU bootloader enables internal pull-up resistors on PB10 (AUX_H)
      * and PB11 (AUX_L), thereby causing shoot-through on the brake resistor
      * FETs and obliterating them unless external 3.3k pull-down resistors are
      * present. Pull-downs are only present on ODrive 3.5 or newer.
      * On older boards we disable DFU by default but if the user insists
      * there's only one thing left that might save it: time.
      * The brake resistor gate driver needs a certain 10V supply (GVDD) to
      * make it work. This voltage is supplied by the motor gate drivers which get
      * disabled at system reset. So over time GVDD voltage _should_ below
      * dangerous levels. This is completely handwavy and should not be relied on
      * so you are on your own on if you ignore this warning.
      *
      * This loop takes 5 cycles per iteration and at this point the system runs
      * on the internal 16MHz RC oscillator so the delay is about 2 seconds.
      */
      for (size_t i = 0; i < (16000000UL / 5UL * 2UL); ++i) {
          __NOP();
      }
      _reboot_cookie = 0xDEADBEEF;
  }

  /* We could jump to the bootloader directly on demand without rebooting
  but that requires us to reset several peripherals and interrupts for it
  to function correctly. Therefore it's easier to just reset the entire chip. */
  if(_reboot_cookie == 0xDEADBEEF) {
      _reboot_cookie = 0xCAFEFEED;  //Reset bootloader trigger
      Jump_To_BOOT();
  }

  /* The bootloader might fail to properly clean up after itself,
  so if we're not sure that the system is in a clean state we
  just reset it again */
  if(_reboot_cookie != 42) {
      _reboot_cookie = 42;
      NVIC_SystemReset();
  }
}

// static void watchdog_init(void){
// 	/* Independent watchdog, kicked by the commutation interrupt.  If the control
// 	 * loop ever stops running (wedged SPI, stuck ADC poll, crash), this resets
// 	 * the chip instead of leaving the PWM frozen with the gate drive enabled.
// 	 *
// 	 * Timeout is ~4 s nominal.  It can't be much shorter: saving preferences
// 	 * erases a 128 KB flash sector, which stalls the whole CPU (including the
// 	 * kicking interrupt) for up to ~2 s, and the LSI clock feeding the watchdog
// 	 * is loose (17-47 kHz across parts and temperature). */
// 	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;	// pause the watchdog while halted on a breakpoint
// 	IWDG->KR = 0x5555;								// enable register access
// 	IWDG->PR = 0x3;									// prescaler /32: ~1 kHz tick from the ~32 kHz LSI
// 	IWDG->RLR = 4000;								// ~4 s nominal timeout
// 	IWDG->KR = 0xAAAA;								// latch the reload value
// 	IWDG->KR = 0xCCCC;								// start the watchdog.  No way to stop it now
// }

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
  early_start_checks(); // check dfu status
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
  MX_PGA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  // MX_ADC3_Init();
  MX_USB_Init();
  /* USER CODE BEGIN 2 */

  /* Start the watchdog first, so a hang anywhere - including during init -
   * resets the chip.  The commutation interrupt kicks it once running;
   * all of init takes well under the ~4 s timeout */
  // watchdog_init();

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

  myprintf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);
  // if(RCC->CSR & RCC_CSR_IWDGRSTF){
	//   myprintf("!!! Last reset caused by the watchdog - control loop stopped running !!!\r\n");
  // }
  // __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Controller Setup */
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

  //for(int i = 0; i<128; i++){myprintf("%d\r\n", comm_encoder.offset_lut[i]);}

  /* Turn on ADCs */
  ADC_Enable(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) == RESET);
  ADC_CalibrationOperation(ADC1, ADC_CALIBRATION_SINGLE_MODE);
  while (ADC_GetCalibrationStatus(ADC1, ADC_CALIBRATION_SINGLE_MODE));
  ADC_Enable(ADC2, ENABLE);
  while(ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY) == RESET);
  ADC_CalibrationOperation(ADC2, ADC_CALIBRATION_SINGLE_MODE);
  while (ADC_GetCalibrationStatus(ADC2, ADC_CALIBRATION_SINGLE_MODE));

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
  // TIM_EnableCtrlPwmOutputs(ATIM1, ENABLE); // 打开PWM输出使能
  DBG_ConfigPeriph(DBG_ATIM1_STOP, ENABLE);

  controller.dtc_u = 0.0f;
  controller.dtc_v = 0.0f;
  controller.dtc_w = 0.0f;
  set_dtc(&controller);				// command all phases low before the gate driver wakes up

  /* DRV8323 setup */
  // GPIO_WriteBits(DRV_CS, Bit_SET ); 	// CS high
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
  myprintf("ADC A OFFSET: %d     ADC B OFFSET: %d\r\n", controller.adc_a_offset, controller.adc_b_offset);

  /* Set Interrupt Priorities */
  NVIC_SetPriority(PWM_ISR, 1); // commutation > communication
  NVIC_SetPriority(CAN_ISR, 2);
  NVIC_SetPriority(USB_ISR, 0);
  NVIC_SetPriority(USB_WKUP_ISR, 0);
  NVIC_SetPriority(USART_ISR, 3);

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  FDCAN_Start(FDCAN2); //start CAN.  RX is polled from the commutation interrupt
  //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can interrupt

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;
  state.print_pending = 1;	// print the menu on power-up


  /* Turn on interrupts */
  USART_ConfigInt(USART3, USART_INT_RXDNE, ENABLE);

  TIM_ClearFlag(ATIM1, TIM_FLAG_UPDATE);
  TIM_ClrIntPendingBit(ATIM1, TIM_INT_UPDATE);
  TIM_ConfigInt(ATIM1, TIM_INT_UPDATE, ENABLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Slow state machine work: printing, flash saves.  All of it was moved
	   * out of the interrupts so it can't stall commutation */
	  run_fsm_slow(&state);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static uint8_t usb_rx_buffer[64];
// USB接收数据处理函数
void usb_rx_process_packet(uint16_t bufaddr, uint32_t len, uint8_t endpoint_pair) {
  USB_CopyPMAToUserBuf(usb_rx_buffer, bufaddr, len);
  for (uint8_t i = 0; i < len; i++) {
    update_fsm(&state, (char)usb_rx_buffer[i]);
  }
  USB_Receive_DATA(endpoint_pair);
}

// 映射printf到串口
// int fputc(int ch, FILE *stream) {
//   if (bDeviceState == CONFIGURED) {
//     uint8_t dat = (uint8_t)ch;
//     USB_Send_DATA((uint8_t *)&dat, 1, ENDP1);
//   } else { 
//     USART_SendData(USART3, ch);
//     while (USART_GetFlagStatus(USART3, USART_FLAG_TXC) == RESET);
//   }
//   return ch;
// }

char printf_buf[256];
void myprintf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(printf_buf, sizeof(printf_buf), fmt, args);
  if (bDeviceState == CONFIGURED) {
    int bytes_to_be_sent = strlen(printf_buf);
    uint8_t *send_ptr = (uint8_t *)printf_buf;
    while (bytes_to_be_sent > 0) {
      int bytes_to_send = (bytes_to_be_sent > 32) ? 32 : bytes_to_be_sent; // 每次发送32字节
      packet_sent = false;
      USB_Send_DATA(send_ptr, bytes_to_send, ENDP1);
      int timeout_cnt = 0;
      while (!packet_sent) {
        for (int i = 0; i < 1000; i++) {
          __NOP();
        }
        if (timeout_cnt++ > 10000) {
          break; // 超时退出
        }
      }
      bytes_to_be_sent -= bytes_to_send;
      send_ptr += bytes_to_send;
    }
    packet_sent = false;
  } else {
    for (int i = 0; i < strlen(printf_buf); i++) {
      USART_SendData(USART3, printf_buf[i]);
      while (USART_GetFlagStatus(USART3, USART_FLAG_TXC) == RESET);
    }
  }
  va_end(args);
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
    Error_Handler();
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
  /* Get the power stage into a safe state, then wait.  If the watchdog is
   * running it will reset the chip; if this happens during init (before the
   * watchdog starts), hang here with the gate drive off. */
  __disable_irq();
  fault_shutdown();
  while (1)
  {
  }
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
     tex: myprintf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
