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
#include "timing.h"
#include "usb.h"
#include "usb_pwr.h"
#include "endpoint_serial.h"
#include "prefs_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* VERSION_NUM moved to main.h so can_read_variable() can report it over CAN */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
FSMStruct state;
EncoderStruct comm_encoder;
CalStruct comm_encoder_cal;
CANTxMessage can_tx;

/* Latched in early_start_checks() (before RCC_DeInit could touch the flags) and
 * reported once myprintf() works - tells the user the previous run was killed
 * by the watchdog, i.e. the control loop had stopped running */
static uint8_t watchdog_reset_flag = 0;

// 参见https://www.eet-china.com/mp/a269236.html “Keil、IAR、CubeIDE中变量不被初始化方法”
// 下述方法仅限于用Keil+AC6
uint32_t _reboot_cookie __attribute__ ((section(".bss.NO_INIT")));

/* packet_sent is written from the USB interrupt (EP1_IN_Callback) and polled in
 * the main loop, so it must be volatile - without it a compiler is free to hoist
 * the load out of the wait loop and stall myprintf() for its whole timeout */
extern volatile bool packet_sent;
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
 * @brief Carries out early startup tasks that need to run before the clocks are
 * reconfigured.
 * NOTE: unlike upstream (where the startup assembly called it), this is called
 * from main() right after the C runtime has initialised .data/.bss and before
 * SystemClock_Config() - keep it there, the reset-cause latch below depends on
 * running before RCC_DeInit().
 */
void early_start_checks(void) {
  /* Latch (and clear) the reset cause here: this runs before SystemClock_Config
   * calls RCC_DeInit(), and the flags are sticky until they are cleared */
  watchdog_reset_flag = (RCC_GetFlagStatus(RCC_FLAG_IWDGRSTF) == SET);
  RCC_ClearResetFlag();

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

static void watchdog_init(void){
	/* Independent watchdog, kicked by the commutation interrupt.  If the control
	 * loop ever stops running (wedged SPI, stuck ADC poll, crash), this resets
	 * the chip instead of leaving the PWM frozen with the gate drive enabled.
	 *
	 * Timeout is ~4 s nominal: the ~32 kHz LSI through the /32 prescaler gives a
	 * 1 kHz tick, and RELV = 4000.  It must not be much shorter - zero_current()
	 * blocks for ~1.5 s at boot and saving preferences erases a flash page (the
	 * CPU, including the kicking interrupt, stalls for the whole erase), and the
	 * LSI is loose (17-47 kHz across parts and temperature). */
	DBG_ConfigPeriph(DBG_IWDG_STOP, ENABLE);		// pause the watchdog while halted at a breakpoint
	IWDG_WriteConfig(IWDG_WRITE_ENABLE);			// open PREDIV/RELV for writing
	IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV32);		// ~32 kHz LSI / 32 -> 1 kHz tick
	IWDG_CntReload(4000);							// ~4 s nominal timeout
	IWDG_ReloadKey();								// latch the reload value
	IWDG_Enable();									// start it - there is no way to stop it now
}

uint16_t adc_measurements_[3] = { 0 };

void start_general_purpose_adc() {
    // MOS_TEMP
    DMA_SetCurrDataCounter(DMA1_CH1, 1);
    DMA1_CH1->MADDR = (uint32_t)(&adc_measurements_[0]);
    DMA_EnableChannel(DMA1_CH1, ENABLE);

    // VBUS, MOT_TEMP
    DMA_SetCurrDataCounter(DMA1_CH2, 2);
    DMA1_CH2->MADDR = (uint32_t)(&adc_measurements_[1]);
    DMA_EnableChannel(DMA1_CH2, ENABLE);
}

/* ISR timing instrumentation (see timing.h).  Kept in RAM only - read it from
 * the debugger's watch window, there is no printing path for it */
volatile IsrTiming isr_timing;

void isr_timing_init(void){
	/* Enable the DWT cycle counter once; it then free-runs for the rest of the
	 * session.  These are the same three writes the SDK's CPU_DELAY_INTI() macro
	 * does, but its matching CPU_DELAY_DISABLE() clears TRCENA again (and
	 * __SYSTEM_DELAY_US() zeroes CYCCNT on the way in), so the counter cannot be
	 * set up with that macro.  It also has to run after the last
	 * __SYSTEM_DELAY_US(), i.e. after SetSysClock() has finished. */
	DEM_CR |= DEM_CR_TRCENA;		// enable the trace / DWT block
	DWT_CYCCNT = 0;
	DWT_CR |= DWT_CR_CYCCNTENA;		// start counting core cycles

	/* One PWM update period, from the timer as configured in tim.c: centre
	 * aligned, AR = 3000, REPCNT = 1 -> one update event every
	 * AR*(REPCNT+1) = 6000 cycles.  ATIM1 is clocked from SYSCLK and the DWT from
	 * HCLK, both 240 MHz here, so the two cycle counts compare directly */
	isr_timing.isr_period = ATIM1->AR*(ATIM1->REPCNT + 1u);
	isr_timing.period_last = DWT_CYCCNT;
	isr_timing.reset_req = 1;		// start the statistics on the first cycle
}

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
  MX_SPI1_Init();
  // MX_SPI3_Init();
  MX_PGA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USB_Init();
  /* USER CODE BEGIN 2 */

  /* Start the watchdog early, so a hang anywhere - including during the rest of
   * init - resets the chip.  The commutation interrupt kicks it once running;
   * the slowest thing before that is zero_current() (~1.5 s), well inside the
   * ~4 s timeout */
  watchdog_init();

  /* 默认值 -> 覆盖 flash 里的有效记录 -> 统一限幅。
   * 记录以 endpoint ID 为 key，所以新增参数缺记录时会保持这里写的默认值；
   * 不再需要 NaN/-1 哨兵，也不需要在这里逐个列默认值（都在 params.json）。 */
  config_apply_defaults();
  preference_writer_load(&prefs);
  config_apply_limits();

  myprintf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);
  if(watchdog_reset_flag){
	  myprintf("!!! Last reset was caused by the watchdog - the control loop had stopped running !!!\r\n");
  }

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

	/* ATIM1 is already running.  Keep both groups untriggered while the ADCs
	 * are calibrated; the triggers are restored after start_general_purpose_adc().
	 * ADC1 only uses the injected group, ADC2's regular group is MOS_TEMP and
	 * ADC3's regular group is VBUS + MOT_TEMP - so BOTH of those regular groups
	 * must be restored, or their DMA channels never finish a transfer and
	 * adc_measurements_[] stays 0 (see the note below). */
	ADC_ConfigExternalTrigRegularConv(ADC1, ADC_EXT_TRIG_REG_CONV_SOFTWARE);
	ADC_ConfigExternalTrigRegularConv(ADC2, ADC_EXT_TRIG_REG_CONV_SOFTWARE);
	ADC_ConfigExternalTrigRegularConv(ADC3, ADC_EXT_TRIG_REG_CONV_SOFTWARE);
	ADC_ConfigExternalTrigInjectedConv(ADC1, ADC_EXT_TRIG_INJ_CONV_SOFTWARE);
	ADC_ConfigExternalTrigInjectedConv(ADC2, ADC_EXT_TRIG_INJ_CONV_SOFTWARE);
	ADC_ConfigExternalTrigInjectedConv(ADC3, ADC_EXT_TRIG_INJ_CONV_SOFTWARE);

	/* Wait for ENCAL itself: the SDK status helper can report completion
	 * early when a calibration factor is already nonzero. */
	ADC_Enable(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) == RESET){;}
	ADC_CalibrationOperation(ADC1, ADC_CALIBRATION_DIFF_MODE);
	while(ADC1->CTRL2 & ADC_CTRL2_ENCAL){;}
	ADC_SetDMATransferMode(ADC1, ADC_MULTI_REG_DMA_EACH_ADC);
	ADC_Enable(ADC2, ENABLE);
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY) == RESET){;}
	ADC_CalibrationOperation(ADC2, ADC_CALIBRATION_SINGLE_MODE);
	while(ADC2->CTRL2 & ADC_CTRL2_ENCAL){;}
	/* ADC2's regular group is MOS_TEMP (PA7 / CH2), ADC3's is VBUS (PE7 / CH4)
	 * and MOT_TEMP (PE8 / CH6). */
	ADC_CalibrationOperation(ADC2, ADC_CALIBRATION_DIFF_MODE);
	while(ADC2->CTRL2 & ADC_CTRL2_ENCAL){;}
	ADC_SetDMATransferMode(ADC2, ADC_MULTI_REG_DMA_EACH_ADC);
	ADC_Enable(ADC3, ENABLE);
	while(ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY) == RESET){;}
	ADC_CalibrationOperation(ADC3, ADC_CALIBRATION_DIFF_MODE);
	while(ADC3->CTRL2 & ADC_CTRL2_ENCAL){;}
	ADC_SetDMATransferMode(ADC3, ADC_MULTI_REG_DMA_EACH_ADC);

	start_general_purpose_adc();
	ADC_ConfigExternalTrigInjectedConv(ADC1, ADC_EXT_TRIG_INJ_CONV_ATIM1_TRGO);
	ADC_ConfigExternalTrigInjectedConv(ADC2, ADC_EXT_TRIG_INJ_CONV_ATIM1_TRGO);
	ADC_ConfigExternalTrigInjectedConv(ADC3, ADC_EXT_TRIG_INJ_CONV_ATIM1_TRGO);
	/* Both regular groups are trigger-driven now (ADC2 = MOS_TEMP,
	 * ADC3 = VBUS/MOT_TEMP).  ADC3 used to be left on the software trigger set
	 * above and nothing ever software-started it, so VBUS never converted:
	 * adc_measurements_[1] stayed 0 -> v_bus = 0 -> v_max = 0 -> svm() clamped
	 * every duty cycle to the midpoint, i.e. no voltage at all.  The motor then
	 * does not turn in open loop and calibration reports "measured 0 pole
	 * pairs". */
	ADC_ConfigExternalTrigRegularConv(ADC2, ADC_EXT_TRIG_REG_CONV_ATIM1_TRGO);
	ADC_ConfigExternalTrigRegularConv(ADC3, ADC_EXT_TRIG_REG_CONV_ATIM1_TRGO);

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

  zero_current(&controller);

  controller.gate_drive_enabled = 0;
  myprintf("ADC OFFSETS: A %d  B %d  C %d    VBUS: %.1f V\r\n", controller.adc_a_offset, controller.adc_b_offset, controller.adc_c_offset, controller.v_bus);

  /* Set Interrupt Priorities.
   * NVIC_PriorityGroup_4 = 4 bits of preemption priority, and the lower the
   * number the higher the priority.  The 40 kHz commutation interrupt must not
   * be delayed by communications, so it is the highest; USB (which also parses
   * the CDC stream into the FSM) sits just below it, then CAN, then the serial
   * console. */
  NVIC_SetPriority(PWM_ISR, 0);			// commutation - highest
  NVIC_SetPriority(USB_ISR, 1);			// just below commutation
  NVIC_SetPriority(USB_WKUP_ISR, 1);
  NVIC_SetPriority(CAN_ISR, 2);
  NVIC_SetPriority(USART_ISR, 3);

  /* CAN setup.  The protocol (config.can_prot) and bitrate (config.can_baud) are
   * persisted, so the peripheral - timing, message RAM layout and acceptance
   * filters - can only be configured after the configuration has been loaded
   * above.  Both values take effect on power-up. */
  MX_CAN1_Init();
  can_tx_init(&can_tx);
  can_proto_init();
  //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING); // Start can interrupt

  /* Start the FSM */
  state.state = MENU_MODE;
  state.next_state = MENU_MODE;
  state.ready = 1;
  state.print_pending = 1;	// print the menu on power-up


  /* Turn on interrupts */
  USART_ConfigInt(USART3, USART_INT_RXDNE, ENABLE);

  /* Start the ISR timing instrumentation.  It has to be before the first
   * commutation interrupt, and after SystemClock_Config()/MX_TIM1_Init() - the
   * DWT counts HCLK and the PWM period is read from the timer */
  isr_timing_init();

  TIM_ClearFlag(ATIM1, TIM_FLAG_UPDATE);
  TIM_ClrIntPendingBit(ATIM1, TIM_INT_UPDATE);
  TIM_ConfigInt(ATIM1, TIM_INT_UPDATE, ENABLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* endpoint 请求的解析和应答（USB 中断只负责收帧）。放在打印/存 flash
	   * 之前，避免应答被菜单输出拖慢 */
	  endpoint_serial_poll();

      /* CAN 协议的慢活（目前是 CYBERBEAST 的 JSON 描述符流式发送）。
       * 收帧和心跳仍在 40 kHz 控制中断里 */
      can_proto_poll();
	  run_fsm_slow(&state);
    /* USER CODE END WHILE */
		// myprintf("ADC OFFSETS: A %d  B %d  C %d    VBUS: %.1f V\r\n", controller.adc_a_offset, controller.adc_b_offset, controller.adc_c_offset, controller.v_bus);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static uint8_t usb_rx_buffer[64];
// USB接收数据处理函数
void usb_rx_process_packet(uint16_t bufaddr, uint32_t len, uint8_t endpoint_pair) {
  USB_CopyPMAToUserBuf(usb_rx_buffer, bufaddr, len);
  for (uint8_t i = 0; i < len; i++) {
    /* 二进制 endpoint 帧和 ASCII 菜单共用同一条 CDC 数据流：feed() 吃掉
     * 属于帧的字节，剩下的才是菜单输入 */
    if (!endpoint_serial_feed(usb_rx_buffer[i])) {
      update_fsm(&state, (char)usb_rx_buffer[i]);
    }
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

/* 原始字节输出：USB CDC 已配置时走 USB，否则轮询 USART3。
 * myprintf() 和 endpoint 应答共用这一条路径。只能在主循环调用。 */
void console_write_raw(const uint8_t *buf, uint32_t len)
{
  if (bDeviceState == CONFIGURED) {
    uint8_t *send_ptr = (uint8_t *)buf;
    while (len > 0) {
      uint32_t bytes_to_send = (len > 32) ? 32 : len; // 每次发送32字节
      packet_sent = false;
      USB_Send_DATA(send_ptr, (uint8_t)bytes_to_send, ENDP1);
      int timeout_cnt = 0;
      while (!packet_sent) {
        for (int i = 0; i < 1000; i++) {
          __NOP();
        }
        if (timeout_cnt++ > 10000) {
          break; // 超时退出
        }
      }
      len -= bytes_to_send;
      send_ptr += bytes_to_send;
    }
    packet_sent = false;
  } else {
    for (uint32_t i = 0; i < len; i++) {
      USART_SendData(USART3, buf[i]);
      while (USART_GetFlagStatus(USART3, USART_FLAG_TXC) == RESET);
    }
  }
}

void myprintf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(printf_buf, sizeof(printf_buf), fmt, args);
  va_end(args);
  console_write_raw((const uint8_t *)printf_buf, (uint32_t)strlen(printf_buf));
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
