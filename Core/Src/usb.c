/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file hw_config.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
/* Includes ------------------------------------------------------------------*/

#include "usbfsd_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/
extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configures USB Clock input (48MHz)
 */
void MX_USB_Init(void)
{
    NVIC_InitType NVIC_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    GPIO_InitType GPIO_InitStructure;

    RCC_ConfigUSBFSClk(RCC_USBFS_CLKSRC_PLLPRES);
    RCC_ConfigUSBPLLPresClk(RCC_USBPLLCLK_SRC_PLL, RCC_USBPLLCLK_DIV5);
    RCC->CFG3 |= RCC_CFG3_USBFSTM;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USBFS, ENABLE);

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA, ENABLE);
    
    GPIO_InitStruct(&GPIO_InitStructure);
  
    GPIO_InitStructure.Pin              = USB_DP_Pin | USB_DM_Pin;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = GPIO_AF10;
    GPIO_InitPeripheral(USB_GPIO_Port, &GPIO_InitStructure);

    /* Enable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_FS_LP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_FS_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClrITPendBit(EXTI_LINE18);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE18; 
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    USB_Init();
}

/**
 * @brief  Power-off system clocks and power while entering suspend mode
 */
void Enter_LowPowerMode(void)
{
    /* Set the device state to suspend */
    bDeviceState = SUSPENDED;
}

/**
 * @brief  Restores system clocks and power while exiting suspend mode
 */
void Leave_LowPowerMode(void)
{
    USB_DeviceMess *pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if (pInfo->CurrentConfiguration != 0)
    {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
    else
    {
        bDeviceState = ATTACHED;
    }
}

/**
 * @brief  send one packet to the PC through a bulk IN endpoint
 * @param  ptrBuffer: send data pointer.
 * @param  Send_length: send data length (must be < 64; callers use <= 32).
 * @param  endpoint_num: ENDP1 (CDC) or ENDP3 (fibre), both configured in Reset.
 */
void USB_Send_DATA(uint8_t *ptrBuffer, uint8_t Send_length, uint8_t endpoint_num)
{
    uint16_t n = (Send_length >= VIRTUAL_COM_PORT_DATA_SIZE)
                     ? VIRTUAL_COM_PORT_DATA_SIZE - 1 : Send_length;
    USB_CopyUserToPMABuf((unsigned char*)ptrBuffer, USB_GetEpTxAddr(endpoint_num), n);
    USB_SetEpTxCnt(endpoint_num, n);
    USB_SetEpTxValid(endpoint_num);
}

/**
 * @brief  receive the data from the PC to N32G45x and send it through USB
 */
void USB_Receive_DATA(uint8_t endpoint_num)
{
    USB_SetEpRxValid(endpoint_num); 
}


