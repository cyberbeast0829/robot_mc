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



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF_H
#define __USB_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/

/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/

#define EP_NUM                          (4)

/*-------------------------------------------------------------*/
/* --------------   Buffer Description Table  -----------------*/
/*-------------------------------------------------------------*/
/* buffer table base address */
/* buffer table base address */
#define BTABLE_ADDRESS      (0x00)

/* EP0  */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x40)
#define ENDP0_TXADDR        (0x80)

/* EP1/EP2/EP3 buffer addresses.
 *
 * NOTE: these are the Nations VCP example's addresses (usbfsd_conf.h), not the
 * ODrive/STM32 ones this port started from.  In this layout EP1 is the CDC data
 * IN endpoint (device -> host), EP2 the notification endpoint and EP3 the CDC
 * data OUT endpoint (host -> device) - there is deliberately no EP1 RX and no
 * EP3 TX buffer, see Virtual_Com_Port_Reset().  Keeping the addresses identical
 * to the example also keeps the whole table inside the packet memory */
#define ENDP1_TXADDR        (0xC0)
#define ENDP2_TXADDR        (0x100)
#define ENDP3_RXADDR        (0x110)


/*-------------------------------------------------------------*/
/* -------------------   ISTR events  -------------------------*/
/*-------------------------------------------------------------*/
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CTRL_CTRSM  | CTRL_WKUPM | CTRL_ERRORM | CTRL_RSTM )
/* NOTE: CTRL_ERRORM is in the mask so that the ERR flag gets cleared by
 * USB_Istr() (the branch is compiled out without it) - a sticky error flag that
 * nobody clears is the one thing the vendor's mask handles and this one did not.
 * CTRL_SUSPDM/CTRL_SOFM/CTRL_ESOFM are deliberately left out: suspend handling
 * would set bDeviceState = SUSPENDED and make myprintf() fall back to USART3 */
//#define IMR_MSK    ( CTRL_CTRSM | CTRL_WKUPM | CTRL_SUSPDM | CTRL_ERRORM  | CTRL_SOFM \
                   | CTRL_ESOFM | CTRL_RSTM )

/*#define CTR_CALLBACK*/
/*#define DOVR_CALLBACK*/
/*#define ERR_CALLBACK*/
/*#define WKUP_CALLBACK*/
/*#define SUSP_CALLBACK*/
/*#define RESET_CALLBACK*/
/*#define SOF_CALLBACK*/
/*#define ESOF_CALLBACK*/

/* CTR service routines */
/* associated to defined endpoints */
/*#define  EP1_IN_Callback   USB_ProcessNop*/
#define  EP2_IN_Callback   USB_ProcessNop
//#define  EP3_IN_Callback   USB_ProcessNop
#define  EP4_IN_Callback   USB_ProcessNop
#define  EP5_IN_Callback   USB_ProcessNop
#define  EP6_IN_Callback   USB_ProcessNop
#define  EP7_IN_Callback   USB_ProcessNop

//#define  EP1_OUT_Callback   USB_ProcessNop
#define  EP2_OUT_Callback   USB_ProcessNop
//#define  EP3_OUT_Callback   USB_ProcessNop
#define  EP4_OUT_Callback   USB_ProcessNop
#define  EP5_OUT_Callback   USB_ProcessNop
#define  EP6_OUT_Callback   USB_ProcessNop
#define  EP7_OUT_Callback   USB_ProcessNop

#endif /* __USB_CONF_H */

