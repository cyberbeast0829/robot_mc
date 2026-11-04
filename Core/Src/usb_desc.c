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
 * @file usb_desc.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "usbfsd_lib.h"
#include "usb_desc.h"

/* USB Standard Device Descriptor */
const uint8_t Virtual_Com_Port_DeviceDescriptor[] =
{
    0x12,   /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,     /* bDescriptorType */
    0x00,
    0x02,   /* bcdUSB = 2.00 */
    0xEF,   /* bDeviceClass: Miscellaneous (composite device, IADs used) */
    0x02,   /* bDeviceSubClass: Common Class */
    0x01,   /* bDeviceProtocol: Interface Association Descriptor */
    0x40,   /* bMaxPacketSize0 */
    0x09,   /* idVendor (0x1209) */
    0x12,
    0x42,   /* idProduct = 0x4342 ("CB" = CyberBeast) */
    0x43,
    0x00,
    0x03,   /* bcdDevice = 2.00 */
    1,              /* Index of string descriptor describing manufacturer */
    2,              /* Index of string descriptor describing product */
    3,              /* Index of string descriptor describing the device's serial number */
    0x01    /* bNumConfigurations */
};

/* Configuration descriptor.
 *
 * Composite device, mirroring the ODrive/N32 layout:
 *   interface 0 + 1 : CDC (virtual COM port, ASCII menu + 0xAA endpoint frames)
 *   interface 2     : vendor specific, fibre-style endpoint protocol
 *
 * The CDC half keeps the Nations VCP example's endpoint assignment bit for bit
 * (data IN = EP1, data OUT = EP3, notification = EP2): that is the shape verified
 * on this board.  The fibre interface uses the opposite directions of the same
 * endpoint numbers (EP1 OUT / EP3 IN), so adding it does not disturb the CDC
 * pipes.  Both functions are described by an IAD because bDeviceClass = 0xEF.
 *
 * Note: the earlier composite attempt used the ODrive VID/PID 0x1209/0x0D32, for
 * which Windows already had a driver bound; it enumerated a COM port but never
 * started the data pipes.  Our own PID (0x4342) has no such history - see
 * docs/DEV.md for the Windows/WinUSB setup of the vendor interface. */
const uint8_t Virtual_Com_Port_ConfigDescriptor[] =
{
    /*Configuration Descriptor*/
    0x09,   /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
    VIRTUAL_COM_PORT_SIZ_CONFIG_DESC,       /* wTotalLength:no of returned bytes */
    0x00,
    0x03,   /* bNumInterfaces: 3 interface */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
    0xC0,   /* bmAttributes: self powered */
    0x32,   /* MaxPower 100 mA */

    /*Interface Association Descriptor: CDC (virtual COM port)*/
    0x08,   /* bLength */
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,
    0x00,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x02,   /* bFunctionClass: Communication Interface Class */
    0x02,   /* bFunctionSubClass: Abstract Control Model */
    0x01,   /* bFunctionProtocol: Common AT commands */
    0x04,   /* iFunction: "CyberBeast Virtual Port" */

    /*Interface Descriptor (CDC communication class interface = interface 0)*/
    0x09,   /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x06,   /* iInterface: "Protocol" */
    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,
    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */
    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */
    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */
    /*Endpoint 2 Descriptor (notification, IN)*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x82,   /* bEndpointAddress: (IN2) */
    0x03,   /* bmAttributes: Interrupt */
    VIRTUAL_COM_PORT_INT_SIZE,      /* wMaxPacketSize: */
    0x00,
    0x10,   /* bInterval: */

    /*Data class interface descriptor (interface 1)*/
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC Data */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */
    /*Endpoint 3 Descriptor (data OUT: host -> device)*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x03,   /* bEndpointAddress: (OUT3) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: ignore for Bulk transfer */
    /*Endpoint 1 Descriptor (data IN: device -> host)*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x81,   /* bEndpointAddress: (IN1) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: ignore for Bulk transfer */

    /*Interface Association Descriptor: vendor specific (fibre)*/
    0x08,   /* bLength */
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,
    0x02,   /* bFirstInterface */
    0x01,   /* bInterfaceCount */
    0xFF,   /* bFunctionClass: Vendor specific */
    0xFF,   /* bFunctionSubClass */
    0xFF,   /* bFunctionProtocol */
    0x05,   /* iFunction: "CyberBeast Motor Driver Device" */

    /*Vendor interface descriptor (interface 2, fibre)*/
    0x09,   /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
    0x02,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0xFF,   /* bInterfaceClass: Vendor specific */
    0x01,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x07,   /* iInterface: "CBA Series" */
    /*Endpoint 1 Descriptor (fibre OUT: host -> device)*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x01,   /* bEndpointAddress: (OUT1) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: ignore for Bulk transfer */
    /*Endpoint 3 Descriptor (fibre IN: device -> host)*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x83,   /* bEndpointAddress: (IN3) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00    /* bInterval: ignore for Bulk transfer */
};

/* USB String Descriptors */
const uint8_t Virtual_Com_Port_StringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04 /* LangID = 0x0409: U.S. English */
};

const uint8_t Virtual_Com_Port_StringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_VENDOR,     /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,             /* bDescriptorType*/
    /* Manufacturer: "CYBERBEAST" */
    'C', 0, 'Y', 0, 'B', 0, 'E', 0, 'R', 0, 'B', 0, 'E', 0, 'A', 0,
    'S', 0, 'T', 0
};

const uint8_t Virtual_Com_Port_StringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    /* Product name: "CyberBeast CBA" */
    'C', 0, 'y', 0, 'b', 0, 'e', 0, 'r', 0, 'B', 0, 'e', 0, 'a', 0, 
    's', 0, 't', 0, ' ', 0, 'C', 0, 'B', 0, 'A', 0
};

uint8_t Virtual_Com_Port_StringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
    'C', 0, 'B', 0, 'A', 0, '.', 0, 'C', 0, 'y', 0, 'b', 0, 'e', 0, 
    'r', 0, 'B', 0, 'e', 0, 'a', 0, 's', 0, 't', 0, '.', 0, 'c', 0, 
    'n', 0
};

uint8_t Virtual_Com_Port_StringComIntfc[VIRTUAL_COM_PORT_SIZ_STRING_COM_INTFC] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_COM_INTFC,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
    'C', 0, 'y', 0, 'b', 0, 'e', 0, 'r', 0, 'B', 0, 'e', 0, 'a', 0,
    's', 0, 't', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
    'a', 0, 'l', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0
};

uint8_t Virtual_Com_Port_StringNativeIntfc[VIRTUAL_COM_PORT_SIZ_STRING_NATIVE_INTFC] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_NATIVE_INTFC,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
    'C', 0, 'y', 0, 'b', 0, 'e', 0, 'r', 0, 'B', 0, 'e', 0, 'a', 0,
    's', 0, 't', 0, ' ', 0, 'M', 0, 'o', 0, 't', 0, 'o', 0, 'r', 0,
    ' ', 0, 'D', 0, 'r', 0, 'i', 0, 'v', 0, 'e', 0, 'r', 0, ' ', 0,
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0
};

uint8_t Virtual_Com_Port_StringComIntfcAbbr[VIRTUAL_COM_PORT_SIZ_STRING_COM_INTFC_ABBR] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_COM_INTFC_ABBR,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
    'P', 0, 'r', 0, 'o', 0, 't', 0, 'o', 0, 'c', 0, 'o', 0, 'l', 0
};

uint8_t Virtual_Com_Port_StringNativeIntfcAbbr[VIRTUAL_COM_PORT_SIZ_STRING_NATIVE_INTFC_ABBR] =
{
    VIRTUAL_COM_PORT_SIZ_STRING_NATIVE_INTFC_ABBR,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
    'C', 0, 'B', 0, 'A', 0, ' ', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 
    'e', 0, 's', 0
};

