/**
  ******************************************************************************
  * @file    usbd_hid.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Joystick
  *             - Collection : Application
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_ctlreq.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
//#include "stm32f4xx_hal.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_HID
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_HID_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_HID_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_HID_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef  *pdev);
static uint8_t USBD_HID_EP0_TxSent(USBD_HandleTypeDef  *pdev);
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);

static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);

/**
  * @}
  */

/** @defgroup USBD_HID_Private_Variables
  * @{
  */

uint8_t hid_setup_ran = 0;
uint8_t dataout_ran = 0;
uint8_t usb_failed = 0;
uint8_t rumble_brequest_sent = 0;

uint8_t USBD_HID_Report_ID = 0;
uint8_t USBD_HID_Report_LENGTH = 0;
uint8_t ctl_report_buf[6] = {0};
uint8_t rumble_flag = 0;
uint8_t rx_buf[HID_EPOUT_SIZE];

char caller_str[100];

USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  USBD_HID_EP0_TxSent,              /* EP0_TxSent */
  USBD_HID_EP0_RxReady,              /* EP0_RxReady */
  USBD_HID_DataIn,   /* DataIn */
  USBD_HID_DataOut,              /* DataOut To get Rumble Data */
  NULL,              /* SOF */
  NULL,
  NULL,
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,

  USBD_HID_GetOtherSpeedCfgDesc,

  USBD_HID_GetDeviceQualifierDesc,
};

#if PC_SETUP
/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                               /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
  0x00,
  0x01,                                               /* bNumInterfaces: 1 interface */
  0x01,                                               /* bConfigurationValue: Configuration value */
  0x00,                                               /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
  0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER,                                     /* MaxPower 100 mA: this current is used for detecting Vbus */

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,                                               /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
  0x00,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints */
  0x03,                                               /* bInterfaceClass: HID */
  0x00,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x04,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0,                                                  /* iInterface: Index of string descriptor */
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/

  HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
  0x03,                                               /* bmAttributes: Interrupt endpoint */
  HID_EPIN_SIZE,                                      /* wMaxPacketSize: 4 Byte max */
  0x00,
  HID_FS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 34 */
};
#endif

#if OG_XBOX_SETUP
/* USB HID device FS Configuration Descriptor */
/* From OGX360 Project + My Duke Controller, data is slightly different than OGX360 Project*/
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
	//Configuration Descriptor//
	0x09,       //bLength of config descriptor
	0x02,       //bDescriptorType, 2=Configuration Descriptor
	0x20, 0x00, //wTotalLength 2-bytes, total length (including interface and endpoint descriptors)
	0x01,       //bNumInterfaces, just 1
	0x01,       //bConfigurationValue
	0x00,       //iConfiguration - index to string descriptors. we dont use them
	0x80,       //bmAttributes - 0x80 = USB Bus Powered
	0x32,       //bMaxPower - maximum power in 2mA units. 0xFA=500mA. Genuine OG controller is normally 100mA (0x32)

	//Interface Descriptor//
	0x09, //bLength of interface descriptor
	0x04, //bDescriptorType, 4=Interface  Descriptor
	0x00, //bInterfaceNumber
	0x00, //bAlternateSetting
	0x02, //bNumEndpoints - we have two endpoints (IN for button presses, and OUT for rumble values)
	0x58, //bInterfaceClass - From OG Xbox controller
	0x42, //bInterfaceSubClass - From OG Xbox controller
	0x00, //bInterfaceProtocol
	0x00, //iInterface - index to string descriptors. we dont use them

	//Endpoint Descriptor (IN)//
	0x07,       //bLength of endpoint descriptor
	0x05,       //bDescriptorType, 5=Endpoint Descriptor
	0x81,       //bEndpointAddress, Address=1, Direction IN
	0x03,       //bmAttributes, 3=Interrupt Endpoint
	0x20, 0x00, //wMaxPacketSize
	0x04,       //bInterval, Interval for polling the interrupt endpoint. 4ms

	//Endpoint Descriptor (OUT)//
	0x07,       //bLength of endpoint descriptor
	0x05,       //bDescriptorType, 5=Endpoint Descriptor
	0x02,       //bEndpointAddress, Address=2, Direction OUT
	0x03,       //bmAttributes, 3=Interrupt Endpoint
	0x20, 0x00, //wMaxPacketSize
	0x04        //bInterval, Interval for polling the interrupt endpoint. 4ms
};
#endif

#if PC_SETUP
/* USB HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgHSDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                               /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
  0x00,
  0x01,                                               /* bNumInterfaces: 1 interface */
  0x01,                                               /* bConfigurationValue: Configuration value */
  0x00,                                               /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
  0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER,                                     /* MaxPower 100 mA: this current is used for detecting Vbus */

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,                                               /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
  0x00,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints */
  0x03,                                               /* bInterfaceClass: HID */
  0x00,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x04,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0,                                                  /* iInterface: Index of string descriptor */
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType: */

  HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
  0x03,                                               /* bmAttributes: Interrupt endpoint */
  HID_EPIN_SIZE,                                      /* wMaxPacketSize: 4 Byte max */
  0x00,
  HID_HS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 34 */
};
#endif

#if PC_SETUP
/* USB HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                               /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
  0x00,
  0x01,                                               /* bNumInterfaces: 1 interface */
  0x01,                                               /* bConfigurationValue: Configuration value */
  0x00,                                               /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
  0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER,                                     /* MaxPower 100 mA: this current is used for detecting Vbus */

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,                                               /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
  0x00,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints */
  0x03,                                               /* bInterfaceClass: HID */
  0x00,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x04,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0,                                                  /* iInterface: Index of string descriptor */
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType: */

  HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
  0x03,                                               /* bmAttributes: Interrupt endpoint */
  HID_EPIN_SIZE,                                      /* wMaxPacketSize: 4 Byte max */
  0x00,
  HID_FS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 34 */
};
#endif

/*This will not be used but filled anyway since the generated code created it*/
#if OG_XBOX_SETUP
/* USB HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
	//Configuration Descriptor//
	0x09,       //bLength of config descriptor
	0x02,       //bDescriptorType, 2=Configuration Descriptor
	0x20, 0x00, //wTotalLength 2-bytes, total length (including interface and endpoint descriptors)
	0x01,       //bNumInterfaces, just 1
	0x01,       //bConfigurationValue
	0x00,       //iConfiguration - index to string descriptors. we dont use them
	0x80,       //bmAttributes - 0x80 = USB Bus Powered
	0x32,       //bMaxPower - maximum power in 2mA units. 0xFA=500mA. Genuine OG controller is normally 100mA (0x32)

	//Interface Descriptor//
	0x09, //bLength of interface descriptor
	0x04, //bDescriptorType, 4=Interface  Descriptor
	0x00, //bInterfaceNumber
	0x00, //bAlternateSetting
	0x02, //bNumEndpoints - we have two endpoints (IN for button presses, and OUT for rumble values)
	0x58, //bInterfaceClass - From OG Xbox controller
	0x42, //bInterfaceSubClass - From OG Xbox controller
	0x00, //bInterfaceProtocol
	0x00, //iInterface - index to string descriptors. we dont use them

	//Endpoint Descriptor (IN)//
	0x07,       //bLength of endpoint descriptor
	0x05,       //bDescriptorType, 5=Endpoint Descriptor
	0x81,       //bEndpointAddress, Address=1, Direction IN
	0x03,       //bmAttributes, 3=Interrupt Endpoint
	0x20, 0x00, //wMaxPacketSize
	0x04,       //bInterval, Interval for polling the interrupt endpoint. 4ms

	//Endpoint Descriptor (OUT)//
	0x07,       //bLength of endpoint descriptor
	0x05,       //bDescriptorType, 5=Endpoint Descriptor
	0x02,       //bEndpointAddress, Address=2, Direction OUT
	0x03,       //bmAttributes, 3=Interrupt Endpoint
	0x20, 0x00, //wMaxPacketSize
	0x04        //bInterval, Interval for polling the interrupt endpoint. 4ms
};
#endif


#if PC_SETUP
/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
};
#endif

#if OG_XBOX_SETUP
/* USB HID device Configuration Descriptor XID */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[16] __ALIGN_END =
{
	    0x10,                                          //bLength - Length of report. 16 bytes
	    0x42,                                          //bDescriptorType - always 0x42
	    0x00, 0x01,                                    //bcdXid
	    0x01,                                          //bType - 1=Xbox Gamecontroller
	    0x02,                                          //bSubType, 0x02 = Gamepad S, 0x01 = Gamepad (Duke)
	    0x14,                                          //bMaxInputReportSize //HID Report from controller - 20 bytes
	    0x06,                                          //bMaxOutputReportSize - Rumble report from host - 6 bytes
	    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF //wAlternateProductIds
};
#endif

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


#if PC_SETUP
/* HID Report Descriptor */
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END = {
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x05,                    // USAGE (Game Pad)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	0x09, 0xbb,                    //   USAGE (Throttle)
	0x15, 0x80,                    //   LOGICAL_MINIMUM (-128)
	0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	0x09, 0xbb,                    //   USAGE (Throttle)
	0x15, 0x80,                    //   LOGICAL_MINIMUM (-128)
	0x25, 0x7f,                    //   LOGICAL_MAXIMUM (127)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
	0xa1, 0x00,                    //   COLLECTION (Physical)
	0x09, 0x30,                    //     USAGE (X)
	0x09, 0x31,                    //     USAGE (Y)
	0x09, 0x32,                    //     USAGE (Z)
	0x09, 0x33,                    //     USAGE (Rx)
	0x15, 0x80,                    //     LOGICAL_MINIMUM (-128)
	0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
	0x75, 0x08,                    //     REPORT_SIZE (8)
	0x95, 0x04,                    //     REPORT_COUNT (4)
	0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	0x05, 0x09,                    //     USAGE_PAGE (Button)
	0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
	0x29, 0x10,                    //     USAGE_MAXIMUM (Button 16)
	0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
	0x75, 0x01,                    //     REPORT_SIZE (1)
	0x95, 0x10,                    //     REPORT_COUNT (16)
	0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	0xc0,                          //     END_COLLECTION
	0xc0                           // END_COLLECTION
};
#endif

__ALIGN_BEGIN static uint8_t DUKE_HID_CAPABILITIES_IN[20] __ALIGN_END = {
	    0x00, //Always 0x00
	    0x14, //bLength - length of packet in bytes
	    0xFF,
	    0x00, //This byte is 0x00 because this particular byte is not used in the button report.
	    0xFF,
	    0xFF, 0xFF, 0xFF,
	    0xFF, 0xFF, 0xFF,
	    0xFF, 0xFF, 0xFF,
	    0xFF, 0xFF, 0xFF,
	    0xFF, 0xFF, 0xFF
};

__ALIGN_BEGIN static uint8_t DUKE_HID_CAPABILITIES_OUT[6] = {
	    0x00,                  //Always 0x00
	    0x06,                  //bLength - length of packet in bytes
	    0xFF, 0xFF, 0xFF, 0xFF //bits corresponding to the rumble bits. all 0xFF as they are used.
};


/**
  * @}
  */

/** @defgroup USBD_HID_Private_Functions
  * @{
  */

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
/* We have to add the dataout/ep_out function here to get USB pipe data out
 * This is only used with XBCD or THPS 2, every other XBOX game sends rumble through control requests */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_HID_HandleTypeDef *hhid;

  hhid = USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (hhid == NULL)
  {
	  //usb_failed = 1;
    pdev->pClassData = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassData = (void *)hhid;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_in[HID_EPIN_ADDR & 0xFU].bInterval = HID_HS_BINTERVAL;
    pdev->ep_out[HID_EPOUT_ADDR & 0xFU].bInterval = HID_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_in[HID_EPIN_ADDR & 0xFU].bInterval = HID_FS_BINTERVAL;
    pdev->ep_out[HID_EPOUT_ADDR & 0xFU].bInterval = HID_FS_BINTERVAL;
  }

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, HID_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 1U;

  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, HID_EPOUT_ADDR, USBD_EP_TYPE_INTR, HID_EPOUT_SIZE);
  pdev->ep_out[HID_EPOUT_ADDR & 0xFU].is_used = 1U;
  (void)USBD_LL_PrepareReceive(pdev, HID_EPOUT_ADDR,(uint8_t*)rx_buf, HID_EPOUT_SIZE);

  hhid->state = HID_IDLE;

  //((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->Init();

  /* Prepare Out endpoint to receive 1st packet */

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_DeInit
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close HID EPs */
  (void)USBD_LL_CloseEP(pdev, HID_EPIN_ADDR);
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].is_used = 0U;
  pdev->ep_in[HID_EPIN_ADDR & 0xFU].bInterval = 0U;

  /* Close CUSTOM_HID EP OUT */
  (void)USBD_LL_CloseEP(pdev, HID_EPOUT_ADDR);
  pdev->ep_out[HID_EPOUT_ADDR & 0xFU].is_used = 0U;
  pdev->ep_out[HID_EPOUT_ADDR & 0xFU].bInterval = 0U;

  /* Free allocated memory */
  if (pdev->pClassData != NULL)
  {
    (void)USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{


  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;

  if (hhid == NULL)
  {
	  usb_failed = 1;
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case HID_REQ_GET_PROTOCOL:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
          break;

        case HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case HID_REQ_GET_IDLE:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
          break;

        /*We need to get Control request data for the rumble data from the XBOX, we have to add this ourselves */
        case HID_REQ_SET_REPORT:
          rumble_flag = 1;
          USBD_HID_Report_ID = (uint8_t)(req->wValue);
          USBD_HID_Report_LENGTH = (uint8_t)(req->wLength);
          USBD_CtlPrepareRx (pdev, ctl_report_buf, HID_EPOUT_SIZE);
          break;

          /*Commented out below, this is if you wanted to send data through a USB control request
           * This is not needed here, but is good reference if you ever wanted to do so */
//        case HID_REQ_GET_REPORT:
//          flag = 1;
//          Report_buf[0] = 0x11;
//          Report_buf[1] = 0x22;
//          Report_buf[2] = 0x33;
//          Report_buf[3] = 0x44;
//          Report_buf[4] = 0x55;
//          Report_buf[5] = 0x66;
//          Report_buf[6] = 0x77;
//          Report_buf[7] = 0x88;
//          USBD_CtlSendData (pdev,
//                            (uint8_t *)&Report_buf,
//                            8);
//          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:

      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
#if PC_SETUP
          if ((req->wValue >> 8) == HID_REPORT_DESC)
          {
            len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_MOUSE_ReportDesc;
          }
#endif
           if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_Desc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          (void)USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    /*This is for the og XBOX, this is the custom vendor request
     * We intercept these and return either the controller's capabilities
     * This is equivalent to the USB HID Descriptor which would normally be used for most devices */
    case (0xC1 & USB_REQ_TYPE_MASK):
    	 hid_setup_ran++;
    	if(req->bRequest == 0x06 && req->wValue == 0x4200) {
    		len = 16;
    		pbuf = USBD_HID_Desc;
    		(void)USBD_CtlSendData(pdev, pbuf, len);
    	}
    	else if(req->bRequest == 0x01 && req->wValue == 0x0100) {
    		len = 20;
    		pbuf = DUKE_HID_CAPABILITIES_IN;
    		(void)USBD_CtlSendData(pdev, pbuf, len);
    	}
    	else if (req->bRequest == 0x01 && req->wValue == 0x0200) {
    		rumble_brequest_sent = 1;
    		len = 6;
    		pbuf = DUKE_HID_CAPABILITIES_OUT;
    		(void)USBD_CtlSendData(pdev, pbuf, len);
    	}
    break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

  if (hhid == NULL)
  {
	  //usb_failed = 1;
    return (uint8_t)USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == HID_IDLE)
    {
      hhid->state = HID_BUSY;
      (void)USBD_LL_Transmit(pdev, HID_EPIN_ADDR, report, len);
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_GetPollingInterval
  *         return polling interval from endpoint descriptor
  * @param  pdev: device instance
  * @retval polling interval
  */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
  uint32_t polling_interval;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Sets the data transfer polling interval for high speed transfers.
     Values between 1..16 are allowed. Values correspond to interval
     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
  }
  else   /* LOW and FULL-speed endpoints */
  {
    /* Sets the data transfer polling interval for low and full
    speed transfers */
    polling_interval =  HID_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

/**
  * @brief  USBD_HID_GetCfgFSDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_CfgFSDesc);

  return USBD_HID_CfgFSDesc;
}

/**
  * @brief  USBD_HID_GetCfgHSDesc
  *         return HS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length)
{
#if PC_SETUP
  *length = (uint16_t)sizeof(USBD_HID_CfgHSDesc);

  return USBD_HID_CfgHSDesc;
#endif
}

/**
  * @brief  USBD_HID_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */

static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_OtherSpeedCfgDesc);

  return USBD_HID_OtherSpeedCfgDesc;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassData)->state = HID_IDLE;

  return (uint8_t)USBD_OK;
}
/* DataOut will only run for THPS 2 or activating rumble in XBCD on a PC */
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	dataout_ran++;
	USBD_LL_PrepareReceive(pdev, HID_EPOUT_ADDR, (uint8_t*) (rx_buf), HID_EPOUT_SIZE);
	return USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
/*This is to put the rumble data from the USB control requests into rx_buf so we can read that in main
 * This is the standard way XBOX sends rumble data, this different from a PC which would send data through USB Out Pipe */
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
	if (rumble_flag) {
		rumble_flag = 0;
		if (USBD_HID_Report_LENGTH == HID_EPOUT_SIZE) {
			for (uint8_t i = 0; i < HID_EPOUT_SIZE; i++) {
				rx_buf[i] = ctl_report_buf[i];
			}
		}
	}
	return (uint8_t) USBD_OK;
}

/**
  * @brief  USBD_CUSTOM_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_HID_EP0_TxSent(USBD_HandleTypeDef *pdev)
{
	//USBD_CtlPrepareRx (pdev, rx_buf, 6);
//  USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef *)pdev->pClassData;
//
//  if (hhid->IsReportAvailable == 1U)
//  {
//    ((USBD_CUSTOM_HID_ItfTypeDef *)pdev->pUserData)->OutEvent(hhid->Report_buf[0],
//                                                              hhid->Report_buf[1]);
//    hhid->IsReportAvailable = 0U;
//  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{

  *length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
