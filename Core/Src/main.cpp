/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"	//st library
#include "cmsis_os.h"	//st freertos
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <PS4BT.h>	//usb host shield library
#include <usbhub.h>	//usb host shield library
#include "usbd_hid.h" //st library
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define XBOX_DUP (1 << 0)
#define XBOX_DDOWN (1 << 1)
#define XBOX_DLEFT (1 << 2)
#define XBOX_DRIGHT (1 << 3)
#define XBOX_START_BTN (1 << 4)
#define XBOX_BACK_BTN (1 << 5)
#define XBOX_LS_BTN (1 << 6)
#define XBOX_RS_BTN (1 << 7)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim14;

/* Definitions for getBT */
osThreadId_t getBTHandle;
const osThreadAttr_t getBT_attributes = {
  .name = "getBT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendUSB */
osThreadId_t sendUSBHandle;
const osThreadAttr_t sendUSB_attributes = {
  .name = "sendUSB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controllerJoin */
osThreadId_t controllerJoinHandle;
const osThreadAttr_t controllerJoin_attributes = {
  .name = "controllerJoin",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for buttonPress */
osThreadId_t buttonPressHandle;
const osThreadAttr_t buttonPress_attributes = {
  .name = "buttonPress",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for updateLCD */
osThreadId_t updateLCDHandle;
const osThreadAttr_t updateLCD_attributes = {
  .name = "updateLCD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};

/* USER CODE BEGIN PV */
osThreadId_t getLatencies;
const osThreadAttr_t getLatencies_attributes = {
  .name = "getLatencies",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
void StartGetBT(void *argument);
void StartSendUSB(void *argument);
void StartControllerJoin(void *argument);
void StartButtonPress(void *argument);
void StartUpdateLCD(void *argument);
/* USER CODE BEGIN PFP */
void ProcessKeyCodeInContext(uint8_t keyCode);
void StartGetLatencies(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define PC_SETUP 1

/* Digital Button Masks */


SPI_HandleTypeDef SPI_Handle;
UART_HandleTypeDef UART_Handle;
SerialClass Serial(&huart2);

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
//PS4BT PS4(&Btd, PAIR);
static bool printAngle, printTouch;
static uint8_t oldL2Value, oldR2Value;
static bool buttonPressed;
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t rumble_once = 0;
uint8_t ps4_connected = 0;

typedef struct ps4ButtonsTag
{
 uint8_t dummy:1;
 uint8_t button_ps:1;
 uint8_t button_start:1;
 uint8_t button_share:1;
 uint8_t button_right_trigger:1;
 uint8_t button_left_trigger:1;
 uint8_t button_cross:1;
 uint8_t button_circle:1;

 uint8_t button_triangle:1;
 uint8_t button_square:1;
 uint8_t button_dpad_up:1;
 uint8_t button_dpad_right:1;
 uint8_t button_dpad_left:1;
 uint8_t button_dpad_down:1;
 uint8_t button_left_thumb:1;
 uint8_t button_right_thumb:1;

} PS4_CC_BUTTONS;

// HID Game

struct gameHID_t {

	int8_t Joy_LT;
	int8_t Joy_RT;
    int8_t JoyX; 	// X 1 byte, signed value
    int8_t JoyY; 	// Y 1 byte, signed value
    int8_t Joy2X;
    int8_t Joy2Y;

      PS4_CC_BUTTONS ps4ButtonsTag;
      	// Button, one byte, button is bit #0
};

/*Used temporarily for adjusting contorller offsets*/
uint8_t LeftHatX_val;
uint8_t LeftHatY_val;
uint8_t RightHatX_val;
uint8_t RightHatY_val;

/* Used for verifying CPU HCLK and Timer Functionality */
uint32_t cpu_freq = 0;
uint16_t timer_val = 0 ;
uint32_t hal_gettick = 0;

/* Display Controls */
uint8_t display_no = 0;

/*Deadzone control */
#define deadzone_enable 0
/* Buttons
 * PA8 = BACK (keyCode value of 3)
 * PB10 = SELECT (keyCode value of 5)
 * PB4 = FORWARD (keyCode value of 6) */
#define NO_BUTTON_PRESSED 0x07
#define BUTTON_PRESSED keyCode != NO_BUTTON_PRESSED
#define BACK_BTN_GPIO GPIOA, GPIO_PIN_8
#define SELECT_BTN_GPIO GPIOB, GPIO_PIN_10
#define FORWARD_BTN_GPIO GPIOB, GPIO_PIN_4
#define BACK_BTN 3
#define SELECT_BTN 5
#define FORWARD_BTN 6

uint8_t keyCode = NO_BUTTON_PRESSED;
uint8_t buttonDebounced = 0;
uint8_t buttonProcessed = 0;
uint8_t display_force_update = 0;

/* Display Settings */
uint8_t display_run_once = 0; //Used to update the screen

/* Debugging for freeRTOS */
#define rtos_delay_view 1 // Measure the delay of tasks
						  //Set to 2 for verbose
uint16_t timer_val_getBT = 0 ;
uint16_t timer_val_getUSB = 0 ;
uint16_t timer_val_LCD = 0 ;
/* Thanks to the OGX360 Project for the Byte Order */
struct xboxHID_t
{
    uint8_t startByte;
    uint8_t bLength;
    uint8_t dButtons;
    uint8_t reserved;
    uint8_t A;
    uint8_t B;
    uint8_t X;
    uint8_t Y;
    uint8_t BLACK;
    uint8_t WHITE;
    uint8_t L;
    uint8_t R;
    int16_t leftStickX;
    int16_t leftStickY;
    int16_t rightStickX;
    int16_t rightStickY;
};

struct gameHID_t gameHID;
struct xboxHID_t xboxHID;
extern uint8_t hid_setup_ran;
extern uint8_t usb_failed;
extern uint8_t usb_failed2;
extern uint8_t unknown_bmrequest;
extern uint8_t entered_xid_req;
extern uint8_t dataout_ran;
extern uint8_t rumble_brequest_sent;

uint8_t old_rumble_val_L = 0;
uint8_t old_rumble_val_R = 0;
uint8_t new_rumble_val_L = 0;
uint8_t new_rumble_val_R = 0;

uint32_t button_press_idle = 0;


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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();

  //MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* For the USB Host Shield Library*/
  SPI_Handle = hspi1;
  UART_Handle = huart2;

  /* The Primary Timer, since using freeRTOS, not using systick */
  HAL_TIM_Base_Start_IT(&htim14);

  /* Verify our CPU Frequency
   * We should get a 500ms delay here */
  Serial.print(F("\r\nCPU Frequency is: "));
  cpu_freq = HAL_RCC_GetHCLKFreq()/1000000;
  Serial.print((int)cpu_freq);
  Serial.print("MHz");
  Serial.print("\r\nStart");
  timer_val = __HAL_TIM_GET_COUNTER(&htim14);
  HAL_Delay(500);
  timer_val = __HAL_TIM_GET_COUNTER(&htim14) - timer_val;
  Serial.print("\r\nTime Elapsed is: ");
  Serial.print((int)timer_val/10);
  Serial.print(" ms");
//  hal_gettick = HAL_GetTick();
//  hal_gettick/1000;

  Serial.print((int)hal_gettick);




  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of getBT */
  getBTHandle = osThreadNew(StartGetBT, NULL, &getBT_attributes);

  /* creation of sendUSB */
  sendUSBHandle = osThreadNew(StartSendUSB, NULL, &sendUSB_attributes);

  /* creation of controllerJoin */
  controllerJoinHandle = osThreadNew(StartControllerJoin, NULL, &controllerJoin_attributes);

  /* creation of buttonPress */
  buttonPressHandle = osThreadNew(StartButtonPress, NULL, &buttonPress_attributes);

  /* creation of updateLCD */
  updateLCDHandle = osThreadNew(StartUpdateLCD, NULL, &updateLCD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Used to Meaesure latencies of tasks*/
  getLatencies = osThreadNew(StartGetLatencies, NULL, &getLatencies_attributes);


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = (168/2)*100 -1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void ProcessKeyCodeInContext(uint8_t keyCode) {
/*Updates the display_no
* We could also just call display funcitons directly here, but since we have extra processing speed
* Let's play with freeRTOS */
  if(display_no == 0) { /* This is the status screen, show if controller or not connected */
    if(keyCode == BACK_BTN)
      display_no = 1;
    else if(keyCode == FORWARD_BTN)
      display_no = 1;
  } else if (display_no == 1) { /* Pair Controller Screen */
    if(keyCode == BACK_BTN)
      display_no = 0;
    else if(keyCode == FORWARD_BTN)
      display_no = 0;
    else if(keyCode == SELECT_BTN) {
      display_no = 7;  /* Only get to the pair status screen from here */
    }
  } else if (display_no == 2) {

  }
//  Serial.print("\r\nDisplay no is: ");
//  Serial.print(display_no);
  display_run_once = 0;
  display_force_update = 1;
}

/* USER CODE BEGIN Header_StartControllerJoin */
/**
* @brief Function implementing the controllerJoin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerJoin */
void StartGetLatencies(void *argument)
{
  /* USER CODE BEGIN StartGetLatencies */
  /* Infinite loop */
  for(;;)
  {
#if rtos_delay_view //this is just used to measure the delay of StartGetBT task
	  if(timer_val_getBT >= 7) {
		  Serial.print("\r\nWarning High CPU/BT Latency, getBT latency is: ");
		  Serial.print(timer_val_getBT);
	  }
	  if(timer_val_getUSB >= 4) {
		  Serial.print("\r\nWarning High CPU/USB Latency, getUSB latency is: ");
		  Serial.print(timer_val_getUSB);
	  }
#if rtos_delay_view == 2
	  Serial.print("\r\ngetBT execution time is: ");
	  Serial.print(timer_val_getBT);

	  Serial.print("\r\ngetUSB execution time is: ");
	  Serial.print(timer_val_getUSB);

	  Serial.print(" ");
	  Serial.print(timer_val_LCD);
#endif
#endif
#if rtos_delay_view
	/* Only rx_buf[3] and rx[5] have the rumble data */
	/* dataout ran should only run for THPS 2 or if used on XBCD on a PC */
	Serial.print("\r\nRumble Data: ");
//	Serial.print(rx_buf[0]);
//	Serial.print(" ");
//	Serial.print(rx_buf[1]);
//	Serial.print(" ");
//	Serial.print(rx_buf[2]);
//	Serial.print(" ");
	Serial.print(rx_buf[3]);
	Serial.print(" ");
//	Serial.print(rx_buf[4]);
//	Serial.print(" ");
	Serial.print(rx_buf[5]);
	Serial.print("   ");
	Serial.print(dataout_ran);
	Serial.print(" ");
	Serial.print(rumble_brequest_sent);
	Serial.print(" ");
	Serial.print(button_press_idle);
#endif
	osDelay(1000);


  }
  /* USER CODE END StartGetLatencies */
}
/* USER CODE END 4 */

void StartGetBT(void *argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN 5 */
  if (Usb.Init() == -1) {
  		Serial.print(F("\r\nOSC did not start"));
  		while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  /*Initialize our gamepad, this is for use on a PC*/
  gameHID.JoyX = 0;
  gameHID.JoyY = 0;
  gameHID.Joy2X = 0;
  gameHID.Joy2Y = 0;
  gameHID.Joy_LT = 0;
  gameHID.Joy_RT = 0;
  gameHID.ps4ButtonsTag.dummy = 0;


  /* Initalize our Xbox Controller data that we will send in our hid reports */
  xboxHID.startByte = 0x00;
  xboxHID.bLength = 20;
  xboxHID.dButtons = 0x00;
  xboxHID.A = 0;
  xboxHID.B = 0;
  xboxHID.X = 0;
  xboxHID.Y = 0;
  xboxHID.BLACK = 0;
  xboxHID.WHITE = 0;
  xboxHID.L = 0;
  xboxHID.R = 0;
  xboxHID.leftStickX = 0;
  xboxHID.leftStickY = 0;
  xboxHID.rightStickX = 0;
  xboxHID.rightStickY = 0;
  //ssd1306_TestAll();
  /* Infinite loop */
  for(;;)
  {

	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
//	    if(entered_xid_req) {
//		    Serial.print("\r\nEntered xid req");
//	    }
//		if(unknown_bmrequest) {
//			Serial.print("\r\nUnknown bmrequest");
//		}
//		if(hid_setup_ran > 0) {
//			Serial.print("\r\nHey the xid code ran ");
//			Serial.print("\r\n");
//			Serial.print(caller_str);
//		}
//		if(usb_failed || usb_failed2) {
//			Serial.print("\r\nUSBd failed");
//		}
#if rtos_delay_view
	  	timer_val_getBT = __HAL_TIM_GET_COUNTER(&htim14);
#endif
		Usb.Task();
		if (PS4.connected()) {
			ps4_connected = 1;
			LeftHatX_val = PS4.getAnalogHat(LeftHatX);
			LeftHatY_val = PS4.getAnalogHat(LeftHatY);
			RightHatX_val = PS4.getAnalogHat(RightHatX);
			RightHatY_val = PS4.getAnalogHat(RightHatY);
#if deadzone_enable
			/* Let's have a builtin deadzone */
			if (LeftHatX_val > 137 || LeftHatX_val < 117 || LeftHatY_val > 137 || LeftHatY_val < 117) {// || RightHatX_val > 137 || RightHatX_val < 117 || RightHatY_val > 137 || RightHatY_val < 117) {
				gameHID.JoyX = PS4.getAnalogHat(LeftHatX) - 128;
				gameHID.JoyY = PS4.getAnalogHat(LeftHatY) - 128;
				xboxHID.leftStickX = gameHID.JoyX << 8;	//only getting 8 bit value from bt
				xboxHID.leftStickY = gameHID.JoyY << 8;	//xbox uses 16 bit signed
				/* The Y axis by default is inverted on the Xbox */
				xboxHID.leftStickY = -xboxHID.leftStickY-128;

			} else {
				gameHID.JoyX = 0;
				gameHID.JoyY = 0;
				xboxHID.leftStickX = 0;
				xboxHID.leftStickY = 0;
			}
			if(RightHatX_val > 137 || RightHatX_val < 117 || RightHatY_val > 137 || RightHatY_val < 117) {
				gameHID.Joy2X = PS4.getAnalogHat(RightHatX) - 128;
				gameHID.Joy2Y = PS4.getAnalogHat(RightHatY) - 128;
				xboxHID.rightStickX = gameHID.Joy2X << 8;
				xboxHID.rightStickY = gameHID.Joy2Y << 8;

				/* The Y axis by default is inverted on the Xbox */
				xboxHID.rightStickY = -xboxHID.rightStickY - 128;

			} else {
				gameHID.Joy2X = 0;
				gameHID.Joy2Y = 0;
				xboxHID.rightStickX = 0;
				xboxHID.rightStickY = 0;
			}
#elif !deadzone_enable
			gameHID.JoyX = PS4.getAnalogHat(LeftHatX) - 128;
			gameHID.JoyY = PS4.getAnalogHat(LeftHatY) - 128;
			xboxHID.leftStickX = gameHID.JoyX << 8;	//only getting 8 bit value from bt
			xboxHID.leftStickY = gameHID.JoyY << 8;	//xbox uses 16 bit signed
			/* The Y axis by default is inverted on the Xbox */
			xboxHID.leftStickY = -xboxHID.leftStickY-128;

			gameHID.Joy2X = PS4.getAnalogHat(RightHatX) - 128;
			gameHID.Joy2Y = PS4.getAnalogHat(RightHatY) - 128;
			xboxHID.rightStickX = gameHID.Joy2X << 8;
			xboxHID.rightStickY = gameHID.Joy2Y << 8;

			/* The Y axis by default is inverted on the Xbox */
			xboxHID.rightStickY = -xboxHID.rightStickY - 128;
#endif

			xboxHID.L = PS4.getAnalogButton(L2);
			xboxHID.R = PS4.getAnalogButton(R2);
			gameHID.Joy_LT = xboxHID.L - 128;
			gameHID.Joy_RT = xboxHID.R - 128;

			if (PS4.getButtonClick(PS)) {
				PS4.disconnect();
				display_run_once = 0;
				rumble_once = 0;
        
			}
			if (PS4.getButtonPress(TRIANGLE)) {
				gameHID.ps4ButtonsTag.button_triangle = 1;
				xboxHID.Y = 0xFF;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_triangle = 0;
				xboxHID.Y = 0;
			}

			if (PS4.getButtonPress(CIRCLE)) {
				gameHID.ps4ButtonsTag.button_circle = 1;
				xboxHID.B = 0xFF;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_circle = 0;
				xboxHID.B = 0;
			}

			if (PS4.getButtonPress(CROSS)) {
				gameHID.ps4ButtonsTag.button_cross = 1;
				xboxHID.A = 0xFF;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_cross = 0;
				xboxHID.A = 0;
			}

			if (PS4.getButtonPress(SQUARE)) {
				gameHID.ps4ButtonsTag.button_square = 1;
				xboxHID.X = 0xFF;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_square = 0;
				xboxHID.X = 0;
			}

			if (PS4.getButtonPress(UP)) {
				gameHID.ps4ButtonsTag.button_dpad_up = 1;
				xboxHID.dButtons |= XBOX_DUP;
			} else {
				gameHID.ps4ButtonsTag.button_dpad_up = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_DUP;
			}

			if (PS4.getButtonPress(RIGHT)) {
				gameHID.ps4ButtonsTag.button_dpad_right = 1;
				xboxHID.dButtons |= XBOX_DRIGHT;
			} else {
				gameHID.ps4ButtonsTag.button_dpad_right = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_DRIGHT;
			}

			if (PS4.getButtonPress(DOWN)) {
				gameHID.ps4ButtonsTag.button_dpad_down = 1;
				xboxHID.dButtons |= XBOX_DDOWN;
			} else {
				gameHID.ps4ButtonsTag.button_dpad_down = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_DDOWN;
			}

			if (PS4.getButtonPress(LEFT)) {
				gameHID.ps4ButtonsTag.button_dpad_left = 1;
				xboxHID.dButtons |= XBOX_DLEFT;

			} else {
				gameHID.ps4ButtonsTag.button_dpad_left = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_DLEFT;
			}

			if (PS4.getButtonPress(L1)) {
				gameHID.ps4ButtonsTag.button_left_trigger = 1;
				xboxHID.WHITE = 0xFF;
				button_press_idle = 0;

			} else {
				gameHID.ps4ButtonsTag.button_left_trigger = 0;
				xboxHID.WHITE = 0;
			}

			if (PS4.getButtonPress(L3)) {
				gameHID.ps4ButtonsTag.button_left_thumb = 1;
				xboxHID.dButtons |= XBOX_LS_BTN;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_left_thumb = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_LS_BTN;
			}

			if (PS4.getButtonPress(R1)) {
				gameHID.ps4ButtonsTag.button_right_trigger = 1;
				xboxHID.BLACK = 0xFF;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_right_trigger = 0;
				xboxHID.BLACK = 0;
			}

			if (PS4.getButtonPress(R3)) {
				gameHID.ps4ButtonsTag.button_right_thumb = 1;
				xboxHID.dButtons |= XBOX_RS_BTN;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_right_thumb = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_RS_BTN;
			}

			if (PS4.getButtonPress(SHARE)) {
				gameHID.ps4ButtonsTag.button_share = 1;
				xboxHID.dButtons |= XBOX_BACK_BTN;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_share = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_BACK_BTN;
			}

			if (PS4.getButtonPress(OPTIONS)) {
				gameHID.ps4ButtonsTag.button_start = 1;
				xboxHID.dButtons |= XBOX_START_BTN;
				button_press_idle = 0;
			} else {
				gameHID.ps4ButtonsTag.button_start = 0;
				xboxHID.dButtons = xboxHID.dButtons & ~XBOX_START_BTN;
			}

			new_rumble_val_L = rx_buf[3];
			new_rumble_val_R = rx_buf[5];

			if(new_rumble_val_L != old_rumble_val_L || new_rumble_val_R != old_rumble_val_R) {
				PS4.setRumbleOn(new_rumble_val_L, new_rumble_val_R);
				old_rumble_val_L = new_rumble_val_L;
				old_rumble_val_R = new_rumble_val_R;
			}
			/* After roughly 5+minutes of idle time, disconnect controller
			 * Not the best solution since the rate the counter increases is based on BT Latency */
			if(button_press_idle > 400000) {
				PS4.disconnect();
				rumble_once = 0;
				button_press_idle = 0;
			}
			button_press_idle++;

		} else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
			if (!buttonPressed) {
				Serial.print(F("\r\nButton Pressed"));
				PS4.pair(); // Start paring routine if user button was just pressed
			}
			buttonPressed = true;
		} else
			buttonPressed = false;
#if rtos_delay_view
		timer_val_getBT = __HAL_TIM_GET_COUNTER(&htim14) - timer_val_getBT;
#endif
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSendUSB */
/**
* @brief Function implementing the sendUSB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendUSB */
void StartSendUSB(void *argument)
{
  /* USER CODE BEGIN StartSendUSB */
	MX_USB_DEVICE_Init();
  /* Infinite loop */
  for(;;)
  {
	/*We are defined as a USB Fullspeed device.
	Polling rate is determined in usbd_hid.h as HID_FS_BINTERVAL, set to 0x04U
	Most settings are defined in usbd_conf.h
	So even though we are updating the report ~1000hz, we still only send at 250hz.
	Host determines when to read with USB, not the slave, we are writing to a buffer */
#if(PC_SETUP)
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
#endif

#if OG_XBOX_SETUP
#if rtos_delay_view
	timer_val_getUSB = __HAL_TIM_GET_COUNTER(&htim14);
#endif
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &xboxHID, sizeof(struct xboxHID_t));
#if rtos_delay_view
	timer_val_getUSB = __HAL_TIM_GET_COUNTER(&htim14) - timer_val_getUSB;
#endif
#endif
    osDelay(1);
  }
  /* USER CODE END StartSendUSB */
}

/* USER CODE BEGIN Header_StartControllerJoin */
/**
* @brief Function implementing the controllerJoin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerJoin */
void StartControllerJoin(void *argument)
{
  /* USER CODE BEGIN StartControllerJoin */
  /* Infinite loop */
  for(;;)
  {
	if(PS4.connected() && !rumble_once) {
	  PS4.setRumbleOn(RumbleLow);
	  osDelay(500);
	  PS4.setRumbleOff();
	  rumble_once = 1;
	}
	osDelay(300);
  }
  /* USER CODE END StartControllerJoin */
}

/* USER CODE BEGIN Header_StartButtonPress */
/**
* @brief Function implementing the buttonPress thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonPress */
void StartButtonPress(void *argument)
{
  /* USER CODE BEGIN StartButtonPress */
  /* Infinite loop */
  for(;;)
  {
  keyCode = (HAL_GPIO_ReadPin(BACK_BTN_GPIO) << 2) |
			(HAL_GPIO_ReadPin(SELECT_BTN_GPIO) << 1) |
			(HAL_GPIO_ReadPin(FORWARD_BTN_GPIO) << 0);

  if (BUTTON_PRESSED) {
	  if(buttonDebounced == 1) {  // you only get here if the same button combination has been pressed for 100mS
		  if (buttonProcessed == 0) { // here's where we do the real work on the keyboard, and ensure we only do it once/keypress
			  buttonProcessed = 1;
			  ProcessKeyCodeInContext(keyCode);
		  }
	  } else {
		  buttonDebounced = true;
	  }
  } else {
	  buttonDebounced = false;
	  buttonProcessed = false;
  }
//  	Serial.print("\r\n");
//  	Serial.print(keyCode);
    osDelay(100);
  }
  /* USER CODE END StartButtonPress */
}

/* USER CODE BEGIN Header_StartUpdateLCD */
/**
* @brief Function implementing the updateLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdateLCD */
void StartUpdateLCD(void *argument)
{
  /* USER CODE BEGIN StartUpdateLCD */
  /* Infinite loop */

  ssd1306_Fill(Black_);
  ssd1306_UpdateScreen();
  for(;;)
  {
#if rtos_delay_view
	  	timer_val_LCD = __HAL_TIM_GET_COUNTER(&htim14);
#endif
	if(display_run_once == 0) {
		ssd1306_Fill(Black_);
		ssd1306_UpdateScreen();
		switch(display_no)
		{
			case 0 :
			{
				uint8_t alternate_print = 1;
				if(!PS4.connected()) {
				  ssd1306_SetCursor((128-11*3)/2,0);
				  ssd1306_WriteString("Not", Font_11x18, White_);
				  display_force_update = 0;
				  ssd1306_SetCursor((128-11*9)/2, 26);
				  ssd1306_WriteString("Connected", Font_11x18, White_);
				} else if (PS4.connected()) {
				  ssd1306_SetCursor((128-11*9)/2, 26);
				  ssd1306_WriteString("Connected", Font_11x18, White_);
				  alternate_print = 0;
				  display_run_once = 1;
				  display_force_update = 0;
				}
				ssd1306_UpdateScreen();

				while(!PS4.connected() && display_force_update == 0) {
					osDelay(100);
				}
				/* If the user presses a Button, interrupt and show next screen */
				if(display_force_update == 1) {
					display_force_update = 0;
					display_run_once = 0;
					break;
				}
				/*When the controller is finally paired update current screen
				 * Only runs if the first PS4.connected() above does not run*/
				if(PS4.connected() && alternate_print) {
				    ssd1306_Fill(Black_);
				    ssd1306_UpdateScreen();
//				    ssd1306_SetCursor(25,0);
//				    ssd1306_WriteString("Status:", Font_11x18, White_);
					ssd1306_SetCursor((128-11*10)/2, 26);
					ssd1306_WriteString("Connected!", Font_11x18, White_);
				    ssd1306_UpdateScreen();
				    display_run_once = 1;
				}
				break;
			}

			case 1 :
				display_run_once = 1;
				display_force_update = 0;
				ssd1306_Fill(Black_);
				ssd1306_SetCursor((128-11*5)/2,0);
				ssd1306_WriteString("Pair?", Font_11x18, White_);
				ssd1306_UpdateScreen();
				break;

			case 7 :
				display_run_once = 1;
				display_force_update = 0;
				ssd1306_Fill(Black_);
				ssd1306_UpdateScreen();
				ssd1306_SetCursor((128-11*10)/2,0);
				ssd1306_WriteString("Pairing...", Font_11x18, White_);
				ssd1306_UpdateScreen();
				PS4.pair();
				while(PS4.connected() == 0) {
				  osDelay(100);
				}
				ssd1306_SetCursor((128-11*10)/2,26);
				ssd1306_WriteString("Paired!", Font_11x18, White_);
				ssd1306_UpdateScreen();
				break;
		}
	}
#if rtos_delay_view
		timer_val_LCD = __HAL_TIM_GET_COUNTER(&htim14) - timer_val_LCD;
#endif
    osDelay(100);
  }
  /* USER CODE END StartUpdateLCD */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	Serial.print("\r\nSomething went wrong!");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
