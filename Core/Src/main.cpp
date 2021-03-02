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
#include <PS4BT.h>	//usb host shield library
#include <usbhub.h>	//usb host shield library
#include "usbd_hid.h" //st library
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SPI_HandleTypeDef SPI_Handle;
UART_HandleTypeDef UART_Handle;
//SerialClass Serial(&UART_Handle);
SerialClass Serial(&huart2);

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
//PS4BT PS4(&Btd, PAIR);
static bool printAngle, printTouch;
static uint8_t oldL2Value, oldR2Value;
static bool buttonPressed;
extern USBD_HandleTypeDef hUsbDeviceFS;


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
      //WII_CC_BUTTONS Buttons; 	// Button, one byte, button is bit #0
};



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
  //MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SPI_Handle = hspi1;
  UART_Handle = huart2;

  struct gameHID_t gameHID;
  gameHID.JoyX = 0;
  gameHID.JoyY = 0;
  gameHID.Joy2X = 0;
  gameHID.Joy2Y = 0;
  gameHID.Joy_LT = 0;
  gameHID.Joy_RT = 0;

  uint8_t LeftHatX_val;
  uint8_t LeftHatY_val;
  uint8_t RightHatX_val;
  uint8_t RightHatY_val;
  uint32_t cpu_freq = 0;
  uint8_t rumble_once = 0;

//  uint8_t L2_val;
//  uint8_t R2_val;
  Serial.print(F("\r\nCPU Frequency is: "));
  cpu_freq = HAL_RCC_GetHCLKFreq()/1000000;
  Serial.print((int)cpu_freq);
  Serial.print("MHz");
  if (Usb.Init() == -1) {
  		Serial.print(F("\r\nOSC did not start"));
  		while (1); // Halt
  	}
  	Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Usb.Task();
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));

		if (PS4.connected()) {
			if(!rumble_once) {
				PS4.setRumbleOn(RumbleLow);
				PS4.setRumbleOff();
				rumble_once = 1;
			}

			LeftHatX_val = PS4.getAnalogHat(LeftHatX);
			LeftHatY_val = PS4.getAnalogHat(LeftHatY);
			RightHatX_val = PS4.getAnalogHat(RightHatX);
			RightHatY_val = PS4.getAnalogHat(RightHatY);

			if (LeftHatX_val > 137 || LeftHatX_val < 117 || LeftHatY_val > 137 || LeftHatY_val < 117 || RightHatX_val > 137 || RightHatX_val < 117 || RightHatY_val > 137 || RightHatY_val < 117) {
				gameHID.JoyX = PS4.getAnalogHat(LeftHatX) - 128;
				gameHID.JoyY = PS4.getAnalogHat(LeftHatY) - 128;
				gameHID.Joy2X = PS4.getAnalogHat(RightHatX) - 128;
				gameHID.Joy2Y = PS4.getAnalogHat(RightHatY) - 128;

				//Serial.print(F("\r\nLeftHatX: "));
				//Serial.print(PS4.getAnalogHat(LeftHatX));
				//Serial.print(F("\tLeftHatY: "));
				//Serial.print(PS4.getAnalogHat(LeftHatY));
				//Serial.print(F("\tRightHatX: "));
				//Serial.print(PS4.getAnalogHat(RightHatX));
				//Serial.print(F("\tRightHatY: "));
				//Serial.print(PS4.getAnalogHat(RightHatY));
			} else {
				gameHID.JoyX = 0;
				gameHID.JoyY = 0;
				gameHID.Joy2X = 0;
				gameHID.Joy2Y = 0;
			}

			//if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) { // These are the only analog buttons on the PS4 controller
				gameHID.Joy_LT = PS4.getAnalogButton(L2) - 128;
				gameHID.Joy_RT = PS4.getAnalogButton(R2) - 128;

//				Serial.print(F("\rnJoy_LT: "));
//				Serial.print(gameHID.Joy_LT);

				//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
				//Serial.print(F("\r\nL2: "));
				//Serial.print(PS4.getAnalogButton(L2));
				//Serial.print(F("\tR2: "));
				//Serial.print(PS4.getAnalogButton(R2));
			//}
			if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value) {
				// Only write value if it's different
				//PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
			}

			oldL2Value = PS4.getAnalogButton(L2);
			oldR2Value = PS4.getAnalogButton(R2);
//			Serial.print(F("\r\nL2: "));
//			Serial.print(gameHID.Joy_LT);

			if (PS4.getButtonClick(PS)) {
				gameHID.ps4ButtonsTag.button_ps = 1;
				//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
				//Serial.print(F("\r\nPS"));
				//PS4.disconnect();
			} else {
				if (PS4.getButtonPress(TRIANGLE)) {
					gameHID.ps4ButtonsTag.button_triangle = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nTraingle"));
					//PS4.setRumbleOn(RumbleLow);
				} else {
					gameHID.ps4ButtonsTag.button_triangle = 0;
				}
				if (PS4.getButtonPress(CIRCLE)) {
					gameHID.ps4ButtonsTag.button_circle = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nCircle"));
					//PS4.setRumbleOn(RumbleHigh);
				} else {
					gameHID.ps4ButtonsTag.button_circle = 0;
				}
				if (PS4.getButtonPress(CROSS)) {
					gameHID.ps4ButtonsTag.button_cross = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nCross"));
					//PS4.setLedFlash(10, 10); // Set it to blink rapidly
				} else {
					gameHID.ps4ButtonsTag.button_cross = 0;;
				}
				if (PS4.getButtonPress(SQUARE)) {
					gameHID.ps4ButtonsTag.button_square = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nSquare"));
					//PS4.setLedFlash(0, 0); // Turn off blinking
				} else {
					gameHID.ps4ButtonsTag.button_square = 0;
				}

				if (PS4.getButtonPress(UP)) {
					gameHID.ps4ButtonsTag.button_dpad_up = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nUp"));
					//PS4.setLed(Red);
				} else {
					gameHID.ps4ButtonsTag.button_dpad_up = 0;
				}
				if (PS4.getButtonPress(RIGHT)) {
					gameHID.ps4ButtonsTag.button_dpad_right = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nRight"));
					//PS4.setLed(Blue);
				} else {
					gameHID.ps4ButtonsTag.button_dpad_right = 0;
				}
				if (PS4.getButtonPress(DOWN)) {
					gameHID.ps4ButtonsTag.button_dpad_down = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nDown"));
					//PS4.setLed(Yellow);
				} else {
					gameHID.ps4ButtonsTag.button_dpad_down = 0;
				}
				if (PS4.getButtonPress(LEFT)) {
					gameHID.ps4ButtonsTag.button_dpad_left = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nLeft"));
					//PS4.setLed(Green);
				} else {
					gameHID.ps4ButtonsTag.button_dpad_left = 0;
				}

				if (PS4.getButtonPress(L1)) {
					gameHID.ps4ButtonsTag.button_left_trigger = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nL1"));
				} else {
					gameHID.ps4ButtonsTag.button_left_trigger = 0;
				}
				if (PS4.getButtonPress(L3)) {
					gameHID.ps4ButtonsTag.button_left_thumb = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nL3"));
				} else {
					gameHID.ps4ButtonsTag.button_left_thumb = 0;
				}
				if (PS4.getButtonPress(R1)) {
					gameHID.ps4ButtonsTag.button_right_trigger = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nR1"));
				} else {
					gameHID.ps4ButtonsTag.button_right_trigger = 0;
				}
				if (PS4.getButtonPress(R3)) {
					gameHID.ps4ButtonsTag.button_right_thumb = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nR3"));
				} else {
					gameHID.ps4ButtonsTag.button_right_thumb = 0;
				}

				if (PS4.getButtonPress(SHARE)) {
					gameHID.ps4ButtonsTag.button_share = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nShare"));
				} else {
					gameHID.ps4ButtonsTag.button_share = 0;
				}

				if (PS4.getButtonPress(OPTIONS)) {
					gameHID.ps4ButtonsTag.button_start = 1;
					//USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
					//Serial.print(F("\r\nOptions"));
					printAngle = !printAngle;
				} else {
					gameHID.ps4ButtonsTag.button_start = 0;
				}
				if (PS4.getButtonClick(TOUCHPAD)) {
					//Serial.print(F("\r\nTouchpad"));
					printTouch = !printTouch;
				}

				if (printAngle) { // Print angle calculated using the accelerometer only
					//Serial.print("\r\nPitch: "); // As I have set "-specs=nano.specs" in the linker flags, printf does not support printing floating point number
					//Serial.print(PS4.getAngle(Pitch));
					//Serial.print("\tRoll: ");
					//Serial.print(PS4.getAngle(Roll));
				}

				if (printTouch) { // Print the x, y coordinates of the touchpad
					if (PS4.isTouching(0) || PS4.isTouching(1)) // Print newline and carriage return if any of the fingers are touching the touchpad
						//Serial.print(F("\r\n"));
					for (uint8_t i = 0; i < 2; i++) { // The touchpad track two fingers
						if (PS4.isTouching(i)) { // Print the position of the finger if it is touching the touchpad
							//Serial.print(F("X")); //Serial.print(i + 1); //Serial.print(F(": "));
							//Serial.print(PS4.getX(i));
							//Serial.print(F("\tY")); //Serial.print(i + 1); //Serial.print(F(": "));
							//Serial.print(PS4.getY(i));
							//Serial.print(F("\t"));
						}
					}
				}
			}
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &gameHID, sizeof(struct gameHID_t));
		} else if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
		    if (!buttonPressed) {
		    	Serial.print(F("\r\nButton Pressed"));
		        PS4.pair(); // Start paring routine if user button was just pressed
		    }
		    buttonPressed = true;
		} else
		    buttonPressed = false;
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
	//HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000000); // NOTE: Edited, so it increments every us
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() /   1680000); // NOTE: Edited, so it increments every us

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // SysTick_IRQn interrupt configuration

}


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
//
//  /* USER CODE END ADC1_Init 0 */
//
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC1_Init 1 */
//
//  /* USER CODE END ADC1_Init 1 */
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
//  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc1.Init.ScanConvMode = DISABLE;
//  hadc1.Init.ContinuousConvMode = DISABLE;
//  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc1.Init.NbrOfConversion = 1;
//  hadc1.Init.DMAContinuousRequests = DISABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_13;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */
//
//  /* USER CODE END ADC1_Init 2 */
//
//}

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
