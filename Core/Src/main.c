/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
#define bufferSize 8
uint8_t keyboardHID[bufferSize] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t KEY[3] = { 0x04, 0x16, 0x07 };
uint32_t USBKEYBOARDfailCOUNTER = 0;
uint32_t USBKEYBOARDbusyCOUNTER = 0;
uint32_t USBJOYSTICKfailCOUNTER = 0;
uint32_t USBJOYSTICKbusyCOUNTER = 0;
uint32_t USBKEYBOARDokTIME = 0;
uint32_t USBJOYSTICKokTIME = 0;
uint32_t USBKEYBOARDokSTART = 0;
uint32_t USBJOYSTICKokSTART = 0;
bool ISsendKEYBOARD = SET;

uint32_t loopTime = 0;
int32_t HE_value[3] = { 0, 0, 0 };
uint32_t HE_TOPvalue[3] = { 0, 0, 0 };
uint32_t HE_BOTTOMvalue[3] = { 0, 0, 0 };
uint32_t HE_RANGEvalue[3] = { 0, 0, 0 };
float HE_SCALEvalue[3] = { 0.00, 0.00, 0.00 };
float HE_MMvalue[3] = { 0.00, 0.00, 0.00 };
float HE_lastMMvalue[3] = { 0.00, 0.00, 0.00 };
uint8_t HE_Status[3] = { RESET, RESET, RESET };
float HE_StatusChangeOffset = 0.15;
float HE_trigger = 0.30;
int32_t HE_RTcount = 0;
static float SwitchTravelDistance = 4.02;
static uint32_t ADC_HE_Channel[3] = {
ADC_CHANNEL_1, ADC_CHANNEL_2,
ADC_CHANNEL_3 };
bool HE_calibration = RESET;

volatile int32_t encoderPos = 0;
volatile bool encoderPrevA = RESET;
volatile bool encoderPrevB = RESET;
int32_t lastEncoderPos = 0;
uint32_t lastEncoderTime = 0;
volatile bool pushButtonState = RESET;
bool encoderCW = RESET;
bool encoderCCW = RESET;

const unsigned char bitmap_Press[] = { 0x10, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38,
		0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xfe, 0xfe,
		0x7c, 0x38, 0x10, };
const unsigned char bitmap_Realese[] = { 0x10, 0x38, 0x7c, 0xfe, 0xfe, 0x38,
		0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38,
		0x38, 0x38, 0x38, 0x10, };
const unsigned char bitmap_OSUlogo[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0,
		0x00, 0x00, 0x00, 0x00, 0x0f, 0xf8, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0x00,
		0x3f, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x3f,
		0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x03,
		0xc0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x01,
		0xe0, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x1e,
		0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00,
		0x3c, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x70,
		0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
		0x07, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0xc0,
		0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00,
		0x03, 0x80, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x03, 0x80,
		0x00, 0x00, 0x00, 0x01, 0xc1, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01,
		0xc0, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0xe0, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x01, 0xc0, 0xe0, 0x07, 0x01, 0xf8, 0x1f, 0x9c, 0x71,
		0xc0, 0xe0, 0x07, 0x03, 0xfc, 0x3f, 0x1c, 0x71, 0xc0, 0xe0, 0x06, 0x03,
		0x8e, 0x70, 0x1c, 0x71, 0xc0, 0x60, 0x06, 0x07, 0x0e, 0x78, 0x1c, 0x71,
		0xc0, 0x60, 0x06, 0x07, 0x0e, 0x3f, 0x1c, 0x71, 0xc0, 0x60, 0x06, 0x07,
		0x06, 0x1f, 0x9c, 0x71, 0xc0, 0x60, 0x06, 0x07, 0x0e, 0x03, 0x9c, 0x70,
		0x00, 0x60, 0x06, 0x03, 0x8e, 0x03, 0x9c, 0x70, 0x80, 0x60, 0x07, 0x03,
		0xde, 0x3f, 0x8f, 0xf1, 0xc0, 0xe0, 0x07, 0x01, 0xfc, 0x7f, 0x0f, 0xf1,
		0xc0, 0xe0, 0x07, 0x00, 0x70, 0x3c, 0x03, 0xe1, 0xc0, 0xe0, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xc0, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x03, 0x80,
		0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00,
		0x03, 0x80, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0xe0,
		0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
		0x07, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x78,
		0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00,
		0x3c, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x0f,
		0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x01,
		0xe0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x01,
		0xf0, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x3f,
		0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00,
		0x0f, 0xf8, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00 };
char RTosuPAD[] = "RTosuPAD";
char By[] = "By:";
char fikr_hrd[] = "@fikr_hrd";
char RTcount[] = "RTcount:";
char Trig[] = "Trig:";
char RT[] = "RT:";
uint32_t ssd1306_timeTaken = 0;
bool ssd1306_Enable = RESET;
int8_t ssd1306_displayIndex = 0;
bool ssd1306_displayEditing = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void ADC_HE_Init(void);
void ADC_Read_HE1(void);
void ADC_Read_HE2(void);
void ADC_Read_HE3(void);
void ADC_HE_Reading(void);
void Blink(void);
void EncoderUpdate(void);
void Encoder(void);
void PushButtonUpdate(void);
void keyUpdate(void);
void ssd1306_display(void);
void ssd1306_LOGO(void);
float Fconstrain(float, float, float);
int32_t constrain(int32_t, int32_t, int32_t);

// TODO change below variable if generate new code
//#define USB_CUSTOM_HID_CONFIG_DESC_SIZ               59U

//#ifndef CUSTOM_HID_KEYBOARD_EPIN_ADDR
//#define CUSTOM_HID_KEYBOARD_EPIN_ADDR                         0x81U
//#endif /* CUSTOM_HID_KEYBOARD_EPIN_ADDR */
//
//#ifndef CUSTOM_HID_KEYBOARD_EPIN_SIZE
//#define CUSTOM_HID_KEYBOARD_EPIN_SIZE                         0x02U
//#endif /* CUSTOM_HID_KEYBOARD_EPIN_SIZE */
//
//#ifndef CUSTOM_HID_JOYSTICK_EPIN_ADDR
//#define CUSTOM_HID_JOYSTICK_EPIN_ADDR                         0x82U
//#endif /* CUSTOM_HID_JOYSTICK_EPIN_ADDR */
//
//#ifndef CUSTOM_HID_JOYSTICK_EPIN_SIZE
//#define CUSTOM_HID_JOYSTICK_EPIN_SIZE                         0x02U
//#endif /* CUSTOM_HID_JOYSTICK_EPIN_SIZE */

//#define USBD_CUSTOM_HID_KEYBOARD_REPORT_DESC_SIZE     2U
//#define USBD_CUSTOM_HID_JOYSTICK_REPORT_DESC_SIZE     2U
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	ssd1306_Init();
	ADC_HE_Init();
	ssd1306_LOGO();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint32_t x = HAL_GetTick();
//				ssd1306_TestAll();
		ADC_HE_Reading();
		ssd1306_display();
		Encoder();
		keyUpdate();
		//		Blink();

		loopTime = HAL_GetTick() - x;
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 99;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 419;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(UserLED_GPIO_Port, UserLED_Pin);

	/**/
	GPIO_InitStruct.Pin = UserLED_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(UserLED_GPIO_Port, &GPIO_InitStruct);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE0);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE2);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	LL_GPIO_SetPinPull(PushButton_GPIO_Port, PushButton_Pin, LL_GPIO_PULL_NO);

	/**/
	LL_GPIO_SetPinPull(EnB_GPIO_Port, EnB_Pin, LL_GPIO_PULL_NO);

	/**/
	LL_GPIO_SetPinPull(EnA_GPIO_Port, EnA_Pin, LL_GPIO_PULL_NO);

	/**/
	LL_GPIO_SetPinMode(PushButton_GPIO_Port, PushButton_Pin,
	LL_GPIO_MODE_INPUT);

	/**/
	LL_GPIO_SetPinMode(EnB_GPIO_Port, EnB_Pin, LL_GPIO_MODE_INPUT);

	/**/
	LL_GPIO_SetPinMode(EnA_GPIO_Port, EnA_Pin, LL_GPIO_MODE_INPUT);

	/* EXTI interrupt init*/
	NVIC_SetPriority(EXTI0_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_SetPriority(EXTI1_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI2_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(EXTI2_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC_HE_Init(void) {
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	for (uint8_t i = 0; i < 3; ++i) {
		ADC_ChannelConfTypeDef sConfig = { 0 };
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
		sConfig.Channel = ADC_HE_Channel[i];
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}

		for (uint8_t j = 0; j < 10; ++j) {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			HE_TOPvalue[i] += HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			HAL_Delay(40);
		}
		HE_TOPvalue[i] /= 10;
	}
	LL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
}

void ADC_HE_Reading(void) {
	if (!HE_calibration) {
		if (pushButtonState) {
			pushButtonState = RESET;
			for (uint8_t i = 0; i < 3; ++i) {
				ADC_ChannelConfTypeDef sConfig = { 0 };
				/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
				 */
				sConfig.Channel = ADC_HE_Channel[i];
				sConfig.Rank = 1;
				sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
				if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
					Error_Handler();
				}

				for (uint8_t j = 0; j < 10; ++j) {
					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, 1000);
					HE_BOTTOMvalue[i] += HAL_ADC_GetValue(&hadc1);
					HAL_ADC_Stop(&hadc1);
					HAL_Delay(40);
				}
				HE_BOTTOMvalue[i] /= 10;
				HE_RANGEvalue[i] = HE_TOPvalue[i] - HE_BOTTOMvalue[i];
				HE_SCALEvalue[i] = SwitchTravelDistance / HE_RANGEvalue[i];
			}
			HE_calibration = SET;
		}
	} else {
		for (uint8_t i = 0; i < 3; ++i) {
			ADC_ChannelConfTypeDef sConfig = { 0 };
			/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
			 */
			sConfig.Channel = ADC_HE_Channel[i];
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
				Error_Handler();
			}
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			HE_value[i] = HAL_ADC_GetValue(&hadc1) - HE_BOTTOMvalue[i];
			HAL_ADC_Stop(&hadc1);
			HE_MMvalue[i] = HE_value[i] * HE_SCALEvalue[i];
		}
	}
}

void keyUpdate(void) {
	static uint32_t time = 0;
	if (HAL_GetTick() - time > 1) {
		time = HAL_GetTick();

		if (HE_calibration) {
			for (int8_t i = 0; i < 3; ++i) {
				if (HE_MMvalue[i] < 4.00 - HE_trigger) {
					if (HE_MMvalue[i]
							> HE_lastMMvalue[i] + HE_StatusChangeOffset
//						|| HE_MMvalue[i] > 4.00
									) {
						HE_RTcount++;
						HE_lastMMvalue[i] = HE_MMvalue[i];
						HE_Status[i] = RESET;
						keyboardHID[i + 2] = 0x00;
					}

					if (HE_MMvalue[i]
							< HE_lastMMvalue[i] - HE_StatusChangeOffset) {
						HE_lastMMvalue[i] = HE_MMvalue[i];
						HE_Status[i] = SET;
						keyboardHID[i + 2] = KEY[i];
					}
				} else {
					HE_lastMMvalue[i] = HE_MMvalue[i] + HE_StatusChangeOffset;
					HE_Status[i] = RESET;
					keyboardHID[i + 2] = 0x00;
				}

			}
		}

		keyboardHID[5] = 0x00;
		keyboardHID[6] = 0x00;
		if (!ssd1306_Enable) {
			if (encoderCW) {
				encoderCW = RESET;
				keyboardHID[5] = 0x27;
			}

			if (encoderCCW) {
				encoderCCW = RESET;
				keyboardHID[6] = 0x1E;
			}
		}

		uint8_t buffer[CUSTOM_HID_JOYSTICK_EPIN_SIZE] = { 0 };
		uint32_t PB_activity = 2863311530;
		buffer[0] = PB_activity;
		buffer[1] = PB_activity >> 8;
		buffer[2] = PB_activity >> 16;
		buffer[3] = PB_activity >> 24;
		buffer[4] = LOBYTE(65535 / 2);
		buffer[5] = HIBYTE(65535 / 2);
		buffer[6] = LOBYTE(65535 / 2);
		buffer[7] = HIBYTE(65535 / 2);

		if (ISsendKEYBOARD) {
			switch (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, keyboardHID,
			bufferSize, CUSTOM_HID_KEYBOARD_EPIN_ADDR)) {
			case USBD_FAIL:
				USBKEYBOARDfailCOUNTER++;
				USBKEYBOARDokSTART = HAL_GetTick();
				break;
			case USBD_BUSY:
				USBKEYBOARDbusyCOUNTER++;
				USBKEYBOARDokSTART = HAL_GetTick();
				break;
			default:
				USBKEYBOARDokTIME = HAL_GetTick() - USBKEYBOARDokSTART;
				ISsendKEYBOARD = RESET;
				break;
			}
		} else {
			switch (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buffer,
			CUSTOM_HID_JOYSTICK_EPIN_SIZE, CUSTOM_HID_JOYSTICK_EPIN_ADDR)) {
			case USBD_FAIL:
				USBJOYSTICKfailCOUNTER++;
				USBJOYSTICKokSTART = HAL_GetTick();
				break;
			case USBD_BUSY:
				USBJOYSTICKbusyCOUNTER++;
				USBJOYSTICKokSTART = HAL_GetTick();
				break;
			default:
				USBJOYSTICKokTIME = HAL_GetTick() - USBJOYSTICKokSTART;
				ISsendKEYBOARD = SET;
				break;
			}
		}

	}
}

void Blink(void) {
	static uint32_t timeElpased = 0;
	if (HAL_GetTick() - timeElpased >= HE_value[1] / 10) {
		timeElpased = HAL_GetTick();
		HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
	}
}

void EncoderUpdate(void) {
	bool pinA = LL_GPIO_IsInputPinSet(EnA_GPIO_Port, EnA_Pin);
	bool pinB = LL_GPIO_IsInputPinSet(EnB_GPIO_Port, EnB_Pin);

	if ((encoderPrevA == pinA && encoderPrevB == pinB))
		return;  // no change since last time (i.e. reject bounce)

	// same direction (alternating between 0,1 and 1,0 in one direction or 1,1 and 0,0 in the other direction)
	if (encoderPrevA == 1 && encoderPrevB == 0 && pinA == 0 && pinB == 1)
		encoderPos -= 1;
	else if (encoderPrevA == 0 && encoderPrevB == 1 && pinA == 1 && pinB == 0)
		encoderPos -= 1;
	else if (encoderPrevA == 0 && encoderPrevB == 0 && pinA == 1 && pinB == 1)
		encoderPos += 1;
	else if (encoderPrevA == 1 && encoderPrevB == 1 && pinA == 0 && pinB == 0)
		encoderPos += 1;

	// change of direction
	else if (encoderPrevA == 1 && encoderPrevB == 0 && pinA == 0 && pinB == 0)
		encoderPos += 1;
	else if (encoderPrevA == 0 && encoderPrevB == 1 && pinA == 1 && pinB == 1)
		encoderPos += 1;
	else if (encoderPrevA == 0 && encoderPrevB == 0 && pinA == 1 && pinB == 0)
		encoderPos -= 1;
	else if (encoderPrevA == 1 && encoderPrevB == 1 && pinA == 0 && pinB == 1)
		encoderPos -= 1;

	//else if (serialDebug) Serial.println("Error: invalid rotary encoder pin state - prev=" + String(encoderPrevA) + ","
	//                                      + String(encoderPrevB) + " new=" + String(pinA) + "," + String(pinB));

	// update previous readings
	encoderPrevA = pinA;
	encoderPrevB = pinB;

//	if (encoderPos < lastEncoderPos - 1) {
//		lastEncoderPos = encoderPos;
//		encoderCCW = SET;
//	} else if (encoderPos > lastEncoderPos + 1) {
//		lastEncoderPos = encoderPos;
//		encoderCW = SET;
//	}
}

void Encoder(void) {
	if (HAL_GetTick() - lastEncoderTime > 100) {
		if (encoderPos < lastEncoderPos - 1) {
			lastEncoderPos = encoderPos;
			encoderCCW = SET;
			lastEncoderTime = HAL_GetTick();
//					buffer[6] = 0x1E;
		} else if (encoderPos > lastEncoderPos + 1) {
			lastEncoderPos = encoderPos;
			encoderCW = SET;
			lastEncoderTime = HAL_GetTick();
//					buffer[5] = 0x27;
		}
	}
}

void PushButtonUpdate(void) {
	static uint32_t lastPreased = 0;
	if (HAL_GetTick() - lastPreased >= 300) {
		lastPreased = HAL_GetTick();
		pushButtonState = SET;
	}
}

float Fconstrain(float value, float min, float max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}

int32_t constrain(int32_t value, int32_t min, int32_t max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}

void ssd1306_display(void) {
	if (pushButtonState && !ssd1306_Enable) {
		pushButtonState = RESET;
		ssd1306_Enable = SET;
		lastEncoderPos = encoderPos;
		LL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
	}

	if (ssd1306_Enable) {
		uint32_t i = HAL_GetTick();

		ssd1306_Fill(Black);
		const int8_t x[3] = { 29, 65, 101 };
		const int8_t x1[3] = { 19, 55, 91 };
		const int8_t x2[3] = { 15, 51, 87 };
		char num[6] = "";
		for (int i = 0; i < 3; ++i) {
			float y = Fconstrain((float) HE_MMvalue[i] / 4.0, 0.0, 1.0);
			ssd1306_FillRectangle(x[i], 49 - (int) (y * 38), x[i] + 8, 49,
					White);
			ssd1306_SetCursor(x1[i], 11);
			ssd1306_WriteChar(KEY[i] + 0x3D, Font_7x10, White);
			if (HE_Status[i]) {
				ssd1306_DrawBitmap(x1[i], 24, bitmap_Press, 7, 22, White);
			} else {
				ssd1306_DrawBitmap(x1[i], 24, bitmap_Realese, 7, 22, White);
			}
			snprintf(num, sizeof(num), "%.3f",
					Fconstrain(HE_MMvalue[i], 0.0, 4.0));
			ssd1306_SetCursor(x2[i], 0);
			ssd1306_WriteString(num, Font_6x8, White);
		}

		if (!ssd1306_displayEditing) {
			if (encoderCCW) {
				encoderCCW = RESET;
				ssd1306_displayIndex++;
				if (ssd1306_displayIndex > 2) {
					ssd1306_displayIndex = 0;
				}
			}

			if (encoderCW) {
				encoderCW = RESET;
				ssd1306_displayIndex--;
				if (ssd1306_displayIndex < 0) {
					ssd1306_displayIndex = 2;
				}
			}
		}

		switch (ssd1306_displayIndex) {
		case 0:
//			ssd1306_SetCursor(15, 54);
//			ssd1306_WriteString(RTcount, Font_7x10, White);
//			snprintf(num, sizeof(num), "%ld", constrain(HE_RTcount, 0, 999999));
//			ssd1306_SetCursor(71, 54);
//			ssd1306_WriteString(num, Font_7x10, White);
			if (pushButtonState) {
				pushButtonState = RESET;
				ssd1306_Enable = RESET;
				lastEncoderPos = encoderPos;
				LL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
				ssd1306_LOGO();
			}
			break;
		case 1:
		case 2:
			ssd1306_SetCursor(8, 54);
			ssd1306_WriteString(Trig, Font_7x10, White);
			ssd1306_SetCursor(78, 54);
			ssd1306_WriteString(RT, Font_7x10, White);

			snprintf(num, sizeof(num), "%.2f", HE_trigger);
			ssd1306_SetCursor(43, 54);
			ssd1306_WriteString(num, Font_7x10, White);

			snprintf(num, sizeof(num), "%.2f", HE_StatusChangeOffset);
			memmove(num, num + 1, strlen(num));
			ssd1306_SetCursor(99, 54);
			ssd1306_WriteString(num, Font_7x10, White);

			if (ssd1306_displayIndex == 1) {
				if (!ssd1306_displayEditing) {
					ssd1306_FillRectangle(33, 51, 46, 52, White);
					if (pushButtonState) {
						pushButtonState = RESET;
						ssd1306_displayEditing = SET;
					}
				} else {
					ssd1306_FillRectangle(8, 51, 71, 52, White);
					if (pushButtonState) {
						pushButtonState = RESET;
						ssd1306_displayEditing = RESET;
					}

					if (encoderCCW) {
						encoderCCW = RESET;
						HE_trigger += 0.01;
						if (HE_trigger > 3.00) {
							HE_trigger = 3.00;
						}
					}

					if (encoderCW) {
						encoderCW = RESET;
						HE_trigger -= 0.01;
						if (HE_trigger < 0.10) {
							HE_trigger = 0.10;
						}
					}
				}
			}

			if (ssd1306_displayIndex == 2) {
				if (!ssd1306_displayEditing) {
					ssd1306_FillRectangle(92, 51, 106, 52, White);
					if (pushButtonState) {
						pushButtonState = RESET;
						ssd1306_displayEditing = SET;
					}
				} else {
					ssd1306_FillRectangle(78, 51, 120, 52, White);
					if (pushButtonState) {
						pushButtonState = RESET;
						ssd1306_displayEditing = RESET;
					}

					if (encoderCCW) {
						encoderCCW = RESET;
						HE_StatusChangeOffset += 0.01;
						if (HE_StatusChangeOffset > 0.99) {
							HE_StatusChangeOffset = 0.99;
						}
					}

					if (encoderCW) {
						encoderCW = RESET;
						HE_StatusChangeOffset -= 0.01;
						if (HE_StatusChangeOffset < 0.10) {
							HE_StatusChangeOffset = 0.10;
						}
					}
				}
			}
			break;

		}
//		char buffer[20] = "";
//		snprintf(buffer, sizeof(buffer), "%lu", USBfailCOUNTER);
//		ssd1306_SetCursor(0, 54);
//		ssd1306_WriteString(buffer, Font_7x10, White);
		ssd1306_UpdateScreen();

		ssd1306_timeTaken = HAL_GetTick() - i;
	}
}

void ssd1306_LOGO(void) {
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(0, 0, bitmap_OSUlogo, 64, 64, White);
	ssd1306_SetCursor(68, 15);
	ssd1306_WriteString(RTosuPAD, Font_7x10, White);
	ssd1306_SetCursor(86, 27);
	ssd1306_WriteString(By, Font_7x10, White);
	ssd1306_SetCursor(65, 39);
	ssd1306_WriteString(fikr_hrd, Font_7x10, White);
	ssd1306_UpdateScreen();
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
