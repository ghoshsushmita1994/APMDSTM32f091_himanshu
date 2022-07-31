/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/*
 * main.c
 *
 *  Created on: May 12 , 2022
 *  Device Specification : STM32F091RCT6
 *      Author: Harshita Jain
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT.h"
#include "Gas_Data.h"
#include "ESPDataLogger.h"
#include "UartRingbuffer.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define n_PARAMS 11
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

float buffer[n_PARAMS];

ADC_ChannelConfTypeDef sConfig;


float PM1,PM2,PM3,PM4,PM5;

#define PORTB GPIOB
#define MDM_1V8 GPIO_PIN_15
#define PORTA GPIOA
#define MDM_PWR GPIO_PIN_8
#define PCON_PM GPIO_PIN_15
#define PORTC GPIOC
#define PCON_DHT GPIO_PIN_6
#define PCON_AFE2 GPIO_PIN_7
#define PCON_AFE1 GPIO_PIN_8
#define PCON_FAN GPIO_PIN_9
#define ESP_CH_EN GPIO_PIN_11



/***************************
 * printf() Function
 * For receiving data on Serial terminal
 * UART3 RX TX
 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	char str[15];
//	int sample_count = 1;
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
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */

  //HAL_GPIO_WritePin(GPIOC, PCON_AFE1, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOC, ESP_CH_EN, GPIO_PIN_SET);

//  HAL_Delay(100);
  HAL_UART_Transmit(&huart3, (uint8_t*)"Here1", sizeof("Here1"), HAL_MAX_DELAY);
////  ESP_RST();
//  ESP_Init();

  HAL_TIM_Base_Start(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/*****************************  POWERING THE SENSOR THROUGH GPIO PIN **************************/

	HAL_GPIO_WritePin(GPIOC, PCON_DHT, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, PCON_AFE1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, PCON_PM, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, ESP_CH_EN, GPIO_PIN_SET);
	HAL_Delay(5000);
	//HAL_UART_Transmit(&huart3, (uint8_t*)"Here1", sizeof("Here1"), HAL_MAX_DELAY);
	HAL_Delay(100);



	float a = 0.00; float b = 0.00; float c = 0.00; float d = 0.00; float e = 0.00; float f = 0.00;
	float p1 = 0.00; float p2 = 0.00; float p3 = 0.00; float p4 = 0.00; float p5 = 0.00;

    uint8_t k;
    for (k=0; k<1; k++)	// 5
    {

	/****************************  DHT Data Read *****************************************/

  	DHT11_Start();
  	Presence = DHT11_Check_Response();
  	Rh_byte1 = DHT11_Read ();
  	Rh_byte2 = DHT11_Read ();
  	Temp_byte1 = DHT11_Read ();
  	Temp_byte2 = DHT11_Read ();
  	SUM = DHT11_Read();

  	TEMP = Temp_byte1;
  	RH = Rh_byte1;

  	Temperature = (float) TEMP;
  	a=a+Temperature;
  	Humidity = (float) RH;
  	b=b+Humidity;
  	printf("Temperature: %f \r\n", Temperature);
	printf("Humidity: %f \r\n",	Humidity);


	/*************************** Gas Sensor Data Read *******************************/

	float N02_value =  NO2_Read(TEMP);					//NO2_read_value; 		//ppb value
	float NO2_1= N02_value*1.88;						//ug/m^3 value
	c=c+NO2_1;
	printf("NO2 value in ug/m^3: %.4f\r\n", NO2_1);

	float OX_value =  OX_Read(TEMP); 					//ppb value
	float OX_1= OX_value*2.00;							//ug/m^3 value
	d=d+OX_1;
	printf("OX value in ug/m^3: %.4f \r\n" ,OX_1);

	float CO_value =  CO_Read(TEMP); 					//ppb value
	float CO_1= CO_value*1.145;							//ug/m^3 value
	e=e+CO_1;
	printf("CO value in mg/m^3 %.4f \r\n" ,CO_1);

	float SO2_value = SO2_Read(TEMP); 					//ppb value
	float SO2_1= SO2_value*2.62;						//ug/m^3 value
	f=f+SO2_1;
	printf("SO2 value in ug/m^3: %.4f \r\n" ,SO2_1);


	/*************************** Gas Sensor Data Read *******************************/

	PM_Read(&PM1, &PM2, &PM3, &PM4, &PM5);
	p1=p1+PM1; p2=p2+PM2; p3=p3+PM3; p4=p4+PM4; p5=p5+PM5;
	printf("PM1= %f\r\n", PM1);
	printf("PM2= %f\r\n", PM2);
	printf("PM3= %f\r\n", PM3);
	printf("PM4= %f\r\n", PM4);
	printf("PM5= %f\r\n", PM5);
	HAL_Delay(1000);

    }

    a=a/5; b=b/5; c=c/5; d=d/5; e=e/5; f=f/5; p1=p1/5; p2=p2/5; p3=p3/5; p4=p4/5; p5=p5/5;
    a=0.45*a+19; b=0.813*b+0.99;
    //g1=0.0022*g*g-2.54*g+762.2;
    //h1=1.9*g+9.82;



	/**************** Sending Data to ThingsSpeak ********************************/

//	buffer[0]= 	Temperature;
//	buffer[1]=	Humidity;
//	buffer[2]=	NO2_1;
//	buffer[3]=	OX_1;
//	buffer[4]=	CO_1;
//	buffer[5]=	SO2_1;
//	buffer[6]=	PM1;
//	buffer[7]=	PM2;
//	buffer[8]=	PM3;
//	buffer[9]=	PM4;
//	buffer[10]=	PM5;

	buffer[0]= 	a;
	buffer[1]=	b;
	buffer[2]=	c;
	buffer[3]=	d;
	buffer[4]=	e;
	buffer[5]=	f;
	buffer[6]=	p1;
	buffer[7]=	p2;
	buffer[8]=	p3;
	buffer[9]=	p4;
	buffer[10]=	p5;


	ESP_Init();
	HAL_Delay(2000);
//	printf("Here\r\n");

	ESP_Send_Data(buffer);

	HAL_Delay(2000);


	//sample_count++;


	/************* After 5 sample has been transfer to ThinkSpeak,
	 * Microcontroller will sleep for 3 Minute*********/

	//if (sample_count > 5)
	//{


		/******* Powering Off the sensor ****************/
		HAL_GPIO_WritePin(GPIOC, PCON_DHT, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, PCON_AFE1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, PCON_PM, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, ESP_CH_EN, GPIO_PIN_RESET);

		HAL_Delay(30000); 		//4 Minute

		//sample_count =	1;


	//}


 }  /*while loop end*/

}   /*main loop end*/





/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }


  /**********************************
   * ADC channel are redefine in Gas_Data.c,.h File
   ******************/


  /** Configure for the selected ADC regular channel to be converted. 
  */
//  sConfig.Channel = ADC_CHANNEL_0;
//  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_5;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_6;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_7;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_8;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_12;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_13;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_14;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted.
//  */
//  sConfig.Channel = ADC_CHANNEL_15;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */

/******************
 * MCU Clock Frequecy is 48MHz
 * Prescaler - 48
 * ARR - 65536
 * Will generate 1us delay.
 */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65536-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 
                           PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
