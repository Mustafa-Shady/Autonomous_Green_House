/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/********************************************************************************/
/**************************Including External Libraries**************************/
/********************************************************************************/
#include <stdio.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MY_DHT22.h"
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/********************************************************************************/
/**********************************Task Handlers*********************************/
/********************************************************************************/
TaskHandle_t xDHT_Reading_Handler;
TaskHandle_t xSoilMoisture_Reading_Handler;
TaskHandle_t xWaterLevel_Reading_Handler;

TaskHandle_t xHR_Control_System;
TaskHandle_t xTemp_Control_System;
TaskHandle_t xIrrigation_Control_System;
TaskHandle_t xWaterTank_Control_System;

TaskHandle_t xUART_Transmit;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/********************************************************************************/
/*******************************Tasks' Prototypes********************************/
/********************************************************************************/
void vDHT11_Reading(void *pvParameters);
void vSoilMoisture_Reading(void *pvParameters);
void vWaterLevel_Reading(void *pvParameters);

void vHR_Control_System(void *pvParameters);
void vTemp_Control_System(void *pvParameters);
void vIrrigation_Control_System(void *pvParameters);
void vWaterTank_Control_System(void *pvParameters);

void vUART_Transmit(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/********************************************************************************/
/*******************************Global Variables*********************************/
/********************************************************************************/
DHT_DataTypedef DHT11_Data;   //Struct of Type DHT_DataTypedef To Receive the Reading of DHT11
uint8_t Temperature, Humidity;  //2 Float Global Variables for Holding Temp and Humidity Readings from DHT11

float SoilMoisture;         //Global u8 Variable to Read the ADC Output of the SoilMoisture Sensor
uint8_t SM_Percentage;        //Global u8 Variable to Hold the Mapped Value of Soil Moisture Sensor (0~100)%
uint8_t Water_Tank_Level;    //Global u8 Variable to Read the Analog Output of the Bottom Water Level Sensor

uint8_t Cooling_System_Flag      = 0;	 //Global Flag to Activate the Cooling System
uint8_t Heating_System_Flag      = 0; 	 //Global Flag to Activate the Heating System
uint8_t Irrigation_System_Flag   = 0; 	 //Global Flag to Activate the Irrigation System
uint8_t Humidifier_System_Flag   = 0;	 //Global Flag to Activate the Humidifier System
uint8_t DeHumidifier_System_Flag = 0;	 //Global Flag to Activate the DeHumidifier System
uint8_t WaterTank_System_Flag    = 0;    //Global Flag to Activate the Water Tank Filling System

char Global_u8Data[50] = {0};	 /* Global Array to send Data to uart */


ADC_ChannelConfTypeDef sConfig = {0}; //Global Variable for Selecting ADC Channel

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /********************************************************************************/
  /*******************************Creation of Tasks********************************/
  /********************************************************************************/
  xTaskCreate(vDHT11_Reading, "DHT11_READING", 64, NULL, 1, &xDHT_Reading_Handler);
  xTaskCreate(vSoilMoisture_Reading, "SoilMoisture_READING", 64, NULL, 1, &xSoilMoisture_Reading_Handler);
  xTaskCreate(vWaterLevel_Reading, "WaterLevel_READING", 64, NULL, 1, &xWaterLevel_Reading_Handler);

  xTaskCreate(vHR_Control_System, "HR_Contorl_System", 64, NULL, 1, &xHR_Control_System);
  xTaskCreate(vTemp_Control_System, "Temp_Contorl_System", 64, NULL, 1, &xTemp_Control_System);
  xTaskCreate(vIrrigation_Control_System, "Irrigation_Contorl_System", 64, NULL, 1, &xIrrigation_Control_System);
  xTaskCreate(vWaterTank_Control_System, "WaterTank_Control_System", 64, NULL, 1, &xWaterTank_Control_System);

  xTaskCreate(vUART_Transmit, "UART_Transmit", 64, NULL, 1, &xUART_Transmit);

  /* Start Scheduler */
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB12
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/********************************************************************************/
/******************************Tasks' Implementation*****************************/
/********************************************************************************/
void vDHT11_Reading(void *pvParameters)
{
	while(1)
	{
	  DHT_GetData(&DHT11_Data);
	  Temperature = DHT11_Data.Temperature;
	  Humidity = DHT11_Data.Humidity;
	  vTaskDelay(15);
	}
}

void vSoilMoisture_Reading(void *pvParameters)
{
	while(1)
	{
		sConfig.Channel = ADC_CHANNEL_6; //Switch To Channel 6

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) //Call Channel Configuration Function
	    {
	    	Error_Handler();
	    }

		HAL_ADC_Start(&hadc1);	//Start Conversion sync and Passing the ADC Handler

		if(HAL_ADC_PollForConversion(&hadc1,20) == HAL_OK); //Busy Wait till end of Conversion

		SoilMoisture = HAL_ADC_GetValue(&hadc1); //Read the ADC Value

		vTaskDelay(16);
	}
}

void vWaterLevel_Reading(void *pvParameters)
{
	while(1)
	{
		sConfig.Channel = ADC_CHANNEL_4; //Switch To Channel 4

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) //Call Channel Configuration Function
	    {
	    	Error_Handler();
	    }

		HAL_ADC_Start(&hadc1);	//Start Conversion sync and Passing the ADC Handler

		if(HAL_ADC_PollForConversion(&hadc1,20) == HAL_OK); //Busy Wait till end of Conversion

		Water_Tank_Level = HAL_ADC_GetValue(&hadc1); //Read the ADC Value

		vTaskDelay(17);
	}
}

void vHR_Control_System(void *pvParameters)
{
	while(1)
	{
		if((Humidifier_System_Flag == 0) && (DeHumidifier_System_Flag == 0))
		{
			if(Humidity > 70)
			{
				/* Activate DeHumidifier System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);

				DeHumidifier_System_Flag = 1;
			}
			else if (Humidity < 40)
			{
				/* Activate Humidifier System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

				Humidifier_System_Flag = 1;
			}
		}
		else if ((Humidity > 50) && (Humidity < 60))
		{
			if (Humidifier_System_Flag)
			{
				/* Stop Humidifier System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

				Humidifier_System_Flag = 0;
			}
			else if (DeHumidifier_System_Flag)
			{
				/* Stop DeHumidifier System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);

				DeHumidifier_System_Flag = 0;
			}
		}

		vTaskDelay(18);
	}
}

void vTemp_Control_System(void *pvParameters)
{
	while(1)
	{
		if((Heating_System_Flag == 0) && (Cooling_System_Flag == 0))
		{
			if(Temperature > 40)
			{
				/* Activate Cooling System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1); //In take
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); //Suction

				Cooling_System_Flag = 1;
			}
			else if (Temperature < 30)
			{
				/* Activate Heating System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

				Heating_System_Flag = 1;
			}
		}
		else if ((Temperature > 34) || (Temperature < 36))
		{
			if (Heating_System_Flag)
			{
				/* Stop Heating System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);

				Heating_System_Flag = 0;
			}
			else if (Cooling_System_Flag)
			{
				/* Stop Cooling System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0); //In take
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); //Suction

				Cooling_System_Flag = 0;
			}
		}

		vTaskDelay(19);
	}
}

void vIrrigation_Control_System(void *pvParameters)
{
	while(1)
	{
		if (SoilMoisture > 200) {SoilMoisture=200;}
	    if (SoilMoisture < 140) {SoilMoisture=140;}

		SM_Percentage = (((200 - SoilMoisture)/60)*100); //Remove Negative

		if (Irrigation_System_Flag == 0)
		{
			if(SM_Percentage < 40)
			{
				/* Activate Irrigation System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);

				Irrigation_System_Flag = 1;
			}
		}
		else
		{
			if(SM_Percentage > 55)
			{
				/* Deactivate Irrigation System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);

				Irrigation_System_Flag = 0;
			}
		}

		vTaskDelay(20);
	}
}

void vWaterTank_Control_System(void *pvParameters)
{
	while(1)
	{
		if (WaterTank_System_Flag == 0)
		{
			if(Water_Tank_Level < 50)
			{
				/* Activate Water Tank Fill System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

				WaterTank_System_Flag = 1;
			}
		}
		else
		{
			if(Water_Tank_Level > 200)
			{
				/* Activate Water Tank Fill System */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);

				WaterTank_System_Flag = 0;
			}
		}

		vTaskDelay(21);
	}
}

void vUART_Transmit(void *pvParameters)
{
	while(1)
	{

		sprintf(Global_u8Data, "%d\r\n", Humidity);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", Humidifier_System_Flag);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", DeHumidifier_System_Flag);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", Temperature);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", Heating_System_Flag);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", Cooling_System_Flag);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", SM_Percentage);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", Irrigation_System_Flag);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r\n", WaterTank_System_Flag);
		HAL_UART_Transmit(&huart1,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", Humidity);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", Humidifier_System_Flag);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", DeHumidifier_System_Flag);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", Temperature);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", Heating_System_Flag);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", Cooling_System_Flag);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", SM_Percentage);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", Irrigation_System_Flag);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		sprintf(Global_u8Data, "%d\r", WaterTank_System_Flag);
		HAL_UART_Transmit(&huart2,(char *) Global_u8Data, strlen(Global_u8Data),3);

		vTaskDelay(22);
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
