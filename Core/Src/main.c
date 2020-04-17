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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <limits.h>
void usDelay(uint32_t us)
{
	 __HAL_TIM_SET_COUNTER(&htim4, 0);
	  while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int sensor_time; //zmienna pomocnicza
int distance; //zmienna pomocnicze
int local_time=0; //zmienan pomocnicza
/* USER CODE END PTD */ 

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN PV */
#define ADXL345_DEVICE 0x53		//adres czujnika
#define ADXL345_POWER_CTL 0x2D	//wlaczenie
#define ADXL345_H_X 0x08		//mniej szczegołowe dane z osi x
#define ADXL345_H_Y 0x0B
#define ADXL345_H_Z 0x0E
#define ADXL345_RANGE (19.6246)	//aktualna maksymalna wartość mierzona

uint8_t Data = 0;	//przechowuje zczytane dane
int16_t Xaxis = 0;	//surowe dane z osi x
float XaxisMS = 0;	//dane z osi x w m/s^2
int16_t Yaxis = 0;
float YaxisMS = 0;
int16_t Zaxis = 0;
float ZaxisMS = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t hcsr04_read (void)
{
	local_time=0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // pull the TRIG pin low
	usDelay(2);  // wait for 2 us


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	usDelay(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // pull the TRIG pin low

	// read the time for which the pin is high

	while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)));  // wait for the ECHO pin to go high
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9))    // while the pin is high
	 {
		local_time++;   // measure time for which the pin is high
		usDelay (1);
	 }
	return local_time;
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
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  void usDelay(uint32_t us);
  uint8_t powerSet = 0x2;																//ustawienie bitow POWER_CTL
  HAL_I2C_Mem_Write(&hi2c1, ADXL345_DEVICE, ADXL345_POWER_CTL, 1, &powerSet, 1, 100);	//wpisanie bitow do rejestru

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  HAL_I2C_Mem_Read(&hi2c1, ADXL345_DEVICE, ADXL345_H_X, 1, &Data, 1, 100);	//zczytanie danych z osi x
	  Xaxis = Data << 8;														//zapisanie danych
	  XaxisMS = ((float)Xaxis*ADXL345_RANGE)/(float)INT16_MAX;					//przetworzenie na jednostke

	  HAL_I2C_Mem_Read(&hi2c1, ADXL345_DEVICE, ADXL345_H_Y, 1, &Data, 1, 100);
	  Yaxis = Data << 8;
	  YaxisMS = ((float)Yaxis*ADXL345_RANGE)/(float)INT16_MAX;

	  HAL_I2C_Mem_Read(&hi2c1, ADXL345_DEVICE, ADXL345_H_Z, 1, &Data, 1, 100);
	  Zaxis = Data << 8;
	  ZaxisMS = ((float)Zaxis*ADXL345_RANGE)/(float)INT16_MAX;
    /* USER CODE BEGIN 3 */
	 	  sensor_time = hcsr04_read();
	 	  distance  = sensor_time * .034/2;


	 	  HAL_Delay(200);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
