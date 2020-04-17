/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ECHO_SENSOR_Pin GPIO_PIN_0
#define ECHO_SENSOR_GPIO_Port GPIOC
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Silnik2_Pin GPIO_PIN_1
#define Silnik2_GPIO_Port GPIOB
#define SIlnik2_Pin GPIO_PIN_2
#define SIlnik2_GPIO_Port GPIOB
#define Silnik2B10_Pin GPIO_PIN_10
#define Silnik2B10_GPIO_Port GPIOB
#define Silnik2B11_Pin GPIO_PIN_11
#define Silnik2B11_GPIO_Port GPIOB
#define Silnik1_Pin GPIO_PIN_12
#define Silnik1_GPIO_Port GPIOB
#define Silnik1B13_Pin GPIO_PIN_13
#define Silnik1B13_GPIO_Port GPIOB
#define Silnik1B14_Pin GPIO_PIN_14
#define Silnik1B14_GPIO_Port GPIOB
#define Silnik1B15_Pin GPIO_PIN_15
#define Silnik1B15_GPIO_Port GPIOB
#define TRIGGER_SENSOR_Pin GPIO_PIN_8
#define TRIGGER_SENSOR_GPIO_Port GPIOC
#define ECHO_SENSORC9_Pin GPIO_PIN_9
#define ECHO_SENSORC9_GPIO_Port GPIOC
#define TRIGGER_SENSORA8_Pin GPIO_PIN_8
#define TRIGGER_SENSORA8_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
