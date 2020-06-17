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

#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define n 100 //rozmiar historii pomiarów
/*
#define ADXL345_DEVICE_W 0xA6 // adres urządzenia w trybie zapisu
#define ADXL345_DEVICE_R 0xA7 // adres urządzenia w trybie odczytu
#define ADXL345_POWER 0x2D
#define ADXL345_X 0x32 // rejestr przyspieszenia na OX (LSB)
#define ADXL345_Y 0x34 // rejestr przyspieszenia na OY (LSB)
//#define ADXL345_RANGE 0x31
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int komenda;

volatile _Bool ECHOL_High;
volatile _Bool ECHOP_High;
volatile uint32_t ECHOL_Pulse; // surowy pomiar z lewego czujnika odległości
volatile uint32_t ECHOP_Pulse; // surowy pomiar z prawego czujnika odległości
volatile int nP; // wskaźnik historii
volatile _Bool resetuj;

//float XaxisMS;
//float YaxisMS;
int sL[n][2]; //odleglości z lewego czujnika
int sP[n][2]; //odleglości z prawego czunika
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
	return len;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ECHOP_Pin)
	{
		ECHOP_High=!ECHOP_High;
		if(ECHOP_High)
			ECHOP_Pulse=DWT->CYCCNT;
		else
			ECHOP_Pulse=DWT->CYCCNT-ECHOP_Pulse;
	}

	if(GPIO_Pin == ECHOL_Pin){
		ECHOL_High=!ECHOL_High;
		if(ECHOL_High)
			ECHOL_Pulse=DWT->CYCCNT;
		else
			ECHOL_Pulse=DWT->CYCCNT-ECHOL_Pulse;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
// zapis do urządzenia
void ADXL345_zapis(uint8_t adresRejestru, uint16_t ileDanych, uint8_t wskDane)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x53, adresRejestru, 1, &wskDane, ileDanych, 100);
}

// odczyt z urządzenia
// \return uint8_t dane zwrotne
uint8_t ADXL345_odczyt(uint8_t adresRejestru, uint16_t ileDanych)
{
	uint8_t bufor;
	HAL_I2C_Mem_Read(&hi2c1, 0x53, adresRejestru, 1, &bufor, ileDanych, 100);
	return bufor;
}
*/

void initDWT()
{
	CoreDebug->DEMCR &= ~0x01000000;
	CoreDebug->DEMCR |=  0x01000000;
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |=  0x00000001;
	DWT->CYCCNT = 0;
}
/*
void initADXL345()
{
	ADXL345_zapis(ADXL345_POWER, 1, 0x08);
}
*/
void initSTEP()
{
	HAL_GPIO_WritePin(MP_POW_GPIO_Port, MP_POW_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ML_POW_GPIO_Port, ML_POW_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_1) ;
	HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_4) ;
}

void initTRIGER()
{
	HAL_TIM_PWM_Start(&htim2 , TIM_CHANNEL_1) ;
	HAL_TIM_PWM_Start(&htim2 , TIM_CHANNEL_2) ;
}

void sprawdzRadio()
{
	int radio1=HAL_GPIO_ReadPin(KOMENDA1_GPIO_Port, KOMENDA1_Pin);
	int radio2=HAL_GPIO_ReadPin(KOMENDA2_GPIO_Port, KOMENDA2_Pin);
	int radio3=HAL_GPIO_ReadPin(KOMENDA3_GPIO_Port, KOMENDA3_Pin);
	int radio4=HAL_GPIO_ReadPin(KOMENDA4_GPIO_Port, KOMENDA4_Pin);
	  if(radio1 && !radio2 && radio3 && !radio4)
		  komenda=1; //gora
	  else if(!radio1 && radio2 && !radio3 && radio4)
		  komenda=0; //dol
	  else if(radio1 && !radio2 && !radio3 && radio4)
		  komenda=2; //prawo
	  else if(!radio1 && radio2 && radio3 && !radio4)
		  komenda=3; //lewo
}

void jazda(int kierunek)
{
	if (kierunek==0){ //stop
		HAL_GPIO_WritePin(MP_POW_GPIO_Port, MP_POW_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ML_POW_GPIO_Port, ML_POW_Pin, GPIO_PIN_SET);
	}else { //jazda
		if(kierunek==1){ //prosto
			HAL_GPIO_WritePin(ML_DIR_GPIO_Port, ML_DIR_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MP_DIR_GPIO_Port, MP_DIR_Pin, GPIO_PIN_SET);
		}
		else if(kierunek==2){ //prawo
			HAL_GPIO_WritePin(ML_DIR_GPIO_Port, ML_DIR_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MP_DIR_GPIO_Port, MP_DIR_Pin, GPIO_PIN_RESET);
		}
		else if(kierunek==3){ //lewo
			HAL_GPIO_WritePin(ML_DIR_GPIO_Port, ML_DIR_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MP_DIR_GPIO_Port, MP_DIR_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(MP_POW_GPIO_Port, MP_POW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ML_POW_GPIO_Port, ML_POW_Pin, GPIO_PIN_RESET);
	}
}

void pokazPomiar()
{
	int sL=ECHOL_Pulse/64/58;
	int sP=ECHOP_Pulse/64/58;
	if(sL>320)
		sL=320;
	if(sP>320)
		sP=320;

	//printf("Przyspieszenie : %f X | %f Y | m/s^2 \r\n", XaxisMS, YaxisMS);
	printf("%i L | %i P | cm         \r\n\n", sL, sP);
}

void rejestruj()
{
	int P=ECHOP_Pulse/64/58; //odległość w cm
	int L=ECHOL_Pulse/64/58; //odległość w cm
	//wykrywamy błędne dane i zawieszenia
	if(P>500 || L>500)
	{
		resetuj=1;
		return ;
	}
	else if(P<=0 || L<=0 )
		return ;
	else
		resetuj=0;

	//resetuje jezeli blad sie powtorzy
	if(resetuj==1)
	{
		NVIC_SystemReset();
		HAL_Delay(1000);
		resetuj=0;
		return;
	}

	//zapisz dane do histori
	if(nP<n){
		sP[nP][0]=P;
		sL[nP][0]=L;
		sP[nP][1]=0;
		sL[nP][1]=0;
		nP++;
	}else
	{
		sP[nP%n][1]=P;
		sL[nP%n][1]=L;
		sP[nP%n][0]=0;
		sL[nP%n][0]=0;
		nP++;
	}

	//cofnij wskaźnik
	if(nP==2*n)
		nP=0;
}

void decyduj()
{
	int P[n];
	int L[n];
	int i=0, maxP=0, maxPid=0, maxL=0, maxLid=0;
	P[0]=0;
	L[0]=0;

	// Układa dane chronologicznie w tablicy
	if(sP[0][0]>0)
	{
		for(int j=n-1; j>=0; j--)
			if(sP[j][0]>0)
				P[i++]=sP[j][0];
		for(int j=n-1; j>=0; j--)
			if(sP[j][1]>0)
				P[i++]=sP[j][1];
	}else
	{
		for(int j=n-1; j>=0; j--)
			if(sP[j][1]>0)
				P[i++]=sP[j][1];
		for(int j=n-1; j>=0; j--)
			if(sP[j][0]>0)
				P[i++]=sP[j][0];
	}

	i=0;

	if(sL[0][0]>0)
	{
		for(int j=n-1; j>=0; j--)
			if(sL[j][0]>0)
				L[i++]=sL[j][0];
		for(int j=n-1; j>=0; j--)
			if(sL[j][1]>0)
				L[i++]=sL[j][1];
	}else
	{
		for(int j=n-1; j>=0; j--)
			if(sL[j][1]>0)
				L[i++]=sL[j][1];
		for(int j=n-1; j>=0; j--)
			if(sL[j][0]>0)
				L[i++]=sL[j][0];
	}

	// podstawowy algorytm
	if(komenda==1 && P[0]!=0 && L[0]!=0)
	{
		//szukanie większej przestrzeni
		for(int j=0; j<n; j++)
			if(maxP<P[j])
			{
				maxP=P[j];
				maxPid=j;
			}

		for(int j=0; j<n; j++)
			if(maxL<L[j])
			{
				maxL=L[j];
				maxLid=j;
			}



		//zabezpieczona komenda jedź
		if(P[0]>30 && L[0]>30)
			jazda(1);

		//skręt w razie ściany
		if(P[0]<=30 || L[0]<=30)
		{
			if(maxP>maxL)
			{
				jazda(2);
				HAL_Delay(maxPid*100);
				jazda(0);
			}else{
				jazda(3);
				HAL_Delay(maxLid*100);
				jazda(0);
			}
			if(P[0]>30 && L[0]>30)
				jazda(1);
		}
	}
}
/*
void odczytPrzys()
{
	int16_t Xaxis = 1, Yaxis = 1;
	int8_t X = 1, Y = 1;
	X=ADXL345_odczyt(ADXL345_X, 1);
	Xaxis = X << 8;
	XaxisMS = ((float)Xaxis*19.8)/(float)INT16_MAX;

	Y=ADXL345_odczyt(ADXL345_Y, 1);
	Yaxis = Y << 8;
	YaxisMS = ((float)Yaxis*19.8)/(float)INT16_MAX;
}
*/
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  initSTEP();
  //initADXL345();
  initDWT();
  initTRIGER();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sprawdzRadio();
	  if(komenda==0)
		  jazda(0);
	  else if(komenda==2)
		  jazda(2);
	  else if(komenda==3)
		  jazda(3);

	  rejestruj();

	  decyduj();

	  HAL_Delay(20);
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
