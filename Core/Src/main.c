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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UartRingbuffer.h"
//#include "Steppers.h"
#include "MyFunctions.h"
#include "Communication.hpp"
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

/* USER CODE BEGIN PV */
extern struct Stepper xStepper;
extern struct Stepper zStepper;
extern struct Stepper cStepper;
extern struct StepperUV uStepper;
extern struct StepperUV vStepper;

extern Cbool wasInitialized;
extern Mode workMode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);

  /* Inicjalizacja bufora kołowego i wszystkich komponentów */
  Ringbuf_init();
  InitializeComponents();

  HAL_Delay(200);

  /* Wysłanie wiadomości do Raspberry Pi o starcie programu */
  ChangePinState(TX_FLAG_GPIO_Port, TX_FLAG_Pin, 1);
  HAL_Delay(1);
  SendSingleChar('s');

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
}

/* USER CODE BEGIN 4 */

/**
 * Obsluga przerwania od glownego licznika
 * Wykonywanie glownej funkcji programu co okreslony okres - 500 mikrosekund
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == MAIN_TIMER)
	{
		MainFunction();
	}
	if (htim -> Instance == UV_TIMER)
	{
		UVStepperHandler(&uStepper);
		UVStepperHandler(&vStepper);
	}
}


/**
 * Obsluga przerwania od zakonczenia trwania pulsu PWM,
 * generowanie sygnalu STEP sterujacego ruchem silnikow krokowych X, Z, C
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == X_TIMER)
	{
		StepperHandler(&xStepper);
	}
	if (htim -> Instance == Z_TIMER)
	{
		StepperHandler(&zStepper);
	}
	if (htim -> Instance == C_TIMER)
	{
		StepperHandler(&cStepper);
	}
}



/**
 * Obsluga przerwan od zmiany stanu pinow zewnetrznych
 *  - wykrycie zderzenia STALLGUARD,
 *  - reczny ruch silnikami X, Z, C, U, V
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Zderzenie - wykrycie wzrostu momentu na silniku
	/*
	if (GPIO_Pin == STALLGUARD_X_Pin || (HAL_GPIO_ReadPin(STALLGUARD_X_GPIO_Port, STALLGUARD_X_Pin) == GPIO_PIN_SET)) // wykryto zderzenie w X
	{
		ChangeStepperState(&xStepper, stopped); //Zatrzymanie silnika
		if (xStepper.LastState == movingMinus) 	//Pozycja zerowa
		{
			xStepper.currentPosition = 0.0f;	//Wyzeruj pozycję
			xStepper.Steps = 0;
		}
		return;
	}

	if (GPIO_Pin == STALLGUARD_Z_Pin ||
			(HAL_GPIO_ReadPin(STALLGUARD_Z_GPIO_Port, STALLGUARD_Z_Pin) == GPIO_PIN_SET))	// wykryto zderzenie w X
	{
		ChangeStepperState(&zStepper, stopped); //Zatrzymanie silnika
		if (zStepper.LastState == movingMinus) 	//Pozycja zerowa
		{
			zStepper.currentPosition = 0.0f;	//Wyzeruj pozycję
			zStepper.Steps = 0;
		}
		return;
	}
*/
	// Reczny ruch osiami
	if (wasInitialized == TRUE && workMode == manual)
	{
		switch (GPIO_Pin)
		{
		// -------------------X------------------
		case MANUAL_X_Pin:
			if (xStepper.isLocked == FALSE)
			{
				if (HAL_GPIO_ReadPin(MANUAL_X_GPIO_Port, MANUAL_X_Pin) == GPIO_PIN_RESET)
					ChangeStepperState(&xStepper, stopped);
				else
				{
					if (HAL_GPIO_ReadPin(MANUAL_DIR_GPIO_Port, MANUAL_DIR_Pin) == GPIO_PIN_SET)
						ChangeStepperState(&xStepper, movingPlus);
					else
						ChangeStepperState(&xStepper, movingMinus);
				}
			}

			break;

		// -------------------Z------------------
		case MANUAL_Z_Pin:
			if (zStepper.isLocked == FALSE)
			{
				if (HAL_GPIO_ReadPin(MANUAL_Z_GPIO_Port, MANUAL_Z_Pin) == GPIO_PIN_RESET)
					ChangeStepperState(&zStepper, stopped);
				else
				{
					if (HAL_GPIO_ReadPin(MANUAL_DIR_GPIO_Port, MANUAL_DIR_Pin) == GPIO_PIN_SET)
						ChangeStepperState(&zStepper, movingPlus);
					else
						ChangeStepperState(&zStepper, movingMinus);
				}
			}
			break;

		// -------------------C------------------
		case MANUAL_C_Pin:
			if (cStepper.isLocked == FALSE)
			{
				if (HAL_GPIO_ReadPin(MANUAL_C_GPIO_Port, MANUAL_C_Pin) == GPIO_PIN_RESET)
					ChangeStepperState(&cStepper, stopped);
				else
				{
					if (HAL_GPIO_ReadPin(MANUAL_DIR_GPIO_Port, MANUAL_DIR_Pin) == GPIO_PIN_SET)
						ChangeStepperState(&cStepper, movingPlus);
					else
						ChangeStepperState(&cStepper, movingMinus);
				}
			}
			break;

		// -------------------U------------------
		case MANUAL_U_Pin:
			if (HAL_GPIO_ReadPin(MANUAL_U_GPIO_Port, MANUAL_U_Pin) == GPIO_PIN_RESET)
				ChangeStepperUVState(&uStepper, stopped);
			else
			{
				if (HAL_GPIO_ReadPin(MANUAL_DIR_GPIO_Port, MANUAL_DIR_Pin) == GPIO_PIN_SET)
					ChangeStepperUVState(&uStepper, movingPlus);
				else
					ChangeStepperUVState(&uStepper, movingMinus);
			}
			break;

		// -------------------V------------------
		case MANUAL_V_Pin:
			if (HAL_GPIO_ReadPin(MANUAL_V_GPIO_Port, MANUAL_V_Pin) == GPIO_PIN_RESET)
				ChangeStepperUVState(&vStepper, stopped);
			else
			{
				if (HAL_GPIO_ReadPin(MANUAL_DIR_GPIO_Port, MANUAL_DIR_Pin) == GPIO_PIN_SET)
					ChangeStepperUVState(&vStepper, movingPlus);
				else
					ChangeStepperUVState(&vStepper, movingMinus);
			}
			break;
		}
	}
}
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
