/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define MANUAL_DIR_Pin GPIO_PIN_0
#define MANUAL_DIR_GPIO_Port GPIOC
#define MANUAL_X_Pin GPIO_PIN_1
#define MANUAL_X_GPIO_Port GPIOC
#define MANUAL_X_EXTI_IRQn EXTI1_IRQn
#define MANUAL_Z_Pin GPIO_PIN_2
#define MANUAL_Z_GPIO_Port GPIOC
#define MANUAL_Z_EXTI_IRQn EXTI2_IRQn
#define MANUAL_C_Pin GPIO_PIN_3
#define MANUAL_C_GPIO_Port GPIOC
#define MANUAL_C_EXTI_IRQn EXTI3_IRQn
#define Z_ENC_A_Pin GPIO_PIN_0
#define Z_ENC_A_GPIO_Port GPIOA
#define Z_ENC_B_Pin GPIO_PIN_1
#define Z_ENC_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define C_STEP_Pin GPIO_PIN_6
#define C_STEP_GPIO_Port GPIOA
#define C_ENC_B_Pin GPIO_PIN_7
#define C_ENC_B_GPIO_Port GPIOA
#define X_DIR_Pin GPIO_PIN_0
#define X_DIR_GPIO_Port GPIOB
#define Z_DIR_Pin GPIO_PIN_1
#define Z_DIR_GPIO_Port GPIOB
#define C_DIR_Pin GPIO_PIN_2
#define C_DIR_GPIO_Port GPIOB
#define U_STEP_Pin GPIO_PIN_12
#define U_STEP_GPIO_Port GPIOB
#define U_DIR_Pin GPIO_PIN_13
#define U_DIR_GPIO_Port GPIOB
#define V_STEP_Pin GPIO_PIN_14
#define V_STEP_GPIO_Port GPIOB
#define V_DIR_Pin GPIO_PIN_15
#define V_DIR_GPIO_Port GPIOB
#define C_ENC_A_Pin GPIO_PIN_6
#define C_ENC_A_GPIO_Port GPIOC
#define MANUAL_U_Pin GPIO_PIN_7
#define MANUAL_U_GPIO_Port GPIOC
#define MANUAL_U_EXTI_IRQn EXTI9_5_IRQn
#define MANUAL_V_Pin GPIO_PIN_8
#define MANUAL_V_GPIO_Port GPIOC
#define MANUAL_V_EXTI_IRQn EXTI9_5_IRQn
#define C_ENC_ZERO_Pin GPIO_PIN_9
#define C_ENC_ZERO_GPIO_Port GPIOC
#define C_ENC_ZERO_EXTI_IRQn EXTI9_5_IRQn
#define X_ENC_A_Pin GPIO_PIN_8
#define X_ENC_A_GPIO_Port GPIOA
#define X_ENC_B_Pin GPIO_PIN_9
#define X_ENC_B_GPIO_Port GPIOA
#define STALLGUARD_X_Pin GPIO_PIN_10
#define STALLGUARD_X_GPIO_Port GPIOA
#define STALLGUARD_X_EXTI_IRQn EXTI15_10_IRQn
#define STALLGUARD_Z_Pin GPIO_PIN_11
#define STALLGUARD_Z_GPIO_Port GPIOA
#define STALLGUARD_Z_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define X_ENABLE_Pin GPIO_PIN_15
#define X_ENABLE_GPIO_Port GPIOA
#define V_ENABLE_Pin GPIO_PIN_10
#define V_ENABLE_GPIO_Port GPIOC
#define TX_FLAG_Pin GPIO_PIN_11
#define TX_FLAG_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Z_ENABLE_Pin GPIO_PIN_4
#define Z_ENABLE_GPIO_Port GPIOB
#define C_ENABLE_Pin GPIO_PIN_5
#define C_ENABLE_GPIO_Port GPIOB
#define U_ENABLE_Pin GPIO_PIN_6
#define U_ENABLE_GPIO_Port GPIOB
#define X_STEP_Pin GPIO_PIN_8
#define X_STEP_GPIO_Port GPIOB
#define Z_STEP_Pin GPIO_PIN_9
#define Z_STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
