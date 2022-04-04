/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

void uart1_idleHandler();
void uart1_dataHandler();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_SHUFFLE_Pin GPIO_PIN_11
#define SW_SHUFFLE_GPIO_Port GPIOB
#define SW_SHUFFLE_EXTI_IRQn EXTI4_15_IRQn
#define SW_LEARN_Pin GPIO_PIN_8
#define SW_LEARN_GPIO_Port GPIOA
#define SW_LEARN_EXTI_IRQn EXTI4_15_IRQn
#define LED_LEARN_Pin GPIO_PIN_9
#define LED_LEARN_GPIO_Port GPIOA
#define LED_DAY_Pin GPIO_PIN_10
#define LED_DAY_GPIO_Port GPIOA
#define LED_NIGHT_Pin GPIO_PIN_11
#define LED_NIGHT_GPIO_Port GPIOA
#define LED_TWILIGHT_Pin GPIO_PIN_12
#define LED_TWILIGHT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
