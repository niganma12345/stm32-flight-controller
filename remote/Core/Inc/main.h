/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define LH_Pin GPIO_PIN_0
#define LH_GPIO_Port GPIOA
#define LV_Pin GPIO_PIN_1
#define LV_GPIO_Port GPIOA
#define RH_Pin GPIO_PIN_2
#define RH_GPIO_Port GPIOA
#define RV_Pin GPIO_PIN_3
#define RV_GPIO_Port GPIOA
#define K1_Pin GPIO_PIN_4
#define K1_GPIO_Port GPIOA
#define K2_Pin GPIO_PIN_5
#define K2_GPIO_Port GPIOA
#define K3_Pin GPIO_PIN_6
#define K3_GPIO_Port GPIOA
#define K4_Pin GPIO_PIN_7
#define K4_GPIO_Port GPIOA
#define K5_Pin GPIO_PIN_0
#define K5_GPIO_Port GPIOB
#define K6_Pin GPIO_PIN_1
#define K6_GPIO_Port GPIOB
#define M_SCL_Pin GPIO_PIN_10
#define M_SCL_GPIO_Port GPIOB
#define M_SDA_Pin GPIO_PIN_11
#define M_SDA_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_12
#define CSN_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_9
#define CE_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
