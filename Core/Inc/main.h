/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define User_Bottom_Pin GPIO_PIN_13
#define User_Bottom_GPIO_Port GPIOC
#define XR_Switch_Sensor_Pin GPIO_PIN_3
#define XR_Switch_Sensor_GPIO_Port GPIOF
#define XR_Switch_Sensor_EXTI_IRQn EXTI3_IRQn
#define YDirection_Pin GPIO_PIN_4
#define YDirection_GPIO_Port GPIOF
#define YEnable_Pin GPIO_PIN_5
#define YEnable_GPIO_Port GPIOF
#define XDirection_Pin GPIO_PIN_7
#define XDirection_GPIO_Port GPIOE
#define XL_Switch_Sensor_Pin GPIO_PIN_11
#define XL_Switch_Sensor_GPIO_Port GPIOE
#define XL_Switch_Sensor_EXTI_IRQn EXTI15_10_IRQn
#define XEnable_Pin GPIO_PIN_15
#define XEnable_GPIO_Port GPIOE
#define ZEnable_Pin GPIO_PIN_10
#define ZEnable_GPIO_Port GPIOD
#define YSwitch_Bottom_Pin GPIO_PIN_12
#define YSwitch_Bottom_GPIO_Port GPIOD
#define YSwitch_Bottom_EXTI_IRQn EXTI15_10_IRQn
#define YSwitch_Front_Pin GPIO_PIN_13
#define YSwitch_Front_GPIO_Port GPIOD
#define YSwitch_Front_EXTI_IRQn EXTI15_10_IRQn
#define ZDirection_Pin GPIO_PIN_5
#define ZDirection_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
