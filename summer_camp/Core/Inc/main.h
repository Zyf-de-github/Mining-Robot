/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define motor_control_Pin GPIO_PIN_0
#define motor_control_GPIO_Port GPIOC
#define motor_controlC1_Pin GPIO_PIN_1
#define motor_controlC1_GPIO_Port GPIOC
#define motor_controlC2_Pin GPIO_PIN_2
#define motor_controlC2_GPIO_Port GPIOC
#define motor_controlC3_Pin GPIO_PIN_3
#define motor_controlC3_GPIO_Port GPIOC
#define pwm_ch2_Pin GPIO_PIN_1
#define pwm_ch2_GPIO_Port GPIOA
#define pwm_ch3_Pin GPIO_PIN_2
#define pwm_ch3_GPIO_Port GPIOA
#define pwm_ch4_Pin GPIO_PIN_3
#define pwm_ch4_GPIO_Port GPIOA
#define motor_controlC4_Pin GPIO_PIN_4
#define motor_controlC4_GPIO_Port GPIOC
#define motor_controlC5_Pin GPIO_PIN_5
#define motor_controlC5_GPIO_Port GPIOC
#define motor_controlC10_Pin GPIO_PIN_10
#define motor_controlC10_GPIO_Port GPIOC
#define motor_controlC11_Pin GPIO_PIN_11
#define motor_controlC11_GPIO_Port GPIOC
#define up_motor_control_Pin GPIO_PIN_5
#define up_motor_control_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
