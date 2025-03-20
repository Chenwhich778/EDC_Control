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
#define Key2_Pin GPIO_PIN_2
#define Key2_GPIO_Port GPIOE
#define Key2_EXTI_IRQn EXTI2_IRQn
#define Key3_Pin GPIO_PIN_3
#define Key3_GPIO_Port GPIOE
#define Key3_EXTI_IRQn EXTI3_IRQn
#define PWM2_Pin GPIO_PIN_6
#define PWM2_GPIO_Port GPIOE
#define Direction_0_Pin GPIO_PIN_5
#define Direction_0_GPIO_Port GPIOF
#define Direction_1_Pin GPIO_PIN_6
#define Direction_1_GPIO_Port GPIOF
#define Direction_2_Pin GPIO_PIN_0
#define Direction_2_GPIO_Port GPIOC
#define Direction_3_Pin GPIO_PIN_1
#define Direction_3_GPIO_Port GPIOC
#define Direction_4_Pin GPIO_PIN_2
#define Direction_4_GPIO_Port GPIOC
#define Direction_5_Pin GPIO_PIN_3
#define Direction_5_GPIO_Port GPIOC
#define Direction_6_Pin GPIO_PIN_3
#define Direction_6_GPIO_Port GPIOA
#define OLED_SCK_Pin GPIO_PIN_13
#define OLED_SCK_GPIO_Port GPIOF
#define OLED_CS_Pin GPIO_PIN_12
#define OLED_CS_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_14
#define OLED_DC_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_8
#define OLED_RST_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_10
#define OLED_SDA_GPIO_Port GPIOD
#define bin1_Pin GPIO_PIN_7
#define bin1_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOB
#define bin2_Pin GPIO_PIN_8
#define bin2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOB
#define Key0_Pin GPIO_PIN_0
#define Key0_GPIO_Port GPIOE
#define Key0_EXTI_IRQn EXTI0_IRQn
#define Key1_Pin GPIO_PIN_1
#define Key1_GPIO_Port GPIOE
#define Key1_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
