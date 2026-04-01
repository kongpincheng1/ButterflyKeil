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
#include "stm32g0xx_hal.h"

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
#define OSC_IN_Pin GPIO_PIN_14
#define OSC_IN_GPIO_Port GPIOC
#define OSC_EN_Pin GPIO_PIN_15
#define OSC_EN_GPIO_Port GPIOC
#define DTX_Pin GPIO_PIN_0
#define DTX_GPIO_Port GPIOA
#define DRX_Pin GPIO_PIN_1
#define DRX_GPIO_Port GPIOA
#define BAT_Pin GPIO_PIN_2
#define BAT_GPIO_Port GPIOA
#define ANG1_Pin GPIO_PIN_3
#define ANG1_GPIO_Port GPIOA
#define ANG2_Pin GPIO_PIN_4
#define ANG2_GPIO_Port GPIOA
#define ANG3_Pin GPIO_PIN_5
#define ANG3_GPIO_Port GPIOA
#define ANG4_Pin GPIO_PIN_6
#define ANG4_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_0
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_1
#define PWM4_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_8
#define DIR1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_11
#define DIR2_GPIO_Port GPIOA
#define DIR3_Pin GPIO_PIN_12
#define DIR3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DIR4_Pin GPIO_PIN_15
#define DIR4_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_6
#define TX_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_7
#define RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
