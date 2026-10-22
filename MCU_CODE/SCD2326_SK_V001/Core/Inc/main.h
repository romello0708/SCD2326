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
#define MOTOR_ALL_Pin GPIO_PIN_0
#define MOTOR_ALL_GPIO_Port GPIOA
#define MOTOR_U_Pin GPIO_PIN_1
#define MOTOR_U_GPIO_Port GPIOA
#define IN_10_Pin GPIO_PIN_4
#define IN_10_GPIO_Port GPIOA
#define IN_9_Pin GPIO_PIN_5
#define IN_9_GPIO_Port GPIOA
#define IN_8_Pin GPIO_PIN_6
#define IN_8_GPIO_Port GPIOA
#define MCU_PWMUL_Pin GPIO_PIN_7
#define MCU_PWMUL_GPIO_Port GPIOA
#define MCU_PWMVL_Pin GPIO_PIN_0
#define MCU_PWMVL_GPIO_Port GPIOB
#define IN_7_Pin GPIO_PIN_1
#define IN_7_GPIO_Port GPIOB
#define MCU_EEPROM_SCL_Pin GPIO_PIN_10
#define MCU_EEPROM_SCL_GPIO_Port GPIOB
#define MCU_EEPROM_SDA_Pin GPIO_PIN_11
#define MCU_EEPROM_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_13
#define OLED_SCL_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_15
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_12
#define OLED_CS_GPIO_Port GPIOB
#define MCU_PWMUH_Pin GPIO_PIN_8
#define MCU_PWMUH_GPIO_Port GPIOA
#define MCU_PWMVH_Pin GPIO_PIN_9
#define MCU_PWMVH_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_10
#define OLED_RES_GPIO_Port GPIOA
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOA
#define MCU_OUT1_Pin GPIO_PIN_12
#define MCU_OUT1_GPIO_Port GPIOA
#define IN_1_Pin GPIO_PIN_15
#define IN_1_GPIO_Port GPIOA
#define IN_2_Pin GPIO_PIN_3
#define IN_2_GPIO_Port GPIOB
#define IN_3_Pin GPIO_PIN_4
#define IN_3_GPIO_Port GPIOB
#define IN_4_Pin GPIO_PIN_5
#define IN_4_GPIO_Port GPIOB
#define IN_5_Pin GPIO_PIN_6
#define IN_5_GPIO_Port GPIOB
#define IN_6_Pin GPIO_PIN_7
#define IN_6_GPIO_Port GPIOB
#define RELAY_1_Pin GPIO_PIN_8
#define RELAY_1_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_9
#define RELAY_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
