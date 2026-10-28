/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* tim.h —— 定时器句柄与初始化接口
 *
 * 三个定时器分工（详见 tim.c 头部说明）：
 *   TIM1：装填电机 U/V 高边 PWM（CH1/CH2，PA8/PA9），20kHz；
 *   TIM3：装填电机 U/V 低边 PWM（CH2/CH3，PA7/PB0），20kHz；
 *   TIM2：1ms 时基，更新中断中依次执行 DI_ScanIsr / Work_LimitGuardIsr / Work_CountTask。
 * 本文件由 CubeMX 生成，本项目仅补充注释，未改动任何定义。
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* 定时器初始化：
 *   MX_TIM1_Init  TIM1 CH1/CH2 高边 PWM，ARR=3599 → 20kHz
 *   MX_TIM2_Init  TIM2 1ms 时基（PSC=71, ARR=999），只配置不启动
 *   MX_TIM3_Init  TIM3 CH2/CH3 低边 PWM，ARR=3599 → 20kHz
 *   HAL_TIM_MspPostInit  PWM 引脚复用配置（由上面两个 PWM 初始化在末尾调用） */
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);   /* 1ms 时基，更新中断内做输入扫描与射击计数 */
void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

