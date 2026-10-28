/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
/* tim.c —— 定时器初始化与 1ms 中断调度
 *
 * 本文件由 CubeMX 生成，仅补充注释；三个定时器分工：
 *   TIM1（高级定时器）CH1/CH2 -> PA8/PA9  = 装填电机 U/V 高边 PWM，20kHz（ARR=3599，不分频）
 *   TIM3（通用定时器）CH2/CH3 -> PA7/PB0  = 装填电机 U/V 低边 PWM，20kHz（与 TIM1 同频）
 *   TIM2（通用定时器）无引脚     = 1ms 时基：72MHz / 72 / 1000 = 1kHz 更新中断
 *
 * TIM2 的更新中断是整个控制逻辑的节拍源，回调内按固定顺序执行：
 *   1) DI_ScanIsr()         10 路采样 + 积分消抖 + 稳定上升沿计数 + 挡位解码
 *   2) Work_LimitGuardIsr() 到限位立即关断电机（不等主循环）
 *   3) Work_CountTask()     击发窗口判定 + IN_7 射击计数
 * 顺序不能颠倒：后两步依赖本次扫描刚更新好的稳定电平与边沿计数。
 *
 * NVIC：TIM2 抢占优先级 1、子优先级 0（HAL 默认分组下属较高优先级），
 *       保证 1ms 节拍不被 OLED/串口等长任务拖慢。
 */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "digital_input.h"
#include "work.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function：72MHz/72/1000 = 1kHz → 1ms 更新中断
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main），在 MX_TIM1_Init / MX_TIM3_Init 之后
 * 注意：此处只配置时基，不启动；真正开中断在 main 里 HAL_TIM_Base_Start_IT(&htim2)，
 *       且必须在 DI_Init / Work_Init 完成之后，否则第一拍中断会在未初始化状态下执行。
 *       分频：Prescaler = 72-1 → 计数时钟 1MHz；Period = 1000-1 → 每 1000 计数溢出一次 = 1ms。
 *       引脚：TIM2 无外部引脚，纯粹作为软件节拍。 */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM1 init function
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main），在 MX_SPI2_Init 之后、Motor_Init 之前
 * 注意：TIM1 是高级定时器，U/V 两相高边使用 CH1/CH2（PA8/PA9）；
 *       DeadTime 配置为 0 —— 因为本方案并非同一 TIM 的互补输出，
 *       四路门极的互锁与死区完全由软件保证（见 motor.c）；
 *       比较值初始为 0，PWM 通道由 Motor_Init 里 HAL_TIM_PWM_Start 打开。
 */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600 - 1;   /* 72MHz / 3600 = 20kHz，与装填电机驱动一致 */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM3 init function
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main），紧随 MX_TIM1_Init 之后
 * 注意：TIM3 输出 U/V 两相低边（CH2->PA7、CH3->PB0），与 TIM1 同为 20kHz；
 *       两个定时器各自独立启动，相位不保证对齐，靠软件“先关后开 + 2ms 死区”保证安全；
 *       TIM3 没有 BreakDeadTime 配置（通用定时器无此功能）。
 */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3600 - 1;   /* 72MHz / 3600 = 20kHz，与 TIM1 同频 */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/* TIM2 时基中断的时钟与 NVIC 配置
 * 参数：tim_baseHandle —— HAL 传入的时基句柄，仅处理 TIM2
 * 返回：无
 * 调用：HAL_TIM_Base_Init(&htim2) 内部自动回调（MX_TIM2_Init 中）
 * 注意：抢占优先级 1、子优先级 0；比 SysTick 默认最低优先级高，
 *       使 1ms 扫描与限位急停可以打断主循环里的长任务（OLED、EEPROM、RTT）
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}

/* 1ms 定时中断回调：输入扫描（含消抖） + 射击计数
 * 参数：htim —— 触发回调的定时器句柄，这里只处理 TIM2
 * 返回：无
 * 调用：TIM2 更新中断（每 1ms 一次），由 HAL 在 HAL_TIM_IRQHandler 中回调
 * 注意：三步的顺序固定且不能交换：
 *       1) DI_ScanIsr 先把稳定电平与边沿计数更新好；
 *       2) Work_LimitGuardIsr 用最新电平判断限位，立即断电；
 *       3) Work_CountTask 用最新边沿计数扣减余弹。
 *       中断内禁止打印/延时；工作模式下的余弹变化由主循环 Work_Task 负责打印。
 * 说明：TIM2 只做时基，不占用任何引脚；扫描周期由硬件保证为 1ms，
 *       不受主循环任务耗时影响。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    DI_ScanIsr();          /* 10 路输入采样 + 积分式消抖 + 上升沿计数 */
    Work_LimitGuardIsr();  /* 到限位(IN_9/IN_10)立即断电，不等主循环 */
    Work_CountTask();      /* 击发信号(MCU_OUT1)窗口内的 IN_7 射击计数 */
  }
}

/* PWM 通道的 MSP 初始化：打开 TIM1 / TIM3 的外设时钟
 * 参数：tim_pwmHandle —— HAL 传入的 PWM 句柄，只处理 TIM1 / TIM3
 * 返回：无
 * 调用：HAL_TIM_PWM_Init 内部自动回调（MX_TIM1_Init / MX_TIM3_Init 中）
 * 注意：这里只开时钟，GPIO 复用配置在 HAL_TIM_MspPostInit 中完成
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
/* PWM 引脚复用配置（HAL_TIM_MspPostInit 回调）
 * 参数：timHandle —— TIM1 时配置 PA8/PA9；TIM3 时配置 PA7/PB0
 * 返回：无
 * 调用：MX_TIM1_Init / MX_TIM3_Init 末尾自动调用
 * 注意：四路均为 GPIO_MODE_AF_PP 推挽复用、低速档；速度档只影响边沿斜率，
 *       20kHz 下低速档即可，且有利于降低驱动端干扰
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    */
    GPIO_InitStruct.Pin = MCU_PWMUH_Pin|MCU_PWMVH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    */
    GPIO_InitStruct.Pin = MCU_PWMUL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MCU_PWMUL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MCU_PWMVL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MCU_PWMVL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

/* PWM 通道的 MSP 反初始化：关闭 TIM1 / TIM3 外设时钟
 * 参数：tim_pwmHandle —— HAL 传入的 PWM 句柄，只处理 TIM1 / TIM3
 * 返回：无
 * 调用：HAL_TIM_PWM_DeInit 内部自动回调；本项目正常运行不会调用
 * 注意：仅关时钟，不恢复 GPIO 复用配置
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
