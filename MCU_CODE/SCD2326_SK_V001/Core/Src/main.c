/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "AnalogQuantityAcquisition.h"
#include "comm.h"
#include "global.h"
#include "digital_input.h"
#include "oled.h"
#include "menu.h"
#include "eeprom.h"
#include "relay.h"
#include "work.h"
#include "log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Global_Init();  /* 全局配置变量初始化 */
  SEGGER_RTT_Init();          /* J-Link RTT 调试通道 */
  LOG("\r\n[BOOT] %s SW %d.%d.%d HW %s\r\n", BOARD_NAME, SW_VERSION_MAJOR, SW_VERSION_MINOR, SW_VERSION_BUILD, BOARD_HW_VERSION);
  (void)EEPROM_Init();        /* 检测 BL24C128A 是否在线 */
  if (EEPROM_LoadConfig() == HAL_OK)
  {
    LOG("[CFG] eeprom loaded\r\n");
  }
  else
  {
    LOG("[CFG] eeprom invalid, use default\r\n");
  }
  LOG("[CFG] mode=%d s1=%d s2=%d fire=%d module=%d dir=%d ammo=%d target=%d pulse=%d gap=%d burst=%d lburst=%d\r\n",
      g_cfg.work_mode, g_cfg.sensor1_type, g_cfg.sensor2_type, g_cfg.fire_mode,
      g_cfg.module_type, g_cfg.load_dir, g_cfg.ammo, g_cfg.ammo_target,
      g_cfg.pulse_ms, g_cfg.gap_ms, g_cfg.burst_shots, g_cfg.long_burst_shots);
  Relay_Init();   /* 按配置的传感器类型（NPN/PNP）设置继电器 */
  Motor_Init();   /* 装填电机 PWM 初始化，初始为停止状态 */
  AD_Init();      /* 电流采集初始化，需在电机停止时做零点校准 */
  Comm_Init();    /* 串口 DMA 接收挂起，开始收发 */
  if ((g_cfg.baudrate >= 9600U) && (g_cfg.baudrate != huart2.Init.BaudRate))
  {
    Comm_SetBaud(g_cfg.baudrate);   /* 应用 EEPROM 中保存的波特率 */
  }
  DI_Init();      /* 10 路数字输入扫描初始化 */
  Work_Init();    /* 工作/调试模式动作初始化：电机停止、射击输出拉低 */
  OLED_Init();    /* OLED 上电时序与显示配置 */
  Menu_Init();    /* 三级菜单初始化 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Motor_Task();   /* 1ms 周期：加减速曲线、换向死区、PWM 输出 */
    AD_Task();      /* 1ms 周期：电流采样、中值滤波、换算 */
    Work_Task();    /* 1ms 周期：装填/拉栓/复位到位、超时、堵转、余弹计数 */
    Work_KeyTask(); /* 按键动作：工作模式装填/射击，调试模式拉栓/复位/射击 */
    Comm_Task();    /* 串口回传测试：收到的帧原样发回 */
    Comm_StatusTask(); /* 500ms 打印一次 10 路输入状态与工作模式 */
    DI_Task();      /* 1ms 周期：10 路输入扫描与去抖 */
    Global_Task();  /* 1s 周期：运行时间等系统计数 */
    Menu_Task();    /* 菜单按键处理与页面刷新 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
