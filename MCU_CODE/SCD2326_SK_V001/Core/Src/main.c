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
/* main.c —— 系统入口：时钟/外设初始化、上电顺序、主循环任务调度
 *
 * 上电顺序（顺序有依赖，不可随意调整）：
 *   HAL_Init → SystemClock_Config(72MHz)
 *   → MX_GPIO_Init / MX_DMA_Init / MX_ADC1_Init / MX_I2C2_Init / MX_SPI2_Init
 *   → MX_TIM1_Init / MX_TIM3_Init（电机 PWM 时基）/ MX_TIM2_Init（1ms 时基）/ MX_USART2_UART_Init
 *   → Global_Init（配置默认值）→ RTT 初始化 → EEPROM_Init/LoadConfig（覆盖默认值）
 *   → Relay_Init（按传感器类型设继电器）→ Motor_Init
 *   → AD_Init（必须在电机静止时做零点校准）→ Comm_Init → DI_Init → Work_Init
 *   → 启动 TIM2 1ms 中断 → OLED_Init → Menu_Init → 主循环
 *
 * 主循环任务（每个任务自带节拍判断，非阻塞）：
 *   Motor_Task      1ms   换向死区、加减速曲线、PWM 输出
 *   AD_Task         1ms   电流采样/滤波/换算
 *   Work_Task       1ms   动作状态机（到位/超时/堵转）、射击时序
 *   Work_KeyTask    -     工作/调试模式按键（边沿 + 长按）
 *   Comm_Task       -     串口回传
 *   Comm_StatusTask 500ms 输入状态打印
 *   Global_Task     1s    运行时间等系统计数
 *   Menu_Task       200ms/500ms 菜单按键、页面刷新、RTT 镜像
 *
 * 注意：输入扫描（DI_ScanIsr）与射击计数（Work_CountTask）已移入 TIM2 中断，
 *       主循环不再调用，避免重复扫描与重复计数。
 */
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
/* 程序入口：完成时钟与外设初始化后进入不死循环
 * 参数：无
 * 返回：int —— 实际不会返回（while(1) 不会退出）
 * 调用：复位后由启动文件调用
 * 注意：初始化顺序存在硬性依赖，改动前请确认：
 *   1) AD_Init 必须在 Motor_Init 之后且电机确实静止时执行，否则电流零点校准偏掉，
 *      后续堵转/过流判定会失准；
 *   2) TIM2 必须最后启动（DI_Init / Work_Init 之后），否则第一拍中断会在
 *      未初始化的消抖状态与计数基准上运行，可能误判挡位或漏计；
 *   3) Relay_Init 依赖 EEPROM 读出的传感器类型，必须在 LoadConfig 之后；
 *   4) Menu_Init 依赖 OLED_Init（上电时序完成后才能刷屏）。
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
  MX_TIM2_Init();      /* TIM2：1ms 时基，不占用引脚，仅做输入扫描与射击计数的节拍 */
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
  LOG("[CFG] mode=%d s1=%d s2=%d fire=%d module=%d dir=%d ammo=%d pulse=%d gap=%d burst=%d lburst=%d\r\n",
      g_cfg.work_mode, g_cfg.sensor1_type, g_cfg.sensor2_type, g_cfg.fire_mode,
      g_cfg.module_type, g_cfg.load_dir, g_cfg.ammo,
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
  /* TIM2 1ms 更新中断启动：中断内完成 10 路输入扫描（普通通道 4ms / 限位通道 2ms 积分消抖）
   * 与 IN_7 射击计数。放在 DI_Init / Work_Init 之后启动，保证扫描状态与计数基准已就绪。 */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  OLED_Init();    /* OLED 上电时序与显示配置 */
  Menu_Init();    /* 三级菜单初始化 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 主循环：所有任务都是“查节拍 → 干活”的非阻塞写法；
   * 1ms 级关键时序（输入扫描、限位急停、射击计数）已在 TIM2 中断内完成，
   * 因此这里的短暂阻塞（OLED 刷新、EEPROM 写页、RTT 打印）不会影响这些时序 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Motor_Task();   /* 1ms 周期：加减速曲线、换向死区、PWM 输出 */
    AD_Task();      /* 1ms 周期：电流采样、中值滤波、换算 */
    Work_Task();    /* 1ms 周期：装填/拉栓/复位到位、超时、堵转判定、射击时序 */
    Work_KeyTask(); /* 按键动作：工作模式装填/射击，调试模式拉栓/复位/射击 */
    Comm_Task();    /* 串口回传测试：收到的帧原样发回 */
    Comm_StatusTask(); /* 500ms 打印一次 10 路输入状态与工作模式 */
    /* 输入扫描与射击计数已移入 TIM2 1ms 中断（DI_ScanIsr + Work_CountTask），
     * 主循环不再调用，避免与中断重复扫描、重复计数。 */
    Global_Task();  /* 1s 周期：运行时间等系统计数 */
    Menu_Task();    /* 菜单按键处理与页面刷新 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/* 系统时钟配置：HSE 8MHz × 9 = 72MHz，APB1 二分频 36MHz，APB2 = 72MHz
 * 参数：无
 * 返回：无
 * 调用：main() 中 HAL_Init 之后，所有外设初始化之前
 * 注意：APB1 = 36MHz，但定时器时钟有 ×2 倍频，TIM2 实际仍是 72MHz
 *       （TIM2 的 72MHz/72/1000 = 1ms 即依此计算）；
 *       TIM1/TIM3 挂在 APB2 = 72MHz；
 *       ADC 时钟 = PCLK2(72MHz) / 6 = 12MHz（不超过 14MHz 上限）；
 *       FLASH_LATENCY_2 为 72MHz 必需的等待周期，不能改小
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
/* 致命错误处理：关中断后原地死循环
 * 参数：无
 * 返回：无（不会返回）
 * 调用：各外设初始化失败（HAL_xxx_Init 返回非 HAL_OK）或 TIM2 启动失败时
 * 注意：关中断是为了让电机/击发输出保持在最后一次写入的状态，
 *       实际产品中可在此加入看门狗复位或安全停机（关闭 MCU_OUT1 与四路 PWM）
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
/* 参数断言失败回调（仅 USE_FULL_ASSERT 时编译）
 * 参数：file —— 源文件名，line —— 出错行号
 * 返回：无
 * 调用：HAL 内部 assert_param 判定失败时
 * 注意：当前为空实现；调试时可在 USER CODE BEGIN 6 内打印 file/line 定位问题
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
