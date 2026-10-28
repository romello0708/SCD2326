/* 全局配置变量与宏定义集中管理
 *
 * 模块职责：
 *   1) 定义全局配置实例 g_cfg（全系统唯一，各模块通过 extern 共享）；
 *   2) 提供一份确定的上电默认参数（Global_Init）；
 *   3) 提供 1s 系统节拍任务（Global_Task），累计运行秒数。
 * 说明：宏定义见 global.h，此处只放全局变量的定义与系统级周期任务。
 * 硬件：不直接操作外设，仅依赖 SysTick 提供的 HAL_GetTick() 1ms 时基。
 * 关键参数：默认值（余弹 200、射击模式无限制、传感器 PNP、电机占空比 90%、过流 20A、
 *           波特率 115200 等）逐项见 Global_Init 内注释；这些值只在 EEPROM 无效时生效。
 * 运行上下文：Global_Init 在上电初始化阶段（main.c，外设初始化之后）；
 *           Global_Task 在主循环，非阻塞。
 */
#include "global.h"
#include "relay.h"

GlobalConfig_t g_cfg;   /* 全局配置实例，定义在此，头文件中 extern 声明 */

static uint32_t s_tick_1s;   /* 1s 任务的累加基准（HAL_GetTick 的毫秒值） */

/* 全局配置初始化：写入一份确定的默认参数
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main.c，HAL 外设初始化之后、EEPROM_LoadConfig 之前）
 * 注意：这里是 EEPROM 无效或首次上电时使用的默认值；
 *       EEPROM_LoadConfig 成功时只会覆盖其中掉电保存的字段（不会整体覆盖）。
 */
void Global_Init(void)
{
    uint8_t i;

    g_cfg.di_state = 0U;    /* 去抖后的输入位图，bit0 = IN_1 */
    g_cfg.di_raw = 0U;      /* 原始电平位图，由 DI_ScanIsr 刷新 */
    for (i = 0; i < DI_NUM; i++)
    {
        g_cfg.di_rise_cnt[i] = 0U;
    }

    g_cfg.work_mode = WORK_MODE_WORK;
    g_cfg.sensor1_type = SENSOR_TYPE_PNP;   /* 默认传感器类型：PNP */
    g_cfg.sensor2_type = SENSOR_TYPE_PNP;
    g_cfg.ammo = AMMO_MAX;              /* 默认余弹数 200（不装填目标，也不再掉电保存） */
    g_cfg.fire_mode = FIRE_MODE_UNLIMITED;  /* 默认射击模式：无限制 */
    g_cfg.module_type = MODULE_FIRE;        /* 默认模块型号：火力模块 */
    g_cfg.shot_total = 0U;              /* 累计射击次数：目前仅保留字段，无自增点 */
    g_cfg.pulse_ms = 100U;              /* 默认单发脉冲 100ms */
    g_cfg.gap_ms = 150U;                /* 默认双发间隔 150ms */
    g_cfg.burst_shots = 3U;             /* 默认点射 3 发 */
    g_cfg.long_burst_shots = 6U;        /* 默认长点射 6 发 */
    g_cfg.motor_duty_pct = 90U;         /* 电机固定 90% 占空比（设置项已移除，不从 EEPROM 恢复） */
    g_cfg.load_dir = LOAD_DIR_FORWARD;  /* 默认装填电机正向 */
    g_cfg.over_current_a = 20U;         /* 默认过流阈值 20A，Work_Task 中用于堵转判定 */
    g_cfg.echo_enable = 1U;             /* 默认打开串口回传 */
    g_cfg.status_print_enable = 1U;     /* 默认打开 500ms 状态帧 */
    g_cfg.baudrate = 115200U;           /* 与 MX_USART2_UART_Init 一致，可被 EEPROM 覆盖 */
    g_cfg.run_seconds = 0U;             /* 运行秒数，由 Global_Task 每秒 +1，掉电不保存 */
    g_cfg.reset_count = 0U;             /* 预留：当前只初始化，无自增点 */

    s_tick_1s = HAL_GetTick();
}

/* 1s 周期任务：累计运行秒数
 * 参数：无
 * 返回：无
 * 调用：主循环（main.c while(1)，每轮调用）
 * 注意：用 s_tick_1s += 1000 递推而不是直接取 now，避免主循环抖动造成长期累计误差；
 *       若主循环被阻塞超过 1s，下一轮会连续补齐多个 1s；
 *       run_seconds 为 uint32，按秒计约 136 年才回绕。
 */
void Global_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_tick_1s) < 1000U)
    {
        return;     /* 未到 1s，直接返回（无符号相减，tick 回绕也安全） */
    }

    s_tick_1s += 1000U;
    g_cfg.run_seconds++;
}
