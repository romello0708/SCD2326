/* 全局配置变量与宏定义集中管理
 * 说明：宏定义见 global.h，此处只放全局变量的定义与系统级周期任务。
 */
#include "global.h"
#include "relay.h"

GlobalConfig_t g_cfg;

static uint32_t s_tick_1s;

void Global_Init(void)
{
    uint8_t i;

    g_cfg.di_state = 0U;
    g_cfg.di_raw = 0U;
    for (i = 0; i < DI_NUM; i++)
    {
        g_cfg.di_rise_cnt[i] = 0U;
    }

    g_cfg.work_mode = WORK_MODE_WORK;
    g_cfg.sensor1_type = SENSOR_TYPE_NPN;
    g_cfg.sensor2_type = SENSOR_TYPE_NPN;
    g_cfg.ammo = 0U;
    g_cfg.fire_mode = FIRE_MODE_SINGLE;
    g_cfg.module_type = MODULE_FIRE;
    g_cfg.shot_total = 0U;
    g_cfg.motor_duty_pct = 90U;
    g_cfg.over_current_a = 20U;
    g_cfg.echo_enable = 1U;
    g_cfg.status_print_enable = 1U;
    g_cfg.baudrate = 115200U;
    g_cfg.run_seconds = 0U;
    g_cfg.reset_count = 0U;

    s_tick_1s = HAL_GetTick();
}

/* 1s 周期任务 */
void Global_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_tick_1s) < 1000U)
    {
        return;
    }

    s_tick_1s += 1000U;
    g_cfg.run_seconds++;
}
