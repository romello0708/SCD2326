/* 传感器类型选择继电器控制
 * 硬件：RELAY_1 = PB8，RELAY_2 = PB9，NPN 三极管驱动 5V 线圈，高电平吸合
 * 说明：两路继电器同时动作，对应 IN7~IN10 的 NPN / PNP 光耦通道切换。
 */
#include "relay.h"
#include "global.h"

/* 不吸合 = NPN 光耦通道，吸合 = PNP 光耦通道（与原理图触点对应） */
#define RELAY_LEVEL_NPN       GPIO_PIN_RESET
#define RELAY_LEVEL_PNP       GPIO_PIN_SET

static uint8_t s_energized;

static void Relay_Write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, state);
    HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, state);
}

void Relay_Init(void)
{
    /* 上电先按配置的传感器类型设置，默认（配置无效时）为 NPN */
    Relay_SetSensorType(g_cfg.sensor_type);
}

void Relay_SetSensorType(uint8_t type)
{
    if (type == SENSOR_TYPE_PNP)
    {
        g_cfg.sensor_type = SENSOR_TYPE_PNP;
        Relay_Write(RELAY_LEVEL_PNP);
        s_energized = 1U;
    }
    else
    {
        g_cfg.sensor_type = SENSOR_TYPE_NPN;
        Relay_Write(RELAY_LEVEL_NPN);
        s_energized = 0U;
    }
}

uint8_t Relay_GetSensorType(void)
{
    return g_cfg.sensor_type;
}

/* 测试用：直接吸合/释放，不改变保存的传感器类型 */
void Relay_Force(uint8_t on)
{
    if (on != 0U)
    {
        Relay_Write(RELAY_LEVEL_PNP);
        s_energized = 1U;
    }
    else
    {
        Relay_Write(RELAY_LEVEL_NPN);
        s_energized = 0U;
    }
}

uint8_t Relay_IsEnergized(void)
{
    return s_energized;
}
