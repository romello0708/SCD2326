/* 传感器类型选择继电器控制
 * 硬件：RELAY_1 = PB8（传感器1），RELAY_2 = PB9（传感器2）
 *       NPN 三极管驱动 5V 线圈，高电平吸合
 * 电平：低电平 = NPN（不吸合），高电平 = PNP（吸合），与原理图触点对应
 */
#include "relay.h"
#include "global.h"

#define RELAY_LEVEL_NPN       GPIO_PIN_RESET    /* 低电平 = NPN */
#define RELAY_LEVEL_PNP       GPIO_PIN_SET      /* 高电平 = PNP */

static void Relay_Write(uint8_t ch, GPIO_PinState state)
{
    if (ch == 2U)
    {
        HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, state);
    }
    else
    {
        HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, state);
    }
}

void Relay_Init(void)
{
    /* 上电按配置的传感器类型设置，默认（配置无效时）为 NPN */
    Relay_SetType(1U, g_cfg.sensor1_type);
    Relay_SetType(2U, g_cfg.sensor2_type);
}

void Relay_SetType(uint8_t ch, uint8_t type)
{
    GPIO_PinState level;

    if (type != SENSOR_TYPE_PNP)
    {
        type = SENSOR_TYPE_NPN;
    }

    level = (type == SENSOR_TYPE_PNP) ? RELAY_LEVEL_PNP : RELAY_LEVEL_NPN;
    Relay_Write(ch, level);

    if (ch == 2U)
    {
        g_cfg.sensor2_type = type;
    }
    else
    {
        g_cfg.sensor1_type = type;
    }
}

uint8_t Relay_GetType(uint8_t ch)
{
    return (ch == 2U) ? g_cfg.sensor2_type : g_cfg.sensor1_type;
}

/* 测试用：直接给高/低电平，不改变保存的传感器类型 */
void Relay_Force(uint8_t ch, uint8_t on)
{
    Relay_Write(ch, (on != 0U) ? RELAY_LEVEL_NPN : RELAY_LEVEL_PNP);
}
