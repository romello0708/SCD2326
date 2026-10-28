/* 传感器类型选择继电器控制
 *
 * 模块职责：
 *   把 g_cfg.sensor1_type / sensor2_type 映射成两路继电器的通断，
 *   用双刀双掷触点切换每组传感器的 NPN/PNP 接线方式（IN7~IN10）。
 *
 * 硬件：
 *   RELAY_1 = PB8（传感器1，对应 IN7 / IN8），RELAY_2 = PB9（传感器2，对应 IN9 / IN10）；
 *   GPIO -> R60/R63 3.3K -> S8050(NPN) -> G6K-2F-Y-5V 线圈，高电平三极管导通、线圈吸合。
 *   电平：低电平 = NPN（不吸合），高电平 = PNP（吸合），与原理图触点对应。
 *
 * 关键参数与时序：
 *   无软件延时要求；继电器机械动作约几 ms，切换后应等输入稳定再采样。
 *
 * 运行上下文：
 *   Relay_Init：上电初始化（main.c，EEPROM_LoadConfig 之后）；
 *   Relay_SetType：菜单切换传感器类型时调用；
 *   均在主循环上下文，函数内无阻塞，也不在中断中调用。
 */
#include "relay.h"
#include "global.h"

#define RELAY_LEVEL_NPN       GPIO_PIN_RESET    /* 低电平 = NPN */
#define RELAY_LEVEL_PNP       GPIO_PIN_SET      /* 高电平 = PNP */

/* 按通道号写继电器线圈驱动电平
 * 参数：ch = 1 或 2（其它值按通道 1 处理）
 *       state = GPIO_PIN_RESET（三极管截止、线圈不吸合）/ GPIO_PIN_SET（导通、吸合）
 * 返回：无
 * 调用：Relay_Init / Relay_SetType / Relay_Force（主循环）
 * 注意：只有通道 2 单独判断，其余值都落到 RELAY_1，避免越界。
 */
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

/* 上电初始化两路继电器
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main.c，EEPROM_LoadConfig 之后、Motor_Init 之前）
 * 注意：按 g_cfg 中保存的类型设置；g_cfg 默认 PNP，因此首次上电两路均吸合（PNP 通路）。
 */
void Relay_Init(void)
{
    /* 上电按配置的传感器类型设置，默认（配置无效时）为 PNP */
    Relay_SetType(1U, g_cfg.sensor1_type);
    Relay_SetType(2U, g_cfg.sensor2_type);
}

/* 设置某一路传感器的 NPN/PNP 类型，同时更新硬件电平与 g_cfg
 * 参数：ch = 1（传感器1 / IN7、IN8）或 2（传感器2 / IN9、IN10）
 *       type = SENSOR_TYPE_NPN(0) 或 SENSOR_TYPE_PNP(1)，其它值按 NPN 处理
 * 返回：无
 * 调用：Relay_Init（上电）、菜单“传感器类型”项切换（主循环 Menu_Task）
 * 注意：函数会同步写回 g_cfg，菜单保存时随 EEPROM_SaveConfig 掉电保存；
 *       继电器机械动作有 ms 级延迟，切换后不要立刻用相关输入做判定。
 */
void Relay_SetType(uint8_t ch, uint8_t type)
{
    GPIO_PinState level;

    if (type != SENSOR_TYPE_PNP)
    {
        type = SENSOR_TYPE_NPN;     /* 非法值统一按 NPN，保证电平和配置一致 */
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

/* 读取某一路当前保存的传感器类型
 * 参数：ch = 1 或 2（其它值按通道 1 返回）
 * 返回：SENSOR_TYPE_NPN / SENSOR_TYPE_PNP
 * 调用：菜单显示、状态查询（主循环）
 * 注意：返回 g_cfg 中的软件状态，不回读 GPIO；Relay_Force 不会改变它。
 */
uint8_t Relay_GetType(uint8_t ch)
{
    return (ch == 2U) ? g_cfg.sensor2_type : g_cfg.sensor1_type;
}

/* 测试用：直接给高/低电平，不改变保存的传感器类型
 * 参数：ch = 1 或 2；on = 0 输出高电平（吸合 / PNP 通路），非 0 输出低电平（不吸合 / NPN 通路）
 * 返回：无
 * 调用：产测、硬件调试时手动调用（主循环）
 * 注意：on 的语义与“吸合”相反（on≠0 表示不吸合），且不更新 g_cfg；
 *       正常业务不要用，否则硬件状态会与菜单显示不一致。
 */
void Relay_Force(uint8_t ch, uint8_t on)
{
    Relay_Write(ch, (on != 0U) ? RELAY_LEVEL_NPN : RELAY_LEVEL_PNP);
}
