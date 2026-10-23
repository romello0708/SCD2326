#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 两路继电器（U16 / U17，G6K-2F-Y-5V）分别配置两组传感器的类型
 * 驱动：PB8=RELAY_1（传感器1）、PB9=RELAY_2（传感器2）
 *       -> R60/R63 3.3K -> S8050(NPN) -> 5V 线圈；高电平 = 三极管导通 = 线圈吸合
 *
 * 电平约定（按原理图触点）：低电平 = NPN 通道，高电平 = PNP 通道；
 *       即不吸合为 NPN，吸合为 PNP
 *
 * 触点（每路为双刀双掷，各带 2 个输入通道）：
 *   RELAY_1：IN7 / IN8      RELAY_2：IN9 / IN10
 */
#define SENSOR_TYPE_NPN       0U    /* 继电器输出低电平（不吸合） */
#define SENSOR_TYPE_PNP       1U    /* 继电器输出高电平（吸合） */
#define SENSOR_TYPE_NUM       2U

void    Relay_Init(void);                       /* 按 g_cfg 中保存的类型设置两路输出 */
void    Relay_SetType(uint8_t ch, uint8_t type);/* ch: 1=传感器1, 2=传感器2 */
uint8_t Relay_GetType(uint8_t ch);
void    Relay_Force(uint8_t ch, uint8_t on);    /* 测试用：不改变保存的类型 */

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */
