#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 两路继电器（U16 / U17，G6K-2F-Y-5V）用于切换输入通道的传感器类型
 * 驱动：PB8=RELAY_1、PB9=RELAY_2 -> R60/R63 3.3K -> S8050(NPN) -> 5V 线圈
 *       高电平 = 三极管导通 = 线圈吸合
 *
 * 触点（每路为双刀双掷，各带 2 个输入通道）：
 *   RELAY_1：IN7 / IN8      RELAY_2：IN9 / IN10
 *   不吸合：输入接 NPN 光耦通道（LED 阳极接 VEE_24V，传感器拉低点亮）
 *   吸合  ：输入接 PNP 光耦通道（LED 阳极由传感器输出驱动，拉高点亮）
 */
#define SENSOR_TYPE_NPN       0U
#define SENSOR_TYPE_PNP       1U

void    Relay_Init(void);
void    Relay_SetSensorType(uint8_t type);   /* SENSOR_TYPE_NPN / SENSOR_TYPE_PNP */
uint8_t Relay_GetSensorType(void);
void    Relay_Force(uint8_t on);             /* 测试用：1=吸合，0=释放 */
uint8_t Relay_IsEnergized(void);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */
