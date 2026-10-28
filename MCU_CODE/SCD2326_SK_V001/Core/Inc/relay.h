#ifndef __RELAY_H
#define __RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：relay.h —— 传感器类型选择继电器接口
 *
 * 模块职责：把 SENSOR_TYPE_NPN / SENSOR_TYPE_PNP 映射成两路继电器电平，
 *           同时把当前类型记录到 g_cfg（供菜单显示与 EEPROM 保存）。
 * 硬件：RELAY_1=PB8（传感器1：IN7/IN8）、RELAY_2=PB9（传感器2：IN9/IN10）；
 *       R60/R63 -> S8050(NPN) -> G6K-2F-Y-5V 线圈，高电平三极管导通、继电器吸合。
 * 电平约定：低电平 = NPN（不吸合），高电平 = PNP（吸合）。
 * 运行上下文：Relay_Init 在上电初始化调用；Relay_SetType/GetType 在主循环（菜单）调用。
 */

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
#define SENSOR_TYPE_NUM       2U    /* 可选类型数量（菜单循环用），取值 0 ~ 1 */

void    Relay_Init(void);                       /* 按 g_cfg 中保存的类型设置两路输出 */
void    Relay_SetType(uint8_t ch, uint8_t type);/* ch: 1=传感器1, 2=传感器2；type: SENSOR_TYPE_*，非法值按 NPN */
uint8_t Relay_GetType(uint8_t ch);              /* 返回 g_cfg 中记录的类型（不回读 GPIO） */
void    Relay_Force(uint8_t ch, uint8_t on);    /* 测试用：不改变保存的类型；on≠0 输出低电平（不吸合） */

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_H */
