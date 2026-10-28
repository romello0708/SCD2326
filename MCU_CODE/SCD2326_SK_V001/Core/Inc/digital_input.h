/* digital_input.h —— 10 路数字输入（IN_1~IN_10）扫描与消抖接口
 *
 * 模块职责：对外提供去抖后的电平、原始电平、各路上升沿计数与通道号校验封装。
 * 运行上下文：
 *   DI_Init    —— 上电初始化（启动 TIM2 之前）
 *   DI_ScanIsr —— TIM2 1ms 更新中断内调用（采样 + 积分消抖 + 上升沿计数 + 挡位解码）
 *   其余查询函数 —— 主循环与中断均可调用（只读）
 * 参数约定：通道号 ch 取 1~10，对应 IN_1~IN_10；越界统一返回 0。
 * 相关配置（DI_NUM、DI_DEBOUNCE_CNT、DI_DEBOUNCE_LIMIT_CNT）见 global.h。
 */

#ifndef __DIGITAL_INPUT_H
#define __DIGITAL_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void      DI_Init(void);                  /* 上电初始化：以实际电平作为消抖初值 */
void      DI_ScanIsr(void);               /* TIM2 1ms 中断内调用：扫描 + 消抖 + 上升沿计数 */
uint8_t   DI_IsHigh(uint8_t ch);          /* ch: 1~10，返回 1=高电平（去抖后） */
uint8_t   DI_GetHighCount(void);          /* 当前为高的通道数 0~10 */
uint16_t  DI_GetState(void);              /* 去抖后的位图，bit0 = IN_1 */
uint16_t  DI_GetRaw(void);                /* 原始电平位图 */
uint16_t  DI_GetRiseCount(uint8_t ch);    /* 该路稳定上升沿累计数（16 位回绕） */
void      DI_ClearCounters(void);         /* 清零所有上升沿计数（仅停机时使用） */

#ifdef __cplusplus
}
#endif

#endif /* __DIGITAL_INPUT_H */
