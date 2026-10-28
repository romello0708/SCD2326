#ifndef __DIGITAL_INPUT_H
#define __DIGITAL_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void      DI_Init(void);
void      DI_ScanIsr(void);               /* TIM2 1ms 中断内调用：扫描 + 消抖 + 上升沿计数 */
uint8_t   DI_IsHigh(uint8_t ch);          /* ch: 1~10，返回 1=高电平 */
uint8_t   DI_GetHighCount(void);
uint16_t  DI_GetState(void);              /* 去抖后的位图，bit0 = IN_1 */
uint16_t  DI_GetRaw(void);                /* 原始电平位图 */
uint16_t  DI_GetRiseCount(uint8_t ch);    /* 上升沿计数 */
void      DI_ClearCounters(void);

#ifdef __cplusplus
}
#endif

#endif /* __DIGITAL_INPUT_H */
