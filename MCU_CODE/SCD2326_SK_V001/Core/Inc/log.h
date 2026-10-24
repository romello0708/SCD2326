#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* J-Link RTT 调试打印开关：1 = 打开；0 = 全部编译为空（不影响功能，量产可关） */
#define LOG_ENABLE   1

#if LOG_ENABLE
#include "SEGGER_RTT.h"
#define LOG(...)     SEGGER_RTT_printf(0, __VA_ARGS__)
#else
#define LOG(...)     ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */
