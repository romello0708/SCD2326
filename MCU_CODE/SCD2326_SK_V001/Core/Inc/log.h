#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：log.h —— J-Link RTT 调试打印封装
 *
 * 模块职责：把 LOG(...) 映射到 SEGGER_RTT_printf(0, ...)，可直接在 J-Link RTT Viewer 查看；
 *           关闭时整个宏展开为空语句，不产生代码。
 * 硬件：SWD 调试口（J-Link），不占用串口/USB；RTT 缓冲配置见 SEGGER_RTT_Conf.h。
 * 注意：LOG 为阻塞式格式化输出，只能在主循环/初始化中调用，禁止在中断内打印；
 *       量产固件可把 LOG_ENABLE 置 0 以彻底去掉字符串与格式化开销。
 */

/* J-Link RTT 调试打印开关：1 = 打开；0 = 全部编译为空（不影响功能，量产可关） */
#define LOG_ENABLE   1

#if LOG_ENABLE
#include "SEGGER_RTT.h"
#define LOG(...)     SEGGER_RTT_printf(0, __VA_ARGS__)   /* 通道 0 输出，支持 printf 格式 */
#else
#define LOG(...)     ((void)0)                          /* 关闭时消除参数与调用，避免 unused 警告 */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */
