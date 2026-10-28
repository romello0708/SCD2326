/* menu.h —— OLED 三挡界面（设置 / 工作 / 调试）接口
 *
 * 模块职责：设置模式的参数编辑与 EEPROM 保存、工作/调试页的状态显示，
 *           以及把当前页面内容镜像打印到 RTT（500ms 一帧）。
 * 运行上下文：Menu_Init 上电初始化（OLED_Init 之后）；Menu_Task 由主循环每轮调用。
 * 依赖：g_cfg（global.h）中的配置项、Work_GetState()/Work_GetAmmo() 的状态、
 *       DI_IsHigh() 的位置信号；显示驱动见 oled.h。
 */

#ifndef __MENU_H
#define __MENU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Menu_Init(void);   /* 上电初始化：清除界面状态、同步按键快照、强制首帧重绘 */
void Menu_Task(void);   /* 主循环任务：按键扫描 + 页面刷新 */

#ifdef __cplusplus
}
#endif

#endif /* __MENU_H */
