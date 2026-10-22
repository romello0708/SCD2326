#ifndef __MENU_H
#define __MENU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Menu_Init(void);
void Menu_Task(void);   /* 主循环任务：按键扫描 + 页面刷新 */

#ifdef __cplusplus
}
#endif

#endif /* __MENU_H */
