#ifndef __WORK_H
#define __WORK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 工作 / 调试模式动作控制
 * 面板：A=IN_6、B=IN_5、C=IN_4；拨动开关 IN_2 / IN_3 决定设置/工作/调试
 * 输入：IN_7  = 装填计数脉冲（上升沿 +1）
 *       IN_9  = 初始到位信号（后到位）
 *       IN_10 = 到位位置信号（前到位）
 * 输出：MCU_OUT1(PA12) 高电平 = 射击（击发）
 */

/* 输入通道 */
#define WORK_COUNT_CH        7U
#define WORK_REAR_CH         9U     /* 初始到位 */
#define WORK_FRONT_CH        10U    /* 到位位置 */

/* 动作方向：装填/复位向前，拉栓向后；实物相反时只改这三行 */
#define WORK_DIR_LOAD        MOTOR_STATE_FORWARD
#define WORK_DIR_BOLT        MOTOR_STATE_REVERSE
#define WORK_DIR_RESET       MOTOR_STATE_FORWARD

/* 射击时序 */
#define WORK_SHOT_PULSE_MS       100U   /* 单发脉宽 */
#define WORK_SHOT_GAP_MS         150U   /* 两发之间的间隔 */
#define WORK_BURST_SHOTS         3U     /* 点射发数 */
#define WORK_LONG_BURST_SHOTS    6U     /* 长点射发数 */
#define WORK_FIRE_CONTINUOUS     0xFFU  /* 持续射击标记 */

/* 设置模式按键连加/连减 */
#define WORK_KEY_REPEAT_DELAY_MS 500U
#define WORK_KEY_REPEAT_MS       100U

/* 保护 */
#define WORK_LOAD_TIMEOUT_MS     5000U  /* 装填最长运行时间 */
#define WORK_MOVE_TIMEOUT_MS     5000U  /* 拉栓/复位最长运行时间 */
#define WORK_STALL_MS            100U   /* 电流持续超阈值判定为堵转 */

/* 状态机 */
#define WORK_ST_IDLE         0U
#define WORK_ST_LOAD         1U
#define WORK_ST_BOLT         2U
#define WORK_ST_RESET        3U
#define WORK_ST_FIRE         4U
#define WORK_ST_TIMEOUT      5U
#define WORK_ST_OVER_CUR     6U

void     Work_Init(void);
void     Work_Task(void);        /* 1ms：到位/超时/堵转判定、射击时序、装填计数 */
void     Work_KeyTask(void);     /* 主循环：工作/调试模式按键动作 */
void     Work_Abort(void);       /* 立即停止电机与射击输出 */

uint8_t  Work_GetState(void);
uint8_t  Work_IsFiring(void);
uint16_t Work_GetAmmo(void);

#ifdef __cplusplus
}
#endif

#endif /* __WORK_H */
