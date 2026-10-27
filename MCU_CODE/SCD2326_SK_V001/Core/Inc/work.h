#ifndef __WORK_H
#define __WORK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 工作 / 调试模式动作控制
 * 面板：A=IN_6、B=IN_5、C=IN_4；拨动开关 IN_2 / IN_3 决定设置/工作/调试
 *
 * 输入（V0.3 实际接线，两个传感器类型分别由两路继电器配置）：
 *   IN_7  = 计数传感器 —— 只做“射击计数”：每个上升沿代表实际打出 1 发，
 *           余弹数 -1；装填/回退/拉栓/复位等空行程期间忽略该信号。
 *           其 NPN/PNP 类型由 RELAY_1 配置（设置模式“计数传感器类型”）。
 *   IN_10 = 位置传感器·前到位：装填正转的终点，
 *           也是调试模式“拉栓”的终点；OLED 调试页第二行“初始到位信号”即此路。
 *   IN_9  = 位置传感器·后到位：装填回退的终点，
 *           也是调试模式“复位”的终点；OLED 调试页第三行“到位位置信号”即此路。
 *   （两路位置信号共用 RELAY_2，由设置模式“位置传感器类型”选择 NPN/PNP）
 *   装填过程不再使用任何计数信号，只靠 IN_9 / IN_10 两个位置信号判断行程。
 *
 * 输出：MCU_OUT1(PA12) 高电平 = 射击（击发）
 */

/* 输入通道 */
#define WORK_COUNT_CH         7U     /* 计数传感器：射击计数（上升沿 -1 发） */
#define WORK_FRONT_CH        10U     /* 前到位：装填正转终点、拉栓终点（OLED“初始到位信号”） */
#define WORK_REAR_CH          9U     /* 后到位：装填回退终点、复位终点（OLED“到位位置信号”） */

/* 动作方向：装填朝前到位(IN_10)运行，到位后反向退回后到位(IN_9)。
 *  调试模式必须与工作模式同向：
 *      拉栓 = 装填正转方向，终点为前到位 IN_10（OLED“初始到位信号”）；
 *      复位 = 装填回退方向，终点为后到位 IN_9（OLED“到位位置信号”）。
 *  本版本再次整体反转：装填正转用 FORWARD。
 *  若还需整体反转，可在设置模式把“电机方向”设为反向。 */
#define WORK_DIR_LOAD        MOTOR_STATE_FORWARD

/* 装填回退方向：与装填方向相反 */
#define WORK_DIR_LOAD_BACK   ((WORK_DIR_LOAD == MOTOR_STATE_FORWARD) ? MOTOR_STATE_REVERSE : MOTOR_STATE_FORWARD)

/* 调试模式方向：直接引用工作模式方向，保证两者永远一致 */
#define WORK_DIR_BOLT        WORK_DIR_LOAD        /* 拉栓：与装填正转同向 */
#define WORK_DIR_RESET       WORK_DIR_LOAD_BACK   /* 复位：与装填回退同向 */

/* 射击时序：单发脉宽、双发间隔、点射/长点射发数均由设置模式装定（见 g_cfg） */
#define WORK_FIRE_CONTINUOUS     0xFFU  /* 持续射击标记 */

/* 设置模式按键连加/连减 */
#define WORK_KEY_REPEAT_DELAY_MS 500U
#define WORK_KEY_REPEAT_MS       100U

/* 调试模式长按启动判定时间 */
#define WORK_HOLD_MS             300U

/* 余弹掉电保存：已取消。余弹数只在内存中维护，上电回到默认值，
 * 由设置模式重新装定；详见 work.c 中 IN_7 计数说明 */

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
#define WORK_ST_LOAD_BACK    7U    /* 装填到位后回退到初始到位 */

void     Work_Init(void);
void     Work_Task(void);        /* 1ms：到位/超时/堵转判定、射击时序、IN_7 射击计数 */
void     Work_KeyTask(void);     /* 主循环：工作/调试模式按键动作 */
void     Work_Abort(void);       /* 立即停止电机与射击输出 */

uint8_t  Work_GetState(void);
uint8_t  Work_IsFiring(void);
uint16_t Work_GetAmmo(void);

#ifdef __cplusplus
}
#endif

#endif /* __WORK_H */
