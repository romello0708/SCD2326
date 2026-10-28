/* work.h —— 工作 / 调试模式动作控制接口
 *
 * 模块职责：
 *   1) 定义动作状态机（装填 / 回退 / 拉栓 / 复位 / 射击 / 超时 / 过流）；
 *   2) 定义输入通道号、逻辑方向、保护时间与射击计数窗口参数；
 *   3) 声明 work.c 的初始化、主循环任务、中断任务与状态查询接口。
 *
 * 运行上下文：
 *   Work_Init       —— 上电初始化（DI_Init 之后、TIM2 启动之前）
 *   Work_Task       —— 主循环 1ms 任务（状态机、到位/超时/堵转、射击时序）
 *   Work_CountTask / Work_LimitGuardIsr —— TIM2 1ms 中断（计数窗口/限位急停）
 *   Work_KeyTask    —— 主循环按键任务
 *   Work_Abort      —— 切挡时立即停机
 *
 * 依赖：通道号与 g_cfg（global.h）中的参数配合使用；本文件不含任何代码。
 */

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
 *   IN_10 = 位置传感器·初始到位信号：装填回退的终点，
 *           也是调试模式“复位”的终点；OLED 调试页第二行“初始到位信号”即此路。
 *   IN_9  = 位置传感器·到位位置信号：装填正转的终点，
 *           也是调试模式“拉栓”的终点；OLED 调试页第三行“到位位置信号”即此路。
 *   （两路位置信号共用 RELAY_2，由设置模式“位置传感器类型”选择 NPN/PNP）
 *   装填过程不使用任何计数信号，只靠 IN_9 / IN_10 两个位置信号判断行程。
 *
 * 输出：MCU_OUT1(PA12) 高电平 = 射击（击发）
 */

/* 输入通道 */
#define WORK_COUNT_CH         7U     /* 计数传感器：射击计数（上升沿 -1 发） */
#define WORK_INIT_CH         10U     /* 初始到位信号：装填回退、复位终点 */
#define WORK_TARGET_CH        9U     /* 到位位置信号：装填正转、拉栓终点 */

/* 动作方向：装填正转朝“到位位置信号”(IN_9)运行，到达后反向回退，
 *  退回“初始到位信号”(IN_10)停止。
 *  调试模式必须与工作模式同向：
 *      拉栓 = 装填正转方向，终点为到位位置 IN_9；
 *      复位 = 装填回退方向，终点为初始到位 IN_10。
 *  本版本按实物调整：装填正转用 FORWARD。
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
#define WORK_LOAD_TIMEOUT_MS    15000U  /* 装填（正转 + 回退）最长运行时间 */
#define WORK_MOVE_TIMEOUT_MS    15000U  /* 调试拉栓/复位最长运行时间 */
#define WORK_STALL_MS            100U   /* 电流持续超阈值判定为堵转 */

/* 工作模式射击计数窗口（IN_7） */
#define WORK_CNT_DELAY_MS        200U   /* 击发信号结束后屏蔽时间 */
#define WORK_CNT_HOLD_MS         600U   /* 无击发后计数窗口保持时间 */

/* 状态机
 * 0 IDLE       : 空闲，可接受新动作或射击
 * 1 LOAD       : 装填正转，终点 IN_9
 * 2 BOLT       : 调试拉栓，终点 IN_9，松开即停
 * 3 RESET      : 调试复位，终点 IN_10，松开即停
 * 4 FIRE       : 射击中，按 pulse_ms/gap_ms 时序推进
 * 5 TIMEOUT    : 动作超时（15s），可再次按键启动
 * 6 OVER_CUR   : 堵转/过流保护停机，可再次按键启动
 * 7 LOAD_BACK  : 装填到位后回退，终点 IN_10
 * 说明：只有 0/5/6 允许启动新动作（见 work.c 的 Work_CanStart） */
#define WORK_ST_IDLE         0U
#define WORK_ST_LOAD         1U
#define WORK_ST_BOLT         2U
#define WORK_ST_RESET        3U
#define WORK_ST_FIRE         4U
#define WORK_ST_TIMEOUT      5U
#define WORK_ST_OVER_CUR     6U
#define WORK_ST_LOAD_BACK    7U    /* 装填到位后回退到初始到位 */

/* 接口约定：
 *   Work_Init        上电初始化，必须在 DI_Init 之后、TIM2 启动之前调用
 *   Work_Task        主循环 1ms：到位/超时/堵转判定、射击时序
 *   Work_CountTask   TIM2 1ms 中断：击发窗口判定 + IN_7 射击计数（-1 发/上升沿）
 *   Work_LimitGuardIsr TIM2 1ms 中断：到限位立即断电，不等主循环
 *   Work_KeyTask     主循环：工作/调试模式按键动作（边沿 + 长按）
 *   Work_Abort       立即停止电机与射击输出（切挡时用），并复位按键快照 */
void     Work_Init(void);
void     Work_Task(void);        /* 主循环 1ms：到位/超时/堵转判定、射击时序 */
void     Work_CountTask(void);   /* TIM2 1ms 中断：击发窗口判定 + IN_7 射击计数 */
void     Work_LimitGuardIsr(void); /* TIM2 1ms 中断：到限位立即断电，不等主循环 */
void     Work_KeyTask(void);     /* 主循环：工作/调试模式按键动作 */
void     Work_Abort(void);       /* 立即停止电机与射击输出 */

uint8_t  Work_GetState(void);     /* 返回 WORK_ST_* 当前状态（供界面显示） */
uint8_t  Work_IsFiring(void);     /* 返回 1 = 正在射击 */
uint16_t Work_GetAmmo(void);      /* 返回当前余弹数 0~200（中断内可能正在递减） */

#ifdef __cplusplus
}
#endif

#endif /* __WORK_H */
