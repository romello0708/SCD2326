/* motor.h —— 装填电机（U/V 两相半桥）PWM 控制接口
 *
 * 模块职责：定义电机状态、PWM 时基参数、加减速曲线与换向死区，声明电机接口。
 * 硬件：TIM1_CH1/CH2 = U/V 高边（PA8/PA9），TIM3_CH2/CH3 = U/V 低边（PA7/PB0），
 *       20kHz（ARR = MOTOR_PWM_PERIOD - 1 = 3599，不分频）。
 * 运行上下文：Motor_Task 在主循环 1ms 调用；Motor_EmergencyStop 同时被 TIM2 中断调用。
 * 关键约束：驱动芯片无硬件死区，换向必须先降到 0 再等 MOTOR_DIR_GUARD_MS。
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 装填电机状态（U/V 两相半桥，EG3112 驱动）
 * STOP(0)：四路门极全关；FORWARD(1)：UH PWM + VL 常通；REVERSE(2)：VH PWM + UL 常通 */
#define MOTOR_STATE_STOP     0U
#define MOTOR_STATE_FORWARD  1U
#define MOTOR_STATE_REVERSE  2U

/* PWM: 72MHz / 3600 = 20kHz */
#define MOTOR_PWM_PERIOD     3600U
#define MOTOR_PWM_FULL       (MOTOR_PWM_PERIOD)  /* 常通比较值，必须为 ARR+1 */
#define MOTOR_DUTY_DEFAULT   3240U               /* 默认占空比 90% */

/* 加减速曲线：每 10ms 变化 200 个计数(约 5.6%)，0→90% 约 170ms。
 * 数值越大启停越猛：过大会造成启动电流冲击/机构抖动，
 * 过小则装填动作起停迟滞；按实物手感调整本常量即可。 */
#define MOTOR_CURVE_PERIOD_MS 10U    /* 步进周期 ms（Motor_Task 内计时） */
#define MOTOR_CURVE_STEP      200U   /* 每步变化量（比较值计数，满量程 3600） */

/* 换向前全关的等待时间，需大于一个 PWM 周期(50us)，保证上下桥不同时导通 */
#define MOTOR_DIR_GUARD_MS    2U

/* 电机运行状态。duty_* 均为比较值（0~MOTOR_PWM_FULL），不是百分比：
 *   duty_default 由 Motor_SetDutyPercent 从百分比换算得到，
 *   duty_set 是目标值，duty_curve 是斜坡后的实际输出值（Motor_Task 使用） */
typedef struct
{
    uint8_t  state;         /* 目标状态 MOTOR_STATE_* */
    uint8_t  state_old;     /* 当前实际输出状态（换向时滞后于 state） */
    uint8_t  guard_ms;      /* 换向全关计时，单位 1ms，累计到 MOTOR_DIR_GUARD_MS 才换向 */
    uint16_t arr;           /* 自动重装值（= MOTOR_PWM_PERIOD - 1 = 3599） */
    uint16_t duty_default;  /* 默认占空比（计数值，0~3600；90% = 3240） */
    uint16_t duty_set;      /* 目标占空比（计数值） */
    uint16_t duty_curve;    /* 斜坡后的当前占空比（计数值，实际输出） */
    uint32_t task_tick;     /* 1ms 任务计时（去重用） */
    uint32_t curve_tick;    /* 加减速曲线计时（每 10ms 一步） */
} Motor_t;

extern Motor_t motor;

/* 接口说明：
 *   Motor_Init          上电初始化：20kHz、四路归零、启动 PWM 通道
 *   Motor_SetState      设定目标方向（内部由 Motor_Task 先降速再换向）
 *   Motor_SetDutyPercent 设定默认占空比（百分比 0~100）
 *   Motor_EmergencyStop 立即全关（到位/松开/过流保护，可在中断内调用）
 *   Motor_Task          主循环 1ms：换向死区、加减速曲线、PWM 输出 */
void Motor_Init(void);
void Motor_SetState(uint8_t state);
void Motor_SetDutyPercent(uint8_t percent);
void Motor_EmergencyStop(void);   /* 立即全关（到位/松开/过流保护） */
void Motor_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
