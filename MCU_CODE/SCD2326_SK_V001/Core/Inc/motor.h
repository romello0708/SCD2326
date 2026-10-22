#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 装填电机状态（U/V 两相半桥，EG3112 驱动） */
#define MOTOR_STATE_STOP     0U
#define MOTOR_STATE_FORWARD  1U
#define MOTOR_STATE_REVERSE  2U

/* PWM: 72MHz / 3600 = 20kHz */
#define MOTOR_PWM_PERIOD     3600U
#define MOTOR_PWM_FULL       (MOTOR_PWM_PERIOD)  /* 常通比较值，必须为 ARR+1 */
#define MOTOR_DUTY_DEFAULT   3240U               /* 默认占空比 90% */

/* 加减速曲线：每 10ms 变化 45 个计数(1.25%)，0→90% 约 720ms */
#define MOTOR_CURVE_PERIOD_MS 10U
#define MOTOR_CURVE_STEP      45U

/* 换向前全关的等待时间，需大于一个 PWM 周期(50us)，保证上下桥不同时导通 */
#define MOTOR_DIR_GUARD_MS    2U

typedef struct
{
    uint8_t  state;         /* 目标状态 */
    uint8_t  state_old;     /* 当前实际输出状态 */
    uint8_t  guard_ms;      /* 换向全关计时 */
    uint16_t arr;           /* 自动重装值 */
    uint16_t duty_default;  /* 默认占空比（计数值） */
    uint16_t duty_set;      /* 目标占空比 */
    uint16_t duty_curve;    /* 斜坡后的当前占空比 */
    uint32_t task_tick;     /* 1ms 任务计时 */
    uint32_t curve_tick;    /* 加减速曲线计时 */
} Motor_t;

extern Motor_t motor;

void Motor_Init(void);
void Motor_SetState(uint8_t state);
void Motor_SetDutyPercent(uint8_t percent);
void Motor_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
