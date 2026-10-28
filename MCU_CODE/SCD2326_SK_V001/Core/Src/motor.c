/* 装填电机 PWM 控制
 * 硬件：PA8=MCU_PwmUH(TIM1_CH1)  PA9=MCU_PwmVH(TIM1_CH2)
 *       PA7=MCU_PwmUL(TIM3_CH2)  PB0=MCU_PwmVL(TIM3_CH3)
 *       经 U14 电平转换 -> U1/U2(EG3112) -> 功率板 U9/U11、U10/U12 两个半桥
 * 说明：两个半桥的上下管由 MCU 分别给信号，驱动芯片无硬件互锁/死区，
 *       因此同一桥臂只允许一只管子导通，换向前必须先全关并等待死区。
 */
#include "motor.h"
#include "tim.h"

Motor_t motor;

/* 四路门极全部关闭
 * 注意：TIM2 1ms 中断里的限位急停（Work_LimitGuardIsr）也会调用本函数，
 * 如果它正好插在下面 4 次比较值写入的中间，剩余几次写入会把已经关断的桥臂
 * 重新打开（例如先关 VH 被急停清零后，再继续写 VL 又会导通）。
 * 因此这里用 PRIMASK 短暂关中断，保证 4 次写入的原子性（仅几微秒）。 */
static void Motor_OutputOff(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  /* UH */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);  /* UL */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);  /* VH */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);  /* VL */
    __set_PRIMASK(primask);
}

/* 按状态和占空比输出：同一桥臂始终只有一只管子导通
 * 同样需要保证 4 次比较值写入不被限位急停插断，见 Motor_OutputOff 的说明。 */
static void Motor_Output(uint8_t state, uint16_t duty)
{
    uint32_t primask;

    if (duty > MOTOR_PWM_FULL) { duty = MOTOR_PWM_FULL; }

    primask = __get_PRIMASK();
    __disable_irq();

    switch (state)
    {
        case MOTOR_STATE_FORWARD:
            /* U 相上桥 PWM，V 相下桥常通 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);               /* VH 关 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_FULL);  /* VL 常通 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);               /* UL 关 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);            /* UH PWM */
            break;

        case MOTOR_STATE_REVERSE:
            /* V 相上桥 PWM，U 相下桥常通 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);               /* UH 关 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_PWM_FULL);  /* UL 常通 */
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);               /* VL 关 */
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);            /* VH PWM */
            break;

        default:
            Motor_OutputOff();
            break;
    }

    __set_PRIMASK(primask);
}

/* 加减速曲线：每个 MOTOR_CURVE_PERIOD_MS 改变一步 */
static void Motor_CurveStep(void)
{
    if (motor.duty_curve < motor.duty_set)
    {
        motor.duty_curve += MOTOR_CURVE_STEP;
        if (motor.duty_curve > motor.duty_set) { motor.duty_curve = motor.duty_set; }
    }
    else if (motor.duty_curve > motor.duty_set)
    {
        if (motor.duty_curve > MOTOR_CURVE_STEP) { motor.duty_curve -= MOTOR_CURVE_STEP; }
        else { motor.duty_curve = 0; }
    }
}

/* 初始化：20kHz PWM，四路先输出低电平 */
void Motor_Init(void)
{
    motor.arr          = MOTOR_PWM_PERIOD - 1U;
    motor.duty_default = MOTOR_DUTY_DEFAULT;
    motor.duty_set     = 0;
    motor.duty_curve   = 0;
    motor.state        = MOTOR_STATE_STOP;
    motor.state_old    = MOTOR_STATE_STOP;
    motor.guard_ms     = 0;
    motor.task_tick    = HAL_GetTick();
    motor.curve_tick   = motor.task_tick;

    Motor_OutputOff();

    __HAL_TIM_SET_PRESCALER(&htim1, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, motor.arr);
    __HAL_TIM_SET_PRESCALER(&htim3, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim3, motor.arr);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* UH */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* VH */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  /* UL */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  /* VL */
}

/* 设置目标状态：0 停止 / 1 正转 / 2 反转 */
void Motor_SetState(uint8_t state)
{
    if (state > MOTOR_STATE_REVERSE) { return; }

    motor.state = state;
    motor.duty_set = (state == MOTOR_STATE_STOP) ? 0U : motor.duty_default;
}

/* 设置默认占空比，0~100% */
void Motor_SetDutyPercent(uint8_t percent)
{
    if (percent > 100U) { percent = 100U; }

    motor.duty_default = (uint16_t)(((uint32_t)MOTOR_PWM_PERIOD * percent) / 100U);

    if (motor.state != MOTOR_STATE_STOP)
    {
        motor.duty_set = motor.duty_default;
    }
}

/* 立即停机：不做斜坡，四路门极直接关闭（用于到位、松开、过流等保护） */
void Motor_EmergencyStop(void)
{
    motor.state      = MOTOR_STATE_STOP;
    motor.state_old  = MOTOR_STATE_STOP;
    motor.duty_set   = 0U;
    motor.duty_curve = 0U;
    motor.guard_ms   = 0U;
    Motor_OutputOff();
}

/* 1ms 周期任务，放在主循环中调用 */
void Motor_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (now == motor.task_tick) { return; }
    motor.task_tick = now;

    /* 1. 方向切换：带载不换向，先把占空比降到 0 */
    if (motor.state != motor.state_old)
    {
        if (motor.duty_curve > 0U)
        {
            motor.duty_set = 0;
            if ((now - motor.curve_tick) >= MOTOR_CURVE_PERIOD_MS)
            {
                motor.curve_tick = now;
                Motor_CurveStep();
            }
            Motor_Output(motor.state_old, motor.duty_curve);
            return;
        }

        /* 占空比已为 0：四路全关，等待死区后切换方向 */
        Motor_OutputOff();
        if (++motor.guard_ms < MOTOR_DIR_GUARD_MS) { return; }
        motor.guard_ms = 0;
        motor.state_old = motor.state;
    }

    /* 2. 加减速曲线 */
    if ((now - motor.curve_tick) >= MOTOR_CURVE_PERIOD_MS)
    {
        motor.curve_tick = now;
        Motor_CurveStep();
    }

    /* 3. 输出 */
    if (motor.state_old == MOTOR_STATE_STOP)
    {
        motor.duty_set   = 0;
        motor.duty_curve = 0;
        Motor_OutputOff();
    }
    else
    {
        Motor_Output(motor.state_old, motor.duty_curve);
    }
}
