/* 装填电机 PWM 控制
 * 硬件：PA8=MCU_PwmUH(TIM1_CH1)  PA9=MCU_PwmVH(TIM1_CH2)
 *       PA7=MCU_PwmUL(TIM3_CH2)  PB0=MCU_PwmVL(TIM3_CH3)
 *       经 U14 电平转换 -> U1/U2(EG3112) -> 功率板 U9/U11、U10/U12 两个半桥
 * 说明：两个半桥的上下管由 MCU 分别给信号，驱动芯片无硬件互锁/死区，
 *       因此同一桥臂只允许一只管子导通，换向前必须先全关并等待死区。
 *
 * 调制方式：PWM 频率 20kHz（TIM1/TIM3 都是 ARR=3599、不分频：72MHz/3600 = 20kHz），
 *           U/V 两相采用“上桥 PWM + 另一相下桥常通”的方式驱动，真值表见 Motor_Output：
 *             正转：UH = duty（PWM），VL = 常通，其余三路 0；
 *             反转：VH = duty（PWM），UL = 常通，其余三路 0。
 *           占空比即电机端电压比例，默认 90%（MOTOR_DUTY_DEFAULT）。
 *
 * 运行上下文：
 *   Motor_Init / Motor_SetState / Motor_SetDutyPercent —— 上电初始化、Work 状态机（主循环）
 *   Motor_EmergencyStop —— 主循环（到位/松开/过流）与 TIM2 中断（限位急停）都会调用，
 *                          内部用 PRIMASK 保护四路 CCR 的原子写入
 *   Motor_Task —— 主循环 1ms 任务：斜坡、换向死区、输出刷新
 *
 * 关键时间参数：
 *   斜坡步进 MOTOR_CURVE_PERIOD_MS = 10ms，步长 MOTOR_CURVE_STEP = 200 计数（约 5.6%），
 *   0 → 90% 约 170ms；换向死区 MOTOR_DIR_GUARD_MS = 2ms（> 1 个 PWM 周期 50us）。
 */
#include "motor.h"
#include "tim.h"

Motor_t motor;

/* 四路门极全部关闭
 * 参数：无
 * 返回：无
 * 调用：Motor_Init / Motor_EmergencyStop / Motor_Task（换向死区、停止态）/ Motor_Output 默认分支
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
 * 参数：state —— MOTOR_STATE_FORWARD（正转）/ MOTOR_STATE_REVERSE（反转）/ 其它 = 全关
 *       duty  —— 比较值 0 ~ MOTOR_PWM_FULL(3600)，3600 即 100%（常通）
 * 返回：无
 * 调用：Motor_Task 每 1ms 输出刷新（主循环）
 * 注意：写入顺序是“先关该桥臂的另一只管子，再开目标管子”，
 *       即使中途被中断打断也不会出现同桥臂直通；
 *       同样需要保证 4 次比较值写入不被限位急停插断，见 Motor_OutputOff 的说明。 */
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

/* 加减速曲线：每个 MOTOR_CURVE_PERIOD_MS 改变一步
 * 参数：无
 * 返回：无
 * 调用：Motor_Task（正常斜坡与换向降速两个分支），由 curve_tick 保证 10ms 一步
 * 注意：只改 duty_curve（实际输出值），不改 duty_set（目标值）；
 *       下降时若不足一步则直接归 0，避免残值使电机嗡嗡响
 */
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

/* 初始化：20kHz PWM，四路先输出低电平
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main），在 Relay_Init 之后、AD_Init 之前
 * 注意：必须在 AD_Init 之前调用，保证电流零点校准时电机确实处于静止；
 *       先 OutputOff 再启动 PWM 通道，避免启动瞬间输出旧的比较值
 */
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

/* 设置目标状态：0 停止 / 1 正转 / 2 反转
 * 参数：state —— MOTOR_STATE_STOP(0) / MOTOR_STATE_FORWARD(1) / MOTOR_STATE_REVERSE(2)，越界忽略
 * 返回：无
 * 调用：Work_StartMotion（主循环）启动动作时
 * 注意：这里只改目标状态和目标占空比，真正的换向由 Motor_Task 先降到 0 再切；
 *       停止时目标占空比置 0，转起来时恢复到 duty_default
 */
void Motor_SetState(uint8_t state)
{
    if (state > MOTOR_STATE_REVERSE) { return; }

    motor.state = state;
    motor.duty_set = (state == MOTOR_STATE_STOP) ? 0U : motor.duty_default;
}

/* 设置默认占空比，0~100%
 * 参数：percent —— 百分比 0~100，超过 100 按 100 处理
 * 返回：无
 * 调用：Work_StartMotion（启动动作前按 g_cfg.motor_duty_pct 设置）
 * 注意：换算成 0~MOTOR_PWM_PERIOD 的比较值；仅在电机非停止时才同步到 duty_set，
 *       避免停止状态下改占空比把电机意外启动
 */
void Motor_SetDutyPercent(uint8_t percent)
{
    if (percent > 100U) { percent = 100U; }

    motor.duty_default = (uint16_t)(((uint32_t)MOTOR_PWM_PERIOD * percent) / 100U);

    if (motor.state != MOTOR_STATE_STOP)
    {
        motor.duty_set = motor.duty_default;
    }
}

/* 立即停机：不做斜坡，四路门极直接关闭（用于到位、松开、过流等保护）
 * 参数：无
 * 返回：无
 * 调用：主循环（Work_StopMotion / Work_Init）与 TIM2 1ms 中断（Work_LimitGuardIsr）
 * 注意：可重入且中断安全 —— 函数内只做赋值和 CCR 写入，CCR 写入受 PRIMASK 保护；
 *       同时把 state_old 归为 STOP，使 Motor_Task 下一轮不再按旧方向输出
 */
void Motor_EmergencyStop(void)
{
    motor.state      = MOTOR_STATE_STOP;
    motor.state_old  = MOTOR_STATE_STOP;
    motor.duty_set   = 0U;
    motor.duty_curve = 0U;
    motor.guard_ms   = 0U;
    Motor_OutputOff();
}

/* 1ms 周期任务，放在主循环中调用
 * 参数：无
 * 返回：无
 * 调用：主循环每轮调用，内部用 task_tick 去重到 1ms
 * 注意：处理顺序固定为“换向 → 斜坡 → 输出”：
 *       1) 方向不同且还有占空比时，按曲线降到 0 并保持旧方向输出，然后 return；
 *       2) 占空比已为 0 时四路全关，累计 guard_ms 达到 MOTOR_DIR_GUARD_MS 才允许换向；
 *       3) 之后每 10ms 推进一步斜坡，并按 state_old 输出。
 *       换向段直接 return 是有意的：本函数末尾的输出刷新会用旧方向覆盖，
 *       只有在 guard 完成、state_old 更新后才会走正常输出路径
 */
void Motor_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (now == motor.task_tick) { return; }
    motor.task_tick = now;

    /* 1. 方向切换：带载不换向，先把占空比降到 0
     * 降速期间仍按“旧方向”输出，否则成对桥臂瞬间互换会造成反电动势短路 */
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

        /* 占空比已为 0：四路全关，等待死区后切换方向
         * guard_ms 每 1ms 自增一次（本函数已按 1ms 去重），达到 2ms 才真正换向 */
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
