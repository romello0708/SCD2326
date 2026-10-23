/* 工作 / 调试模式动作控制
 *
 * 工作模式（拨动开关中挡）：
 *   A = 装填：电机向前运行到到位位置(IN_10)，超时或堵转则停机
 *   C = 射击：按设置模式选定的射击模式，经 MCU_OUT1 输出
 *   B = 无作用
 *   装填与射击互斥：装填过程中不响应射击，射击过程中不响应装填
 *
 * 调试模式（拨动开关下挡）：
 *   A 长按(>=300ms) = 拉栓：电机向后运行到初始到位(IN_9)，松开立即停止
 *   B 长按(>=300ms) = 复位：电机向前运行到到位位置(IN_10)，松开立即停止
 *   C 长按(>=300ms) = 射击：MCU_OUT1 按固定时序连续输出，松开立即停止，
 *                     余弹同样逐发递减；各动作之间保持安全互斥
 *
 * 射击模式：
 *   无限制：按住连发，不检查、不扣减余弹
 *   连射  ：按住连发，每发扣减余弹，余弹为 0 停止
 *   长点射：每按一次 6 发
 *   点射  ：每按一次 3 发
 *   单发  ：每按一次 1 发
 *
 * 余弹：设置模式可直接装定 0~200；装填过程中 IN_7 每个上升沿 +1（上限 200）。
 */
#include "work.h"
#include "global.h"
#include "digital_input.h"
#include "motor.h"
#include "AnalogQuantityAcquisition.h"
#include "eeprom.h"

static uint8_t  s_state;          /* WORK_ST_* */
static uint8_t  s_last_mode;      /* 上次工作模式，用于切挡中止 */
static uint32_t s_tick;           /* 1ms 节拍 */
static uint32_t s_start_tick;     /* 电机动作开始时刻 */
static uint32_t s_fire_tick;      /* 当前脉宽/间隔起点 */
static uint16_t s_stall_ms;       /* 电流超阈值持续时间 */
static uint16_t s_last_cnt;       /* IN_7 上升沿计数上次值 */
static uint8_t  s_prev[3];        /* A/B/C 上次电平 */
static uint32_t s_hold_tick[3];   /* A/B/C 按下时刻，用于长按判定 */
static uint8_t  s_hold_act[3];    /* A/B/C 长按已触发标记 */
static uint8_t  s_ammo_dirty;     /* 1 = 余弹有未保存的变化 */
static uint32_t s_idle_tick;      /* 最近一次非空闲时刻 */

static uint8_t  s_fire_left;      /* 剩余发数，WORK_FIRE_CONTINUOUS 表示持续 */
static uint8_t  s_fire_level;     /* 1 = MCU_OUT1 当前为高 */
static uint8_t  s_fire_unlimit;   /* 1 = 不检查、不扣减余弹 */

/*--------------------- 输出 ---------------------*/
static void Work_FireOn(void)
{
    HAL_GPIO_WritePin(MCU_OUT1_GPIO_Port, MCU_OUT1_Pin, GPIO_PIN_SET);
}

static void Work_FireOff(void)
{
    HAL_GPIO_WritePin(MCU_OUT1_GPIO_Port, MCU_OUT1_Pin, GPIO_PIN_RESET);
}

static void Work_StopMotion(void)
{
    Motor_EmergencyStop();
    s_stall_ms = 0U;
    if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_BOLT) || (s_state == WORK_ST_RESET))
    {
        s_state = WORK_ST_IDLE;
    }
}

/* 结束射击：关输出并回到空闲 */
static void Work_FireStop(void)
{
    Work_FireOff();
    s_fire_level = 0U;
    s_fire_left  = 0U;
    if (s_state == WORK_ST_FIRE)
    {
        s_state = WORK_ST_IDLE;
    }
}

/* 是否允许再打一发 */
static uint8_t Work_FireReady(void)
{
    if (s_fire_unlimit != 0U)
    {
        return 1U;
    }
    return (g_cfg.ammo > AMMO_MIN) ? 1U : 0U;
}

/* 是否允许启动新动作：空闲，或上一次动作异常结束（超时/过流） */
static uint8_t Work_CanStart(void)
{
    if ((s_state == WORK_ST_IDLE) || (s_state == WORK_ST_TIMEOUT) || (s_state == WORK_ST_OVER_CUR))
    {
        return 1U;
    }
    return 0U;
}

/* 打出一发：拉高输出、扣余弹、计数 */
static void Work_FirePulse(void)
{
    Work_FireOn();
    s_fire_level = 1U;
    s_fire_tick  = HAL_GetTick();

    if (s_fire_unlimit == 0U)
    {
        g_cfg.ammo--;
        s_ammo_dirty = 1U;
    }
    g_cfg.shot_total++;
    s_state = WORK_ST_FIRE;
}

/* 启动射击序列：shots = 发数，WORK_FIRE_CONTINUOUS 为按住连发 */
static void Work_FireStart(uint8_t shots, uint8_t unlimit)
{
    s_fire_left    = shots;
    s_fire_unlimit = unlimit;

    if (Work_FireReady() == 0U)
    {
        Work_FireStop();
        return;
    }

    Work_FirePulse();
}

/*--------------------- 对外：初始化 / 中止 ---------------------*/
void Work_Init(void)
{
    Work_FireOff();
    Motor_EmergencyStop();

    s_state      = WORK_ST_IDLE;
    s_last_mode  = g_cfg.work_mode;
    s_tick       = HAL_GetTick();
    s_start_tick = s_tick;
    s_fire_tick  = s_tick;
    s_stall_ms   = 0U;
    s_last_cnt   = DI_GetRiseCount(WORK_COUNT_CH);

    s_fire_left    = 0U;
    s_fire_level   = 0U;
    s_fire_unlimit = 0U;

    s_ammo_dirty = 0U;
    s_idle_tick  = s_tick;

    s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
    s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
    s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);

    s_hold_tick[0] = s_tick;
    s_hold_tick[1] = s_tick;
    s_hold_tick[2] = s_tick;
    s_hold_act[0]  = 0U;
    s_hold_act[1]  = 0U;
    s_hold_act[2]  = 0U;
}

void Work_Abort(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  key[3];

    Work_FireOff();
    Motor_EmergencyStop();

    s_fire_level = 0U;
    s_fire_left  = 0U;
    s_stall_ms   = 0U;
    s_state      = WORK_ST_IDLE;

    /* 切挡时同步按键快照：仍按住的键视为已触发，松开后才能再次启动 */
    key[0] = DI_IsHigh(MENU_KEY_A_CH);
    key[1] = DI_IsHigh(MENU_KEY_B_CH);
    key[2] = DI_IsHigh(MENU_KEY_C_CH);

    s_prev[0] = key[0];
    s_prev[1] = key[1];
    s_prev[2] = key[2];

    s_hold_tick[0] = now;
    s_hold_tick[1] = now;
    s_hold_tick[2] = now;
    s_hold_act[0]  = key[0];
    s_hold_act[1]  = key[1];
    s_hold_act[2]  = key[2];
}

/*--------------------- 动作启动 ---------------------*/
static void Work_StartMotion(uint8_t state, uint8_t dir)
{
    Work_FireStop();                 /* 动作前先断开击发输出，避免同时动作 */

    Motor_SetDutyPercent(g_cfg.motor_duty_pct);
    Motor_SetState(dir);

    s_state      = state;
    s_start_tick = HAL_GetTick();
    s_stall_ms   = 0U;
}

/*--------------------- 1ms 任务 ---------------------*/
void Work_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t cnt;
    float imag;
    float iu;

    if (now == s_tick)
    {
        return;
    }
    s_tick = now;

    /* 切挡：立即停止所有输出，避免带载切换 */
    if (g_cfg.work_mode != s_last_mode)
    {
        s_last_mode = g_cfg.work_mode;
        Work_Abort();
        return;
    }

    /* IN_7：装填过程中每个上升沿余弹 +1 */
    cnt = DI_GetRiseCount(WORK_COUNT_CH);
    if (cnt != s_last_cnt)
    {
        if ((s_state == WORK_ST_LOAD) && (g_cfg.work_mode != WORK_MODE_SET))
        {
            uint16_t add = (uint16_t)(cnt - s_last_cnt);
            uint16_t ammo = (uint16_t)(g_cfg.ammo + add);

            g_cfg.ammo = (ammo > AMMO_MAX) ? AMMO_MAX : ammo;
            s_ammo_dirty = 1U;
        }
        s_last_cnt = cnt;
    }

    /* 电机动作中：总电流与 U 相取较大值做堵转判定 */
    if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_BOLT) || (s_state == WORK_ST_RESET))
    {
        imag = AD_GetCurrent(AD_INDEX_ALL);
        if (imag < 0.0f) { imag = -imag; }

        iu = AD_GetCurrent(AD_INDEX_U);
        if (iu < 0.0f) { iu = -iu; }
        if (iu > imag) { imag = iu; }

        if (imag >= (float)g_cfg.over_current_a)
        {
            if (++s_stall_ms >= WORK_STALL_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_OVER_CUR;
                return;
            }
        }
        else
        {
            s_stall_ms = 0U;
        }
    }

    switch (s_state)
    {
        case WORK_ST_LOAD:
            if (DI_IsHigh(WORK_FRONT_CH) != 0U)
            {
                Work_StopMotion();                 /* 到位位置 */
            }
            else if ((now - s_start_tick) >= WORK_LOAD_TIMEOUT_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_BOLT:
            if (DI_IsHigh(WORK_REAR_CH) != 0U)
            {
                Work_StopMotion();                 /* 初始到位 */
            }
            else if ((now - s_start_tick) >= WORK_MOVE_TIMEOUT_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_RESET:
            if (DI_IsHigh(WORK_FRONT_CH) != 0U)
            {
                Work_StopMotion();                 /* 到位位置 */
            }
            else if ((now - s_start_tick) >= WORK_MOVE_TIMEOUT_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_FIRE:
            if (s_fire_level != 0U)
            {
                /* 脉宽结束 */
                if ((now - s_fire_tick) >= WORK_SHOT_PULSE_MS)
                {
                    Work_FireOff();
                    s_fire_level = 0U;
                    s_fire_tick  = now;

                    if ((s_fire_left != WORK_FIRE_CONTINUOUS) && (s_fire_left > 0U))
                    {
                        s_fire_left--;
                    }
                }
            }
            else
            {
                /* 间隔结束：还有弹就继续打 */
                if ((now - s_fire_tick) >= WORK_SHOT_GAP_MS)
                {
                    if ((s_fire_left == 0U) || (Work_FireReady() == 0U))
                    {
                        Work_FireStop();
                    }
                    else
                    {
                        Work_FirePulse();
                    }
                }
            }
            break;

        default:
            break;
    }

    /* 余弹掉电保存：动作结束并空闲 WORK_AMMO_SAVE_MS 后写一次 EEPROM */
    if (s_state != WORK_ST_IDLE)
    {
        s_idle_tick = now;
    }
    else if ((s_ammo_dirty != 0U) && ((now - s_idle_tick) >= WORK_AMMO_SAVE_MS))
    {
        s_idle_tick = now;              /* 失败则再等一个周期重试，避免频繁阻塞 */
        if (EEPROM_SaveConfig() == HAL_OK)
        {
            s_ammo_dirty = 0U;
        }
    }
}

/*--------------------- 按键任务（工作 / 调试模式） ---------------------*/
void Work_KeyTask(void)
{
    uint8_t kA = DI_IsHigh(MENU_KEY_A_CH);
    uint8_t kB = DI_IsHigh(MENU_KEY_B_CH);
    uint8_t kC = DI_IsHigh(MENU_KEY_C_CH);

    if (g_cfg.work_mode == WORK_MODE_WORK)
    {
        /* 装填：按一下自动运行到到位位置 */
        if ((kA != 0U) && (s_prev[0] == 0U))
        {
            if (Work_CanStart() != 0U)
            {
                Work_StartMotion(WORK_ST_LOAD, WORK_DIR_LOAD);
            }
        }

        /* 射击：按射击模式发起；连发/无限制需持续按住 */
        if ((kC != 0U) && (s_prev[2] == 0U))
        {
            if (Work_CanStart() != 0U)        /* 与装填互斥 */
            {
                switch (g_cfg.fire_mode)
                {
                    case FIRE_MODE_BURST:
                        Work_FireStart(WORK_BURST_SHOTS, 0U);
                        break;
                    case FIRE_MODE_LONG_BURST:
                        Work_FireStart(WORK_LONG_BURST_SHOTS, 0U);
                        break;
                    case FIRE_MODE_AUTO:
                        Work_FireStart(WORK_FIRE_CONTINUOUS, 0U);
                        break;
                    case FIRE_MODE_UNLIMITED:
                        Work_FireStart(WORK_FIRE_CONTINUOUS, 1U);
                        break;
                    default:                   /* 单发 */
                        Work_FireStart(1U, 0U);
                        break;
                }
            }
        }
        else if ((kC == 0U) && (s_prev[2] != 0U))
        {
            /* 松开：连发/无限制立即停止，点射按完整序列打完 */
            if ((g_cfg.fire_mode == FIRE_MODE_AUTO) || (g_cfg.fire_mode == FIRE_MODE_UNLIMITED))
            {
                Work_FireStop();
            }
        }
    }
    else if (g_cfg.work_mode == WORK_MODE_DEBUG)
    {
        uint32_t now = HAL_GetTick();

        /* 拉栓：长按 300ms 启动，松开立即停止 */
        if (kA != 0U)
        {
            if (s_prev[0] == 0U)
            {
                s_hold_tick[0] = now;
                s_hold_act[0]  = 0U;
            }
            if ((s_hold_act[0] == 0U) && ((now - s_hold_tick[0]) >= WORK_HOLD_MS))
            {
                s_hold_act[0] = 1U;
                if (Work_CanStart() != 0U)
                {
                    Work_StartMotion(WORK_ST_BOLT, WORK_DIR_BOLT);
                }
            }
        }
        else
        {
            s_hold_act[0] = 0U;
            if ((s_prev[0] != 0U) && (s_state == WORK_ST_BOLT))
            {
                Work_StopMotion();
            }
        }

        /* 复位：长按 300ms 启动，松开立即停止（A 优先） */
        if ((kB != 0U) && (kA == 0U))
        {
            if (s_prev[1] == 0U)
            {
                s_hold_tick[1] = now;
                s_hold_act[1]  = 0U;
            }
            if ((s_hold_act[1] == 0U) && ((now - s_hold_tick[1]) >= WORK_HOLD_MS))
            {
                s_hold_act[1] = 1U;
                if (Work_CanStart() != 0U)
                {
                    Work_StartMotion(WORK_ST_RESET, WORK_DIR_RESET);
                }
            }
        }
        else
        {
            s_hold_act[1] = 0U;
            if ((s_prev[1] != 0U) && (s_state == WORK_ST_RESET))
            {
                Work_StopMotion();
            }
        }

        /* 射击：长按 300ms 启动，按固定时序连续击发并扣减余弹，松开立即停止 */
        if (kC != 0U)
        {
            if (s_prev[2] == 0U)
            {
                s_hold_tick[2] = now;
                s_hold_act[2]  = 0U;
            }
            if ((s_hold_act[2] == 0U) && ((now - s_hold_tick[2]) >= WORK_HOLD_MS))
            {
                s_hold_act[2] = 1U;
                if (Work_CanStart() != 0U)
                {
                    Work_FireStart(WORK_FIRE_CONTINUOUS, 0U);
                }
            }
        }
        else
        {
            s_hold_act[2] = 0U;
            if (s_prev[2] != 0U)
            {
                Work_FireStop();
            }
        }
    }

    s_prev[0] = kA;
    s_prev[1] = kB;
    s_prev[2] = kC;
}

/*--------------------- 状态查询 / 装定 ---------------------*/
uint8_t Work_GetState(void)
{
    return s_state;
}

uint8_t Work_IsFiring(void)
{
    return (s_state == WORK_ST_FIRE) ? 1U : 0U;
}

uint16_t Work_GetAmmo(void)
{
    return g_cfg.ammo;
}
