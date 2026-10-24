/* 工作 / 调试模式动作控制
 *
 * 工作模式（拨动开关中挡）：
 *   A = 装填：按下后电机按设定方向转动，收到前到位信号(IN_10)或余弹达到装定值后停止，
 *       随即反向回退，收到后到位信号(IN_9)停止；前后到位信号同时有效时禁止电机动作；
 *       装填/回退过程中再次按下 A 立即停止
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
#include "log.h"

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
static uint16_t s_load_ammo0;     /* 本次装填开始时的余弹数（用于判断装填中是否增加） */
static uint8_t  s_di_front;       /* 上次前到位电平（日志用） */
static uint8_t  s_di_rear;        /* 上次后到位电平（日志用） */

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
    LOG("[电机] 停止 状态=%d\r\n", s_state);
    Motor_EmergencyStop();
    s_stall_ms = 0U;
    if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_LOAD_BACK) ||
        (s_state == WORK_ST_BOLT) || (s_state == WORK_ST_RESET))
    {
        s_state = WORK_ST_IDLE;
    }
}

/* 结束射击：关输出并回到空闲 */
static void Work_FireStop(void)
{
    if (s_state == WORK_ST_FIRE)
    {
        LOG("[射击] 停止 余弹=%d\r\n", g_cfg.ammo);
    }
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

/* 前后到位信号同时有效：位置异常，禁止电机动作 */
static uint8_t Work_PosInvalid(void)
{
    if ((DI_IsHigh(WORK_FRONT_CH) != 0U) && (DI_IsHigh(WORK_REAR_CH) != 0U))
    {
        return 1U;
    }
    return 0U;
}

/* 按设置模式的“电机方向”换算实际转动方向（工作/调试模式通用） */
static uint8_t Work_ApplyDir(uint8_t dir)
{
    if (g_cfg.load_dir == LOAD_DIR_REVERSE)
    {
        return (dir == MOTOR_STATE_FORWARD) ? MOTOR_STATE_REVERSE : MOTOR_STATE_FORWARD;
    }
    return dir;
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
    LOG("[射击] 开始 模式=%d 发数=%d 无限制=%d 余弹=%d\r\n",
        g_cfg.fire_mode, shots, unlimit, g_cfg.ammo);
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

    s_di_front = DI_IsHigh(WORK_FRONT_CH);
    s_di_rear  = DI_IsHigh(WORK_REAR_CH);
}

void Work_Abort(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  key[3];

    LOG("[中止] 切挡或取消，全部输出关闭\r\n");
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
    uint8_t real_dir;

    if (Work_PosInvalid() != 0U)
    {
        LOG("[电机] 拒绝启动：前后到位同时有效\r\n");
        return;                      /* 前后到位同时有效：禁止电机动作 */
    }

    Work_FireStop();                 /* 动作前先断开击发输出，避免同时动作 */

    real_dir = Work_ApplyDir(dir);
    LOG("[电机] 启动 状态=%d 方向=%d\r\n", state, real_dir);
    Motor_SetDutyPercent(g_cfg.motor_duty_pct);
    Motor_SetState(real_dir);

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
        LOG("[挡位] %d\r\n", g_cfg.work_mode);
        Work_Abort();
        return;
    }

    /* 到位信号变化打印 */
    {
        uint8_t f = DI_IsHigh(WORK_FRONT_CH);
        uint8_t r = DI_IsHigh(WORK_REAR_CH);

        if ((f != s_di_front) || (r != s_di_rear))
        {
            s_di_front = f;
            s_di_rear  = r;
            LOG("[到位] 前=%d 后=%d\r\n", f, r);
        }
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
            LOG("[余弹] %d\r\n", g_cfg.ammo);
        }
        s_last_cnt = cnt;
    }

    /* 电机动作中：总电流与 U 相取较大值做堵转判定 */
    if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_LOAD_BACK) ||
        (s_state == WORK_ST_BOLT) || (s_state == WORK_ST_RESET))
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
                LOG("[电机] 过流 %dmA\r\n", (int)(imag * 1000.0f));
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
            if (Work_PosInvalid() != 0U)
            {
                Work_StopMotion();                 /* 前后到位同时有效：立即停止 */
            }
            else if ((DI_IsHigh(WORK_FRONT_CH) != 0U) ||
                     ((g_cfg.ammo >= g_cfg.ammo_target) && (g_cfg.ammo > s_load_ammo0)))
            {
                /* 前到位，或装填中余弹增加到装定值：先立即断电再从静止反向回退。
                 * 注意：不能直接切换方向。换向需要先把占空比按曲线降到 0（约数百 ms），
                 * 这期间电机已被限位挡住仍持续通电，会被堵转过流判定直接停机，
                 * 表现为“到前到位就停、不反转”。此处立即关断可避免堵转电流。 */
                LOG("[装填] 完成(%s) 余弹=%d 装定=%d -> 回退\r\n",
                    (DI_IsHigh(WORK_FRONT_CH) != 0U) ? "前到位" : "达装定值",
                    g_cfg.ammo, g_cfg.ammo_target);
                Work_StopMotion();               /* 先急停，清掉斜坡与堵转计时 */
                Work_StartMotion(WORK_ST_LOAD_BACK, WORK_DIR_LOAD_BACK);
            }
            else if ((now - s_start_tick) >= WORK_LOAD_TIMEOUT_MS)
            {
                LOG("[电机] 装填超时\r\n");
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_LOAD_BACK:
            if (DI_IsHigh(WORK_REAR_CH) != 0U)
            {
                LOG("[装填] 回退完成\r\n");
                Work_StopMotion();                 /* 后到位：回退结束 */
            }
            else if ((now - s_start_tick) >= WORK_MOVE_TIMEOUT_MS)
            {
                LOG("[电机] 回退超时\r\n");
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
                if ((now - s_fire_tick) >= g_cfg.pulse_ms)
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
                if ((now - s_fire_tick) >= g_cfg.gap_ms)
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
            LOG("[EEPROM] 余弹已保存 %d\r\n", g_cfg.ammo);
        }
        else
        {
            LOG("[EEPROM] 余弹保存失败\r\n");
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
        /* 装填：按一下自动运行，再次按下立即停止 */
        if ((kA != 0U) && (s_prev[0] == 0U))
        {
            LOG("[按键] A 装填\r\n");
            if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_LOAD_BACK))
            {
                Work_StopMotion();           /* 装填/回退中再次按下：立即停止 */
            }
            else if (Work_CanStart() != 0U)
            {
                s_load_ammo0 = g_cfg.ammo;   /* 记录本次装填起点，用于余弹达标判断 */
                Work_StartMotion(WORK_ST_LOAD, WORK_DIR_LOAD);
            }
        }

        /* 射击：按射击模式发起；连发/无限制需持续按住 */
        if ((kC != 0U) && (s_prev[2] == 0U))
        {
            LOG("[按键] C 射击\r\n");
            if (Work_CanStart() != 0U)        /* 与装填互斥 */
            {
                switch (g_cfg.fire_mode)
                {
                    case FIRE_MODE_BURST:
                        Work_FireStart(g_cfg.burst_shots, 0U);
                        break;
                    case FIRE_MODE_LONG_BURST:
                        Work_FireStart(g_cfg.long_burst_shots, 0U);
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
