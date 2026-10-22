/* 工作 / 调试模式动作控制
 *
 * 工作模式（拨动开关中挡）：
 *   A = 装填：电机向前运行到前到位(IN_10)，超时或堵转则停机报警
 *   C = 射击：MCU_OUT1 输出高电平 WORK_SHOT_PULSE_MS，每次计一发
 *   B = 无作用
 *   装填与射击互斥：动作执行期间忽略另一功能的按键
 *
 * 调试模式（拨动开关下挡）：
 *   A 长按 = 拉栓：电机向后运行到后到位(IN_9)，松开立即停止
 *   B 长按 = 复位：电机向前运行到前到位(IN_10)，松开立即停止
 *   C 长按 = 射击：MCU_OUT1 保持高电平，松开拉低；按下时计一发
 *   本模式不做功能互斥，请勿同时操作
 *
 * 余弹：IN_7 上升沿（装填过程中）+1 计入装填数，
 *       余弹 = 装填数 - 射击次数，最小为 0。
 */
#include "work.h"
#include "global.h"
#include "digital_input.h"
#include "motor.h"
#include "AnalogQuantityAcquisition.h"

static uint8_t  s_state;
static uint8_t  s_last_mode;
static uint8_t  s_fire_hold;      /* 1 = 调试模式按住射击，不自动关断 */
static uint32_t s_tick;           /* 1ms 节拍 */
static uint32_t s_start_tick;     /* 动作开始时刻 */
static uint32_t s_fire_tick;      /* 射击开始时刻 */
static uint16_t s_stall_ms;       /* 电流超阈值持续时间 */
static uint16_t s_last_cnt;       /* IN_7 上升沿计数的上次值 */
static uint8_t  s_prev[3];        /* A/B/C 上次电平 */

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

/*--------------------- 对外：初始化 / 中止 ---------------------*/
void Work_Init(void)
{
    Work_FireOff();
    Motor_EmergencyStop();

    s_state      = WORK_ST_IDLE;
    s_last_mode  = g_cfg.work_mode;
    s_fire_hold  = 0U;
    s_tick       = HAL_GetTick();
    s_start_tick = s_tick;
    s_fire_tick  = s_tick;
    s_stall_ms   = 0U;
    s_last_cnt   = DI_GetRiseCount(WORK_COUNT_CH);

    s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
    s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
    s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);
}

void Work_Abort(void)
{
    Work_FireOff();
    Motor_EmergencyStop();
    s_fire_hold = 0U;
    s_stall_ms  = 0U;
    s_state     = WORK_ST_IDLE;
}

/*--------------------- 动作启动 ---------------------*/
static void Work_StartMotion(uint8_t state, uint8_t dir)
{
    Motor_SetDutyPercent(g_cfg.motor_duty_pct);
    Motor_SetState(dir);

    s_state      = state;
    s_start_tick = HAL_GetTick();
    s_stall_ms   = 0U;
}

/* 工作模式：装填（自动运行到前到位） */
static void Work_StartLoad(void)
{
    if (s_state != WORK_ST_IDLE) { return; }   /* 射击或其它动作中，互斥 */
    Work_StartMotion(WORK_ST_LOAD, WORK_DIR_LOAD);
}

/* 工作模式：射击（固定脉宽，计一发） */
static void Work_StartShot(void)
{
    if (s_state != WORK_ST_IDLE) { return; }   /* 装填中，互斥 */
    if (g_cfg.shot_enable == 0U) { return; }   /* 射击禁止 */
    if (Work_GetAmmo() == 0U) { return; }      /* 无余弹 */

    Work_FireOn();
    s_fire_hold = 0U;
    s_fire_tick = HAL_GetTick();
    s_state     = WORK_ST_FIRE;

    if (g_cfg.shot_count < WORK_AMMO_MAX)
    {
        g_cfg.shot_count++;
    }
}

/*--------------------- 1ms 任务 ---------------------*/
void Work_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t cnt;
    float imag;
    float iu;

    if (now == s_tick) { return; }
    s_tick = now;

    /* 模式切换：立即停止所有输出，避免带载切挡 */
    if (g_cfg.work_mode != s_last_mode)
    {
        s_last_mode = g_cfg.work_mode;
        Work_Abort();
        return;
    }

    /* IN_7 上升沿：只在装填动作中计入装填数 */
    cnt = DI_GetRiseCount(WORK_COUNT_CH);
    if (cnt != s_last_cnt)
    {
        if (s_state == WORK_ST_LOAD)
        {
            g_cfg.load_total = (uint16_t)(g_cfg.load_total + (uint16_t)(cnt - s_last_cnt));
            if (g_cfg.load_total > WORK_AMMO_MAX)
            {
                g_cfg.load_total = WORK_AMMO_MAX;
            }
        }
        s_last_cnt = cnt;
    }

    /* 电机动作中：电流堵转判定（总电流与 U 相取较大值） */
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
                Work_StopMotion();          /* 前到位 */
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
                Work_StopMotion();          /* 后到位 */
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
                Work_StopMotion();          /* 前到位 */
            }
            else if ((now - s_start_tick) >= WORK_MOVE_TIMEOUT_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_FIRE:
            if ((s_fire_hold == 0U) && ((now - s_fire_tick) >= WORK_SHOT_PULSE_MS))
            {
                Work_FireOff();
                s_state = WORK_ST_IDLE;
            }
            break;

        default:
            break;
    }
}

/*--------------------- 按键任务 ---------------------*/
void Work_KeyTask(void)
{
    uint8_t kA = DI_IsHigh(MENU_KEY_A_CH);
    uint8_t kB = DI_IsHigh(MENU_KEY_B_CH);
    uint8_t kC = DI_IsHigh(MENU_KEY_C_CH);

    if (g_cfg.work_mode == WORK_MODE_WORK)
    {
        if ((kA != 0U) && (s_prev[0] == 0U)) { Work_StartLoad(); }
        if ((kC != 0U) && (s_prev[2] == 0U)) { Work_StartShot(); }
    }
    else if (g_cfg.work_mode == WORK_MODE_DEBUG)
    {
        /* 拉栓：A 按住运行，松开停止 */
        if ((kA != 0U) && (s_prev[0] == 0U))
        {
            Work_StartMotion(WORK_ST_BOLT, WORK_DIR_BOLT);
        }
        else if ((kA == 0U) && (s_prev[0] != 0U))
        {
            if (s_state == WORK_ST_BOLT)
            {
                Work_StopMotion();
            }
        }

        /* 复位：B 按住运行，松开停止（A 优先，避免同时按下冲突） */
        if ((kB != 0U) && (s_prev[1] == 0U) && (kA == 0U))
        {
            Work_StartMotion(WORK_ST_RESET, WORK_DIR_RESET);
        }
        else if ((kB == 0U) && (s_prev[1] != 0U))
        {
            if (s_state == WORK_ST_RESET)
            {
                Work_StopMotion();
            }
        }

        /* 射击：C 按住输出高电平，松开拉低；按下计一发 */
        if ((kC != 0U) && (s_prev[2] == 0U))
        {
            Work_FireOn();
            s_fire_hold = 1U;
            s_fire_tick = HAL_GetTick();
            s_state     = WORK_ST_FIRE;

            if (g_cfg.shot_count < WORK_AMMO_MAX)
            {
                g_cfg.shot_count++;
            }
        }
        else if ((kC == 0U) && (s_prev[2] != 0U))
        {
            Work_FireOff();
            s_fire_hold = 0U;
            if (s_state == WORK_ST_FIRE)
            {
                s_state = WORK_ST_IDLE;
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
    if (g_cfg.load_total > g_cfg.shot_count)
    {
        return (uint16_t)(g_cfg.load_total - g_cfg.shot_count);
    }
    return 0U;
}

uint16_t Work_GetLoadCount(void)
{
    return g_cfg.load_total;
}

uint16_t Work_GetShotCount(void)
{
    return g_cfg.shot_count;
}

/* 余弹装定：把装填数改为“已射击数 + rounds”，余弹即为 rounds */
void Work_SetAmmo(uint16_t rounds)
{
    uint32_t load = (uint32_t)g_cfg.shot_count + (uint32_t)rounds;

    g_cfg.load_total = (load > WORK_AMMO_MAX) ? WORK_AMMO_MAX : (uint16_t)load;
}

void Work_ClearCounts(void)
{
    g_cfg.load_total = 0U;
    g_cfg.shot_count = 0U;
    s_last_cnt = DI_GetRiseCount(WORK_COUNT_CH);
}
