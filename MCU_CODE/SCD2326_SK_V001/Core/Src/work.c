/* 工作 / 调试模式动作控制
 *
 * 输入通道（V0.3 实际接线）：
 *   IN_7  = 计数传感器（射击计数，每个上升沿余弹 -1；装填/动作期间忽略）
 *   IN_10 = 位置传感器·初始到位信号（装填回退的终点、调试“复位”的终点，
 *           OLED 调试页“初始到位信号”）
 *   IN_9  = 位置传感器·到位位置信号（装填正转的终点、调试“拉栓”的终点，
 *           OLED 调试页“到位位置信号”）
 *
 * 工作模式（拨动开关中挡）：
 *   A = 装填：按下后电机朝到位位置(IN_9)运行，收到 IN_9 后停止，
 *       随即反向回退，收到初始到位(IN_10)停止；两路位置信号同时有效时禁止电机动作；
 *       装填/回退过程中再次按下 A 立即停止。
 *       装填过程不做任何计数，余弹数只由射击计数传感器 IN_7 递减。
 *   C = 射击：按设置模式选定的射击模式，经 MCU_OUT1 输出有效电平
 *   B = 无作用
 *   装填与射击互斥：装填过程中不响应射击，射击过程中不响应装填
 *
 * 调试模式（拨动开关下挡）：
 *   A 长按(>=300ms) = 拉栓：与装填正转同向，运行到到位位置(IN_9)，松开立即停止
 *   B 长按(>=300ms) = 复位：与装填回退同向，运行到初始到位(IN_10)，松开立即停止
 *   C 长按(>=300ms) = 射击：MCU_OUT1 持续输出有效电平，松开立即停止；
 *                     余弹同样由 IN_7 计数传感器递减，各动作之间保持安全互斥
 *
 * 射击模式与输出方式（非单发模式均为“持续有效电平”，只有单发是单个脉冲）：
 *   无限制：按住期间 MCU_OUT1 持续为高，松开才拉低；不受余弹限制（余弹仍按实际发数递减）
 *   连射  ：按住期间持续为高，松开或余弹减到 0 时拉低
 *   长点射：按下后持续为高，IN_7 计满“长点射用弹数”或松开时拉低（先到先停）
 *   点射  ：按下后持续为高，IN_7 计满“点射用弹数”或松开时拉低（先到先停）
 *   单发  ：按一次输出一个 g_cfg.pulse_ms 宽的高电平脉冲
 *
 * 余弹：设置模式可直接装定当前余弹数 0~200；实际余弹数由 IN_7 射击计数递减，
 *       不写入 EEPROM，掉电后回到默认值。
 *
 * 输出：MCU_OUT1(PA12) 高电平 = 击发。
 *
 * 运行上下文（谁在什么时机调用）：
 *   Work_Task / Work_KeyTask        —— 主循环，每轮调用；Work_Task 内部用 s_tick 去重到 1ms；
 *   Work_CountTask / Work_LimitGuardIsr —— TIM2 1ms 更新中断，排在 DI_ScanIsr() 之后，
 *                                      因此拿到的是本拍已经消抖完成的稳定电平与最新边沿计数；
 *   Work_Init                       —— 上电初始化（DI_Init 之后、TIM2 启动之前）；
 *   Work_Abort                      —— 切挡时由 Work_Task 调用，立即停机并复位按键快照。
 *   中断与主循环共享的只有 s_state（读）和 g_cfg.ammo（读改写），
 *   均为单字节/半字访问，Cortex-M3 上原子，无需额外互斥。
 *
 * 关键时间参数（宏定义见 work.h）：
 *   WORK_HOLD_MS           300ms   调试模式长按启动门限
 *   WORK_LOAD_TIMEOUT_MS   15s     装填（正转）超时
 *   WORK_MOVE_TIMEOUT_MS   15s     回退 / 拉栓 / 复位超时
 *   WORK_STALL_MS          100ms   电流持续超阈值判为堵转
 *   WORK_CNT_DELAY_MS      200ms   击发信号开始后屏蔽 IN_7（躲开击发动作自身的抖动）
 *   WORK_CNT_HOLD_MS       600ms   无击发后关闭计数窗口
 *   单发脉宽：g_cfg.pulse_ms（设置模式可调，仅单发模式使用）。
 *   g_cfg.gap_ms（双发间隔）在“持续电平输出”方式下已不再参与射击时序。
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
static uint32_t s_fire_tick;      /* 单发脉冲起点（仅脉冲式单发使用） */
static uint16_t s_stall_ms;       /* 电流超阈值持续时间 */
static uint16_t s_last_cnt;       /* IN_7（射击计数）上升沿计数上次值 */
static uint8_t  s_prev[3];        /* A/B/C 上次电平 */
static uint32_t s_hold_tick[3];   /* A/B/C 按下时刻，用于长按判定 */
static uint8_t  s_hold_act[3];    /* A/B/C 长按已触发标记 */
static uint8_t  s_di_init;        /* 上次初始到位电平（日志用） */
static uint8_t  s_di_target;      /* 上次到位位置电平（日志用） */
static uint8_t  s_cnt_sess;       /* 1 = 工作模式计数窗口已打开（击发后 600ms 内有效） */
static uint32_t s_fire_last_tick; /* 最近一次击发信号有效（高电平）时刻：窗口保持计时基准 */
static uint32_t s_fire_rise_tick; /* 本次击发信号“上升沿”时刻（窗口打开那一拍记录）：
                                   * 持续电平输出没有下降沿，故以击发起点为基准，
                                   * 该时刻起 WORK_CNT_DELAY_MS(200ms) 内的 IN_7 忽略，
                                   * 用于躲开击发动作本身的机械抖动/余振 */
static uint16_t s_ammo_dbg;       /* 上次打印的余弹数（主循环侧变化打印用） */

static uint8_t  s_fire_shots;     /* 本次射击目标发数，WORK_FIRE_CONTINUOUS(0xFF) = 持续到松开 */
static uint8_t  s_fire_done;      /* 本次射击已发出数（由 IN_7 计数在 Work_CountTask 中递增） */
static uint8_t  s_fire_level;     /* 1 = MCU_OUT1 当前为高 */
static uint8_t  s_fire_unlimit;   /* 1 = 不检查、不扣减余弹 */
static uint8_t  s_fire_pulse;     /* 1 = 脉冲式输出（单发）；0 = 持续有效电平（其余模式） */

/*--------------------- 输出 ---------------------*/
/* 拉高击发输出（MCU_OUT1）
 * 参数：无
 * 返回：无
 * 调用：Work_FirePulse / Work_FireHold（开始输出）、Work_Task 持续式维持电平
 *       （主循环 Work_Task/Work_KeyTask 上下文）
 * 注意：只改 GPIO，不改状态机；状态切换由调用者负责
 */
static void Work_FireOn(void)
{
    HAL_GPIO_WritePin(MCU_OUT1_GPIO_Port, MCU_OUT1_Pin, GPIO_PIN_SET);
}

/* 拉低击发输出（MCU_OUT1）
 * 参数：无
 * 返回：无
 * 调用：Work_FireStop / Work_Abort 等，脉宽结束或需要立即停火时
 */
static void Work_FireOff(void)
{
    HAL_GPIO_WritePin(MCU_OUT1_GPIO_Port, MCU_OUT1_Pin, GPIO_PIN_RESET);
}

/* 立即停止电机动作，并把“动作类状态”退回空闲
 * 参数：无
 * 返回：无
 * 调用：Work_Task 各状态分支（到位 / 超时 / 过流）、Work_KeyTask（再次按下 / 松开）
 * 注意：调用 Motor_EmergencyStop() 是“立刻关四路门极”，不走斜坡；
 *       只有 LOAD / LOAD_BACK / BOLT / RESET 会退回 IDLE，
 *       WORK_ST_TIMEOUT / WORK_ST_OVER_CUR 由调用者继续赋值，射击态不受影响。
 */
static void Work_StopMotion(void)
{
    LOG("[MOT] stop st=%d\r\n", s_state);
    Motor_EmergencyStop();
    s_stall_ms = 0U;
    if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_LOAD_BACK) ||
        (s_state == WORK_ST_BOLT) || (s_state == WORK_ST_RESET))
    {
        s_state = WORK_ST_IDLE;
    }
}

/* 结束射击：关输出并回到空闲
 * 参数：无
 * 返回：无
 * 调用：Work_FireStart（余弹为 0 时）、Work_Task 射击结束条件成立、Work_KeyTask 松开按键
 * 注意：同时清掉目标发数、已发数、电平标记与脉冲标志；
 *       若当前不在 FIRE 态（例如装填中途调用），只关输出不动状态机，避免误改其它动作的状态
 */
static void Work_FireStop(void)
{
    if (s_state == WORK_ST_FIRE)
    {
        LOG("[FIRE] stop ammo=%d done=%d\r\n", g_cfg.ammo, s_fire_done);
    }
    Work_FireOff();
    s_fire_level = 0U;
    s_fire_shots = 0U;
    s_fire_done  = 0U;
    s_fire_pulse = 0U;
    if (s_state == WORK_ST_FIRE)
    {
        s_state = WORK_ST_IDLE;
    }
}

/* 是否允许再打一发
 * 参数：无
 * 返回：1 = 可以打（无限制模式恒为 1；否则要求 g_cfg.ammo > AMMO_MIN），0 = 余弹耗尽
 * 调用：Work_FireStart、Work_Task 的持续输出分支（余弹耗尽判停）
 * 注意：只读判断，不修改余弹；余弹实际扣减在 Work_CountTask（中断）里按 IN_7 完成
 */
static uint8_t Work_FireReady(void)
{
    if (s_fire_unlimit != 0U)
    {
        return 1U;
    }
    return (g_cfg.ammo > AMMO_MIN) ? 1U : 0U;
}

/* 前后到位信号同时有效：位置异常，禁止电机动作
 * 参数：无
 * 返回：1 = IN_9 与 IN_10 同时为高（异常），0 = 正常
 * 调用：Work_StartMotion（动作启动前）、Work_Task 的装填分支（动作中复查）
 * 注意：两路同时有效说明机构卡在中间或传感器故障，此时任何一个方向都可能继续顶死，
 *       所以宁可不动；本函数只做判断，由调用者决定停机
 */
static uint8_t Work_PosInvalid(void)
{
    if ((DI_IsHigh(WORK_INIT_CH) != 0U) && (DI_IsHigh(WORK_TARGET_CH) != 0U))
    {
        return 1U;
    }
    return 0U;
}

/* 电机是否正在动作（装填/装填回退/拉栓/复位）
 * 参数：无
 * 返回：1 = 处于上述四种动作状态之一，0 = 空闲/射击/故障态
 * 调用：Work_Task（堵转判定入口）、Work_CountTask（调试模式判断是否允许计数）
 * 注意：用于两处：
 *   1) 只有电机动作时才做堵转电流判定；
 *   2) 机构动作期间忽略 IN_7 计数脉冲，避免空行程被误当成射击而扣减余弹 */
static uint8_t Work_IsMoving(void)
{
    if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_LOAD_BACK) ||
        (s_state == WORK_ST_BOLT) || (s_state == WORK_ST_RESET))
    {
        return 1U;
    }
    return 0U;
}

/* 按设置模式的“电机方向”换算实际转动方向（工作/调试模式通用）
 * 参数：dir —— WORK_DIR_* 逻辑方向（MOTOR_STATE_FORWARD / MOTOR_STATE_REVERSE）
 * 返回：换算后的实际方向（MOTOR_STATE_FORWARD / MOTOR_STATE_REVERSE）
 * 调用：Work_StartMotion
 * 注意：装填、回退、拉栓、复位全部经过本函数，因此设置里的“电机方向”对四种动作同时生效；
 *       IN_9/IN_10 的物理含义不随该设置改变，只是电机转向相反
 */
static uint8_t Work_ApplyDir(uint8_t dir)
{
    if (g_cfg.load_dir == LOAD_DIR_REVERSE)
    {
        return (dir == MOTOR_STATE_FORWARD) ? MOTOR_STATE_REVERSE : MOTOR_STATE_FORWARD;
    }
    return dir;
}

/* 是否允许启动新动作：空闲，或上一次动作异常结束（超时/过流）
 * 参数：无
 * 返回：1 = 允许启动，0 = 正在动作或正在射击
 * 调用：Work_KeyTask（按键发起装填/拉栓/复位/调试射击）
 * 注意：把 TIMEOUT / OVER_CUR 也视为“可启动”，使故障后按一次键即可重新尝试；
 *       启动时 Work_StartMotion 会重写状态，故障码自然被清掉
 */
static uint8_t Work_CanStart(void)
{
    if ((s_state == WORK_ST_IDLE) || (s_state == WORK_ST_TIMEOUT) || (s_state == WORK_ST_OVER_CUR))
    {
        return 1U;
    }
    return 0U;
}

/* 单发模式：拉高击发输出并记录脉冲起点，由 Work_Task 计时到 g_cfg.pulse_ms 后拉低
 * 参数：无
 * 返回：无
 * 调用：Work_FireStart（s_fire_pulse != 0 时）
 * 注意：这是唯一保留“脉冲式”输出的模式（按一次只输出一个高电平脉冲）；
 *       余弹扣减仍由 IN_7 计数传感器在 Work_CountTask（TIM2 中断）中完成 */
static void Work_FirePulse(void)
{
    Work_FireOn();
    s_fire_level = 1U;
    s_fire_tick  = HAL_GetTick();
    s_state = WORK_ST_FIRE;
}

/* 持续输出模式：拉高击发输出并一直保持，直到满足结束条件（松开 / 计满发数 / 余弹耗尽）
 * 参数：无
 * 返回：无
 * 调用：Work_FireStart（s_fire_pulse == 0 时：点射/长点射/连射/无限制、调试射击）
 * 注意：只在这里拉高一次；后续是否拉低完全由 Work_Task 的 WORK_ST_FIRE 分支判定 */
static void Work_FireHold(void)
{
    Work_FireOn();
    s_fire_level = 1U;
    s_state = WORK_ST_FIRE;
}

/* 启动射击：shots = 目标发数，WORK_FIRE_CONTINUOUS 为持续到松开
 * 参数：shots   —— 本次要打出的发数（1~n，或 WORK_FIRE_CONTINUOUS=0xFF 表示持续）
 *       unlimit —— 1 = 无限制模式（不检查、不扣减余弹），0 = 受余弹限制
 *       pulse   —— 1 = 脉冲式输出（单发）；0 = 持续有效电平（其余模式）
 * 返回：无
 * 调用：Work_KeyTask（工作模式按 C、调试模式长按 C）
 * 注意：先记录参数再判断余弹，是为了让 Work_FireStop 能统一清理；
 *       余弹为 0 时直接调用 Work_FireStop 结束（不会留下半启动状态）
 */
static void Work_FireStart(uint8_t shots, uint8_t unlimit, uint8_t pulse)
{
    LOG("[FIRE] start mode=%d shots=%d unlim=%d pulse=%d ammo=%d\r\n",
        g_cfg.fire_mode, shots, unlimit, pulse, g_cfg.ammo);
    s_fire_shots   = shots;
    s_fire_done    = 0U;
    s_fire_unlimit = unlimit;
    s_fire_pulse   = pulse;

    if (Work_FireReady() == 0U)
    {
        Work_FireStop();
        return;
    }

    if (pulse != 0U)
    {
        Work_FirePulse();                /* 单发：输出一个脉冲 */
    }
    else
    {
        Work_FireHold();                 /* 其余模式：立即持续输出有效电平 */
    }
}

/*--------------------- 对外：初始化 / 中止 ---------------------*/
/* 上电初始化：停机、清状态、建立按键与计数基准
 * 参数：无
 * 返回：无
 * 调用：上电初始化阶段，main() 中在 DI_Init() 之后、启动 TIM2 之前调用
 * 注意：必须在 DI_Init() 之后，因为这里要读初始电平/边沿计数作为比较基准；
 *       必须在 TIM2 启动之前，否则中断可能在未初始化完成时进来操作状态
 */
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
    s_last_cnt   = DI_GetRiseCount(WORK_COUNT_CH);  /* 记下上电时的边沿计数，作为增量基线 */

    s_fire_shots   = 0U;
    s_fire_done    = 0U;
    s_fire_level   = 0U;
    s_fire_unlimit = 0U;
    s_fire_pulse   = 0U;

    /* 射击计数窗口：上电时关闭，只由击发信号打开 */
    s_cnt_sess       = 0U;
    s_fire_last_tick = s_tick;
    s_fire_rise_tick = s_tick;
    s_ammo_dbg       = g_cfg.ammo;

    s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
    s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
    s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);

    s_hold_tick[0] = s_tick;
    s_hold_tick[1] = s_tick;
    s_hold_tick[2] = s_tick;
    s_hold_act[0]  = 0U;
    s_hold_act[1]  = 0U;
    s_hold_act[2]  = 0U;

    s_di_init   = DI_IsHigh(WORK_INIT_CH);
    s_di_target = DI_IsHigh(WORK_TARGET_CH);
}

/* 立即中止：关闭击发与电机，并把按键快照同步到“当前按住的键”
 * 参数：无
 * 返回：无
 * 调用：Work_Task 检测到拨动开关挡位变化时调用（切挡不允许带载）
 * 注意：s_hold_act 被赋成当前电平，等价于“仍按住的键视为长按已触发”，
 *       必须松开再按下才能再次启动，防止切挡瞬间误动作；
 *       计数窗口一并关闭，切挡后必须由新的击发信号重新打开
 */
void Work_Abort(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  key[3];

    LOG("[ABORT]\r\n");
    Work_FireOff();
    Motor_EmergencyStop();

    s_fire_level = 0U;
    s_fire_shots = 0U;
    s_fire_done  = 0U;
    s_fire_pulse = 0U;
    s_stall_ms   = 0U;
    s_state      = WORK_ST_IDLE;

    /* 计数窗口复位：切挡后必须重新由击发信号打开 */
    s_cnt_sess       = 0U;
    s_fire_rise_tick = now;

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
/* 启动一段电机动作（装填/回退/拉栓/复位）
 * 参数：state —— 目标状态（WORK_ST_LOAD / LOAD_BACK / BOLT / RESET）
 *       dir   —— 逻辑方向（WORK_DIR_*），内部再经 Work_ApplyDir 换算
 * 返回：无
 * 调用：Work_Task（装填到位后转回退）、Work_KeyTask（按键发起动作）
 * 注意：进入前先确认位置信号正常，再关掉击发输出（动作与射击互斥）；
 *       装填/射击互斥的“另一半”由 Work_CanStart 在按键侧保证
 */
static void Work_StartMotion(uint8_t state, uint8_t dir)
{
    uint8_t real_dir;

    if (Work_PosInvalid() != 0U)
    {
        LOG("[MOT] refuse: front&rear both on\r\n");
        return;                      /* 前后到位同时有效：禁止电机动作 */
    }

    Work_FireStop();                 /* 动作前先断开击发输出，避免同时动作 */

    real_dir = Work_ApplyDir(dir);
    LOG("[MOT] start st=%d dir=%d\r\n", state, real_dir);
    Motor_SetDutyPercent(g_cfg.motor_duty_pct);
    Motor_SetState(real_dir);

    s_state      = state;
    s_start_tick = HAL_GetTick();
    s_stall_ms   = 0U;
}

/*--------------------- 射击计数（TIM2 1ms 中断内调用） ---------------------*/
/* 与 Work_Task 解耦，由 TIM2 更新中断每 1ms 调用一次：
 *   1) 维护“击发计数窗口”：击发信号(MCU_OUT1)有效即打开，600ms 无击发则关闭；
 *   2) 检查 IN_7 计数传感器的稳定上升沿，在窗口内把余弹数按实际发数递减。
 * 之所以放进中断：IN_7 的脉冲可能只有几毫秒，主循环一旦被 OLED 刷新、
 * EEPROM 读写或串口打印阻塞就会漏掉边沿；中断里读的是 DI_ScanIsr 已经
 * 消抖并累加好的边沿计数，不会丢脉冲。
 * 本函数运行在中断上下文，因此不直接调用 RTT 打印（本项目 RTT 未做互斥），
 * 余弹变化由 Work_Task 在主循环侧检测并打印。
 *
 * 工作模式计数规则：
 *   开始：击发信号有效（高电平）；
 *   条件：击发信号出现（上升沿）后 200ms 内的 IN_7 忽略，之后的上升沿才计数；
 *         因为非单发模式改为“持续有效电平”，击发期间没有下降沿，
 *         所以屏蔽基准取击发起点（窗口打开那一拍记录），而不是“击发结束”；
 *   停止：距最后一次击发 600ms 内再无击发信号 → 关闭窗口，IN_7 不再计数。
 * 调试模式：保持原逻辑 —— 非设置模式且电机静止（空行程脉冲忽略）时计数。
 * 设置模式：不计数，便于装定参数。
 * 余弹最低到 AMMO_MIN(0) 不会下溢；“无限制”射击模式同样按实际发数递减。
 * 每次计数同时累加 s_fire_done（本次已发出数），供点射/长点射“计满设定发数”判停。
 *
 * 参数：无
 * 返回：无
 * 调用：TIM2 1ms 更新中断（HAL_TIM_PeriodElapsedCallback），排在 DI_ScanIsr 之后
 * 注意：中断上下文 —— 只做整数比较与自减，不打印、不延时；
 *       s_last_cnt 与 DI_GetRiseCount 都是 16 位无符号，用相减得到增量可自然处理回绕
 */
void Work_CountTask(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t cnt;
    uint16_t delta;
    uint8_t  allow;

    /*==== 1) 击发信号 → 计数窗口 ====*/
    if (s_fire_level != 0U)
    {
        if (s_cnt_sess == 0U)
        {
            s_fire_rise_tick = now;        /* 窗口打开那一拍：记录击发起点（200ms 屏蔽基准） */
        }
        s_cnt_sess       = 1U;             /* 击发有效：打开/维持计数窗口 */
        s_fire_last_tick = now;
    }
    else if ((s_cnt_sess != 0U) && ((now - s_fire_last_tick) >= WORK_CNT_HOLD_MS))
    {
        s_cnt_sess = 0U;                   /* 600ms 无击发：关闭计数窗口 */
    }

    /*==== 2) IN_7 上升沿 → 余弹递减 ====*/
    cnt = DI_GetRiseCount(WORK_COUNT_CH);
    if (cnt == s_last_cnt)
    {
        return;                            /* 无边沿：直接返回，中断开销最小 */
    }

    delta      = (uint16_t)(cnt - s_last_cnt);   /* 无符号相减，计数器回绕仍正确 */
    s_last_cnt = cnt;

    if (g_cfg.work_mode == WORK_MODE_WORK)
    {
        allow = ((s_cnt_sess != 0U) &&
                 ((now - s_fire_rise_tick) >= WORK_CNT_DELAY_MS)) ? 1U : 0U;
    }
    else
    {
        /* 调试模式：非设置模式且电机未动作时才计数；设置模式不扣减 */
        allow = ((g_cfg.work_mode != WORK_MODE_SET) && (Work_IsMoving() == 0U)) ? 1U : 0U;
    }

    if (allow == 0U)
    {
        return;
    }

    while (delta > 0U)
    {
        if (g_cfg.ammo > AMMO_MIN)
        {
            g_cfg.ammo--;
        }
        if (s_fire_done < 0xFFU)
        {
            s_fire_done++;               /* 累计本次射击已发出数（点射/长点射据此判停） */
        }
        delta--;
    }
}

/*--------------------- 限位急停（TIM2 1ms 中断内调用） ---------------------*/
/* 到限位时立刻关断四路门极，不经过主循环。
 * 原因：Work_Task 在主循环里跑，可能正被 OLED 整屏刷新（SPI 1024 字节约 4ms）、
 *       EEPROM 写页（约 5ms）或 RTT 打印阻塞，若只在 Work_Task 里判到位，
 *       电机会多走 0~10ms，叠加消抖后停止点明显滞后。
 * 这里只做“停机”这一件事，不改状态机：
 *   正转（装填 / 拉栓，WORK_DIR_LOAD）    终点 = IN_9 到位位置信号；
 *   反转（回退 / 复位，WORK_DIR_LOAD_BACK）终点 = IN_10 初始到位信号。
 * 停机后 Work_Task 仍能在同一个位置信号上完成状态切换（装填转回退、动作结束等），
 * 逻辑与原来完全一致，只是断电更早。
 *
 * 参数：无
 * 返回：无
 * 调用：TIM2 1ms 更新中断，排在 DI_ScanIsr 之后（用的是本拍刚消抖完的稳定电平）
 * 注意：中断上下文；只调用 Motor_EmergencyStop()（内部已用 PRIMASK 保护四路 CCR），
 *       不修改 s_state，状态机推进仍由主循环负责，避免中断与主循环同时改状态
 */
void Work_LimitGuardIsr(void)
{
    if (motor.state == WORK_DIR_LOAD)
    {
        if (DI_IsHigh(WORK_TARGET_CH) != 0U)
        {
            Motor_EmergencyStop();
        }
    }
    else if (motor.state == WORK_DIR_LOAD_BACK)
    {
        if (DI_IsHigh(WORK_INIT_CH) != 0U)
        {
            Motor_EmergencyStop();
        }
    }
}

/*--------------------- 1ms 任务 ---------------------*/
/* 主循环侧动作状态机：切挡处理、堵转判定、到位/超时、射击脉宽与间隔
 * 参数：无
 * 返回：无
 * 调用：主循环每轮调用（main 的 while(1)），内部用 s_tick 去重到 1ms 执行一次
 * 注意：主循环可能被 OLED 刷新（SPI 整屏约 4ms）、EEPROM 写页（约 5ms）阻塞，
 *       因此“到限位立即断电”不依赖本函数，而由 Work_LimitGuardIsr 在中断里先做；
 *       本函数只负责状态机推进与保护（超时/过流）
 */
void Work_Task(void)
{
    uint32_t now = HAL_GetTick();
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
        LOG("[MODE] %d\r\n", g_cfg.work_mode);
        Work_Abort();
        return;
    }

    /* 到位信号变化打印 */
    {
        uint8_t i = DI_IsHigh(WORK_INIT_CH);
        uint8_t t = DI_IsHigh(WORK_TARGET_CH);

        if ((i != s_di_init) || (t != s_di_target))
        {
            s_di_init   = i;
            s_di_target = t;
            LOG("[DI] init=%d target=%d\r\n", i, t);
        }
    }

    /* 余弹变化打印：射击计数已在 TIM2 中断（Work_CountTask）内完成，
     * 中断内不做 RTT 输出，这里在主循环侧检测变化后打印一次 */
    if (g_cfg.ammo != s_ammo_dbg)
    {
        s_ammo_dbg = g_cfg.ammo;
        LOG("[AMMO] %d\r\n", g_cfg.ammo);
    }

    /* 电机动作中：总电流与 U 相取较大值做堵转判定
     * 总电流反映母线电流，U 相电流对单桥臂堵转更灵敏，取两者的较大值可避免漏判；
     * 必须连续 WORK_STALL_MS(100ms) 超阈值才停机，用来躲开启动瞬间的电流冲击 */
    if (Work_IsMoving() != 0U)
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
                LOG("[MOT] overcurrent %dmA\r\n", (int)(imag * 1000.0f));
                Work_StopMotion();       /* 先停机（会把动作态置 IDLE） */
                s_state = WORK_ST_OVER_CUR;  /* 再标记故障态，供 Work_CanStart 放行下一次启动 */
                return;
            }
        }
        else
        {
            s_stall_ms = 0U;             /* 电流回落立即清零，不累计非连续超限 */
        }
    }

    switch (s_state)
    {
        case WORK_ST_LOAD:
            if (Work_PosInvalid() != 0U)
            {
                Work_StopMotion();                 /* 前后到位同时有效：立即停止 */
            }
            else if (DI_IsHigh(WORK_TARGET_CH) != 0U)
            {
                /* 到达到位位置(IN_9)：先立即断电，再从静止反向回退。
                 * 注意：不能直接切换方向。换向需要先把占空比按曲线降到 0（约数百 ms），
                 * 这期间电机已被限位挡住仍持续通电，会被堵转过流判定直接停机，
                 * 表现为“到前到位就停、不反转”。此处立即关断可避免堵转电流。
                 * 装填过程不计数，余弹数只在射击时由 IN_7 递减。 */
                LOG("[LOAD] done(target) ammo=%d -> back\r\n", g_cfg.ammo);
                Work_StopMotion();               /* 先急停，清掉斜坡与堵转计时 */
                Work_StartMotion(WORK_ST_LOAD_BACK, WORK_DIR_LOAD_BACK);
            }
            else if ((now - s_start_tick) >= WORK_LOAD_TIMEOUT_MS)
            {
                LOG("[MOT] load timeout\r\n");
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_LOAD_BACK:
            /* 回退段：由 LOAD 到达 IN_9 后启动，终点是 IN_10；
             * 超时独立计时（进入本状态时 s_start_tick 已重置，回退同样有 15s） */
            if (DI_IsHigh(WORK_INIT_CH) != 0U)
            {
                LOG("[LOAD] back done\r\n");
                Work_StopMotion();                 /* 初始到位：回退结束 */
            }
            else if ((now - s_start_tick) >= WORK_LOAD_TIMEOUT_MS)
            {
                LOG("[MOT] back timeout\r\n");
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_BOLT:
            /* 调试拉栓：从静止出发运行到 IN_9；按键松开时由 Work_KeyTask 提前停机，
             * 这里是“到限位 / 超时”两条自动结束路径 */
            if (DI_IsHigh(WORK_TARGET_CH) != 0U)
            {
                Work_StopMotion();                 /* 拉栓结束：到达到位位置(IN_9) */
            }
            else if ((now - s_start_tick) >= WORK_MOVE_TIMEOUT_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_RESET:
            /* 调试复位：从静止出发运行到 IN_10；按键松开时由 Work_KeyTask 提前停机 */
            if (DI_IsHigh(WORK_INIT_CH) != 0U)
            {
                Work_StopMotion();                 /* 复位结束：到达初始到位(IN_10) */
            }
            else if ((now - s_start_tick) >= WORK_MOVE_TIMEOUT_MS)
            {
                Work_StopMotion();
                s_state = WORK_ST_TIMEOUT;
            }
            break;

        case WORK_ST_FIRE:
            /* 射击输出：
             *   脉冲式（单发，s_fire_pulse != 0）：高电平保持 g_cfg.pulse_ms 后结束；
             *   持续式（其余模式）：MCU_OUT1 一直保持高，满足以下任一条件才拉低——
             *     ① 余弹耗尽（连射/点射/长点射，Work_FireReady 为 0）；
             *     ② 已发出数达到设定发数（点射/长点射，s_fire_done >= s_fire_shots）；
             *     ③ 松开按键（由 Work_KeyTask 调用 Work_FireStop）。
             * 注意：持续式不再有“脉宽/间隔”交替，g_cfg.pulse_ms / gap_ms 均不参与。 */
            if (s_fire_pulse != 0U)
            {
                if ((now - s_fire_tick) >= g_cfg.pulse_ms)
                {
                    Work_FireStop();               /* 单发：脉冲结束即停 */
                }
            }
            else if (s_fire_level == 0U)
            {
                Work_FireOn();                     /* 持续式：维持输出为高 */
                s_fire_level = 1U;
            }
            else if (Work_FireReady() == 0U)
            {
                Work_FireStop();                   /* 余弹耗尽 */
            }
            else if ((s_fire_shots != WORK_FIRE_CONTINUOUS) &&
                     (s_fire_done >= s_fire_shots))
            {
                Work_FireStop();                   /* 计满设定发数（点射/长点射） */
            }
            else
            {
                /* 持续输出中：等待松开 / 计数达成 / 余弹耗尽 */
            }
            break;

        default:
            break;
    }

    /* 余弹数不再掉电保存：这里已删除原先“空闲 5s 后写 EEPROM”的逻辑，
     * 余弹只随 IN_7 射击计数递减，其余参数仍由设置模式保存。 */
}

/*--------------------- 按键任务（工作 / 调试模式） ---------------------*/
/* 按键任务：工作模式 A=装填 / C=射击；调试模式 A/B/C 长按 300ms 启动、松开停止
 * 参数：无
 * 返回：无
 * 调用：主循环每轮调用（不做 1ms 去重，靠 s_prev 边沿检测，主循环周期远快于人手动作）
 * 注意：s_prev[] 保存上一轮扫描电平，用于识别“刚按下 / 刚松开”；
 *       s_hold_tick[] 记录按下时刻，s_hold_act[] 标记长按已触发（一次按住只触发一次）；
 *       设置模式不会进入本函数（Work_Task/Work_KeyTask 只在工作/调试态有意义，
 *       设置模式的按键由 Menu_SetKeys 处理，两者不会同时响应）
 */
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
            LOG("[KEY] A work\r\n");
            if ((s_state == WORK_ST_LOAD) || (s_state == WORK_ST_LOAD_BACK))
            {
                Work_StopMotion();           /* 装填/回退中再次按下：立即停止 */
            }
            else if (Work_CanStart() != 0U)
            {
                /* 已取消“装填上限/装填目标”：不再按余弹数拦截，
                 * 无论当前余弹多少，按 A 都直接启动一轮装填。
                 * 装填过程只走位置信号，不做计数。 */
                Work_StartMotion(WORK_ST_LOAD, WORK_DIR_LOAD);
            }
        }

        /* 射击：按下发起。除单发为脉冲外，其余模式按下即持续输出有效电平，
         * 由 Work_Task 在“计满发数/余弹耗尽”时自动拉低，或松开按键拉低 */
        if ((kC != 0U) && (s_prev[2] == 0U))
        {
            LOG("[KEY] C work\r\n");
            if (Work_CanStart() != 0U)        /* 与装填互斥 */
            {
                switch (g_cfg.fire_mode)
                {
                    case FIRE_MODE_BURST:      /* 点射：持续输出，计满发数或松开停止 */
                        Work_FireStart(g_cfg.burst_shots, 0U, 0U);
                        break;
                    case FIRE_MODE_LONG_BURST: /* 长点射：持续输出，计满发数或松开停止 */
                        Work_FireStart(g_cfg.long_burst_shots, 0U, 0U);
                        break;
                    case FIRE_MODE_AUTO:       /* 连射：持续输出，松开或余弹耗尽停止 */
                        Work_FireStart(WORK_FIRE_CONTINUOUS, 0U, 0U);
                        break;
                    case FIRE_MODE_UNLIMITED:  /* 无限制：持续输出，仅松开停止 */
                        Work_FireStart(WORK_FIRE_CONTINUOUS, 1U, 0U);
                        break;
                    default:                   /* 单发：按一次输出一个脉冲 */
                        Work_FireStart(1U, 0U, 1U);
                        break;
                }
            }
        }
        else if ((kC == 0U) && (s_prev[2] != 0U))
        {
            /* 松开：单发脉冲自行打完，不受影响；其余模式（持续输出）立即拉低 */
            if (g_cfg.fire_mode != FIRE_MODE_SINGLE)
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
                s_hold_tick[0] = now;      /* 刚按下：重新开始计长按时间 */
                s_hold_act[0]  = 0U;
            }
            if ((s_hold_act[0] == 0U) && ((now - s_hold_tick[0]) >= WORK_HOLD_MS))
            {
                s_hold_act[0] = 1U;        /* 只在本轮按住期间触发一次 */
                if (Work_CanStart() != 0U)
                {
                    Work_StartMotion(WORK_ST_BOLT, WORK_DIR_BOLT);
                }
            }
        }
        else
        {
            s_hold_act[0] = 0U;
            /* 松开：只有确实处于拉栓态才停机，防止松手时误停其他动作 */
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

        /* 射击：长按 300ms 启动，MCU_OUT1 持续输出有效电平并扣减余弹，松开立即停止 */
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
                    Work_FireStart(WORK_FIRE_CONTINUOUS, 0U, 0U);
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
/* 读取当前动作状态
 * 参数：无
 * 返回：WORK_ST_* （0 空闲 / 1 装填 / 2 拉栓 / 3 复位 / 4 射击 / 5 超时 / 6 过流 / 7 回退）
 * 调用：Menu_Task 刷新工作页状态文本、RTT 镜像打印
 * 注意：只返回单字节状态，读到的必然是某一次完整赋值，无需关中断
 */
uint8_t Work_GetState(void)
{
    return s_state;
}

/* 是否正在射击
 * 参数：无
 * 返回：1 = 处于射击态，0 = 其它
 * 调用：外部状态展示/联锁查询
 * 注意：以状态机为准，不直接看 MCU_OUT1 电平
 */
uint8_t Work_IsFiring(void)
{
    return (s_state == WORK_ST_FIRE) ? 1U : 0U;
}

/* 读取当前余弹数
 * 参数：无
 * 返回：0~200（g_cfg.ammo 的当前值）
 * 调用：Menu_Task 显示工作/调试页、RTT 镜像打印
 * 注意：该值每 1ms 可能在 TIM2 中断（Work_CountTask）里被递减，
 *       16 位读取在 Cortex-M3 上是原子的，这里直接读即可
 */
uint16_t Work_GetAmmo(void)
{
    return g_cfg.ammo;
}
