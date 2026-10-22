/* 三级菜单（每级 5 项），128x64 OLED 显示
 * 结构：主菜单(第 1 级) -> 功能页(第 2 级) -> 参数/动作页(第 3 级)
 * 按键：借用 IN_1~IN_4（高电平有效），通道号见 global.h 的 MENU_KEY_*_CH
 *       上 / 下 / 确认 / 返回
 */
#include "menu.h"
#include "oled.h"
#include "digital_input.h"
#include "global.h"
#include "motor.h"
#include "AnalogQuantityAcquisition.h"
#include "comm.h"
#include "eeprom.h"
#include "usart.h"
#include "relay.h"
#include "work.h"

#define INFO_LINES      5U
#define TEXT_LEN        21U

/* 列表页布局：每屏 3 项（16x16 汉字），逻辑上每级仍为 5 项，超出窗口自动滚动 */
#define LIST_ROWS       3U
#define LIST_ITEM_H     16U
#define LIST_TOP_Y      16U
#define LIST_TEXT_X     8U

/*------------------------- 菜单数据结构 -------------------------*/
typedef struct MenuPage MenuPage;
typedef void (*MenuAction_t)(uint8_t arg);

typedef struct
{
    const char     *text;    /* 显示文字（UTF-8：汉字 16x16，ASCII 6x8） */
    const MenuPage *sub;     /* 子页面，NULL 表示叶子 */
    MenuAction_t    action;  /* 叶子动作 */
    uint8_t         arg;     /* 动作参数 */
} MenuItem;

struct MenuPage
{
    const char       *title;  /* 页面标题（UTF-8 中文，16x16） */
    const MenuItem    items[MENU_ITEMS];
};

#define ITEM_SUB(t, p)     { (t), (p),  NULL,     0U }
#define ITEM_ACT(t, f, a)  { (t), NULL, (f),      (a) }

/*------------------------- 页面声明 -------------------------*/
static const MenuPage P_ROOT;
static const MenuPage P_DI;
static const MenuPage P_DI_A;
static const MenuPage P_DI_B;
static const MenuPage P_DI_ALL;
static const MenuPage P_SENSOR;
static const MenuPage P_MOTOR;
static const MenuPage P_MOTOR_FWD;
static const MenuPage P_MOTOR_REV;
static const MenuPage P_MOTOR_DUTY;
static const MenuPage P_MOTOR_WORK;
static const MenuPage P_ADC;
static const MenuPage P_ADC_LIVE;
static const MenuPage P_ADC_PEAK;
static const MenuPage P_ADC_MV;
static const MenuPage P_ADC_THR;
static const MenuPage P_SYS;
static const MenuPage P_SYS_SW;
static const MenuPage P_SYS_HW;
static const MenuPage P_SYS_TIME;
static const MenuPage P_SYS_RST;
static const MenuPage P_SYS_PIN;
static const MenuPage P_EEPROM;
static const MenuPage P_COMM;
static const MenuPage P_COMM_ECHO;
static const MenuPage P_COMM_CNT;
static const MenuPage P_COMM_BAUD;

/*------------------------- 状态 -------------------------*/
static const MenuPage *s_page[MENU_LEVELS];
static uint8_t s_sel[MENU_LEVELS];
static uint8_t s_level;
static uint8_t s_view;        /* 0 = 列表，1 = 信息页 */
static uint8_t s_dirty;
static uint8_t s_key_prev[4];
static uint32_t s_key_c_tick;
static uint32_t s_work_tick;      /* 工作/调试模式状态页刷新计时 */
static uint8_t s_draw_mode = 0xFFU;

static char s_info_title[TEXT_LEN];
static char s_info[INFO_LINES][TEXT_LEN];

/*------------------------- 字符串工具 -------------------------*/
static void Str_Set(char *dst, const char *src)
{
    uint8_t i = 0U;

    while ((src[i] != '\0') && (i < (TEXT_LEN - 1U)))
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void Str_Add(char *dst, const char *src)
{
    uint8_t i = 0U;

    while ((dst[i] != '\0') && (i < (TEXT_LEN - 1U)))
    {
        i++;
    }

    while ((*src != '\0') && (i < (TEXT_LEN - 1U)))
    {
        dst[i] = *src;
        i++;
        src++;
    }
    dst[i] = '\0';
}

static void Fmt_U32(char *dst, uint32_t v)
{
    char tmp[11];
    uint8_t n = 0U;

    if (v == 0U)
    {
        dst[0] = '0';
        dst[1] = '\0';
        return;
    }

    while ((v > 0U) && (n < 10U))
    {
        tmp[n] = (char)('0' + (v % 10U));
        v /= 10U;
        n++;
    }

    while (n > 0U)
    {
        n--;
        *dst = tmp[n];
        dst++;
    }
    *dst = '\0';
}

/* 一位小数，例如 12.3 / -0.5 */
static void Fmt_F1(char *dst, float v)
{
    int32_t x;
    char tmp[8];

    x = (int32_t)(v * 10.0f);
    if (x < 0)
    {
        *dst = '-';
        dst++;
        x = -x;
    }

    Fmt_U32(tmp, (uint32_t)(x / 10));
    Str_Set(dst, tmp);
    Str_Add(dst, ".");
    Fmt_U32(tmp, (uint32_t)(x % 10));
    Str_Add(dst, tmp);
}

static void Fmt_Hex4(char *dst, uint16_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t i;

    Str_Set(dst, "0x");
    for (i = 0U; i < 4U; i++)
    {
        uint8_t nib = (uint8_t)((v >> ((3U - i) * 4U)) & 0x0FU);
        char c[2];
        c[0] = hex[nib];
        c[1] = '\0';
        Str_Add(dst, c);
    }
}

static void Info_Clear(void)
{
    uint8_t i;

    s_info_title[0] = '\0';
    for (i = 0; i < INFO_LINES; i++)
    {
        s_info[i][0] = '\0';
    }
}

static void Info_Title(const char *t)
{
    Str_Set(s_info_title, t);
}

/* line: 0~4 */
static void Info_Text(uint8_t line, const char *text)
{
    if (line < INFO_LINES)
    {
        Str_Set(s_info[line], text);
    }
}

/* 生成 "LABEL   : xxx" 形式的行 */
static void Info_KV(uint8_t line, const char *label, const char *value)
{
    if (line >= INFO_LINES)
    {
        return;
    }

    Str_Set(s_info[line], label);
    Str_Add(s_info[line], value);
}

static void Info_KV_U32(uint8_t line, const char *label, uint32_t value)
{
    char buf[12];

    Fmt_U32(buf, value);
    Info_KV(line, label, buf);
}

static void Info_KV_F1(uint8_t line, const char *label, float value)
{
    char buf[16];

    Fmt_F1(buf, value);
    Info_KV(line, label, buf);
}

/*------------------------- 动作：数字输入 -------------------------*/
static void Act_DiDetail(uint8_t ch)
{
    char buf[4];

    Info_Clear();
    Str_Set(s_info_title, "INPUT ");
    Fmt_U32(buf, ch);
    Str_Add(s_info_title, buf);

    Info_KV(0U, "LEVEL : ", DI_IsHigh(ch) ? "HIGH (1)" : "LOW (0)");
    Info_KV_U32(1U, "RISE  : ", DI_GetRiseCount(ch));
    Info_KV_U32(2U, "ALL   : ", DI_GetState());
    Info_KV_U32(3U, "HIGH N: ", DI_GetHighCount());
}

static void Act_DiMap(uint8_t arg)
{
    uint8_t i;
    char bit[3];

    (void)arg;

    Info_Clear();
    Info_Title("STATE MAP");

    Info_Text(0U, "IN 1-5 :");
    s_info[1][0] = '\0';
    for (i = 1U; i <= 5U; i++)
    {
        bit[0] = DI_IsHigh(i) ? '1' : '0';
        bit[1] = '\0';
        Str_Add(s_info[1], bit);
        if (i < 5U) { Str_Add(s_info[1], " "); }
    }

    Info_Text(2U, "IN 6-10:");
    s_info[3][0] = '\0';
    for (i = 6U; i <= 10U; i++)
    {
        bit[0] = DI_IsHigh(i) ? '1' : '0';
        bit[1] = '\0';
        Str_Add(s_info[3], bit);
        if (i < 10U) { Str_Add(s_info[3], " "); }
    }
}

static void Act_DiHigh(uint8_t arg)
{
    uint8_t i;
    char buf[4];

    (void)arg;

    Info_Clear();
    Info_Title("HIGH LIST");
    Info_KV_U32(0U, "COUNT : ", DI_GetHighCount());

    s_info[1][0] = '\0';
    for (i = 1U; i <= 10U; i++)
    {
        if (DI_IsHigh(i) == 0U)
        {
            continue;
        }

        Fmt_U32(buf, i);
        Str_Add(s_info[1], buf);
        Str_Add(s_info[1], " ");
    }
    if (s_info[1][0] == '\0')
    {
        Str_Set(s_info[1], "NONE");
    }
}

static void Act_DiHex(uint8_t arg)
{
    char buf[8];

    (void)arg;

    Info_Clear();
    Info_Title("STATE WORD");

    Fmt_Hex4(buf, DI_GetState());
    Info_KV(0U, "HEX   : ", buf);
    Info_KV_U32(1U, "DEC   : ", DI_GetState());
    Info_KV_U32(2U, "RAW   : ", DI_GetRaw());
    Info_KV_U32(3U, "HIGH N: ", DI_GetHighCount());
}

/* arg = 起始通道(0 或 5) */
static void Act_DiCounts(uint8_t arg)
{
    uint8_t i;
    char label[10];

    Info_Clear();
    Str_Set(s_info_title, "RISE COUNT");

    for (i = 0U; i < INFO_LINES; i++)
    {
        uint8_t ch = (uint8_t)(arg + i + 1U);
        if (ch > DI_NUM)
        {
            break;
        }

        Str_Set(label, "IN ");
        {
            char num[4];
            Fmt_U32(num, ch);
            Str_Add(label, num);
        }
        Str_Add(label, "    : ");
        Info_KV_U32(i, label, DI_GetRiseCount(ch));
    }
}

static void Act_DiClear(uint8_t arg)
{
    (void)arg;

    DI_ClearCounters();

    Info_Clear();
    Info_Title("CLEAR COUNT");
    Info_Text(0U, "COUNT CLEARED");
}

/*------------------------- 动作：电机 -------------------------*/
static void Act_MotorFwd(uint8_t pct)
{
    char buf[12];

    Motor_SetDutyPercent(pct);
    Motor_SetState(MOTOR_STATE_FORWARD);

    Info_Clear();
    Info_Title("FORWARD");
    Str_Set(buf, "DUTY  : ");
    {
        char num[4];
        Fmt_U32(num, pct);
        Str_Add(buf, num);
        Str_Add(buf, " %");
    }
    Info_Text(0U, buf);
    Info_KV(1U, "STATE : ", (motor.state == MOTOR_STATE_FORWARD) ? "FORWARD" : "STOP");
}

static void Act_MotorRev(uint8_t pct)
{
    char buf[12];

    Motor_SetDutyPercent(pct);
    Motor_SetState(MOTOR_STATE_REVERSE);

    Info_Clear();
    Info_Title("REVERSE");
    Str_Set(buf, "DUTY  : ");
    {
        char num[4];
        Fmt_U32(num, pct);
        Str_Add(buf, num);
        Str_Add(buf, " %");
    }
    Info_Text(0U, buf);
    Info_KV(1U, "STATE : ", (motor.state == MOTOR_STATE_REVERSE) ? "REVERSE" : "STOP");
}

static void Act_MotorStop(uint8_t arg)
{
    (void)arg;

    Motor_SetState(MOTOR_STATE_STOP);

    Info_Clear();
    Info_Title("MOTOR STOP");
    Info_Text(0U, "OUTPUT OFF");
    Info_KV(1U, "STATE : ", "STOP");
}

static void Act_MotorDuty(uint8_t pct)
{
    Motor_SetDutyPercent(pct);

    Info_Clear();
    Info_Title("DUTY SET");
    Info_KV_U32(0U, "DUTY  : ", pct);
    Info_KV(1U, "STATE : ", (motor.state == MOTOR_STATE_STOP) ? "STOP" : "RUN");
}

static void Act_MotorStatus(uint8_t arg)
{
    (void)arg;

    Info_Clear();
    Info_Title("MOTOR STATE");
    Info_KV(0U, "STATE : ", (motor.state == MOTOR_STATE_FORWARD) ? "FORWARD" :
                             ((motor.state == MOTOR_STATE_REVERSE) ? "REVERSE" : "STOP"));
    Info_KV_U32(1U, "DUTY  : ", motor.duty_curve);
    Info_KV_U32(2U, "PERIOD: ", motor.arr);
    Info_KV_F1(3U, "I(U)  : ", AD_GetCurrent(AD_INDEX_U));
}

/*------------------------- 动作：电流 -------------------------*/
static void Act_AdcLive(uint8_t arg)
{
    (void)arg;

    Info_Clear();
    Info_Title("LIVE CURRENT");
    Info_KV_F1(0U, "I(U)  : ", AD_GetCurrent(AD_INDEX_U));
    Info_KV_F1(1U, "I(V)  : ", AD_GetCurrent(AD_INDEX_ALL));
    Info_KV_F1(2U, "I SUM : ", AD_GetCurrent(AD_INDEX_U) + AD_GetCurrent(AD_INDEX_ALL));
    Info_KV_F1(3U, "PEAK U: ", AD_GetPeak(AD_INDEX_U));
    Info_KV_F1(4U, "PEAK V: ", AD_GetPeak(AD_INDEX_ALL));
}

static void Act_AdcPeak(uint8_t ch)
{
    Info_Clear();
    Info_Title("PEAK CURRENT");
    Info_KV_F1(0U, "PEAK  : ", AD_GetPeak(ch));
    Info_KV_F1(1U, "NOW   : ", AD_GetCurrent(ch));
}

static void Act_AdcClearPeak(uint8_t arg)
{
    (void)arg;

    AD_ClearPeak(0xFFU);

    Info_Clear();
    Info_Title("CLEAR PEAK");
    Info_Text(0U, "PEAK CLEARED");
}

static void Act_AdcClearPeakCh(uint8_t ch)
{
    AD_ClearPeak(ch);

    Info_Clear();
    Info_Title("CLEAR PEAK");
    Info_Text(0U, "PEAK CLEARED");
}

static void Act_AdcVolt(uint8_t ch)
{
    Info_Clear();
    Info_Title("SAMPLE VOLT");
    Info_KV_F1(0U, "VOLT  : ", AD_GetVoltage(ch) / 1000.0f);
    Info_KV_U32(1U, "RAW   : ", ad.filt[ch]);
    Info_KV_F1(2U, "ZERO  : ", ad.zero_mv[ch] / 1000.0f);
}

static void Act_AdcZeroCal(uint8_t arg)
{
    (void)arg;

    AD_CalibrateZero();

    Info_Clear();
    Info_Title("ZERO CAL");
    Info_Text(0U, "ZERO UPDATED");
    Info_KV_F1(1U, "ZERO U: ", ad.zero_mv[AD_INDEX_U] / 1000.0f);
    Info_KV_F1(2U, "ZERO V: ", ad.zero_mv[AD_INDEX_ALL] / 1000.0f);
}

static void Act_AdcThr(uint8_t amp)
{
    g_cfg.over_current_a = amp;

    Info_Clear();
    Info_Title("OC THRESHOLD");
    Info_KV_U32(0U, "THRESH: ", amp);
    Info_Text(1U, "UNIT  : A");
}

/*------------------------- 动作：系统 -------------------------*/
static void Act_SysSw(uint8_t id)
{
    Info_Clear();
    Info_Title("SOFT VERSION");

    switch (id)
    {
        case 0U: Info_KV_U32(0U, "MAJOR : ", SW_VERSION_MAJOR); break;
        case 1U: Info_KV_U32(0U, "MINOR : ", SW_VERSION_MINOR); break;
        case 2U: Info_KV(0U, "DATE  : ", __DATE__); break;
        case 3U: Info_KV(0U, "TIME  : ", __TIME__); break;
        default: Info_KV_U32(0U, "BUILD : ", SW_VERSION_BUILD); break;
    }
}

static void Act_SysHw(uint8_t id)
{
    Info_Clear();
    Info_Title("HARD VERSION");

    switch (id)
    {
        case 0U: Info_KV(0U, "BOARD : ", BOARD_NAME); break;
        case 1U: Info_KV(0U, "HW VER: ", BOARD_HW_VERSION); break;
        case 2U: Info_KV(0U, "SCH   : ", BOARD_SCH_VERSION); break;
        case 3U: Info_KV(0U, "SCREEN: ", SCREEN_NAME); break;
        default: Info_KV(0U, "DRIVER: ", SCREEN_DRIVER_IC); break;
    }
}

static void Act_SysTime(uint8_t id)
{
    uint32_t s = g_cfg.run_seconds;

    Info_Clear();
    Info_Title("RUN TIME");

    switch (id)
    {
        case 0U: Info_KV_U32(0U, "SECOND: ", s); break;
        case 1U: Info_KV_U32(0U, "MINUTE: ", s / 60U); break;
        case 2U: Info_KV_U32(0U, "HOUR  : ", s / 3600U); break;
        default: Info_KV_U32(0U, "RESET : ", g_cfg.reset_count); break;
    }
}

static void Act_SysReset(uint8_t id)
{
    uint32_t csr = RCC->CSR;

    Info_Clear();
    Info_Title("RESET SOURCE");

    switch (id)
    {
        case 0U:
            Info_KV(0U, "PIN   : ", ((csr & RCC_CSR_PINRSTF) != 0U) ? "YES" : "NO");
            Info_KV(1U, "POR   : ", ((csr & RCC_CSR_PORRSTF) != 0U) ? "YES" : "NO");
            Info_KV(2U, "SOFT  : ", ((csr & RCC_CSR_SFTRSTF) != 0U) ? "YES" : "NO");
            Info_KV(3U, "IWDG  : ", ((csr & RCC_CSR_IWDGRSTF) != 0U) ? "YES" : "NO");
            Info_KV(4U, "WWDG  : ", ((csr & RCC_CSR_WWDGRSTF) != 0U) ? "YES" : "NO");
            break;

        case 3U:
            RCC->CSR |= RCC_CSR_RMVF;
            Info_Text(0U, "FLAGS CLEARED");
            break;

        default:
            Info_KV_U32(0U, "CSR   : ", csr);
            break;
    }
}

static void Act_SysPin(uint8_t id)
{
    Info_Clear();
    Info_Title("PIN MAP");

    switch (id)
    {
        case 0U:
            Info_Text(0U, "IN1 PA15 IN2 PB3");
            Info_Text(1U, "IN3 PB4  IN4 PB5");
            Info_Text(2U, "IN5 PB6  IN6 PB7");
            Info_Text(3U, "IN7 PB1  IN8 PA6");
            Info_Text(4U, "IN9 PA5  IN10 PA4");
            break;

        case 1U:
            Info_Text(0U, "UH PA8  VH PA9");
            Info_Text(1U, "UL PA7  VL PB0");
            Info_Text(2U, "TIM1 CH1/CH2");
            Info_Text(3U, "TIM3 CH2/CH3");
            break;

        case 2U:
            Info_Text(0U, "TX PA2  RX PA3");
            Info_Text(1U, "USART2 115200");
            Info_Text(2U, "TP3232 RS232");
            break;

        case 3U:
            Info_Text(0U, "SCK PB13 MOSI PB15");
            Info_Text(1U, "CS  PB12 DC   PA11");
            Info_Text(2U, "RES PA10 SPI2");
            break;

        default:
            Info_Text(0U, "IA PA0  IV PA1");
            Info_Text(1U, "ADC1 IN0/IN1");
            break;
    }
}

/*------------------------- 动作：通信 -------------------------*/
static void Act_CommEcho(uint8_t en)
{
    g_cfg.echo_enable = en;

    Info_Clear();
    Info_Title("ECHO TEST");
    Info_KV(0U, "ECHO  : ", (en != 0U) ? "ON" : "OFF");
}

static void Act_CommSendTest(uint8_t arg)
{
    static const char msg[] = "SCD2326 TEST\r\n";
    HAL_StatusTypeDef st;

    (void)arg;

    st = Comm_Send((const uint8_t *)msg, (uint16_t)(sizeof(msg) - 1U));

    Info_Clear();
    Info_Title("SEND TEST");
    Info_KV(0U, "RESULT: ", (st == HAL_OK) ? "OK" : "BUSY");
    Info_KV_U32(1U, "TX    : ", comm_tx_frames);
}

static void Act_CommCount(uint8_t arg)
{
    (void)arg;

    Info_Clear();
    Info_Title("COMM COUNT");
    Info_KV_U32(0U, "RX    : ", comm_rx_frames);
    Info_KV_U32(1U, "TX    : ", comm_tx_frames);
    Info_KV_U32(2U, "DROP  : ", comm_drop_frames);
    Info_KV_U32(3U, "BAUD  : ", g_cfg.baudrate);
}

static void Act_CommClear(uint8_t arg)
{
    (void)arg;

    Comm_ClearCounters();

    Info_Clear();
    Info_Title("CLEAR COUNT");
    Info_Text(0U, "COUNTERS CLEARED");
}

static void Act_CommBaud(uint8_t idx)
{
    static const uint32_t baud[5] = { 9600U, 19200U, 38400U, 57600U, 115200U };
    uint32_t value = (idx < 5U) ? baud[idx] : 115200U;

    Comm_SetBaud(value);

    Info_Clear();
    Info_Title("BAUDRATE");
    Info_KV_U32(0U, "SET   : ", value);
}

/* 输入状态串口打印开关 */
static void Act_CommStatusToggle(uint8_t arg)
{
    (void)arg;

    g_cfg.status_print_enable = (g_cfg.status_print_enable != 0U) ? 0U : 1U;

    Info_Clear();
    Info_Title("STATUS PRINT");
    Info_KV(0U, "STATE : ", (g_cfg.status_print_enable != 0U) ? "ON" : "OFF");
    Info_KV_U32(1U, "PERIOD: ", COMM_STATUS_PERIOD_MS);
    Info_Text(2U, "UNIT  : ms");
}

/*------------------------- 动作：传感器类型 -------------------------*/
static void Act_SensorSet(uint8_t type)
{
    Relay_SetSensorType(type);

    Info_Clear();
    Info_Title("SENSOR TYPE");
    Info_KV(0U, "TYPE  : ", (g_cfg.sensor_type == SENSOR_TYPE_PNP) ? "PNP" : "NPN");
    Info_KV(1U, "RELAY : ", (Relay_IsEnergized() != 0U) ? "ON" : "OFF");
    Info_KV(2U, "INPUT : ", "IN7-IN10");
    Info_Text(3U, "PRESS SAVE CFG");
    Info_Text(4U, "TO STORE");
}

static void Act_SensorShow(uint8_t arg)
{
    (void)arg;

    Info_Clear();
    Info_Title("SENSOR TYPE");
    Info_KV(0U, "TYPE  : ", (Relay_GetSensorType() == SENSOR_TYPE_PNP) ? "PNP" : "NPN");
    Info_KV(1U, "RELAY : ", (Relay_IsEnergized() != 0U) ? "ON" : "OFF");
    Info_KV(2U, "INPUT : ", "IN7-IN10");
    Info_KV(3U, "PB8/9 : ", (Relay_IsEnergized() != 0U) ? "HIGH" : "LOW");
}

/* 继电器吸合测试：只动作线圈，不改变已保存的传感器类型 */
static void Act_RelayTest(uint8_t arg)
{
    uint8_t on;

    (void)arg;

    on = (Relay_IsEnergized() != 0U) ? 0U : 1U;
    Relay_Force(on);

    Info_Clear();
    Info_Title("RELAY TEST");
    Info_KV(0U, "RELAY : ", (on != 0U) ? "ON" : "OFF");
    Info_Text(1U, "TYPE UNCHANGED");
    Info_Text(3U, "PRESS AGAIN");
    Info_Text(4U, "TO TOGGLE");
}

/*------------------------- 动作：工作设置 -------------------------*/
static const char *Work_StateName(uint8_t st)
{
    switch (st)
    {
        case WORK_ST_LOAD:    return "LOADING";
        case WORK_ST_BOLT:    return "BOLTING";
        case WORK_ST_RESET:   return "RESET";
        case WORK_ST_FIRE:    return "FIRING";
        case WORK_ST_TIMEOUT: return "TIMEOUT";
        case WORK_ST_OVER_CUR: return "OVER CUR";
        default:              return "IDLE";
    }
}

/* 射击允许 / 禁止切换（需保存配置才掉电保持） */
static void Act_ShotEnableToggle(uint8_t arg)
{
    (void)arg;

    g_cfg.shot_enable = (g_cfg.shot_enable != 0U) ? 0U : 1U;

    Info_Clear();
    Info_Title("SHOT ENABLE");
    Info_KV(0U, "SHOT  : ", (g_cfg.shot_enable != 0U) ? "ENABLE" : "DISABLE");
    Info_KV_U32(1U, "AMMO  : ", Work_GetAmmo());
    Info_Text(3U, "PRESS SAVE CFG");
    Info_Text(4U, "TO STORE");
}

/* 余弹装定：在当前余弹基础上增加 */
static void Act_AmmoAdd(uint8_t add)
{
    uint32_t ammo = (uint32_t)Work_GetAmmo() + (uint32_t)add;

    if (ammo > WORK_AMMO_MAX) { ammo = WORK_AMMO_MAX; }
    Work_SetAmmo((uint16_t)ammo);

    Info_Clear();
    Info_Title("AMMO SET");
    Info_KV_U32(0U, "ADD   : ", add);
    Info_KV_U32(1U, "AMMO  : ", Work_GetAmmo());
    Info_KV_U32(2U, "LOAD  : ", Work_GetLoadCount());
    Info_KV_U32(3U, "SHOT  : ", Work_GetShotCount());
}

/* 装填计数与射击次数清零 */
static void Act_WorkClear(uint8_t arg)
{
    (void)arg;

    Work_ClearCounts();

    Info_Clear();
    Info_Title("CLEAR COUNT");
    Info_KV_U32(0U, "LOAD  : ", Work_GetLoadCount());
    Info_KV_U32(1U, "SHOT  : ", Work_GetShotCount());
    Info_KV_U32(2U, "AMMO  : ", Work_GetAmmo());
}

/* 查看当前动作状态、到位信号与余弹 */
static void Act_WorkStatus(uint8_t arg)
{
    (void)arg;

    Info_Clear();
    Info_Title("WORK STATUS");
    Info_KV(0U, "MOTOR : ", Work_StateName(Work_GetState()));
    Info_KV_U32(1U, "AMMO  : ", Work_GetAmmo());
    Info_KV_U32(2U, "LOAD  : ", Work_GetLoadCount());
    Info_KV_U32(3U, "SHOT  : ", Work_GetShotCount());
    Info_KV(4U, "IN9/10: ", (DI_IsHigh(WORK_REAR_CH) != 0U) ? "1/0" : ((DI_IsHigh(WORK_FRONT_CH) != 0U) ? "0/1" : "0/0"));
}

/*------------------------- 动作：EEPROM -------------------------*/
static void Act_EepSave(uint8_t arg)
{
    HAL_StatusTypeDef st;

    (void)arg;

    st = EEPROM_SaveConfig();

    Info_Clear();
    Info_Title("EEPROM SAVE");
    Info_KV(0U, "RESULT: ", (st == HAL_OK) ? "OK" : "FAIL");
    Info_KV_U32(1U, "ADDR  : ", EEPROM_CFG_ADDR);
    Info_KV_U32(2U, "SIZE  : ", (uint32_t)sizeof(EEPROM_Config_t));
}

static void Act_EepLoad(uint8_t arg)
{
    HAL_StatusTypeDef st;

    (void)arg;

    st = EEPROM_LoadConfig();

    if ((st == HAL_OK) && (g_cfg.baudrate >= 9600U) && (g_cfg.baudrate != huart2.Init.BaudRate))
    {
        Comm_SetBaud(g_cfg.baudrate);
    }

    Info_Clear();
    Info_Title("EEPROM LOAD");
    Info_KV(0U, "RESULT: ", (st == HAL_OK) ? "OK" : "FAIL");
    Info_KV_U32(1U, "DUTY  : ", g_cfg.motor_duty_pct);
    Info_KV_U32(2U, "OC(A) : ", g_cfg.over_current_a);
    Info_KV_U32(3U, "BAUD  : ", g_cfg.baudrate);
}

static void Act_EepTest(uint8_t arg)
{
    HAL_StatusTypeDef st;

    (void)arg;

    st = EEPROM_SelfTest();

    Info_Clear();
    Info_Title("EEPROM TEST");
    Info_KV(0U, "RESULT: ", (st == HAL_OK) ? "OK" : "FAIL");
    Info_KV_U32(1U, "TEST  : ", EEPROM_TEST_ADDR);
}

static void Act_EepErase(uint8_t arg)
{
    HAL_StatusTypeDef st;

    (void)arg;

    st = EEPROM_EraseConfig();

    Info_Clear();
    Info_Title("ERASE CFG");
    Info_KV(0U, "RESULT: ", (st == HAL_OK) ? "OK" : "FAIL");
    Info_Text(1U, "USE DEFAULTS NEXT");
}

static void Act_EepInfo(uint8_t arg)
{
    HAL_StatusTypeDef st;

    (void)arg;

    st = EEPROM_IsReady();

    Info_Clear();
    Info_Title("EEPROM INFO");
    Info_KV(0U, "DEVICE: ", (st == HAL_OK) ? "BL24C128A" : "NOT FOUND");
    Info_KV_U32(1U, "I2C   : ", (uint32_t)(EEPROM_I2C_ADDR >> 1));
    Info_KV_U32(2U, "SIZE  : ", EEPROM_SIZE);
    Info_KV_U32(3U, "PAGE  : ", EEPROM_PAGE_SIZE);
}

/*------------------------- 页面定义 -------------------------*/
static const MenuPage P_ROOT =
{
    "主菜单",
    {
        ITEM_SUB("输入状态", &P_DI),
        ITEM_SUB("电机控制", &P_MOTOR),
        ITEM_SUB("电流监视", &P_ADC),
        ITEM_SUB("系统信息", &P_SYS),
        ITEM_SUB("通信测试", &P_COMM)
    }
};

/* 第 2 级：输入状态 */
static const MenuPage P_DI =
{
    "输入状态",
    {
        ITEM_SUB("输入 1-5",   &P_DI_A),
        ITEM_SUB("输入 6-10",  &P_DI_B),
        ITEM_SUB("全部状态",   &P_DI_ALL),
        ITEM_ACT("清除计数",   Act_DiClear, 0U),
        ITEM_SUB("传感器类型", &P_SENSOR)
    }
};

/* 第 3 级：输入 1-5 */
static const MenuPage P_DI_A =
{
    "输入 1-5",
    {
        ITEM_ACT("输入 1",   Act_DiDetail, 1U),
        ITEM_ACT("输入 2",   Act_DiDetail, 2U),
        ITEM_ACT("输入 3",   Act_DiDetail, 3U),
        ITEM_ACT("输入 4",   Act_DiDetail, 4U),
        ITEM_ACT("输入 5",   Act_DiDetail, 5U)
    }
};

/* 第 3 级：输入 6-10 */
static const MenuPage P_DI_B =
{
    "输入 6-10",
    {
        ITEM_ACT("输入 6",  Act_DiDetail, 6U),
        ITEM_ACT("输入 7",  Act_DiDetail, 7U),
        ITEM_ACT("输入 8",  Act_DiDetail, 8U),
        ITEM_ACT("输入 9",  Act_DiDetail, 9U),
        ITEM_ACT("输入 10", Act_DiDetail, 10U)
    }
};

/* 第 3 级：全部状态 */
static const MenuPage P_DI_ALL =
{
    "全部状态",
    {
        ITEM_ACT("状态图",      Act_DiMap,     0U),
        ITEM_ACT("高电平表",    Act_DiHigh,    0U),
        ITEM_ACT("十六进制",    Act_DiHex,     0U),
        ITEM_ACT("计数 1-5",    Act_DiCounts,  0U),
        ITEM_ACT("计数 6-10",   Act_DiCounts,  5U)
    }
};

/* 第 3 级：传感器类型（两路继电器切换 NPN / PNP） */
static const MenuPage P_SENSOR =
{
    "传感器类型",
    {
        ITEM_ACT("设为 NPN",   Act_SensorSet,  SENSOR_TYPE_NPN),
        ITEM_ACT("设为 PNP",   Act_SensorSet,  SENSOR_TYPE_PNP),
        ITEM_ACT("查看类型",   Act_SensorShow, 0U),
        ITEM_ACT("保存配置",   Act_EepSave,    0U),
        ITEM_ACT("继电器测试", Act_RelayTest,  0U)
    }
};

/* 第 2 级：电机控制 */
static const MenuPage P_MOTOR =
{
    "电机控制",
    {
        ITEM_SUB("正转设置",   &P_MOTOR_FWD),
        ITEM_SUB("反转设置",   &P_MOTOR_REV),
        ITEM_ACT("停止",       Act_MotorStop,   0U),
        ITEM_SUB("占空比设置", &P_MOTOR_DUTY),
        ITEM_SUB("工作设置",   &P_MOTOR_WORK)
    }
};

static const MenuPage P_MOTOR_FWD =
{
    "正转设置",
    {
        ITEM_ACT("正转 30%",   Act_MotorFwd, 30U),
        ITEM_ACT("正转 50%",   Act_MotorFwd, 50U),
        ITEM_ACT("正转 70%",   Act_MotorFwd, 70U),
        ITEM_ACT("正转 90%",   Act_MotorFwd, 90U),
        ITEM_ACT("查看状态",   Act_MotorStatus, 0U)
    }
};

static const MenuPage P_MOTOR_REV =
{
    "反转设置",
    {
        ITEM_ACT("反转 30%",   Act_MotorRev, 30U),
        ITEM_ACT("反转 50%",   Act_MotorRev, 50U),
        ITEM_ACT("反转 70%",   Act_MotorRev, 70U),
        ITEM_ACT("反转 90%",   Act_MotorRev, 90U),
        ITEM_ACT("查看状态",   Act_MotorStatus, 0U)
    }
};

static const MenuPage P_MOTOR_DUTY =
{
    "占空比设置",
    {
        ITEM_ACT("占空 20%",   Act_MotorDuty, 20U),
        ITEM_ACT("占空 40%",   Act_MotorDuty, 40U),
        ITEM_ACT("占空 60%",   Act_MotorDuty, 60U),
        ITEM_ACT("占空 80%",   Act_MotorDuty, 80U),
        ITEM_ACT("占空 90%",   Act_MotorDuty, 90U)
    }
};

/* 第 3 级：工作设置（射击允许、余弹装定、计数清零） */
static const MenuPage P_MOTOR_WORK =
{
    "工作设置",
    {
        ITEM_ACT("射击允许", Act_ShotEnableToggle, 0U),
        ITEM_ACT("余弹 +10", Act_AmmoAdd,          10U),
        ITEM_ACT("余弹 +50", Act_AmmoAdd,          50U),
        ITEM_ACT("清零计数", Act_WorkClear,        0U),
        ITEM_ACT("状态查看", Act_WorkStatus,       0U)
    }
};

/* 第 2 级：电流监视 */
static const MenuPage P_ADC =
{
    "电流监视",
    {
        ITEM_SUB("实时电流",   &P_ADC_LIVE),
        ITEM_SUB("峰值电流",   &P_ADC_PEAK),
        ITEM_SUB("采样电压",   &P_ADC_MV),
        ITEM_ACT("零点校准",   Act_AdcZeroCal,  0U),
        ITEM_SUB("过流阈值",   &P_ADC_THR)
    }
};

static const MenuPage P_ADC_LIVE =
{
    "实时电流",
    {
        ITEM_ACT("U 相电流",   Act_AdcLive, AD_INDEX_U),
        ITEM_ACT("V 相电流",   Act_AdcLive, AD_INDEX_ALL),
        ITEM_ACT("总电流",     Act_AdcLive, 0U),
        ITEM_ACT("U 相峰值",   Act_AdcPeak, AD_INDEX_U),
        ITEM_ACT("V 相峰值",   Act_AdcPeak, AD_INDEX_ALL)
    }
};

static const MenuPage P_ADC_PEAK =
{
    "峰值电流",
    {
        ITEM_ACT("U 相峰值",   Act_AdcPeak,        AD_INDEX_U),
        ITEM_ACT("V 相峰值",   Act_AdcPeak,        AD_INDEX_ALL),
        ITEM_ACT("清 U 峰值",  Act_AdcClearPeakCh, AD_INDEX_U),
        ITEM_ACT("清 V 峰值",  Act_AdcClearPeakCh, AD_INDEX_ALL),
        ITEM_ACT("全部清零",   Act_AdcClearPeak,  0U)
    }
};

static const MenuPage P_ADC_MV =
{
    "采样电压",
    {
        ITEM_ACT("U 相电压",   Act_AdcVolt, AD_INDEX_U),
        ITEM_ACT("V 相电压",   Act_AdcVolt, AD_INDEX_ALL),
        ITEM_ACT("U 零点",     Act_AdcVolt, AD_INDEX_U),
        ITEM_ACT("V 零点",     Act_AdcVolt, AD_INDEX_ALL),
        ITEM_ACT("刷新",       Act_AdcLive, 0U)
    }
};

static const MenuPage P_ADC_THR =
{
    "过流阈值",
    {
        ITEM_ACT("5 A",  Act_AdcThr, 5U),
        ITEM_ACT("10 A", Act_AdcThr, 10U),
        ITEM_ACT("15 A", Act_AdcThr, 15U),
        ITEM_ACT("20 A", Act_AdcThr, 20U),
        ITEM_ACT("30 A", Act_AdcThr, 30U)
    }
};

/* 第 2 级：系统信息 */
static const MenuPage P_SYS =
{
    "系统信息",
    {
        ITEM_SUB("软件版本", &P_SYS_SW),
        ITEM_SUB("硬件版本", &P_SYS_HW),
        ITEM_SUB("运行时间", &P_SYS_TIME),
        ITEM_SUB("复位原因", &P_SYS_RST),
        ITEM_SUB("存储器",   &P_EEPROM)
    }
};

static const MenuPage P_SYS_SW =
{
    "软件版本",
    {
        ITEM_ACT("主版本",   Act_SysSw, 0U),
        ITEM_ACT("次版本",   Act_SysSw, 1U),
        ITEM_ACT("编译日期", Act_SysSw, 2U),
        ITEM_ACT("编译时间", Act_SysSw, 3U),
        ITEM_ACT("编译号",   Act_SysSw, 4U)
    }
};

static const MenuPage P_SYS_HW =
{
    "硬件版本",
    {
        ITEM_ACT("板卡型号", Act_SysHw, 0U),
        ITEM_ACT("硬件版本", Act_SysHw, 1U),
        ITEM_ACT("原理图版", Act_SysHw, 2U),
        ITEM_ACT("屏幕型号", Act_SysHw, 3U),
        ITEM_ACT("引脚配置", Act_SysPin, 0U)
    }
};

static const MenuPage P_SYS_TIME =
{
    "运行时间",
    {
        ITEM_ACT("秒",       Act_SysTime, 0U),
        ITEM_ACT("分钟",     Act_SysTime, 1U),
        ITEM_ACT("小时",     Act_SysTime, 2U),
        ITEM_ACT("复位次数", Act_SysTime, 3U),
        ITEM_ACT("刷新",     Act_SysTime, 0U)
    }
};

static const MenuPage P_SYS_RST =
{
    "复位原因",
    {
        ITEM_ACT("全部标志",   Act_SysReset, 0U),
        ITEM_ACT("CSR 寄存器", Act_SysReset, 1U),
        ITEM_ACT("上电标志",   Act_SysReset, 2U),
        ITEM_ACT("清除标志",   Act_SysReset, 3U),
        ITEM_ACT("刷新",       Act_SysReset, 0U)
    }
};

static const MenuPage P_SYS_PIN =
{
    "引脚配置",
    {
        ITEM_ACT("输入引脚",  Act_SysPin, 0U),
        ITEM_ACT("PWM 引脚",  Act_SysPin, 1U),
        ITEM_ACT("串口引脚",  Act_SysPin, 2U),
        ITEM_ACT("OLED 引脚", Act_SysPin, 3U),
        ITEM_ACT("ADC 引脚",  Act_SysPin, 4U)
    }
};

/* 第 3 级：EEPROM（BL24C128A） */
static const MenuPage P_EEPROM =
{
    "存储器",
    {
        ITEM_ACT("保存配置", Act_EepSave,  0U),
        ITEM_ACT("加载配置", Act_EepLoad,  0U),
        ITEM_ACT("读写测试", Act_EepTest,  0U),
        ITEM_ACT("擦除配置", Act_EepErase, 0U),
        ITEM_ACT("存储信息", Act_EepInfo,  0U)
    }
};

/* 第 2 级：通信测试 */
static const MenuPage P_COMM =
{
    "通信测试",
    {
        ITEM_SUB("回传测试", &P_COMM_ECHO),
        ITEM_ACT("发送测试", Act_CommSendTest, 0U),
        ITEM_SUB("接收计数", &P_COMM_CNT),
        ITEM_SUB("波特率",   &P_COMM_BAUD),
        ITEM_ACT("状态打印", Act_CommStatusToggle, 0U)
    }
};

static const MenuPage P_COMM_ECHO =
{
    "回传测试",
    {
        ITEM_ACT("回传开",   Act_CommEcho, 1U),
        ITEM_ACT("回传关",   Act_CommEcho, 0U),
        ITEM_ACT("发送测试", Act_CommSendTest, 0U),
        ITEM_ACT("接收计数", Act_CommCount, 0U),
        ITEM_ACT("清除计数", Act_CommClear, 0U)
    }
};

static const MenuPage P_COMM_CNT =
{
    "接收计数",
    {
        ITEM_ACT("接收帧数", Act_CommCount, 0U),
        ITEM_ACT("发送帧数", Act_CommCount, 0U),
        ITEM_ACT("丢弃帧数", Act_CommCount, 0U),
        ITEM_ACT("清除计数", Act_CommClear, 0U),
        ITEM_ACT("刷新",     Act_CommCount, 0U)
    }
};

static const MenuPage P_COMM_BAUD =
{
    "波特率设置",
    {
        ITEM_ACT("9600",   Act_CommBaud, 0U),
        ITEM_ACT("19200",  Act_CommBaud, 1U),
        ITEM_ACT("38400",  Act_CommBaud, 2U),
        ITEM_ACT("57600",  Act_CommBaud, 3U),
        ITEM_ACT("115200", Act_CommBaud, 4U)
    }
};

/*------------------------- 绘制 -------------------------*/
/* 列表：标题 16x16 汉字，每屏 3 项，选中项超出窗口时自动滚动 */
static void Menu_DrawList(void)
{
    const MenuPage *page = s_page[s_level];
    uint8_t sel = s_sel[s_level];
    uint8_t first = 0U;
    uint8_t idx;
    uint8_t i;
    uint8_t y;

    OLED_Clear();
    OLED_ShowUTF8(0U, 0U, page->title, 0U);

    if (sel >= LIST_ROWS)
    {
        first = (uint8_t)(sel - (LIST_ROWS - 1U));
    }

    for (i = 0U; i < LIST_ROWS; i++)
    {
        idx = (uint8_t)(first + i);
        if (idx >= MENU_ITEMS)
        {
            break;
        }

        y = (uint8_t)(LIST_TOP_Y + (i * LIST_ITEM_H));
        if (idx == sel)
        {
            OLED_ShowChar(0U, (uint8_t)(y + 4U), '>', OLED_SIZE_8, 0U);
        }
        OLED_ShowUTF8(LIST_TEXT_X, y, page->items[idx].text, 0U);
    }

    /* 上下滚动提示 */
    if (first > 0U)
    {
        OLED_ShowChar(121U, 2U, '^', OLED_SIZE_8, 0U);
    }
    if ((uint8_t)(first + LIST_ROWS) < MENU_ITEMS)
    {
        OLED_ShowChar(121U, 55U, 'v', OLED_SIZE_8, 0U);
    }

    OLED_Refresh();
}

static void Menu_DrawInfo(void)
{
    uint8_t i;

    OLED_Clear();
    OLED_ShowString(0U, 0U, s_info_title, OLED_SIZE_8, 0U);
    OLED_ShowString(96U, 0U, "BACK", OLED_SIZE_8, 0U);
    OLED_DrawHLine(10U, 0U);

    for (i = 0U; i < INFO_LINES; i++)
    {
        OLED_ShowString(0U, (uint8_t)(13U + (i * 10U)), s_info[i], OLED_SIZE_8, 0U);
    }

    OLED_Refresh();
}

/* 工作/调试模式状态页：标题中文，数据行用 6x8 ASCII */
static void Menu_DrawWork(void)
{
    char buf[32];
    char num[11];
    const char *title;

    switch (g_cfg.work_mode)
    {
        case WORK_MODE_WORK:  title = "工作模式"; break;
        case WORK_MODE_DEBUG: title = "调试模式"; break;
        default:              title = "模式异常"; break;
    }

    OLED_Clear();
    OLED_ShowUTF8(0U, 0U, title, 0U);

    Str_Set(buf, "AMMO : ");
    Fmt_U32(num, Work_GetAmmo());
    Str_Add(buf, num);
    OLED_ShowString(0U, 18U, buf, OLED_SIZE_8, 0U);

    Str_Set(buf, "LOAD : ");
    Fmt_U32(num, Work_GetLoadCount());
    Str_Add(buf, num);
    Str_Add(buf, " SHOT: ");
    Fmt_U32(num, Work_GetShotCount());
    Str_Add(buf, num);
    OLED_ShowString(0U, 28U, buf, OLED_SIZE_8, 0U);

    Str_Set(buf, "IN9:");
    Str_Add(buf, (DI_IsHigh(WORK_REAR_CH) != 0U) ? "1" : "0");
    Str_Add(buf, " IN10:");
    Str_Add(buf, (DI_IsHigh(WORK_FRONT_CH) != 0U) ? "1" : "0");
    Str_Add(buf, " EN:");
    Str_Add(buf, (g_cfg.shot_enable != 0U) ? "1" : "0");
    OLED_ShowString(0U, 38U, buf, OLED_SIZE_8, 0U);

    Str_Set(buf, "MOTOR: ");
    Str_Add(buf, Work_StateName(Work_GetState()));
    OLED_ShowString(0U, 48U, buf, OLED_SIZE_8, 0U);

    OLED_Refresh();
}

/*------------------------- 按键处理 -------------------------*/
static void Menu_Enter(void)
{
    const MenuPage *page = s_page[s_level];
    const MenuItem *item = &page->items[s_sel[s_level]];

    if (item->sub != NULL)
    {
        if (s_level < (MENU_LEVELS - 1U))
        {
            s_level++;
            s_page[s_level] = item->sub;
            s_sel[s_level] = 0U;
            s_view = 0U;
            s_dirty = 1U;
        }
    }
    else if (item->action != NULL)
    {
        item->action(item->arg);
        s_view = 1U;
        s_dirty = 1U;
    }
}

static void Menu_Back(void)
{
    if (s_view != 0U)
    {
        s_view = 0U;
        s_dirty = 1U;
        return;
    }

    if (s_level > 0U)
    {
        s_level--;
        s_dirty = 1U;
    }
}

static void Menu_Up(void)
{
    if (s_view != 0U)
    {
        return;
    }

    s_sel[s_level] = (uint8_t)((s_sel[s_level] + MENU_ITEMS - 1U) % MENU_ITEMS);
    s_dirty = 1U;
}

static void Menu_Down(void)
{
    if (s_view != 0U)
    {
        return;
    }

    s_sel[s_level] = (uint8_t)((s_sel[s_level] + 1U) % MENU_ITEMS);
    s_dirty = 1U;
}

/* 按键检测：A/B 上升沿触发；C 短按=进入/退出，长按=返回上级 */
static void Menu_KeyScan(void)
{
    uint8_t kA = DI_IsHigh(MENU_KEY_A_CH);
    uint8_t kB = DI_IsHigh(MENU_KEY_B_CH);
    uint8_t kC = DI_IsHigh(MENU_KEY_C_CH);

    if ((kA != 0U) && (s_key_prev[0] == 0U))
    {
        Menu_Up();
    }

    if ((kB != 0U) && (s_key_prev[1] == 0U))
    {
        Menu_Down();
    }

    if ((kC != 0U) && (s_key_prev[2] == 0U))
    {
        s_key_c_tick = HAL_GetTick();          /* 记录按下时刻 */
    }
    else if ((kC == 0U) && (s_key_prev[2] != 0U))
    {
        if ((HAL_GetTick() - s_key_c_tick) >= MENU_LONG_PRESS_MS)
        {
            Menu_Back();                       /* 长按 = 退出到上级 */
        }
        else
        {
            Menu_Enter();                      /* 短按 = 进入/确认 */
        }
    }

    s_key_prev[0] = kA;
    s_key_prev[1] = kB;
    s_key_prev[2] = kC;
}

/*------------------------- 对外接口 -------------------------*/
void Menu_Init(void)
{
    uint8_t i;

    s_level = 0U;
    s_page[0] = &P_ROOT;
    for (i = 0U; i < MENU_LEVELS; i++)
    {
        s_sel[i] = 0U;
        if (i > 0U)
        {
            s_page[i] = &P_ROOT;
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        s_key_prev[i] = 0U;
    }

    s_view = 0U;
    s_dirty = 1U;
}

/* 非设置模式时只同步按键历史，避免切回设置模式时误触发 */
static void Menu_SyncKeys(void)
{
    s_key_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
    s_key_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
    s_key_prev[2] = DI_IsHigh(MENU_KEY_C_CH);

    if (s_key_prev[2] != 0U)
    {
        s_key_c_tick = HAL_GetTick();
    }
}

void Menu_Task(void)
{
    uint8_t mode = g_cfg.work_mode;

    /* 切换挡位：进入设置模式时强制重画；其它模式只同步按键状态 */
    if (mode != s_draw_mode)
    {
        s_draw_mode = mode;
        Menu_SyncKeys();
        if (mode == WORK_MODE_SET)
        {
            s_dirty = 1U;
        }
    }

    /* 工作/调试模式：显示状态页，按键由 Work_KeyTask 处理 */
    if (mode != WORK_MODE_SET)
    {
        if ((HAL_GetTick() - s_work_tick) >= 200U)
        {
            s_work_tick = HAL_GetTick();
            Menu_DrawWork();
        }
        return;
    }

    Menu_KeyScan();

    if (s_dirty == 0U)
    {
        return;
    }

    if (s_view != 0U)
    {
        Menu_DrawInfo();
    }
    else
    {
        Menu_DrawList();
    }

    s_dirty = 0U;
}
