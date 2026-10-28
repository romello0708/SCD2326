/* 三挡界面：设置模式 / 工作模式 / 调试模式
 *
 * 设置模式（拨动开关上挡）：
 *   A = 向上切换设置项 / 增大数值     B = 向下切换设置项 / 减小数值
 *   C = 进入 / 退出调节；退出时自动保存到 EEPROM
 *   标题行固定，下面三行循环显示设置项：
 *   余弹数（0~200）、射击模式、模块型号、计数传感器类型、位置传感器类型、电机方向、
 *   单发脉冲时长、双发间隔时长、点射用弹数、长点射用弹数
 *
 * 工作模式（中挡）：显示余弹数、射击模式、工作状态（未装填/装填中/射击中）
 * 调试模式（下挡）：显示余弹数、初始到位信号(IN_10)、到位位置信号(IN_9)
 *
 * 工作 / 调试模式下 A、B、C 的功能由 work.c 处理，本文件只负责显示。
 *
 * 运行上下文：Menu_Task 由主循环调用（非阻塞），内部按以下节拍工作：
 *   按键处理     —— 每轮扫描（设置模式），边沿触发；A/B 长按 500ms 后每 100ms 连加
 *   设置页重绘   —— 仅在 s_dirty 置位时（有按键动作/切挡）整屏重绘
 *   运行页刷新   —— 每 RUN_REFRESH_MS(200ms) 整屏重绘一次
 *   RTT 镜像打印 —— 每 UI_LOG_PERIOD_MS(500ms) 输出一帧当前页面内容
 *   EEPROM 保存  —— 参数改动后 CFG_SAVE_DELAY_MS(300ms) 无新改动才写入
 *
 * 显示布局：SSD1309 128x64，13x13 汉字 + 8x15 ASCII，4 行（标题 y=5，正文 y=20/35/50）。
 * 设置页共有 SET_ITEM_NUM(10) 个设置项，屏幕只显示 3 行，用 s_top 做滚动窗口。
 */
#include "menu.h"
#include "oled.h"
#include "digital_input.h"
#include "global.h"
#include "work.h"
#include "eeprom.h"
#include "relay.h"
#include "log.h"

/* 页面行坐标：整体下移 5px，13x13 字模，行距 15px，共 4 行 */
#define SET_TITLE_Y      5U
#define SET_ROW1_Y       20U
#define SET_ROW2_Y       35U
#define SET_ROW3_Y       50U
#define SET_ROW_STEP     15U
#define SET_ROW_NUM      3U      /* 标题下方可见行数，其余行滚动显示 */
#define UI_RIGHT_X       120U    /* 数值右对齐边界，右侧留 8px 余量（与示意图一致） */
#define AMMO_NUM_UP_PX   3U      /* 余弹数值相对文字行上移的像素数 */

/* 设置项 */
#define SET_ITEM_AMMO    0U
#define SET_ITEM_FIRE    1U
#define SET_ITEM_MODULE  2U
#define SET_ITEM_SENSOR1 3U
#define SET_ITEM_SENSOR2 4U
#define SET_ITEM_DIR     5U     /* 装填电机方向 */
#define SET_ITEM_PULSE   6U     /* 单发脉冲时长 */
#define SET_ITEM_GAP     7U     /* 双发间隔时长 */
#define SET_ITEM_BURST   8U     /* 点射用弹数 */
#define SET_ITEM_LBURST  9U     /* 长点射用弹数 */
#define SET_ITEM_NUM     10U

/* 运行页刷新周期 */
#define RUN_REFRESH_MS   200U

/* RTT 镜像打印周期：把 OLED 当前页内容每 500ms 打印一次 */
#define UI_LOG_PERIOD_MS 500U

/* 参数改动后延时写入 EEPROM（避开连续按键），保证直接断电也不丢参数 */
#define CFG_SAVE_DELAY_MS 300U

static const char * const s_fire_name[FIRE_MODE_NUM] =
{
    "无限制", "连射", "长点射", "点射", "单发"
};

static const char * const s_module_name[MODULE_NUM] =
{
    "火力模块", "侦打车", "C车", "B车", "A车", "8*8"
};

static const char * const s_sensor_name[SENSOR_TYPE_NUM] =
{
    "NPN", "PNP"
};

static const char * const s_dir_name[LOAD_DIR_NUM] =
{
    "正向", "反向"
};

/* RTT 镜像打印用的 ASCII 名称（RTT 无法显示中文，故另备一份英文/代号） */
static const char * const s_fire_en[FIRE_MODE_NUM] =
{
    "UNLIMITED", "AUTO", "LONG_BURST", "BURST", "SINGLE"
};

static const char * const s_module_en[MODULE_NUM] =
{
    "FIRE", "SCOUT", "CAR_C", "CAR_B", "CAR_A", "8X8"
};

static const char * const s_dir_en[LOAD_DIR_NUM] =
{
    "FWD", "REV"
};

static uint8_t  s_mode = 0xFFU;   /* 上次绘制的挡位（0xFF 保证首帧必定重绘） */
static uint8_t  s_sel;            /* 设置项选中位置：0 ~ SET_ITEM_NUM-1 */
static uint8_t  s_top;            /* 设置页滚动窗口首项：屏幕上第一行显示第几个设置项 */
static uint8_t  s_edit;           /* 1 = 正在调节数值（调节态），0 = 列表态 */
static uint8_t  s_dirty;          /* 1 = 需要重绘（按键/切挡后置位，绘完清零） */
static uint8_t  s_cfg_dirty;      /* 1 = 配置已改，需写 EEPROM */
static uint32_t s_save_tick;      /* 最近一次参数改动时刻（延时保存用，300ms 无改动才写） */
static uint8_t  s_prev[3];        /* A/B/C 上次电平，用于边沿检测（索引 0/1/2 对应 A/B/C） */
static uint32_t s_press_tick;     /* A/B 按下时刻（连加判定：按住超过 500ms 开始连加） */
static uint32_t s_rep_tick;       /* 连加上次触发时刻（连加间隔 100ms） */
static uint32_t s_refresh_tick;   /* 运行页刷新计时 */
static uint32_t s_log_tick;       /* RTT 镜像打印计时 */

/*------------------------- 字符串工具 -------------------------*/
/* UTF-8 字符串像素宽度：ASCII 8px，汉字 15px（与 OLED_ShowUTF8 一致）
 * 参数：str —— UTF-8 字符串，需以 '\0' 结尾
 * 返回：像素宽度（uint8_t）；字符串很长时可能溢出，本项目都是短标签，无需担心
 * 调用：ShowTextRight / Menu_DrawRun 计算右对齐起点
 * 注意：非 ASCII 字符一律按 3 字节汉字处理并计 15px，与字库实现保持一致
 */
static uint8_t Utf8Width(const char *str)
{
    uint8_t w = 0U;

    while (*str != '\0')
    {
        if ((uint8_t)*str < 0x80U)
        {
            w = (uint8_t)(w + OLED_ASC_WIDTH);
            str++;
        }
        else
        {
            w = (uint8_t)(w + OLED_CJK_WIDTH);
            str += 3;
        }
    }

    return w;
}

/* 数字宽度（8x15，按位数计算）
 * 参数：v —— 待显示的无符号数值 0~65535
 * 返回：像素宽度 = 位数 × 8px
 * 调用：ShowNumRight 计算右对齐起点
 * 注意：只支持无符号整数；0 按 1 位计算
 */
static uint8_t NumWidth(uint16_t v)
{
    uint8_t len = 1U;

    while (v >= 10U)
    {
        v /= 10U;
        len++;
    }

    return (uint8_t)(len * OLED_ASC_WIDTH);
}

/* 右对齐显示数字（余弹数值上移 3px）
 * 参数：y    —— 所在文字行的起始 y 像素
 *       v    —— 数值（0~65535）
 *       mode —— 显示方式：0 = 正常，1 = 反显（调节态高亮）；直接传给 OLED_ShowNum
 * 返回：无
 * 调用：Menu_DrawSetItem / Menu_DrawRun 绘制数值项
 * 注意：数值字模比汉字矮，统一上移 AMMO_NUM_UP_PX(3px) 才能与同行中文基线对齐
 */
static void ShowNumRight(uint8_t y, uint16_t v, uint8_t mode)
{
    uint8_t w = NumWidth(v);
    uint8_t ny = (y >= AMMO_NUM_UP_PX) ? (uint8_t)(y - AMMO_NUM_UP_PX) : 0U;

    OLED_ShowNum((uint8_t)(UI_RIGHT_X - w), ny, (uint32_t)v, (uint8_t)(w / OLED_ASC_WIDTH), mode);
}

/* 右对齐显示文本
 * 参数：y    —— 文字行起始 y 像素
 *       str  —— UTF-8 文本（中文/ASCII 均可）
 *       mode —— 0 = 正常，1 = 反显（调节态/选中态高亮）
 * 返回：无
 * 调用：Menu_DrawSetItem / Menu_DrawRun 绘制枚举值文本
 * 注意：宽度超过 UI_RIGHT_X 时截断到 120px，避免起点算出负数回绕成很大的 x
 */
static void ShowTextRight(uint8_t y, const char *str, uint8_t mode)
{
    uint8_t w = Utf8Width(str);

    if (w > UI_RIGHT_X)
    {
        w = UI_RIGHT_X;
    }

    OLED_ShowUTF8((uint8_t)(UI_RIGHT_X - w), y, str, mode);
}

/*------------------------- 设置模式 -------------------------*/
/* 列表态：切换设置项，并保证选中项落在可见窗口内
 * 参数：dir —— +1 下一项 / -1 上一项（由 Menu_Adjust 传入，已做过方向取反）
 * 返回：无
 * 调用：Menu_Adjust 在 s_edit == 0（列表态）时调用
 * 注意：设置项首尾循环；窗口 s_top 只在选中项越界时移动一行，
 *       这样连续按键时列表是“逐行滚动”而不是整屏跳动
 */
static void Menu_SelectMove(int16_t dir)
{
    int16_t idx = (int16_t)s_sel + dir;

    if (idx < 0) { idx = (int16_t)SET_ITEM_NUM - 1; }
    if (idx >= (int16_t)SET_ITEM_NUM) { idx = 0; }
    s_sel = (uint8_t)idx;

    if (s_sel < s_top)
    {
        s_top = s_sel;
    }
    else if (s_sel >= (uint8_t)(s_top + SET_ROW_NUM))
    {
        s_top = (uint8_t)(s_sel - SET_ROW_NUM + 1U);
    }
}

/* 当前设置项的数值（日志用）
 * 参数：item —— 设置项索引 0~SET_ITEM_NUM-1
 * 返回：该项当前的数值（枚举项返回索引，时长项返回 ms，余弹返回发数）
 * 调用：Menu_Adjust 修改后打印 [SET] 日志
 * 注意：只用于日志，不参与任何判断；default 分支按“长点射用弹数”返回
 */
static int16_t Menu_ItemValue(uint8_t item)
{
    switch (item)
    {
        case SET_ITEM_AMMO:    return (int16_t)g_cfg.ammo;
        case SET_ITEM_FIRE:    return (int16_t)g_cfg.fire_mode;
        case SET_ITEM_MODULE:  return (int16_t)g_cfg.module_type;
        case SET_ITEM_SENSOR1: return (int16_t)g_cfg.sensor1_type;
        case SET_ITEM_SENSOR2: return (int16_t)g_cfg.sensor2_type;
        case SET_ITEM_DIR:     return (int16_t)g_cfg.load_dir;
        case SET_ITEM_PULSE:   return (int16_t)g_cfg.pulse_ms;
        case SET_ITEM_GAP:     return (int16_t)g_cfg.gap_ms;
        case SET_ITEM_BURST:   return (int16_t)g_cfg.burst_shots;
        default:               return (int16_t)g_cfg.long_burst_shots;
    }
}

/* 按键调整入口：列表态切换设置项，调节态修改数值
 * 参数：dir —— +1（A 键，增大/向上）/ -1（B 键，减小/向下）；长按连加时为 ±10
 * 返回：无
 * 调用：Menu_SetKeys（按下边沿与长按重复两个时机）
 * 注意：1) 列表态方向取反 —— A 在屏幕上对应“向上”，而设置项索引是向下递增的；
 *       2) 所有数值项都做首尾循环，方便快速从 200 回到 0；
 *       3) 传感器类型改完立刻调用 Relay_SetType 并立即落盘，避免停在设置页时掉电丢失；
 *       4) 其余项只置 s_cfg_dirty，由 Menu_Task 在 300ms 无改动后统一写 EEPROM，
 *          避免连续按键时频繁擦写
 */
static void Menu_Adjust(int16_t dir)
{
    int16_t idx;

    if (s_edit == 0U)
    {
        /* 列表态：A(↑) 向上、B(↓) 向下，与调值方向相反 */
        Menu_SelectMove((int16_t)(-dir));
        s_dirty = 1U;
        return;
    }

    switch (s_sel)
    {
        case SET_ITEM_AMMO:
            /* 装填目标值已取消：本项直接装定“当前余弹数”（仅内存，不写 EEPROM）。
             * 运行页显示同一个变量，射击时由 IN_7 计数递减。
             * 到达上下限后回到另一端，循环变化 */
            idx = (int16_t)g_cfg.ammo + dir;
            if (idx > (int16_t)AMMO_MAX)
            {
                idx = (int16_t)AMMO_MIN;
            }
            else if (idx < (int16_t)AMMO_MIN)
            {
                idx = (int16_t)AMMO_MAX;
            }
            g_cfg.ammo = (uint16_t)idx;
            break;

        case SET_ITEM_FIRE:
            idx = (int16_t)g_cfg.fire_mode + dir;
            if (idx < 0) { idx = (int16_t)FIRE_MODE_NUM - 1; }
            if (idx >= (int16_t)FIRE_MODE_NUM) { idx = 0; }
            g_cfg.fire_mode = (uint8_t)idx;
            break;

        case SET_ITEM_MODULE:
            idx = (int16_t)g_cfg.module_type + dir;
            if (idx < 0) { idx = (int16_t)MODULE_NUM - 1; }
            if (idx >= (int16_t)MODULE_NUM) { idx = 0; }
            g_cfg.module_type = (uint8_t)idx;
            break;

        case SET_ITEM_SENSOR1:
            idx = (int16_t)g_cfg.sensor1_type + dir;
            if (idx < 0) { idx = (int16_t)SENSOR_TYPE_NUM - 1; }
            if (idx >= (int16_t)SENSOR_TYPE_NUM) { idx = 0; }
            Relay_SetType(1U, (uint8_t)idx);       /* 计数传感器(IN_7)：立即作用于 RELAY_1 */
            break;

        case SET_ITEM_SENSOR2:
            idx = (int16_t)g_cfg.sensor2_type + dir;
            if (idx < 0) { idx = (int16_t)SENSOR_TYPE_NUM - 1; }
            if (idx >= (int16_t)SENSOR_TYPE_NUM) { idx = 0; }
            Relay_SetType(2U, (uint8_t)idx);       /* 位置传感器(IN_9/IN_10)：立即作用于 RELAY_2 */
            break;

        case SET_ITEM_DIR:                         /* 装填电机方向 */
            g_cfg.load_dir = (g_cfg.load_dir == LOAD_DIR_FORWARD) ?
                             LOAD_DIR_REVERSE : LOAD_DIR_FORWARD;
            break;

        case SET_ITEM_PULSE:
            /* 到上下限后跳到另一端，循环变化 */
            idx = (int16_t)g_cfg.pulse_ms + dir;
            if (idx > (int16_t)SHOT_PULSE_MAX)
            {
                idx = (int16_t)SHOT_PULSE_MIN;
            }
            else if (idx < (int16_t)SHOT_PULSE_MIN)
            {
                idx = (int16_t)SHOT_PULSE_MAX;
            }
            g_cfg.pulse_ms = (uint16_t)idx;
            break;

        case SET_ITEM_GAP:
            idx = (int16_t)g_cfg.gap_ms + dir;
            if (idx > (int16_t)SHOT_GAP_MAX)
            {
                idx = (int16_t)SHOT_GAP_MIN;
            }
            else if (idx < (int16_t)SHOT_GAP_MIN)
            {
                idx = (int16_t)SHOT_GAP_MAX;
            }
            g_cfg.gap_ms = (uint16_t)idx;
            break;

        case SET_ITEM_BURST:
            idx = (int16_t)g_cfg.burst_shots + dir;
            if (idx > (int16_t)BURST_SHOTS_MAX)
            {
                idx = (int16_t)BURST_SHOTS_MIN;
            }
            else if (idx < (int16_t)BURST_SHOTS_MIN)
            {
                idx = (int16_t)BURST_SHOTS_MAX;
            }
            g_cfg.burst_shots = (uint8_t)idx;
            break;

        case SET_ITEM_LBURST:
            idx = (int16_t)g_cfg.long_burst_shots + dir;
            if (idx > (int16_t)LONG_BURST_MAX)
            {
                idx = (int16_t)LONG_BURST_MIN;
            }
            else if (idx < (int16_t)LONG_BURST_MIN)
            {
                idx = (int16_t)LONG_BURST_MAX;
            }
            g_cfg.long_burst_shots = (uint8_t)idx;
            break;

        default:
            break;
    }

    s_dirty = 1U;
    s_cfg_dirty = 1U;
    s_save_tick = HAL_GetTick();
    LOG("[SET] item=%d val=%d\r\n", s_sel, Menu_ItemValue(s_sel));

    /* 传感器类型改完立即保存，避免停在设置模式时掉电丢失 */
    if ((s_sel == SET_ITEM_SENSOR1) || (s_sel == SET_ITEM_SENSOR2))
    {
        if (EEPROM_SaveConfig() == HAL_OK)
        {
            LOG("[EEPROM] cfg saved\r\n");
            s_cfg_dirty = 0U;
        }
        else
        {
            LOG("[EEPROM] save failed\r\n");
        }
    }
}

/* 绘制一个设置项，y 为该行起始行像素
 * 参数：item —— 设置项索引 0~SET_ITEM_NUM-1
 *       y    —— 该行起始 y 像素（20 / 35 / 50）
 * 返回：无
 * 调用：Menu_DrawSet 逐行绘制
 * 注意：两种反显含义不同 —— 左侧标题行只要“选中”就反显（hl），
 *       右侧数值只在“选中且正在调节”时反显（edit），用颜色区分是“选中”还是“可改”
 */
static void Menu_DrawSetItem(uint8_t item, uint8_t y)
{
    uint8_t hl   = (item == s_sel) ? 1U : 0U;      /* 1 = 当前选中行（左侧标题反显） */
    uint8_t edit = (uint8_t)(hl & s_edit);         /* 1 = 选中且处于调节态（右侧数值反显） */

    switch (item)
    {
        case SET_ITEM_AMMO:
            OLED_ShowUTF8(0U, y, "余弹数", hl);
            ShowNumRight(y, g_cfg.ammo, edit);
            break;

        case SET_ITEM_FIRE:
            OLED_ShowUTF8(0U, y, "射击模式", hl);
            ShowTextRight(y, s_fire_name[g_cfg.fire_mode], edit);
            break;

        case SET_ITEM_MODULE:
            OLED_ShowUTF8(0U, y, "模块型号", hl);
            ShowTextRight(y, s_module_name[g_cfg.module_type], edit);
            break;

        case SET_ITEM_SENSOR1:
            /* 计数传感器（接 IN_7）：只用于射击计数，由 RELAY_1 选择 NPN/PNP */
            OLED_ShowUTF8(0U, y, "计数传感器类型", hl);
            ShowTextRight(y, s_sensor_name[g_cfg.sensor1_type], edit);
            break;

        case SET_ITEM_SENSOR2:
            /* 位置传感器（IN_10 初始到位 / IN_9 到位位置）：由 RELAY_2 选择 NPN/PNP */
            OLED_ShowUTF8(0U, y, "位置传感器类型", hl);
            ShowTextRight(y, s_sensor_name[g_cfg.sensor2_type], edit);
            break;

        case SET_ITEM_DIR:                         /* 装填电机方向 */
            OLED_ShowUTF8(0U, y, "电机方向", hl);
            ShowTextRight(y, s_dir_name[g_cfg.load_dir], edit);
            break;

        case SET_ITEM_PULSE:
            OLED_ShowUTF8(0U, y, "单发脉冲时长", hl);
            ShowNumRight(y, g_cfg.pulse_ms, edit);
            break;

        case SET_ITEM_GAP:
            OLED_ShowUTF8(0U, y, "双发间隔时长", hl);
            ShowNumRight(y, g_cfg.gap_ms, edit);
            break;

        case SET_ITEM_BURST:
            OLED_ShowUTF8(0U, y, "点射用弹数", hl);
            ShowNumRight(y, g_cfg.burst_shots, edit);
            break;

        case SET_ITEM_LBURST:
            OLED_ShowUTF8(0U, y, "长点射用弹数", hl);
            ShowNumRight(y, g_cfg.long_burst_shots, edit);
            break;

        default:
            break;
    }
}

/* 整屏绘制设置页
 * 参数：无
 * 返回：无
 * 调用：Menu_Task 在 s_dirty 置位时调用（按键/切挡）
 * 注意：先 OLED_Clear 再逐行绘制，最后统一 OLED_Refresh（一次 SPI 传输整屏），
 *       避免逐行刷新造成闪烁；标题行固定，正文只显示 3 行，
 *       用 (s_top + i) % SET_ITEM_NUM 取模，使窗口本身也是循环的
 */
static void Menu_DrawSet(void)
{
    uint8_t i;
    uint8_t item;

    OLED_Clear();
    OLED_ShowUTF8(0U, SET_TITLE_Y, "设置模式", 0U);   /* 标题行固定 */

    /* 标题下方三行循环显示设置项 */
    for (i = 0U; i < SET_ROW_NUM; i++)
    {
        item = (uint8_t)((s_top + i) % SET_ITEM_NUM);
        Menu_DrawSetItem(item, (uint8_t)(SET_ROW1_Y + (i * SET_ROW_STEP)));
    }

    OLED_Refresh();
}

/* 长按连加/连减（步进 10）适用的项：余弹数、双发间隔
 * 参数：无
 * 返回：1 = 当前项支持大步进（SET_ITEM_AMMO / SET_ITEM_GAP），0 = 只按 1 步进
 * 调用：Menu_SetKeys 的长按重复分支
 * 注意：只对“量程较大、需要快速调”的两项开放大步进；
 *       其它项范围小（如点射发数 2~6），大步进容易跳过目标值
 */
static uint8_t Menu_IsFastItem(void)
{
    return ((s_sel == SET_ITEM_AMMO) || (s_sel == SET_ITEM_GAP)) ? 1U : 0U;
}

/* 设置模式按键处理：A/B 调整、C 进入/退出调节
 * 参数：now —— 本次调用时刻（HAL_GetTick，ms），用于长按/连加计时
 * 返回：无
 * 调用：Menu_Task 在设置模式下每轮调用
 * 注意：1) A/B 用边沿触发单步；按住超过 WORK_KEY_REPEAT_DELAY_MS(500ms) 后
 *          每 WORK_KEY_REPEAT_MS(100ms) 连发一次（仅调节态且为快调项，步进 10）；
 *       2) C 是“进入 / 退出调节”的切换键，退出时立即保存 EEPROM；
 *       3) 退出调节后 s_edit=0，此时 A/B 变成切换设置项而不是改数值
 */
static void Menu_SetKeys(uint32_t now)
{
    uint8_t kA = DI_IsHigh(MENU_KEY_A_CH);
    uint8_t kB = DI_IsHigh(MENU_KEY_B_CH);
    uint8_t kC = DI_IsHigh(MENU_KEY_C_CH);

    if (kA != 0U)
    {
        if (s_prev[0] == 0U)
        {
            s_press_tick = now;
            s_rep_tick   = now;
            Menu_Adjust(1);
        }
        else if ((s_edit != 0U) && Menu_IsFastItem() &&
                 ((now - s_press_tick) >= WORK_KEY_REPEAT_DELAY_MS) &&
                 ((now - s_rep_tick) >= WORK_KEY_REPEAT_MS))
        {
            s_rep_tick = now;   /* 长按连加：按住满 500ms 后每 100ms 加 10（仅调节态的快调项） */
            Menu_Adjust(10);
        }
    }

    if (kB != 0U)
    {
        if (s_prev[1] == 0U)
        {
            s_press_tick = now;
            s_rep_tick   = now;
            Menu_Adjust(-1);
        }
        else if ((s_edit != 0U) && Menu_IsFastItem() &&
                 ((now - s_press_tick) >= WORK_KEY_REPEAT_DELAY_MS) &&
                 ((now - s_rep_tick) >= WORK_KEY_REPEAT_MS))
        {
            s_rep_tick = now;
            Menu_Adjust(-10);
        }
    }

    /* C：进入 / 退出调节，退出时保存 */
    if ((kC != 0U) && (s_prev[2] == 0U))
    {
        if (s_edit != 0U)
        {
            /* 退出调节：立即落盘，避免用户改完直接断电丢参数 */
            s_edit = 0U;
            if (EEPROM_SaveConfig() == HAL_OK)
            {
                LOG("[EEPROM] cfg saved\r\n");
                s_cfg_dirty = 0U;
            }
            else
            {
                LOG("[EEPROM] save failed\r\n");
            }
        }
        else
        {
            s_edit = 1U;
        }
        s_dirty = 1U;
    }

    s_prev[0] = kA;
    s_prev[1] = kB;
    s_prev[2] = kC;
}

/*------------------------- 工作 / 调试模式 -------------------------*/
/* 绘制工作 / 调试运行页
 * 参数：mode —— WORK_MODE_WORK / WORK_MODE_DEBUG，其它值按“模式异常”显示
 * 返回：无
 * 调用：Menu_Task 每 RUN_REFRESH_MS(200ms) 刷新一次，切挡时立即刷新
 * 注意：工作页第三行只区分“装填中 / 射击中 / 未装填”（其余瞬态归为未装填），
 *       调试页显示 IN_10 初始到位与 IN_9 到位位置的实时电平；
 *       标题右侧的模块型号只在模式有效时显示
 */
static void Menu_DrawRun(uint8_t mode)
{
    const char *mode_name;
    const char *module_name;
    uint8_t w;
    uint8_t st;
    const char *state_name;

    if (mode == WORK_MODE_WORK)
    {
        mode_name = "工作模式";
    }
    else if (mode == WORK_MODE_DEBUG)
    {
        mode_name = "调试模式";
    }
    else
    {
        mode_name = "模式异常";
    }

    OLED_Clear();

    /* 标题：左侧挡位，右侧模块型号靠右对齐（右边界 UI_RIGHT_X） */
    OLED_ShowUTF8(0U, SET_TITLE_Y, mode_name, 0U);

    if (mode != WORK_MODE_INVALID)
    {
        module_name = s_module_name[g_cfg.module_type];
        w = Utf8Width(module_name);
        if (w > UI_RIGHT_X)
        {
            w = UI_RIGHT_X;
        }
        OLED_ShowUTF8((uint8_t)(UI_RIGHT_X - w), SET_TITLE_Y, module_name, 0U);
    }

    /* 余弹数（三挡都显示） */
    OLED_ShowUTF8(0U, SET_ROW1_Y, "余弹数", 0U);
    ShowNumRight(SET_ROW1_Y, Work_GetAmmo(), 0U);

    if (mode == WORK_MODE_WORK)
    {
        OLED_ShowUTF8(0U, SET_ROW2_Y, "射击模式", 0U);
        ShowTextRight(SET_ROW2_Y, s_fire_name[g_cfg.fire_mode], 0U);

        st = Work_GetState();
        if (st == WORK_ST_LOAD)
        {
            state_name = "装填中";
        }
        else if (st == WORK_ST_FIRE)
        {
            state_name = "射击中";
        }
        else
        {
            state_name = "未装填";
        }

        OLED_ShowUTF8(0U, SET_ROW3_Y, "工作状态", 0U);
        ShowTextRight(SET_ROW3_Y, state_name, 0U);
    }
    else
    {
        /* 调试模式：显示两路位置信号
         *   初始到位 = IN_10（WORK_INIT_CH）
         *   到位位置 = IN_9（WORK_TARGET_CH） */
        OLED_ShowUTF8(0U, SET_ROW2_Y, "初始到位信号", 0U);
        ShowTextRight(SET_ROW2_Y, (DI_IsHigh(WORK_INIT_CH) != 0U) ? "有" : "无", 0U);

        OLED_ShowUTF8(0U, SET_ROW3_Y, "到位位置信号", 0U);
        ShowTextRight(SET_ROW3_Y, (DI_IsHigh(WORK_TARGET_CH) != 0U) ? "有" : "无", 0U);
    }

    OLED_Refresh();
}

/*------------------------- RTT 镜像打印 -------------------------*/
/* 把 OLED 当前页面的内容以 ASCII 打印到 RTT（每 UI_LOG_PERIOD_MS 一次）。
 * 三种模式与本文件 Menu_DrawSet / Menu_DrawRun 的显示项一一对应：
 *   SET    ：标题“设置模式” + 全部 9 个设置项（含选中项/调节态/滚动位置）
 *   WORK   ：模块型号、余弹数、射击模式、工作状态
 *   DEBUG  ：余弹数、初始到位信号(IN_10)、到位位置信号(IN_9)
 *   INVALID：标题“模式异常” + 模块型号
 * 说明：RTT 不支持中文，枚举值统一用 ASCII 代号输出。
 *
 * 参数：无
 * 返回：无
 * 调用：Menu_Task 每 UI_LOG_PERIOD_MS(500ms) 调用一次（主循环上下文）
 * 注意：只读 g_cfg / Work_GetState()，不修改任何状态；
 *       这是“屏幕看不全时”的辅助调试手段，字段顺序与 OLED 逐行对应
 */
static void Menu_LogUi(void)
{
    uint8_t     st;
    const char *state;

    switch (g_cfg.work_mode)
    {
        case WORK_MODE_SET:
            /* 设置页：sel = 当前选中项，edit = 是否处于调节态，top = 可见窗口首项 */
            LOG("[UI] SET top=%d sel=%d edit=%d fire=%s module=%s cnt=%s pos=%s dir=%s pulse=%d gap=%d burst=%d lburst=%d ammo=%d\r\n",
                s_top, s_sel, s_edit,
                s_fire_en[g_cfg.fire_mode], s_module_en[g_cfg.module_type],
                s_sensor_name[g_cfg.sensor1_type], s_sensor_name[g_cfg.sensor2_type],
                s_dir_en[g_cfg.load_dir], g_cfg.pulse_ms, g_cfg.gap_ms,
                g_cfg.burst_shots, g_cfg.long_burst_shots, g_cfg.ammo);
            break;

        case WORK_MODE_WORK:
            /* 工作页：工作状态与 OLED 一致（装填中/射击中/未装填），另附原始状态码 */
            st = Work_GetState();
            if (st == WORK_ST_LOAD)
            {
                state = "LOADING";
            }
            else if (st == WORK_ST_FIRE)
            {
                state = "FIRING";
            }
            else
            {
                state = "IDLE";
            }
            LOG("[UI] WORK module=%s ammo=%d fire=%s state=%s st=%d\r\n",
                s_module_en[g_cfg.module_type], g_cfg.ammo,
                s_fire_en[g_cfg.fire_mode], state, st);
            break;

        case WORK_MODE_DEBUG:
            /* 调试页：init = 初始到位信号(前到位 IN_10)，target = 到位位置信号(后到位 IN_9) */
            LOG("[UI] DEBUG ammo=%d init(IN10)=%d target(IN9)=%d\r\n",
                g_cfg.ammo,
                DI_IsHigh(WORK_INIT_CH), DI_IsHigh(WORK_TARGET_CH));
            break;

        default:
            /* 拨动开关两路同时为高：标题显示“模式异常” */
            LOG("[UI] INVALID module=%s ammo=%d\r\n",
                s_module_en[g_cfg.module_type], g_cfg.ammo);
            break;
    }
}

/*------------------------- 对外接口 -------------------------*/
/* 菜单初始化：复位界面状态并以当前按键电平作为边沿基准
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main），在 OLED_Init 之后
 * 注意：s_mode 置 0xFF 是为了让 Menu_Task 第一次执行时必定判为“切挡”，
 *       从而强制绘制首帧；s_dirty 置 1 同理
 */
void Menu_Init(void)
{
    s_mode         = 0xFFU;
    s_sel          = SET_ITEM_AMMO;
    s_top          = 0U;
    s_edit         = 0U;
    s_dirty        = 1U;
    s_cfg_dirty    = 0U;
    s_save_tick    = HAL_GetTick();
    s_refresh_tick = HAL_GetTick();
    s_press_tick   = s_refresh_tick;
    s_rep_tick     = s_refresh_tick;
    s_log_tick     = s_refresh_tick;

    s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
    s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
    s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);
}

/* 菜单主任务：切挡处理、按键、延时保存、页面刷新、RTT 镜像
 * 参数：无
 * 返回：无
 * 调用：主循环每轮调用（非阻塞；各子动作自带节拍判断）
 * 注意：1) 切挡时若仍在设置页且有未保存改动，先落盘再切换；
 *       2) 切挡后同步按键快照，防止仍按住的键在新挡位被当成新按下；
 *       3) 设置页的 EEPROM 写入延迟 300ms，写失败会把 s_save_tick 刷新后重试；
 *       4) 非设置页每 200ms 整屏刷新一次，用于更新余弹/状态
 */
void Menu_Task(void)
{
    uint8_t mode = g_cfg.work_mode;
    uint32_t now = HAL_GetTick();

    /* 切挡：离开设置模式时保存配置，并同步按键状态防止误触发 */
    if (mode != s_mode)
    {
        if ((s_mode == WORK_MODE_SET) && (s_cfg_dirty != 0U))
        {
            (void)EEPROM_SaveConfig();
            s_cfg_dirty = 0U;
        }

        s_mode = mode;
        s_edit = 0U;
        /* 按当前选中项重算窗口首行，保证进入设置页时选中项一定可见 */
        s_top  = (s_sel >= SET_ROW_NUM) ? (uint8_t)(s_sel - SET_ROW_NUM + 1U) : 0U;
        s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
        s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
        s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);
        s_dirty = 1U;
    }

    /* 把 OLED 当前页面的内容镜像到 RTT，每 500ms 一帧 */
    if ((now - s_log_tick) >= UI_LOG_PERIOD_MS)
    {
        s_log_tick = now;
        Menu_LogUi();
    }

    if (mode == WORK_MODE_SET)
    {
        Menu_SetKeys(now);

        /* 参数改动后 300ms 无操作即写入 EEPROM，防止直接断电丢失 */
        if ((s_cfg_dirty != 0U) && ((now - s_save_tick) >= CFG_SAVE_DELAY_MS))
        {
            if (EEPROM_SaveConfig() == HAL_OK)
            {
                s_cfg_dirty = 0U;
                LOG("[EEPROM] cfg saved\r\n");
            }
            else
            {
                s_save_tick = now;       /* 写失败稍后重试 */
                LOG("[EEPROM] save failed\r\n");
            }
        }

        if (s_dirty != 0U)
        {
            Menu_DrawSet();
            s_dirty = 0U;
        }
    }
    else
    {
        /* 工作 / 调试模式：定时刷新状态 */
        if ((s_dirty != 0U) || ((now - s_refresh_tick) >= RUN_REFRESH_MS))
        {
            s_refresh_tick = now;
            Menu_DrawRun(mode);
            s_dirty = 0U;
        }
    }
}
