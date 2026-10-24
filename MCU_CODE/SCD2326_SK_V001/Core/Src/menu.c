/* 三挡界面：设置模式 / 工作模式 / 调试模式
 *
 * 设置模式（拨动开关上挡）：
 *   A = 向上切换设置项 / 增大数值     B = 向下切换设置项 / 减小数值
 *   C = 进入 / 退出调节；退出时自动保存到 EEPROM
 *   标题行固定，下面三行循环显示设置项：
 *   余弹数（0~200）、射击模式、模块型号、传感器1类型、传感器2类型
 *
 * 工作模式（中挡）：显示余弹数、射击模式、工作状态（未装填/装填中/射击中）
 * 调试模式（下挡）：显示余弹数、初始到位信号、到位位置信号
 *
 * 工作 / 调试模式下 A、B、C 的功能由 work.c 处理，本文件只负责显示。
 */
#include "menu.h"
#include "oled.h"
#include "digital_input.h"
#include "global.h"
#include "work.h"
#include "eeprom.h"
#include "relay.h"

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
#define SET_ITEM_NUM     5U

/* 运行页刷新周期 */
#define RUN_REFRESH_MS   200U

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

static uint8_t  s_mode = 0xFFU;   /* 上次绘制的挡位 */
static uint8_t  s_sel;            /* 设置项选中位置 */
static uint8_t  s_top;            /* 设置页滚动窗口首项 */
static uint8_t  s_edit;           /* 1 = 正在调节数值 */
static uint8_t  s_dirty;          /* 1 = 需要重绘 */
static uint8_t  s_cfg_dirty;      /* 1 = 配置已改，需写 EEPROM */
static uint8_t  s_prev[3];        /* A/B/C 上次电平 */
static uint32_t s_press_tick;     /* A/B 按下时刻（连加判定） */
static uint32_t s_rep_tick;       /* 连加上次触发时刻 */
static uint32_t s_refresh_tick;   /* 运行页刷新计时 */

/*------------------------- 字符串工具 -------------------------*/
/* UTF-8 字符串像素宽度：ASCII 8px，汉字 15px（与 OLED_ShowUTF8 一致） */
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

/* 数字宽度（8x15，按位数计算） */
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

/* 右对齐显示数字（余弹数值上移 3px） */
static void ShowNumRight(uint8_t y, uint16_t v, uint8_t mode)
{
    uint8_t w = NumWidth(v);
    uint8_t ny = (y >= AMMO_NUM_UP_PX) ? (uint8_t)(y - AMMO_NUM_UP_PX) : 0U;

    OLED_ShowNum((uint8_t)(UI_RIGHT_X - w), ny, (uint32_t)v, (uint8_t)(w / OLED_ASC_WIDTH), mode);
}

/* 右对齐显示文本 */
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
/* 列表态：切换设置项，并保证选中项落在可见窗口内 */
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
            /* 设置页装定的是“装填目标值”，运行页显示的是实际余弹数；
             * 到达上下限后回到另一端，循环变化 */
            idx = (int16_t)g_cfg.ammo_target + dir;
            if (idx > (int16_t)AMMO_MAX)
            {
                idx = (int16_t)AMMO_MIN;
            }
            else if (idx < (int16_t)AMMO_MIN)
            {
                idx = (int16_t)AMMO_MAX;
            }
            g_cfg.ammo_target = (uint16_t)idx;
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
            Relay_SetType(1U, (uint8_t)idx);       /* 立即作用于继电器 */
            break;

        default:                                   /* 传感器2类型 */
            idx = (int16_t)g_cfg.sensor2_type + dir;
            if (idx < 0) { idx = (int16_t)SENSOR_TYPE_NUM - 1; }
            if (idx >= (int16_t)SENSOR_TYPE_NUM) { idx = 0; }
            Relay_SetType(2U, (uint8_t)idx);       /* 立即作用于继电器 */
            break;
    }

    s_dirty = 1U;
    s_cfg_dirty = 1U;

    /* 传感器类型改完立即保存，避免停在设置模式时掉电丢失 */
    if ((s_sel == SET_ITEM_SENSOR1) || (s_sel == SET_ITEM_SENSOR2))
    {
        if (EEPROM_SaveConfig() == HAL_OK)
        {
            s_cfg_dirty = 0U;
        }
    }
}

/* 绘制一个设置项，y 为该行起始行像素 */
static void Menu_DrawSetItem(uint8_t item, uint8_t y)
{
    uint8_t hl   = (item == s_sel) ? 1U : 0U;
    uint8_t edit = (uint8_t)(hl & s_edit);

    switch (item)
    {
        case SET_ITEM_AMMO:
            OLED_ShowUTF8(0U, y, "余弹数", hl);
            ShowNumRight(y, g_cfg.ammo_target, edit);
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
            OLED_ShowUTF8(0U, y, "传感器1类型", hl);
            ShowTextRight(y, s_sensor_name[g_cfg.sensor1_type], edit);
            break;

        default:
            OLED_ShowUTF8(0U, y, "传感器2类型", hl);
            ShowTextRight(y, s_sensor_name[g_cfg.sensor2_type], edit);
            break;
    }
}

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
        else if ((s_edit != 0U) && (s_sel == SET_ITEM_AMMO) &&
                 ((now - s_press_tick) >= WORK_KEY_REPEAT_DELAY_MS) &&
                 ((now - s_rep_tick) >= WORK_KEY_REPEAT_MS))
        {
            s_rep_tick = now;
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
        else if ((s_edit != 0U) && (s_sel == SET_ITEM_AMMO) &&
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
            s_edit = 0U;
            (void)EEPROM_SaveConfig();
            s_cfg_dirty = 0U;
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
        OLED_ShowUTF8(0U, SET_ROW2_Y, "初始到位信号", 0U);
        ShowTextRight(SET_ROW2_Y, (DI_IsHigh(WORK_REAR_CH) != 0U) ? "有" : "无", 0U);

        OLED_ShowUTF8(0U, SET_ROW3_Y, "到位位置信号", 0U);
        ShowTextRight(SET_ROW3_Y, (DI_IsHigh(WORK_FRONT_CH) != 0U) ? "有" : "无", 0U);
    }

    OLED_Refresh();
}

/*------------------------- 对外接口 -------------------------*/
void Menu_Init(void)
{
    s_mode         = 0xFFU;
    s_sel          = SET_ITEM_AMMO;
    s_top          = 0U;
    s_edit         = 0U;
    s_dirty        = 1U;
    s_cfg_dirty    = 0U;
    s_refresh_tick = HAL_GetTick();
    s_press_tick   = s_refresh_tick;
    s_rep_tick     = s_refresh_tick;

    s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
    s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
    s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);
}

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
        s_top  = (s_sel >= SET_ROW_NUM) ? (uint8_t)(s_sel - SET_ROW_NUM + 1U) : 0U;
        s_prev[0] = DI_IsHigh(MENU_KEY_A_CH);
        s_prev[1] = DI_IsHigh(MENU_KEY_B_CH);
        s_prev[2] = DI_IsHigh(MENU_KEY_C_CH);
        s_dirty = 1U;
    }

    if (mode == WORK_MODE_SET)
    {
        Menu_SetKeys(now);

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
