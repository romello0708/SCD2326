#ifndef __GLOBAL_H
#define __GLOBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*============================================================================*/
/* 系统信息宏定义                                                              */
/*============================================================================*/
#define SW_VERSION_MAJOR      0U
#define SW_VERSION_MINOR      2U
#define SW_VERSION_BUILD      0U

#define BOARD_NAME            "SCD2326"
#define BOARD_HW_VERSION      "V0.3"
#define BOARD_SCH_VERSION     "V0.3"
#define SCREEN_NAME           "ZJY_M242_OLED"
#define SCREEN_DRIVER_IC      "SSD1309"

/*============================================================================*/
/* 数字输入（DI）配置                                                          */
/*============================================================================*/
#define DI_NUM                10U   /* 10 路输入 IN_1~IN_10 */
#define DI_ACTIVE_LEVEL       1U    /* 全部为高电平有效 */
#define DI_FILTER_CNT         5U    /* 连续 5 次(5ms)电平一致才更新 */

/*============================================================================*/
/* 操作面板输入配置（见《装填工装简易操作说明》）                               */
/*============================================================================*/
/* 菜单结构：三级，每级 5 项 */
#define MENU_LEVELS           3U
#define MENU_ITEMS            5U

/* 拨动开关：单刀双掷，三挡，两个输入通道（高电平有效） */
#define SWITCH_SET_CH         2U    /* 上挡：设置模式 */
#define SWITCH_DEBUG_CH       3U    /* 下挡：调试模式（两路都低 = 中挡工作模式） */

/* 按键 A/B/C */
#define MENU_KEY_A_CH         6U    /* A：设置模式"↑"，工作模式"装填"，调试模式"拉栓" */
#define MENU_KEY_B_CH         5U    /* B：设置模式"↓"，工作模式无效，调试模式"复位" */
#define MENU_KEY_C_CH         4U    /* C：设置模式"进入/退出"，工作模式"射击"，调试模式"射击" */

/* 兼容旧命名（菜单内部使用） */
#define MENU_KEY_UP_CH        MENU_KEY_A_CH
#define MENU_KEY_DOWN_CH      MENU_KEY_B_CH
#define MENU_KEY_ENTER_CH     MENU_KEY_C_CH

#define MENU_LONG_PRESS_MS    800U  /* C 键长按判定时间 */

/* 工作模式 */
#define WORK_MODE_SET         0U    /* 设置模式 */
#define WORK_MODE_WORK        1U    /* 工作模式 */
#define WORK_MODE_DEBUG       2U    /* 调试模式 */
#define WORK_MODE_INVALID     3U    /* 拨动开关两路同时为高（异常） */

/*============================================================================*/
/* 串口状态打印                                                                */
/*============================================================================*/
#define COMM_STATUS_PERIOD_MS 500U  /* 输入状态打印周期 */

/*============================================================================*/
/* 全局配置变量                                                                */
/*============================================================================*/
typedef struct
{
    /* 数字输入 */
    uint16_t di_state;              /* 去抖后的输入状态，bit0 = IN_1 */
    uint16_t di_raw;                /* 最近一次扫描的原始电平 */
    uint16_t di_rise_cnt[DI_NUM];   /* 各路上升沿计数 */

    /* 工作模式（由拨动开关 IN_2 / IN_3 解码） */
    uint8_t  work_mode;

    /* 传感器类型（两路继电器切换 IN7~IN10 的 NPN / PNP 通道） */
    uint8_t  sensor_type;

    /* 工作/调试模式 */
    uint8_t  shot_enable;           /* 1 = 允许射击，0 = 禁止 */
    uint16_t load_total;            /* 装填计数（IN_7 上升沿累加） */
    uint16_t shot_count;            /* 射击次数（余弹 = load_total - shot_count） */

    /* 电机 */
    uint8_t  motor_duty_pct;        /* 电机默认占空比 % */

    /* 电流 */
    uint8_t  over_current_a;        /* 过流阈值 A */

    /* 通信 */
    uint8_t  echo_enable;           /* 1 = 串口回传打开 */
    uint8_t  status_print_enable;   /* 1 = 定时打印输入状态 */
    uint32_t baudrate;              /* 当前波特率 */

    /* 系统 */
    uint32_t run_seconds;           /* 运行秒数 */
    uint32_t reset_count;           /* 软件复位（看门狗/软复位）计数 */
} GlobalConfig_t;

extern GlobalConfig_t g_cfg;

void Global_Init(void);
void Global_Task(void);   /* 1s 周期：运行时间等系统级计数 */

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_H */
