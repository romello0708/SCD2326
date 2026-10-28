#ifndef __GLOBAL_H
#define __GLOBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：global.h —— 系统信息、功能宏与全局配置结构
 *
 * 模块职责：集中定义板卡/版本信息、输入输出通道映射、工作模式及参数取值范围，
 *           并声明全系统共享的配置结构体 g_cfg。
 * 硬件：本文件不直接操作外设；DI 通道号对应 digital_input.c 的扫描表，
 *       继电器/电机/串口等硬件细节见各自模块的头文件。
 * 使用约定：
 *   1) 带 MIN/MAX 的宏既用于菜单调节，也用于 EEPROM 载入时的越界校验，改动需同步 menu.c；
 *   2) 需要掉电保留的字段必须同步加入 EEPROM_Config_t（见 eeprom.h）；
 *   3) 结构体布局变化后递增 EEPROM_CFG_MAGIC，避免旧数据被误用。
 * 运行上下文：g_cfg 会被主循环与 TIM2 1ms 中断同时访问（DI 位图、上升沿计数），
 *           单字节/单半字访问在 Cortex-M3 上是原子的；复合字段的时序由业务代码保证。
 */

/*============================================================================*/
/* 系统信息宏定义                                                              */
/*============================================================================*/
#define SW_VERSION_MAJOR      0U       /* 主版本号，RTT 启动日志打印 */
#define SW_VERSION_MINOR      1U       /* 次版本号 */
#define SW_VERSION_BUILD      0U       /* 构建号 */

#define BOARD_NAME            "SCD2326"        /* 板卡型号，RTT 启动日志用 */
#define BOARD_HW_VERSION      "V0.3"           /* 硬件版本（对应原理图） */
#define BOARD_SCH_VERSION     "V0.3"           /* 原理图版本 */
#define SCREEN_NAME           "ZJY_M242_OLED"  /* 屏体型号（信息性定义） */
#define SCREEN_DRIVER_IC      "SSD1309"        /* 屏驱动 IC，与 oled.c 初始化序列对应 */

/*============================================================================*/
/* 数字输入（DI）配置                                                          */
/*============================================================================*/
#define DI_NUM                10U   /* 10 路输入 IN_1~IN_10（PA15/PB3~PB7/PB1/PA6/PA5/PA4，见 digital_input.c） */
#define DI_ACTIVE_LEVEL       1U    /* 全部为高电平有效（保留定义，扫描代码固定按高有效实现） */

/* 软件消抖深度（单位 = 1 个 1ms 采样）：
 * 每路输入维护一个 0~DI_DEBOUNCE_CNT 的积分值，采样为高加 1、为低减 1，
 * 积分打满才判为高、归零才判为低。数值即消抖时间(ms)，
 * 机械触点/行程开关一般取 4~8ms；传感器脉冲较窄时可减到 3。 */
#define DI_DEBOUNCE_CNT       4U

/* 限位通道（IN_9 到位位置、IN_10 初始到位）专用的消抖深度。
 * 这两路直接决定“到限位→停机”的快慢，取更小的积分深度以缩短响应延迟；
 * 限位开关为行程开关/霍尔，动作后电平基本不再抖动，2ms 已能滤掉毛刺。
 * 若现场出现误停机，可改回 DI_DEBOUNCE_CNT(4)。 */
#define DI_DEBOUNCE_LIMIT_CNT 2U

/*============================================================================*/
/* 操作面板输入配置（见《装填工装简易操作说明》）                               */
/*============================================================================*/
/* 菜单结构：三级，每级 5 项 */
#define MENU_LEVELS           3U    /* 菜单层级数（根/设置/子项） */
#define MENU_ITEMS            5U    /* 每屏最多显示 5 项，与 OLED 13x13 字高匹配 */

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

/* 射击模式（设置模式可调，工作模式生效） */
#define FIRE_MODE_UNLIMITED   0U    /* 无限制：按住连发，不受余弹限制 */
#define FIRE_MODE_AUTO        1U    /* 连射：按住连发，受余弹限制 */
#define FIRE_MODE_LONG_BURST  2U    /* 长点射：每按一次 6 发 */
#define FIRE_MODE_BURST       3U    /* 点射：每按一次 3 发 */
#define FIRE_MODE_SINGLE      4U    /* 单发：每按一次 1 发 */
#define FIRE_MODE_NUM         5U    /* 射击模式数量：菜单取模、EEPROM 越界校验用 */

/* 模块型号（设置模式可调，显示在各模式标题栏） */
#define MODULE_FIRE           0U    /* 火力模块 */
#define MODULE_SCOUT          1U    /* 侦打车 */
#define MODULE_CAR_C          2U    /* C 车 */
#define MODULE_CAR_B          3U    /* B 车 */
#define MODULE_CAR_A          4U    /* A 车 */
#define MODULE_8X8            5U    /* 8*8 */
#define MODULE_NUM            6U    /* 模块型号数量：菜单取模、EEPROM 越界校验用 */

/* 余弹数范围 */
#define AMMO_MIN              0U    /* 下限 0：余弹耗尽后不允许再射击（无限制模式除外） */
#define AMMO_MAX              200U  /* 上限 200：也是上电默认余弹数 */
#define AMMO_STEP             1U    /* 菜单每次调整步长 */

/* 装填电机方向（设置模式可切换，工作/调试模式都生效） */
#define LOAD_DIR_FORWARD      0U    /* 正向：按程序设定方向 */
#define LOAD_DIR_REVERSE      1U    /* 反向：与设定方向相反 */
#define LOAD_DIR_NUM          2U    /* 方向选项数量，菜单循环用 */

/* 射击时序参数范围（设置模式可调） */
#define SHOT_PULSE_MIN        1U    /* 单发脉冲时长 ms */
#define SHOT_PULSE_MAX        200U
#define SHOT_GAP_MIN          10U   /* 双发间隔时长 ms */
#define SHOT_GAP_MAX          1000U
#define BURST_SHOTS_MIN       2U    /* 点射用弹数 */
#define BURST_SHOTS_MAX       6U
#define LONG_BURST_MIN        3U    /* 长点射用弹数 */
#define LONG_BURST_MAX        10U

/*============================================================================*/
/* 串口状态打印                                                                */
/*============================================================================*/
#define COMM_STATUS_PERIOD_MS 500U  /* 输入状态打印周期 */

/*============================================================================*/
/* 全局配置变量                                                                */
/*============================================================================*/
typedef struct
{
    /* 数字输入（由 DI_ScanIsr 在 TIM2 1ms 中断中刷新） */
    uint16_t di_state;              /* 去抖后的输入状态，bit0 = IN_1 … bit9 = IN_10，1 = 高电平 */
    uint16_t di_raw;                /* 最近一次扫描的原始电平，位定义同 di_state */
    uint16_t di_rise_cnt[DI_NUM];   /* 各路上升沿计数 0~65535，溢出回绕（IN_7 用于射击计数） */

    /* 工作模式（由拨动开关 IN_2 / IN_3 解码） */
    uint8_t  work_mode;             /* WORK_MODE_SET / WORK / DEBUG / INVALID */

    /* 传感器类型（两路继电器分别配置：高电平 = NPN，低电平 = PNP） */
    /* 注：实际电平约定以 relay.h / relay.c 为准：低电平 = NPN（不吸合），高电平 = PNP（吸合）。 */
    uint8_t  sensor1_type;          /* 传感器1：SENSOR_TYPE_NPN / SENSOR_TYPE_PNP */
    uint8_t  sensor2_type;          /* 传感器2：SENSOR_TYPE_NPN / SENSOR_TYPE_PNP */

    /* 工作/调试模式 */
    uint16_t ammo;                  /* 当前余弹数 0~200：射击时由 IN_7 计数递减，掉电不保存 */
    uint8_t  fire_mode;             /* 射击模式 FIRE_MODE_*（0~4） */
    uint8_t  module_type;           /* 模块型号 MODULE_*（0~5），仅影响标题栏显示 */
    uint16_t shot_total;            /* 累计射击次数（仅统计，当前无自增点） */

    /* 射击时序参数（设置模式可调） */
    uint16_t pulse_ms;              /* 单发脉冲时长 ms，1~200，默认 100 */
    uint16_t gap_ms;                /* 双发间隔时长 ms，10~1000，默认 150 */
    uint8_t  burst_shots;           /* 点射用弹数，2~6，默认 3 */
    uint8_t  long_burst_shots;      /* 长点射用弹数，3~10，默认 6 */

    /* 电机 */
    uint8_t  motor_duty_pct;        /* 电机默认占空比 %（已固定 90，不再作为设置项） */
    uint8_t  load_dir;              /* 装填电机方向 LOAD_DIR_*（工作/调试模式通用） */

    /* 电流 */
    uint8_t  over_current_a;        /* 过流阈值 A（默认 20），Work_Task 中与实测电流比较 */

    /* 通信 */
    uint8_t  echo_enable;           /* 1 = 串口收到帧后原样回传；0 = 直接丢弃 */
    uint8_t  status_print_enable;   /* 1 = 每 500ms 打印输入状态帧 */
    uint32_t baudrate;              /* 当前波特率，常用 9600 / 19200 / 38400 / 57600 / 115200 */

    /* 系统 */
    uint32_t run_seconds;           /* 运行秒数，Global_Task 每秒 +1，掉电不保存 */
    uint32_t reset_count;           /* 软件复位（看门狗/软复位）计数，当前预留未使用 */
} GlobalConfig_t;

extern GlobalConfig_t g_cfg;   /* 全局唯一配置实例，定义在 global.c */

void Global_Init(void);   /* 上电写入默认参数（须在 EEPROM_LoadConfig 之前调用） */
void Global_Task(void);   /* 1s 周期：运行时间等系统级计数 */

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_H */
