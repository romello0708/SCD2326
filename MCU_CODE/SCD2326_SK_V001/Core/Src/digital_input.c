/* 10 路数字输入扫描（积分式软件消抖）
 * 硬件（均为高电平有效，GPIO 已在 MX_GPIO_Init 中配置为输入）：
 *   IN_1  PA15   IN_2  PB3    IN_3  PB4    IN_4  PB5    IN_5  PB6
 *   IN_6  PB7    IN_7  PB1    IN_8  PA6    IN_9  PA5    IN_10 PA4
 *
 * 扫描节拍：由 TIM2 1ms 更新中断调用，1 次采样 = 1ms。
 *
 * 消抖方式：积分式（滞回）消抖
 *   每路维护一个积分值 s_integ[i]，范围 0 ~ DI_DEBOUNCE_CNT：
 *     本次采样为高 → 积分 +1；本次采样为低 → 积分 -1（到 0 / 到上限饱和）。
 *     稳定值当前为低时，积分涨到 DI_DEBOUNCE_CNT 才翻转为高（并计一次上升沿）；
 *     稳定值当前为高时，积分跌到 0 才翻转为低。
 *   特点：
 *     1) 单个噪声脉冲只把积分往回拉一格，不会让整个计时清零重来，
 *        抖动期间不会长时间等不到“连续 N 次一致”；
 *     2) 两个翻转门限分别在两端，中间区间保持原状态，形成天然滞回，
 *        边沿附近的抖动不会被反复判成上升沿（IN_7 射击计数不会多算）；
 *     3) 每路只多一个字节状态，中断内运算量恒定（加减各一次比较）。
 *   代价：从电平稳定变化到输出翻转需要本路积分深度 ms 的时间
 *         （普通通道 DI_DEBOUNCE_CNT=4ms，限位通道 IN_9/IN_10 用 2ms）。
 *
 * 运行上下文：
 *   DI_Init    —— 上电初始化（main，在启动 TIM2 之前），把“上电时的实际电平”直接当作稳定值；
 *   DI_ScanIsr —— TIM2 1ms 更新中断的第一件事，后续 Work_LimitGuardIsr / Work_CountTask
 *                 都基于本次扫描的结果；
 *   DI_IsHigh 等查询函数 —— 主循环（menu / work / comm）与中断内均可调用，只读静态数组。
 *
 * 关键参数：DI_DEBOUNCE_CNT = 4（普通）、DI_DEBOUNCE_LIMIT_CNT = 2（限位），单位 1ms 采样。
 */
#include "digital_input.h"
#include "global.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} DI_Pin_t;

static const DI_Pin_t s_di_pin[DI_NUM] =
{
    { IN_1_GPIO_Port,  IN_1_Pin  },
    { IN_2_GPIO_Port,  IN_2_Pin  },
    { IN_3_GPIO_Port,  IN_3_Pin  },
    { IN_4_GPIO_Port,  IN_4_Pin  },
    { IN_5_GPIO_Port,  IN_5_Pin  },
    { IN_6_GPIO_Port,  IN_6_Pin  },
    { IN_7_GPIO_Port,  IN_7_Pin  },
    { IN_8_GPIO_Port,  IN_8_Pin  },
    { IN_9_GPIO_Port,  IN_9_Pin  },
    { IN_10_GPIO_Port, IN_10_Pin }
};

static uint8_t  s_stable[DI_NUM];   /* 消抖后的稳定电平 */
static uint8_t  s_integ[DI_NUM];    /* 积分值 0 ~ 本路消抖深度 */

/* 每路消抖深度（单位 1ms 采样）：按键、计数通道抗扰优先，用 DI_DEBOUNCE_CNT；
 * 限位通道 IN_9/IN_10 决定停机快慢，用更浅的 DI_DEBOUNCE_LIMIT_CNT。 */
static const uint8_t s_deb[DI_NUM] =
{
    DI_DEBOUNCE_CNT, DI_DEBOUNCE_CNT, DI_DEBOUNCE_CNT, DI_DEBOUNCE_CNT, DI_DEBOUNCE_CNT,
    DI_DEBOUNCE_CNT, DI_DEBOUNCE_CNT, DI_DEBOUNCE_CNT,
    DI_DEBOUNCE_LIMIT_CNT,   /* IN_9：到位位置信号 */
    DI_DEBOUNCE_LIMIT_CNT    /* IN_10：初始到位信号 */
};

/* 上电初始化：以各通道当前实际电平作为消抖初值
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main），在 DI_ScanIsr 开始运行之前、Work_Init 之前
 * 注意：积分值同时置到对应边界（高 → 满量程，低 → 0），
 *       避免 TIM2 启动后的第一拍把“上电前已存在的电平”误判成一次上升沿
 */
void DI_Init(void)
{
    uint8_t i;
    uint8_t raw;

    /* 上电直接以实际电平作为初始稳定值（积分同时置到对应边界），
     * 避免把上电前就存在的电平当成一次上升沿而误计一次数 */
    for (i = 0; i < DI_NUM; i++)
    {
        raw = (HAL_GPIO_ReadPin(s_di_pin[i].port, s_di_pin[i].pin) == GPIO_PIN_SET) ? 1U : 0U;
        s_stable[i] = raw;
        s_integ[i]  = (raw != 0U) ? s_deb[i] : 0U;

        if (raw != 0U)
        {
            g_cfg.di_state |= (uint16_t)(1U << i);
        }
        else
        {
            g_cfg.di_state &= (uint16_t)~(1U << i);
        }
    }
}

/* TIM2 1ms 中断内调用：采样 + 积分式消抖 + 稳定上升沿计数
 * 参数：无
 * 返回：无
 * 调用：TIM2 1ms 更新中断的第一项（HAL_TIM_PeriodElapsedCallback）
 * 注意：中断上下文，只做 GPIO 读与整数运算，耗时固定且很短；
 *       每拍都会刷新 g_cfg.di_raw（原始电平）与 g_cfg.di_state（去抖后位图），
 *       g_cfg.di_rise_cnt[] 只在“稳定值由低翻高”时 +1，因此抖动不会多计数
 */
void DI_ScanIsr(void)
{
    uint8_t i;
    uint8_t raw;

    g_cfg.di_raw = 0U;

    for (i = 0; i < DI_NUM; i++)
    {
        raw = (HAL_GPIO_ReadPin(s_di_pin[i].port, s_di_pin[i].pin) == GPIO_PIN_SET) ? 1U : 0U;
        if (raw != 0U)
        {
            g_cfg.di_raw |= (uint16_t)(1U << i);
        }

        /* 积分：高电平累加、低电平递减，两端饱和（上限 = 本路消抖深度） */
        if (raw != 0U)
        {
            if (s_integ[i] < s_deb[i])
            {
                s_integ[i]++;
            }
        }
        else if (s_integ[i] > 0U)
        {
            s_integ[i]--;
        }

        /* 只有积分到达两端才翻转稳定值；中间区间保持原状态（滞回） */
        if ((s_stable[i] == 0U) && (s_integ[i] >= s_deb[i]))
        {
            s_stable[i] = 1U;
            g_cfg.di_rise_cnt[i]++;          /* 只在稳定上升沿计数 */
        }
        else if ((s_stable[i] != 0U) && (s_integ[i] == 0U))
        {
            s_stable[i] = 0U;
        }

        if (s_stable[i] != 0U)
        {
            g_cfg.di_state |= (uint16_t)(1U << i);
        }
        else
        {
            g_cfg.di_state &= (uint16_t)~(1U << i);
        }
    }

    /* 拨动开关解码：上挡=设置，下挡=调试，中挡(两路都低)=工作 */
    {
        uint8_t up    = DI_IsHigh(SWITCH_SET_CH);
        uint8_t down  = DI_IsHigh(SWITCH_DEBUG_CH);

        if ((up != 0U) && (down != 0U))
        {
            g_cfg.work_mode = WORK_MODE_INVALID;
        }
        else if (up != 0U)
        {
            g_cfg.work_mode = WORK_MODE_SET;
        }
        else if (down != 0U)
        {
            g_cfg.work_mode = WORK_MODE_DEBUG;
        }
        else
        {
            g_cfg.work_mode = WORK_MODE_WORK;
        }
    }
}

/* 读取某一路去抖后的稳定电平
 * 参数：ch —— 通道号 1~10（对应 IN_1~IN_10），越界返回 0
 * 返回：1 = 高电平有效，0 = 低电平 / 通道号非法
 * 调用：主循环（work.c / menu.c / comm.c）与 TIM2 中断（挡位解码、限位判定）
 * 注意：读的是 s_stable[]，与本拍扫描结果一致；单字节读取天然原子
 */
uint8_t DI_IsHigh(uint8_t ch)
{
    if ((ch < 1U) || (ch > DI_NUM))
    {
        return 0U;
    }

    return s_stable[ch - 1U];
}

/* 统计当前为高电平的通道数量
 * 参数：无
 * 返回：0~10
 * 调用：主循环（状态打印/调试查看）
 * 注意：以去抖后的稳定值为准；用于快速判断是否有输入动作
 */
uint8_t DI_GetHighCount(void)
{
    uint8_t i;
    uint8_t cnt = 0U;

    for (i = 0; i < DI_NUM; i++)
    {
        if (s_stable[i] != 0U)
        {
            cnt++;
        }
    }

    return cnt;
}

/* 读取去抖后的通道位图
 * 参数：无
 * 返回：16 位位图，bit0 = IN_1，bit9 = IN_10，高位恒为 0
 * 调用：主循环（RTT 状态打印）
 * 注意：与 DI_GetRaw 对照可看出消抖是否正在起作用
 */
uint16_t DI_GetState(void)
{
    return g_cfg.di_state;
}

/* 读取最近一次扫描的原始电平位图（未经消抖）
 * 参数：无
 * 返回：16 位位图，bit0 = IN_1，bit9 = IN_10
 * 调用：主循环（调试/诊断）
 * 注意：仅在 DI_ScanIsr 中整字刷新；读取时可能刚好跨过更新，允许有 1ms 误差
 */
uint16_t DI_GetRaw(void)
{
    return g_cfg.di_raw;
}

/* 读取某一路的稳定上升沿累计次数
 * 参数：ch —— 通道号 1~10，越界返回 0
 * 返回：累加计数（16 位，自然回绕）
 * 调用：Work_CountTask（中断，IN_7 射击计数）、Work_Init（记录基准）
 * 注意：调用者应保存上次值并用无符号相减求增量，可正确处理回绕
 */
uint16_t DI_GetRiseCount(uint8_t ch)
{
    if ((ch < 1U) || (ch > DI_NUM))
    {
        return 0U;
    }

    return g_cfg.di_rise_cnt[ch - 1U];
}

/* 清零所有通道的上升沿计数
 * 参数：无
 * 返回：无
 * 调用：调试/诊断时手工调用；正常运行流程不调用
 * 注意：若在运行中调用，Work_CountTask 的 s_last_cnt 会与新计数不一致，
 *       可能产生一次“虚假增量”，建议仅在停机状态下使用
 */
void DI_ClearCounters(void)
{
    uint8_t i;

    for (i = 0; i < DI_NUM; i++)
    {
        g_cfg.di_rise_cnt[i] = 0U;
    }
}
