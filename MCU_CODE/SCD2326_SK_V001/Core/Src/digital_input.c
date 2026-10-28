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
 *   代价：从电平稳定变化到输出翻转需要 DI_DEBOUNCE_CNT ms 的积分时间。
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
static uint8_t  s_integ[DI_NUM];    /* 积分值 0 ~ DI_DEBOUNCE_CNT */

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
        s_integ[i]  = (raw != 0U) ? DI_DEBOUNCE_CNT : 0U;

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

/* TIM2 1ms 中断内调用：采样 + 积分式消抖 + 稳定上升沿计数 */
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

        /* 积分：高电平累加、低电平递减，两端饱和 */
        if (raw != 0U)
        {
            if (s_integ[i] < DI_DEBOUNCE_CNT)
            {
                s_integ[i]++;
            }
        }
        else if (s_integ[i] > 0U)
        {
            s_integ[i]--;
        }

        /* 只有积分到达两端才翻转稳定值；中间区间保持原状态（滞回） */
        if ((s_stable[i] == 0U) && (s_integ[i] >= DI_DEBOUNCE_CNT))
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

uint8_t DI_IsHigh(uint8_t ch)
{
    if ((ch < 1U) || (ch > DI_NUM))
    {
        return 0U;
    }

    return s_stable[ch - 1U];
}

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

uint16_t DI_GetState(void)
{
    return g_cfg.di_state;
}

uint16_t DI_GetRaw(void)
{
    return g_cfg.di_raw;
}

uint16_t DI_GetRiseCount(uint8_t ch)
{
    if ((ch < 1U) || (ch > DI_NUM))
    {
        return 0U;
    }

    return g_cfg.di_rise_cnt[ch - 1U];
}

void DI_ClearCounters(void)
{
    uint8_t i;

    for (i = 0; i < DI_NUM; i++)
    {
        g_cfg.di_rise_cnt[i] = 0U;
    }
}
