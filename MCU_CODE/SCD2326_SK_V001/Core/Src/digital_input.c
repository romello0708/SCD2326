/* 10 路数字输入扫描
 * 硬件（均为高电平有效，GPIO 已在 MX_GPIO_Init 中配置为输入）：
 *   IN_1  PA15   IN_2  PB3    IN_3  PB4    IN_4  PB5    IN_5  PB6
 *   IN_6  PB7    IN_7  PB1    IN_8  PA6    IN_9  PA5    IN_10 PA4
 * 扫描：由 TIM2 1ms 更新中断调用，连续 DI_FILTER_CNT 次(3ms)电平一致才更新稳定值，
 *       抖动期间计数器清零重新计时，抑制触点/传感器抖动造成的误计数。
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

static uint8_t  s_stable[DI_NUM];
static uint8_t  s_filter[DI_NUM];

void DI_Init(void)
{
    uint8_t i;

    for (i = 0; i < DI_NUM; i++)
    {
        s_stable[i] = 0U;
        s_filter[i] = 0U;
    }
}

/* TIM2 1ms 中断内调用：采样 + 消抖 + 稳定上升沿计数 */
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

        if (raw == s_stable[i])
        {
            s_filter[i] = 0U;
            continue;
        }

        if (++s_filter[i] < DI_FILTER_CNT)
        {
            continue;
        }

        s_filter[i] = 0U;
        s_stable[i] = raw;

        if (raw != 0U)
        {
            g_cfg.di_rise_cnt[i]++;          /* 只在稳定上升沿计数 */
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
