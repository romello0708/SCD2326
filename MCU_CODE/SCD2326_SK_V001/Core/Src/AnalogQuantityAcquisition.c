/* 模拟量（电流）采集
 * 硬件：PA0 = MOTOR_ALL（U13 / CC6920SO-50A，串在 V 相）
 *       PA1 = MOTOR_U  （U6  / CC6920SO-50A，串在 U 相）
 * ADC1 扫描 + DMA1_Channel1 循环搬运，1ms 取一个点，16 点中值滤波后换算成电流
 */
#include "AnalogQuantityAcquisition.h"
#include "adc.h"

AD_t ad;

/* 冒泡排序（样本数很少，直接排序取中值） */
static void AD_Sort(uint16_t *buf, uint8_t num)
{
    uint8_t i, j;
    uint16_t temp;

    for (j = 0; j < num; j++)
    {
        for (i = 0; i < (uint8_t)(num - 1U - j); i++)
        {
            if (buf[i] > buf[i + 1U])
            {
                temp = buf[i];
                buf[i] = buf[i + 1U];
                buf[i + 1U] = temp;
            }
        }
    }
}

/* 零点校准：电机必须处于停止状态，否则零点会偏 */
void AD_CalibrateZero(void)
{
    uint8_t  ch;
    uint8_t  i;
    uint32_t sum[AD_CH] = {0};

    for (i = 0; i < AD_ZERO_NUM; i++)
    {
        for (ch = 0; ch < AD_CH; ch++)
        {
            sum[ch] += ad.raw[ch];
        }
        HAL_Delay(1);
    }

    for (ch = 0; ch < AD_CH; ch++)
    {
        ad.zero_mv[ch] = ((float)sum[ch] / (float)AD_ZERO_NUM) * AD_MV_PER_LSB;
    }
}

/* 初始化：ADC 校准 -> 启动 DMA -> 零电流零点校准 */
void AD_Init(void)
{
    uint8_t ch;

    ad.count = 0;
    ad.task_tick = HAL_GetTick();

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad.raw, AD_CH);

    AD_CalibrateZero();

    for (ch = 0; ch < AD_CH; ch++)
    {
        ad.filt[ch] = 0U;
        ad.mv[ch] = 0.0f;
        ad.I[ch] = 0.0f;
        ad.peak[ch] = 0.0f;
    }
}

/* 1ms 周期任务，放在主循环中调用 */
void AD_Task(void)
{
    uint8_t  ch;
    uint8_t  i;
    uint32_t now = HAL_GetTick();
    uint16_t tmp[AD_NUM];

    if (now == ad.task_tick) { return; }
    ad.task_tick = now;

    /* 1. 采样 */
    for (ch = 0; ch < AD_CH; ch++)
    {
        ad.buf[ch][ad.count] = ad.raw[ch];
    }

    if (++ad.count < AD_NUM) { return; }
    ad.count = 0;

    /* 2. 中值滤波 + 换算 */
    for (ch = 0; ch < AD_CH; ch++)
    {
        for (i = 0; i < AD_NUM; i++)
        {
            tmp[i] = ad.buf[ch][i];
        }

        AD_Sort(tmp, AD_NUM);

        ad.filt[ch] = tmp[AD_NUM / 2U];
        ad.mv[ch]   = (float)ad.filt[ch] * AD_MV_PER_LSB;
        ad.I[ch]    = (ad.mv[ch] - ad.zero_mv[ch]) / CC6920_SENS_MV_PER_A;

        /* 峰值跟踪（取绝对值） */
        {
            float mag = (ad.I[ch] >= 0.0f) ? ad.I[ch] : -ad.I[ch];
            if (mag > ad.peak[ch])
            {
                ad.peak[ch] = mag;
            }
        }
    }
}

/* 当前电流(A)，index: AD_INDEX_ALL / AD_INDEX_U */
float AD_GetCurrent(uint8_t index)
{
    if (index >= AD_CH) { return 0.0f; }
    return ad.I[index];
}

/* 当前采样电压(mV) */
float AD_GetVoltage(uint8_t index)
{
    if (index >= AD_CH) { return 0.0f; }
    return ad.mv[index];
}

/* 电流峰值(A) */
float AD_GetPeak(uint8_t index)
{
    if (index >= AD_CH) { return 0.0f; }
    return ad.peak[index];
}

/* 清零峰值，index = 0xFF 表示全部 */
void AD_ClearPeak(uint8_t index)
{
    uint8_t ch;

    for (ch = 0; ch < AD_CH; ch++)
    {
        if ((index == 0xFFU) || (index == ch))
        {
            ad.peak[ch] = 0.0f;
        }
    }
}
