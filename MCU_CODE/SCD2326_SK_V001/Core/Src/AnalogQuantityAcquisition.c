/* 模拟量（电流）采集
 *
 * 模块职责：
 *   1) ADC1 扫描 + DMA1_Channel1 循环搬运，取得两路电流传感器的原始码值；
 *   2) 主循环 1ms 取一点，累计 16 点做中值滤波，抑制电机 PWM 开关噪声；
 *   3) 上电时在电机停止状态下做零点校准，扣除传感器零点偏置；
 *   4) 按 40mV/A 换算成安培，并跟踪绝对值峰值，供 Work_Task 做堵转/过流判定。
 *
 * 硬件：
 *   PA0 = MOTOR_ALL（U13 / CC6920SO-50A，串在 V 相，测总电流）
 *   PA1 = MOTOR_U  （U6  / CC6920SO-50A，串在 U 相）
 *   ADC1 规则组扫描：rank1=PA0、rank2=PA1，采样时间 55.5 周期；
 *   DMA1_Channel1 循环模式，目标为 ad.raw[2]（半字）。
 *
 * 关键参数与时序：
 *   3.3V 参考、12 位 ADC -> 0.8057mV/LSB；CC6920 灵敏度 40mV/A，零点约 VCC/2；
 *   AD_NUM = 16：中值滤波窗口（约 16ms），AD_ZERO_NUM = 32：上电零点平均（约 32ms）；
 *   主循环节拍 1ms，实际采样周期受主循环负载影响，用于趋势判断已足够。
 *
 * 运行上下文：
 *   AD_Init：上电初始化（main.c，Motor_Init 之后，必须电机停转）；
 *   AD_Task：主循环 1ms 任务；AD_GetXxx 供 Work_Task 等主循环代码读取。
 */
#include "AnalogQuantityAcquisition.h"
#include "adc.h"

AD_t ad;    /* 全局采集实例：raw 由 DMA 写入，其余字段由 AD_Task 维护 */

/* 冒泡排序（升序）
 * 参数：buf = 待排序数组；num = 元素个数（1 ~ AD_NUM）
 * 返回：无
 * 调用：AD_Task 中值滤波前调用（主循环）
 * 注意：只有 16 个样本，O(n²) 开销可忽略；原地排序会打乱原顺序，
 *       调用者先把样本拷到 tmp 再排序，避免破坏滑动窗口。
 */
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

/* 零点校准：电机必须处于停止状态，否则零点会偏
 * 参数：无
 * 返回：无
 * 调用：AD_Init 内、DMA 启动之后调用（上电初始化，电机未通电）
 * 注意：连续读 32 次 DMA 最新值求平均，每次间隔 1ms，共阻塞约 32ms；
 *       零点以 mV 保存，后续电流 = (当前 mV - 零点 mV) / 灵敏度。
 */
void AD_CalibrateZero(void)
{
    uint8_t  ch;
    uint8_t  i;
    uint32_t sum[AD_CH] = {0};

    for (i = 0; i < AD_ZERO_NUM; i++)
    {
        for (ch = 0; ch < AD_CH; ch++)
        {
            sum[ch] += ad.raw[ch];   /* DMA 每完成一轮扫描就更新 raw[]，这里取最新值 */
        }
        HAL_Delay(1);                /* 1ms 一个样本，等 ADC/DMA 刷新 */
    }

    for (ch = 0; ch < AD_CH; ch++)
    {
        /* 原始码值求平均后换算成 mV，作为零电流基准 */
        ad.zero_mv[ch] = ((float)sum[ch] / (float)AD_ZERO_NUM) * AD_MV_PER_LSB;
    }
}

/* 初始化：ADC 校准 -> 启动 DMA -> 零电流零点校准
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main.c，Motor_Init 之后；此时电机必须已停止、无 PWM 输出）
 * 注意：HAL_ADCEx_Calibration_Start 必须在 ADC 启动前执行，否则零点误差偏大；
 *       零点校准阻塞约 32ms，会推迟后续初始化（如 TIM2 中断启动）。
 */
void AD_Init(void)
{
    uint8_t ch;

    ad.count = 0;
    ad.task_tick = HAL_GetTick();   /* 让 1ms 任务从当前时刻开始计时 */

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ad.raw, AD_CH);   /* 循环模式，DMA 自动覆盖 raw[] */

    AD_CalibrateZero();

    for (ch = 0; ch < AD_CH; ch++)
    {
        ad.filt[ch] = 0U;
        ad.mv[ch] = 0.0f;
        ad.I[ch] = 0.0f;
        ad.peak[ch] = 0.0f;
    }
}

/* 1ms 周期任务：采样 -> 中值滤波 -> 换算电流 -> 峰值跟踪
 * 参数：无
 * 返回：无
 * 调用：主循环（main.c while(1)，每轮调用）
 * 注意：用 HAL_GetTick 做节拍，同一毫秒内不重复采样；
 *       每 AD_NUM(16) 个点才更新一次 ad.I[]，即电流刷新周期约 16ms；
 *       中值滤波对窄脉冲不敏感，适合 PWM 驱动下的电流采样。
 */
void AD_Task(void)
{
    uint8_t  ch;
    uint8_t  i;
    uint32_t now = HAL_GetTick();
    uint16_t tmp[AD_NUM];

    if (now == ad.task_tick) { return; }   /* 同一毫秒内不重复采样 */
    ad.task_tick = now;

    /* 1. 采样 */
    for (ch = 0; ch < AD_CH; ch++)
    {
        ad.buf[ch][ad.count] = ad.raw[ch];   /* 两通道取自同一轮 DMA 扫描 */
    }

    if (++ad.count < AD_NUM) { return; }   /* 样本未满，等下一次 */
    ad.count = 0;

    /* 2. 中值滤波 + 换算 */
    for (ch = 0; ch < AD_CH; ch++)
    {
        for (i = 0; i < AD_NUM; i++)
        {
            tmp[i] = ad.buf[ch][i];   /* 拷贝后再排序，保留滑动窗口原顺序 */
        }

        AD_Sort(tmp, AD_NUM);

        ad.filt[ch] = tmp[AD_NUM / 2U];   /* 16 点取第 8 个（上中位）作为中值 */
        ad.mv[ch]   = (float)ad.filt[ch] * AD_MV_PER_LSB;
        ad.I[ch]    = (ad.mv[ch] - ad.zero_mv[ch]) / CC6920_SENS_MV_PER_A;   /* 40mV/A，结果有符号 */

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

/* 读取当前电流
 * 参数：index = AD_INDEX_ALL(0) 总电流 / AD_INDEX_U(1) U 相电流
 * 返回：电流（A，有符号，正负对应电机转向），index 越界返回 0
 * 调用：Work_Task 堵转/过流判定、菜单显示（主循环）
 * 注意：返回的是最近一次滤波结果，刷新周期约 16ms，不要用于高速闭环。
 */
float AD_GetCurrent(uint8_t index)
{
    if (index >= AD_CH) { return 0.0f; }
    return ad.I[index];
}

/* 读取当前采样电压
 * 参数：index = AD_INDEX_ALL / AD_INDEX_U；越界返回 0
 * 返回：传感器输出电压（mV），含零点偏置
 * 调用：调试观察、自检（主循环）
 * 注意：该值是“绝对电压”，判断电流要用 AD_GetCurrent（已扣零点）。
 */
float AD_GetVoltage(uint8_t index)
{
    if (index >= AD_CH) { return 0.0f; }
    return ad.mv[index];
}

/* 读取电流峰值
 * 参数：index = AD_INDEX_ALL / AD_INDEX_U；越界返回 0
 * 返回：自上次清零以来的最大电流绝对值（A）
 * 调用：调试观察、过流事件记录（主循环）
 * 注意：峰值只增不减，需显式调用 AD_ClearPeak 清零。
 */
float AD_GetPeak(uint8_t index)
{
    if (index >= AD_CH) { return 0.0f; }
    return ad.peak[index];
}

/* 清零峰值，index = 0xFF 表示全部
 * 参数：index = 0 ~ AD_CH-1 指定通道，0xFF 清全部
 * 返回：无
 * 调用：动作开始前/结束后由业务代码调用（主循环）
 * 注意：其它越界值不会匹配任何通道，等效于什么都不做。
 */
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
