#ifndef __ANALOG_QUANTITY_ACQUISITION_H
#define __ANALOG_QUANTITY_ACQUISITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：AnalogQuantityAcquisition.h —— 电机电流采集接口
 *
 * 模块职责：ADC1 + DMA 连续采样两路电流传感器，中值滤波、零点校准、mV->A 换算与峰值跟踪。
 * 硬件：PA0 = MOTOR_ALL（U13 / CC6920SO-50A，串在 V 相，测总电流）；
 *       PA1 = MOTOR_U  （U6  / CC6920SO-50A，串在 U 相）；
 *       ADC1 规则组扫描 rank1/rank2，DMA1_Channel1 循环搬运。
 * 关键参数：3.3V/12 位（0.8057mV/LSB），CC6920 灵敏度 40mV/A，零点约 VCC/2；
 *           中值滤波 16 点、零点校准 32 点；电流刷新周期约 16ms。
 * 运行上下文：AD_Init 在上电初始化（电机必须停转）；AD_Task 在主循环 1ms 调用；
 *           AD_GetXxx 供主循环业务读取。
 */

/* ADC1 扫描顺序：rank1 = PA0(MOTOR_ALL)，rank2 = PA1(MOTOR_U) */
#define AD_CH                 2U
#define AD_INDEX_ALL          0U   /* PA0：U13 电流传感器（V 相，总电流） */
#define AD_INDEX_U            1U   /* PA1：U6  电流传感器（U 相） */

#define AD_NUM                16U  /* 中值滤波样本数：16 点约 16ms 窗口，兼顾响应与抗噪 */
#define AD_ZERO_NUM           32U  /* 上电零电流校准样本数：32 点平均，约阻塞 32ms */

/* 采样换算：3.3V 参考、12 位 ADC */
#define AD_VREF_MV            3300.0f
#define AD_FULL_SCALE         4096.0f
#define AD_MV_PER_LSB         (AD_VREF_MV / AD_FULL_SCALE)   /* 0.8057 mV/LSB */

/* CC6920SO-50A：3.3V 供电，零点 VCC/2，灵敏度 40mV/A
 * 若实际贴装的是其它量程（30A=66mV/A、20A=100mV/A），改这里即可 */
#define CC6920_ZERO_MV        1650.0f     /* 理论零点，仅作参考；实际使用上电校准值 zero_mv */
#define CC6920_SENS_MV_PER_A  40.0f       /* 灵敏度：每安培对应 40mV */

typedef struct
{
    volatile uint16_t raw[AD_CH];        /* DMA 搬运的最新原始值（0~4095），DMA 随时更新 */
    uint16_t buf[AD_CH][AD_NUM];         /* 滤波样本滑动窗口，count 指向下一个写入位置 */
    uint16_t filt[AD_CH];                /* 中值滤波后的原始值（0~4095） */
    float    zero_mv[AD_CH];             /* 零电流时的零点电压(mV)，上电校准，运行中不变 */
    float    mv[AD_CH];                  /* 当前电压(mV)，含零点偏置 */
    float    I[AD_CH];                   /* 当前电流(A)，有符号（正负对应电机转向） */
    float    peak[AD_CH];                /* 电流峰值(A)，绝对值，只增不减直到清零 */
    uint8_t  count;                      /* 滤波样本计数 0 ~ AD_NUM-1 */
    uint32_t task_tick;                  /* 1ms 任务基准时刻（HAL_GetTick） */
} AD_t;

extern AD_t ad;   /* 全局采集数据，定义在 AnalogQuantityAcquisition.c */

void  AD_Init(void);                     /* 上电：ADC 校准 + 启动 DMA + 零点校准（电机须停转） */
void  AD_Task(void);                     /* 主循环 1ms：采样、中值滤波、换算、峰值跟踪 */
void  AD_CalibrateZero(void);            /* 重新做零点校准（电机必须停止，阻塞约 32ms） */
float AD_GetCurrent(uint8_t index);      /* 当前电流 A，index 越界返回 0 */
float AD_GetVoltage(uint8_t index);      /* 当前电压 mV，index 越界返回 0 */
float AD_GetPeak(uint8_t index);         /* 峰值电流 A，index 越界返回 0 */
void  AD_ClearPeak(uint8_t index);       /* 0xFF = 全部清零，0/1 = 指定通道 */

#ifdef __cplusplus
}
#endif

#endif /* __ANALOG_QUANTITY_ACQUISITION_H */
