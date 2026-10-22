#ifndef __ANALOG_QUANTITY_ACQUISITION_H
#define __ANALOG_QUANTITY_ACQUISITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ADC1 扫描顺序：rank1 = PA0(MOTOR_ALL)，rank2 = PA1(MOTOR_U) */
#define AD_CH                 2U
#define AD_INDEX_ALL          0U   /* PA0：U13 电流传感器 */
#define AD_INDEX_U            1U   /* PA1：U6  电流传感器 */

#define AD_NUM                16U  /* 中值滤波样本数 */
#define AD_ZERO_NUM           32U  /* 上电零电流校准样本数 */

/* 采样换算：3.3V 参考、12 位 ADC */
#define AD_VREF_MV            3300.0f
#define AD_FULL_SCALE         4096.0f
#define AD_MV_PER_LSB         (AD_VREF_MV / AD_FULL_SCALE)   /* 0.8057 mV/LSB */

/* CC6920SO-50A：3.3V 供电，零点 VCC/2，灵敏度 40mV/A
 * 若实际贴装的是其它量程（30A=66mV/A、20A=100mV/A），改这里即可 */
#define CC6920_ZERO_MV        1650.0f
#define CC6920_SENS_MV_PER_A  40.0f

typedef struct
{
    volatile uint16_t raw[AD_CH];        /* DMA 搬运的最新原始值 */
    uint16_t buf[AD_CH][AD_NUM];         /* 滤波样本 */
    uint16_t filt[AD_CH];                /* 中值滤波后的原始值 */
    float    zero_mv[AD_CH];             /* 零电流时的零点电压(mV) */
    float    mv[AD_CH];                  /* 当前电压(mV) */
    float    I[AD_CH];                   /* 当前电流(A)，有符号 */
    float    peak[AD_CH];                /* 电流峰值(A) */
    uint8_t  count;                      /* 滤波样本计数 */
    uint32_t task_tick;                  /* 1ms 任务计时 */
} AD_t;

extern AD_t ad;

void  AD_Init(void);
void  AD_Task(void);
void  AD_CalibrateZero(void);
float AD_GetCurrent(uint8_t index);
float AD_GetVoltage(uint8_t index);
float AD_GetPeak(uint8_t index);
void  AD_ClearPeak(uint8_t index);       /* 0xFF = 全部清零 */

#ifdef __cplusplus
}
#endif

#endif /* __ANALOG_QUANTITY_ACQUISITION_H */
