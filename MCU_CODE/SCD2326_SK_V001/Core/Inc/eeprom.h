#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* BL24C128A-NTRC：128Kbit = 16KB，I2C2（PB10/PB11），器件地址 0x50（A2/A1/A0 接地）
 * 页大小 64 字节，字节写周期最大 5ms，地址为 16 位 */
#define EEPROM_I2C_ADDR        (0x50U << 1)
#define EEPROM_PAGE_SIZE       64U
#define EEPROM_SIZE            16384U
#define EEPROM_WRITE_TIMEOUT   20U

#define EEPROM_CFG_ADDR        0x0000U    /* 系统配置区 */
#define EEPROM_TEST_ADDR       0x3F00U    /* 自检区（最后一个页区） */
#define EEPROM_CFG_MAGIC       0x5AAEU    /* 结构变更时递增，旧数据自动失效 */

/* 掉电保存的系统配置（字段顺序已按对齐排布，避免填充字节影响校验和） */
typedef struct
{
    uint32_t baudrate;
    uint16_t magic;
    uint8_t  motor_duty_pct;
    uint8_t  over_current_a;
    uint8_t  echo_enable;
    uint8_t  status_print_enable;
    uint8_t  sensor1_type;                 /* 传感器1：0 = NPN，1 = PNP */
    uint8_t  sensor2_type;                 /* 传感器2：0 = NPN，1 = PNP */
    uint8_t  fire_mode;                    /* 射击模式 FIRE_MODE_* */
    uint8_t  module_type;                  /* 模块型号 MODULE_* */
    uint8_t  load_dir;                     /* 装填电机方向：0 = 正向，1 = 反向 */
    uint16_t ammo;                         /* 保留字段：余弹数不再掉电保存，固定写 0 */
    uint16_t ammo_target;                  /* 保留字段：装填目标值已取消，固定写 0 */
    uint16_t shot_total;                   /* 保留字段：累计射击次数不再保存，固定写 0 */
    uint16_t pulse_ms;                     /* 单发脉冲时长 ms */
    uint16_t gap_ms;                       /* 双发间隔时长 ms */
    uint8_t  burst_shots;                  /* 点射用弹数 */
    uint8_t  long_burst_shots;             /* 长点射用弹数 */
    uint16_t check;                        /* 前面所有字节的累加和 */
} EEPROM_Config_t;

HAL_StatusTypeDef EEPROM_Init(void);
HAL_StatusTypeDef EEPROM_IsReady(void);

HAL_StatusTypeDef EEPROM_Read(uint16_t addr, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef EEPROM_Write(uint16_t addr, const uint8_t *buf, uint16_t len);
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *val);
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t val);
HAL_StatusTypeDef EEPROM_ReadU16(uint16_t addr, uint16_t *val);
HAL_StatusTypeDef EEPROM_WriteU16(uint16_t addr, uint16_t val);
HAL_StatusTypeDef EEPROM_ReadU32(uint16_t addr, uint32_t *val);
HAL_StatusTypeDef EEPROM_WriteU32(uint16_t addr, uint32_t val);

HAL_StatusTypeDef EEPROM_SelfTest(void);
HAL_StatusTypeDef EEPROM_SaveConfig(void);
HAL_StatusTypeDef EEPROM_LoadConfig(void);
HAL_StatusTypeDef EEPROM_EraseConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
