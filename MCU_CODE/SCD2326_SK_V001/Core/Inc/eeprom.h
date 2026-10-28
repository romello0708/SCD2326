#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：eeprom.h —— BL24C128A-NTRC（16KB，I2C2）驱动与掉电配置接口
 *
 * 模块职责：16 位地址的字节/块读写（自动按 64 字节页拆分）、配置结构体存取与校验、自检。
 * 硬件：I2C2（PB10=SCL / PB11=SDA，100kHz 标准模式），器件地址 0x50（A2/A1/A0 接地、
 *       WP 接地可写）。
 * 关键参数：页 64 字节，字节写周期最大 5ms；配置区从 0x0000 开始，自检区在 0x3F00。
 * 运行上下文：全部为阻塞式 I2C 操作，只能在上电初始化或主循环调用，禁止在中断中使用。
 * 注意：结构体增删字段或调整顺序时，必须递增 EEPROM_CFG_MAGIC，让旧数据自动失效。
 */

/* BL24C128A-NTRC：128Kbit = 16KB，I2C2（PB10/PB11），器件地址 0x50（A2/A1/A0 接地）
 * 页大小 64 字节，字节写周期最大 5ms，地址为 16 位 */
#define EEPROM_I2C_ADDR        (0x50U << 1)   /* 7 位地址左移 1 位，得到 HAL 使用的 8 位地址 0xA0 */
#define EEPROM_PAGE_SIZE       64U            /* 页写边界：一次最多 64 字节，跨页会回卷覆盖页首 */
#define EEPROM_SIZE            16384U         /* 总容量 16KB，合法地址 0x0000 ~ 0x3FFF */
#define EEPROM_WRITE_TIMEOUT   20U            /* 写周期轮询次数（每次 1ms），覆盖 5ms 写周期并留余量 */

#define EEPROM_CFG_ADDR        0x0000U    /* 系统配置区 */
#define EEPROM_TEST_ADDR       0x3F00U    /* 自检区（最后一个页区） */
#define EEPROM_CFG_MAGIC       0x5AAEU    /* 结构变更时递增，旧数据自动失效 */

/* 掉电保存的系统配置（字段顺序已按对齐排布，避免填充字节影响校验和）
 * 注意：保存前会 memset 清零，填充字节确定；校验只累加 check 之前的字节。 */
typedef struct
{
    uint32_t baudrate;                     /* 串口波特率，上电后由 Comm_SetBaud 应用，默认 115200 */
    uint16_t magic;                        /* 魔数 EEPROM_CFG_MAGIC，用于判断是否保存过 */
    uint8_t  motor_duty_pct;               /* 保留字段：电机占空比已固定 90%，不再是设置项 */
    uint8_t  over_current_a;               /* 过流/堵转判定阈值 A，默认 20 */
    uint8_t  echo_enable;                  /* 1 = 串口收到的帧原样回传 */
    uint8_t  status_print_enable;          /* 1 = 每 500ms 打印一帧输入状态 */
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

HAL_StatusTypeDef EEPROM_Init(void);       /* 上电探测器件是否在线（仅 ACK 检测，不格式化） */
HAL_StatusTypeDef EEPROM_IsReady(void);    /* 只发器件地址看是否应答，可用于随时探测 */

HAL_StatusTypeDef EEPROM_Read(uint16_t addr, uint8_t *buf, uint16_t len);            /* 16 位地址连续读，无页限制 */
HAL_StatusTypeDef EEPROM_Write(uint16_t addr, const uint8_t *buf, uint16_t len);     /* 自动按 64 字节分页，页间等写周期 */
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *val);                      /* 读 1 字节 */
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t val);                      /* 写 1 字节（约 5ms） */
HAL_StatusTypeDef EEPROM_ReadU16(uint16_t addr, uint16_t *val);                      /* 读 16 位小端 */
HAL_StatusTypeDef EEPROM_WriteU16(uint16_t addr, uint16_t val);                      /* 写 16 位小端 */
HAL_StatusTypeDef EEPROM_ReadU32(uint16_t addr, uint32_t *val);                      /* 读 32 位小端 */
HAL_StatusTypeDef EEPROM_WriteU32(uint16_t addr, uint32_t val);                      /* 写 32 位小端 */

HAL_StatusTypeDef EEPROM_SelfTest(void);   /* 测试区写固定图案再读回比对 */
HAL_StatusTypeDef EEPROM_SaveConfig(void); /* 把 g_cfg 中可保存字段写入配置区（阻塞约 5~10ms） */
HAL_StatusTypeDef EEPROM_LoadConfig(void); /* 校验魔数 + 累加和后回填 g_cfg，越界项回落默认值 */
HAL_StatusTypeDef EEPROM_EraseConfig(void);/* 魔数写 0，下次上电使用默认参数 */

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
