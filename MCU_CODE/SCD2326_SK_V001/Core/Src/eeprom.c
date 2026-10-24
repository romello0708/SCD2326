/* BL24C128A-NTRC EEPROM 操作（16KB，I2C2，地址 0x50）
 * 硬件：PB10=SCL PB11=SDA，A2/A1/A0 接地，WP 接地（可写）
 * 说明：页大小 64 字节，写操作按页拆分并等待内部写周期结束；
 *       读操作直接使用 16 位地址的连续读。
 */
#include "eeprom.h"
#include "i2c.h"
#include "global.h"
#include <string.h>

/* 按 100kHz 粗略估算传输时间，给足余量 */
#define EEPROM_TIMEOUT(len)   (20U + ((uint32_t)(len) * 10U) / 8U)

/* 等待器件内部写周期结束（应答轮询） */
static HAL_StatusTypeDef EEPROM_WaitReady(void)
{
    uint8_t i;

    for (i = 0U; i < EEPROM_WRITE_TIMEOUT; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_I2C_ADDR, 1U, 5U) == HAL_OK)
        {
            return HAL_OK;
        }
        HAL_Delay(1);
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef EEPROM_IsReady(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_I2C_ADDR, 2U, 20U);
}

HAL_StatusTypeDef EEPROM_Init(void)
{
    return EEPROM_IsReady();
}

HAL_StatusTypeDef EEPROM_Read(uint16_t addr, uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    if (((uint32_t)addr + (uint32_t)len) > EEPROM_SIZE)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2, EEPROM_I2C_ADDR, addr, I2C_MEMADD_SIZE_16BIT,
                            buf, len, EEPROM_TIMEOUT(len));
}

HAL_StatusTypeDef EEPROM_Write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t done = 0U;
    uint16_t cur;
    uint16_t page_left;
    uint16_t chunk;
    HAL_StatusTypeDef st;

    if ((buf == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    if (((uint32_t)addr + (uint32_t)len) > EEPROM_SIZE)
    {
        return HAL_ERROR;
    }

    while (done < len)
    {
        cur = (uint16_t)(addr + done);
        page_left = (uint16_t)(EEPROM_PAGE_SIZE - (cur % EEPROM_PAGE_SIZE));
        chunk = (uint16_t)(len - done);
        if (chunk > page_left)
        {
            chunk = page_left;          /* 不能跨页写 */
        }

        st = HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR, cur, I2C_MEMADD_SIZE_16BIT,
                               (uint8_t *)&buf[done], chunk, EEPROM_TIMEOUT(chunk));
        if (st != HAL_OK)
        {
            return st;
        }

        st = EEPROM_WaitReady();
        if (st != HAL_OK)
        {
            return st;
        }

        done = (uint16_t)(done + chunk);
    }

    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *val)
{
    return EEPROM_Read(addr, val, 1U);
}

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t val)
{
    return EEPROM_Write(addr, &val, 1U);
}

HAL_StatusTypeDef EEPROM_ReadU16(uint16_t addr, uint16_t *val)
{
    uint8_t b[2];
    HAL_StatusTypeDef st;

    if (val == NULL)
    {
        return HAL_ERROR;
    }

    st = EEPROM_Read(addr, b, 2U);
    if (st == HAL_OK)
    {
        *val = (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    }

    return st;
}

HAL_StatusTypeDef EEPROM_WriteU16(uint16_t addr, uint16_t val)
{
    uint8_t b[2];

    b[0] = (uint8_t)(val & 0xFFU);
    b[1] = (uint8_t)(val >> 8);

    return EEPROM_Write(addr, b, 2U);
}

HAL_StatusTypeDef EEPROM_ReadU32(uint16_t addr, uint32_t *val)
{
    uint8_t b[4];
    HAL_StatusTypeDef st;

    if (val == NULL)
    {
        return HAL_ERROR;
    }

    st = EEPROM_Read(addr, b, 4U);
    if (st == HAL_OK)
    {
        *val = ((uint32_t)b[0]) | ((uint32_t)b[1] << 8) |
               ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    }

    return st;
}

HAL_StatusTypeDef EEPROM_WriteU32(uint16_t addr, uint32_t val)
{
    uint8_t b[4];

    b[0] = (uint8_t)(val & 0xFFU);
    b[1] = (uint8_t)((val >> 8) & 0xFFU);
    b[2] = (uint8_t)((val >> 16) & 0xFFU);
    b[3] = (uint8_t)((val >> 24) & 0xFFU);

    return EEPROM_Write(addr, b, 4U);
}

/* 读写自检：在测试区写固定图案再读回比对 */
HAL_StatusTypeDef EEPROM_SelfTest(void)
{
    static const uint8_t pattern[4] = { 0xA5U, 0x5AU, 0x12U, 0x34U };
    uint8_t rd[4];
    HAL_StatusTypeDef st;

    st = EEPROM_Write(EEPROM_TEST_ADDR, pattern, sizeof(pattern));
    if (st != HAL_OK)
    {
        return st;
    }

    st = EEPROM_Read(EEPROM_TEST_ADDR, rd, sizeof(rd));
    if (st != HAL_OK)
    {
        return st;
    }

    if ((rd[0] != pattern[0]) || (rd[1] != pattern[1]) ||
        (rd[2] != pattern[2]) || (rd[3] != pattern[3]))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* 保存当前配置到 EEPROM */
HAL_StatusTypeDef EEPROM_SaveConfig(void)
{
    EEPROM_Config_t cfg;
    uint8_t *p = (uint8_t *)&cfg;
    uint16_t sum = 0U;
    uint16_t i;

    memset(&cfg, 0, sizeof(cfg));

    cfg.magic = EEPROM_CFG_MAGIC;
    cfg.motor_duty_pct = g_cfg.motor_duty_pct;
    cfg.over_current_a = g_cfg.over_current_a;
    cfg.echo_enable = g_cfg.echo_enable;
    cfg.status_print_enable = g_cfg.status_print_enable;
    cfg.sensor1_type = g_cfg.sensor1_type;
    cfg.sensor2_type = g_cfg.sensor2_type;
    cfg.fire_mode = g_cfg.fire_mode;
    cfg.module_type = g_cfg.module_type;
    cfg.load_dir = g_cfg.load_dir;
    cfg.ammo = g_cfg.ammo;
    cfg.ammo_target = g_cfg.ammo_target;
    cfg.shot_total = g_cfg.shot_total;
    cfg.pulse_ms = g_cfg.pulse_ms;
    cfg.gap_ms = g_cfg.gap_ms;
    cfg.burst_shots = g_cfg.burst_shots;
    cfg.long_burst_shots = g_cfg.long_burst_shots;
    cfg.baudrate = g_cfg.baudrate;
    cfg.check = 0U;

    for (i = 0U; i < (uint16_t)(sizeof(cfg) - sizeof(cfg.check)); i++)
    {
        sum = (uint16_t)(sum + p[i]);
    }
    cfg.check = sum;

    return EEPROM_Write(EEPROM_CFG_ADDR, p, (uint16_t)sizeof(cfg));
}

/* 从 EEPROM 读取配置，校验通过后覆盖当前配置 */
HAL_StatusTypeDef EEPROM_LoadConfig(void)
{
    EEPROM_Config_t cfg;
    uint8_t *p = (uint8_t *)&cfg;
    uint16_t sum = 0U;
    uint16_t i;
    HAL_StatusTypeDef st;

    st = EEPROM_Read(EEPROM_CFG_ADDR, p, (uint16_t)sizeof(cfg));
    if (st != HAL_OK)
    {
        return st;
    }

    if (cfg.magic != EEPROM_CFG_MAGIC)
    {
        return HAL_ERROR;               /* 未保存过 */
    }

    for (i = 0U; i < (uint16_t)(sizeof(cfg) - sizeof(cfg.check)); i++)
    {
        sum = (uint16_t)(sum + p[i]);
    }

    if (sum != cfg.check)
    {
        return HAL_ERROR;               /* 数据损坏 */
    }

    g_cfg.motor_duty_pct = cfg.motor_duty_pct;
    g_cfg.over_current_a = cfg.over_current_a;
    g_cfg.echo_enable = cfg.echo_enable;
    g_cfg.status_print_enable = cfg.status_print_enable;
    g_cfg.sensor1_type = (cfg.sensor1_type == 1U) ? 1U : 0U;
    g_cfg.sensor2_type = (cfg.sensor2_type == 1U) ? 1U : 0U;
    g_cfg.fire_mode = (cfg.fire_mode < FIRE_MODE_NUM) ? cfg.fire_mode : FIRE_MODE_SINGLE;
    g_cfg.module_type = (cfg.module_type < MODULE_NUM) ? cfg.module_type : MODULE_FIRE;
    g_cfg.load_dir = (cfg.load_dir == LOAD_DIR_REVERSE) ? LOAD_DIR_REVERSE : LOAD_DIR_FORWARD;
    g_cfg.ammo = (cfg.ammo > AMMO_MAX) ? AMMO_MAX : cfg.ammo;
    g_cfg.ammo_target = (cfg.ammo_target > AMMO_MAX) ? AMMO_MAX : cfg.ammo_target;
    g_cfg.shot_total = cfg.shot_total;

    /* 射击时序参数：越界时回落到默认值，避免异常配置导致动作异常 */
    g_cfg.pulse_ms = ((cfg.pulse_ms < SHOT_PULSE_MIN) || (cfg.pulse_ms > SHOT_PULSE_MAX)) ?
                     100U : cfg.pulse_ms;
    g_cfg.gap_ms = ((cfg.gap_ms < SHOT_GAP_MIN) || (cfg.gap_ms > SHOT_GAP_MAX)) ?
                   150U : cfg.gap_ms;
    g_cfg.burst_shots = ((cfg.burst_shots < BURST_SHOTS_MIN) || (cfg.burst_shots > BURST_SHOTS_MAX)) ?
                        3U : cfg.burst_shots;
    g_cfg.long_burst_shots = ((cfg.long_burst_shots < LONG_BURST_MIN) || (cfg.long_burst_shots > LONG_BURST_MAX)) ?
                             6U : cfg.long_burst_shots;

    g_cfg.baudrate = cfg.baudrate;

    return HAL_OK;
}

/* 清除配置（下次上电使用默认值） */
HAL_StatusTypeDef EEPROM_EraseConfig(void)
{
    return EEPROM_WriteU16(EEPROM_CFG_ADDR, 0x0000U);
}
