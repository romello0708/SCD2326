/* BL24C128A-NTRC EEPROM 操作（16KB，I2C2，地址 0x50）
 *
 * 模块职责：
 *   1) 提供 16 位地址的字节/半字/字/块读写，按 64 字节页自动拆分，保证页写不跨页；
 *   2) 掉电保存系统配置（EEPROM_Config_t），用魔数 + 累加和判断数据是否有效；
 *   3) 提供上电自检（固定图案写读比对）与配置擦除。
 * 说明：页大小 64 字节，写操作按页拆分并等待内部写周期结束；读操作直接使用 16 位地址的连续读。
 *
 * 硬件：
 *   BL24C128A-NTRC：128Kbit = 16384 字节，I2C2（PB10=SCL、PB11=SDA，100kHz 标准模式），
 *   A2/A1/A0 接地 -> 器件地址 0x50（HAL 使用 8 位地址 0xA0），WP 接地（可写）；
 *   页大小 64 字节，字节写周期最大 5ms。
 *
 * 关键参数与时序：
 *   EEPROM_TIMEOUT(len) = 20 + len*10/8 ms，相对 100kHz 下约 0.1ms/字节留了 10 倍以上余量；
 *   连续写必须“写一页 -> 等内部写周期结束 -> 再写下一页”，否则器件会丢弃后续数据；
 *   所有写操作均为阻塞式（HAL_Delay 轮询），只能在主循环/初始化上下文调用。
 *
 * 运行上下文：
 *   上电初始化（main.c）：EEPROM_Init、EEPROM_LoadConfig，在挂 TIM2 中断之前；
 *   菜单保存参数（menu.c）：EEPROM_SaveConfig，主循环 Menu_Task 内。
 *   不使用中断，也不可在中断内调用（内部有 HAL_Delay）。
 */
#include "eeprom.h"
#include "i2c.h"
#include "global.h"
#include <string.h>
#include <stddef.h>

/* 按 100kHz 粗略估算传输时间，给足余量 */
#define EEPROM_TIMEOUT(len)   (20U + ((uint32_t)(len) * 10U) / 8U)

/* 等待器件内部写周期结束（应答轮询）
 * 参数：无
 * 返回：HAL_OK = 器件重新应答（写周期结束）；HAL_ERROR = 超时（20ms）仍未应答
 * 调用：EEPROM_Write 每写完一页后调用；阻塞式，仅限主循环/初始化上下文
 * 注意：写周期内器件不响应地址，用“只发地址看 ACK”的方式探测；
 *       每次探测 5ms 超时，失败后延时 1ms 再试，最坏阻塞约 EEPROM_WRITE_TIMEOUT ms。
 */
static HAL_StatusTypeDef EEPROM_WaitReady(void)
{
    uint8_t i;

    for (i = 0U; i < EEPROM_WRITE_TIMEOUT; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_I2C_ADDR, 1U, 5U) == HAL_OK)
        {
            return HAL_OK;
        }
        HAL_Delay(1);   /* 给内部写周期留时间，避免高频轮询占用 I2C 总线 */
    }

    return HAL_ERROR;
}

/* 探测 EEPROM 是否在线（只发器件地址，看是否 ACK）
 * 参数：无
 * 返回：HAL_OK = 在线；其它 = 无应答或总线错误
 * 调用：EEPROM_Init（上电）
 * 注意：2 次试验、20ms 超时，比写后就绪轮询宽松，用于判断焊接/供电是否正常。
 */
HAL_StatusTypeDef EEPROM_IsReady(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_I2C_ADDR, 2U, 20U);
}

/* EEPROM 初始化：仅做在线检测，不做格式化
 * 参数：无
 * 返回：HAL_OK 在线；其它 未检测到器件
 * 调用：上电初始化（main.c），返回值仅记录，不阻塞启动
 * 注意：器件缺失时后续读写都会失败，配置加载会走“使用默认值”分支，系统仍可运行。
 */
HAL_StatusTypeDef EEPROM_Init(void)
{
    return EEPROM_IsReady();
}

/* 从指定 16 位地址连续读取 len 字节
 * 参数：addr = 0 ~ EEPROM_SIZE-1 的字节地址；
 *       buf  = 接收缓冲，不可为 NULL；len = 1 ~ (EEPROM_SIZE - addr)
 * 返回：HAL_OK；HAL_ERROR（空指针/长度为 0/越界）；或底层 I2C 错误码
 * 调用：EEPROM_LoadConfig、EEPROM_SelfTest 及各 ReadXxx 封装（主循环/初始化）
 * 注意：I2C_MEMADD_SIZE_16BIT 表示器件地址之后先发 2 字节地址（高字节在前）；
 *       读不受页边界限制，可一次读整片。
 */
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

/* 从指定 16 位地址写入 len 字节，自动按 64 字节页拆分
 * 参数：addr = 0 ~ EEPROM_SIZE-1；buf = 源数据，不可为 NULL；len = 1 ~ (EEPROM_SIZE - addr)
 * 返回：HAL_OK；HAL_ERROR（空指针/长度为 0/越界）；或底层 I2C 错误码
 * 调用：EEPROM_SaveConfig、EEPROM_SelfTest 及各 WriteXxx 封装（主循环/初始化）
 * 注意：1) 器件页写不能跨页，跨页会回卷覆盖页首，故按 page_left 切块；
 *       2) 每页写完必须等内部写周期（EEPROM_WaitReady），否则下一页数据被丢弃；
 *       3) 全程阻塞，最坏耗时约 (len/64) * (页传输时间 + 5ms)。
 */
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
        page_left = (uint16_t)(EEPROM_PAGE_SIZE - (cur % EEPROM_PAGE_SIZE));   /* 当前页剩余空间 */
        chunk = (uint16_t)(len - done);
        if (chunk > page_left)
        {
            chunk = page_left;          /* 不能跨页写 */
        }

        /* 16 位地址 + 页内连续写；HAL 内部会组织 START/地址/数据/STOP */
        st = HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDR, cur, I2C_MEMADD_SIZE_16BIT,
                               (uint8_t *)&buf[done], chunk, EEPROM_TIMEOUT(chunk));
        if (st != HAL_OK)
        {
            return st;
        }

        st = EEPROM_WaitReady();   /* 等本页真正写进存储单元，才能写下一页 */
        if (st != HAL_OK)
        {
            return st;
        }

        done = (uint16_t)(done + chunk);
    }

    return HAL_OK;
}

/* 读 1 字节
 * 参数：addr = 0 ~ EEPROM_SIZE-1；val = 输出指针，不可为 NULL
 * 返回：同 EEPROM_Read
 * 调用：主循环/初始化上下文
 * 注意：内部走完整“发地址 + 读 1 字节”的 I2C 事务，比块读开销大，连续读建议用 EEPROM_Read。
 */
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *val)
{
    return EEPROM_Read(addr, val, 1U);
}

/* 写 1 字节
 * 参数：addr = 0 ~ EEPROM_SIZE-1；val = 待写数据
 * 返回：同 EEPROM_Write
 * 调用：主循环/初始化上下文
 * 注意：单字节也会触发一次内部写周期，耗时约 5ms，不要在高频任务里逐字节写。
 */
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t val)
{
    return EEPROM_Write(addr, &val, 1U);
}

/* 读 16 位小端数据
 * 参数：addr = 0 ~ EEPROM_SIZE-2；val = 输出指针，不可为 NULL
 * 返回：同 EEPROM_Read
 * 调用：主循环/初始化上下文
 * 注意：低字节在低地址（b[0] | b[1]<<8），与写入顺序一致。
 */
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

/* 写 16 位小端数据
 * 参数：addr = 0 ~ EEPROM_SIZE-2；val = 待写数据
 * 返回：同 EEPROM_Write
 * 调用：主循环/初始化上下文
 * 注意：拆成低字节在前存放；若地址落在页尾附近，EEPROM_Write 会自动拆分页写。
 */
HAL_StatusTypeDef EEPROM_WriteU16(uint16_t addr, uint16_t val)
{
    uint8_t b[2];

    b[0] = (uint8_t)(val & 0xFFU);
    b[1] = (uint8_t)(val >> 8);

    return EEPROM_Write(addr, b, 2U);
}

/* 读 32 位小端数据
 * 参数：addr = 0 ~ EEPROM_SIZE-4；val = 输出指针，不可为 NULL
 * 返回：同 EEPROM_Read
 * 调用：主循环/初始化上下文
 * 注意：低字节在低地址，与 EEPROM_WriteU32 对应。
 */
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

/* 写 32 位小端数据
 * 参数：addr = 0 ~ EEPROM_SIZE-4；val = 待写数据
 * 返回：同 EEPROM_Write
 * 调用：主循环/初始化上下文
 * 注意：4 字节可能跨 64 字节页边界，由 EEPROM_Write 内部拆分。
 */
HAL_StatusTypeDef EEPROM_WriteU32(uint16_t addr, uint32_t val)
{
    uint8_t b[4];

    b[0] = (uint8_t)(val & 0xFFU);
    b[1] = (uint8_t)((val >> 8) & 0xFFU);
    b[2] = (uint8_t)((val >> 16) & 0xFFU);
    b[3] = (uint8_t)((val >> 24) & 0xFFU);

    return EEPROM_Write(addr, b, 4U);
}

/* 读写自检：在测试区写固定图案再读回比对
 * 参数：无
 * 返回：HAL_OK 读写一致；HAL_ERROR 写/读失败或数据不一致
 * 调用：调试、产测时手动调用（不在上电流程中），主循环上下文
 * 注意：会覆盖 EEPROM_TEST_ADDR(0x3F00) 起的 4 字节，该区域独立于配置区，不影响参数。
 */
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

/* 保存当前配置到 EEPROM
 * 参数：无（数据来源为全局 g_cfg）
 * 返回：HAL_OK 写入成功；其它 写入失败（总线错误/写周期超时）
 * 调用：菜单确认/退出保存时由 menu.c 调用（主循环上下文，阻塞约 5~10ms）
 * 注意：1) ammo/ammo_target/shot_total 为保留字段，固定写 0（这些量不再掉电保存）；
 *       2) 校验和只累加 check 之前的字节，结构体尾部有对齐填充，不能用 sizeof-2；
 *       3) 先 memset 清零，保证填充字节确定，写入内容可复现。
 */
HAL_StatusTypeDef EEPROM_SaveConfig(void)
{
    EEPROM_Config_t cfg;
    uint8_t *p = (uint8_t *)&cfg;
    uint16_t sum = 0U;
    uint16_t i;

    memset(&cfg, 0, sizeof(cfg));   /* 填充字节也置 0，避免残留栈数据 */

    cfg.magic = EEPROM_CFG_MAGIC;   /* 魔数用于区分“从未保存”与“数据有效” */
    cfg.motor_duty_pct = g_cfg.motor_duty_pct;
    cfg.over_current_a = g_cfg.over_current_a;
    cfg.echo_enable = g_cfg.echo_enable;
    cfg.status_print_enable = g_cfg.status_print_enable;
    cfg.sensor1_type = g_cfg.sensor1_type;
    cfg.sensor2_type = g_cfg.sensor2_type;
    cfg.fire_mode = g_cfg.fire_mode;
    cfg.module_type = g_cfg.module_type;
    cfg.load_dir = g_cfg.load_dir;
    /* 余弹数 g_cfg.ammo、装填目标值、累计射击次数均不再掉电保存：
     * 结构体中保留这些字段以维持原有布局（其它参数不失效），写 0 占位。 */
    cfg.pulse_ms = g_cfg.pulse_ms;
    cfg.gap_ms = g_cfg.gap_ms;
    cfg.burst_shots = g_cfg.burst_shots;
    cfg.long_burst_shots = g_cfg.long_burst_shots;
    cfg.baudrate = g_cfg.baudrate;
    cfg.check = 0U;                 /* 先置 0，写入值确定（校验只覆盖该字段之前的字节） */

    /* 只累加 check 之前的字节：结构体尾部有对齐填充，不能用 sizeof-2 */
    for (i = 0U; i < (uint16_t)offsetof(EEPROM_Config_t, check); i++)
    {
        sum = (uint16_t)(sum + p[i]);
    }
    cfg.check = sum;

    return EEPROM_Write(EEPROM_CFG_ADDR, p, (uint16_t)sizeof(cfg));
}

/* 从 EEPROM 读取配置，校验通过后覆盖当前配置
 * 参数：无（结果写入全局 g_cfg）
 * 返回：HAL_OK 读取且校验通过；HAL_ERROR 读失败/魔数不符/校验和错误
 * 调用：上电初始化（main.c，Global_Init 之后、Relay_Init/AD_Init 之前）
 * 注意：1) 魔数不符视为“从未保存”，直接返回且不改动 g_cfg（保留默认值）；
 *       2) 逐字段做范围校验，越界回落默认值，避免异常配置导致动作异常；
 *       3) 电机占空比、余弹数、装填目标值不从 EEPROM 恢复。
 */
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

    for (i = 0U; i < (uint16_t)offsetof(EEPROM_Config_t, check); i++)
    {
        sum = (uint16_t)(sum + p[i]);
    }

    if (sum != cfg.check)
    {
        return HAL_ERROR;               /* 数据损坏 */
    }

    /* 电机占空比已从设置项移除，固定使用默认 90%，不再从 EEPROM 恢复 */
    g_cfg.over_current_a = cfg.over_current_a;
    g_cfg.echo_enable = cfg.echo_enable;
    g_cfg.status_print_enable = cfg.status_print_enable;
    g_cfg.sensor1_type = (cfg.sensor1_type == 1U) ? 1U : 0U;   /* 只接受 0/1，异常值按 NPN */
    g_cfg.sensor2_type = (cfg.sensor2_type == 1U) ? 1U : 0U;
    g_cfg.fire_mode = (cfg.fire_mode < FIRE_MODE_NUM) ? cfg.fire_mode : FIRE_MODE_SINGLE;
    g_cfg.module_type = (cfg.module_type < MODULE_NUM) ? cfg.module_type : MODULE_FIRE;
    g_cfg.load_dir = (cfg.load_dir == LOAD_DIR_REVERSE) ? LOAD_DIR_REVERSE : LOAD_DIR_FORWARD;
    /* 余弹数/装填目标值不从 EEPROM 恢复：保持 Global_Init() 里的默认值，
     * 之后由 IN_7 计数传感器递减，或在设置模式手动装定。 */

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

/* 清除配置（下次上电使用默认值）
 * 参数：无
 * 返回：同 EEPROM_WriteU16
 * 调用：调试/恢复出厂设置时调用（主循环上下文）
 * 注意：只需把魔数写成 0 让 LoadConfig 判定“未保存”，无需擦除整片；
 *       器件不支持整片擦除，逐字节写 0 反而更慢。
 */
HAL_StatusTypeDef EEPROM_EraseConfig(void)
{
    return EEPROM_WriteU16(EEPROM_CFG_ADDR, 0x0000U);
}
