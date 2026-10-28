/* OLED 驱动（SSD1309，128x64，SPI）
 *
 * 模块职责：
 *   1) 底层 SPI 命令/数据写入（软件片选 + DC 区分命令与数据）；
 *   2) 内存显存 s_gram[8][128] 的清屏、画点、画线、字符/汉字绘制；
 *   3) 整屏刷新到 SSD1309；
 *   4) 上电初始化命令序列与可选整屏自检。
 * 说明：GRAM 先画在内存，再整屏刷新，避免逐字节刷屏造成的撕裂和慢速。
 *
 * 硬件：
 *   中景园 ZJY_M242_OLED（2.42 寸 128x64，SSD1309）；
 *   SPI2 主机、两线模式（仅 SCK+MOSI），模式 0、8 位、MSB 在前；
 *   PB13=SCK、PB15=MOSI、PB12=CS（软件片选）、PA11=DC（0=命令 1=数据）、PA10=RES；
 *   时钟 36MHz/16 = 2.25MHz，与原厂程序一致；发送使用 HAL_SPI_Transmit（阻塞）。
 *
 * 显示结构：
 *   SSD1309 页寻址：8 页 x 128 列，每字节纵向 8 像素、bit0 在上；
 *   内存显存与屏上一一对应：s_gram[page][x] 的 bit(y&7) = 像素 (x, page*8 + y%8)。
 *   字库：ASCII 8x15（存储 8x16，最后一行空白），汉字 13x13（存储 13 列 x 2 页）。
 *
 * 运行上下文：
 *   OLED_Init：上电初始化（main.c，最后一个初始化步骤）；
 *   绘制与刷新：主循环 Menu_Task 内调用；全部为阻塞 SPI，不可在中断中使用。
 */
#include "oled.h"
#include "spi.h"
#include "oledfont.h"

static uint8_t s_gram[OLED_PAGES][OLED_WIDTH];   /* 内存显存：[页][列]，与屏上页结构一致 */

/*----------------------------- 底层 SPI 收发 -----------------------------*/
/* 发送 1 字节命令
 * 参数：cmd = SSD1309 命令字节
 * 返回：无
 * 调用：OLED_Init、OLED_Refresh（主循环）
 * 注意：片选软件控制：CS 拉低 -> DC=0（命令）-> SPI 阻塞发送（10ms 超时）-> CS 拉高。
 */
static void OLED_WriteCmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);   /* DC=0 表示命令 */
    HAL_SPI_Transmit(&hspi2, &cmd, 1U, 10U);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

/* 连续发送数据字节
 * 参数：buf = 数据首地址；len = 字节数（一次最多一页 128 字节）
 * 返回：无
 * 调用：OLED_Refresh（主循环）
 * 注意：DC=1 表示写显存数据；100ms 超时相对 128 字节 x 8bit / 2.25MHz 约 0.46ms 留足余量。
 */
static void OLED_WriteData(const uint8_t *buf, uint16_t len)
{
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, 100U);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

/*----------------------------- 显存操作 -----------------------------*/
/* 清空内存显存（不刷新到屏）
 * 参数：无
 * 返回：无
 * 调用：OLED_Init、菜单切换页面（主循环）
 * 注意：只清 s_gram（8x128 字节），屏上内容保持不变，必须再调 OLED_Refresh 才可见。
 */
void OLED_Clear(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0; page < OLED_PAGES; page++)
    {
        for (col = 0; col < OLED_WIDTH; col++)
        {
            s_gram[page][col] = 0x00U;
        }
    }
}

/* 把内存显存整屏刷新到 SSD1309
 * 参数：无
 * 返回：无
 * 调用：菜单绘制完成后调用（主循环 Menu_Task）
 * 注意：页寻址模式下逐页设置页地址(0xB0+page)和列地址（低 4 位 / 高 4 位各一条命令），
 *       再连续写 128 字节；整屏 1024 字节，2.25MHz 下约 3.6ms，加上命令开销约 4~5ms。
 */
void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t cmd[3];

    for (page = 0; page < OLED_PAGES; page++)
    {
        cmd[0] = (uint8_t)(0xB0U + page);   /* 页地址 */
        cmd[1] = 0x00U;                     /* 列地址低 4 位 */
        cmd[2] = 0x10U;                     /* 列地址高 4 位 */
        OLED_WriteCmd(cmd[0]);
        OLED_WriteCmd(cmd[1]);
        OLED_WriteCmd(cmd[2]);
        OLED_WriteData(s_gram[page], OLED_WIDTH);   /* 本页 128 列一次写完 */
    }
}

/* 在内存显存画一个点
 * 参数：x = 0~127 列；y = 0~63 行（越界直接返回）
 *       mode = 0 置位点亮，1 清位熄灭（用于反显/高亮）
 * 返回：无
 * 调用：所有绘图函数（主循环）
 * 注意：只改内存不上屏；按 y>>3 选页，1<<(y&7) 定位页内 bit（bit0 在最上方）。
 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode)
{
    uint8_t mask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    mask = (uint8_t)(1U << (y & 0x07U));

    if (mode == 0U)
    {
        s_gram[y >> 3][x] |= mask;
    }
    else
    {
        s_gram[y >> 3][x] &= (uint8_t)~mask;
    }
}

/* 画一条整行横线
 * 参数：y = 0~63 行号；mode = 0 点亮 / 1 熄灭
 * 返回：无
 * 调用：菜单分隔线、反显行（主循环）
 * 注意：逐点写 128 次，只改内存显存，需配合 OLED_Refresh 才上屏。
 */
void OLED_DrawHLine(uint8_t y, uint8_t mode)
{
    uint8_t x;

    for (x = 0; x < OLED_WIDTH; x++)
    {
        OLED_DrawPoint(x, y, mode);
    }
}

/* 位图块写入：数据按“列优先、每字节 8 行、LSB 在上”排列
 * 参数：x,y = 左上角坐标；w = 列数；h = 行数（必须为 8 的倍数）
 *       data = 字模数据（长度 = w*h/8 字节）；mode = 0 正常显示 / 1 整块反显
 * 返回：无
 * 调用：OLED_ShowChar、OLED_ShowUTF8（主循环）
 * 注意：mode 同时作用于前景和背景（位 1 画 mode，位 0 画 !mode），
 *       所以 mode=1 是连背景一起反色，菜单用它做选中行高亮；
 *       每 w 个字节换一页（y0 += 8），因此 h 必须按页对齐。
 */
static void OLED_Blit(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *data, uint8_t mode)
{
    uint8_t i;
    uint8_t m;
    uint8_t x0 = x;
    uint8_t y0 = y;
    uint8_t temp;

    for (i = 0; i < (uint8_t)((w * h) / 8U); i++)
    {
        temp = data[i];     /* 1 字节 = 同一列的 8 个像素，LSB 在最上方 */
        for (m = 0; m < 8U; m++)
        {
            /* 位 1 画前景色 mode，位 0 画背景色 !mode，实现正常/反显两种效果 */
            OLED_DrawPoint(x, y, (temp & 0x01U) ? mode : (uint8_t)(!mode));
            temp >>= 1;
            y++;
        }

        x++;
        if ((uint8_t)(x - x0) == w)
        {
            x = x0;                     /* 本页宽度写完，回到左边 */
            y0 = (uint8_t)(y0 + 8U);    /* 下移一页（8 行） */
        }
        y = y0;
    }
}

/* ASCII 字符：8x15 点阵（存储 8x16，最后一行留空）
 * 参数：x,y = 左上角坐标；chr = 字符，' '(0x20) ~ '~'(0x7E)，其它按空格显示
 *       mode = 0 正常 / 1 反显
 * 返回：无
 * 调用：OLED_ShowString / OLED_ShowNum / OLED_ShowUTF8（主循环）
 * 注意：字库 asc2_0815 从空格开始按 ASCII 顺序排列，每字符 16 字节（8 列 x 2 页）。
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t mode)
{
    uint8_t idx;

    if ((chr < ' ') || (chr > '~'))
    {
        chr = ' ';
    }

    idx = (uint8_t)(chr - ' ');   /* 字库索引：空格为 0 */
    OLED_Blit(x, y, OLED_ASC_WIDTH, 16U, asc2_0815[idx], mode);
}

/* 显示 ASCII 字符串
 * 参数：x,y = 起始坐标；str = 以 '\0' 结尾的可打印 ASCII；mode = 0 正常 / 1 反显
 * 返回：无
 * 调用：菜单、调试页面（主循环）
 * 注意：遇到不可打印字符（<0x20 或 >0x7E）即停止，不处理换行；
 *       每字符等宽 8 像素，右侧越界部分由 OLED_DrawPoint 丢弃。
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t mode)
{
    while ((*str >= ' ') && (*str <= '~'))
    {
        OLED_ShowChar(x, y, *str, mode);
        x = (uint8_t)(x + OLED_ASC_WIDTH);
        str++;
    }
}

/* 显示定长十进制数（高位补 0）
 * 参数：x,y = 起始坐标；num = 待显示值 0~4294967295；len = 位数 1~10；mode 同上
 * 返回：无
 * 调用：菜单数值显示（主循环）
 * 注意：先算出最高位权 10^(len-1)，再逐位取整显示；len > 10 时高位会重复显示 0。
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t mode)
{
    uint8_t i;
    uint32_t pow10 = 1U;

    for (i = 1U; i < len; i++)
    {
        pow10 *= 10U;   /* 最高位对应的权值 */
    }

    for (i = 0; i < len; i++)
    {
        OLED_ShowChar((uint8_t)(x + (OLED_ASC_WIDTH * i)), y,
                      (char)('0' + ((num / pow10) % 10U)), mode);
        pow10 /= 10U;
    }
}

/* 在字库中查找 UTF-8 汉字，返回索引，找不到返回 cjk_font_num
 * 参数：utf8 = 指向汉字 UTF-8 首字节的指针（只比较前 3 字节）
 * 返回：0 ~ cjk_font_num-1 为字库索引；cjk_font_num 表示未收录
 * 调用：OLED_ShowUTF8（主循环）
 * 注意：61 个字线性查表，未做排序/二分；返回 cjk_font_num 作为“无效索引”哨兵。
 */
static uint16_t OLED_FindGlyph(const char *utf8)
{
    uint16_t i;

    for (i = 0; i < cjk_font_num; i++)
    {
        if ((cjk_font[i].utf8[0] == utf8[0]) &&
            (cjk_font[i].utf8[1] == utf8[1]) &&
            (cjk_font[i].utf8[2] == utf8[2]))
        {
            return i;
        }
    }

    return cjk_font_num;
}

/* UTF-8 字符串显示：ASCII 8x15，汉字 15x15（实际字模为 13x13，占 13x16 单元格）
 * 参数：x,y = 起始坐标；str = UTF-8 字符串；mode = 0 正常 / 1 反显
 * 返回：无
 * 调用：菜单页面（主循环）
 * 注意：只识别 1 字节 ASCII 与 3 字节汉字；未收录汉字不绘制但仍占 13 像素宽度，
 *       保证后续字符不错位；2/4 字节 UTF-8（如 emoji）不支持。
 */
void OLED_ShowUTF8(uint8_t x, uint8_t y, const char *str, uint8_t mode)
{
    uint16_t idx;

    while (*str != '\0')
    {
        if ((uint8_t)*str < 0x80U)
        {
            OLED_ShowChar(x, y, *str, mode);
            x = (uint8_t)(x + OLED_ASC_WIDTH);
            str++;
        }
        else
        {
            idx = OLED_FindGlyph(str);
            if (idx < cjk_font_num)
            {
                OLED_Blit(x, y, OLED_CJK_WIDTH, 16U, cjk_font[idx].data, mode);
            }
            x = (uint8_t)(x + OLED_CJK_WIDTH);
            str += 3;      /* UTF-8 常用汉字为 3 字节 */
        }
    }
}

/* 初始化：SSD1309 上电时序 + 配置
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main.c，最后一个外设初始化步骤）
 * 注意：RES 低 200ms、高 100ms 满足 SSD1309 上电复位要求；
 *       0xFD/0x12 是 SSD1309 的解锁命令，不发则后续命令不生效；
 *       全程阻塞（含可选自检 500ms），只在上电阶段调用。
 */
void OLED_Init(void)
{
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);     /* 片选先释放 */
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET); /* 复位拉低 */

    HAL_Delay(200);

    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);   /* 释放复位 */
    HAL_Delay(100);

    /* 以下为 SSD1309 初始化命令序列：先解锁，再关显示配置，最后开显示 */
    OLED_WriteCmd(0xFD);   /* 解锁命令 */
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xAE);   /* 关显示 */
    OLED_WriteCmd(0x20);   /* 显存寻址方式：页寻址 */
    OLED_WriteCmd(0x02);
    OLED_WriteCmd(0x8D);   /* 电荷泵使能（部分模组默认关闭，必须显式打开） */
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xD5);   /* 时钟分频 */
    OLED_WriteCmd(0xA0);
    OLED_WriteCmd(0xA8);   /* 复用率 */
    OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3);   /* 显示偏移 */
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);   /* 起始行 */
    OLED_WriteCmd(0xA1);   /* 列方向 */
    OLED_WriteCmd(0xC8);   /* 行方向 */
    OLED_WriteCmd(0xDA);   /* COM 引脚配置 */
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81);   /* 对比度 */
    OLED_WriteCmd(0x7F);
    OLED_WriteCmd(0xD9);   /* 预充电 */
    OLED_WriteCmd(0x82);
    OLED_WriteCmd(0xDB);   /* VCOMH */
    OLED_WriteCmd(0x34);
    OLED_WriteCmd(0xA4);   /* 按显存显示 */
    OLED_WriteCmd(0xA6);   /* 正常显示 */

    OLED_Clear();
    OLED_Refresh();

    OLED_WriteCmd(0xAF);   /* 开显示 */

    /* 上电自检：整屏点亮 500ms，用于确认屏体与 SPI 通路正常 */
#if (OLED_BOOT_TEST != 0U)
    {
        uint8_t page;
        uint8_t col;

        for (page = 0U; page < OLED_PAGES; page++)
        {
            for (col = 0U; col < OLED_WIDTH; col++)
            {
                s_gram[page][col] = 0xFFU;
            }
        }
        OLED_Refresh();
        HAL_Delay(500);

        OLED_Clear();
        OLED_Refresh();
    }
#endif
}
