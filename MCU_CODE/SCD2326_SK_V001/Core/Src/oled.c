/* OLED 驱动（SSD1309，128x64，SPI）
 * 硬件：SPI2 两线模式，仅用 MOSI 发送，PB13=SCK PB15=MOSI，PB12=CS（软件片选），PA11=DC，PA10=RES
 * 说明：GRAM 先画在内存，再整屏刷新，避免逐字节刷屏造成的撕裂和慢速。
 *       字库：ASCII 8x15、汉字 13x13。
 */
#include "oled.h"
#include "spi.h"
#include "oledfont.h"

static uint8_t s_gram[OLED_PAGES][OLED_WIDTH];

/*----------------------------- 底层 SPI 收发 -----------------------------*/
static void OLED_WriteCmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1U, 10U);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

static void OLED_WriteData(const uint8_t *buf, uint16_t len)
{
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, len, 100U);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

/*----------------------------- 显存操作 -----------------------------*/
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
        OLED_WriteData(s_gram[page], OLED_WIDTH);
    }
}

/* mode: 0 正常（点亮），1 反色（用于选中行高亮） */
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

void OLED_DrawHLine(uint8_t y, uint8_t mode)
{
    uint8_t x;

    for (x = 0; x < OLED_WIDTH; x++)
    {
        OLED_DrawPoint(x, y, mode);
    }
}

/* 位图块写入：数据按"列优先、每字节 8 行、LSB 在上"排列 */
static void OLED_Blit(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *data, uint8_t mode)
{
    uint8_t i;
    uint8_t m;
    uint8_t x0 = x;
    uint8_t y0 = y;
    uint8_t temp;

    for (i = 0; i < (uint8_t)((w * h) / 8U); i++)
    {
        temp = data[i];
        for (m = 0; m < 8U; m++)
        {
            OLED_DrawPoint(x, y, (temp & 0x01U) ? mode : (uint8_t)(!mode));
            temp >>= 1;
            y++;
        }

        x++;
        if ((uint8_t)(x - x0) == w)
        {
            x = x0;
            y0 = (uint8_t)(y0 + 8U);
        }
        y = y0;
    }
}

/* ASCII 字符：8x15 点阵（存储 8x16，最后一行留空） */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t mode)
{
    uint8_t idx;

    if ((chr < ' ') || (chr > '~'))
    {
        chr = ' ';
    }

    idx = (uint8_t)(chr - ' ');
    OLED_Blit(x, y, OLED_ASC_WIDTH, 16U, asc2_0815[idx], mode);
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t mode)
{
    while ((*str >= ' ') && (*str <= '~'))
    {
        OLED_ShowChar(x, y, *str, mode);
        x = (uint8_t)(x + OLED_ASC_WIDTH);
        str++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t mode)
{
    uint8_t i;
    uint32_t pow10 = 1U;

    for (i = 1U; i < len; i++)
    {
        pow10 *= 10U;
    }

    for (i = 0; i < len; i++)
    {
        OLED_ShowChar((uint8_t)(x + (OLED_ASC_WIDTH * i)), y,
                      (char)('0' + ((num / pow10) % 10U)), mode);
        pow10 /= 10U;
    }
}

/* 在字库中查找 UTF-8 汉字，返回索引，找不到返回 cjk_font_num */
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

/* UTF-8 字符串显示：ASCII 8x15，汉字 15x15 */
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

/* 初始化：SSD1309 上电时序 + 配置 */
void OLED_Init(void)
{
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET);

    HAL_Delay(200);

    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

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
