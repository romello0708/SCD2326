#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 屏幕：中景园 ZJY_M242_OLED（2.42 寸 128x64，SSD1309）
 * 接口：SPI 半双工（1-line），SCK=PB13 MOSI=PB15 CS=PB12 DC=PA11 RES=PA10 */
#define OLED_WIDTH        128U
#define OLED_HEIGHT       64U
#define OLED_PAGES        (OLED_HEIGHT / 8U)

#define OLED_CMD          0U
#define OLED_DATA         1U

/* 字库尺寸：8 = 6x8 ASCII，16 = 8x16 ASCII；中文固定 16x16 */
#define OLED_SIZE_8       8U
#define OLED_SIZE_16      16U

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode);
void OLED_DrawHLine(uint8_t y, uint8_t mode);

void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode);

/* 混合显示：ASCII 用 6x8，中文用 16x16，字符串为 UTF-8 编码 */
void OLED_ShowUTF8(uint8_t x, uint8_t y, const char *str, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
