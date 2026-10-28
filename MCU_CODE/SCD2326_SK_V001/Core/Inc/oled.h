#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：oled.h —— SSD1309 128x64 OLED 显示接口
 *
 * 模块职责：内存显存绘制（点/横线/ASCII/汉字）、整屏刷新、上电初始化命令序列。
 * 硬件：ZJY_M242_OLED（2.42 寸 128x64，SSD1309）；SPI2 两线模式（SCK=PB13、MOSI=PB15），
 *       CS=PB12（软件片选）、DC=PA11（0=命令 1=数据）、RES=PA10；SPI 时钟 2.25MHz。
 * 显示结构：8 页 x 128 列，每字节对应纵向 8 个像素，bit0 在最上方。
 * 运行上下文：初始化在上电阶段；绘制与刷新在主循环 Menu_Task 内，
 *           全部为阻塞 SPI 操作，不可在中断中使用。
 */

/* 屏幕：中景园 ZJY_M242_OLED（2.42 寸 128x64，SSD1309）
 * 接口：SPI2 两线模式（只用到 MOSI 发送），SCK=PB13 MOSI=PB15 CS=PB12 DC=PA11 RES=PA10 */
#define OLED_WIDTH        128U                    /* 水平像素数（= 每页列数） */
#define OLED_HEIGHT       64U                     /* 垂直像素数（= 8 页 x 8 行） */
#define OLED_PAGES        (OLED_HEIGHT / 8U)      /* 页数 = 8，SSD1309 页寻址的单位 */

#define OLED_CMD          0U                      /* DC 电平：0 = 命令（保留定义，驱动直接写 GPIO） */
#define OLED_DATA         1U                      /* DC 电平：1 = 显存数据 */

/* 上电自检：1 = 初始化后整屏点亮 500ms（用于确认屏体/连线），量产置 0 */
#define OLED_BOOT_TEST    0U

/* 字库尺寸：ASCII 为 8x15，中文固定 13x13 */
#define OLED_ASC_WIDTH    8U                      /* ASCII 字宽 8 列（8x15 字模存于 8x16 单元格） */
#define OLED_CJK_WIDTH    13U                     /* 汉字字宽（13 列 x 16 行单元格，底部 3 行空白） */

void OLED_Init(void);      /* 上电：复位时序 + SSD1309 初始化命令，阻塞约 300ms */
void OLED_Clear(void);     /* 只清内存显存，需再 Refresh 才上屏 */
void OLED_Refresh(void);   /* 整屏 1024 字节刷新到屏，2.25MHz 下约 4~5ms */

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode);   /* mode：0 点亮，1 熄灭 */
void OLED_DrawHLine(uint8_t y, uint8_t mode);              /* 画整行（0~127 列） */

void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t mode);          /* 单个 ASCII，8x15 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t mode); /* ASCII 字符串，遇到不可打印字符停止 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t mode); /* 定长十进制，左补 0 */

/* 混合显示：ASCII 用 8x15，中文用 15x15，字符串为 UTF-8 编码
 * （汉字字模实际为 13x13，占 13x16 单元格；只支持 3 字节 UTF-8 汉字，未收录字符留空） */
void OLED_ShowUTF8(uint8_t x, uint8_t y, const char *str, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
