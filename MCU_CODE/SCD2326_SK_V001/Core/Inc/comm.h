#ifndef __COMM_H
#define __COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 文件：comm.h —— 串口应用层（USART2 / RS232）对外接口
 *
 * 模块职责：DMA + IDLE 接收分帧、帧环形队列缓冲、DMA 回传、周期状态帧输出。
 * 硬件：USART2（PA2=TX / PA3=RX）-> U14(RS8T245) 电平转换 -> TP3232 -> RS232；
 *       DMA1_Channel6 = RX、DMA1_Channel7 = TX。
 * 关键参数：单帧上限 256 字节；队列 4 槽（牺牲 1 槽区分空/满，可用 3 帧）；
 *           默认 115200-8-N-1，可由 EEPROM 配置覆盖；状态帧周期 500ms。
 * 运行上下文：Comm_Init/Comm_SetBaud 在上电阶段；Comm_Task 等在主循环；
 *            HAL_UARTEx_RxEventCallback/HAL_UART_ErrorCallback 在中断上下文。
 */

/* USART2 通过 U14(RS8T245) 送 TP3232 转 RS232，与上位机通信 */
#define COMM_RX_SIZE      256U   /* 单帧最大长度（DMA 接收缓冲）；超长帧按 256 字节截断 */
#define COMM_TX_SIZE      256U   /* 回传缓冲；应 >= COMM_RX_SIZE，否则长帧回传会被截断 */
#define COMM_QUEUE_LEN    4U     /* 待处理帧队列深度；环形队列空/满判定占 1 槽，实际可存 3 帧 */

/* 调试计数：便于断点确认收发是否正常（在中断内更新，故加 volatile） */
extern volatile uint32_t comm_rx_frames;   /* 收到的帧数（成功入队） */
extern volatile uint32_t comm_tx_frames;   /* 已回传的帧数（DMA 启动成功） */
extern volatile uint32_t comm_drop_frames; /* 队列满丢弃的帧数 */

void Comm_Init(void);             /* 上电：清队列/计数，挂起第一次 DMA 接收 */
void Comm_Task(void);             /* 主循环：从队列取一帧回传（非阻塞，发送忙则下轮再试） */
void Comm_StatusTask(void);       /* 主循环：每 500ms 把 10 路输入状态 + 工作模式打印到串口 */
void Comm_SendStatus(void);       /* 立即打印一帧输入状态（格式 IN1=.. MODE=.. HIGH=..） */
HAL_StatusTypeDef Comm_Send(const uint8_t *data, uint16_t len);   /* 启动 DMA 发送；缓冲须保持到发送完成 */
void Comm_SetBaud(uint32_t baud); /* 改变波特率：停接收 -> 重配 USART2 -> 重新挂接收 */
void Comm_ClearCounters(void);    /* 清零上面三个统计计数，不影响队列与硬件 */

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H */
