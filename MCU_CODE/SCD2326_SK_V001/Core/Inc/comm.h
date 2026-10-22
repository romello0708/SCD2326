#ifndef __COMM_H
#define __COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USART2 通过 U14(RS8T245) 送 TP3232 转 RS232，与上位机通信 */
#define COMM_RX_SIZE      256U   /* 单帧最大长度（DMA 接收缓冲） */
#define COMM_TX_SIZE      256U   /* 回传缓冲 */
#define COMM_QUEUE_LEN    4U     /* 待处理帧队列深度 */

/* 调试计数：便于断点确认收发是否正常 */
extern volatile uint32_t comm_rx_frames;   /* 收到的帧数 */
extern volatile uint32_t comm_tx_frames;   /* 已回传的帧数 */
extern volatile uint32_t comm_drop_frames; /* 队列满丢弃的帧数 */

void Comm_Init(void);
void Comm_Task(void);
void Comm_StatusTask(void);       /* 定时把 10 路输入状态 + 工作模式打印到串口 */
void Comm_SendStatus(void);       /* 立即打印一帧输入状态 */
HAL_StatusTypeDef Comm_Send(const uint8_t *data, uint16_t len);
void Comm_SetBaud(uint32_t baud);
void Comm_ClearCounters(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H */
