/* 串口应用层（USART2 / RS232）
 * 接收：DMA + IDLE 空闲中断，天然按帧切分，支持不定长数据；
 *       每帧拷入队列后立即重新挂接收，主循环不参与搬运。
 * 发送：DMA 发送，主循环只负责把队列里的帧送上发送通道。
 * 当前为回传（echo）测试：收到的每一帧原样发回上位机。
 */
#include "comm.h"
#include "usart.h"
#include "global.h"
#include "digital_input.h"
#include <string.h>

volatile uint32_t comm_rx_frames   = 0;
volatile uint32_t comm_tx_frames   = 0;
volatile uint32_t comm_drop_frames = 0;

static uint8_t  rx_dma_buf[COMM_RX_SIZE];                 /* DMA 接收目标 */
static uint8_t  tx_dma_buf[COMM_TX_SIZE];                 /* DMA 发送缓冲 */
static uint8_t  frame_buf[COMM_QUEUE_LEN][COMM_RX_SIZE];  /* 待回传帧队列 */
static uint16_t frame_len[COMM_QUEUE_LEN];

static volatile uint8_t q_head = 0;   /* 中断写入位置 */
static volatile uint8_t q_tail = 0;   /* 主循环取出位置 */

/* 重新挂起一帧接收 */
static void Comm_StartReceive(void)
{
    if (huart2.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, COMM_RX_SIZE);
    }
}

void Comm_Init(void)
{
    q_head = 0;
    q_tail = 0;
    comm_rx_frames   = 0;
    comm_tx_frames   = 0;
    comm_drop_frames = 0;

    Comm_StartReceive();
}

/* DMA 接收事件：IDLE（一帧结束）或缓冲写满时进入，Size 即本帧字节数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t next;

    if (huart->Instance != USART2)
    {
        return;
    }

    if (Size > 0U)
    {
        next = (uint8_t)((q_head + 1U) % COMM_QUEUE_LEN);
        if (next == q_tail)
        {
            comm_drop_frames++;   /* 队列满，丢弃本帧 */
        }
        else
        {
            if (Size > COMM_RX_SIZE) { Size = COMM_RX_SIZE; }
            memcpy(frame_buf[q_head], rx_dma_buf, Size);
            frame_len[q_head] = Size;
            q_head = next;
            comm_rx_frames++;
        }
    }

    Comm_StartReceive();   /* 立刻重新挂接收，不丢下一帧 */
}

/* 接收出错（溢出/帧错/噪声/校验）后恢复接收，避免通道停摆 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_OREFLAG(huart);
    Comm_StartReceive();
}

/* 发送一帧（DMA，调用后立即返回；缓冲需保持到发送完成） */
HAL_StatusTypeDef Comm_Send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    return HAL_UART_Transmit_DMA(&huart2, data, len);
}

/* 主循环任务：把收到的帧原样回传，一次只送一帧，避免通道冲突 */
void Comm_Task(void)
{
    uint16_t len;

    if (q_tail == q_head)
    {
        return;   /* 无待回传帧 */
    }

    /* 回传关闭时直接丢弃该帧，避免队列积压 */
    if (g_cfg.echo_enable == 0U)
    {
        q_tail = (uint8_t)((q_tail + 1U) % COMM_QUEUE_LEN);
        return;
    }

    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return;   /* 上一帧还没发完 */
    }

    len = frame_len[q_tail];
    if (len > COMM_TX_SIZE) { len = COMM_TX_SIZE; }
    memcpy(tx_dma_buf, frame_buf[q_tail], len);

    if (HAL_UART_Transmit_DMA(&huart2, tx_dma_buf, len) == HAL_OK)
    {
        q_tail = (uint8_t)((q_tail + 1U) % COMM_QUEUE_LEN);
        comm_tx_frames++;
    }
}

/* 修改波特率：停接收 -> 重配 USART2 -> 重新挂接收 */
void Comm_SetBaud(uint32_t baud)
{
    (void)HAL_UART_AbortReceive(&huart2);

    huart2.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return;
    }

    g_cfg.baudrate = baud;
    Comm_StartReceive();
}

void Comm_ClearCounters(void)
{
    comm_rx_frames = 0;
    comm_tx_frames = 0;
    comm_drop_frames = 0;
}

/*--------------------- 输入状态打印 ---------------------*/
static uint8_t  s_status_buf[128];
static uint32_t s_status_tick;

static void Status_PutStr(uint16_t *pos, const char *s)
{
    while (*s != '\0')
    {
        s_status_buf[*pos] = (uint8_t)*s;
        (*pos)++;
        s++;
    }
}

static void Status_PutU32(uint16_t *pos, uint32_t v)
{
    char tmp[11];
    uint8_t n = 0U;

    if (v == 0U)
    {
        s_status_buf[*pos] = (uint8_t)'0';
        (*pos)++;
        return;
    }

    while ((v > 0U) && (n < 10U))
    {
        tmp[n] = (char)('0' + (v % 10U));
        v /= 10U;
        n++;
    }

    while (n > 0U)
    {
        n--;
        s_status_buf[*pos] = (uint8_t)tmp[n];
        (*pos)++;
    }
}

/* 打印一帧：IN1=0 ... IN10=1 MODE=SET HIGH=1 */
void Comm_SendStatus(void)
{
    static const char * const mode_name[4] = { "SET", "WORK", "DEBUG", "INVALID" };
    uint16_t pos = 0U;
    uint8_t i;
    uint8_t mode = g_cfg.work_mode;

    if (mode > WORK_MODE_INVALID)
    {
        mode = WORK_MODE_INVALID;
    }

    for (i = 1U; i <= DI_NUM; i++)
    {
        Status_PutStr(&pos, "IN");
        Status_PutU32(&pos, i);
        Status_PutStr(&pos, "=");
        Status_PutU32(&pos, DI_IsHigh(i));
        Status_PutStr(&pos, " ");
    }

    Status_PutStr(&pos, "MODE=");
    Status_PutStr(&pos, mode_name[mode]);
    Status_PutStr(&pos, " HIGH=");
    Status_PutU32(&pos, DI_GetHighCount());
    Status_PutStr(&pos, "\r\n");

    (void)Comm_Send(s_status_buf, pos);
}

/* 周期任务：按 COMM_STATUS_PERIOD_MS 打印输入状态 */
void Comm_StatusTask(void)
{
    uint32_t now = HAL_GetTick();

    if (now == s_status_tick)
    {
        return;
    }

    if ((now - s_status_tick) < COMM_STATUS_PERIOD_MS)
    {
        return;
    }

    s_status_tick = now;

    if (g_cfg.status_print_enable == 0U)
    {
        return;
    }

    Comm_SendStatus();
}
