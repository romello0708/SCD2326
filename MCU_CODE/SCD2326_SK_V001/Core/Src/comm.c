/* 串口应用层（USART2 / RS232）
 *
 * 模块职责：
 *   1) 接收上位机数据帧：DMA + IDLE 空闲中断分帧，天然按帧切分，支持不定长数据；
 *      每帧拷入环形队列后立即重新挂接收，主循环不参与搬运；
 *   2) 发送：DMA 发送，主循环只负责把队列里的帧送上发送通道；
 *   3) 当前为回传（echo）测试：收到的每一帧原样发回上位机（可由 g_cfg.echo_enable 关闭）；
 *   4) 另提供周期输入状态文本帧：IN1=0 ... IN10=1 MODE=SET HIGH=1。
 *
 * 硬件：
 *   USART2：PA2=TX、PA3=RX，经 U14(RS8T245) 电平转换后送 TP3232 转 RS232，与上位机通信；
 *   DMA1_Channel6 = USART2_RX，DMA1_Channel7 = USART2_TX（NVIC 优先级均为 1，见 dma.c）。
 *
 * 关键参数与时序：
 *   默认 115200-8-N-1，可由 EEPROM 中保存的 g_cfg.baudrate 覆盖（Comm_SetBaud）；
 *   COMM_RX_SIZE = 256：单帧上限，也是 DMA 接收缓冲大小；
 *   COMM_QUEUE_LEN = 4：待处理帧队列深度，环形队列牺牲一个槽区分空/满，实际可缓存 3 帧；
 *   COMM_STATUS_PERIOD_MS = 500ms：状态帧输出周期。
 *
 * 运行上下文：
 *   上电初始化（main.c，主循环之前）：Comm_Init、Comm_SetBaud；
 *   USART2 DMA 中断：HAL_UARTEx_RxEventCallback（IDLE/缓冲满）、HAL_UART_ErrorCallback；
 *   主循环：Comm_Task（搬运并回传）、Comm_StatusTask（周期状态帧）、Comm_SendStatus。
 *   中断只做 memcpy 与索引更新，不打印、不阻塞，与主循环通过 q_head/q_tail 通信。
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

/* 重新挂起一次“接收至空闲”的 DMA 接收
 * 参数：无
 * 返回：无
 * 调用：Comm_Init（上电）、接收事件回调末尾、错误回调末尾（初始化或中断上下文）
 * 注意：必须先确认 RxState 为 READY。若上一次接收尚未收尾就重复启动，
 *       HAL 会返回 BUSY，DMA 目标仍指向同一个 rx_dma_buf，可能覆盖正在处理的帧。
 */
static void Comm_StartReceive(void)
{
    if (huart2.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, COMM_RX_SIZE);
    }
}

/* 串口应用层初始化：清空接收队列与统计计数，并挂起第一次 DMA 接收
 * 参数：无
 * 返回：无
 * 调用：上电初始化（main.c，MX_USART2_UART_Init 之后、主循环之前）
 * 注意：只复位软件队列，不重新配置 USART2；波特率由 Comm_SetBaud 单独设置。
 */
void Comm_Init(void)
{
    q_head = 0;
    q_tail = 0;
    comm_rx_frames   = 0;
    comm_tx_frames   = 0;
    comm_drop_frames = 0;

    Comm_StartReceive();
}

/* DMA 接收事件：IDLE（一帧结束）或缓冲写满时进入，Size 即本帧字节数
 * 参数：huart = 触发中断的串口句柄（只处理 USART2，其它串口直接返回）
 *       Size  = 本次已接收的字节数，1 ~ COMM_RX_SIZE
 * 返回：无
 * 调用：USART2 接收 DMA 触发 IDLE（总线空闲）或接收缓冲写满时由 HAL 回调，中断上下文
 * 注意：中断内只做 memcpy 和索引更新，不做打印、不加延时；
 *       队列满时整帧丢弃（不覆盖旧帧），丢弃计入 comm_drop_frames 供现场判断上位机发得太快。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t next;

    if (huart->Instance != USART2)
    {
        return;     /* 其它串口不归本模块处理，避免误吞回调 */
    }

    if (Size > 0U)
    {
        /* 环形队列“生产”指针预推进：留一个空槽区分空/满，故满时 next == tail */
        next = (uint8_t)((q_head + 1U) % COMM_QUEUE_LEN);
        if (next == q_tail)
        {
            comm_drop_frames++;   /* 队列满，丢弃本帧 */
        }
        else
        {
            if (Size > COMM_RX_SIZE) { Size = COMM_RX_SIZE; }   /* 防御：不超过 DMA 缓冲长度 */
            memcpy(frame_buf[q_head], rx_dma_buf, Size);        /* 立即搬走，DMA 缓冲马上要被重新挂接 */
            frame_len[q_head] = Size;
            q_head = next;
            comm_rx_frames++;
        }
    }

    Comm_StartReceive();   /* 立刻重新挂接收，不丢下一帧 */
}

/* 接收出错（溢出/帧错/噪声/校验）后恢复接收，避免通道停摆
 * 参数：huart = 触发错误的串口句柄（只处理 USART2）
 * 返回：无
 * 调用：HAL 在 USART2 检测到 ORE/NE/FE/PE 时回调，中断上下文
 * 注意：必须清 ORE 并复位错误码，否则 HAL 认为串口仍处于错误态，
 *       后续 ReceiveToIdle_DMA 会直接返回 BUSY；清错后重新挂接收。
 */
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

/* 发送一帧（DMA，调用后立即返回；缓冲需保持到发送完成）
 * 参数：data = 待发送数据首地址，不可为 NULL
 *       len  = 字节数，1 ~ 65535；RS232 侧实际受上位机接收能力限制
 * 返回：HAL_OK 已启动 DMA；HAL_BUSY 上一帧仍在发送；HAL_ERROR 参数非法
 * 调用：主循环（Comm_SendStatus）及后续协议层
 * 注意：函数返回只代表 DMA 已启动，不代表数据已全部移出；
 *       传入的 data 必须一直有效到发送完成（因此回传时先拷进 tx_dma_buf）。
 */
HAL_StatusTypeDef Comm_Send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;    /* 发送通道忙，交给调用者在下一轮重试 */
    }

    return HAL_UART_Transmit_DMA(&huart2, data, len);
}

/* 主循环任务：把收到的帧原样回传，一次只送一帧，避免通道冲突
 * 参数：无
 * 返回：无
 * 调用：主循环（main.c while(1)，每轮调用）
 * 注意：非阻塞。发送忙时保留 q_tail 不动，下一轮再试，因此队列起到“削峰”作用；
 *       回传关闭（g_cfg.echo_enable == 0）时直接丢帧，防止队列长期占满。
 */
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

    /* 拷到独立发送缓冲：DMA 传输期间队列槽位可能被中断写者回收复用，
     * 同时把长度截断到发送缓冲大小，避免越界 */
    len = frame_len[q_tail];
    if (len > COMM_TX_SIZE) { len = COMM_TX_SIZE; }
    memcpy(tx_dma_buf, frame_buf[q_tail], len);

    if (HAL_UART_Transmit_DMA(&huart2, tx_dma_buf, len) == HAL_OK)
    {
        q_tail = (uint8_t)((q_tail + 1U) % COMM_QUEUE_LEN);   /* 只有启动成功才出队，失败下轮重试 */
        comm_tx_frames++;
    }
}

/* 修改波特率：停接收 -> 重配 USART2 -> 重新挂接收
 * 参数：baud = 目标波特率，本项目取值范围 9600 ~ 115200（8-N-1 由 MX_USART2_UART_Init 保证）
 * 返回：无；重配失败时直接返回，g_cfg.baudrate 保持原值
 * 调用：上电初始化（main.c，EEPROM 配置加载之后、主循环之前）
 * 注意：必须先停接收再 HAL_UART_Init：
 *       1) HAL_UART_Init 会先关闭 USART 再改 BRR，若 DMA 接收仍挂着，
 *          旧配置的 DMA 请求/RxState 会残留，之后的 ReceiveToIdle_DMA 会返回 BUSY；
 *       2) 主动 Abort 也能丢弃可能存在的半帧，避免旧波特率下的残帧被当成完整帧。
 *       本函数不等待发送完成，调用前应保证发送通道空闲（当前仅上电阶段调用）。
 */
void Comm_SetBaud(uint32_t baud)
{
    (void)HAL_UART_AbortReceive(&huart2);   /* 停 DMA 接收并把 RxState 复位为 READY */

    huart2.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return;     /* 重配失败：不记录新波特率，保持原配置继续运行 */
    }

    g_cfg.baudrate = baud;   /* 只有重配成功才回写配置，避免掉电保存错误的波特率 */
    Comm_StartReceive();     /* 用新波特率重新挂起接收 */
}

/* 清零收发统计计数（comm_rx_frames / comm_tx_frames / comm_drop_frames）
 * 参数：无
 * 返回：无
 * 调用：调试时由上位机命令或初始化流程调用（主循环上下文）
 * 注意：只清软件统计，不影响队列内容、DMA 状态与串口配置。
 */
void Comm_ClearCounters(void)
{
    comm_rx_frames = 0;
    comm_tx_frames = 0;
    comm_drop_frames = 0;
}

/*--------------------- 输入状态打印 ---------------------*/
static uint8_t  s_status_buf[128];    /* 状态帧拼装缓冲，main 循环独占，不可重入 */
static uint32_t s_status_tick;        /* 上次状态帧发送时刻（HAL_GetTick，1ms 单位） */

/* 把 C 字符串追加到 s_status_buf 的 pos 位置
 * 参数：pos = 当前写入偏移的指针，写完后自动前移；s = 以 '\0' 结尾的字符串
 * 返回：无
 * 调用：Comm_SendStatus（主循环）
 * 注意：不做边界检查，由调用者保证 s_status_buf（128 字节）不溢出；
 *       按 10 路输入 + MODE/HIGH 字段估算，最大约 110 字节，留有余量。
 */
static void Status_PutStr(uint16_t *pos, const char *s)
{
    while (*s != '\0')
    {
        s_status_buf[*pos] = (uint8_t)*s;
        (*pos)++;
        s++;
    }
}

/* 把无符号整数按十进制追加到 s_status_buf（无前导零）
 * 参数：pos = 当前写入偏移的指针；v = 待转换值 0 ~ 4294967295
 * 返回：无
 * 调用：Comm_SendStatus（主循环），用于拼接 IN 序号、电平、高电平计数
 * 注意：先取最低位逆序存入 tmp，再反序输出；32 位最多 10 位，tmp[11] 足够。
 */
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

/* 立即组一帧输入状态文本并发送
 * 参数：无
 * 返回：无
 * 调用：主循环（由 Comm_StatusTask 周期调用，也可调试时直接调用）
 * 注意：格式 "IN1=0 ... IN10=1 MODE=SET HIGH=1\r\n"；
 *       work_mode 越界时按 INVALID 显示；Comm_Send 忙时本帧直接丢弃，不做重试。
 */
void Comm_SendStatus(void)
{
    static const char * const mode_name[4] = { "SET", "WORK", "DEBUG", "INVALID" };
    uint16_t pos = 0U;
    uint8_t i;
    uint8_t mode = g_cfg.work_mode;

    if (mode > WORK_MODE_INVALID)
    {
        mode = WORK_MODE_INVALID;   /* 防止数组越界，异常模式统一显示 INVALID */
    }

    for (i = 1U; i <= DI_NUM; i++)   /* IN 通道号从 1 开始，与原理图/菜单一致 */
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

/* 周期任务：每 COMM_STATUS_PERIOD_MS 输出一次串口状态帧
 * 说明：输入通道的 RTT 打印已关闭（改为由 menu.c 通过 [UI] 镜像 OLED 页面）。
 * 参数：无
 * 返回：无
 * 调用：主循环（main.c while(1)，每轮调用，非阻塞）
 * 注意：以“上次发送时刻”为基准，周期误差不超过一个主循环循环时间；
 *       关闭打印期间仍然推进 s_status_tick，重新打开后最多再等一个周期才出帧。
 */
void Comm_StatusTask(void)
{
    uint32_t now = HAL_GetTick();

    if (now == s_status_tick)
    {
        return;     /* 同一毫秒内重复调用，避免重复发送 */
    }

    if ((now - s_status_tick) < COMM_STATUS_PERIOD_MS)
    {
        return;
    }

    s_status_tick = now;

    if (g_cfg.status_print_enable == 0U)
    {
        return;     /* 打印关闭：只推进计时，不组帧 */
    }

    Comm_SendStatus();
}
