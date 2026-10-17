#include "bsp_usart.hpp"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;

/*---------------------------接收上下文---------------------------*/
// 用 "DMA + 空闲中断" 收不定长帧：
//   DMA 负责把字节搬进缓冲区，硬件 IDLE 中断负责判定"这一帧收完了"。
// 每个串口记住"当前武装在哪个缓冲区、多大、上一帧多长"，回调里才有依据。
static uint8_t usart1_rx_buf[USART_RX_BUF_SIZE];
static uint8_t usart6_rx_buf[USART_RX_BUF_SIZE];

static uint8_t *usart1_rx_ptr = usart1_rx_buf;
static uint8_t *usart6_rx_ptr = usart6_rx_buf;

static volatile uint16_t usart1_rx_size = USART_RX_BUF_SIZE;
static volatile uint16_t usart6_rx_size = USART_RX_BUF_SIZE;

static volatile uint16_t usart1_rx_len = 0;
static volatile uint16_t usart6_rx_len = 0;

/**
 * @brief 重新武装一次 DMA 接收。
 * @param huart 串口句柄
 * @param buf 接收缓冲区首地址
 * @param size 缓冲区大小
 */
static void USART_StartReceive(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    if (huart == nullptr || buf == nullptr || size == 0)
        return;

    if (huart == &huart1)
    {
        usart1_rx_ptr = buf;
        usart1_rx_size = size;
    }
    else if (huart == &huart6)
    {
        usart6_rx_ptr = buf;
        usart6_rx_size = size;
    }

    // ReceiveToIdle_DMA 内部会同时打开"半传输(HT)"和"传输完成(TC)"中断，
    // 半帧也会进一次回调。这里把 HT 关掉，保证一次回调 = 一整帧。
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, size) != HAL_OK)
        return;
    if (huart->hdmarx != nullptr)
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void USART_Init(void)
{
    USART1_Init();
    USART6_Init();
}

void USART1_Init(void)
{
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx); ///< 把 DMA 句柄挂到串口上，之后 Transmit_DMA 才找得到
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
    USART_StartReceive(&huart1, usart1_rx_buf, USART_RX_BUF_SIZE);
}

void USART6_Init(void)
{
    __HAL_LINKDMA(&huart6, hdmatx, hdma_usart6_tx);
    __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);
    USART_StartReceive(&huart6, usart6_rx_buf, USART_RX_BUF_SIZE);
}

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
    if (huart == nullptr || pData == nullptr || Size == 0)
        return;

    switch (mode)
    {
    case USART_MODE_DMA:
        HAL_UART_Transmit_DMA(huart, pData, Size);
        break;
    case USART_MODE_IT:
        HAL_UART_Transmit_IT(huart, pData, Size);
        break;
    case USART_MODE_BLOCK:
    default:
        // 阻塞发送给一个超时上限，避免对方不取数据时把控制线程挂死在这里
        HAL_UART_Transmit(huart, pData, Size, 100);
        break;
    }
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    // 接收统一走 "DMA + 空闲中断"：调用方只提供缓冲区，帧的边界由 IDLE 判定
    USART_StartReceive(huart, pData, Size);
}

uint16_t USART_GetRxLength(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        return usart1_rx_len;
    if (huart == &huart6)
        return usart6_rx_len;
    return 0;
}

uint8_t *USART_GetRxBuffer(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        return usart1_rx_ptr;
    if (huart == &huart6)
        return usart6_rx_ptr;
    return nullptr;
}

/**
 * @brief 整帧到达回调弱定义：不实现时数据被丢弃，但接收会继续武装。
 */
__weak void USART_RxIdleCallback(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart;
    (void)pData;
    (void)Size;
}

/**
 * @brief DMA + 空闲中断的整帧回调（HAL 在 IDLE 或 DMA 传输完成时调用）。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t *buf = nullptr;
    uint16_t size = 0;

    if (huart == &huart1)
    {
        usart1_rx_len = Size;
        buf = usart1_rx_ptr;
        size = usart1_rx_size;
    }
    else if (huart == &huart6)
    {
        usart6_rx_len = Size;
        buf = usart6_rx_ptr;
        size = usart6_rx_size;
    }
    else
    {
        return;
    }

    if (Size > 0 && buf != nullptr)
        USART_RxIdleCallback(huart, buf, Size);

    // 一帧处理完立刻重新武装，否则下一个字节开始就丢数据
    USART_StartReceive(huart, buf, size);
}

/**
 * @brief 串口错误回调：溢出/帧错误会让接收停摆，这里无条件重新武装。
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        USART_StartReceive(&huart1, usart1_rx_ptr, usart1_rx_size);
    else if (huart == &huart6)
        USART_StartReceive(&huart6, usart6_rx_ptr, usart6_rx_size);
}
