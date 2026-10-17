#ifndef BSP_USART_HPP
#define BSP_USART_HPP

#include "main.h"
#include "stm32f4xx_hal_uart.h"

enum USART_Mode
{
    USART_MODE_BLOCK = 0,
    USART_MODE_DMA = 1,
    USART_MODE_IT = 2
  };

/** 单个串口接收缓冲区大小（不定长帧上限） */
#define USART_RX_BUF_SIZE 64

/**
 * @brief 初始化串口。
 */
void USART_Init(void);

/**
 * @brief 初始化串口1。
 * @note 串口1用于调试信息输出。使用DMA发送和接收。
 */
void USART1_Init(void);

/**
 * @brief 初始化串口6。
 * @note 串口6用于和裁判系统通信。使用DMA发送和接收。
 */
void USART6_Init(void);

/**
 * @brief 串口发送数据。
 * @param huart 指向串口句柄的指针
 * @param pData 发送的数据
 * @param Size 数据长度
 * @param mode 发送模式
 */
void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode);

/**
 * @brief 串口接收数据。
 * @param huart 指向串口句柄的指针
 * @param pData 接收数据的缓冲区
 * @param Size 数据长度
 */
void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief 取最近一帧（空闲中断判定结束）的长度。
 * @param huart 串口句柄
 * @return 最近一帧的字节数；0 表示还没有收到完整帧
 */
uint16_t USART_GetRxLength(UART_HandleTypeDef *huart);

/**
 * @brief 取接收缓冲区首地址。
 * @param huart 串口句柄
 * @return 缓冲区指针；未初始化的串口返回 nullptr
 */
uint8_t *USART_GetRxBuffer(UART_HandleTypeDef *huart);

/**
 * @brief 整帧到达回调（弱定义，由上层实现接管）。
 * @param huart 触发空闲中断的串口
 * @param pData 接收缓冲区
 * @param Size 本帧字节数
 * @note 在中断上下文被调用，里面不要做耗时操作。
 */
void USART_RxIdleCallback(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#endif //  __BSP_USART_H
