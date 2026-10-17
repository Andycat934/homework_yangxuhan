#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "can.h"

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void);

/**
 * @brief 发送CAN数据。
 * @param hcan 指向CAN句柄的指针，用于配置CAN传输。
 * @param StdId CAN消息的标准标识符
 * @param msg 发送的数据
 * @param len 数据长度
 * @note 该函数用于发送CAN数据，目前只支持标准帧
 */
void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len);

/**
 * @brief CAN 接收分发回调（弱定义，由上层实现接管）。
 * @param hcan 收到数据的CAN总线
 * @param StdId 标准帧标识符
 * @param data 指向数据场的指针（长度由帧头决定，电机反馈固定 8 字节）
 * @note BSP 只负责"把帧收下来"，不关心数据是谁的；谁用谁在这个函数里接。
 *       这样依赖方向保持 modules → bsp 单向，BSP 不需要认识任何电机类。
 */
void CAN_RxCallback(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data);

#endif // BSP_CAN_HPP
