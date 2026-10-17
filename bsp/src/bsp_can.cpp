#include "bsp_can.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st;                   ///< 定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位

    can_filter_st.FilterBank = 0;                                      ///< 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                 ///< 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                             ///< 使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断

    can_filter_st.SlaveStartFilterBank = 14;                           ///< 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                     ///< 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                             ///< 使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断
}

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    // 参数校验：任何一条不满足都直接放弃这一帧。
    // 宁可不发，也不要把垃圾数据丢上总线——总线上挂着的是电机。
    if (hcan == nullptr || msg == nullptr)
        return;
    if (len == 0 || len > 8) ///< 标准帧数据场最长 8 字节
        return;
    if (Id > 0x7FF)          ///< 本函数只支持标准帧（11 位 ID）
        return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) ///< 三个发送邮箱都占满时不再等待，避免阻塞控制线程
        return;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = Id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t used_mailbox;
    HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &used_mailbox);
}

/**
 * @brief 接收分发回调弱定义。
 * @note 上层（module 层）实现同名函数即可接管，不实现时数据被丢弃。
 */
__weak void CAN_RxCallback(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data)
{
    (void)hcan;
    (void)StdId;
    (void)data;
}

/**
 * @brief CAN RX FIFO0 非空中断的 HAL 回调。
 * @note 中断里只做"取帧 + 交给上层"两件事，解析和运算留给线程/上层，
 *       保证中断服务时间尽可能短。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return; ///< 取帧失败（FIFO 已被读空等），直接放弃本次回调

    if (rx_header.IDE != CAN_ID_STD)
        return; ///< 目前只处理标准帧

    CAN_RxCallback(hcan, rx_header.StdId, rx_data);
}
