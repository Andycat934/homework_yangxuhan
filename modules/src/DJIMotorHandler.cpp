#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#include <cmath>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*---------------------------常量---------------------------*/
#define MOTOR_ID_FIRST 0x201u  ///< 反馈 ID 起点
#define MOTOR_ID_LAST 0x208u   ///< 反馈 ID 终点
#define ENCODER_RANGE 8192.0f  ///< 转子编码器一圈 8192 个计数（0~8191）
#define RPM_TO_RAD_S (2.0f * 3.14159265358979323846f / 60.0f)

/**
 * @brief 减速箱减速比（转子转多少圈 = 输出轴转 1 圈）
 */
static float GearRatioOf(GearBox gearBox)
{
    switch (gearBox)
    {
    case GearBox_M2006:
        return 36.0f;      ///< M2006 P36
    case GearBox_M3508:
        return 3591.0f / 187.0f; ///< M3508 行星减速箱
    case GearBox_None:
    case GearBox_XRoll:
    default:
        return 1.0f;
    }
}

/**
 * @brief 由 CAN 句柄判断是第几条总线：0 = CAN1，1 = CAN2，-1 = 不认识
 */
static int BusIndexOf(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
        return 0;
    if (hcan == &hcan2)
        return 1;
    return -1;
}

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    if (motor == nullptr || hcan == nullptr)
        return;

    // 只接受 0x201~0x208：C610/M3508 电调反馈 ID 的合法范围
    if (canId < MOTOR_ID_FIRST || canId > MOTOR_ID_LAST)
        return;

    int bus = BusIndexOf(hcan);
    if (bus < 0)
        return;

    int index = (int)(canId - MOTOR_ID_FIRST); ///< 槽位 = ID − 0x201
    if (DJIMotorList[bus][index] != nullptr)
        return; ///< 同一 ID 不允许注册两次，直接拒绝而不是悄悄覆盖

    motor->hcan = hcan;
    motor->canId = canId;
    DJIMotorList[bus][index] = motor;

    // 反馈 ID 0x201~0x204 由控制报文 0x200 驱动，0x205~0x208 由 0x1FF 驱动
    if (index < 4)
    {
        if (bus == 0)
            CAN1_0x200_Exist = true;
        else
            CAN2_0x200_Exist = true;
    }
    else
    {
        if (bus == 0)
            CAN1_0x1FF_Exist = true;
        else
            CAN2_0x1FF_Exist = true;
    }
}

void DJIMotorHandler::sendControlData()
{
    for (int bus = 0; bus < 2; bus++)
    {
        uint8_t *frame_0x200 = (bus == 0) ? can1_send_data_0 : can2_send_data_0;
        uint8_t *frame_0x1FF = (bus == 0) ? can1_send_data_1 : can2_send_data_1;

        // 每轮先清零：没注册的槽位必须是 0 电流，不能残留上一轮的旧值
        for (int i = 0; i < 8; i++)
        {
            frame_0x200[i] = 0;
            frame_0x1FF[i] = 0;
        }

        for (int index = 0; index < 8; index++)
        {
            DJIMotor *motor = DJIMotorList[bus][index];
            if (motor == nullptr)
                continue;

            // 电流是 int16，按大端序放进报文：高字节在前（与 C610 协议一致）
            int16_t current = motor->currentSet;
            uint8_t *frame = (index < 4) ? frame_0x200 : frame_0x1FF;
            int offset = (index % 4) * 2;
            frame[offset] = (uint8_t)((current >> 8) & 0xFF);
            frame[offset + 1] = (uint8_t)(current & 0xFF);
        }

        // 只发"确实有电机在里面"的报文：总线上没有的电机就不发，减少无谓流量
        CAN_HandleTypeDef *hcan = (bus == 0) ? &hcan1 : &hcan2;
        bool exist_0x200 = (bus == 0) ? CAN1_0x200_Exist : CAN2_0x200_Exist;
        bool exist_0x1FF = (bus == 0) ? CAN1_0x1FF_Exist : CAN2_0x1FF_Exist;

        if (exist_0x200)
            CAN_Transmit(hcan, 0x200, frame_0x200, 8);
        if (exist_0x1FF)
            CAN_Transmit(hcan, 0x1FF, frame_0x1FF, 8);
    }
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (rx_data == nullptr || index < 0 || index > 7)
        return;

    int bus = BusIndexOf(hcan);
    if (bus < 0)
        return;

    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor == nullptr)
        return; ///< 这一帧对应的槽位没有电机（比如电调 ID 撞车），忽略

    UpdateSensorData(motor, rx_data);
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    if (motor == nullptr || can_data == nullptr)
        return;

    DJIMotor::MotorFeedBack &fb = motor->motorFeedback;

    fb.last_ecd = (int16_t)fb.ecd;                                        ///< 保存上一次编码器值，用于算增量
    fb.ecd = (uint16_t)(((uint16_t)can_data[0] << 8) | can_data[1]);      ///< 转子机械角度 0~8191
    fb.speed_rpm = (int16_t)(((uint16_t)can_data[2] << 8) | can_data[3]); ///< 转子转速 [rpm]
    fb.currentFdb = (float)(int16_t)(((uint16_t)can_data[4] << 8) | can_data[5]); ///< 实际转矩电流
    fb.temperatureFdb = (float)can_data[6];                               ///< 温度 [℃]

    // 编码器回绕：ecd 在 0~8191 之间循环，直接从 8191 跳到 0 会得到 −8191 的假增量。
    // 超过半圈就按反向补偿一圈，得到真实的短弧增量。
    int16_t delta = (int16_t)(fb.ecd - fb.last_ecd);
    if (delta > (int16_t)(ENCODER_RANGE / 2.0f))
        delta -= (int16_t)ENCODER_RANGE;
    else if (delta < -(int16_t)(ENCODER_RANGE / 2.0f))
        delta += (int16_t)ENCODER_RANGE;

    // 转子 → 输出轴：位置累加、速度除以减速比
    float gear = GearRatioOf(motor->gearBox);
    float delta_rad = (float)delta * (2.0f * 3.14159265358979323846f / ENCODER_RANGE);

    fb.lastPositionFdb = fb.positionFdb;
    fb.positionFdb += delta_rad / gear;
    fb.lastSpeedFdb = fb.speedFdb;
    fb.speedFdb = (float)fb.speed_rpm * RPM_TO_RAD_S / gear;

    motor->AliveFlag++; ///< 收到一帧就 +1，AliveCheck 用它判断"这一轮有没有反馈"
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    for (int bus = 0; bus < 2; bus++)
    {
        for (int index = 0; index < 8; index++)
        {
            DJIMotor *motor = DJIMotorList[bus][index];
            if (motor != nullptr)
                motor->AliveCheck();
        }
    }
}

/**
 * @brief 接管 BSP 层收到的 CAN 帧。
 * @note 依赖方向是 modules → bsp：BSP 只发出"收到一帧"，由这里翻译成"哪台电机的数据"。
 */
void CAN_RxCallback(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data)
{
    if (StdId < MOTOR_ID_FIRST || StdId > MOTOR_ID_LAST)
        return; ///< 不是电调反馈帧（例如裁判系统/自定义报文），不在这里处理

    DJIMotorHandler::Instance()->updateFeedback(hcan, data, (int)(StdId - MOTOR_ID_FIRST));
}
