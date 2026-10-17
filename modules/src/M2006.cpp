#include "M2006.hpp"
#include "math.hpp"

/*---------------------------M2006 + C610 参数---------------------------*/
// 减速比：M2006 P36 减速箱，转子:输出轴 = 36:1
// 所有"输出轴"的量（位置/速度）都要除以它
#define M2006_GEAR_RATIO 36.0f

// 电流：C610 电流指令范围 -10000~10000 对应 ±10A，这里按满量程给上限
#define M2006_MAX_CURRENT 10000.0f

// 速度环输出的是电流（mA 量级，直接对齐 C610 的指令刻度）
// 位置环输出的是速度给定（rad/s），maxOut 就是允许的最大输出轴转速
#define M2006_SPEED_KP 12.0f
#define M2006_SPEED_KI 0.20f
#define M2006_SPEED_KD 0.0f
#define M2006_SPEED_I_LIMIT 2000.0f

#define M2006_POSITION_KP 8.0f
#define M2006_POSITION_KI 0.0f
#define M2006_POSITION_KD 0.30f
#define M2006_POSITION_MAX_SPEED 60.0f ///< 位置环输出的速度上限 [rad/s]
#define M2006_POSITION_I_LIMIT 0.0f

M2006::M2006()
{
    // 参数先在构造函数里定好，运行中还可以用 Tuning() 在线改
    gearBox = GearBox_M2006;
    maxCurrent = (uint16_t)M2006_MAX_CURRENT;

    speedPid = PID(M2006_SPEED_KP, M2006_SPEED_KI, M2006_SPEED_KD,
                   M2006_MAX_CURRENT, M2006_SPEED_I_LIMIT, PID_POSITION);
    positionPid = PID(M2006_POSITION_KP, M2006_POSITION_KI, M2006_POSITION_KD,
                      M2006_POSITION_MAX_SPEED, M2006_POSITION_I_LIMIT, PID_POSITION);

    // 上电默认松开：没有收到明确指令前绝不给电流
    controlMode = RELAX_MODE;
    currentSet = 0;
    speedSet = 0.0f;
    positionSet = 0.0f;
}

void M2006::setOutput()
{
    switch (controlMode)
    {
    case SPD_MODE:
        speedPid.ref = speedSet;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        break;

    case POS_MODE:
    {
        // 串级双环：外环（位置）算出"该用多大速度过去"，作为内环（速度）的给定。
        // 内环必须比外环快，否则外环的输出还没被执行完就被下一次覆盖，表现为抖动。
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();

        speedPid.ref = Numeric::LimitABS(positionPid.result, M2006_POSITION_MAX_SPEED);
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        break;
    }

    case RELAX_MODE:
    default:
        // 松开模式：清积分、输出置零，避免下次切回速度/位置模式时带着旧积分冲出去
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        return;
    }

    // 无论哪个环，最终都要落到电流指令上，并且不能超过电调允许的电流
    currentSet = (int16_t)Numeric::LimitABS(speedPid.result, M2006_MAX_CURRENT);
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // 反馈帧每收到一次 AliveFlag 就 +1（见 DJIMotorHandler::UpdateSensorData），
    // 所以"和上一轮相比没变化"就说明这一轮没收到任何反馈 → 离线
    if (AliveFlag == Pre_AliveFlag)
    {
        MotorState = MOTOR_OFFLINE;
    }
    else
    {
        MotorState = MOTOR_ONLINE;
        Pre_AliveFlag = AliveFlag;
    }
    return MotorState;
}
