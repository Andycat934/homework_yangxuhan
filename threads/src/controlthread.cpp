#include "threads.hpp"
#include "M2006.hpp"
#include "DJIMotorHandler.hpp"
#include "math.hpp"
#include "can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

/*--------------------------- 线程与栈 ---------------------------*/
TX_THREAD control_thread;
static uint8_t control_thread_stack[2048] = {0};

/** 本任务控制的单台电机：M2006 + C610，反馈 ID 0x201（= 控制报文 0x200 的第 1 槽） */
static M2006 motor;

/*--------------------------- 控制目标 ---------------------------*/
// 默认松开：确认电机空载固定、手边有断电开关之后，再通过 Control_SetMode 打开
static volatile enum ControlMode s_mode = CONTROL_RELAX;
static volatile float s_target_speed = 0.0f;
static volatile float s_target_position = 0.0f;

void Control_SetMode(enum ControlMode mode)
{
    s_mode = mode;
}

void Control_SetTargetSpeed(float rad_per_s)
{
    s_target_speed = rad_per_s;
}

void Control_SetTargetPosition(float rad)
{
    s_target_position = rad;
}

[[noreturn]] static void control_thread_entry(ULONG initial_input)
{
    UNUSED(initial_input);

    /*---------------- 注册电机 ----------------*/
    // 注册之后 DJIMotorHandler 才知道 0x201 这台电机的反馈该给谁、控制帧要不要发
    motor.controlMode = DJIMotor::RELAX_MODE;
    DJIMotorHandler::Instance()->registerMotor(&motor, &hcan1, 0x201);
    g_init_flags |= INIT_FLAG_MOTOR_OK;

    for (;;)
    {
        /*---------------- 1. 设定模式与目标 ----------------*/
        motor.controlMode = (DJIMotor::MotorControlModeType)s_mode;
        motor.speedSet = s_target_speed;
        motor.positionSet = s_target_position;

        /*---------------- 2. 双环计算 → currentSet ----------------
         * 串级结构（位置模式时）：
         *     位置环(P)   fdb = 输出轴角度        → 给出速度给定
         *     速度环(PI)  fdb = 输出轴角速度      → 给出电流指令
         * 两个环的具体实现在 M2006::setOutput() 里，这里只负责喂目标、喂模式。
         */
        motor.setOutput();

        /*---------------- 3. 把电流发上总线 ----------------*/
        // 这一步必须在算完之后调用：它把所有已注册电机的 currentSet 打包成 0x200/0x1FF 帧
        DJIMotorHandler::Instance()->sendControlData();

        /*---------------- 4. 在线检查 ----------------*/
        // 连续收不到反馈就判定掉线；掉线的电机不应该再被给电流
        if (motor.AliveCheck() == DJIMotor::MOTOR_OFFLINE)
        {
            motor.controlMode = DJIMotor::RELAX_MODE;
            motor.currentSet = 0;
        }

        // 心跳：告诉 alive 线程"这一轮我干完了"
        tx_semaphore_put(&control_heartbeat_sem);

        tx_thread_sleep(1);
    }
}

void ControlThread_Create(void)
{
    tx_thread_create(&control_thread,
                     const_cast<CHAR *>("control"),
                     control_thread_entry,
                     0,
                     control_thread_stack, sizeof(control_thread_stack),
                     9, 9, ///< 略低于 imu：先保证传感器数据新鲜，再做控制
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}
