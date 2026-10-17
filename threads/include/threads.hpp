#ifndef THREADS_HPP
#define THREADS_HPP

#include <cstdint>
#include "tx_api.h"

/*--------------------------- 线程对象与栈 ---------------------------*/
// 栈都放在静态数组里：ThreadX 允许这样创建，省得动内存池的分配
extern TX_THREAD alive_thread;
extern TX_THREAD imu_thread;
extern TX_THREAD control_thread;

/*--------------------------- 心跳信号量 ---------------------------*/
// 约定：被监控的线程每跑完一轮就 put 一次；alive 线程 get 得到就说明它活着。
// 这是"用信号量做进程监控"的核心机制——比共享一个 flag 好在：
// 计数由内核维护，不需要额外的互斥保护。
extern TX_SEMAPHORE imu_heartbeat_sem;
extern TX_SEMAPHORE control_heartbeat_sem;

/*--------------------------- 初始化状态 ---------------------------*/
// bit0 = IMU 初始化成功，bit1 = 电机初始化成功（对应控制线程注册完成）
#define INIT_FLAG_IMU_OK 0x01u
#define INIT_FLAG_MOTOR_OK 0x02u
extern volatile uint8_t g_init_flags;

/*--------------------------- 对其它线程开放的数据 ---------------------------*/
// IMU 线程写、控制/监控线程读；单个 float 的读写本身是原子的，够用不必加锁
typedef struct
{
    float acc_x, acc_y, acc_z; ///< 加速度 [m/s²]
    float gyro_x, gyro_y, gyro_z; ///< 角速度 [rad/s]
    float temperature;         ///< 芯片温度 [℃]
    uint8_t valid;             ///< 本轮数据是否通过校验
} imu_snapshot_t;

extern volatile imu_snapshot_t g_imu;

/*--------------------------- 控制接口 ---------------------------*/
enum ControlMode
{
    CONTROL_RELAX = 0, ///< 松开：不给电流（上电默认，安全）
    CONTROL_SPEED = 1, ///< 单速度环
    CONTROL_POSITION = 2 ///< 位置+速度双环
};

void Control_SetMode(enum ControlMode mode);
void Control_SetTargetSpeed(float rad_per_s);
void Control_SetTargetPosition(float rad);

/*--------------------------- 生命周期 ---------------------------*/
/**
 * @brief 创建全部线程与信号量。
 * @note 在 tx_application_define()（Core/Src/app_threadx.c）里调用一次。
 */
void Threads_Init(void);

/**
 * @brief 设置三色灯。
 * @param r/g/b true = 亮
 */
void LED_Set(bool r, bool g, bool b);

#endif // THREADS_HPP
