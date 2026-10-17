#include "threads.hpp"
#include "BMI088.hpp"
#include "main.h"

/*--------------------------- 线程与栈 ---------------------------*/
TX_THREAD imu_thread;
static uint8_t imu_thread_stack[2048] = {0};

/** 对外发布的 IMU 数据快照 */
volatile imu_snapshot_t g_imu = {};

static BMI088::cBMI088 bmi088;

/** 目标温度：BMI088 的加热电阻用来把芯片维持在一个稳定温度，减少零漂 */
#define IMU_TARGET_TEMPERATURE 40.0f

[[noreturn]] static void imu_thread_entry(ULONG initial_input)
{
    UNUSED(initial_input);

    /*---------------- 初始化：配置 → 校验 ID → 标定 ----------------*/
    bmi088.Config();

    bmi088.VerifyAccChipID();
    bmi088.VerifyGyroChipID();

    if (!bmi088.self_test.INIT_ERR)
    {
        bmi088.Calibrate(); ///< 标定期间整车必须静止，否则零偏不可信
        g_init_flags |= INIT_FLAG_IMU_OK;
    }

    for (;;)
    {
        acc_data_t acc{};
        gyro_data_t gyro{};
        float temperature = 0.0f; ///< 先用临时变量接：g_imu 是 volatile，不能取它成员的地址

        bmi088.ReadAccData(&acc);
        bmi088.ReadGyroData(&gyro);
        bmi088.ReadAccTemperature(&temperature);

        // 用芯片自己的 data-ready 位判断这一轮数据是否新鲜
        bmi088.VerifyAccData();
        bmi088.VerifyGyroData();

        g_imu.acc_x = acc.x;
        g_imu.acc_y = acc.y;
        g_imu.acc_z = acc.z;
        g_imu.gyro_x = gyro.x;
        g_imu.gyro_y = gyro.y;
        g_imu.gyro_z = gyro.z;
        g_imu.temperature = temperature;
        g_imu.valid = (bmi088.self_test.ACC_DATA_ERR || bmi088.self_test.GYRO_DATA_ERR) ? 0u : 1u;

        // 温度闭环：把芯片稳定在目标温度（内部会做限幅和"读不到就不加热"的保护）
        bmi088.TemperatureControl(IMU_TARGET_TEMPERATURE);

        // 心跳：告诉 alive 线程"这一轮我干完了"
        tx_semaphore_put(&imu_heartbeat_sem);

        // 1kHz 左右的采样节拍（tick 频率决定）
        tx_thread_sleep(1);
    }
}

void ImuThread_Create(void)
{
    tx_thread_create(&imu_thread,
                     const_cast<CHAR *>("imu"),
                     imu_thread_entry,
                     0,
                     imu_thread_stack, sizeof(imu_thread_stack),
                     8, 8, ///< 优先级高于控制线程：姿态数据要先就绪
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}
