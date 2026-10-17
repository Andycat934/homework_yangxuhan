#include "threads.hpp"
#include "main.h"

/*--------------------------- 线程与栈 ---------------------------*/
TX_THREAD alive_thread;
static uint8_t alive_thread_stack[1024] = {0};

TX_SEMAPHORE imu_heartbeat_sem;
TX_SEMAPHORE control_heartbeat_sem;

volatile uint8_t g_init_flags = 0;

/*--------------------------- 三色灯 ---------------------------*/
// C 板三色灯：PH10 / PH11 / PH12（具体哪个是 R/G/B 以板子丝印为准，必要时对调宏）
#define LED_R_PORT GPIOH
#define LED_R_PIN GPIO_PIN_10
#define LED_G_PORT GPIOH
#define LED_G_PIN GPIO_PIN_11
#define LED_B_PORT GPIOH
#define LED_B_PIN GPIO_PIN_12

// C 板的灯是高电平点亮：引脚 → 1k/10k 分压 → NPN 三极管基极，高电平导通（教程 1.5.1）
// 因此初始电平配成 Low（灭），写 SET 才是点亮
#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET

static void LED_Write(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin, on ? LED_ON : LED_OFF);
}

void LED_Set(bool r, bool g, bool b)
{
    LED_Write(LED_R_PORT, LED_R_PIN, r);
    LED_Write(LED_G_PORT, LED_G_PIN, g);
    LED_Write(LED_B_PORT, LED_B_PIN, b);
}

/*--------------------------- 心跳检测 ---------------------------*/
/**
 * @brief 判断某个心跳信号量"这一轮有没有被 put 过"。
 * @param sem 心跳信号量
 * @param timeout_ticks 等待多久算超时
 * @return true = 还活着
 * @note 先把计数抽干再等：否则被监控线程跑得快时计数会堆积，
 *       即使它已经卡死，alive 线程也会因为"库存"而误判为存活。
 */
static bool HeartbeatAlive(TX_SEMAPHORE *sem, ULONG timeout_ticks)
{
    while (tx_semaphore_get(sem, TX_NO_WAIT) == TX_SUCCESS)
    {
        // 抽干历史计数
    }
    return (tx_semaphore_get(sem, timeout_ticks) == TX_SUCCESS);
}

/**
 * @brief alive 线程：监控初始化、IMU 线程、控制线程，并用灯效反映。
 *
 * 灯效约定：
 *   绿灯常亮   —— 初始化全部成功
 *   绿灯闪烁   —— 初始化未完成/失败
 *   蓝灯       —— IMU 线程存活
 *   红灯       —— 控制线程存活
 */
[[noreturn]] static void alive_thread_entry(ULONG initial_input)
{
    UNUSED(initial_input);

    const ULONG heartbeat_timeout = 20; ///< 20 个 tick 没心跳就算掉线
    uint32_t blink_counter = 0;

    for (;;)
    {
        const bool imu_alive = HeartbeatAlive(&imu_heartbeat_sem, heartbeat_timeout);
        const bool control_alive = HeartbeatAlive(&control_heartbeat_sem, heartbeat_timeout);

        const bool init_ok = (g_init_flags == (INIT_FLAG_IMU_OK | INIT_FLAG_MOTOR_OK));

        // 初始化成功 → 绿灯常亮；否则绿灯闪烁（1Hz 左右）
        blink_counter++;
        const bool green = init_ok || ((blink_counter % 10u) < 5u);

        LED_Set(!control_alive, green, imu_alive);

        tx_thread_sleep(10);
    }
}

void Threads_Init(void)
{
    // 心跳信号量初值 0：谁干活谁 put
    tx_semaphore_create(&imu_heartbeat_sem, const_cast<CHAR *>("imu_heartbeat"), 0);
    tx_semaphore_create(&control_heartbeat_sem, const_cast<CHAR *>("ctrl_heartbeat"), 0);

    extern void ImuThread_Create(void);
    extern void ControlThread_Create(void);

    ImuThread_Create();
    ControlThread_Create();

    tx_thread_create(&alive_thread,
                     const_cast<CHAR *>("alive"),
                     alive_thread_entry,
                     0,
                     alive_thread_stack, sizeof(alive_thread_stack),
                     10, 10, ///< 优先级最低：监控线程不该抢占控制线程
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}

/**
 * @brief 供 C 文件调用的入口。
 * @note CubeMX 生成的 Core/Src/app_threadx.c 是 C 文件，直接调 C++ 函数会因名字修饰
 *       （name mangling）链接失败，所以这里包一层 C 链接名的包装函数。
 *       C 侧只需写：extern void Threads_Init_C(void);
 */
extern "C" void Threads_Init_C(void)
{
    Threads_Init();
}
