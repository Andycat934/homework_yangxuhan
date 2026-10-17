//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

/*---------------------------C 板的 PWM 资源---------------------------*/
// 开发板 C 型的 PWM 输出接口在 TIM1(CH1~4) 与 TIM8(CH1~3) 上，
// 时钟树配好后（PSC=167, ARR=19999）输出 50Hz，即周期 20ms：
//   舵机：高电平 0.5ms~2.5ms 对应角度行程
//   比较值：500  -> 0.5ms，2000 -> 2ms （1 个计数 = 20ms/20000 = 1us）
#define PWM_SERVO_TIM htim1
#define PWM_SERVO_CHANNEL TIM_CHANNEL_1

// 加热电阻：BMI088 自带加热电阻，用 TIM10_CH1 的 PWM 控制功率（见 BMI088.hpp）
#define PWM_HEAT_TIM htim10
#define PWM_HEAT_CHANNEL TIM_CHANNEL_1

#define PWM_SERVO_MIN_COMPARE 500u  ///< 0.5ms
#define PWM_SERVO_MAX_COMPARE 2500u ///< 2.5ms

void PWM_Init(void)
{
    // 舵机通道先给最小脉宽再启动，避免上电瞬间输出一个大占空比把舵机甩到底
    __HAL_TIM_SET_COMPARE(&PWM_SERVO_TIM, PWM_SERVO_CHANNEL, PWM_SERVO_MIN_COMPARE);
    PWM_Start(&PWM_SERVO_TIM, PWM_SERVO_CHANNEL);

    // 加热电阻初始占空比为 0：温度没测到之前不允许加热
    PWM_SetDutyRatio(&PWM_HEAT_TIM, 0.0f, PWM_HEAT_CHANNEL);
    PWM_Start(&PWM_HEAT_TIM, PWM_HEAT_CHANNEL);
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (htim == nullptr)
        return;
    // HAL_TIM_PWM_Start 会返回 HAL_StatusTypeDef：失败时通道其实没输出，
    // 这里显式判断一下，免得后面"PWM 没反应"时找不到原因。
    if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
    {
        // 占位：可在此处点灯/记录，当前实现保持静默
    }
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if (htim == nullptr)
        return;
    HAL_TIM_PWM_Stop(htim, Channel);
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    if (htim == nullptr || period_s <= 0.0f)
        return;

    // 定时器计数时钟 = 定时器总线时钟 / (PSC + 1)
    // STM32F4 的 APB 预分频不为 1 时，定时器时钟是 APB 时钟的 2 倍，这里按寄存器实际值算。
    uint32_t pclk;
    TIM_TypeDef *inst = htim->Instance;
    if (inst == TIM1 || inst == TIM8 || inst == TIM9 || inst == TIM10 || inst == TIM11)
        pclk = HAL_RCC_GetPCLK2Freq();
    else
        pclk = HAL_RCC_GetPCLK1Freq();

    uint32_t tim_clk = (pclk == HAL_RCC_GetHCLKFreq()) ? pclk : pclk * 2u;
    uint32_t cnt_clk = tim_clk / (htim->Init.Prescaler + 1u);

    // ARR = 计数时钟 × 周期 − 1（+0.5 是四舍五入）
    uint32_t arr = (uint32_t)(period_s * (float)cnt_clk + 0.5f);
    if (arr == 0)
        arr = 1;
    __HAL_TIM_SET_AUTORELOAD(htim, arr - 1u);

    // 立即把影子寄存器内容装入，避免等下一个更新事件才生效
    htim->Instance->EGR = TIM_EGR_UG;
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    if (htim == nullptr)
        return;

    // 限幅到 [0, 1]：调用方（PID、温度控制）算出来的值不保证在范围内
    if (dutyratio < 0.0f)
        dutyratio = 0.0f;
    if (dutyratio > 1.0f)
        dutyratio = 1.0f;

    // 占空比 d 对应 CCR = d × (ARR + 1)：一个周期有 ARR+1 个计数
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t compare = (uint32_t)(dutyratio * (float)(arr + 1u) + 0.5f);
    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}
