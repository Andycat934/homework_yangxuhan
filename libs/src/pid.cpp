#include "pid.hpp"
#include "math.hpp"

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    // 误差按时间顺序移位：err[0] 本次、err[1] 上次、err[2] 上上次
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    if (mode == PID_DELTA)
    {
        // 增量式：输出的是"本次要增加多少"，而不是绝对量。
        // Δu = Kp·(e0−e1) + Ki·e0 + Kd·(e0−2e1+e2)
        // 好处：天然没有积分累加器，切换模式/被限幅后不会积累历史误差。
        pResult = kp * (err[0] - err[1]);
        iResult = ki * err[0];
        dResult = kd * (err[0] - 2.0f * err[1] + err[2]);

        result += pResult + iResult + dResult; // 在上一输出上累加，所以必须先限幅再累加
        result = Numeric::LimitABS(result, maxOut);
    }
    else // PID_POSITION
    {
        // 位置式：P、I、D 三项各自算完再相加。
        pResult = kp * err[0];

        // 积分项先累加再单独限幅：maxIOut 用来防止误差长期存在时积分"涨飞"
        // （积分饱和后即使误差反号，输出也要很久才能回头）。
        iResult += ki * err[0];
        iResult = Numeric::LimitABS(iResult, maxIOut);

        // 微分项用"本次误差 − 上次误差"做近似（离散化），不额外滤波。
        dResult = kd * (err[0] - err[1]);

        result = pResult + iResult + dResult;
    }

    // 最终输出限幅：无论哪种模式都不能超过执行机构允许的范围
    result = Numeric::LimitABS(result, maxOut);
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
