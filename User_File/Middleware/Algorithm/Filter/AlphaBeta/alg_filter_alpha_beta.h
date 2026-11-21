/**
 * @file alg_filter_alpha_beta.h
 * @brief 固定周期和变步长标量 Alpha-Beta 滤波器
 */

#ifndef ALG_FILTER_ALPHA_BETA_H
#define ALG_FILTER_ALPHA_BETA_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 基于常速度模型估计位置和速度。
 * @details 位置和测量值同单位 U，速度为 U/s，时间常数和步长为 s。
 * 时间常数越小响应越快，越大越平滑。对于周期采样，每次 Set_Now 后计算一次。
 * 对于异步采样，Update 接收距上次接受的测量值经过的时间。同一实例由单一上下文调用。
 * Init 或无参 Reset 后，首个有效测量直接确定位置，速度置零。
 * 带参 Reset 则直接设置有效状态，后续测量从该位置和速度继续预测。
 */
class Class_Filter_Alpha_Beta
{
public:
    bool Init(float tau, float d_t = 0.001f);

    bool Init_Gain(float alpha, float beta, float d_t = 0.001f);

    void Reset();

    void Reset(float position, float velocity = 0.0f);

    inline void Set_Now(float now);

    void TIM_Calculate_PeriodElapsedCallback();

    bool Update(float measurement, float d_t);

    float Predict_Position(float delta_t) const;

    inline float Get_Out() const;

    inline float Get_Position() const;

    inline float Get_Velocity() const;

    inline bool Get_Initialized_Flag() const;

private:
    bool Update_State(float measurement, float d_t, float alpha, float velocity_gain);

    float Tau = 0.0f;  // 零表示手动增益模式。
    float D_T = 0.0f;  // 零表示尚未配置。
    float Alpha = 0.0f;
    float Beta = 0.0f;
    float Velocity_Gain = 0.0f;  // 固定周期的 Beta / D_T。

    float Now = NAN;  // 等待 Set_Now 提供周期测量。
    float Position = 0.0f;
    float Velocity = 0.0f;
    bool Initialized_Flag = false;
};

/* Exported function declarations --------------------------------------------*/

inline void Class_Filter_Alpha_Beta::Set_Now(float now)
{
    Now = now;
}

inline float Class_Filter_Alpha_Beta::Get_Out() const
{
    return (Position);
}

inline float Class_Filter_Alpha_Beta::Get_Position() const
{
    return (Position);
}

inline float Class_Filter_Alpha_Beta::Get_Velocity() const
{
    return (Velocity);
}

inline bool Class_Filter_Alpha_Beta::Get_Initialized_Flag() const
{
    return (Initialized_Flag);
}

#endif
