/**
 * @file alg_filter_alpha_beta.cpp
 * @brief 固定周期和变步长标量 Alpha-Beta 滤波器
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_alpha_beta.h"

/* Private functions ---------------------------------------------------------*/

static void Alpha_Beta_Calculate_Gains(float tau, float d_t, float *alpha, float *beta)
{
    const float step = d_t / tau;
    // 计算 1 - exp(-step)。小步长使用泰勒展开，避免相近数相减丢失增益。
    const float pole_distance = step < 0.01f
                                    ? step * (1.0f - step * (0.5f - step * (1.0f / 6.0f - step / 24.0f)))
                                    : 1.0f - expf(-step);
    *alpha = pole_distance * (2.0f - pole_distance);
    *beta = pole_distance * pole_distance;
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 以临界阻尼时间常数配置滤波器，成功后清空状态。
 * @param tau 观测器时间常数，正有限值，单位 s。
 * @param d_t 固定采样周期，正有限值，单位 s。
 * @return 参数有效时为 true，失败时保留原配置和状态。
 */
bool Class_Filter_Alpha_Beta::Init(float tau, float d_t)
{
    if (Basic_Math_Is_Invalid_Float(tau) || Basic_Math_Is_Invalid_Float(d_t) ||
        tau <= 0.0f || d_t <= 0.0f)
    {
        return (false);
    }

    Tau = tau;
    D_T = d_t;
    Alpha_Beta_Calculate_Gains(tau, d_t, &Alpha, &Beta);
    Velocity_Gain = Beta / D_T;
    Reset();
    return (true);
}

/**
 * @brief 使用手动增益配置滤波器，成功后清空状态。
 * @param alpha 位置校正增益，范围 (0, 1]。
 * @param beta 速度校正增益，范围 [0, 2]。
 * @param d_t 固定采样周期，正有限值，单位 s。
 * @return 参数有效时为 true，失败时保留原配置和状态。
 */
bool Class_Filter_Alpha_Beta::Init_Gain(float alpha, float beta, float d_t)
{
    if (Basic_Math_Is_Invalid_Float(alpha) || Basic_Math_Is_Invalid_Float(beta) ||
        Basic_Math_Is_Invalid_Float(d_t) || alpha <= 0.0f || alpha > 1.0f ||
        beta < 0.0f || beta > 2.0f || d_t <= 0.0f)
    {
        return (false);
    }

    Tau = 0.0f;
    D_T = d_t;
    Alpha = alpha;
    Beta = beta;
    Velocity_Gain = beta / d_t;
    Reset();
    return (true);
}

/** @brief 清空样本历史和估计，等待首个有效测量，保留配置。 */
void Class_Filter_Alpha_Beta::Reset()
{
    Now = NAN;
    Position = 0.0f;
    Velocity = 0.0f;
    Initialized_Flag = false;
}

/** @brief 硬复位到已知状态。初始化标志置位，下一帧执行常规预测与更新。 */
void Class_Filter_Alpha_Beta::Reset(float position, float velocity)
{
    if (!isfinite(position) || !isfinite(velocity))
    {
        return;
    }
    Now = NAN;
    Position = position;
    Velocity = velocity;
    Initialized_Flag = true;
}

void Class_Filter_Alpha_Beta::TIM_Calculate_PeriodElapsedCallback()
{
    if (D_T > 0.0f && isfinite(Now))
    {
        (void)Update_State(Now, D_T, Alpha, Velocity_Gain);
    }
}

/**
 * @brief 以实际采样间隔更新，固定周期配置保持不变。
 * @return 已配置且接受该样本时为 true，失败时保留原估计。
 */
bool Class_Filter_Alpha_Beta::Update(float measurement, float d_t)
{
    if (D_T == 0.0f || !isfinite(measurement) || !isfinite(d_t) || d_t <= 0.0f)
    {
        return (false);
    }

    float alpha = Alpha;
    float beta = Beta;
    if (Tau > 0.0f)
    {
        Alpha_Beta_Calculate_Gains(Tau, d_t, &alpha, &beta);
    }
    return (Update_State(measurement, d_t, alpha, beta / d_t));
}

bool Class_Filter_Alpha_Beta::Update_State(float measurement, float d_t, float alpha, float velocity_gain)
{
    if (!Initialized_Flag)
    {
        Position = measurement;
        Velocity = 0.0f;
        Initialized_Flag = true;
        return (true);
    }

    const float predicted = Position + Velocity * d_t;
    const float residual = measurement - predicted;
    const float position = predicted + alpha * residual;
    const float velocity = Velocity + velocity_gain * residual;
    if (!isfinite(position) || !isfinite(velocity))
    {
        return (false);
    }

    Position = position;
    Velocity = velocity;
    return (true);
}

/** @brief 从最后估计时刻外推，非法步长或计算溢出时返回当前位置。 */
float Class_Filter_Alpha_Beta::Predict_Position(float delta_t) const
{
    if (!Initialized_Flag || !isfinite(delta_t) || delta_t < 0.0f)
    {
        return (Position);
    }

    const float predicted = Position + Velocity * delta_t;
    return (isfinite(predicted) ? predicted : Position);
}
