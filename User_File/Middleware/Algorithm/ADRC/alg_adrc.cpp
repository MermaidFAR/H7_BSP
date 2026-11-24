/** @file alg_adrc.cpp */

#include "alg_adrc.h"

#include "alg_basic.h"

bool Class_LESO_Second_Order::Init(float b0, float omega_o, float d_t)
{
    return ESO.Init(b0, omega_o, d_t);
}

bool Class_LESO_Second_Order::Reset(float position, float velocity)
{
    return ESO.Try_Reset(position, velocity);
}

bool Class_LESO_Second_Order::Update(float measurement, float applied_input)
{
    return ESO.Update(measurement, applied_input);
}

bool Class_TD::Init(float omega, float d_t)
{
    if (!isfinite(omega) || omega <= 0.0f || !isfinite(d_t) || d_t <= 0.0f)
    {
        return false;
    }
    const float bandwidth_step = omega * d_t;
    if (!isfinite(bandwidth_step) || bandwidth_step <= 0.0f)
    {
        return false;
    }

    const float pole = expf(-bandwidth_step);
    const float scaled_pole = bandwidth_step * pole;
    // 小步长下以级数计算 1 - (1 + omega*d_t)*pole，避免相近数相减。
    const float position_gain = bandwidth_step < 0.01f
                                    ? bandwidth_step * bandwidth_step *
                                          (0.5f - bandwidth_step * (1.0f / 3.0f - bandwidth_step * 0.125f))
                                    : -expm1f(-bandwidth_step) - scaled_pole;
    const float velocity_gain = omega * scaled_pole;
    if (position_gain <= 0.0f || !isfinite(velocity_gain))
    {
        return false;
    }
    Position_Gain = position_gain;
    Position_From_Velocity = d_t * pole;
    Velocity_Gain = velocity_gain;
    Velocity_Retention = pole - scaled_pole;
    return Reset();
}

bool Class_TD::Reset(float position)
{
    if (!isfinite(position))
    {
        return false;
    }
    Position = position;
    Velocity = 0.0f;
    return true;
}

bool Class_TD::Update(float target)
{
    const float error = target - Position;
    const float position = Position + (Position_Gain * error + Position_From_Velocity * Velocity);
    const float velocity = Velocity_Retention * Velocity + Velocity_Gain * error;
    if (!isfinite(position) || !isfinite(velocity))
    {
        return false;
    }
    Position = position;
    Velocity = velocity;
    return true;
}

bool Class_LADRC_Second_Order::Init(float b0, float omega_o, float omega_c, float out_max, float d_t)
{
    if (!isfinite(b0) || b0 == 0.0f || !isfinite(omega_c) || omega_c <= 0.0f ||
        !isfinite(out_max) || out_max <= 0.0f || !isfinite(d_t) || d_t <= 0.0f)
    {
        return false;
    }
    const float bandwidth_step = omega_c * d_t;
    const float pole_distance = -expm1f(-bandwidth_step);
    const float rate = pole_distance / d_t;
    // 为零阶保持双积分模型配置重复离散极点，同时折入 1/b0。
    const float k_position = (rate / b0) * rate;
    const float k_velocity = (rate / b0) * (2.0f - 0.5f * pole_distance);
    if (!isfinite(bandwidth_step) || !isfinite(k_position) || k_position == 0.0f ||
        !isfinite(k_velocity) || k_velocity == 0.0f || !Observer.Init(b0, omega_o, d_t))
    {
        return false;
    }

    K_Position = k_position;
    K_Velocity = k_velocity;
    Out_Max = out_max;
    Ref_Position = 0.0f;
    Ref_Velocity = 0.0f;
    Out = 0.0f;
    return true;
}

bool Class_LADRC_Second_Order::Reset(float position, float velocity)
{
    if (!Observer.Reset(position, velocity))
    {
        return false;
    }
    Set_Ref(position, 0.0f);
    Out = 0.0f;
    return true;
}

void Class_LADRC_Second_Order::Set_Ref(float position, float velocity)
{
    Ref_Position = position;
    Ref_Velocity = velocity;
}

bool Class_LADRC_Second_Order::TIM_Calculate_PeriodElapsedCallback(float measurement)
{
    return TIM_Calculate_PeriodElapsedCallback(measurement, Out);
}

bool Class_LADRC_Second_Order::TIM_Calculate_PeriodElapsedCallback(float measurement, float applied_input)
{
    if (!Observer.Update(measurement, applied_input))
    {
        Out = 0.0f;
        return false;
    }
    const float command = K_Position * (Ref_Position - Observer.Get_Z1()) +
                          K_Velocity * (Ref_Velocity - Observer.Get_Z2()) - Observer.Get_Compensation();
    if (!isfinite(command))
    {
        Out = 0.0f;
        return false;
    }
    Out = Basic_Math_Constrain(command, -Out_Max, Out_Max);
    return true;
}
