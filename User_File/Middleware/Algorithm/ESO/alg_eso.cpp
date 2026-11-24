/** @file alg_eso.cpp */

#include "alg_eso.h"
#include <math.h>

static bool ESO_Get_Pole_Distance(float b0, float omega_o, float d_t, float *inverse, float *pole_distance)
{
    if (!isfinite(b0) || b0 == 0.0f || !isfinite(omega_o) || omega_o <= 0.0f ||
        !isfinite(d_t) || d_t <= 0.0f)
    {
        return false;
    }
    const float bandwidth_step = omega_o * d_t;
    if (!isfinite(bandwidth_step) || bandwidth_step <= 0.0f)
    {
        return false;
    }

    *inverse = 1.0f / b0;
    *pole_distance = -expm1f(-bandwidth_step);
    // 按 1-p 校验，保留小步长的有效位。p 在 [0, 1) 时离散极点稳定。
    return isfinite(*inverse) && *pole_distance > 0.0f && *pole_distance <= 1.0f;
}

bool Class_ESO_First_Order::Init(float b0, float omega_o, float d_t)
{
    float inverse;
    float pole_distance;
    if (!ESO_Get_Pole_Distance(b0, omega_o, d_t, &inverse, &pole_distance))
    {
        return false;
    }

    const float l1 = pole_distance * (2.0f - pole_distance);
    const float l2 = pole_distance * (pole_distance / d_t);
    if (!isfinite(l2) || l2 <= 0.0f)
    {
        return false;
    }

    B0 = b0;
    B0_Inverse = inverse;
    D_T = d_t;
    L1 = l1;
    L2 = l2;
    Reset();
    return true;
}

void Class_ESO_First_Order::Reset(float initial_y)
{
    if (!isfinite(initial_y))
    {
        return;
    }
    Now = initial_y;
    Input = 0.0f;
    Z1 = initial_y;
    Z2 = 0.0f;
    Compensation = 0.0f;
}

void Class_ESO_First_Order::TIM_Calculate_PeriodElapsedCallback()
{
    const float predicted = Z1 + (Z2 + B0 * Input) * D_T;
    const float error = Now - predicted;
    const float z1 = predicted + L1 * error;
    const float z2 = Z2 + L2 * error;
    const float compensation = z2 * B0_Inverse;
    if (!isfinite(z1) || !isfinite(z2) || !isfinite(compensation))
    {
        return;
    }
    Z1 = z1;
    Z2 = z2;
    Compensation = compensation;
}

bool Class_ESO_Second_Order::Init(float b0, float omega_o, float d_t)
{
    float inverse;
    float pole_distance;
    if (!ESO_Get_Pole_Distance(b0, omega_o, d_t, &inverse, &pole_distance))
    {
        return false;
    }

    const float pole = 1.0f - pole_distance;
    const float rate = pole_distance / d_t;
    const float l1 = pole_distance * (1.0f + pole + pole * pole);
    const float l2 = (1.5f * rate) * (pole_distance * (1.0f + pole));
    const float l3 = (rate * pole_distance) * rate;
    if (!isfinite(l2) || l2 <= 0.0f || !isfinite(l3) || l3 <= 0.0f)
    {
        return false;
    }

    B0 = b0;
    B0_Inverse = inverse;
    D_T = d_t;
    L1 = l1;
    L2 = l2;
    L3 = l3;
    return Try_Reset(0.0f, 0.0f);
}

void Class_ESO_Second_Order::Reset(float initial_y, float initial_dy)
{
    (void) Try_Reset(initial_y, initial_dy);
}

bool Class_ESO_Second_Order::Try_Reset(float initial_y, float initial_dy)
{
    if (!isfinite(initial_y) || !isfinite(initial_dy))
    {
        return false;
    }
    Now = initial_y;
    Input = 0.0f;
    Z1 = initial_y;
    Z2 = initial_dy;
    Z3 = 0.0f;
    Compensation = 0.0f;
    return true;
}

void Class_ESO_Second_Order::TIM_Calculate_PeriodElapsedCallback()
{
    (void) Update(Now, Input);
}

bool Class_ESO_Second_Order::Update(float measurement, float applied_input)
{
    // 对区间 [k-1, k] 做零阶保持预测，再用当前测量校正。
    const float delta_velocity = (Z3 + B0 * applied_input) * D_T;
    const float predicted_position = Z1 + D_T * (Z2 + 0.5f * delta_velocity);
    const float error = measurement - predicted_position;
    const float z1 = predicted_position + L1 * error;
    const float z2 = Z2 + delta_velocity + L2 * error;
    const float z3 = Z3 + L3 * error;
    const float compensation = z3 * B0_Inverse;
    if (!isfinite(z1) || !isfinite(z2) || !isfinite(z3) || !isfinite(compensation))
    {
        return false;
    }
    Z1 = z1;
    Z2 = z2;
    Z3 = z3;
    Compensation = compensation;
    return true;
}
