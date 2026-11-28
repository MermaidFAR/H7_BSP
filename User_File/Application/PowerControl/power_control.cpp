#include "power_control.h"
#include "alg_basic.h"

#include <float.h>
#include <math.h>

bool PowerControl::Init(const PowerControlConfig_t *config)
{
    if (config == NULL || config->motor_count == 0 ||
        config->motor_count > POWER_CONTROL_MAX_MOTORS ||
        Basic_Math_Is_Invalid_Float(config->static_power_w) || config->static_power_w < 0.0f ||
        Basic_Math_Is_Invalid_Float(config->friction_coefficient) || config->friction_coefficient < 0.0f ||
        Basic_Math_Is_Invalid_Float(config->copper_coefficient) || config->copper_coefficient <= 0.0f ||
        Basic_Math_Is_Invalid_Float(config->reserve_power_w) || config->reserve_power_w < 0.0f)
    {
        return false;
    }

    if (config->enable_rls)
    {
        if (Basic_Math_Is_Invalid_Float(config->friction_coefficient_max) ||
            Basic_Math_Is_Invalid_Float(config->copper_coefficient_max) ||
            config->friction_coefficient_max < config->friction_coefficient ||
            config->copper_coefficient_max < config->copper_coefficient)
        {
            return false;
        }
        const float initial[2] = {config->friction_coefficient, config->copper_coefficient};
        if (!Estimator.Init(2, config->rls_forgetting_factor,
                            config->rls_initial_covariance, initial))
        {
            return false;
        }
    }

    Config = *config;
    Friction_Coefficient = config->friction_coefficient;
    Copper_Coefficient = config->copper_coefficient;
    Input = {};
    Output = {};
    Initialized = true;
    Input_Ready = false;
    return true;
}

bool PowerControl::Set_Input(const PowerControlInput_t *input)
{
    if (!Initialized || input == NULL ||
        Basic_Math_Is_Invalid_Float(input->power_limit_w) || input->power_limit_w < 0.0f)
    {
        Input_Ready = false;
        Output = {};
        return false;
    }
    for (uint8_t i = 0; i < Config.motor_count; i++)
    {
        if (Basic_Math_Is_Invalid_Float(input->desired_torque_nm[i]) ||
            Basic_Math_Is_Invalid_Float(input->speed_rad_s[i]))
        {
            Input_Ready = false;
            Output = {};
            return false;
        }
    }
    Input = *input;
    Input_Ready = true;
    return true;
}

bool PowerControl::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Initialized || !Input_Ready)
    {
        Output = {};
        return false;
    }
    Input_Ready = false;

    double speed_sum = 0.0;
    double torque_square_sum = 0.0;
    double motoring_power = 0.0;
    for (uint8_t i = 0; i < Config.motor_count; i++)
    {
        const double torque = Input.desired_torque_nm[i];
        const double speed = Input.speed_rad_s[i];
        speed_sum += fabs(speed);
        torque_square_sum += torque * torque;
        const double mechanical = torque * speed;
        if (mechanical > 0.0)
        {
            motoring_power += mechanical;
        }
    }

    // 所有力矩乘同一比例 s，铜耗按 s²、驱动机械功率按 s 变化，基础损耗独立于力矩。
    const double a = (double) Copper_Coefficient * torque_square_sum;
    const double b = motoring_power;
    const double baseline = (double) Config.static_power_w + (double) Friction_Coefficient * speed_sum;
    const double available = fmax(0.0, (double) Input.power_limit_w - (double) Config.reserve_power_w);
    double scale = 1.0;
    if (a + b + baseline > available)
    {
        if (baseline >= available)
        {
            scale = 0.0;
        }
        else
        {
            const double remainder = available - baseline;
            // 对 a*s² + b*s = remainder 的正根有理化，避免相近数相减丢失精度。
            const double denominator = b + sqrt(b * b + 4.0 * a * remainder);
            scale = denominator > 0.0 ? 2.0 * remainder / denominator : 0.0;
        }
    }
    if (!isfinite(scale))
    {
        Output = {};
        return false;
    }
    scale = fmax(0.0, fmin(1.0, scale));

    PowerControlOutput_t next = {};
    next.scale = (float) scale;
    next.limited = scale < 1.0;
    next.budget_below_baseline = baseline > available;
    next.estimated_power_w = (float) fmin((double) FLT_MAX, baseline + b * scale + a * scale * scale);
    for (uint8_t i = 0; i < Config.motor_count; i++)
    {
        next.torque_nm[i] = (float) ((double) Input.desired_torque_nm[i] * scale);
    }
    Output = next;
    return true;
}

PowerControlOutput_t PowerControl::Get_Output() const
{
    return Output;
}

bool PowerControl::Update_Identification(float measured_power_w,
                                         const float *actual_torque_nm,
                                         const float *speed_rad_s)
{
    if (!Initialized || !Config.enable_rls || actual_torque_nm == NULL || speed_rad_s == NULL ||
        Basic_Math_Is_Invalid_Float(measured_power_w) || measured_power_w < 0.0f)
    {
        return false;
    }

    double speed_sum = 0.0;
    double torque_square_sum = 0.0;
    double mechanical_power = 0.0;
    for (uint8_t i = 0; i < Config.motor_count; i++)
    {
        if (Basic_Math_Is_Invalid_Float(actual_torque_nm[i]) ||
            Basic_Math_Is_Invalid_Float(speed_rad_s[i]))
        {
            return false;
        }
        const double torque = actual_torque_nm[i];
        const double speed = speed_rad_s[i];
        if (torque * speed < 0.0)
        {
            return false;
        }
        mechanical_power += torque * speed;
        speed_sum += fabs(speed);
        torque_square_sum += torque * torque;
    }

    // 扣除静态与机械功率，剩余测量用于拟合速度相关损耗和力矩平方损耗。
    const double residual = (double) measured_power_w - (double) Config.static_power_w - mechanical_power;
    if (residual < 0.0 || !isfinite(residual) || residual > (double) FLT_MAX ||
        !isfinite(speed_sum) || speed_sum > (double) FLT_MAX ||
        !isfinite(torque_square_sum) || torque_square_sum > (double) FLT_MAX ||
        speed_sum + torque_square_sum <= 0.0)
    {
        return false;
    }
    const float features[2] = {(float) speed_sum, (float) torque_square_sum};
    if (!Estimator.Update_Directional(features, (float) residual))
    {
        return false;
    }

    const float friction = Estimator.Get_Parameter(0);
    const float copper = Estimator.Get_Parameter(1);
    if (Basic_Math_Is_Invalid_Float(friction) || friction < 0.0f ||
        friction > Config.friction_coefficient_max ||
        Basic_Math_Is_Invalid_Float(copper) || copper <= 0.0f ||
        copper > Config.copper_coefficient_max)
    {
        // 用最近可用的系数重建估计器，防止异常样本继续影响后续拟合。
        const float previous[2] = {Friction_Coefficient, Copper_Coefficient};
        Estimator.Init(2, Config.rls_forgetting_factor,
                       Config.rls_initial_covariance, previous);
        return false;
    }
    Friction_Coefficient = fmaxf(Config.friction_coefficient, friction);
    Copper_Coefficient = fmaxf(Config.copper_coefficient, copper);
    return true;
}

void PowerControl::Reset_Identification()
{
    if (!Initialized)
    {
        return;
    }
    Friction_Coefficient = Config.friction_coefficient;
    Copper_Coefficient = Config.copper_coefficient;
    if (Config.enable_rls)
    {
        // 越界回退可能以最近系数重建过估计器，显式复位须恢复最初配置。
        const float initial[2] = {Friction_Coefficient, Copper_Coefficient};
        Estimator.Init(2, Config.rls_forgetting_factor, Config.rls_initial_covariance, initial);
    }
}

bool PowerControl::Get_Model_Coefficients(float *friction, float *copper) const
{
    if (!Initialized || friction == NULL || copper == NULL)
    {
        return false;
    }
    *friction = Friction_Coefficient;
    *copper = Copper_Coefficient;
    return true;
}
