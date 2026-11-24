#include "spin_mode.h"
#include "alg_matrix.h"

bool SpinMode::Init(const SpinConfig_t *config)
{
    const SpinConfig_t default_config = {0.0f, SpinMode_GIMBAL_FOLLOW};
    if (config == NULL)
    {
        config = &default_config;
    }
    if (Basic_Math_Is_Invalid_Float(config->zero_point) ||
        config->mode < SpinMode_GIMBAL_FOLLOW || config->mode > SpinMode_CHASSIS_FOLLOW)
    {
        return false;
    }

    /* 角度均为 rad；初始化 theta = zero_point，使初始校准相对角为 0。 */
    zero_point = Basic_Math_Modulus_Normalization(config->zero_point, 2.0f * PI);
    world_angle = 0.0f;
    theta = zero_point;
    x_target = 0.0f;
    y_target = 0.0f;
    w_target = 0.0f;
    Output = {};
    SpinMode_FSM.Init(config->mode);
    return true;
}

void SpinMode::Set_WorldTarget(float target_angle)
{
    world_angle = target_angle;
}

void SpinMode::Set_theta(float measured_theta)
{
    theta = measured_theta;
}

void SpinMode::Set_MoveTarget(float vx, float vy, float vw)
{
    x_target = vx;
    y_target = vy;
    w_target = vw;
}

void SpinMode::Set_Spin_Mode(SpinMode_e mode)
{
    if (mode < SpinMode_GIMBAL_FOLLOW || mode > SpinMode_CHASSIS_FOLLOW ||
        mode == Get_Spin_Mode())
    {
        return;
    }
    SpinMode_FSM.Set_Status(mode);
}

void SpinMode::TIM_Calculate_PeriodElapsedCallback()
{
    SpinMode_FSM.TIM_Calculate_PeriodElapsedCallback();
    /* rad；云台朝向减底盘朝向，扣除零偏并取最短角，正值表示云台在底盘左侧。 */
    const float relative_angle = Basic_Math_Modulus_Normalization(theta - zero_point, 2.0f * PI);
    /* m/s；[x, y] = R(relative_angle) * [vx, vy]，从云台坐标系转到底盘坐标系。 */
    Class_Matrix_f32<2, 1> velocity;
    velocity[0][0] = x_target;
    velocity[1][0] = y_target;
    velocity = Namespace_ALG_Matrix::From_Angle(relative_angle) * velocity;
    Output.x = velocity[0][0];
    Output.y = velocity[1][0];
    /* rad/s；所有模式均直通上层的角速度指令，不在本模块由角度误差生成速度。 */
    Output.w = w_target;

    switch (Get_Spin_Mode())
    {
    case SpinMode_GIMBAL_FOLLOW:
        Output.forward = 0.0f;
        break;
    case SpinMode_GIMBAL_LOCK:
    case SpinMode_CHASSIS_FOLLOW:
        /* rad/s；世界角速度 = 底盘角速度 + 相对角速度，因此相对速度目标补偿 -vw。 */
        Output.forward = -Output.w;
        break;
    }
}

SpinOutput_t SpinMode::Get_Output() const
{
    return Output;
}

SpinMode_e SpinMode::Get_Spin_Mode() const
{
    return (SpinMode_e) SpinMode_FSM.Get_Now_Status_Serial();
}

float SpinMode::Get_WorldTarget() const
{
    return world_angle;
}
