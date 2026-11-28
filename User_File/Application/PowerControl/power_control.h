#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "alg_rls.h"

#define POWER_CONTROL_MAX_MOTORS 4

typedef struct
{
    uint8_t motor_count;
    float static_power_w;
    float friction_coefficient;
    float copper_coefficient;
    float reserve_power_w;
    bool enable_rls;
    float rls_forgetting_factor;
    float rls_initial_covariance;
    float friction_coefficient_max;
    float copper_coefficient_max;
} PowerControlConfig_t;

typedef struct
{
    float desired_torque_nm[POWER_CONTROL_MAX_MOTORS];
    float speed_rad_s[POWER_CONTROL_MAX_MOTORS];
    float power_limit_w;
} PowerControlInput_t;

typedef struct
{
    float torque_nm[POWER_CONTROL_MAX_MOTORS];
    float estimated_power_w;
    float scale;
    bool limited;
    bool budget_below_baseline;
} PowerControlOutput_t;

/**
 * @brief 底盘功率模型与统一力矩限幅；无硬件、任务或动态内存依赖。
 * @details 输入力矩为电机输出轴 N·m，速度为同轴 rad/s，功率为 W。
 * 先用不小于零的机械功率、转速摩擦项和力矩平方项预测总功率，再以共同系数缩放目标力矩。
 * 在线估计值不会将两个损耗系数降到初始化基线以下。
 * 调用方负责电机协议单位转换、功率预算来源、采样时序及实际命令发送。
 * 每个控制周期必须提供一帧新输入；无效输入或计算失败时输出清零。
 * 所有接口由同一执行上下文调用，或由调用方同步保护。
 */
class PowerControl
{
  public:
    bool Init(const PowerControlConfig_t *config);
    bool Set_Input(const PowerControlInput_t *input);
    bool TIM_Calculate_PeriodElapsedCallback();
    PowerControlOutput_t Get_Output() const;

    /** @brief 每帧新的、时间对齐的驱动功率样本最多调用一次，按速度与力矩平方的观测方向遗忘。 */
    bool Update_Identification(float measured_power_w, const float *actual_torque_nm,
                               const float *speed_rad_s);
    void Reset_Identification();
    bool Get_Model_Coefficients(float *friction, float *copper) const;

  protected:
    PowerControlConfig_t Config = {};
    PowerControlInput_t Input = {};
    PowerControlOutput_t Output = {};
    Class_RLS Estimator;
    float Friction_Coefficient = 0.0f;
    float Copper_Coefficient = 0.0f;
    bool Initialized = false;
    bool Input_Ready = false;
};

#endif
