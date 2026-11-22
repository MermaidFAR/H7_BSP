/**
 * @file alg_pid.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 更新 PID 参数，保留目标、输出及积分/微分历史状态。
 *
 * @param k_p P值
 * @param k_i I值
 * @param k_d D值
 * @param k_f 每周期目标增量前馈系数，不除以 D_T
 * @param i_out_max 积分限幅
 * @param out_max 输出限幅
 * @param d_t 时间片长度
 * @param dead_zone 非负死区阈值，含边界的有效误差归零，区外扣除死区宽度
 * @param i_variable_speed_a 变速积分内段阈值，与 B 均为零时关闭变速积分
 * @param i_variable_speed_b 变速积分外段阈值，启用线性段时要求 0 <= A < B
 * @param i_separate_threshold 积分分离阈值，达到时清空积分；零为关闭，非零须为正
 * @param d_first 是否开启微分先行
 * @param d_filter_cutoff D 支路一阶 IIR 截止频率，Hz；0 关闭，采样频率为 1 / D_T
 * @note 持续启用且 D_First 不变时保留滤波速率；首次启用或切换微分来源时从 0 开始。
 */
void Class_PID::Init(float k_p, float k_i, float k_d, float k_f, float i_out_max, float out_max, float d_t, float dead_zone, float i_variable_speed_a, float i_variable_speed_b, float i_separate_threshold, Enum_PID_D_First d_first, float d_filter_cutoff)
{
    const float d_filter_out = D_Filter_Cutoff > 0.0f && D_First == d_first
                                   ? D_Filter.Get_Out() : 0.0f;
    K_P = k_p;
    K_I = k_i;
    K_D = k_d;
    K_F = k_f;
    I_Out_Max = i_out_max;
    Out_Max = out_max;
    D_T = d_t;
    Dead_Zone = dead_zone;
    I_Variable_Speed_A = i_variable_speed_a;
    I_Variable_Speed_B = i_variable_speed_b;
    I_Separate_Threshold = i_separate_threshold;
    D_First = d_first;
    D_Filter_Cutoff = !Basic_Math_Is_Invalid_Float(d_filter_cutoff) && d_filter_cutoff > 0.0f
                          ? d_filter_cutoff : 0.0f;
    if (D_Filter_Cutoff > 0.0f)
    {
        D_Filter.Init(D_Filter_Cutoff, 1.0f / D_T);
        // D 支路使用零初值/保留值，不采用 IIR 默认的首帧直通。
        D_Filter.Reset(d_filter_out);
    }
    else
    {
        D_Filter.Reset(0.0f);
    }
}

float Class_PID::Get_Target() const
{
    return Target;
}

float Class_PID::Get_Now() const
{
    return Now;
}

float Class_PID::Get_Error() const
{
    return Pre_Error;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 */
void Class_PID::TIM_Calculate_PeriodElapsedCallback()
{
    if (Basic_Math_Is_Invalid_Float(Target) || Basic_Math_Is_Invalid_Float(Now))
    {
        return;
    }

    float error = Target - Now;
    if (Basic_Math_Is_Invalid_Float(error))
    {
        return;
    }
    float abs_error = Basic_Math_Abs(error);

    // 死区只处理局部误差，不改写调用者目标；正负边界均连续。
    if (abs_error <= Dead_Zone)
    {
        error = 0.0f;
        abs_error = 0.0f;
    }
    else if (error > 0.0f)
    {
        error -= Dead_Zone;
    }
    else
    {
        error += Dead_Zone;
    }

    const float p_out = K_P * error;

    // 计算i项；为保留阈值调参语义，abs_error 在死区内为零、区外仍为原始误差幅值。
    float speed_ratio = 0.0f;
    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)
    {
        // 非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= I_Variable_Speed_A)
        {
            speed_ratio = 1.0f;
        }
        else if (I_Variable_Speed_A < abs_error && abs_error < I_Variable_Speed_B)
        {
            speed_ratio = (I_Variable_Speed_B - abs_error) / (I_Variable_Speed_B - I_Variable_Speed_A);
        }
        else if (abs_error >= I_Variable_Speed_B)
        {
            speed_ratio = 0.0f;
        }
    }
    // Ki 为零或进入分离区间时清空积分，避免重新启用时带入旧累积量。
    float next_integral = 0.0f;
    float i_out = 0.0f;
    const bool separate_integral = I_Separate_Threshold != 0.0f && abs_error >= I_Separate_Threshold;
    if (K_I != 0.0f && !separate_integral)
    {
        next_integral = Integral_Error + speed_ratio * D_T * error;
        if (!isfinite(next_integral))
        {
            return;
        }
        // 累加后限幅，保证本周期的积分输出也不越界；兼容负 Ki。
        if (I_Out_Max != 0.0f)
        {
            const float integral_max = Basic_Math_Abs(I_Out_Max / K_I);
            Basic_Math_Constrain(&next_integral, -integral_max, integral_max);
        }
        i_out = K_I * next_integral;
    }

    // 计算d项

    const float d_delta = D_First == PID_D_First_DISABLE
                              ? error - Pre_Error : Pre_Now - Now;
    const bool filter_d = D_Filter_Cutoff > 0.0f;
    Class_Filter_IIR_First_Order next_filter;
    float d_out;
    if (filter_d)
    {
        // 先滤波差分速率，再施加 K_D；K_D 为零时仍跟踪速率，但 D 输出为零。
        const float rate = d_delta / D_T;
        if (!isfinite(rate))
        {
            return;
        }
        next_filter = D_Filter;
        next_filter.Set_Now(rate);
        next_filter.TIM_Calculate_PeriodElapsedCallback();
        d_out = K_D * next_filter.Get_Out();
    }
    else
    {
        // 关闭时保留原来的乘除顺序，不引入滤波延迟。
        d_out = K_D * d_delta / D_T;
    }

    const float f_out = K_F * (Target - Pre_Target);
    float next_out = p_out + i_out + d_out + f_out;
    if (!isfinite(error) || !isfinite(next_out))
    {
        return;
    }

    // 输出限幅
    if (Out_Max != 0.0f)
    {
        Basic_Math_Constrain(&next_out, -Out_Max, Out_Max);
    }

    // 所有候选值有效后再提交本周期状态。
    if (filter_d)
    {
        D_Filter = next_filter;
    }
    Integral_Error = next_integral;
    Out = next_out;
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
