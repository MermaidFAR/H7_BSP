/**
 * @file alg_filter_iir.cpp
 * @author zzm
 * @brief IIR滤波器
 * @version 1.1
 * @date 2026-07-31
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_iir.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化一阶IIR低通滤波器
 *
 * @param cutoff_frequency 截止频率, Hz
 * @param sampling_frequency 采样频率, Hz
 */
void Class_Filter_IIR_First_Order::Init(
    float cutoff_frequency,
    float sampling_frequency)
{
    if (cutoff_frequency <= 0.0f || sampling_frequency <= 0.0f)
    {
        // 参数无效时旁路滤波器, 避免产生NaN或冻结输出。
        Alpha = 1.0f;
    }
    else
    {
        const float clamped_cutoff_frequency = Basic_Math_Constrain(
            cutoff_frequency,
            0.0f,
            sampling_frequency / 2.0f);

        Alpha = 1.0f - expf(-2.0f * PI * clamped_cutoff_frequency / sampling_frequency);
        Alpha = Basic_Math_Constrain(Alpha, 0.0f, 1.0f);
    }

    Now = 0.0f;
    Out = 0.0f;
    Initialized_Flag = false;
}

/**
 * @brief 执行一次一阶IIR低通滤波
 */
void Class_Filter_IIR_First_Order::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Initialized_Flag)
    {
        return;
    }

    Out += Alpha * (Now - Out);
}

/**
 * @brief 初始化二阶IIR陷波滤波器
 *
 * @param center_frequency 陷波中心频率, Hz
 * @param sampling_frequency 采样频率, Hz
 * @param quality_factor 品质因数Q
 */
void Class_Filter_IIR_Second_Order_Notch::Init(
    float center_frequency,
    float sampling_frequency,
    float quality_factor)
{
    B_0 = 1.0f;
    B_1 = 0.0f;
    B_2 = 0.0f;
    A_1 = 0.0f;
    A_2 = 0.0f;

    const bool parameter_valid =
        !Basic_Math_Is_Invalid_Float(center_frequency) &&
        !Basic_Math_Is_Invalid_Float(sampling_frequency) &&
        !Basic_Math_Is_Invalid_Float(quality_factor) &&
        center_frequency > 0.0f &&
        sampling_frequency > 0.0f &&
        center_frequency < sampling_frequency / 2.0f &&
        quality_factor > 0.0f;

    if (parameter_valid)
    {
        const float angular_frequency =
            2.0f * PI * center_frequency / sampling_frequency;
        const float cosine = cosf(angular_frequency);
        const float alpha =
            sinf(angular_frequency) / (2.0f * quality_factor);
        const float inverse_a_0 = 1.0f / (1.0f + alpha);

        B_0 = inverse_a_0;
        B_1 = -2.0f * cosine * inverse_a_0;
        B_2 = inverse_a_0;
        A_1 = B_1;
        A_2 = (1.0f - alpha) * inverse_a_0;
    }

    Now = 0.0f;
    Input_1 = 0.0f;
    Input_2 = 0.0f;
    Output_1 = 0.0f;
    Output_2 = 0.0f;
    Out = 0.0f;
    Initialized_Flag = false;
}

/**
 * @brief 执行一次二阶IIR陷波滤波
 */
void Class_Filter_IIR_Second_Order_Notch::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Initialized_Flag)
    {
        return;
    }

    const float output = B_0 * Now + B_1 * Input_1 + B_2 * Input_2 -
                         A_1 * Output_1 - A_2 * Output_2;

    if (Basic_Math_Is_Invalid_Float(output))
    {
        Reset(Now);
        return;
    }

    Input_2 = Input_1;
    Input_1 = Now;
    Output_2 = Output_1;
    Output_1 = output;
    Out = output;
}

/**
 * @brief 初始化二阶IIR低通滤波器
 *
 * @param cutoff_frequency 截止频率, Hz
 * @param sampling_frequency 采样频率, Hz
 * @param quality_factor 品质因数Q
 */
void Class_Filter_IIR_Second_Order_Low_Pass::Init(
    float cutoff_frequency,
    float sampling_frequency,
    float quality_factor)
{
    B_0 = 1.0f;
    B_1 = 0.0f;
    B_2 = 0.0f;
    A_1 = 0.0f;
    A_2 = 0.0f;

    const bool parameter_valid =
        !Basic_Math_Is_Invalid_Float(cutoff_frequency) &&
        !Basic_Math_Is_Invalid_Float(sampling_frequency) &&
        !Basic_Math_Is_Invalid_Float(quality_factor) &&
        cutoff_frequency > 0.0f &&
        sampling_frequency > 0.0f &&
        cutoff_frequency < sampling_frequency / 2.0f &&
        quality_factor > 0.0f;

    if (parameter_valid)
    {
        const float angular_frequency =
            2.0f * PI * cutoff_frequency / sampling_frequency;
        const float cosine = cosf(angular_frequency);
        const float alpha =
            sinf(angular_frequency) / (2.0f * quality_factor);
        const float inverse_a_0 = 1.0f / (1.0f + alpha);
        const float numerator = 1.0f - cosine;

        B_0 = 0.5f * numerator * inverse_a_0;
        B_1 = numerator * inverse_a_0;
        B_2 = B_0;
        A_1 = -2.0f * cosine * inverse_a_0;
        A_2 = (1.0f - alpha) * inverse_a_0;
    }

    Now = 0.0f;
    Input_1 = 0.0f;
    Input_2 = 0.0f;
    Output_1 = 0.0f;
    Output_2 = 0.0f;
    Out = 0.0f;
    Initialized_Flag = false;
}

/**
 * @brief 执行一次二阶IIR低通滤波
 */
void Class_Filter_IIR_Second_Order_Low_Pass::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Initialized_Flag)
    {
        return;
    }

    const float output = B_0 * Now + B_1 * Input_1 + B_2 * Input_2 -
                         A_1 * Output_1 - A_2 * Output_2;

    if (Basic_Math_Is_Invalid_Float(output))
    {
        Reset(Now);
        return;
    }

    Input_2 = Input_1;
    Input_1 = Now;
    Output_2 = Output_1;
    Output_1 = output;
    Out = output;
}
