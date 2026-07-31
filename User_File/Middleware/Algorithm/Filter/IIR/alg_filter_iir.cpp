/**
 * @file alg_filter_iir.cpp
 * @author zzm
 * @brief IIR滤波器
 * @version 1.1
 * @date 2026-07-31
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_iir.h"

#include <cmath>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化一阶IIR低通滤波器
 *
 * @param __Cutoff_Frequency 截止频率, Hz
 * @param __Sampling_Frequency 采样频率, Hz
 */
void Class_Filter_IIR_First_Order::Init(
    const float &__Cutoff_Frequency,
    const float &__Sampling_Frequency)
{
    if (__Cutoff_Frequency <= 0.0f || __Sampling_Frequency <= 0.0f)
    {
        // 参数无效时旁路滤波器, 避免产生NaN或冻结输出。
        Alpha = 1.0f;
    }
    else
    {
        const float cutoff_frequency = Basic_Math_Constrain(
            __Cutoff_Frequency,
            0.0f,
            __Sampling_Frequency / 2.0f);

        Alpha = 1.0f - std::exp(
                           -2.0f * PI * cutoff_frequency /
                           __Sampling_Frequency);
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
 * @param __Center_Frequency 陷波中心频率, Hz
 * @param __Sampling_Frequency 采样频率, Hz
 * @param __Quality_Factor 品质因数Q
 */
void Class_Filter_IIR_Second_Order_Notch::Init(
    const float &__Center_Frequency,
    const float &__Sampling_Frequency,
    const float &__Quality_Factor)
{
    B_0 = 1.0f;
    B_1 = 0.0f;
    B_2 = 0.0f;
    A_1 = 0.0f;
    A_2 = 0.0f;

    const bool parameter_valid =
        !Basic_Math_Is_Invalid_Float(__Center_Frequency) &&
        !Basic_Math_Is_Invalid_Float(__Sampling_Frequency) &&
        !Basic_Math_Is_Invalid_Float(__Quality_Factor) &&
        __Center_Frequency > 0.0f &&
        __Sampling_Frequency > 0.0f &&
        __Center_Frequency < __Sampling_Frequency / 2.0f &&
        __Quality_Factor > 0.0f;

    if (parameter_valid)
    {
        const float angular_frequency =
            2.0f * PI * __Center_Frequency / __Sampling_Frequency;
        const float cosine = std::cos(angular_frequency);
        const float alpha =
            std::sin(angular_frequency) / (2.0f * __Quality_Factor);
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
 * @param __Cutoff_Frequency 截止频率, Hz
 * @param __Sampling_Frequency 采样频率, Hz
 * @param __Quality_Factor 品质因数Q
 */
void Class_Filter_IIR_Second_Order_Low_Pass::Init(
    const float &__Cutoff_Frequency,
    const float &__Sampling_Frequency,
    const float &__Quality_Factor)
{
    B_0 = 1.0f;
    B_1 = 0.0f;
    B_2 = 0.0f;
    A_1 = 0.0f;
    A_2 = 0.0f;

    const bool parameter_valid =
        !Basic_Math_Is_Invalid_Float(__Cutoff_Frequency) &&
        !Basic_Math_Is_Invalid_Float(__Sampling_Frequency) &&
        !Basic_Math_Is_Invalid_Float(__Quality_Factor) &&
        __Cutoff_Frequency > 0.0f &&
        __Sampling_Frequency > 0.0f &&
        __Cutoff_Frequency < __Sampling_Frequency / 2.0f &&
        __Quality_Factor > 0.0f;

    if (parameter_valid)
    {
        const float angular_frequency =
            2.0f * PI * __Cutoff_Frequency / __Sampling_Frequency;
        const float cosine = std::cos(angular_frequency);
        const float alpha =
            std::sin(angular_frequency) / (2.0f * __Quality_Factor);
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
