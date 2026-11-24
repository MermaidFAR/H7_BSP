/**
 * @file alg_filter_one_euro.cpp
 * @author zzm
 * @brief One Euro自适应低通滤波器
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_one_euro.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 配置固定采样周期及滤波参数, 成功后清空历史, 首帧对齐输入
 * @param min_cutoff_frequency 最低截止频率, Hz, 大于0
 * @param beta 速度对截止频率的影响系数, 非负, 数值依赖输入单位
 * @param derivative_cutoff_frequency 速度估计的低通截止频率, Hz, 大于0
 * @param d_t 采样周期, s, 大于0
 * @return 参数及系数有效时返回true, 否则保留原配置和状态
 */
bool Class_Filter_One_Euro::Init(float min_cutoff_frequency, float beta,
                                 float derivative_cutoff_frequency, float d_t)
{
    if (Basic_Math_Is_Invalid_Float(min_cutoff_frequency) ||
        Basic_Math_Is_Invalid_Float(beta) ||
        Basic_Math_Is_Invalid_Float(derivative_cutoff_frequency) ||
        Basic_Math_Is_Invalid_Float(d_t) ||
        min_cutoff_frequency <= 0.0f || beta < 0.0f ||
        derivative_cutoff_frequency <= 0.0f || d_t <= 0.0f)
    {
        return (false);
    }

    float sampling_frequency = 1.0f / d_t;
    float derivative_alpha = 1.0f / (1.0f + sampling_frequency /
                                                (2.0f * PI * derivative_cutoff_frequency));
    float min_alpha = 1.0f / (1.0f + sampling_frequency /
                                         (2.0f * PI * min_cutoff_frequency));
    if (Basic_Math_Is_Invalid_Float(sampling_frequency) ||
        Basic_Math_Is_Invalid_Float(derivative_alpha) || derivative_alpha <= 0.0f ||
        Basic_Math_Is_Invalid_Float(min_alpha) || min_alpha <= 0.0f)
    {
        return (false);
    }

    Min_Cutoff_Frequency = min_cutoff_frequency;
    Beta = beta;
    Sampling_Frequency = sampling_frequency;
    Derivative_Alpha = derivative_alpha;
    Now = 0.0f;
    Derivative_Out = 0.0f;
    Out = 0.0f;
    Initialized_Flag = false;
    return (true);
}

/**
 * @brief 将输入和输出对齐到给定值, 清空速度估计, 保留配置
 */
void Class_Filter_One_Euro::Reset(float value)
{
    Now = value;
    Out = value;
    Derivative_Out = 0.0f;
    Initialized_Flag = true;
}

void Class_Filter_One_Euro::TIM_Calculate_PeriodElapsedCallback()
{
    if (!Initialized_Flag)
    {
        return;
    }

    float derivative = (Now - Out) * Sampling_Frequency;
    Derivative_Out += Derivative_Alpha * (derivative - Derivative_Out);
    float cutoff = Min_Cutoff_Frequency + Beta * fabsf(Derivative_Out);
    float alpha = 1.0f / (1.0f + Sampling_Frequency / (2.0f * PI * cutoff));
    Out += alpha * (Now - Out);
}
