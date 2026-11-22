/**
 * @file alg_filter_frequency.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 频率滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_frequency.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 采样频率
const float FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY = 1000.0f;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 计算对称窗权重, 调用前由Init验证类型与阶数
 */
float Filter_Frequency_Window_Value(Enum_Filter_Frequency_Window window,
                                  uint32_t index, uint32_t order)
{
    if (window == Filter_Frequency_Window_RECTANGULAR)
    {
        return (1.0f);
    }
    if (index == 0 || index == order)
    {
        return (window == Filter_Frequency_Window_HAMMING ? 0.08f : 0.0f);
    }

    float cosine = arm_cos_f32((2.0f * PI) * ((float)index / order));
    switch (window)
    {
    case Filter_Frequency_Window_HAMMING:
        return (0.54f - 0.46f * cosine);
    case Filter_Frequency_Window_HANN:
        return (0.5f - 0.5f * cosine);
    case Filter_Frequency_Window_BLACKMAN:
        return (0.42f - 0.5f * cosine + 0.08f * (2.0f * cosine * cosine - 1.0f));
    default:
        return (0.0f);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
