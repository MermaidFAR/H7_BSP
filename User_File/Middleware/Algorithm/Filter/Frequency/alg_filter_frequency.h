/**
 * @file alg_filter_frequency.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 频率滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ALG_FILTER_FREQUENCY_H
#define ALG_FILTER_FREQUENCY_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

extern const float FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 滤波器类型
 *
 */
enum Enum_Filter_Frequency_Type
{
    Filter_Frequency_Type_LOWPASS = 0,
    Filter_Frequency_Type_HIGHPASS,
    Filter_Frequency_Type_BANDPASS,
    Filter_Frequency_Type_BANDSTOP,
};

enum Enum_Filter_Frequency_Window
{
    Filter_Frequency_Window_RECTANGULAR = 0,
    Filter_Frequency_Window_HAMMING,
    Filter_Frequency_Window_HANN,
    Filter_Frequency_Window_BLACKMAN,
};

float Filter_Frequency_Window_Value(Enum_Filter_Frequency_Window window,
                                  uint32_t index, uint32_t order);

/**
 * @brief Reusable, Frequency滤波器算法
 * @details Init成功后按固定周期输入有限采样值。Init、Reset与采样不得并发调用。
 * 对称系数的群延迟为阶数/(2*采样率), 更换窗函数不改变同阶群延迟。
 */
template<uint32_t Filter_Frequency_Order = 50>
class Class_Filter_Frequency
{
public:
    bool Init(float value_constrain_low = 0.0f, float value_constrain_high = 1.0f, Enum_Filter_Frequency_Type filter_frequency_type = Filter_Frequency_Type_LOWPASS, float frequency_low = 0.0f, float frequency_high = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f, float sampling_frequency = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY, Enum_Filter_Frequency_Window window = Filter_Frequency_Window_RECTANGULAR);

    void Reset();

    inline float Get_Out() const;

    inline void Set_Now(float now);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 输入限幅
    float Value_Constrain_Low = 0.0f;
    float Value_Constrain_High = 0.0f;

    // 滤波器类型
    Enum_Filter_Frequency_Type Filter_Frequency_Type = Filter_Frequency_Type_LOWPASS;
    // 滤波器特征低频
    float Frequency_Low = 0.0f;
    // 滤波器特征高频
    float Frequency_High = 0.0f;
    // 滤波器采样频率
    float Sampling_Frequency = 0.0f;

    // 常量

    // 内部变量

    // 卷积系统函数向量
    float System_Function[Filter_Frequency_Order + 1]{};

    // 输入信号向量
    float Input_Signal[Filter_Frequency_Order + 1]{};

    // 新数据指示向量
    uint32_t Signal_Flag = 0;

    // 读变量

    // 输出值
    float Out = 0;

    // 写变量

    // 读写变量

    // 内部函数
    float Calculate_Coefficient(uint32_t index, Enum_Filter_Frequency_Type type,
                                float omega_low, float omega_high,
                                Enum_Filter_Frequency_Window window) const;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化滤波器
 * @details 成功时重设系数并清空历史; 失败时保留原配置、系数、历史和输出。
 * 阶数必须大于0, 高通与带阻要求偶数阶。默认零截止频率返回false。
 *
 * @tparam Filter_Frequency_Order 滤波器阶数
 * @param value_constrain_low 滤波器最小值, 全0不限制
 * @param value_constrain_high 滤波器最大值, 全0不限制
 * @param filter_frequency_type 滤波器类型
 * @param frequency_low 低通截止频率或带通/带阻下边界, Hz
 * @param frequency_high 高通截止频率或带通/带阻上边界, Hz
 * @param sampling_frequency 滤波器采样频率, Hz
 * @param window 对称窗类型, 默认矩形窗
 * @return 配置与归一化系数有效时返回true
 */
template<uint32_t Filter_Frequency_Order>
bool Class_Filter_Frequency<Filter_Frequency_Order>::Init(float value_constrain_low, float value_constrain_high, Enum_Filter_Frequency_Type filter_frequency_type, float frequency_low, float frequency_high, float sampling_frequency, Enum_Filter_Frequency_Window window)
{
    if (Filter_Frequency_Order == 0 ||
        Basic_Math_Is_Invalid_Float(value_constrain_low) ||
        Basic_Math_Is_Invalid_Float(value_constrain_high) ||
        Basic_Math_Is_Invalid_Float(frequency_low) ||
        Basic_Math_Is_Invalid_Float(frequency_high) ||
        Basic_Math_Is_Invalid_Float(sampling_frequency) ||
        value_constrain_low > value_constrain_high || sampling_frequency <= 0.0f ||
        filter_frequency_type < Filter_Frequency_Type_LOWPASS ||
        filter_frequency_type > Filter_Frequency_Type_BANDSTOP ||
        window < Filter_Frequency_Window_RECTANGULAR || window > Filter_Frequency_Window_BLACKMAN)
    {
        return (false);
    }

    float nyquist = sampling_frequency / 2.0f;
    if ((filter_frequency_type != Filter_Frequency_Type_HIGHPASS &&
         (frequency_low <= 0.0f || frequency_low >= nyquist)) ||
        (filter_frequency_type != Filter_Frequency_Type_LOWPASS &&
         (frequency_high <= 0.0f || frequency_high >= nyquist)) ||
        ((filter_frequency_type == Filter_Frequency_Type_BANDPASS ||
          filter_frequency_type == Filter_Frequency_Type_BANDSTOP) &&
         frequency_low >= frequency_high) ||
        ((filter_frequency_type == Filter_Frequency_Type_HIGHPASS ||
          filter_frequency_type == Filter_Frequency_Type_BANDSTOP) &&
         Filter_Frequency_Order % 2 != 0))
    {
        return (false);
    }

    float omega_low = filter_frequency_type == Filter_Frequency_Type_HIGHPASS ?
                      0.0f : (frequency_low / sampling_frequency) * (2.0f * PI);
    float omega_high = filter_frequency_type == Filter_Frequency_Type_LOWPASS ?
                       0.0f : (frequency_high / sampling_frequency) * (2.0f * PI);
    float omega_reference = 0.0f;
    if (filter_frequency_type == Filter_Frequency_Type_HIGHPASS)
    {
        omega_reference = PI;
    }
    else if (filter_frequency_type == Filter_Frequency_Type_BANDPASS)
    {
        omega_reference = (omega_low + omega_high) / 2.0f;
    }

    double gain = 0.0;
    float coefficient_max = 0.0f;
    // 先验证整组系数与归一化, 通过后再写入, 不占用阶数相关的临时数组。
    for (uint32_t i = 0; i <= Filter_Frequency_Order / 2; i++)
    {
        float coefficient = Calculate_Coefficient(i, filter_frequency_type, omega_low, omega_high, window);
        if (Basic_Math_Is_Invalid_Float(coefficient))
        {
            return (false);
        }
        coefficient_max = fmaxf(coefficient_max, fabsf(coefficient));
        float reference = arm_cos_f32(((float)i - Filter_Frequency_Order / 2.0f) * omega_reference);
        gain += (double)coefficient * (double)reference * (i == Filter_Frequency_Order - i ? 1.0 : 2.0);
    }

    float normalization = (float)gain;
    if (Basic_Math_Is_Invalid_Float(normalization) || fabsf(normalization) <= FLT_EPSILON)
    {
        return (false);
    }
    float scale = 1.0f / normalization;
    if (Basic_Math_Is_Invalid_Float(coefficient_max * scale))
    {
        return (false);
    }

    for (uint32_t i = 0; i <= Filter_Frequency_Order / 2; i++)
    {
        float coefficient = Calculate_Coefficient(i, filter_frequency_type, omega_low, omega_high, window) * scale;
        System_Function[i] = coefficient;
        System_Function[Filter_Frequency_Order - i] = coefficient;
    }

    Value_Constrain_Low = value_constrain_low;
    Value_Constrain_High = value_constrain_high;
    Filter_Frequency_Type = filter_frequency_type;
    Frequency_Low = frequency_low;
    Frequency_High = frequency_high;
    Sampling_Frequency = sampling_frequency;
    Reset();
    return (true);
}

/**
 * @brief 计算未归一化的加窗系数, 调用前由Init验证参数
 */
template<uint32_t Filter_Frequency_Order>
float Class_Filter_Frequency<Filter_Frequency_Order>::Calculate_Coefficient(
    uint32_t index, Enum_Filter_Frequency_Type type, float omega_low,
    float omega_high, Enum_Filter_Frequency_Window window) const
{
    float position = (float)index - Filter_Frequency_Order / 2.0f;
    float coefficient = 0.0f;
    switch (type)
    {
    case Filter_Frequency_Type_LOWPASS:
        coefficient = omega_low / PI * Basic_Math_Sinc(position * omega_low);
        break;
    case Filter_Frequency_Type_HIGHPASS:
        coefficient = Basic_Math_Sinc(position * PI) - omega_high / PI * Basic_Math_Sinc(position * omega_high);
        break;
    case Filter_Frequency_Type_BANDPASS:
        coefficient = omega_high / PI * Basic_Math_Sinc(position * omega_high) - omega_low / PI * Basic_Math_Sinc(position * omega_low);
        break;
    case Filter_Frequency_Type_BANDSTOP:
        coefficient = Basic_Math_Sinc(position * PI) + omega_low / PI * Basic_Math_Sinc(position * omega_low) - omega_high / PI * Basic_Math_Sinc(position * omega_high);
        break;
    }
    return (coefficient * Filter_Frequency_Window_Value(window, index, Filter_Frequency_Order));
}

/**
 * @brief 清空输入历史、环形索引和输出, 保留滤波配置与系数
 */
template<uint32_t Filter_Frequency_Order>
void Class_Filter_Frequency<Filter_Frequency_Order>::Reset()
{
    for (uint32_t i = 0; i <= Filter_Frequency_Order; i++)
    {
        Input_Signal[i] = 0.0f;
    }
    Signal_Flag = 0;
    Out = 0.0f;
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 *
 * @tparam Filter_Frequency_Order 滤波器阶数
 */
template<uint32_t Filter_Frequency_Order>
void Class_Filter_Frequency<Filter_Frequency_Order>::TIM_Calculate_PeriodElapsedCallback()
{
    Out = 0.0f;

    uint32_t coefficient = 0;
    for (uint32_t i = Signal_Flag; i <= Filter_Frequency_Order; i++)
    {
        Out += System_Function[coefficient++] * Input_Signal[i];
    }
    for (uint32_t i = 0; i < Signal_Flag; i++)
    {
        Out += System_Function[coefficient++] * Input_Signal[i];
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
template<uint32_t Filter_Frequency_Order>
inline float Class_Filter_Frequency<Filter_Frequency_Order>::Get_Out() const
{
    return (Out);
}

/**
 * @brief 设定当前值
 *
 * @param now 当前值
 */
template<uint32_t Filter_Frequency_Order>
inline void Class_Filter_Frequency<Filter_Frequency_Order>::Set_Now(float now)
{
    float now_value = now;

    // 输入限幅, 全0为不限制
    if (Value_Constrain_Low != 0.0f || Value_Constrain_High != 0.0f)
    {
        now_value = Basic_Math_Constrain(now, Value_Constrain_Low, Value_Constrain_High);
    }

    // 将当前值放入被卷积的信号中
    Input_Signal[Signal_Flag] = now_value;
    Signal_Flag++;

    // 若越界则轮回
    if (Signal_Flag == Filter_Frequency_Order + 1)
    {
        Signal_Flag = 0;
    }
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
