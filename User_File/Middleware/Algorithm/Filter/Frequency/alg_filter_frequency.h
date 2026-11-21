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

float Filter_Frequency_Window_Value(Enum_Filter_Frequency_Window __Window,
                                  uint32_t __Index, uint32_t __Order);

/**
 * @brief Reusable, Frequency滤波器算法
 * @details Init成功后按固定周期输入有限采样值。Init、Reset与采样不得并发调用。
 * 对称系数的群延迟为阶数/(2*采样率), 更换窗函数不改变同阶群延迟。
 */
template<uint32_t Filter_Frequency_Order = 50>
class Class_Filter_Frequency
{
public:
    bool Init(const float &__Value_Constrain_Low = 0.0f, const float &__Value_Constrain_High = 1.0f, const Enum_Filter_Frequency_Type &__Filter_Frequency_Type = Filter_Frequency_Type_LOWPASS, const float &__Frequency_Low = 0.0f, const float &__Frequency_High = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f, const float &__Sampling_Frequency = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY, const Enum_Filter_Frequency_Window &__Window = Filter_Frequency_Window_RECTANGULAR);

    void Reset();

    inline float Get_Out() const;

    inline void Set_Now(const float &__Now);

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
    float Calculate_Coefficient(uint32_t __Index, Enum_Filter_Frequency_Type __Type,
                                float __Omega_Low, float __Omega_High,
                                Enum_Filter_Frequency_Window __Window) const;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化滤波器
 * @details 成功时重设系数并清空历史; 失败时保留原配置、系数、历史和输出。
 * 阶数必须大于0, 高通与带阻要求偶数阶。默认零截止频率返回false。
 *
 * @tparam Filter_Frequency_Order 滤波器阶数
 * @param __Value_Constrain_Low 滤波器最小值, 全0不限制
 * @param __Value_Constrain_High 滤波器最大值, 全0不限制
 * @param __Filter_Frequency_Type 滤波器类型
 * @param __Frequency_Low 低通截止频率或带通/带阻下边界, Hz
 * @param __Frequency_High 高通截止频率或带通/带阻上边界, Hz
 * @param __Sampling_Frequency 滤波器采样频率, Hz
 * @param __Window 对称窗类型, 默认矩形窗
 * @return 配置与归一化系数有效时返回true
 */
template<uint32_t Filter_Frequency_Order>
bool Class_Filter_Frequency<Filter_Frequency_Order>::Init(const float &__Value_Constrain_Low, const float &__Value_Constrain_High, const Enum_Filter_Frequency_Type &__Filter_Frequency_Type, const float &__Frequency_Low, const float &__Frequency_High, const float &__Sampling_Frequency, const Enum_Filter_Frequency_Window &__Window)
{
    if (Filter_Frequency_Order == 0 ||
        Basic_Math_Is_Invalid_Float(__Value_Constrain_Low) ||
        Basic_Math_Is_Invalid_Float(__Value_Constrain_High) ||
        Basic_Math_Is_Invalid_Float(__Frequency_Low) ||
        Basic_Math_Is_Invalid_Float(__Frequency_High) ||
        Basic_Math_Is_Invalid_Float(__Sampling_Frequency) ||
        __Value_Constrain_Low > __Value_Constrain_High || __Sampling_Frequency <= 0.0f ||
        __Filter_Frequency_Type < Filter_Frequency_Type_LOWPASS ||
        __Filter_Frequency_Type > Filter_Frequency_Type_BANDSTOP ||
        __Window < Filter_Frequency_Window_RECTANGULAR || __Window > Filter_Frequency_Window_BLACKMAN)
    {
        return (false);
    }

    float nyquist = __Sampling_Frequency / 2.0f;
    if ((__Filter_Frequency_Type != Filter_Frequency_Type_HIGHPASS &&
         (__Frequency_Low <= 0.0f || __Frequency_Low >= nyquist)) ||
        (__Filter_Frequency_Type != Filter_Frequency_Type_LOWPASS &&
         (__Frequency_High <= 0.0f || __Frequency_High >= nyquist)) ||
        ((__Filter_Frequency_Type == Filter_Frequency_Type_BANDPASS ||
          __Filter_Frequency_Type == Filter_Frequency_Type_BANDSTOP) &&
         __Frequency_Low >= __Frequency_High) ||
        ((__Filter_Frequency_Type == Filter_Frequency_Type_HIGHPASS ||
          __Filter_Frequency_Type == Filter_Frequency_Type_BANDSTOP) &&
         Filter_Frequency_Order % 2 != 0))
    {
        return (false);
    }

    float omega_low = __Filter_Frequency_Type == Filter_Frequency_Type_HIGHPASS ?
                      0.0f : (__Frequency_Low / __Sampling_Frequency) * (2.0f * PI);
    float omega_high = __Filter_Frequency_Type == Filter_Frequency_Type_LOWPASS ?
                       0.0f : (__Frequency_High / __Sampling_Frequency) * (2.0f * PI);
    float omega_reference = 0.0f;
    if (__Filter_Frequency_Type == Filter_Frequency_Type_HIGHPASS)
    {
        omega_reference = PI;
    }
    else if (__Filter_Frequency_Type == Filter_Frequency_Type_BANDPASS)
    {
        omega_reference = (omega_low + omega_high) / 2.0f;
    }

    double gain = 0.0;
    float coefficient_max = 0.0f;
    // 先验证整组系数与归一化, 通过后再写入, 不占用阶数相关的临时数组。
    for (uint32_t i = 0; i <= Filter_Frequency_Order / 2; i++)
    {
        float coefficient = Calculate_Coefficient(i, __Filter_Frequency_Type, omega_low, omega_high, __Window);
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
        float coefficient = Calculate_Coefficient(i, __Filter_Frequency_Type, omega_low, omega_high, __Window) * scale;
        System_Function[i] = coefficient;
        System_Function[Filter_Frequency_Order - i] = coefficient;
    }

    Value_Constrain_Low = __Value_Constrain_Low;
    Value_Constrain_High = __Value_Constrain_High;
    Filter_Frequency_Type = __Filter_Frequency_Type;
    Frequency_Low = __Frequency_Low;
    Frequency_High = __Frequency_High;
    Sampling_Frequency = __Sampling_Frequency;
    Reset();
    return (true);
}

/**
 * @brief 计算未归一化的加窗系数, 调用前由Init验证参数
 */
template<uint32_t Filter_Frequency_Order>
float Class_Filter_Frequency<Filter_Frequency_Order>::Calculate_Coefficient(
    uint32_t __Index, Enum_Filter_Frequency_Type __Type, float __Omega_Low,
    float __Omega_High, Enum_Filter_Frequency_Window __Window) const
{
    float position = (float)__Index - Filter_Frequency_Order / 2.0f;
    float coefficient = 0.0f;
    switch (__Type)
    {
    case Filter_Frequency_Type_LOWPASS:
        coefficient = __Omega_Low / PI * Basic_Math_Sinc(position * __Omega_Low);
        break;
    case Filter_Frequency_Type_HIGHPASS:
        coefficient = Basic_Math_Sinc(position * PI) - __Omega_High / PI * Basic_Math_Sinc(position * __Omega_High);
        break;
    case Filter_Frequency_Type_BANDPASS:
        coefficient = __Omega_High / PI * Basic_Math_Sinc(position * __Omega_High) - __Omega_Low / PI * Basic_Math_Sinc(position * __Omega_Low);
        break;
    case Filter_Frequency_Type_BANDSTOP:
        coefficient = Basic_Math_Sinc(position * PI) + __Omega_Low / PI * Basic_Math_Sinc(position * __Omega_Low) - __Omega_High / PI * Basic_Math_Sinc(position * __Omega_High);
        break;
    }
    return (coefficient * Filter_Frequency_Window_Value(__Window, __Index, Filter_Frequency_Order));
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
 * @param __Now 当前值
 */
template<uint32_t Filter_Frequency_Order>
inline void Class_Filter_Frequency<Filter_Frequency_Order>::Set_Now(const float &__Now)
{
    float now_value = __Now;

    // 输入限幅, 全0为不限制
    if (Value_Constrain_Low != 0.0f || Value_Constrain_High != 0.0f)
    {
        now_value = Basic_Math_Constrain(__Now, Value_Constrain_Low, Value_Constrain_High);
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
