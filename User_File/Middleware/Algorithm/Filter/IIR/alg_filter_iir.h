/**
 * @file alg_filter_iir.h
 * @author zzm
 * @brief IIR滤波器
 * @version 1.1
 * @date 2026-07-31
 */

#ifndef __ALG_FILTER_IIR_H
#define __ALG_FILTER_IIR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 一阶IIR低通滤波器
 *
 * 使用匹配极点法计算系数:
 * y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 * alpha = 1 - exp(-2 * PI * cutoff / sampling)
 */
class Class_Filter_IIR_First_Order
{
public:
    void Init(const float &__Cutoff_Frequency,
              const float &__Sampling_Frequency);

    inline float Get_Out() const;

    inline float Get_Alpha() const;

    inline bool Get_Initialized_Flag() const;

    inline void Set_Now(const float &__Now);

    inline void Reset(const float &__Value);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    /* 初始化相关常量 --------------------------------------------------------*/

    float Alpha = 1.0f;

    /* 内部变量 --------------------------------------------------------------*/

    float Now = 0.0f;
    float Out = 0.0f;
    bool Initialized_Flag = false;
};

/**
 * @brief Reusable, 二阶IIR陷波滤波器
 *
 * 使用归一化biquad直接I型结构:
 * y[k] = b0*x[k] + b1*x[k-1] + b2*x[k-2]
 *        - a1*y[k-1] - a2*y[k-2]
 */
class Class_Filter_IIR_Second_Order_Notch
{
public:
    void Init(const float &__Center_Frequency,
              const float &__Sampling_Frequency,
              const float &__Quality_Factor);

    inline float Get_Out() const;

    inline bool Get_Initialized_Flag() const;

    inline void Set_Now(const float &__Now);

    inline void Reset(const float &__Value);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    /* 初始化相关常量 --------------------------------------------------------*/

    float B_0 = 1.0f;
    float B_1 = 0.0f;
    float B_2 = 0.0f;
    float A_1 = 0.0f;
    float A_2 = 0.0f;

    /* 内部变量 --------------------------------------------------------------*/

    float Now = 0.0f;
    float Input_1 = 0.0f;
    float Input_2 = 0.0f;
    float Output_1 = 0.0f;
    float Output_2 = 0.0f;
    float Out = 0.0f;
    bool Initialized_Flag = false;
};

/**
 * @brief Reusable, 二阶IIR低通滤波器
 */
class Class_Filter_IIR_Second_Order_Low_Pass
{
public:
    void Init(const float &__Cutoff_Frequency,
              const float &__Sampling_Frequency,
              const float &__Quality_Factor);

    inline float Get_Out() const;

    inline bool Get_Initialized_Flag() const;

    inline void Set_Now(const float &__Now);

    inline void Reset(const float &__Value);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    float B_0 = 1.0f;
    float B_1 = 0.0f;
    float B_2 = 0.0f;
    float A_1 = 0.0f;
    float A_2 = 0.0f;

    float Now = 0.0f;
    float Input_1 = 0.0f;
    float Input_2 = 0.0f;
    float Output_1 = 0.0f;
    float Output_2 = 0.0f;
    float Out = 0.0f;
    bool Initialized_Flag = false;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline float Class_Filter_IIR_First_Order::Get_Out() const
{
    return Out;
}

inline float Class_Filter_IIR_First_Order::Get_Alpha() const
{
    return Alpha;
}

inline bool Class_Filter_IIR_First_Order::Get_Initialized_Flag() const
{
    return Initialized_Flag;
}

inline void Class_Filter_IIR_First_Order::Set_Now(const float &__Now)
{
    Now = __Now;

    // 首帧直接对齐输入, 避免滤波输出从0缓慢爬升造成启动冲击。
    if (!Initialized_Flag)
    {
        Out = Now;
        Initialized_Flag = true;
    }
}

inline void Class_Filter_IIR_First_Order::Reset(const float &__Value)
{
    Now = __Value;
    Out = __Value;
    Initialized_Flag = true;
}

inline float Class_Filter_IIR_Second_Order_Notch::Get_Out() const
{
    return Out;
}

inline bool Class_Filter_IIR_Second_Order_Notch::Get_Initialized_Flag() const
{
    return Initialized_Flag;
}

inline void Class_Filter_IIR_Second_Order_Notch::Set_Now(const float &__Now)
{
    if (Basic_Math_Is_Invalid_Float(__Now))
    {
        return;
    }

    Now = __Now;
    if (!Initialized_Flag)
    {
        Reset(Now);
    }
}

inline void Class_Filter_IIR_Second_Order_Notch::Reset(const float &__Value)
{
    Now = __Value;
    Input_1 = __Value;
    Input_2 = __Value;
    Output_1 = __Value;
    Output_2 = __Value;
    Out = __Value;
    Initialized_Flag = true;
}

inline float Class_Filter_IIR_Second_Order_Low_Pass::Get_Out() const
{
    return Out;
}

inline bool Class_Filter_IIR_Second_Order_Low_Pass::Get_Initialized_Flag() const
{
    return Initialized_Flag;
}

inline void Class_Filter_IIR_Second_Order_Low_Pass::Set_Now(const float &__Now)
{
    if (Basic_Math_Is_Invalid_Float(__Now))
    {
        return;
    }

    Now = __Now;
    if (!Initialized_Flag)
    {
        Reset(Now);
    }
}

inline void Class_Filter_IIR_Second_Order_Low_Pass::Reset(const float &__Value)
{
    Now = __Value;
    Input_1 = __Value;
    Input_2 = __Value;
    Output_1 = __Value;
    Output_2 = __Value;
    Out = __Value;
    Initialized_Flag = true;
}

#endif
