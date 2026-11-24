/**
 * @file alg_filter_one_euro.h
 * @author zzm
 * @brief One Euro自适应低通滤波器
 */

#ifndef ALG_FILTER_ONE_EURO_H
#define ALG_FILTER_ONE_EURO_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 标量One Euro滤波器
 * @details Init成功后按固定周期输入有限测量值, 每个采样只计算一次。
 * 首帧直接对齐输入; 周期或参数改变时重新Init。
 * 算法采用作者2023年修正版: 速度估计使用当前输入与上次滤波输出之差。
 * 参考: https://gery.casiez.net/1euro/
 */
class Class_Filter_One_Euro
{
  public:
    bool Init(float min_cutoff_frequency, float beta = 0.0f,
              float derivative_cutoff_frequency = 1.0f, float d_t = 0.001f);

    void Reset(float value);

    inline float Get_Out() const;

    inline bool Get_Initialized_Flag() const;

    inline void Set_Now(float now);

    void TIM_Calculate_PeriodElapsedCallback();

  protected:
    /* 初始化相关常量 --------------------------------------------------------*/

    float Min_Cutoff_Frequency = 1.0f;
    float Beta = 0.0f;
    float Sampling_Frequency = 1000.0f;
    float Derivative_Alpha = 0.0f;

    /* 内部变量 --------------------------------------------------------------*/

    float Now = 0.0f;
    float Derivative_Out = 0.0f;
    float Out = 0.0f;
    bool Initialized_Flag = false;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline float Class_Filter_One_Euro::Get_Out() const
{
    return (Out);
}

inline bool Class_Filter_One_Euro::Get_Initialized_Flag() const
{
    return (Initialized_Flag);
}

inline void Class_Filter_One_Euro::Set_Now(float now)
{
    if (Basic_Math_Is_Invalid_Float(now))
    {
        return;
    }
    Now = now;
    if (!Initialized_Flag)
    {
        Reset(Now);
    }
}

#endif
