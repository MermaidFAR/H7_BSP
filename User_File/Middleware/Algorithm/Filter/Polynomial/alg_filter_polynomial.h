/**
 * @file alg_filter_polynomial.h
 * @author zzm
 * @brief 固定周期标量多项式滤波与微分
 */

#ifndef ALG_FILTER_POLYNOMIAL_H
#define ALG_FILTER_POLYNOMIAL_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

#define FILTER_POLYNOMIAL_MAX_WINDOW_SIZE 33

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 0~3阶等权滑动多项式滤波器
 * @details 在最新样本时刻求值, 输出原量及其1~3阶时间导数, 单位为U和U/s^n。
 * Init成功后按固定采样周期输入有限值, 每个样本Set_Now后计算一次。
 * 未收满窗口时原量直通、导数为0、Ready为false; 高于拟合阶数的导数恒为0。
 * Ready仅表示窗口完整, 不表示数据新鲜。缺测、时间断续或信号坐标重置后应Reset。
 * 调用方负责量纲、角度展开和数值范围, 保证差值及乘加可由float表示。
 * Init、Reset与采样不得并发调用; 窗口末端求值不代表滤波没有响应延迟。
 */
class Class_Filter_Polynomial
{
public:
    bool Init(uint32_t window_size, float d_t = 0.001f,
              uint32_t polynomial_order = 2);

    void Reset(float value = 0.0f);

    inline void Set_Now(float now);

    void TIM_Calculate_PeriodElapsedCallback();

    inline float Get_Out() const;
    inline float Get_First_Derivative() const;
    inline float Get_Second_Derivative() const;
    inline float Get_Third_Derivative() const;
    inline uint32_t Get_Polynomial_Order() const;
    inline bool Get_Ready_Flag() const;

protected:
    float Coefficient[4][FILTER_POLYNOMIAL_MAX_WINDOW_SIZE] = {};
    float Input_Signal[FILTER_POLYNOMIAL_MAX_WINDOW_SIZE] = {};
    float Out[4] = {};
    float Now = 0.0f;

    uint32_t Window_Size = 0;
    uint32_t Polynomial_Order = 0;
    uint32_t Signal_Flag = 0;
    uint32_t Sample_Count = 0;
};

/* Exported function declarations --------------------------------------------*/

inline void Class_Filter_Polynomial::Set_Now(float now)
{
    Now = now;
}

inline float Class_Filter_Polynomial::Get_Out() const
{
    return (Out[0]);
}

inline float Class_Filter_Polynomial::Get_First_Derivative() const
{
    return (Out[1]);
}

inline float Class_Filter_Polynomial::Get_Second_Derivative() const
{
    return (Out[2]);
}

inline float Class_Filter_Polynomial::Get_Third_Derivative() const
{
    return (Out[3]);
}

inline uint32_t Class_Filter_Polynomial::Get_Polynomial_Order() const
{
    return (Polynomial_Order);
}

inline bool Class_Filter_Polynomial::Get_Ready_Flag() const
{
    return (Window_Size != 0 && Sample_Count == Window_Size);
}

#endif
