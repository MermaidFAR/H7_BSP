/**
 * @file alg_filter_polynomial.cpp
 * @author zzm
 * @brief 固定周期标量多项式滤波与微分
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_polynomial.h"

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 生成窗口末端的等权最小二乘系数, 成功后清空历史
 * @param __Window_Size 样本数, 大于拟合阶数且不超过33
 * @param __D_T 采样周期, 单位s, 正常有限正数
 * @param __Polynomial_Order 拟合阶数, 0~3
 * @return 参数及有效导数系数可由float表示时为true; 失败保留原状态
 */
bool Class_Filter_Polynomial::Init(uint32_t __Window_Size, float __D_T,
                                   uint32_t __Polynomial_Order)
{
    if (__Polynomial_Order > 3 || __Window_Size <= __Polynomial_Order ||
        __Window_Size > FILTER_POLYNOMIAL_MAX_WINDOW_SIZE ||
        Basic_Math_Is_Invalid_Float(__D_T) || __D_T <= 0.0f)
    {
        return (false);
    }

    float coefficient[4][FILTER_POLYNOMIAL_MAX_WINDOW_SIZE] = {};
    float coefficient_max[4] = {};
    double n = __Window_Size;
    double half = (n - 1.0) / 2.0;
    double n_squared = n * n;
    double norm_1 = n * (n_squared - 1.0) / 12.0;
    double mean_2 = (n_squared - 1.0) / 12.0;
    double norm_2 = n * (n_squared - 1.0) * (n_squared - 4.0) / 180.0;
    double projection_3 = (3.0 * n_squared - 7.0) / 20.0;
    double norm_3 = n * (n_squared - 1.0) * (n_squared - 4.0) *
                    (n_squared - 9.0) / 2800.0;
    double sampling_frequency = 1.0 / (double)__D_T;

    for (uint32_t i = 0; i < __Window_Size; i++)
    {
        // 对称采样坐标上的离散正交基, 在窗口末端half处求值及求导。
        double u = (double)i - half;
        double value[4] = {1.0 / n, 0.0, 0.0, 0.0};
        if (__Polynomial_Order >= 1)
        {
            double basis = u / norm_1;
            value[0] += half * basis;
            value[1] += basis;
        }
        if (__Polynomial_Order >= 2)
        {
            double basis = (u * u - mean_2) / norm_2;
            value[0] += (half * half - mean_2) * basis;
            value[1] += 2.0 * half * basis;
            value[2] += 2.0 * basis;
        }
        if (__Polynomial_Order >= 3)
        {
            double basis = (u * u * u - projection_3 * u) / norm_3;
            value[0] += (half * half * half - projection_3 * half) * basis;
            value[1] += (3.0 * half * half - projection_3) * basis;
            value[2] += 6.0 * half * basis;
            value[3] += 6.0 * basis;
        }

        double scale = 1.0;
        for (uint32_t derivative = 0; derivative <= __Polynomial_Order; derivative++)
        {
            double scaled = value[derivative] * scale;
            if (fabs(scaled) > (double)FLT_MAX)
            {
                return (false);
            }
            float result = (float)scaled;
            if (Basic_Math_Is_Invalid_Float(result))
            {
                return (false);
            }
            coefficient[derivative][i] = result;
            if (fabsf(result) > coefficient_max[derivative])
            {
                coefficient_max[derivative] = fabsf(result);
            }
            scale *= sampling_frequency;
        }
    }

    for (uint32_t derivative = 0; derivative <= __Polynomial_Order; derivative++)
    {
        if (coefficient_max[derivative] == 0.0f)
        {
            return (false);
        }
    }

    memcpy(Coefficient, coefficient, sizeof(Coefficient));
    Window_Size = __Window_Size;
    Polynomial_Order = __Polynomial_Order;
    Reset();
    return (true);
}

/**
 * @brief 清空窗口计数, 原量对齐到给定值, 导数清零; 不创建虚构历史样本
 */
void Class_Filter_Polynomial::Reset(float __Value)
{
    Now = __Value;
    Out[0] = __Value;
    Out[1] = 0.0f;
    Out[2] = 0.0f;
    Out[3] = 0.0f;
    Signal_Flag = 0;
    Sample_Count = 0;
}

void Class_Filter_Polynomial::TIM_Calculate_PeriodElapsedCallback()
{
    if (Window_Size == 0)
    {
        return;
    }

    Input_Signal[Signal_Flag] = Now;
    Signal_Flag++;
    if (Signal_Flag == Window_Size)
    {
        Signal_Flag = 0;
    }
    if (Sample_Count < Window_Size)
    {
        Sample_Count++;
    }

    Out[0] = Now;
    Out[1] = 0.0f;
    Out[2] = 0.0f;
    Out[3] = 0.0f;
    if (Sample_Count < Window_Size)
    {
        return;
    }

    uint32_t coefficient = 0;
    for (uint32_t i = Signal_Flag; i < Window_Size; i++)
    {
        float difference = Input_Signal[i] - Now;
        for (uint32_t derivative = 0; derivative <= Polynomial_Order; derivative++)
        {
            Out[derivative] += Coefficient[derivative][coefficient] * difference;
        }
        coefficient++;
    }
    for (uint32_t i = 0; i < Signal_Flag; i++)
    {
        float difference = Input_Signal[i] - Now;
        for (uint32_t derivative = 0; derivative <= Polynomial_Order; derivative++)
        {
            Out[derivative] += Coefficient[derivative][coefficient] * difference;
        }
        coefficient++;
    }
}
