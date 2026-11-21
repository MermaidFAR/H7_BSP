/**
 * @file alg_lqr.h
 * @brief 定长全状态反馈，控制律为 u = K * (x_ref - x_now)。
 */

#ifndef ALG_LQR_H
#define ALG_LQR_H

#include "alg_matrix.h"

/**
 * @details x_ref 和 x_now 分量应使用相同量纲、顺序和坐标系，例如 rad、rad/s。
 * K[i][j] 的量纲为第 i 路控制量除以第 j 个状态量，限幅与控制量同单位。
 * 在应用层对连续或离散模型线性化，选择 Q、R 离线求 K，并核对闭环极点与限幅。
 * 离散 K 要与实际控制周期匹配。腿长等增益调度由应用层计算后调用 Set_Gain，
 * 同时更新对应平衡状态。非零平衡输入应由应用层按实际执行器限幅处理。
 * Init 成功后使用。实例由单一调用上下文持有，Set_Now/Set_Ref 接收已校验的状态。
 * 反馈计算异常时返回 false，整组输出回退到限幅内最接近零的值。
 * @see https://github.com/HKUAstar/Balance-Model-2026
 * @see https://github.com/JerryGong0911/RM2024_Reborn_LegWheel_Chassis
 * @code
 * Class_LQR<2, 1> lqr;
 * const float gain[] = {2.0f, 0.5f};
 * const float lower[] = {-3.0f}, upper[] = {3.0f};
 * if (lqr.Init(Class_Matrix_f32<1, 2>(gain),
 *              Class_Matrix_f32<1, 1>(lower), Class_Matrix_f32<1, 1>(upper)))
 * {
 *     const float measured[] = {0.1f, 0.0f};
 *     lqr.Set_Ref(Class_Matrix_f32<2, 1>());
 *     lqr.Set_Now(Class_Matrix_f32<2, 1>(measured));
 *     if (lqr.TIM_Calculate_PeriodElapsedCallback())
 *     {
 *         const float command = lqr.Get_Out().Data[0];
 *         (void)command;
 *     }
 * }
 * @endcode
 */
template<int State_Dim, int Control_Dim>
class Class_LQR
{
    static_assert(State_Dim > 0 && Control_Dim > 0, "LQR dimensions must be positive");

public:
    bool Init(const Class_Matrix_f32<Control_Dim, State_Dim> &gain,
              const Class_Matrix_f32<Control_Dim, 1> &lower,
              const Class_Matrix_f32<Control_Dim, 1> &upper)
    {
        if (!lower.Is_Finite() || !upper.Is_Finite())
        {
            return false;
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            if (lower.Data[i] > upper.Data[i])
            {
                return false;
            }
        }
        if (!Set_Gain(gain))
        {
            return false;
        }
        U_Min = lower;
        U_Max = upper;
        X_Ref = Class_Matrix_f32<State_Dim, 1>();
        X_Now = Class_Matrix_f32<State_Dim, 1>();
        Set_Zero_Out();
        return true;
    }

    bool Set_Gain(const Class_Matrix_f32<Control_Dim, State_Dim> &gain)
    {
        if (!gain.Is_Finite())
        {
            return false;
        }
        K = gain;
        return true;
    }

    void Set_Ref(const Class_Matrix_f32<State_Dim, 1> &reference) { X_Ref = reference; }
    void Set_Now(const Class_Matrix_f32<State_Dim, 1> &state) { X_Now = state; }

    bool TIM_Calculate_PeriodElapsedCallback()
    {
        Out = K * (X_Ref - X_Now);
        if (!Out.Is_Finite())
        {
            Set_Zero_Out();
            return false;
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            Out.Data[i] = Basic_Math_Constrain(Out.Data[i], U_Min.Data[i], U_Max.Data[i]);
        }
        return true;
    }

    const Class_Matrix_f32<Control_Dim, 1> &Get_Out() const { return Out; }

private:
    void Set_Zero_Out()
    {
        for (int i = 0; i < Control_Dim; ++i)
        {
            Out.Data[i] = Basic_Math_Constrain(0.0f, U_Min.Data[i], U_Max.Data[i]);
        }
    }

    Class_Matrix_f32<Control_Dim, State_Dim> K;
    Class_Matrix_f32<State_Dim, 1> X_Ref;
    Class_Matrix_f32<State_Dim, 1> X_Now;
    Class_Matrix_f32<Control_Dim, 1> U_Min;
    Class_Matrix_f32<Control_Dim, 1> U_Max;
    Class_Matrix_f32<Control_Dim, 1> Out;
};

#endif
