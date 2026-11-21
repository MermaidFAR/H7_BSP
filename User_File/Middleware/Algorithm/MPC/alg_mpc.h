/**
 * @file alg_mpc.h
 * @brief 小维度定长线性 MPC，采用预处理后的稠密 QP 与盒约束 ADMM。
 */

#ifndef ALG_MPC_H
#define ALG_MPC_H

#include "alg_matrix.h"

/**
 * @details 模型为 x[k+1] = A*x[k] + B*u[k]。在控制周期下离散化 A、B，模型使用
 * 平衡点附近的偏差坐标。x 各分量可有不同量纲（如 rad、rad/s），u、u_min 和
 * u_max 使用相同控制量单位（如 N·m）。如果应用在外部叠加平衡输入，限幅也应
 * 转为偏差坐标。在线每次求解使用一个恒定的状态参考，输入参考为零。
 *
 * 代价为 1/2 * (sum_{k=1}^{N-1} e[k]'Qe[k] + e[N]'P e[N]
 *                   + sum_{k=0}^{N-1} u[k]'R u[k])，其中 e[k] = x[k] - x_ref。
 * 先按状态和输入的允许偏差归一化，再选择对称半正定 Q、P 与对称正定 R。
 * 半正定权重按对角项缩放后校验，零主元容许少量浮点舍入误差。
 * rho 为正，和 R 的尺度一致。可从典型 R 对角值起步，用归一化输入比较原始
 * 残差与对偶残差的 L1 范数，然后调整 rho 和绝对容差。Init 在控制循环外生成
 * 凝聚 Hessian 的 Cholesky 因子。病态权重或数值溢出会导致 Init 失败。
 *
 * 在线每次至多执行 max_iterations 次（硬上限 32）。实例按值保存工作内存，
 * 最大规格 6 状态、2 输入、10 步在 STM32H7 上占 2812 B 对象空间，建议静态存放实例。
 * 32 次迭代最多有 12160 个三角求解乘加项，另有对角缩放、投影和残差计算。
 * 480 MHz 下 1 ms 的总预算为 480000 个周期，实际占用需按目标构建与任务负载上板测量。
 * 在线局部数组只有两个长度为 20 的向量（共 160 B），Init 使用较多临时矩阵。
 * Get_Out() 返回投影后的首个输入。即使达到迭代上限仍满足输入盒约束，
 * 调用者用 Is_Converged() 判断是否达到容差。计算数值异常时返回 false 并
 * 以盒内最接近零的输出回退。只在 Init 成功后使用。实例由单一上下文持有。
 * @see https://arxiv.org/abs/2310.16985
 * @code
 * static Class_MPC<2, 1, 5> mpc;
 * // 1 ms 下的归一化双积分模型示例。
 * const float a[] = {1.0f, 0.001f, 0.0f, 1.0f};
 * const float b[] = {0.0000005f, 0.001f};
 * const float q[] = {2.0f, 0.0f, 0.0f, 1.0f};
 * const float r[] = {0.1f}, lower[] = {-1.0f}, upper[] = {1.0f};
 * const Class_Matrix_f32<2, 2> weight(q);
 * if (mpc.Init(Class_Matrix_f32<2, 2>(a), Class_Matrix_f32<2, 1>(b),
 *              weight, Class_Matrix_f32<1, 1>(r), weight,
 *              Class_Matrix_f32<1, 1>(lower), Class_Matrix_f32<1, 1>(upper)))
 * {
 *     const float state[] = {0.1f, 0.0f};
 *     mpc.Set_Now(Class_Matrix_f32<2, 1>(state));
 *     mpc.Set_Ref(Class_Matrix_f32<2, 1>());
 *     if (mpc.TIM_Calculate_PeriodElapsedCallback())
 *     {
 *         const float command = mpc.Get_Out().Data[0];
 *         const bool converged = mpc.Is_Converged();
 *         (void)command;
 *         (void)converged;
 *     }
 * }
 * @endcode
 */
template<int State_Dim, int Control_Dim, int Horizon>
class Class_MPC
{
    static_assert(State_Dim >= 1 && State_Dim <= 6, "MPC supports 1 to 6 states");
    static_assert(Control_Dim >= 1 && Control_Dim <= 2, "MPC supports 1 to 2 inputs");
    static_assert(Horizon >= 1 && Horizon <= 10, "MPC supports horizons of 1 to 10 steps");

    static constexpr int Input_Count = Control_Dim * Horizon;

public:
    static constexpr uint32_t MAX_ITERATIONS = 32U;

    bool Init(const Class_Matrix_f32<State_Dim, State_Dim> &a,
              const Class_Matrix_f32<State_Dim, Control_Dim> &b,
              const Class_Matrix_f32<State_Dim, State_Dim> &q,
              const Class_Matrix_f32<Control_Dim, Control_Dim> &r,
              const Class_Matrix_f32<State_Dim, State_Dim> &terminal_weight,
              const Class_Matrix_f32<Control_Dim, 1> &lower,
              const Class_Matrix_f32<Control_Dim, 1> &upper,
              float rho = 1.0f, uint32_t max_iterations = 16U,
              float tolerance = 1.0e-3f, bool warm_start = true)
    {
        Initialized = false;
        Converged = false;
        Iterations = 0U;
        if (!a.Is_Finite() || !b.Is_Finite() || !lower.Is_Finite() || !upper.Is_Finite() ||
            !isfinite(rho) || rho <= 0.0f || !isfinite(tolerance) || tolerance <= 0.0f ||
            max_iterations == 0U || max_iterations > MAX_ITERATIONS)
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
        if (!Validate_Weight(q, true) || !Validate_Weight(r, false) ||
            !Validate_Weight(terminal_weight, true) ||
            !Prepare_Model(a, b, q, r, terminal_weight, rho))
        {
            return false;
        }

        U_Min = lower;
        U_Max = upper;
        X_Now = Class_Matrix_f32<State_Dim, 1>();
        X_Ref = Class_Matrix_f32<State_Dim, 1>();
        Rho = rho;
        Tolerance = tolerance;
        Iteration_Limit = max_iterations;
        Warm_Start = warm_start;
        Initialized = true;
        Reset_Warm_Start();
        return true;
    }

    void Set_Now(const Class_Matrix_f32<State_Dim, 1> &state) { X_Now = state; }
    void Set_Ref(const Class_Matrix_f32<State_Dim, 1> &reference) { X_Ref = reference; }

    void Reset_Warm_Start()
    {
        Reset_Iterates();
        Converged = false;
        Iterations = 0U;
    }

    const Class_Matrix_f32<Control_Dim, 1> &Get_Out() const { return Out; }
    bool Is_Converged() const { return Converged; }
    uint32_t Get_Iterations() const { return Iterations; }

    bool TIM_Calculate_PeriodElapsedCallback()
    {
        if (!Initialized)
        {
            return false;
        }
        if (!Warm_Start)
        {
            Reset_Iterates();
        }

        float negative_gradient[Input_Count];
        float primal[Input_Count];
        for (int i = 0; i < Input_Count; ++i)
        {
            float value = 0.0f;
            for (int j = 0; j < State_Dim; ++j)
            {
                value += Gradient_Ref[i][j] * X_Ref.Data[j] - Gradient_Now[i][j] * X_Now.Data[j];
            }
            negative_gradient[i] = value;
        }

        float primal_residual = 0.0f;
        float dual_residual = 0.0f;
        Converged = false;
        Iterations = 0U;
        while (Iterations < Iteration_Limit)
        {
            for (int i = 0; i < Input_Count; ++i)
            {
                primal[i] = negative_gradient[i] + Rho * (Projected.Data[i] - Dual.Data[i]);
            }
            Solve_In_Place(primal);

            primal_residual = 0.0f;
            dual_residual = 0.0f;
            for (int i = 0; i < Input_Count; ++i)
            {
                const int channel = i % Control_Dim;
                const float projection = Basic_Math_Constrain(primal[i] + Dual.Data[i],
                                                               U_Min.Data[channel], U_Max.Data[channel]);
                const float residual = primal[i] - projection;
                primal_residual += fabsf(residual);
                dual_residual += fabsf(projection - Projected.Data[i]);
                Projected.Data[i] = projection;
                Dual.Data[i] += residual;
            }
            dual_residual *= Rho;
            ++Iterations;
            if (primal_residual <= Tolerance && dual_residual <= Tolerance)
            {
                Converged = true;
                break;
            }
        }

        // 在发布输出时统一检查整个解。L1 残差会传播任一通道的 NaN/Inf。
        if (!isfinite(primal_residual + dual_residual) || !Dual.Is_Finite())
        {
            Converged = false;
            Reset_Iterates();
            return false;
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            Out.Data[i] = Projected.Data[i];
        }
        if (Warm_Start)
        {
            // 将上一解前移一步，末步输入和对偶量保持不变。
            for (int i = 0; i < Input_Count - Control_Dim; ++i)
            {
                Projected.Data[i] = Projected.Data[i + Control_Dim];
                Dual.Data[i] = Dual.Data[i + Control_Dim];
            }
        }
        return true;
    }

private:
    template<int Dim>
    static bool Cholesky_In_Place(Class_Matrix_f32<Dim, Dim> &matrix,
                                  bool semidefinite = false, float roundoff = 0.0f)
    {
        for (int i = 0; i < Dim; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                float value = matrix[i][j];
                for (int k = 0; k < j; ++k)
                {
                    value -= matrix[i][k] * matrix[j][k];
                }
                if (!isfinite(value))
                {
                    return false;
                }
                if (i == j)
                {
                    if (value > 0.0f)
                    {
                        matrix[i][i] = sqrtf(value);
                    }
                    else if (semidefinite && value >= -roundoff)
                    {
                        matrix[i][i] = 0.0f;
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (matrix[j][j] != 0.0f)
                {
                    matrix[i][j] = value / matrix[j][j];
                    if (!isfinite(matrix[i][j]))
                    {
                        return false;
                    }
                }
                else if (fabsf(value) > roundoff)
                {
                    return false;
                }
                else
                {
                    matrix[i][j] = 0.0f;
                }
            }
        }
        return true;
    }

    template<int Dim>
    static bool Validate_Weight(const Class_Matrix_f32<Dim, Dim> &weight, bool semidefinite)
    {
        Class_Matrix_f32<Dim, Dim> factor;
        float scales[Dim];
        for (int i = 0; i < Dim; ++i)
        {
            const float diagonal = weight[i][i];
            if (!isfinite(diagonal) || diagonal < 0.0f)
            {
                return false;
            }
            scales[i] = semidefinite ? sqrtf(diagonal) : 0.0f;
            factor[i][i] = scales[i] > 0.0f ? 1.0f : diagonal;
            for (int j = 0; j < i; ++j)
            {
                const float cross_weight = weight[i][j];
                if (cross_weight != weight[j][i] ||
                    ((diagonal == 0.0f || weight[j][j] == 0.0f) && cross_weight != 0.0f))
                {
                    return false;
                }
                factor[i][j] = semidefinite && cross_weight != 0.0f
                                   ? (cross_weight / scales[i]) / scales[j]
                                   : cross_weight;
            }
        }
        return Cholesky_In_Place(factor, semidefinite, 8.0f * FLT_EPSILON);
    }

    bool Prepare_Model(const Class_Matrix_f32<State_Dim, State_Dim> &a,
                       const Class_Matrix_f32<State_Dim, Control_Dim> &b,
                       const Class_Matrix_f32<State_Dim, State_Dim> &q,
                       const Class_Matrix_f32<Control_Dim, Control_Dim> &r,
                       const Class_Matrix_f32<State_Dim, State_Dim> &terminal_weight,
                       float rho)
    {
        memset(Factor.Data, 0, sizeof(Factor.Data));
        memset(Gradient_Now.Data, 0, sizeof(Gradient_Now.Data));
        memset(Gradient_Ref.Data, 0, sizeof(Gradient_Ref.Data));

        Class_Matrix_f32<State_Dim, State_Dim> power;
        for (int i = 0; i < State_Dim; ++i)
        {
            power[i][i] = 1.0f;
        }
        Class_Matrix_f32<State_Dim, Input_Count> sensitivity;
        for (int step = 0; step < Horizon; ++step)
        {
            // x[k] = A^k*x[0] + S_k*U。已生效的输入列为 [0, (step+1)*Control_Dim)。
            power = a * power;
            sensitivity = a * sensitivity;
            const int offset = step * Control_Dim;
            for (int i = 0; i < State_Dim; ++i)
            {
                for (int j = 0; j < Control_Dim; ++j)
                {
                    sensitivity[i][offset + j] = b[i][j];
                }
            }
            const Class_Matrix_f32<State_Dim, State_Dim> &state_weight = step + 1 == Horizon ? terminal_weight : q;
            const Class_Matrix_f32<State_Dim, Input_Count> weighted = state_weight * sensitivity;
            for (int i = 0; i < offset + Control_Dim; ++i)
            {
                for (int j = 0; j <= i; ++j)
                {
                    for (int k = 0; k < State_Dim; ++k)
                    {
                        Factor[i][j] += weighted[k][i] * sensitivity[k][j];
                    }
                }
                for (int j = 0; j < State_Dim; ++j)
                {
                    Gradient_Ref[i][j] += weighted[j][i];
                    for (int k = 0; k < State_Dim; ++k)
                    {
                        Gradient_Now[i][j] += weighted[k][i] * power[k][j];
                    }
                }
            }
            for (int i = 0; i < Control_Dim; ++i)
            {
                for (int j = 0; j <= i; ++j)
                {
                    Factor[offset + i][offset + j] += r[i][j];
                }
                Factor[offset + i][offset + i] += rho;
            }
        }
        if (!Gradient_Ref.Is_Finite() || !Gradient_Now.Is_Finite() || !Cholesky_In_Place(Factor))
        {
            return false;
        }
        for (int i = 0; i < Input_Count; ++i)
        {
            Factor[i][i] = 1.0f / Factor[i][i];
        }
        return true;
    }

    void Solve_In_Place(float *values) const
    {
        // 先解 L*y = rhs，再解 L^T*x = y。对角存的是倒数。
        for (int i = 0; i < Input_Count; ++i)
        {
            float value = values[i];
            for (int j = 0; j < i; ++j)
            {
                value -= Factor[i][j] * values[j];
            }
            values[i] = value * Factor[i][i];
        }
        for (int i = Input_Count - 1; i >= 0; --i)
        {
            float value = values[i];
            for (int j = i + 1; j < Input_Count; ++j)
            {
                value -= Factor[j][i] * values[j];
            }
            values[i] = value * Factor[i][i];
        }
    }

    void Reset_Iterates()
    {
        for (int i = 0; i < Input_Count; ++i)
        {
            const int channel = i % Control_Dim;
            Projected.Data[i] = Basic_Math_Constrain(0.0f, U_Min.Data[channel], U_Max.Data[channel]);
            Dual.Data[i] = 0.0f;
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            Out.Data[i] = Projected.Data[i];
        }
    }

    Class_Matrix_f32<Input_Count, Input_Count> Factor;
    Class_Matrix_f32<Input_Count, State_Dim> Gradient_Now;
    Class_Matrix_f32<Input_Count, State_Dim> Gradient_Ref;
    Class_Matrix_f32<Input_Count, 1> Projected;
    Class_Matrix_f32<Input_Count, 1> Dual;
    Class_Matrix_f32<State_Dim, 1> X_Now;
    Class_Matrix_f32<State_Dim, 1> X_Ref;
    Class_Matrix_f32<Control_Dim, 1> U_Min;
    Class_Matrix_f32<Control_Dim, 1> U_Max;
    Class_Matrix_f32<Control_Dim, 1> Out;
    float Rho = 0.0f;
    float Tolerance = 0.0f;
    uint32_t Iteration_Limit = 0U;
    uint32_t Iterations = 0U;
    bool Initialized = false;
    bool Warm_Start = true;
    bool Converged = false;
};

#endif
