/**
 * @file alg_mpc.h
 * @brief 定长线性 MPC，采用缓存 Riccati 递推与状态/输入盒约束 ADMM。
 */

#ifndef ALG_MPC_H
#define ALG_MPC_H

#include "alg_matrix.h"

/**
 * @details x[k+1]=A*x[k]+B*u[k]。使用实际控制周期离散化，并围绕平衡点归一化。
 * 状态、参考和状态限幅使用同一坐标和量纲（如 rad、rad/s）；控制量与输入限幅
 * 同量纲（如 N·m）。选择半正定 Q、P 和正定 R。rho 与归一化权重同量级，
 * 调整 rho 和容差时同时观察状态与输入的原始、对偶残差。
 *
 * Init 分别缓存启用和停用状态投影时的增广 Riccati 系数，保留指定的终端权重；
 * 收敛到稳态的前缀可共用 K∞、P∞、C1、C2。在线改变盒限或状态掩码无需重新分解。
 * 若要与无限时域 LQR 比较，将 P 设为原问题的 DARE 解并给出足够宽的输入限幅。
 * 状态盒作用于可优化的 x[1..N]。输出前用投影后的输入重新递推并检查实际预测，
 * 有限迭代时可沿上次可行序列或稳态反馈生成的可行序列回退；找不到可行序列则返回
 * false。成功返回的预测位于所有启用的盒内。
 * Get_Out() 在失败时返回输入盒中最接近零的值。Is_Converged() 单独报告 ADMM 容差。
 *
 * 在线计算与存储均随 Horizon 线性增长，最多 32 次 ADMM 迭代。实例建议静态存放，
 * 且由单一上下文持有；Init 有额外临时空间，在启动或配置阶段调用。实际周期耗时需
 * 按目标构建和 RTOS 负载测量。
 * @see https://arxiv.org/abs/2310.16985
 * @code
 * static Class_MPC<2, 1, 10> mpc;
 * const float a[] = {1.0f, 0.01f, 0.0f, 1.0f};
 * const float b[] = {0.00005f, 0.01f};
 * const float q[] = {2.0f, 0.0f, 0.0f, 1.0f};
 * const float low[] = {-1.0f}, high[] = {1.0f}, r[] = {0.1f};
 * const float x_low[] = {-0.52f, -1.0f}, x_high[] = {0.52f, 1.0f};
 * if (mpc.Init(Class_Matrix_f32<2, 2>(a), Class_Matrix_f32<2, 1>(b),
 *              Class_Matrix_f32<2, 2>(q), Class_Matrix_f32<1, 1>(r),
 *              Class_Matrix_f32<2, 2>(q), Class_Matrix_f32<1, 1>(low),
 *              Class_Matrix_f32<1, 1>(high)))
 * {
 *     mpc.Set_State_Bounds(Class_Matrix_f32<2, 1>(x_low), Class_Matrix_f32<2, 1>(x_high));
 *     const float sample[] = {0.1f, 0.0f};
 *     mpc.Set_Now(Class_Matrix_f32<2, 1>(sample));
 *     const bool feasible = mpc.TIM_Calculate_PeriodElapsedCallback();
 *     (void)feasible;
 * }
 * @endcode
 */
template <int State_Dim, int Control_Dim, int Horizon>
class Class_MPC
{
    static_assert(State_Dim >= 1 && State_Dim <= 6, "MPC supports 1 to 6 states");
    static_assert(Control_Dim >= 1 && Control_Dim <= 2, "MPC supports 1 to 2 inputs");
    static_assert(Horizon >= 1 && Horizon <= 25, "MPC supports 1 to 25 steps");

  public:
    using State = Class_Matrix_f32<State_Dim, 1>;
    using Input = Class_Matrix_f32<Control_Dim, 1>;
    using State_Matrix = Class_Matrix_f32<State_Dim, State_Dim>;
    using Input_Model = Class_Matrix_f32<State_Dim, Control_Dim>;
    using Input_Matrix = Class_Matrix_f32<Control_Dim, Control_Dim>;
    static constexpr uint32_t MAX_ITERATIONS = 32U;
    static constexpr uint32_t ALL_STATES = (1U << State_Dim) - 1U;

    bool Init(const State_Matrix &a, const Input_Model &b, const State_Matrix &q,
              const Input_Matrix &r, const State_Matrix &terminal_weight,
              const Input &lower, const Input &upper, float rho = 1.0f,
              uint32_t max_iterations = 16U, float tolerance = 1.0e-3f, bool warm_start = true)
    {
        if (!a.Is_Finite() || !b.Is_Finite() || !Valid_Bounds(lower, upper) ||
            !isfinite(rho) || rho <= 0.0f || !isfinite(tolerance) || tolerance <= 0.0f ||
            max_iterations == 0U || max_iterations > MAX_ITERATIONS ||
            !Valid_Weight(q, true) || !Valid_Weight(terminal_weight, true) || !Valid_Weight(r, false))
        {
            return false;
        }

        Riccati_Cache prepared[2];
        if (!Build_Cache(a, b, q, r, terminal_weight, rho, false, prepared[0]) ||
            !Build_Cache(a, b, q, r, terminal_weight, rho, true, prepared[1]))
        {
            return false;
        }

        Cache[0] = prepared[0];
        Cache[1] = prepared[1];
        A = a;
        B = b;
        Q = q;
        P = terminal_weight;
        U_Min = lower;
        U_Max = upper;
        X_Now = State();
        X_Ref = State();
        State_Mask = 0U;
        Rho = rho;
        Tolerance = tolerance;
        Iteration_Limit = max_iterations;
        Warm_Start = warm_start;
        Initialized = true;
        Reset_Warm_Start();
        return true;
    }

    bool Set_Input_Bounds(const Input &lower, const Input &upper)
    {
        if (!Initialized || !Valid_Bounds(lower, upper))
        {
            return false;
        }
        U_Min = lower;
        U_Max = upper;
        Reset_Warm_Start();
        return true;
    }

    /** @brief mask 的第 i 位启用第 i 个状态分量；mask 为零时关闭状态盒。 */
    bool Set_State_Bounds(const State &lower, const State &upper, uint32_t mask = ALL_STATES)
    {
        if (!Initialized || (mask & ~ALL_STATES) != 0U || !Valid_Bounds(lower, upper))
        {
            return false;
        }
        X_Min = lower;
        X_Max = upper;
        State_Mask = mask;
        Reset_Warm_Start();
        return true;
    }

    void Set_Now(const State &state)
    {
        X_Now = state;
    }
    void Set_Ref(const State &reference)
    {
        X_Ref = reference;
    }

    void Reset_Warm_Start()
    {
        Invalidate_Result();
        Iterations = 0U;
        Seeded = false;
        Previous_Plan_Valid = false;
    }

    const Input &Get_Out() const
    {
        return Out;
    }
    bool Is_Converged() const
    {
        return Converged;
    }
    uint32_t Get_Iterations() const
    {
        return Iterations;
    }

    /** @brief 读取最近成功求解的实际预测状态，step 取 1..Horizon。 */
    bool Get_Predicted_State(int step, State &state) const
    {
        if (!Prediction_Valid || step < 1 || step > Horizon)
        {
            return false;
        }
        state = Prediction[step - 1];
        return true;
    }

    bool TIM_Calculate_PeriodElapsedCallback()
    {
        Converged = false;
        Prediction_Valid = false;
        Iterations = 0U;
        if (!Initialized || !X_Now.Is_Finite() || !X_Ref.Is_Finite())
        {
            return Fail();
        }

        const State reference_gradient = (Q * X_Ref) * -1.0f;
        const State terminal_gradient = (P * X_Ref) * -1.0f;
        if (!reference_gradient.Is_Finite() || !terminal_gradient.Is_Finite())
        {
            return Fail();
        }
        if (!Warm_Start || !Seeded)
        {
            Reset_Iterates();
        }

        const Riccati_Cache &cache = Cache[State_Mask == 0U ? 0 : 1];
        while (Iterations < Iteration_Limit)
        {
            float primal = 0.0f;
            float dual = 0.0f;
            if (!Backward(cache, reference_gradient, terminal_gradient) || !Forward(cache, primal, dual))
            {
                return Fail();
            }
            ++Iterations;
            if (primal <= Tolerance && dual <= Tolerance)
            {
                Converged = true;
                break;
            }
        }

        // ADMM 的两组投影值可能还不满足动力学。先复算并核查实际轨迹，再公布输入。
        if (!Certify_Prediction(cache))
        {
            return Fail();
        }
        Out = Projected_U[0];
        Prediction_Valid = true;
        if (Warm_Start)
        {
            for (int step = 0; step + 1 < Horizon; ++step)
            {
                Projected_U[step] = Projected_U[step + 1];
                Dual_U[step] = Dual_U[step + 1];
                Projected_X[step] = Projected_X[step + 1];
                Dual_X[step] = Dual_X[step + 1];
            }
        }
        return true;
    }

  private:
    using Gain = Class_Matrix_f32<Control_Dim, State_Dim>;

    struct Riccati_Stage
    {
        Gain K;
        Input_Matrix C1;
    };

    struct Riccati_Cache
    {
        Riccati_Stage Stages[Horizon];
        Riccati_Stage Steady;
        State_Matrix P_Infinity;
        State_Matrix C2;
        int Steady_Prefix;
    };

    struct Riccati_Work
    {
        double P[State_Dim][State_Dim];
        double K[Control_Dim][State_Dim];
        double C1[Control_Dim][Control_Dim];
    };

    template <int Dim>
    static bool Valid_Bounds(const Class_Matrix_f32<Dim, 1> &lower,
                             const Class_Matrix_f32<Dim, 1> &upper)
    {
        for (int i = 0; i < Dim; ++i)
        {
            if (!isfinite(lower.Data[i]) || !isfinite(upper.Data[i]) || lower.Data[i] > upper.Data[i])
            {
                return false;
            }
        }
        return true;
    }

    template <int Dim>
    static bool Valid_Weight(const Class_Matrix_f32<Dim, Dim> &matrix, bool semidefinite)
    {
        double factor[Dim][Dim] = {};
        double scale[Dim];
        for (int i = 0; i < Dim; ++i)
        {
            const float diagonal = matrix[i][i];
            if (!isfinite(diagonal) || diagonal < 0.0f)
            {
                return false;
            }
            scale[i] = sqrt((double) diagonal);
        }
        for (int i = 0; i < Dim; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                const float entry = matrix[i][j];
                if (!isfinite(entry) || entry != matrix[j][i] ||
                    ((scale[i] == 0.0 || scale[j] == 0.0) && entry != 0.0f))
                {
                    return false;
                }
                double pivot = entry == 0.0f ? 0.0 : (double) entry / (scale[i] * scale[j]);
                for (int k = 0; k < j; ++k)
                {
                    pivot -= factor[i][k] * factor[j][k];
                }
                if (i == j)
                {
                    if (semidefinite && fabs(pivot) <= 1e-12)
                    {
                        factor[i][i] = 0.0;
                    }
                    else if (pivot > 0.0)
                    {
                        factor[i][i] = sqrt(pivot);
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (factor[j][j] != 0.0)
                {
                    factor[i][j] = pivot / factor[j][j];
                }
                else if (fabs(pivot) > 1e-12)
                {
                    return false;
                }
            }
        }
        return true;
    }

    static bool Fits_Float(double value)
    {
        return isfinite(value) && fabs(value) <= (double) FLT_MAX;
    }

    static bool Riccati_Step(const State_Matrix &a, const Input_Model &b, const State_Matrix &q,
                             const Input_Matrix &r, double rho_x, double rho_u,
                             const double (&next_p)[State_Dim][State_Dim], Riccati_Work &work)
    {
        double pa[State_Dim][State_Dim] = {};
        double pb[State_Dim][Control_Dim] = {};
        for (int i = 0; i < State_Dim; ++i)
        {
            for (int k = 0; k < State_Dim; ++k)
            {
                for (int j = 0; j < State_Dim; ++j)
                {
                    pa[i][j] += next_p[i][k] * (double) a[k][j];
                }
                for (int j = 0; j < Control_Dim; ++j)
                {
                    pb[i][j] += next_p[i][k] * (double) b[k][j];
                }
            }
        }
        double hessian[Control_Dim][Control_Dim];
        double gradient[Control_Dim][State_Dim] = {};
        for (int i = 0; i < Control_Dim; ++i)
        {
            for (int j = 0; j < Control_Dim; ++j)
            {
                hessian[i][j] = (double) r[i][j] + (i == j ? rho_u : 0.0);
                for (int k = 0; k < State_Dim; ++k)
                {
                    hessian[i][j] += (double) b[k][i] * pb[k][j];
                }
            }
            for (int k = 0; k < State_Dim; ++k)
            {
                for (int j = 0; j < State_Dim; ++j)
                {
                    gradient[i][j] += (double) b[k][i] * pa[k][j];
                }
            }
        }
        const double h00 = hessian[0][0];
        if (!isfinite(h00) || h00 <= 0.0)
        {
            return false;
        }
        work.C1[0][0] = 1.0 / h00;
        if (Control_Dim == 2)
        {
            const double cross = 0.5 * (hessian[0][1] + hessian[1][0]);
            const double ratio = cross / h00;
            const double pivot = hessian[1][1] - ratio * cross;
            if (!isfinite(pivot) || pivot <= 0.0)
            {
                return false;
            }
            work.C1[1][1] = 1.0 / pivot;
            work.C1[0][1] = work.C1[1][0] = -ratio * work.C1[1][1];
            work.C1[0][0] += ratio * ratio * work.C1[1][1];
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            for (int j = 0; j < State_Dim; ++j)
            {
                double value = 0.0;
                for (int k = 0; k < Control_Dim; ++k)
                {
                    value += work.C1[i][k] * gradient[k][j];
                }
                work.K[i][j] = value;
            }
        }
        for (int i = 0; i < State_Dim; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                double value = (double) q[i][j] + (i == j ? rho_x : 0.0);
                for (int k = 0; k < State_Dim; ++k)
                {
                    value += (double) a[k][i] * pa[k][j];
                }
                for (int k = 0; k < Control_Dim; ++k)
                {
                    value -= gradient[k][i] * work.K[k][j];
                }
                if (!Fits_Float(value))
                {
                    return false;
                }
                work.P[i][j] = work.P[j][i] = value;
            }
        }
        return true;
    }

    static bool Store_Stage(const Riccati_Work &work, Riccati_Stage &stage)
    {
        for (int i = 0; i < Control_Dim; ++i)
        {
            for (int j = 0; j < Control_Dim; ++j)
            {
                const double value = work.C1[i][j];
                if (!Fits_Float(value) || (i == j && (float) value <= 0.0f))
                {
                    return false;
                }
                stage.C1[i][j] = (float) value;
            }
            for (int j = 0; j < State_Dim; ++j)
            {
                const double value = work.K[i][j];
                if (!Fits_Float(value))
                {
                    return false;
                }
                stage.K[i][j] = (float) value;
            }
        }
        return true;
    }

    static bool Build_Cache(const State_Matrix &a, const Input_Model &b, const State_Matrix &q,
                            const Input_Matrix &r, const State_Matrix &terminal_weight,
                            float rho, bool split_state, Riccati_Cache &cache)
    {
        const double rho_x = split_state ? (double) rho : 0.0;
        const double rho_u = (double) rho;
        double next_p[State_Dim][State_Dim] = {};
        Riccati_Work work;
        bool converged = false;
        for (uint32_t iteration = 0; iteration < 32768U; ++iteration)
        {
            if (!Riccati_Step(a, b, q, r, rho_x, rho_u, next_p, work))
            {
                return false;
            }
            double change = 0.0;
            double scale = 0.0;
            for (int i = 0; i < State_Dim; ++i)
            {
                for (int j = 0; j < State_Dim; ++j)
                {
                    change = fmax(change, fabs(work.P[i][j] - next_p[i][j]));
                    scale = fmax(scale, fabs(work.P[i][j]));
                    next_p[i][j] = work.P[i][j];
                }
            }
            if (change <= 1e-11 * scale)
            {
                converged = true;
                break;
            }
        }
        if (!converged || !Riccati_Step(a, b, q, r, rho_x, rho_u, next_p, work) ||
            !Store_Stage(work, cache.Steady))
        {
            return false;
        }
        for (int i = 0; i < State_Dim; ++i)
        {
            for (int j = 0; j < State_Dim; ++j)
            {
                cache.P_Infinity[i][j] = (float) next_p[i][j];
                double closed_loop = (double) a[j][i];
                for (int k = 0; k < Control_Dim; ++k)
                {
                    closed_loop -= (double) b[j][k] * work.K[k][i];
                }
                if (!Fits_Float(closed_loop))
                {
                    return false;
                }
                cache.C2[i][j] = (float) closed_loop;
                next_p[i][j] = (double) terminal_weight[i][j] + (i == j ? rho_x : 0.0);
            }
        }

        // 保留有限时域的终端代价。只有递推到相同的单精度固定点时才复用稳态前缀。
        cache.Steady_Prefix = 0;
        for (int step = Horizon - 1; step >= 0; --step)
        {
            bool at_steady_state = true;
            for (int i = 0; i < State_Dim; ++i)
            {
                for (int j = 0; j < State_Dim; ++j)
                {
                    const double steady = (double) cache.P_Infinity[i][j];
                    const double scale = fmax(fabs(steady), fabs(next_p[i][j]));
                    at_steady_state = at_steady_state && fabs(next_p[i][j] - steady) <= 1e-8 * scale;
                }
            }
            if (at_steady_state)
            {
                cache.Steady_Prefix = step + 1;
                break;
            }
            if (!Riccati_Step(a, b, q, r, rho_x, rho_u, next_p, work) ||
                !Store_Stage(work, cache.Stages[step]))
            {
                return false;
            }
            memcpy(next_p, work.P, sizeof(next_p));
        }
        return true;
    }

    bool Backward(const Riccati_Cache &cache, const State &reference, const State &terminal)
    {
        State p = terminal;
        const bool split_state = State_Mask != 0U;
        if (split_state)
        {
            p = p + (Dual_X[Horizon - 1] - Projected_X[Horizon - 1]) * Rho;
        }
        if (!p.Is_Finite())
        {
            return false;
        }
        for (int step = Horizon - 1; step >= 0; --step)
        {
            const bool steady = step < cache.Steady_Prefix;
            const Riccati_Stage &stage = steady ? cache.Steady : cache.Stages[step];
            const Input input_gradient = (Dual_U[step] - Projected_U[step]) * Rho;
            Input rhs = input_gradient;
            for (int i = 0; i < Control_Dim; ++i)
            {
                for (int j = 0; j < State_Dim; ++j)
                {
                    rhs.Data[i] += B[j][i] * p.Data[j];
                }
            }
            Feedforward[step] = stage.C1 * rhs;
            if (!Feedforward[step].Is_Finite())
            {
                return false;
            }
            if (step == 0)
            {
                break;
            }

            State previous = reference;
            if (split_state)
            {
                previous = previous + (Dual_X[step - 1] - Projected_X[step - 1]) * Rho;
            }
            for (int i = 0; i < State_Dim; ++i)
            {
                for (int j = 0; j < State_Dim; ++j)
                {
                    previous.Data[i] += (steady ? cache.C2[i][j] : A[j][i]) * p.Data[j];
                }
                for (int j = 0; j < Control_Dim; ++j)
                {
                    const float gradient = steady ? input_gradient.Data[j] : rhs.Data[j];
                    previous.Data[i] -= stage.K[j][i] * gradient;
                }
            }
            if (!previous.Is_Finite())
            {
                return false;
            }
            p = previous;
        }
        return true;
    }

    bool Project(float value, float lower, float upper, bool bounded, float &projected,
                 float &scaled_dual, float &primal, float &dual) const
    {
        const float relaxed = State_Mask == 0U ? value : value + 0.5f * (value - projected);
        const float argument = relaxed + scaled_dual;
        if (!isfinite(argument))
        {
            return false;
        }
        const float next = bounded ? Basic_Math_Constrain(argument, lower, upper) : argument;
        const float difference = value - next;
        const float displacement = Rho * fabsf(next - projected);
        if (!isfinite(difference) || !isfinite(displacement))
        {
            return false;
        }
        primal = fmaxf(primal, fabsf(difference));
        dual = fmaxf(dual, displacement);
        projected = next;
        scaled_dual = argument - next;
        return isfinite(scaled_dual);
    }

    bool Forward(const Riccati_Cache &cache, float &primal, float &dual)
    {
        State state = X_Now;
        for (int step = 0; step < Horizon; ++step)
        {
            const Riccati_Stage &stage = step < cache.Steady_Prefix ? cache.Steady : cache.Stages[step];
            const Input control = (stage.K * state + Feedforward[step]) * -1.0f;
            state = A * state + B * control;
            if (!control.Is_Finite() || !state.Is_Finite())
            {
                return false;
            }
            for (int i = 0; i < Control_Dim; ++i)
            {
                if (!Project(control.Data[i], U_Min.Data[i], U_Max.Data[i], true, Projected_U[step].Data[i],
                             Dual_U[step].Data[i], primal, dual))
                {
                    return false;
                }
            }
            if (State_Mask != 0U)
            {
                for (int i = 0; i < State_Dim; ++i)
                {
                    if (!Project(state.Data[i], X_Min.Data[i], X_Max.Data[i], State_Bounded(i),
                                 Projected_X[step].Data[i], Dual_X[step].Data[i], primal, dual))
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool State_Bounded(int index) const
    {
        return (State_Mask & (1U << index)) != 0U;
    }

    bool In_State_Box(const State &state) const
    {
        for (int i = 0; i < State_Dim; ++i)
        {
            if (State_Bounded(i) && (state.Data[i] < X_Min.Data[i] || state.Data[i] > X_Max.Data[i]))
            {
                return false;
            }
        }
        return true;
    }

    enum Enum_MPC_Plan
    {
        MPC_PLAN_ADMM,
        MPC_PLAN_SHIFTED,
        MPC_PLAN_FEEDBACK
    };
    enum Enum_MPC_Plan_Result
    {
        MPC_PLAN_VALID,
        MPC_PLAN_INFEASIBLE,
        MPC_PLAN_INVALID
    };

    Enum_MPC_Plan_Result Build_Plan(const Riccati_Cache &cache, Enum_MPC_Plan plan)
    {
        State state = X_Now;
        State center;
        if (plan == MPC_PLAN_FEEDBACK)
        {
            for (int i = 0; i < State_Dim; ++i)
            {
                if (State_Bounded(i) && (X_Min.Data[i] >= 0.0f || X_Max.Data[i] <= 0.0f))
                {
                    center.Data[i] = 0.5f * X_Min.Data[i] + 0.5f * X_Max.Data[i];
                }
            }
        }
        for (int step = 0; step < Horizon; ++step)
        {
            Input control = Projected_U[step];
            if (plan == MPC_PLAN_SHIFTED)
            {
                control = Previous_Plan[step + 1 < Horizon ? step + 1 : step];
            }
            else if (plan == MPC_PLAN_FEEDBACK)
            {
                control = cache.Steady.K * (center - state);
                if (!control.Is_Finite())
                {
                    return MPC_PLAN_INVALID;
                }
                for (int i = 0; i < Control_Dim; ++i)
                {
                    control.Data[i] = Basic_Math_Constrain(control.Data[i], U_Min.Data[i], U_Max.Data[i]);
                }
            }
            const State base = A * state;
            state = base + B * control;
            if (!state.Is_Finite())
            {
                return MPC_PLAN_INVALID;
            }
            if (!In_State_Box(state))
            {
                const Input nominal = control;
                const Enum_MPC_Plan_Result repair = Repair_Input(base, nominal, control, state);
                if (repair != MPC_PLAN_VALID)
                {
                    return repair;
                }
                for (int i = 0; i < Control_Dim; ++i)
                {
                    if (fabsf(control.Data[i] - nominal.Data[i]) > Tolerance)
                    {
                        Converged = false;
                    }
                }
            }
            // Riccati 递推已结束，复用前馈数组暂存经过核查的输入。
            Feedforward[step] = control;
            Prediction[step] = state;
        }
        return MPC_PLAN_VALID;
    }

    bool Certify_Prediction(const Riccati_Cache &cache)
    {
        Enum_MPC_Plan_Result result = Build_Plan(cache, MPC_PLAN_ADMM);
        const bool repaired_plan = result == MPC_PLAN_INFEASIBLE;
        if (result == MPC_PLAN_INFEASIBLE && Previous_Plan_Valid)
        {
            Converged = false;
            result = Build_Plan(cache, MPC_PLAN_SHIFTED);
        }
        if (result == MPC_PLAN_INFEASIBLE)
        {
            Converged = false;
            result = Build_Plan(cache, MPC_PLAN_FEEDBACK);
        }
        if (result != MPC_PLAN_VALID)
        {
            return false;
        }
        if (repaired_plan && !Approach_ADMM_Plan())
        {
            return false;
        }
        for (int step = 0; step < Horizon; ++step)
        {
            Projected_U[step] = Feedforward[step];
            Projected_X[step] = Prediction[step];
            Previous_Plan[step] = Feedforward[step];
        }
        Previous_Plan_Valid = true;
        return true;
    }

    Input Blend_Input(int step, float fraction) const
    {
        Input input = Feedforward[step] + (Projected_U[step] - Feedforward[step]) * fraction;
        for (int i = 0; i < Control_Dim; ++i)
        {
            input.Data[i] = Basic_Math_Constrain(input.Data[i], U_Min.Data[i], U_Max.Data[i]);
        }
        return input;
    }

    // 线性动力学使可行序列到 ADMM 候选的连线只需一次递推即可求可行步长。
    bool Approach_ADMM_Plan()
    {
        float fraction = 1.0f;
        State difference;
        for (int step = 0; step < Horizon; ++step)
        {
            const Input delta = Projected_U[step] - Feedforward[step];
            difference = A * difference + B * delta;
            if (!difference.Is_Finite())
            {
                return false;
            }
            for (int i = 0; i < State_Dim; ++i)
            {
                if (!State_Bounded(i) || difference.Data[i] == 0.0f)
                {
                    continue;
                }
                const float base = Prediction[step].Data[i];
                const float change = difference.Data[i];
                const float proposed = base + fraction * change;
                if (!isfinite(proposed))
                {
                    return false;
                }
                if (proposed < X_Min.Data[i] || proposed > X_Max.Data[i])
                {
                    const float boundary = change > 0.0f ? X_Max.Data[i] : X_Min.Data[i];
                    fraction = fmaxf(0.0f, (boundary - base) / change);
                }
            }
        }

        // 留出一次单精度舍入裕量；若实际复算仍失约，继续使用已核查的候选。
        fraction *= 0.99999f;
        for (int attempt = 0; attempt < 3 && fraction > 0.0f; ++attempt)
        {
            State state = X_Now;
            bool feasible = true;
            for (int step = 0; step < Horizon; ++step)
            {
                state = A * state + B * Blend_Input(step, fraction);
                if (!state.Is_Finite())
                {
                    return false;
                }
                if (!In_State_Box(state))
                {
                    feasible = false;
                    break;
                }
            }
            if (feasible)
            {
                state = X_Now;
                for (int step = 0; step < Horizon; ++step)
                {
                    Feedforward[step] = Blend_Input(step, fraction);
                    state = A * state + B * Feedforward[step];
                    Prediction[step] = state;
                }
                break;
            }
            fraction *= 0.5f;
        }
        return true;
    }

    struct Half_Plane
    {
        float Normal[Control_Dim];
        float Bound;
    };

    // 维度最多为二。逐条边界求与其他半平面的交线段，再在可行线段中选最近输入。
    Enum_MPC_Plan_Result Repair_Input(const State &base, const Input &nominal,
                                      Input &corrected, State &state) const
    {
        for (int attempt = 0; attempt < 2; ++attempt)
        {
            Half_Plane planes[2 * (State_Dim + Control_Dim)];
            int count = 0;
            for (int i = 0; i < Control_Dim; ++i)
            {
                Half_Plane upper = {}, lower = {};
                upper.Normal[i] = 1.0f;
                lower.Normal[i] = -1.0f;
                upper.Bound = U_Max.Data[i];
                lower.Bound = -U_Min.Data[i];
                planes[count++] = upper;
                planes[count++] = lower;
            }
            for (int i = 0; i < State_Dim; ++i)
            {
                if (!State_Bounded(i))
                {
                    continue;
                }
                float lower = X_Min.Data[i] - base.Data[i];
                float upper = X_Max.Data[i] - base.Data[i];
                if (!isfinite(lower) || !isfinite(upper))
                {
                    return MPC_PLAN_INVALID;
                }
                if (attempt == 0)
                {
                    // 仅在修复舍入违例时给递推留数个 ULP，等式约束保留原宽度。
                    const float margin = 8.0f * FLT_EPSILON *
                                         fmaxf(fabsf(base.Data[i]), fmaxf(fabsf(X_Min.Data[i]), fabsf(X_Max.Data[i])));
                    if (lower + margin < upper - margin)
                    {
                        lower += margin;
                        upper -= margin;
                    }
                }
                Half_Plane lo = {}, hi = {};
                for (int j = 0; j < Control_Dim; ++j)
                {
                    lo.Normal[j] = -B[i][j];
                    hi.Normal[j] = B[i][j];
                }
                lo.Bound = -lower;
                hi.Bound = upper;
                planes[count++] = lo;
                planes[count++] = hi;
            }

            float best_distance = INFINITY;
            for (int i = 0; i < count; ++i)
            {
                const Half_Plane &edge = planes[i];
                Input candidate = nominal;
                if (Control_Dim == 1)
                {
                    if (edge.Normal[0] == 0.0f)
                    {
                        continue;
                    }
                    candidate.Data[0] = edge.Bound / edge.Normal[0];
                }
                else
                {
                    const float scale = fmaxf(fabsf(edge.Normal[0]), fabsf(edge.Normal[1]));
                    if (scale == 0.0f)
                    {
                        continue;
                    }
                    const float n0 = edge.Normal[0] / scale;
                    const float n1 = edge.Normal[1] / scale;
                    const float norm = n0 * n0 + n1 * n1;
                    const float correction = (edge.Bound / scale -
                                              n0 * nominal.Data[0] - n1 * nominal.Data[1]) /
                                             norm;
                    candidate.Data[0] += n0 * correction;
                    candidate.Data[1] += n1 * correction;
                    if (!candidate.Is_Finite())
                    {
                        return MPC_PLAN_INVALID;
                    }
                    float t_min = -INFINITY;
                    float t_max = INFINITY;
                    bool feasible_edge = true;
                    for (int j = 0; j < count; ++j)
                    {
                        if (j == i)
                        {
                            continue;
                        }
                        const Half_Plane &limit = planes[j];
                        const float divisor = limit.Normal[1] * n0 - limit.Normal[0] * n1;
                        const float remainder = limit.Bound - limit.Normal[0] * candidate.Data[0] -
                                                limit.Normal[1] * candidate.Data[1];
                        if (!isfinite(divisor) || !isfinite(remainder))
                        {
                            return MPC_PLAN_INVALID;
                        }
                        if (divisor != 0.0f)
                        {
                            const float bound = remainder / divisor;
                            if (!isfinite(bound))
                            {
                                return MPC_PLAN_INVALID;
                            }
                            if (divisor > 0.0f)
                            {
                                t_max = fminf(t_max, bound);
                            }
                            else
                            {
                                t_min = fmaxf(t_min, bound);
                            }
                        }
                        else if (remainder < 0.0f)
                        {
                            feasible_edge = false;
                            break;
                        }
                    }
                    if (!feasible_edge || t_min > t_max)
                    {
                        continue;
                    }
                    const float tangent = Basic_Math_Constrain(0.0f, t_min, t_max);
                    candidate.Data[0] -= n1 * tangent;
                    candidate.Data[1] += n0 * tangent;
                }

                if (!candidate.Is_Finite())
                {
                    return MPC_PLAN_INVALID;
                }
                float distance = 0.0f;
                for (int j = 0; j < Control_Dim; ++j)
                {
                    candidate.Data[j] = Basic_Math_Constrain(candidate.Data[j], U_Min.Data[j], U_Max.Data[j]);
                    distance = hypotf(distance, candidate.Data[j] - nominal.Data[j]);
                }
                const State next = base + B * candidate;
                if (!isfinite(distance) || !next.Is_Finite())
                {
                    return MPC_PLAN_INVALID;
                }
                if (distance < best_distance && In_State_Box(next))
                {
                    corrected = candidate;
                    state = next;
                    best_distance = distance;
                }
            }
            if (isfinite(best_distance))
            {
                return MPC_PLAN_VALID;
            }
        }
        return MPC_PLAN_INFEASIBLE;
    }

    void Reset_Iterates()
    {
        Input initial;
        State initial_state;
        for (int i = 0; i < Control_Dim; ++i)
        {
            initial.Data[i] = Basic_Math_Constrain(0.0f, U_Min.Data[i], U_Max.Data[i]);
        }
        for (int i = 0; i < State_Dim; ++i)
        {
            if (State_Bounded(i))
            {
                initial_state.Data[i] = Basic_Math_Constrain(0.0f, X_Min.Data[i], X_Max.Data[i]);
            }
        }
        for (int step = 0; step < Horizon; ++step)
        {
            Projected_U[step] = initial;
            Dual_U[step] = Input();
            Projected_X[step] = initial_state;
            Dual_X[step] = State();
        }
        Seeded = true;
    }

    void Invalidate_Result()
    {
        Converged = false;
        Prediction_Valid = false;
        for (int i = 0; i < Control_Dim; ++i)
        {
            Out.Data[i] = Basic_Math_Constrain(0.0f, U_Min.Data[i], U_Max.Data[i]);
        }
    }

    bool Fail()
    {
        Invalidate_Result();
        Seeded = false;
        Previous_Plan_Valid = false;
        return false;
    }

    Riccati_Cache Cache[2];
    State_Matrix A;
    Input_Model B;
    State_Matrix Q;
    State_Matrix P;
    State X_Now;
    State X_Ref;
    State X_Min;
    State X_Max;
    State Projected_X[Horizon];
    State Dual_X[Horizon];
    State Prediction[Horizon];
    Input Projected_U[Horizon];
    Input Dual_U[Horizon];
    Input Feedforward[Horizon];
    Input Previous_Plan[Horizon];
    Input U_Min;
    Input U_Max;
    Input Out;
    float Rho = 0.0f;
    float Tolerance = 0.0f;
    uint32_t State_Mask = 0U;
    uint32_t Iteration_Limit = 0U;
    uint32_t Iterations = 0U;
    bool Initialized = false;
    bool Warm_Start = true;
    bool Converged = false;
    bool Seeded = false;
    bool Prediction_Valid = false;
    bool Previous_Plan_Valid = false;
};

#endif
