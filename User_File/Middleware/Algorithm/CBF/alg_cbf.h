/**
 * @file alg_cbf.h
 * @brief 定长控制屏障函数安全滤波器，包含标量解析投影与多约束软松弛 QP。
 */

#ifndef ALG_CBF_H
#define ALG_CBF_H

#include "alg_matrix.h"

/**
 * @details x、v 的单位分别为 U、U/s，alpha、alpha1、alpha2 的单位为 1/s。
 * 一阶 drift_f0、b0*u 的单位为 U/s，二阶为 U/s²。u 必须与执行器硬限幅同量纲。
 * 先核对模型增益方向和硬限幅，再从较小的正 alpha 开始调节接近边界的速度。
 * 二阶在连续模型下还要求初始状态满足两个高阶屏障的速度条件；控制周期、模型误差
 * 和可用制动能力都会影响安全裕量。约束无交集时输出最大可用纠偏量并返回 false。
 * @code
 * Class_CBF_Scalar cbf;
 * if (cbf.Init_Second_Order(-0.523599f, 0.523599f, 6.0f, 6.0f, 1.0f, -8.0f, 8.0f))
 * {
 *     const bool feasible = cbf.Filter_Second_Order(2.0f, 0.3f, 0.4f);
 *     const float limited_torque = cbf.Get_Out();
 *     (void)feasible;
 *     (void)limited_torque;
 * }
 * @endcode
 */
class Class_CBF_Scalar
{
  public:
    bool Init_First_Order(float x_min, float x_max, float alpha, float b0,
                          float u_min, float u_max)
    {
        return Init(CBF_SCALAR_FIRST_ORDER, x_min, x_max, 0.0f, alpha, b0, u_min, u_max);
    }

    bool Init_Second_Order(float x_min, float x_max, float alpha1, float alpha2,
                           float b0, float u_min, float u_max)
    {
        if (!isfinite(alpha1) || !isfinite(alpha2) || alpha1 <= 0.0f || alpha2 <= 0.0f)
        {
            return false;
        }
        return Init(CBF_SCALAR_SECOND_ORDER, x_min, x_max, alpha1 + alpha2,
                    alpha1 * alpha2, b0, u_min, u_max);
    }

    bool Set_State_Bounds(float x_min, float x_max)
    {
        if (!Valid_State_Bounds(x_min, x_max))
        {
            return false;
        }
        X_Min = x_min;
        X_Max = x_max;
        return true;
    }

    bool Set_Input_Bounds(float u_min, float u_max)
    {
        if (!Valid_Input_Bounds(u_min, u_max))
        {
            return false;
        }
        U_Min = u_min;
        U_Max = u_max;
        Out = Basic_Math_Constrain(Out, U_Min, U_Max);
        return true;
    }

    bool Filter_First_Order(float u_nom, float x, float drift_f0 = 0.0f)
    {
        return Filter(CBF_SCALAR_FIRST_ORDER, u_nom, x, 0.0f, drift_f0);
    }

    bool Filter_Second_Order(float u_nom, float x, float v, float drift_f0 = 0.0f)
    {
        return Filter(CBF_SCALAR_SECOND_ORDER, u_nom, x, v, drift_f0);
    }

    float Get_Out() const
    {
        return Out;
    }
    bool Is_Active() const
    {
        return Active;
    }
    bool Is_Initialized() const
    {
        return Mode != CBF_SCALAR_UNINITIALIZED;
    }

  private:
    enum Enum_CBF_Scalar_Mode
    {
        CBF_SCALAR_UNINITIALIZED,
        CBF_SCALAR_FIRST_ORDER,
        CBF_SCALAR_SECOND_ORDER
    };

    static bool Valid_State_Bounds(float lower, float upper)
    {
        return isfinite(lower) && isfinite(upper) && lower < upper;
    }

    static bool Valid_Input_Bounds(float lower, float upper)
    {
        return isfinite(lower) && isfinite(upper) && lower <= upper;
    }

    bool Init(Enum_CBF_Scalar_Mode mode, float x_min, float x_max, float rate,
              float stiffness, float b0, float u_min, float u_max)
    {
        if (!Valid_State_Bounds(x_min, x_max) || !Valid_Input_Bounds(u_min, u_max) ||
            !isfinite(rate) || !isfinite(stiffness) || stiffness <= 0.0f ||
            !isfinite(b0) || b0 == 0.0f)
        {
            return false;
        }
        X_Min = x_min;
        X_Max = x_max;
        U_Min = u_min;
        U_Max = u_max;
        Rate = rate;
        Stiffness = stiffness;
        Input_Gain = b0;
        Mode = mode;
        Out = Basic_Math_Constrain(0.0f, U_Min, U_Max);
        Active = false;
        return true;
    }

    bool Filter(Enum_CBF_Scalar_Mode mode, float nominal, float x, float v, float drift)
    {
        if (Mode != mode || !isfinite(nominal) || !isfinite(x) || !isfinite(v) || !isfinite(drift))
        {
            Out = Basic_Math_Constrain(0.0f, U_Min, U_Max);
            Active = true;
            return false;
        }

        const float offset = -Rate * v - drift;
        const float from_lower = (Stiffness * (X_Min - x) + offset) / Input_Gain;
        const float from_upper = (Stiffness * (X_Max - x) + offset) / Input_Gain;
        if (isnan(from_lower) || isnan(from_upper))
        {
            Out = Basic_Math_Constrain(0.0f, U_Min, U_Max);
            Active = true;
            return false;
        }

        const float safe_lower = Input_Gain > 0.0f ? from_lower : from_upper;
        const float safe_upper = Input_Gain > 0.0f ? from_upper : from_lower;
        const float lower = fmaxf(safe_lower, U_Min);
        const float upper = fminf(safe_upper, U_Max);
        if (lower > upper)
        {
            Out = safe_upper < U_Min ? U_Min : U_Max;
            Active = true;
            return false;
        }
        Out = Basic_Math_Constrain(nominal, lower, upper);
        Active = Out != nominal;
        return true;
    }

    Enum_CBF_Scalar_Mode Mode = CBF_SCALAR_UNINITIALIZED;
    float X_Min = 0.0f;
    float X_Max = 0.0f;
    float U_Min = 0.0f;
    float U_Max = 0.0f;
    float Rate = 0.0f;
    float Stiffness = 0.0f;
    float Input_Gain = 0.0f;
    float Out = 0.0f;
    bool Active = false;
};

/**
 * @details 每行表示 aᵀu ≥ b。建议先归一化输入和约束行，再从 slack_weight=1000
 * 开始调节；较大的权重减小软约束违例，同时可能增加达到容差所需的迭代数。
 * 对偶加速投影梯度使用 u=clip(u_nom+Aᵀλ)、δ=λ/slack_weight。收尾用小维度
 * Newton 步修正活动约束，尤其适合输入已贴住限幅的情形。硬输入盒始终生效。
 * Solve 返回 true 表示得到有限的盒内输出；用 Is_Converged() 判断求解精度。
 * 软约束即使收敛也可能有违例；安全裕量由应用层决定。
 * @code
 * Class_CBF_QP<2, 4> cbf;
 * const float lo[] = {-1.0f, -1.0f}, hi[] = {1.0f, 1.0f};
 * const float normal[] = {1.0f, 0.0f}, command[] = {-0.4f, 0.3f};
 * if (cbf.Init(Class_Matrix_f32<2, 1>(lo), Class_Matrix_f32<2, 1>(hi)) &&
 *     cbf.Add_Constraint(Class_Matrix_f32<2, 1>(normal), 0.0f))
 * {
 *     cbf.Solve(Class_Matrix_f32<2, 1>(command));
 * }
 * @endcode
 */
template <int Control_Dim, int Max_Constraints>
class Class_CBF_QP
{
    static_assert(Control_Dim >= 1 && Control_Dim <= 4, "CBF QP supports 1 to 4 controls");
    static_assert(Max_Constraints >= 1 && Max_Constraints <= 8, "CBF QP supports 1 to 8 constraints");

  public:
    using Vector = Class_Matrix_f32<Control_Dim, 1>;
    static constexpr uint32_t MAX_ITERATIONS = 64U;

    bool Init(const Vector &u_min, const Vector &u_max, float slack_weight = 1000.0f,
              uint32_t max_iterations = 24U, float tolerance = 1e-3f)
    {
        if (!u_min.Is_Finite() || !u_max.Is_Finite() || !isfinite(slack_weight) ||
            slack_weight <= 0.0f || !isfinite(tolerance) || tolerance <= 0.0f ||
            max_iterations == 0U || max_iterations > MAX_ITERATIONS)
        {
            return Fail();
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            if (u_min.Data[i] > u_max.Data[i])
            {
                return Fail();
            }
        }
        const float inverse_weight = 1.0f / slack_weight;
        if (!isfinite(inverse_weight) || inverse_weight <= 0.0f)
        {
            return Fail();
        }
        U_Min = u_min;
        U_Max = u_max;
        Inverse_Weight = inverse_weight;
        Iteration_Limit = max_iterations;
        Tolerance = tolerance;
        Initialized = true;
        Clear_Constraints();
        Set_Zero_Out();
        Active = false;
        return true;
    }

    void Clear_Constraints()
    {
        Count = 0;
        Converged = false;
    }

    bool Add_Constraint(const Vector &a, float b)
    {
        if (!Initialized || Count == Max_Constraints || !a.Is_Finite() || !isfinite(b))
        {
            return Fail();
        }
        Rows[Count].Normal = a;
        Rows[Count].Bound = b;
        ++Count;
        Converged = false;
        return true;
    }

    bool Solve(const Vector &u_nom)
    {
        if (!Initialized || !u_nom.Is_Finite())
        {
            return Fail();
        }

        Converged = false;
        float previous[Max_Constraints] = {};
        float dual[Max_Constraints] = {};
        float accelerated[Max_Constraints] = {};
        float residual[Max_Constraints];
        Vector candidate;
        if (!Evaluate(u_nom, dual, candidate, residual))
        {
            return Fail();
        }
        if (Residual(dual, residual) <= Tolerance)
        {
            Out = candidate;
            Converged = true;
        }
        else
        {
            // Gershgorin 上界保证步长不超过对偶梯度的 Lipschitz 倒数。
            float lipschitz = 0.0f;
            for (int i = 0; i < Count; ++i)
            {
                float row_sum = Inverse_Weight;
                for (int j = 0; j < Count; ++j)
                {
                    row_sum += fabsf(Dot(Rows[i].Normal, Rows[j].Normal));
                }
                if (!isfinite(row_sum))
                {
                    return Fail();
                }
                lipschitz = fmaxf(lipschitz, row_sum);
            }
            const float step = 1.0f / lipschitz;
            if (!isfinite(step) || step <= 0.0f)
            {
                return Fail();
            }

            const uint32_t polish_limit = Iteration_Limit / 4U < 4U ? Iteration_Limit / 4U : 4U;
            for (uint32_t iteration = 0; iteration < Iteration_Limit - polish_limit; ++iteration)
            {
                if (iteration != 0U && !Evaluate(u_nom, accelerated, candidate, residual))
                {
                    return Fail();
                }
                const float momentum = (float) iteration / ((float) iteration + 3.0f);
                for (int i = 0; i < Count; ++i)
                {
                    const float next = accelerated[i] + step * residual[i];
                    if (!isfinite(next))
                    {
                        return Fail();
                    }
                    dual[i] = fmaxf(0.0f, next);
                    accelerated[i] = dual[i] + momentum * (dual[i] - previous[i]);
                    previous[i] = dual[i];
                }
                if (!Evaluate(u_nom, dual, candidate, residual))
                {
                    return Fail();
                }
                if (Residual(dual, residual) <= Tolerance)
                {
                    Converged = true;
                    break;
                }
            }
            if (!Converged && !Polish(u_nom, candidate, polish_limit))
            {
                return Fail();
            }
            Out = candidate;
        }

        Active = false;
        for (int i = 0; i < Control_Dim; ++i)
        {
            Active = Active || Out.Data[i] != u_nom.Data[i];
        }
        return true;
    }

    const Vector &Get_Out() const
    {
        return Out;
    }
    bool Is_Active() const
    {
        return Active;
    }
    bool Is_Converged() const
    {
        return Converged;
    }

  private:
    struct Constraint
    {
        Vector Normal;
        float Bound;
    };

    static float Dot(const Vector &left, const Vector &right)
    {
        float value = 0.0f;
        for (int i = 0; i < Control_Dim; ++i)
        {
            value += left.Data[i] * right.Data[i];
        }
        return value;
    }

    bool Evaluate(const Vector &nominal, const float *dual, Vector &candidate, float *residual) const
    {
        candidate = nominal;
        for (int i = 0; i < Count; ++i)
        {
            for (int j = 0; j < Control_Dim; ++j)
            {
                candidate.Data[j] += Rows[i].Normal.Data[j] * dual[i];
            }
        }
        if (!candidate.Is_Finite())
        {
            return false;
        }
        for (int i = 0; i < Control_Dim; ++i)
        {
            candidate.Data[i] = Basic_Math_Constrain(candidate.Data[i], U_Min.Data[i], U_Max.Data[i]);
        }
        for (int i = 0; i < Count; ++i)
        {
            residual[i] = Rows[i].Bound - Dot(Rows[i].Normal, candidate) - dual[i] * Inverse_Weight;
            if (!isfinite(residual[i]))
            {
                return false;
            }
        }
        return true;
    }

    float Residual(const float *dual, const float *residual) const
    {
        float largest = 0.0f;
        for (int i = 0; i < Count; ++i)
        {
            const float value = dual[i] > 0.0f ? fabsf(residual[i]) : residual[i];
            largest = fmaxf(largest, value);
        }
        return largest;
    }

    float Merit(const Vector &candidate, const Vector &nominal, float *violation) const
    {
        const Vector change = candidate - nominal;
        float merit = Inverse_Weight * Dot(change, change);
        for (int i = 0; i < Count; ++i)
        {
            const float gap = Rows[i].Bound - Dot(Rows[i].Normal, candidate);
            if (!isfinite(gap))
            {
                return INFINITY;
            }
            violation[i] = fmaxf(0.0f, gap);
            merit += violation[i] * violation[i];
        }
        return merit;
    }

    static bool Solve_Direction(Class_Matrix_f32<Control_Dim, Control_Dim> &factor, Vector &direction)
    {
        for (int i = 0; i < Control_Dim; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                float value = factor[i][j];
                for (int k = 0; k < j; ++k)
                {
                    value -= factor[i][k] * factor[j][k];
                }
                if (i == j && !(value > 0.0f))
                {
                    return false;
                }
                factor[i][j] = i == j ? sqrtf(value) : value / factor[j][j];
            }
            for (int j = 0; j < i; ++j)
            {
                direction.Data[i] -= factor[i][j] * direction.Data[j];
            }
            direction.Data[i] /= factor[i][i];
        }
        for (int i = Control_Dim - 1; i >= 0; --i)
        {
            for (int j = i + 1; j < Control_Dim; ++j)
            {
                direction.Data[i] -= factor[j][i] * direction.Data[j];
            }
            direction.Data[i] /= factor[i][i];
        }
        return direction.Is_Finite();
    }

    bool Polish(const Vector &nominal, Vector &candidate, uint32_t limit)
    {
        for (uint32_t iteration = 0; iteration <= limit; ++iteration)
        {
            float violation[Max_Constraints];
            const float merit = Merit(candidate, nominal, violation);
            if (!isfinite(merit))
            {
                return false;
            }
            Vector direction = (nominal - candidate) * Inverse_Weight;
            for (int i = 0; i < Count; ++i)
            {
                direction = direction + Rows[i].Normal * violation[i];
            }
            if (!direction.Is_Finite())
            {
                return false;
            }
            bool locked[Control_Dim];
            float projected_residual = 0.0f;
            for (int i = 0; i < Control_Dim; ++i)
            {
                const float at = candidate.Data[i];
                const float descent = direction.Data[i];
                locked[i] = (at <= U_Min.Data[i] && descent <= 0.0f) ||
                            (at >= U_Max.Data[i] && descent >= 0.0f);
                projected_residual = fmaxf(projected_residual, locked[i] ? 0.0f : fabsf(descent));
            }
            if (projected_residual <= Tolerance * Inverse_Weight)
            {
                Converged = true;
                return true;
            }
            if (iteration == limit)
            {
                break;
            }

            Class_Matrix_f32<Control_Dim, Control_Dim> factor;
            for (int i = 0; i < Control_Dim; ++i)
            {
                if (locked[i])
                {
                    factor[i][i] = 1.0f;
                    direction.Data[i] = 0.0f;
                    continue;
                }
                factor[i][i] = Inverse_Weight;
                for (int j = 0; j <= i; ++j)
                {
                    if (!locked[j])
                    {
                        for (int k = 0; k < Count; ++k)
                        {
                            if (violation[k] > 0.0f)
                            {
                                factor[i][j] += Rows[k].Normal.Data[i] * Rows[k].Normal.Data[j];
                            }
                        }
                    }
                }
            }
            if (!factor.Is_Finite())
            {
                return false;
            }
            if (!Solve_Direction(factor, direction))
            {
                break;
            }

            for (float step = 1.0f; step >= 0.0078125f; step *= 0.5f)
            {
                Vector trial = candidate + direction * step;
                if (!trial.Is_Finite())
                {
                    return false;
                }
                for (int i = 0; i < Control_Dim; ++i)
                {
                    trial.Data[i] = Basic_Math_Constrain(trial.Data[i], U_Min.Data[i], U_Max.Data[i]);
                }
                const float trial_merit = Merit(trial, nominal, violation);
                if (!isfinite(trial_merit))
                {
                    return false;
                }
                if (trial_merit <= merit)
                {
                    candidate = trial;
                    break;
                }
            }
        }
        return true;
    }

    void Set_Zero_Out()
    {
        for (int i = 0; i < Control_Dim; ++i)
        {
            Out.Data[i] = Basic_Math_Constrain(0.0f, U_Min.Data[i], U_Max.Data[i]);
        }
    }

    bool Fail()
    {
        Set_Zero_Out();
        Active = true;
        Converged = false;
        return false;
    }

    Constraint Rows[Max_Constraints];
    Vector U_Min;
    Vector U_Max;
    Vector Out;
    float Inverse_Weight = 0.0f;
    float Tolerance = 0.0f;
    uint32_t Iteration_Limit = 0U;
    int Count = 0;
    bool Initialized = false;
    bool Active = false;
    bool Converged = false;
};

#endif
