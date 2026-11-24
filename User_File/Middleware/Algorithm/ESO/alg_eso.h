/**
 * @file alg_eso.h
 * @brief 独立的一阶和二阶线性扩张状态观测器。
 */

#ifndef ALG_ESO_H
#define ALG_ESO_H

/**
 * @brief 一阶模型 y' = b0*u + f 的线性扩张状态观测器。
 * @details 速度环中 y、z1 的单位可为 rad/s，z2 是总扰动加速度（rad/s²）。
 * b0 是带符号的名义输入增益，单位为加速度每控制输入单位。omega_o 为 rad/s，
 * d_t 为 s。Get_Compensation() 返回 z2/b0，与实际控制输入 u 同单位。
 * 连续配置为 beta1 = 2*omega_o、beta2 = omega_o²。零阶保持预测加当前测量校正，
 * 两个观测误差极点均为 exp(-omega_o*d_t)。Init 校验正的有限带宽步长和离散增益。
 * 调参顺序为先辨识 b0 的符号和量级，低带宽起步，逐步加快观测，再按测量噪声回退。
 * Init 成功后以实测值 Reset，再按固定周期输入当前 y[k] 和上一周期实际施加的
 * 限幅后输入 u[k-1]。单一上下文使用。非有限的 Reset 参数或周期估计保留上次有效状态。
 * @code
 * #include "alg_pid.h"
 * Class_ESO_First_Order eso;
 * Class_PID pid;
 * const float speed = 0.1f;
 * float last_u = 0.0f;
 * if (eso.Init(2.0f, 60.0f))
 * {
 *     eso.Reset(speed);
 *     pid.Init(1.0f, 2.0f, 0.0f, 0.0f, 1.0f, 3.0f, 0.001f);
 *
 *     // 每 1 ms 用最新测速执行以下步骤。
 *     eso.Set_Now(speed);
 *     eso.Set_Input(last_u);
 *     eso.TIM_Calculate_PeriodElapsedCallback();
 *     pid.Set_Target(0.5f);
 *     pid.Set_Now(eso.Get_State());
 *     pid.TIM_Calculate_PeriodElapsedCallback();
 *     last_u = Basic_Math_Constrain(pid.Get_Out() - eso.Get_Compensation(), -3.0f, 3.0f);
 *     // 发送此命令。若下游再次限幅，下一拍应使用实际施加的输入。
 *     (void)last_u;
 * }
 * @endcode
 */
class Class_ESO_First_Order
{
  public:
    bool Init(float b0, float omega_o, float d_t = 0.001f);
    void Reset(float initial_y = 0.0f);
    void Set_Now(float now_y)
    {
        Now = now_y;
    }
    void Set_Input(float last_u)
    {
        Input = last_u;
    }
    void TIM_Calculate_PeriodElapsedCallback();

    float Get_State() const
    {
        return Z1;
    }
    float Get_Disturbance() const
    {
        return Z2;
    }
    float Get_Compensation() const
    {
        return Compensation;
    }

  private:
    float B0 = 0.0f;
    float B0_Inverse = 0.0f;
    float D_T = 0.0f;
    float L1 = 0.0f;
    float L2 = 0.0f;
    float Now = 0.0f;
    float Input = 0.0f;
    float Z1 = 0.0f;
    float Z2 = 0.0f;
    float Compensation = 0.0f;
};

/**
 * @brief 二阶模型 y'' = b0*u + f 的线性扩张状态观测器。
 * @details y、z1 为位置（如 rad），z2 为速度（rad/s），z3 为总扰动加速度（rad/s²）。
 * b0 为加速度每控制输入单位的带符号名义增益，omega_o 为 rad/s，d_t 为 s。
 * Get_Compensation() 返回 z3/b0，上层从控制量减去它。连续配置为 beta1 = 3*omega_o、
 * beta2 = 3*omega_o²、beta3 = omega_o³。当前观测器的三个离散极点均为 exp(-omega_o*d_t)。
 * Init 成功后先用实测位置和速度对齐，再按一阶 ESO 的步骤由低到高调带宽并检查噪声。
 * 单一上下文使用。周期调用前设置当前测量 y[k] 和上一拍实际施加的输入 u[k-1]。
 * 需要检查数值状态时可改用 Update 和 Try_Reset，异常时返回 false 并保留有效估计。
 * @code
 * Class_ESO_Second_Order eso;
 * const float angle = 0.1f;
 * const float last_u = 0.0f;
 * if (eso.Init(2.0f, 80.0f))
 * {
 *     eso.Reset(angle);
 *     // 每个周期使用新的测量值和实际输入。
 *     eso.Set_Now(angle);
 *     eso.Set_Input(last_u);
 *     eso.TIM_Calculate_PeriodElapsedCallback();
 *     const float position = eso.Get_Position();
 *     const float velocity = eso.Get_Velocity();
 *     (void)position;
 *     (void)velocity;
 * }
 * @endcode
 */
class Class_ESO_Second_Order
{
  public:
    bool Init(float b0, float omega_o, float d_t = 0.001f);
    void Reset(float initial_y = 0.0f, float initial_dy = 0.0f);
    void Set_Now(float now_y)
    {
        Now = now_y;
    }
    void Set_Input(float last_u)
    {
        Input = last_u;
    }
    void TIM_Calculate_PeriodElapsedCallback();

    bool Try_Reset(float initial_y, float initial_dy);
    bool Update(float measurement, float applied_input);

    float Get_Position() const
    {
        return Z1;
    }
    float Get_Velocity() const
    {
        return Z2;
    }
    float Get_Disturbance() const
    {
        return Z3;
    }
    float Get_Compensation() const
    {
        return Compensation;
    }

  private:
    float B0 = 0.0f;
    float B0_Inverse = 0.0f;
    float D_T = 0.0f;
    float L1 = 0.0f;
    float L2 = 0.0f;
    float L3 = 0.0f;
    float Now = 0.0f;
    float Input = 0.0f;
    float Z1 = 0.0f;
    float Z2 = 0.0f;
    float Z3 = 0.0f;
    float Compensation = 0.0f;
};

#endif
