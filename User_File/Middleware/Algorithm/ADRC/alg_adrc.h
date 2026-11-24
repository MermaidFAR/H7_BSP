/**
 * @file alg_adrc.h
 * @brief 二阶 LADRC、线性跟踪微分器和旧 LESO 接口。
 */

#ifndef ALG_ADRC_H
#define ALG_ADRC_H

#include "alg_eso.h"

/** @brief 保留原 LESO 接口，观测状态和计算全部由 Class_ESO_Second_Order 持有。 */
class Class_LESO_Second_Order
{
  public:
    bool Init(float b0, float omega_o, float d_t);
    bool Reset(float position = 0.0f, float velocity = 0.0f);
    bool Update(float measurement, float applied_input);

    float Get_Z1() const
    {
        return ESO.Get_Position();
    }
    float Get_Z2() const
    {
        return ESO.Get_Velocity();
    }
    float Get_Z3() const
    {
        return ESO.Get_Disturbance();
    }
    float Get_Compensation() const
    {
        return ESO.Get_Compensation();
    }

  private:
    Class_ESO_Second_Order ESO;
};

/**
 * @brief 临界阻尼线性跟踪微分器，按零阶保持精确离散。
 * @details omega 单位 rad/s，d_t 单位 s。位置与 target 同单位，速度为该单位每秒。
 * 降低 omega 可平滑目标和目标速度，代价是更大的跟踪滞后。Init 后使用 Reset
 * 对齐初始目标。Update 失败时保留上一组有效输出。
 * @code
 * Class_TD td;
 * if (td.Init(20.0f, 0.001f) && td.Reset(0.0f) && td.Update(0.5f))
 * {
 *     const float position_ref = td.Get_Position();
 *     const float velocity_ref = td.Get_Velocity();
 *     (void)position_ref;
 *     (void)velocity_ref;
 * }
 * @endcode
 */
class Class_TD
{
  public:
    bool Init(float omega, float d_t);
    bool Reset(float position = 0.0f);
    bool Update(float target);

    float Get_Position() const
    {
        return Position;
    }
    float Get_Velocity() const
    {
        return Velocity;
    }

  private:
    float Position_Gain = 0.0f;
    float Position_From_Velocity = 0.0f;
    float Velocity_Gain = 0.0f;
    float Velocity_Retention = 0.0f;
    float Position = 0.0f;
    float Velocity = 0.0f;
};

/**
 * @brief LESO 和带宽参数化反馈组成的二阶 LADRC。
 * @details b0 单位同 LESO，omega_o、omega_c 为 rad/s，d_t 为 s，out_max 与 u 同单位。
 * 辨识 b0，先以较低 omega_c 闭环试调，再以约 3 倍 omega_c 为 omega_o 起点，
 * 根据噪声、滞后和饱和情况调整。离散反馈在理想双积分模型上的两个闭环极点为
 * exp(-omega_c * d_t)。Set_Ref 可传入 TD/轨迹模块给出的同单位位置和速度。
 * Init 成功后使用。Init 和 Reset 后 Get_Out() 为零。单参数版本用上一周期本类输出进行观测，
 * 当输出经外部限幅或覆盖时，使用双参数版本传入上一周期实际施加的控制量。
 * 数值异常时返回 false 且清零本周期输出。单一调用上下文使用。
 * @see https://engagedscholarship.csuohio.edu/enece_facpub/528/
 * @see https://doi.org/10.1007/978-3-031-72687-3
 * @code
 * Class_LADRC_Second_Order ladrc;
 * Class_TD td;
 * if (ladrc.Init(2.0f, 60.0f, 20.0f, 3.0f, 0.001f) &&
 *     ladrc.Reset(0.1f) && td.Init(15.0f, 0.001f) && td.Reset(0.1f))
 * {
 *     // 每 1 ms 执行一次。这里以首个采样为例。
 *     if (td.Update(0.2f))
 *     {
 *         ladrc.Set_Ref(td.Get_Position(), td.Get_Velocity());
 *         const bool valid = ladrc.TIM_Calculate_PeriodElapsedCallback(0.1f);
 *         const float command = ladrc.Get_Out();
 *         (void)valid;
 *         (void)command;
 *     }
 * }
 * @endcode
 */
class Class_LADRC_Second_Order
{
  public:
    bool Init(float b0, float omega_o, float omega_c, float out_max, float d_t);
    bool Reset(float position = 0.0f, float velocity = 0.0f);
    void Set_Ref(float position, float velocity = 0.0f);

    bool TIM_Calculate_PeriodElapsedCallback(float measurement);
    bool TIM_Calculate_PeriodElapsedCallback(float measurement, float applied_input);

    float Get_Out() const
    {
        return Out;
    }
    const Class_LESO_Second_Order &Get_Observer() const
    {
        return Observer;
    }

  private:
    Class_LESO_Second_Order Observer;
    float K_Position = 0.0f;
    float K_Velocity = 0.0f;
    float Out_Max = 0.0f;
    float Ref_Position = 0.0f;
    float Ref_Velocity = 0.0f;
    float Out = 0.0f;
};

#endif
