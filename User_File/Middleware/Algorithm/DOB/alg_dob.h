/**
 * @file alg_dob.h
 * @author zzm
 * @brief 一阶名义模型的Q滤波扰动观测器
 */

#ifndef __ALG_DOB_H
#define __ALG_DOB_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 一阶名义模型DOB
 * @details 模型为 Time_Constant * dy/dt + y = Gain * (u + d)。
 * 使用零阶保持离散模型和两个相同实极点的Q滤波器。
 * y为测量输出, u为实际控制输入, d为输入端等效扰动。
 * 输出为上一采样区间等效扰动的滤波估计, 与u同单位, 可包含模型误差。
 * Init成功后使用, 调用方保证固定采样周期, y[k]与上一周期实际输入u[k-1]对齐。
 */
class Class_DOB_First_Order
{
public:
    bool Init(float __Gain, float __Time_Constant, float __Filter_Frequency, float __D_T = 0.001f);

    void Reset(float __Now = 0.0f);

    inline float Get_Out() const;

    inline void Set_Now(float __Now);

    inline void Set_Input(float __Input);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    float Gain_Inverse = 0.0f;      ///< 名义模型静态增益的倒数
    float Filter_Alpha = 0.0f;      ///< Q滤波器每个一阶节的更新系数
    float Difference_Gain = 0.0f;   ///< 逆模型差分项与第一级低通合并后的增益

    float Now = 0.0f;               ///< 当前测量y[k]
    float Input = 0.0f;             ///< 上一采样区间实际施加的u[k-1]
    float Pre_Now = 0.0f;           ///< 上次计算使用的测量y[k-1]
    float Filter_Out = 0.0f;        ///< 第一级Q滤波输出
    float Out = 0.0f;               ///< 第二级Q滤波输出, 即扰动估计
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取最近一次计算的等效扰动估计
 * @return 与模型输入同单位的扰动估计值
 */
inline float Class_DOB_First_Order::Get_Out() const
{
    return (Out);
}

/**
 * @brief 设置本次观测使用的测量值
 * @param __Now 当前采样时刻的测量y[k]
 */
inline void Class_DOB_First_Order::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @brief 设置产生当前测量的上一周期输入
 * @param __Input 上一采样周期限幅后实际施加的输入, 与名义模型输入同单位
 */
inline void Class_DOB_First_Order::Set_Input(float __Input)
{
    Input = __Input;
}

#endif
