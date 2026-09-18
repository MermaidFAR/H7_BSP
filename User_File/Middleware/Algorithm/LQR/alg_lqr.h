/**
 * @file alg_lqr.h
 * @author xiao
 * @brief LQR 线性二次型最优控制算法
 * @version 0.1
 * @date 2026-09-17 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef ALG_LQR_H
#define ALG_LQR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "alg_matrix.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, LQR 算法
 *
 * @note  增益矩阵 K 由离线设计给出（MATLAB lqr / scipy），本类只做在线求值
 *        u = -K * (x - x_ref)，单输入形式。
 * @note  状态量与增益的单位必须与离线设计保持一致，推荐统一 SI 单位
 *        （角度 rad、角速度 rad/s、位移 m、速度 m/s、输出 A）。
 * @note  模板参数 State_Num 为状态量个数：平衡车取 4（θ, θ̇, p, ṗ）。
 */
template<int State_Num>
class Class_LQR
{
public:
    void Init(const Class_Matrix_f32<1, State_Num> &__K, const float &__Out_Max = 0.0f);

    inline void Set_K(const Class_Matrix_f32<1, State_Num> &__K);

    inline void Set_Now(const Class_Matrix_f32<State_Num, 1> &__Now);

    inline void Set_Target(const Class_Matrix_f32<State_Num, 1> &__Target);

    inline void Set_Out_Max(const float &__Out_Max);

    inline const Class_Matrix_f32<1, State_Num> &Get_K() const;

    inline const Class_Matrix_f32<State_Num, 1> &Get_Now() const;

    inline const Class_Matrix_f32<State_Num, 1> &Get_Error() const;

    inline float Get_Out() const;

    inline bool Get_Invalid_Flag() const;

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 增益矩阵，单位 [输出单位/状态单位]
    Class_Matrix_f32<1, State_Num> K;
    // 输出限幅，0 表示不限幅
    float Out_Max = 0.0f;

    // 读变量

    // 当前状态量
    Class_Matrix_f32<State_Num, 1> Now;
    // 目标状态量
    Class_Matrix_f32<State_Num, 1> Target;

    // 写变量

    // 读写变量

    // 状态误差
    Class_Matrix_f32<State_Num, 1> Error;
    // 控制量
    float Out = 0.0f;
    // 上次解算是否遇到非法输入
    bool Invalid_Flag = false;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief LQR 初始化
 *
 * @param __K       增益矩阵，单位 [输出单位/状态单位]
 * @param __Out_Max 输出限幅，0 表示不限幅
 *
 * @note 调用后对象立即处于可用状态，Now/Target/Error/Out 全部清零。
 */
template<int State_Num>
void Class_LQR<State_Num>::Init(const Class_Matrix_f32<1, State_Num> &__K, const float &__Out_Max)
{
    K = __K;
    Out_Max = __Out_Max;

    Now = Class_Matrix_f32<State_Num, 1>();
    Target = Class_Matrix_f32<State_Num, 1>();
    Error = Class_Matrix_f32<State_Num, 1>();

    Out = 0.0f;
    Invalid_Flag = false;
}

/**
 * @brief 设置增益矩阵
 *
 * @param __K 增益矩阵
 */
template<int State_Num>
inline void Class_LQR<State_Num>::Set_K(const Class_Matrix_f32<1, State_Num> &__K)
{
    K = __K;
}

/**
 * @brief 设置当前状态量
 *
 * @param __Now 当前状态量，单位需与离线设计一致（SI）
 */
template<int State_Num>
inline void Class_LQR<State_Num>::Set_Now(const Class_Matrix_f32<State_Num, 1> &__Now)
{
    Now = __Now;
}

/**
 * @brief 设置目标状态量
 *
 * @param __Target 目标状态量，单位同 Now
 */
template<int State_Num>
inline void Class_LQR<State_Num>::Set_Target(const Class_Matrix_f32<State_Num, 1> &__Target)
{
    Target = __Target;
}

/**
 * @brief 设置输出限幅
 *
 * @param __Out_Max 输出限幅，0 表示不限幅
 */
template<int State_Num>
inline void Class_LQR<State_Num>::Set_Out_Max(const float &__Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief 获取增益矩阵
 *
 * @return const Class_Matrix_f32<1, State_Num>& 增益矩阵
 */
template<int State_Num>
inline const Class_Matrix_f32<1, State_Num> &Class_LQR<State_Num>::Get_K() const
{
    return (K);
}

/**
 * @brief 获取当前状态量
 *
 * @return const Class_Matrix_f32<State_Num, 1>& 当前状态量
 */
template<int State_Num>
inline const Class_Matrix_f32<State_Num, 1> &Class_LQR<State_Num>::Get_Now() const
{
    return (Now);
}

/**
 * @brief 获取状态误差
 *
 * @return const Class_Matrix_f32<State_Num, 1>& 状态误差
 */
template<int State_Num>
inline const Class_Matrix_f32<State_Num, 1> &Class_LQR<State_Num>::Get_Error() const
{
    return (Error);
}

/**
 * @brief 获取控制量
 *
 * @return float 控制量，单位与增益矩阵一致（本项目为电流 A）
 */
template<int State_Num>
inline float Class_LQR<State_Num>::Get_Out() const
{
    return (Out);
}

/**
 * @brief 获取上次解算是否遇到非法输入
 *
 * @return bool true 表示上次解算输入非法、输出已被强制置 0
 */
template<int State_Num>
inline bool Class_LQR<State_Num>::Get_Invalid_Flag() const
{
    return (Invalid_Flag);
}

/**
 * @brief 定时器周期中断回调函数，计算控制量 u = -K * (x - x_ref)
 *
 * @note 输入非法时输出置 0 并置位 Invalid_Flag（fail-safe）。
 * @note 输出限幅由 Out_Max 指定，0 表示不限幅。
 */
template<int State_Num>
void Class_LQR<State_Num>::TIM_Calculate_PeriodElapsedCallback()
{
    // ---- 防线一：状态量合法性检查 ----
    for (int i = 0; i < State_Num; i++)
    {
        if (Basic_Math_Is_Invalid_Float(Now.Data[i]) ||
            Basic_Math_Is_Invalid_Float(Target.Data[i]))
        {
            Error = Class_Matrix_f32<State_Num, 1>();
            Out = 0.0f;
            Invalid_Flag = true;
            return;
        }
    }

    // ---- 核心：u = -K * (x - x_ref) ----
    Error = Now - Target;
    Out = -(K * Error)[0][0];

    // ---- 防线二：输出限幅 ----
    if (Out_Max != 0.0f)
    {
        if (Basic_Math_Is_Invalid_Float(Out))
        {
            Out = 0.0f;
            Invalid_Flag = true;
            return;
        }
        Basic_Math_Constrain(&Out, -Out_Max, Out_Max);
    }

    Invalid_Flag = false;
}

#endif // !ALG_LQR_H

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/