/**
 * @file alg_slope.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef ALG_SLOPE_H
#define ALG_SLOPE_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 *
 */
enum Enum_Slope_First
{
    Slope_First_REAL = 0,
    Slope_First_TARGET,
};

/**
 * @brief Reusable, 斜坡函数本体
 *
 */
class Class_Slope
{
  public:
    void Init(float increase_value, float decrease_value, Enum_Slope_First slope_first = Slope_First_REAL);

    inline float Get_Out() const;

    inline void Set_Now_Real(float now_real);

    inline void Set_Increase_Value(float increase_value);

    inline void Set_Decrease_Value(float decrease_value);

    inline void Set_Target(float target);

    void TIM_Calculate_PeriodElapsedCallback();

  protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // 规划优先类型
    Enum_Slope_First Slope_First = Slope_First_REAL;
    // 当前规划值
    float Now_Planning = 0.0f;
    // 当前真实值
    float Now_Real = 0.0f;

    // 绝对值增量, 一次计算周期改变值
    float Increase_Value = 0.0f;
    // 绝对值减量, 一次计算周期改变值
    float Decrease_Value = 0.0f;
    // 目标值
    float Target = 0.0f;

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return 输出值
 */
inline float Class_Slope::Get_Out() const
{
    return (Out);
}

/**
 * @brief 设定当前规划值
 *
 * @param now_real 当前规划值
 */
inline void Class_Slope::Set_Now_Real(float now_real)
{
    Now_Real = now_real;
}

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param increase_value 绝对值增量, 一次计算周期改变值
 */
inline void Class_Slope::Set_Increase_Value(float increase_value)
{
    Increase_Value = increase_value;
}

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param decrease_value 绝对值减量, 一次计算周期改变值
 */
inline void Class_Slope::Set_Decrease_Value(float decrease_value)
{
    Decrease_Value = decrease_value;
}

/**
 * @brief 设定目标值
 *
 * @param target 目标值
 */
inline void Class_Slope::Set_Target(float target)
{
    Target = target;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/