/**
 * @file alg_slope.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_slope.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 *
 * @param increase_value 增长最大幅度
 * @param decrease_value 降低最大幅度
 * @param slope_first 规划优先类型, 分为目标值优先和真实值优先
 */
void Class_Slope::Init(float increase_value, float decrease_value, Enum_Slope_First slope_first)
{
    Increase_Value = increase_value;
    Decrease_Value = decrease_value;
    Slope_First = slope_first;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 */
void Class_Slope::TIM_Calculate_PeriodElapsedCallback()
{
    if (Basic_Math_Is_Invalid_Float(Target) || Basic_Math_Is_Invalid_Float(Now_Real))
    {
        return;
    }

    float start = Now_Planning;
    if (Slope_First == Slope_First_REAL &&
        ((Target >= Now_Real && Now_Real >= start) || (Target <= Now_Real && Now_Real <= start)))
    {
        start = Now_Real;
    }

    const float remaining = Target - start;
    float next_out = Target;
    if (remaining > 0.0f)
    {
        const float step = start < 0.0f ? Decrease_Value : Increase_Value;
        if (remaining > step)
        {
            next_out = start + step;
        }
    }
    else if (remaining < 0.0f)
    {
        const float step = start > 0.0f ? Decrease_Value : Increase_Value;
        if (-remaining > step)
        {
            next_out = start - step;
        }
    }

    if (isfinite(next_out))
    {
        Out = next_out;
        Now_Planning = next_out;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/