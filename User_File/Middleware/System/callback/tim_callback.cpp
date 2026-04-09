/**
 * @file    tim_callback.cpp
 * @brief   定时器回调函数实现
 * @author  zzm
 * @version 1.0
 * @date    2026-04-09
 * 
 * @details 本文件包含定时器中断回调函数的实现
 *          用于处理定时器相关的中断事件
 * 
 * @note    使用前请确保已正确配置定时器外设
 * 
 * @copyright Copyright (c) 2024
 */

#include "tim_callback.h"

// 全局初始化完成标志位
bool init_finished = false;

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{
}


/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    // BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    // BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // if (!init_finished)
    // {
    //     return;
    // }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        // Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

