/**
 * @file bsp_power.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 板载电源, 包含两个24V和一个5V输出
 * @version 0.1
 * @date 2025-08-15 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_POWER_H
#define BSP_POWER_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_adc.h"
#include "sys_critical.h"
#include "sys_timestamp.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 板载电源相关, 包括两个24V输出和一个5V输出以及一个电源检测器
 *
 */
class Class_Power
{
public:
    void Init(bool dc24_0_status = false, bool dc24_1_status = false, bool dc5_status = false);

    bool Is_Initialized() const
    {
        return ADC_Manage_Object != nullptr && ADC_Manage_Object->ADC_Handler != nullptr;
    }

    inline float Get_Power_Voltage() const;

    /** 获取有效电压及采样时间；失效时电压和采样时间均为零。 */
    bool Get_Power_Voltage(float *voltage, uint64_t *sample_timestamp_us = nullptr) const;

    inline void Set_DC24_0(bool status);

    inline void Set_DC24_1(bool status);

    inline void Set_DC5(bool status);

protected:
    // 初始化相关常量

    // 绑定的ADC
    Struct_ADC_Manage_Object *ADC_Manage_Object = nullptr;
    // 绑定的ADC缓冲区偏移量
    uint16_t ADC_Buffer_Offset = 0;

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

extern Class_Power BSP_Power;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电源电压
 *
 * @return float 电源电压, 单位V
 */
inline float Class_Power::Get_Power_Voltage() const
{
    float voltage;
    Get_Power_Voltage(&voltage);
    return voltage;
}

/**
 * @brief 设置DC24V_0输出
 *
 * @param status 为 true 时启用输出，为 false 时关闭输出。
 */
inline void Class_Power::Set_DC24_0(bool status)
{
    HAL_GPIO_WritePin(DC24_0__OUTPUT_GPIO_Port, DC24_0__OUTPUT_Pin, status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 设置DC24V_1输出
 *
 * @param status 为 true 时启用输出，为 false 时关闭输出。
 */
inline void Class_Power::Set_DC24_1(bool status)
{
    HAL_GPIO_WritePin(DC24_1__OUTPUT_GPIO_Port, DC24_1__OUTPUT_Pin, status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 设置DC5V输出
 *
 * @param status 为 true 时启用输出，为 false 时关闭输出。
 */
inline void Class_Power::Set_DC5(bool status)
{
    HAL_GPIO_WritePin(DC5__OUTPUT_GPIO_Port, DC5__OUTPUT_Pin, status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
