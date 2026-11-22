/**
 * @file bsp_power.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 板载电源, 包含两个24V和一个5V输出
 * @version 0.1
 * @date 2025-08-15 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_power.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_Power BSP_Power;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化电源
 *
 * @param dc24_0_status 是否启用 DC24V_0 输出
 * @param dc24_1_status 是否启用 DC24V_1 输出
 * @param dc5_status 是否启用 DC5V 输出
 */
void Class_Power::Init(bool dc24_0_status, bool dc24_1_status, bool dc5_status)
{
    ADC_Manage_Object = &ADC1_Manage_Object;

    HAL_GPIO_WritePin(DC24_0__OUTPUT_GPIO_Port, DC24_0__OUTPUT_Pin, dc24_0_status ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC24_1__OUTPUT_GPIO_Port, DC24_1__OUTPUT_Pin, dc24_1_status ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC5__OUTPUT_GPIO_Port, DC5__OUTPUT_Pin, dc5_status ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Class_Power::Get_Power_Voltage(float *voltage, uint64_t *sample_timestamp_us) const
{
    if (voltage == nullptr)
    {
        return false;
    }

    uint16_t adc_value = 0U;
    uint64_t timestamp_us = 0U;
    const uint32_t primask = SYS_Enter_Critical();
    const Struct_ADC_Manage_Object *adc = ADC_Manage_Object;
    const bool valid = adc != nullptr && adc->ADC_Handler != nullptr && adc->Sample_Valid &&
                       HAL_ADC_GetError(adc->ADC_Handler) == HAL_ADC_ERROR_NONE &&
                       __HAL_ADC_GET_FLAG(adc->ADC_Handler, ADC_FLAG_OVR) == RESET &&
                       SYS_Timestamp_Get_Microsecond() - adc->Sample_Timestamp <= 100000U;
    if (valid)
    {
        adc_value = adc->ADC_Data[ADC_Buffer_Offset];
        timestamp_us = adc->Sample_Timestamp;
    }
    SYS_Exit_Critical(primask);

    // 16 位采样，参考电压 3.3 V，分压比例 11:1。
    *voltage = ((float)adc_value / 65536.0f) * 3.3f * 11.0f;
    if (sample_timestamp_us != nullptr)
    {
        *sample_timestamp_us = timestamp_us;
    }
    return valid;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
