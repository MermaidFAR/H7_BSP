/**
 * @file bsp_adc.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的ADC初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_adc.h"
#include "sys_timestamp.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__attribute__((section(".dma_buffer"), aligned(32)))
Struct_ADC_Manage_Object ADC1_Manage_Object = {nullptr};
__attribute__((section(".dma_buffer"), aligned(32)))
Struct_ADC_Manage_Object ADC2_Manage_Object = {nullptr};
__attribute__((section(".dma_buffer"), aligned(32)))
Struct_ADC_Manage_Object ADC3_Manage_Object = {nullptr};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

static Struct_ADC_Manage_Object *ADC_Get_Manage_Object(ADC_HandleTypeDef *hadc)
{
    if (hadc == nullptr) return nullptr;
    if (hadc->Instance == ADC1) return &ADC1_Manage_Object;
    if (hadc->Instance == ADC2) return &ADC2_Manage_Object;
    if (hadc->Instance == ADC3) return &ADC3_Manage_Object;
    return nullptr;
}

/**
 * @brief 初始化ADC
 *
 * @param hadc ADC编号
 * @param Sample_Number 一周期被采样的数据数量, 通道数 * 每个通道的传输数
 */
bool ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number)
{
    Struct_ADC_Manage_Object *manage = ADC_Get_Manage_Object(hadc);
    if (manage == nullptr) return false;
    memset(manage, 0, sizeof(*manage));
    if (Sample_Number == 0 || Sample_Number > ADC_BUFFER_SIZE ||
        HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }
    manage->ADC_Handler = hadc;
    return HAL_ADC_Start_DMA(hadc, (uint32_t *)manage->ADC_Data, Sample_Number) == HAL_OK;
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    Struct_ADC_Manage_Object *manage = ADC_Get_Manage_Object(hadc);
    if (manage != nullptr && manage->ADC_Handler == hadc)
    {
        manage->Sample_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
        manage->Sample_Valid = HAL_ADC_GetError(hadc) == HAL_ADC_ERROR_NONE &&
                               __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR) == RESET;
    }
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    Struct_ADC_Manage_Object *manage = ADC_Get_Manage_Object(hadc);
    if (manage != nullptr && manage->ADC_Handler == hadc)
        manage->Sample_Valid = false;
}

// ADC1 板载电源采样：单通道、单次转换，循环 DMA。
void ADC_TIM_10ms_PeriodElapsedCallback(void)
{
    ADC_HandleTypeDef *hadc = ADC1_Manage_Object.ADC_Handler;
    if (hadc == nullptr) return;
    if (HAL_ADC_GetError(hadc) != HAL_ADC_ERROR_NONE ||
        __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR) != RESET ||
        HAL_DMA_GetState(hadc->DMA_Handle) != HAL_DMA_STATE_BUSY)
    {
        ADC1_Manage_Object.Sample_Valid = false;
        if (HAL_ADC_Stop_DMA(hadc) == HAL_OK)
        {
            // HAL Start_DMA 不清除这些错误状态，残留时完成回调仍会转入错误分支。
            CLEAR_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA);
            HAL_ADC_Start_DMA(hadc, (uint32_t *)ADC1_Manage_Object.ADC_Data, 1);
        }
        return;
    }
    if (HAL_ADC_Start(hadc) != HAL_OK)
        ADC1_Manage_Object.Sample_Valid = false;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
