/**
 * @file bsp_adc.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的ADC初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

/* Includes ------------------------------------------------------------------*/

#include "adc.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区采样值个数
#define ADC_BUFFER_SIZE 128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ADC采样信息结构体
 *
 */
struct Struct_ADC_Manage_Object
{
    ADC_HandleTypeDef *ADC_Handler;
    uint16_t ADC_Data[ADC_BUFFER_SIZE];
    volatile bool Sample_Valid;
    volatile uint64_t Sample_Timestamp;
};

/* Exported variables --------------------------------------------------------*/

extern struct Struct_ADC_Manage_Object ADC1_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC2_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC3_Manage_Object;

/* Exported function declarations --------------------------------------------*/

bool ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number);
void ADC_TIM_10ms_PeriodElapsedCallback(void);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
