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

#include "callback.h"
#include "bsp_bmi088.h"
#include "sys_timestamp.h"

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback() { SYS_Timestamp.TIM_3600s_PeriodElapsedCallback(); }

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback() {}

/**
 * @brief 每125us调用一次
 * @note
 * 已废弃,原因过高频率中断打断MCU影响RTOS使用,实际实现极为优雅,仅是不兼容RTOS
 */
// void Task125us_Callback()
// {
//     BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
// }

/**
 * @brief 每10us调用一次
 * @note
 * 已废弃,原因过高频率中断打断MCU影响RTOS使用,实际实现极为优雅,仅是不兼容RTOS
 */
// void Task10us_Callback()
// {
//     BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
// }

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (!init_finished) {
    return;
  }

  if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin ||
      GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin) {
    BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  if (!init_finished) {
    return;
  }

  // 选择回调函数
  // if (htim->Instance == TIM4)
  // {
  //     Task10us_Callback();
  // }

  else if (htim->Instance == TIM5) {
    Task3600s_Callback();
  } else if (htim->Instance == TIM6) {
    Task1s_Callback();
  } else if (htim->Instance == TIM7) {
    // Task1ms_Callback();
  }
  // else if (htim->Instance == TIM8)
  // {
  //     Task125us_Callback();
  // }
}

extern "C" void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length,
                              uint16_t Rx_Length) {
  if ((SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port &&
       SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin) ||
      (SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port &&
       SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)) {
    BSP_BMI088.SPI_RxCpltCallback();
  }
}