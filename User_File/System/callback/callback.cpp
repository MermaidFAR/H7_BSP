/**
 * @file    callback.cpp
 * @brief   中断回调统一分发中心
 * @author  zzm
 * @version 1.0
 * @date    2026-04-09
 *
 * @details 本文件集中实现 HAL 弱回调的覆写与分发，
 *          包括 GPIO 外部中断(EXTI)、定时器(TIM)周期中断与 SPI 完成回调，
 *          按外设实例路由到对应的设备层处理函数
 *
 * @note    使用前请确保相关外设已正确初始化，且 init_finished 已置位
 *
 * @copyright Copyright (c) 2024
 */

#include "callback.h"
#include "SEGGER_RTT.h"
#include "bsp_bmi088.h"
#include "bsp_w25q64jv.h"
#include "cmsis_os2.h"
#include "sys_timestamp.h"
#include <cstdint>

extern "C" {
    extern osThreadId_t GimbalTaskHandle;
}

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
 * @brief 每1ms调用一次
 * @note
 */
void Task1ms_Callback()
{
    // Balance 控制链直接使用 TIM4 的 1 ms 基准，即 1 kHz。
    osThreadFlagsSet(GimbalTaskHandle, 0x0001);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (!init_finished) {
    return;
  }

  if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin ||
      GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin) {
    BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
  }
}

extern "C" void TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (!init_finished)
    {
        return;
    }
    else if (htim->Instance == TIM4)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
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

void OSPI2_Polling_Callback()
{
    SEGGER_RTT_printf(0, "Polling CB\n");
    BSP_W25Q64JV.OSPI_StatusMatchCallback();
}

void OSPI2_Rx_Callback(uint8_t *Buffer)
{
    SEGGER_RTT_printf(0, "Rx CB\n");
    BSP_W25Q64JV.OSPI_RxCallback();
}

void OSPI2_Tx_Callback(uint8_t *Buffer)
{
    SEGGER_RTT_printf(0, "Tx CB\n");
    BSP_W25Q64JV.OSPI_TxCallback();
}
