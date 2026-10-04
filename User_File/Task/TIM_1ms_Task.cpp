/**
 * @file TIM_1ms_Task.cpp
 * @author zzm
 * @brief 定时器回调分发器, 按周期调度各模块的周期处理函数
 * @version 1.0
 * @date 2026-06-06 1.0 接入 W25Q64JV 自动轮询超时检测
 *
 * @details
 * 通过固定回调表分发不同周期的回调函数:
 *   - 1ms:  W25Q64JV 超时检测 / 按键扫描
 *   - 10ms: WS2812 灯效刷新
 *   - 50ms: 按键消抖
 *   - 128ms: BMI088 姿态解算
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#include "user_task.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "bsp_w25q64jv.h"

/**
 * @brief W25Q64JV 自动轮询超时检测回调 (1ms)
 *
 * @note  检测 OSPI 硬件自动轮询是否超时, 防止 Busy_Flag 永久锁死
 * @note  extern "C" 链路: TIM1msTask → Pulse_Dispatch → 本函数 → BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback()
 */
extern "C" {
void W25Q64JV_AutoPolling_Callback(void) {
    BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
}
}

static const PulseEntry_t TIM_1ms_Callback_Table[] = {
    {1, W25Q64JV_AutoPolling_Callback},
    {1, BSP_Key_TIM_1ms_Process_PeriodElapsedCallback},
    {1, BMI088_TIM_1ms_Service_PeriodElapsedCallback},
    {1, UART_TIM_1ms_Recover_PeriodElapsedCallback},
    {10, BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback},
    {10, ADC_TIM_10ms_PeriodElapsedCallback},
    {50, BSP_Key_TIM_50ms_Process_PeriodElapsedCallback},
    {128, BMI088_TIM_128ms_Calculate_PeriodElapsedCallback},
};

/**
 * @brief 1ms 定时器任务入口 (RTOS 线程)
 *
 * @note  每 1ms 执行一次, 通过固定回调表分发给各周期回调
 * @note  使用 osDelayUntil 按绝对节拍延时; 回调执行超时仍会导致任务晚点
 */
extern "C" void TIM1msTask(void *argument) {
  (void)argument;
  uint32_t xLastWakeTime = osKernelGetTickCount();
  uint32_t pulse_tick_ms = 0;
  while (1) {
      Pulse_Dispatch(TIM_1ms_Callback_Table,
                     sizeof(TIM_1ms_Callback_Table) / sizeof(TIM_1ms_Callback_Table[0]),
                     pulse_tick_ms);
      pulse_tick_ms++;

      xLastWakeTime += 1;
      osDelayUntil(xLastWakeTime);
  }
}
