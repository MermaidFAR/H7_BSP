/**
 * @file TIM_1ms_Task.cpp
 * @author zzm
 * @brief 定时器回调分发器, 按周期调度各模块的周期处理函数
 * @version 1.0
 * @date 2026-06-06 1.0 接入 W25Q64JV 自动轮询超时检测
 *
 * @details
 * 通过 pulse() 可变参数接口注册不同周期的回调函数:
 *   - 1ms:  W25Q64JV 超时检测 / 按键扫描
 *   - 10ms: WS2812 灯效刷新
 *   - 50ms: 按键消抖
 *   - 128ms: BMI088 姿态解算
 *
 * @note  pulse 的第二参数为函数指针数量, 后续为可变数量的 void(*)(void) 函数指针
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#include "user_task.h"
#include "bsp_key.h"
#include "bsp_w25q64jv.h"
#include <cstddef>

/** @brief pulse() 内部状态, 记录当前 timesmode 计数 */
static timesmode_t tick;

/**
 * @brief W25Q64JV 自动轮询超时检测回调 (1ms)
 *
 * @note  检测 OSPI 硬件自动轮询是否超时, 防止 Busy_Flag 永久锁死
 * @note  extern "C" 链路: TIM1msTask → pulse → 本函数 → BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback()
 */
extern "C" {
void W25Q64JV_AutoPolling_Callback(void) {
    BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
}
}

/**
 * @brief 定时器回调注册器 (可变参数)
 *
 * @param timesnum 调用周期 (ms), 1 表示每 ms 都调
 * @param Number   注册的函数指针数量
 * @param ...      可变数量的 void (*)(void) 函数指针
 *
 * @note  内部通过 tick.timesmode 计数取模实现分频调度
 */
void pulse(uint8_t timesnum, const int &Number, ...) {
    va_list callback_ptr;
    tick.timesnum = timesnum;
    va_start(callback_ptr, Number);
    for (int i = 0; i < Number; i++) {
        tick.task = (void (*)(void)) va_arg(callback_ptr, int);

        if (tick.timesmode % tick.timesnum == 0) {
            tick.task();
        }
    }
    va_end(callback_ptr);
}

/**
 * @brief 1ms 定时器任务入口 (RTOS 线程)
 *
 * @note  每 1ms 执行一次, 通过 pulse() 分发给各周期回调
 * @note  使用 osDelayUntil 绝对延时保证严格 1ms 周期, 不受 pulse 执行耗时影响
 */
extern "C" void TIM1msTask(void *argument) {
  (void)argument;
  uint32_t xLastWakeTime = osKernelGetTickCount();
  while (1) {
    pulse(1, 2, W25Q64JV_AutoPolling_Callback, BSP_Key_TIM_1ms_Process_PeriodElapsedCallback);
    pulse(10, 1, BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback);
    pulse(50, 1, BSP_Key_TIM_50ms_Process_PeriodElapsedCallback);
    pulse(128, 1, BMI088_TIM_128ms_Calculate_PeriodElapsedCallback);

    tick.timesmode += 1;

    xLastWakeTime += 1;
    osDelayUntil(xLastWakeTime);
  }
}
