/*
*/

#include "cmsis_os2.h"
#include "user_task.h"
#include "bsp_key.h"
#include <cstddef>
static timesmode_t tick;

void mode(uint8_t timesnum, void (*task)(void)) {

  tick.timesnum = timesnum;
  tick.task = task;
  if (tick.timesmode % tick.timesnum == 0) {
    tick.task();
  }
}

extern "C" void Task1ms_Callback() {
  while (1) {
    mode(1, BSP_Key_TIM_1ms_Process_PeriodElapsedCallback);
    mode(10, BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback);
    mode(50, BSP_Key_TIM_50ms_Process_PeriodElapsedCallback);
    mode(128, BMI088_TIM_128ms_Calculate_PeriodElapsedCallback);
    tick.timesmode += 1;
    osDelayUntil(1);
  }
}