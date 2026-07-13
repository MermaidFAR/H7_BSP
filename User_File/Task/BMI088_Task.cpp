#include "user_task.h"
#include "bsp_bmi088.h"
#include "sys_debug.h"

extern "C" void BMI088_Task(void *argument) {
  osThreadSetPriority(osThreadGetId(), osPriorityHigh2);
  for (;;) {
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    do {
      BSP_BMI088.EKF_Calculate();
    } while (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U);
    Sys_Debug_IMU_Update();
  }
}
