#include "user_task.h"
#include "bsp_bmi088.h"
#include "sys_debug.h"

extern "C" void Ins_Task(void *argument) {
  for (;;) {
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    BSP_BMI088.EKF_Calculate();
    Sys_Debug_IMU_Update();
  }
}