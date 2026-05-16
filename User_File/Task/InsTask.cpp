#include "user_task.h"
#include "bsp_bmi088.h"
#include "sys_debug.h"

extern "C" void Ins_Task(void *argument) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    BSP_BMI088.EKF_Calculate();
    Sys_Debug_IMU_Update();
  }
}