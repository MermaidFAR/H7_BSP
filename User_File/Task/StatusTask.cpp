/**
 * @file    StatusTask.cpp
 * @brief   系统状态监控任务骨架
 * @author  zzm
 * @version 1.0
 * @date    2026-05-16
 */

/* Includes ------------------------------------------------------------------*/

#include "user_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/*
 * @brief  系统状态监控任务
 * @note   该任务负责监控系统状态，并通过 EricTool USB 发送遥测数据。
 *         目前示例中仅发送 IMU 数据，后续可根据需要添加更多状态信息。
 *
 * @param argument 任务参数（未使用）
 */
extern "C" void Status_Task(void *argument)
{
EricTool_USB.Set_Data(6,
                        (int)&Debug_IMU_Data.Gyro_X_rad_s,
                        (int)&Debug_IMU_Data.Gyro_Y_rad_s,
                        (int)&Debug_IMU_Data.Gyro_Z_rad_s,
                        (int)&Debug_IMU_Data.Euler_Yaw_rad,
                        (int)&Debug_IMU_Data.Euler_Pitch_rad,
                        (int)&Debug_IMU_Data.Euler_Roll_rad);
  
  for (;;) {
    EricTool_Send_Telemetry(); // 发送遥测数据
    osDelay(1);
  }
}
