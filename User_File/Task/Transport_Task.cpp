/**
 * @file    Transport_Task.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.2
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "usb_device.h"
#include "user_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *)
{
    static const volatile float *const imu_channels[] = {
        &Debug_IMU_Data.Euler_Yaw_rad,
        &Debug_IMU_Data.Euler_Pitch_rad,
        &Debug_IMU_Data.Euler_Roll_rad,
    };

    MX_USB_DEVICE_Init();
    EricTool_USB.Set_Data(imu_channels, sizeof(imu_channels) / sizeof(imu_channels[0]));
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(1);
    }
}
