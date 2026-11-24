/**
 * @file    Transport_Task.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.2
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"
#include "usb_device.h"
#include "user_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *)
{
    const Struct_BMI088_Euler_Telemetry &euler = BSP_BMI088.Get_Euler_Telemetry();
    const volatile float *const imu_channels[] = {
        &euler.Yaw_rad,
        &euler.Pitch_rad,
        &euler.Roll_rad,
    };

    MX_USB_DEVICE_Init();
    EricTool_USB.Set_Data(imu_channels, sizeof(imu_channels) / sizeof(imu_channels[0]));
    // 发送尝试后等待一个 RTOS 节拍，BMI088 通道值最多每 5 ms 刷新一次。
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(1);
    }
}
