/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与 appTuner 下行指令处理
 * @author  zzm
 * @version 1.1
 * @date    2026-06-06 1.1 接入 App_Tuner_RX 处理 EricTool 下行指令
 */

/* Includes ------------------------------------------------------------------*/

#include "user_task.h"
#include "usb_device.h"
#include "app_tuner.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    EricTool_USB.Set_Data(3,
                          (int) &Debug_IMU_Data.Euler_Yaw_rad,
                          (int) &Debug_IMU_Data.Euler_Pitch_rad,
                          (int) &Debug_IMU_Data.Euler_Roll_rad);
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(1);
    }
}
