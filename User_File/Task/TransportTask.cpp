/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.2
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "user_task.h"
#include "usb_device.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    // EricTool_USB.Set_Data(2, (int) &Balance_Instance.Target_Pos, (int) &Balance_Instance.Current_Pos);
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(1);
    }
}
