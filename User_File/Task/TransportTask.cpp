/**
 * @file    TransportTask.cpp
 * @brief   传输任务 —— USB CDC 初始化与遥测输出
 * @author  zzm
 * @version 1.2
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "usb_device.h"
#include "user_task.h"
#include "Balance.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    // CH0=θ CH1=θ̇ CH2=p CH3=ṗ CH4=LQR 输出的单侧共模电流 [A]
    EricTool_USB.Set_Data(5, (int) &Balance.Debug.Theta,
                         (int) &Balance.Debug.Theta_Dot,
                         (int) &Balance.Debug.Position,
                         (int) &Balance.Debug.Velocity,
                         (int) &Balance.Debug.Out);
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(1);
    }
}
