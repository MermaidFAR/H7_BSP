/**
 * @file    StatusTask.cpp
 * @brief   系统状态监控任务骨架
 * @author  zzm
 * @version 1.0
 * @date    2026-05-16
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_erictool.h"
#include "user_task.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Status_Task(void *argument)
{
    (void)argument;

    for (;;) {
        EricTool_Send_Telemetry(); // 发送遥测数据
        osDelay(1);
    }
}
