/**
 * @file    StatusTask.cpp
 * @brief   系统状态监控任务骨架
 * @author  zzm
 * @version 1.0
 * @date    2026-05-16
 */

/* Includes ------------------------------------------------------------------*/

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

    for (;;)
    {
        osDelay(50);
    }
}
