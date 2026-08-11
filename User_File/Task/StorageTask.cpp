/**
 * @file StorageTask.cpp
 * @author zzm
 * @brief 预留存储任务入口
 */

/* Includes ------------------------------------------------------------------*/

extern "C"
{
#include "cmsis_os2.h"
}

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Storage_Task(void *argument)
{
    (void)argument;
    osThreadExit();
}
