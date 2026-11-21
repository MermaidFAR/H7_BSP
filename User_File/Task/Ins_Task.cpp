/** @file Ins_Task.cpp @brief 预留惯性导航任务入口 */
#include "bsp_bmi088.h"
#include "user_task.h"

extern "C" void Ins_Task(void* argument)
{
    (void)argument;

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    }
}