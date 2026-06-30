#include "Gimbal.h"
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