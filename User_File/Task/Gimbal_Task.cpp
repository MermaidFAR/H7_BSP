#include "Gimbal.h"
#include "user_task.h"

extern "C" void Gimbal_Task(void* argument)
{
    Gimbal_Init();
    
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        Gimbal_Loop();

    }
}