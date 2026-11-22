#include "user_task.h"
#include "sys_health.h"

extern "C" void Control_Task(void* argument)
{
    (void)argument;
    // RTOS 以 osPriorityHigh1 创建本任务，调度优先级介于 BMI088 解算与 CAN 发送之间。

    // 在此初始化应用控制模块。

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        if (!Sys_Health_Is_Safe())
        {
            // 应用在此接入零力矩、失能和安全反馈查询。
            continue;
        }
        // 在此计算并更新本周期的控制输出。
    }
}

