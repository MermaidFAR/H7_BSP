#include "user_task.h"
#include "sys_health.h"

extern "C" void Control_Task(void* argument)
{
    (void)argument;
    // 在每个 1 kHz CAN 发送周期前生成最新目标；BMI088 High2 任务仍优先处理传感器数据。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    // Gimbal_Init();
    // Balance_init();

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        if (!Sys_Health_Is_Safe())
        {
            // 应用在此接入零力矩、失能和安全反馈查询。
            continue;
        }
        // Gimbal_Loop();
        // Balance_loop();
    }
}

