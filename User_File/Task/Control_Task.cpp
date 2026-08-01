#include "Gimbal.h"
#include "user_task.h"
#include <cstdint>

extern "C" void Control_Task(void* argument)
{
    // 在每个 1 kHz CAN 发送周期前生成最新目标；BMI088 High2 任务仍优先处理传感器数据。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    // Gimbal_Init();
    Balance_init();
    uint8_t i = 0;
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        // Gimbal_Loop();
        if (RDK_Target_Control == 1 && i == 0)
        {
            Balance_Target_Sequence_Start();
            i++;
        }
        Balance_loop();
    }
}

