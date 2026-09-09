#include "Gimbal.h"
#include "user_task.h"
#include "Balance.h"

extern "C" void Control_Task(void* argument)
{
    // 在每个 1 kHz CAN 发送周期前生成最新目标；BMI088 High2 任务仍优先处理传感器数据。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    // Gimbal_Init();
    VOFA_Init();
    Balance_Init();
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        // Gimbal_Loop();
        Balance_Control();
        /* USART10 在线调参及 100 Hz JustFloat 遥测；新命令下一控制周期生效。 */
        VOFA_Control();
    }
}
