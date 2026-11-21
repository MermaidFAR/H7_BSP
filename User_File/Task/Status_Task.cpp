/** @file Status_Task.cpp
    @author zzm
    @brief 20 Hz 健康状态任务 */
#include "bsp_buzzer.h"
#include "bsp_ws2812.h"
#include "cmsis_os2.h"
#include "sys_health.h"

static void Status_Show_Health(Enum_Sys_Health_State state, uint8_t phase, bool state_changed)
{
    const Struct_WS2812_Color *color = &WS2812_COLOR_BLACK;
    float frequency = 0.0f;

    switch (state)
    {
    case HEALTH_FAULT:
        if (phase % 4U < 2U)
        {
            color = &WS2812_COLOR_RED;
        }
        // 每秒短鸣一个 50 ms 周期，保持 5% 告警占空比。
        if (phase == 0U)
        {
            frequency = 2400.0f;
        }
        break;
    case HEALTH_OFFLINE:
        if (phase < 10U)
        {
            color = &WS2812_COLOR_YELLOW;
        }
        if (state_changed)
        {
            frequency = 800.0f;
        }
        break;
    case HEALTH_UNINIT:
    case HEALTH_STARTING:
    case HEALTH_WARN:
        if (phase % 10U < 5U)
        {
            color = state == HEALTH_WARN ? &WS2812_COLOR_BLUE : &WS2812_COLOR_CYAN;
        }
        break;
    case HEALTH_OK:
        color = &WS2812_COLOR_GREEN;
        break;
    default:
        break;
    }

    BSP_WS2812.Set_Color(*color, 0.2f);
    const float loudness = frequency > 0.0f && SYS_Health_Buzzer_Mute == 0U ? 0.2f : 0.0f;
    BSP_Buzzer.Set_Sound(frequency, loudness);
}

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    Sys_Health_Init();
    uint32_t period = (uint32_t)(((uint64_t)osKernelGetTickFreq() * SYS_HEALTH_PERIOD_MS + 999) / 1000);
    if (period == 0)
        period = 1;
    uint32_t deadline = osKernelGetTickCount();
    Enum_Sys_Health_State previous = HEALTH_DISABLED;
    uint8_t phase = 0U;
    for (;;)
    {
        Sys_Health_Update();
        const Enum_Sys_Health_State state = Sys_Health_Get_Overall_State();
        const bool state_changed = state != previous;
        if (state_changed)
        {
            phase = 0U;
            previous = state;
        }
        Status_Show_Health(state, phase, state_changed);
        phase = (phase + 1U) % 20U;
        deadline += period;
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - deadline) >= 0)
            deadline = now + period;
        osDelayUntil(deadline);
    }
}
