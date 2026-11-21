#ifndef SYS_CRITICAL_H
#define SYS_CRITICAL_H

#include "stm32h7xx.h"

/* 保存并恢复原 PRIMASK，允许任务和 ISR 嵌套使用。临界区内不得调用 RTOS API。 */
static inline uint32_t SYS_Enter_Critical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static inline void SYS_Exit_Critical(uint32_t primask)
{
    __DMB();
    __set_PRIMASK(primask);
}

#endif
