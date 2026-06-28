#include "alg_pulse.h"
#include <stdarg.h>
#include <stdint.h>

/** @brief pulse() 内部状态, 记录当前 timesmode 计数 */
timesmode_t tick;

/**
 * @brief 定时器回调注册器 (可变参数)
 *
 * @param timesnum 调用周期 (ms), 1 表示每 ms 都调
 * @param Number   注册的函数指针数量
 * @param ...      可变数量的 void (*)(void) 函数指针
 *
 * @note  内部通过 tick.timesmode 计数取模实现分频调度
 */
void pulse(uint8_t timesnum, const int& Number, ...)
{
    va_list callback_ptr;
    tick.timesnum = timesnum;
    va_start(callback_ptr, Number);
    for (int i = 0; i < Number; i++)
    {
        tick.task = (void (*)(void)) va_arg(callback_ptr, int);

        if (tick.timesmode % tick.timesnum == 0)
        {
            tick.task();
        }
    }
    va_end(callback_ptr);
}
