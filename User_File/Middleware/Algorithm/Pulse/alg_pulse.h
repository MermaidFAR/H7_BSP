/**
 * @file    alg_pulse.h
 * @brief   Pulse algorithm module template
 */

#ifndef ALG_PULSE_H
#define ALG_PULSE_H
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief pulse() 内部状态, 记录当前 timesmode 计数
     *  @note  该结构体在 alg_pulse.cpp 中定义为全局变量 tick, 用于 pulse() 内部状态管理
     */

    typedef struct
    {
        uint32_t timesmode;
        uint8_t timesnum;
        void (*task)(void);
    } timesmode_t;

    void pulse(uint8_t timesnum, const int& Number, ...);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PULSE_H */
