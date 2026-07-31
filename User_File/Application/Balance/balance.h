#ifndef BALANCE_H
#define BALANCE_H

#ifdef BALANCE

#include <cstdint>

#include "alg_pid.h"
#include "cmsis_os2.h"
#include "QD4310.h"
#include "Com.h"


typedef struct
{
    Class_PID Balance_PID; ///< 位置外环，输出小球目标速度。
    Class_PID Speed_PID;   ///< 速度内环，输出水管目标角度。
    float Target_Pos;
    float Target_Speed;
    float Target_Angle;

    float Current_Pos;
    float Current_Speed;

} Balance_t;

extern Balance_t Balance_Instance;
void Balance_loop(void);
void Balance_init(void);

#endif
#endif
