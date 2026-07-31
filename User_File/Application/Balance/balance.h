#ifndef BALANCE_H
#define BALANCE_H

#ifdef BALANCE

#include <cstdint>

#include "alg_pid.h"
#include "cmsis_os2.h"
#include "QD4310.h"
#include "Com.h"
#include "bsp_bmi088.h"


typedef struct
{
    Class_PID Balance_PID;
    float Target_Pos;
    float Target_Angle;

    float Current_Pos;
    float Acc;

    uint16_t Control_Time;

} Balance_t;

extern Balance_t Balance_Instance;
void Balance_loop(void);
void Balance_init(void);

#endif
#endif
