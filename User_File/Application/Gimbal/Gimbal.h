#if GIMBAL

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "QD4310.h"
#include "alg_pid.h"
#include "alg_fsm.h"
#include "bsp_can.h"

#define YAW_ID 0
#define PITCH_ID 1
#define GIMBAL_PITCH_MIN_ANGLE_RAD 5.85f
#define GIMBAL_PITCH_MAX_ANGLE_RAD 6.25f

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0,
    Gimbal_Status_READY,
    Gimbal_Status_YAW_ERROR,
    Gimbal_Status_PITCH_ERROR,
};


typedef struct
{
    Class_FSM<5> Gimbal_FSM;
    QD4310_t Yaw_Motor;
    QD4310_t Pitch_Motor;

    float Target_Yaw_Angle;
    float Target_Pitch_Angle;
    float Target_Yaw_Speed;
    float Target_Pitch_Speed;

    Class_PID Yaw_Angle_PID;
    Class_PID Pitch_Angle_PID;
    Class_PID Yaw_Speed_PID;
    Class_PID Pitch_Speed_PID;

}QDGimbal_t;


void Gimbal_Init(void);
void Gimbal_Loop(void);

extern QDGimbal_t Gimbal;

#endif
#endif