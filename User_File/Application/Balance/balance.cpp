#if BALANCE

#include "balance.h"

#include <cmath>

#include "fdcan.h"

Balance_t Balance_Instance;
QD4310_t QD4310;

static constexpr float Pipe_Angle_Limit = 3.0f * QD4310_PI / 180.0f;
static constexpr float Motor_Positive_Limit = 1.58f;
static constexpr float Motor_Negative_Limit = QD4310_TWO_PI - 5.12f;
static constexpr float Acc_Enter_Threshold = 0.30f;
static constexpr float Acc_Exit_Threshold = 0.15f;
static constexpr uint16_t Acc_Confirm_Time = 20U;

static float Acc_Compensation(void);
static float Pipe_Angle_To_Motor_Angle(float pipe_angle);

PID_InitTypeDef Balance_Init = {
    .K_P = 0.005f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.0f,
    .Out_Max = Pipe_Angle_Limit,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_ENABLE};

void Balance_init(void)
{
    Balance_Instance.Balance_PID.Init(Balance_Init.K_P,
                                      Balance_Init.K_I,
                                      Balance_Init.K_D,
                                      Balance_Init.K_F,
                                      Balance_Init.I_Out_Max,
                                      Balance_Init.Out_Max,
                                      Balance_Init.D_T,
                                      Balance_Init.Dead_Zone,
                                      Balance_Init.I_Variable_Speed_A,
                                      Balance_Init.I_Variable_Speed_B,
                                      Balance_Init.I_Separate_Threshold,
                                      Balance_Init.D_First);

    Balance_Instance.Target_Pos = 0.0f;
    Balance_Instance.Target_Angle = 0.0f;
    Balance_Instance.Current_Pos = 0.0f;
    Balance_Instance.Acc = 0.0f;
    Balance_Instance.Control_Time = 0U;

    QD4310_Init(&QD4310, 0, &hfdcan2);
    while (!QD4310.enabled)
    {
        QD4310_Enable(&QD4310);
        osDelay(20);
    }
    QD4310_SetAngle(&QD4310, 0.0f);
}

static float Acc_Compensation(void)
{
    Balance_Instance.Acc = BSP_BMI088.Get_Accel_Body()[2][0];
    float acc_abs = fabsf(Balance_Instance.Acc);

    if (Balance_Instance.Control_Time < Acc_Confirm_Time)
    {
        if (acc_abs >= Acc_Enter_Threshold)
        {
            ++Balance_Instance.Control_Time;
        }
        else
        {
            Balance_Instance.Control_Time = 0U;
        }
    }
    else if (acc_abs <= Acc_Exit_Threshold)
    {
        Balance_Instance.Control_Time = 0U;
    }

    if (Balance_Instance.Control_Time < Acc_Confirm_Time)
    {
        return 0.0f;
    }

    return atan2f(-Balance_Instance.Acc, GRAVITY_ACCELERATION);
}

static float Pipe_Angle_To_Motor_Angle(float pipe_angle)
{
    float motor_angle = -pipe_angle;

    if (motor_angle >= 0.0f)
    {
        motor_angle *= Motor_Positive_Limit / Pipe_Angle_Limit;
    }
    else
    {
        motor_angle *= Motor_Negative_Limit / Pipe_Angle_Limit;
        motor_angle += QD4310_TWO_PI;
    }

    return motor_angle;
}

void Balance_loop()
{
    Balance_Instance.Current_Pos = RDK_Position_Cm;

    Balance_Instance.Balance_PID.Set_Target(Balance_Instance.Target_Pos);
    Balance_Instance.Balance_PID.Set_Now(Balance_Instance.Current_Pos);
    Balance_Instance.Balance_PID.TIM_Calculate_PeriodElapsedCallback();

    Balance_Instance.Target_Angle =
        Balance_Instance.Balance_PID.Get_Out() ;//+ Acc_Compensation();

    if (Balance_Instance.Target_Angle > Pipe_Angle_Limit)
    {
        Balance_Instance.Target_Angle = Pipe_Angle_Limit;
    }
    else if (Balance_Instance.Target_Angle < -Pipe_Angle_Limit)
    {
        Balance_Instance.Target_Angle = -Pipe_Angle_Limit;
    }

    QD4310_SetAngle(&QD4310,
                    Pipe_Angle_To_Motor_Angle(Balance_Instance.Target_Angle));
}

#endif
