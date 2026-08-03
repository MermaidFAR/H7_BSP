#if BALANCE

#include "balance.h"
#include "fdcan.h"

Balance_t Balance_Instance;
QD4310_t QD4310;

// 水管机械角度范围为[-3°, +3°]。
static constexpr float Pipe_Angle_Limit = 3.0f * QD4310_PI / 180.0f;
// 水管-3°对应电机+1.58rad。
static constexpr float Motor_Positive_Limit = 1.58f;
// 水管+3°对应电机5.12rad，即有符号角5.12-2π。
static constexpr float Motor_Negative_Limit = 5.12f - QD4310_TWO_PI;
// 位置外环允许给出的最大小球目标速度。
static constexpr float Ball_Speed_Limit = 10.0f;//无意义,最开始想做位置速度串环,实际输出也到不了10所以没管
// 触发后先在0cm保持10s，再用约10s完成0→6.5→0→-5cm。
static constexpr uint32_t Target_Hold_Time = 3000U;
static constexpr uint32_t Target_Segment_Time = 5000U / 3U;
static constexpr float Target_Positive_Position = 6.5f;
static constexpr float Target_Negative_Position = -5.0f;
static constexpr float Target_Slope_Step = Target_Positive_Position / Target_Segment_Time;

static uint32_t Target_Start_Tick;
static bool Target_Sequence_Enabled;

static float Pipe_Angle_To_Motor_Angle(float pipe_angle);
static void Target_Position_Calculate(void);

// 位置外环参数，输出单位为cm/s；默认关闭以维持水平。
PID_InitTypeDef Balance_Init = {
    .K_P = 0.0078f,
    .K_I = 0.0001f,
    .K_D = 0.000000f,
    .K_F = 0.0f,
    .I_Out_Max = 0.032f,
    .Out_Max = Ball_Speed_Limit,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_ENABLE};

// 速度内环参数，输出单位为rad；默认关闭以维持水平。单位cm
PID_InitTypeDef Speed_Init = {
    .K_P = 0.0008f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.032f,
    .Out_Max = Pipe_Angle_Limit,
    .D_T = 0.001f,
    .Dead_Zone = 0.4f,
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
    Balance_Instance.Speed_PID.Init(Speed_Init.K_P,
                                    Speed_Init.K_I,
                                    Speed_Init.K_D,
                                    Speed_Init.K_F,
                                    Speed_Init.I_Out_Max,
                                    Speed_Init.Out_Max,
                                    Speed_Init.D_T,
                                    Speed_Init.Dead_Zone,
                                    Speed_Init.I_Variable_Speed_A,
                                    Speed_Init.I_Variable_Speed_B,
                                    Speed_Init.I_Separate_Threshold,
                                    Speed_Init.D_First);

    Balance_Instance.Target_Pos = 0.0f;
    Balance_Instance.Target_Speed = 0.0f;
    Balance_Instance.Target_Angle = 0.0f;
    Balance_Instance.Current_Pos = 0.0f;
    Balance_Instance.Current_Speed = 0.0f;
    Balance_Instance.Target_Pos_Slope.Init(Target_Slope_Step,
                                           Target_Slope_Step,
                                           Slope_First_TARGET);
    Target_Sequence_Enabled = false;

    QD4310_Init(&QD4310, 0, &hfdcan2);
    while (!QD4310.enabled)
    {
        QD4310_Enable(&QD4310);
        osDelay(20);
    }
    QD4310_SetAngle(&QD4310, 0.0f);
}

void Balance_Target_Sequence_Start(void)
{
    Target_Start_Tick = osKernelGetTickCount();
    Target_Sequence_Enabled = true;
}

static void Target_Position_Calculate(void)
{
    if (!Target_Sequence_Enabled)
    {
        Balance_Instance.Target_Pos = 0.0f;
        return;
    }

    uint32_t target_time = osKernelGetTickCount() - Target_Start_Tick;

    if (target_time < Target_Hold_Time)
    {
        Balance_Instance.Target_Pos_Slope.Set_Target(0.0f);
    }
    else if (target_time < Target_Hold_Time + Target_Segment_Time)
    {
        Balance_Instance.Target_Pos_Slope.Set_Target(Target_Positive_Position);
    }
    else if (target_time < Target_Hold_Time + 2U * Target_Segment_Time)
    {
        Balance_Instance.Target_Pos_Slope.Set_Target(0.0f);
    }
    else
    {
        Balance_Instance.Target_Pos_Slope.Set_Target(Target_Negative_Position);
    }

    Balance_Instance.Target_Pos_Slope.TIM_Calculate_PeriodElapsedCallback();
    Balance_Instance.Target_Pos = Balance_Instance.Target_Pos_Slope.Get_Out();
}

static float Pipe_Angle_To_Motor_Angle(float pipe_angle)
{
    if (pipe_angle > Pipe_Angle_Limit)
    {
        pipe_angle = Pipe_Angle_Limit;
    }
    else if (pipe_angle < -Pipe_Angle_Limit)
    {
        pipe_angle = -Pipe_Angle_Limit;
    }

    float motor_angle = -pipe_angle;

    if (motor_angle >= 0.0f)
    {
        motor_angle *= Motor_Positive_Limit / Pipe_Angle_Limit;
        if (motor_angle > Motor_Positive_Limit)
        {
            motor_angle = Motor_Positive_Limit;
        }
    }
    else
    {
        motor_angle *= Motor_Negative_Limit / -Pipe_Angle_Limit;
        if (motor_angle < Motor_Negative_Limit)
        {
            motor_angle = Motor_Negative_Limit;
        }
        motor_angle += QD4310_TWO_PI;
    }

    return motor_angle;
}

void Balance_loop()
{
    if (RDK_Target_Control == 1)
    {
        Target_Position_Calculate();
    }
    else
    {
        Balance_Instance.Target_Pos = RDK_Target_Control;
    }

    Balance_Instance.Current_Pos = RDK_Position_Cm;
    Balance_Instance.Current_Speed = RDK_Speed_Cm_S;

    // 位置外环输出期望小球速度。
    Balance_Instance.Balance_PID.Set_Target(Balance_Instance.Target_Pos);
    Balance_Instance.Balance_PID.Set_Now(Balance_Instance.Current_Pos);
    Balance_Instance.Balance_PID.TIM_Calculate_PeriodElapsedCallback();

    // 速度内环输出期望水管角度。
    Balance_Instance.Speed_PID.Set_Target(Balance_Instance.Target_Speed);
    Balance_Instance.Speed_PID.Set_Now(Balance_Instance.Current_Speed);
    Balance_Instance.Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    Balance_Instance.Target_Angle = Balance_Instance.Speed_PID.Get_Out() + Balance_Instance.Balance_PID.Get_Out();
    // 神秘0.00125

    // 水管角限位和电机映射限位分别独立生效。
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
