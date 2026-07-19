#include "Gimbal.h"
#include "BSP_BMI088.h"
#include "QD4310.h"
#include "alg_pid.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "sys_timestamp.h"
#include <sys/_intsup.h>

QDGimbal_t Gimbal;

/**
 * @brief Yaw 轴速度内环参数。
 *
 * 输入目标为角速度，反馈来自 BMI088 Z 轴角速度，输出作为 QD4310 电流指令。
 * 该环负责快速抑制速度误差；它的输出上限同时限制 Yaw 电机的最大控制电流。
 */
PID_InitTypeDef Yaw_Speed_PID_Init = {
    .K_P = 0.15f,
    .K_I = 0.63f,
    .K_D = 0.000f,
    .K_F = 0.0f,
    .I_Out_Max = 1.2f,
    .Out_Max = 1.65f,
    .D_T = 0.001f,
    .Dead_Zone = 0.01f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief Yaw 轴角度外环参数。
 *
 * 输入目标为 Yaw 目标角度，反馈来自 BMI088 欧拉角；输出不是电流，而是交给
 * Yaw 速度内环的目标角速度。Kp=32.00 是当前已使用的参数，不在本次注释修改中调整。
 */
PID_InitTypeDef Yaw_Angle_PID_Init = {
    .K_P = 32.00f,
    .K_I = 0.00f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 15.0f,
    .Out_Max = 50.0f,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief Pitch 轴速度 PID 参数。
 *
 * 目标由 Gimbal_SetTargetSpeed() 写入，反馈来自 BMI088 Y 轴角速度。
 * 注意：当前 Gimbal_Loop() 把该 PID 的输出传给 QD4310_SetAngle()，因此它目前
 * 实际被当作电机内置位置环的角度目标使用，而不是作为电流指令使用。
 */
PID_InitTypeDef Pitch_Speed_PID_Init = {
    .K_P = 0.1f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.0f,
    .Out_Max = 10000.0f,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief 预留的 Pitch 角度外环参数。
 *
 * 当前没有初始化和计算这组 PID；保留在此处供后续完成 Pitch 轴辨识、量纲确认
 * 和参数整定后使用。
 */
PID_InitTypeDef Pitch_Angle_PID_Init = {
    .K_P = 0.01f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.0f,
    .Out_Max = 10000.0f,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief 初始化云台状态机、两台 QD4310 电机和已启用的 PID。
 *
 * Yaw 电机连接 FDCAN2，Pitch 电机连接 FDCAN1。初始化末尾会每 20 ms 检查一次
 * 两台电机的 enabled 标志，并依次发送使能命令；只有两轴都报告使能后才返回。
 * 这里的 goto RESET 是使能重试循环，不是 MCU 软件复位。
 */
void Gimbal_Init(void)
{
    // 初始化云台状态机，并绑定两台电机的 CAN 总线和节点 ID。
    Gimbal.Gimbal_FSM.Init();
    QD4310_Init(&Gimbal.Yaw_Motor, YAW_ID, &hfdcan2);
    QD4310_Init(&Gimbal.Pitch_Motor, PITCH_ID, &hfdcan1);

    // Yaw 速度内环：角速度误差 -> 电流指令。
    Gimbal.Yaw_Speed_PID.Init(Yaw_Speed_PID_Init.K_P,
                              Yaw_Speed_PID_Init.K_I,
                              Yaw_Speed_PID_Init.K_D,
                              Yaw_Speed_PID_Init.K_F,
                              Yaw_Speed_PID_Init.I_Out_Max,
                              Yaw_Speed_PID_Init.Out_Max,
                              Yaw_Speed_PID_Init.D_T,
                              Yaw_Speed_PID_Init.Dead_Zone,
                              Yaw_Speed_PID_Init.I_Variable_Speed_A,
                              Yaw_Speed_PID_Init.I_Variable_Speed_B,
                              Yaw_Speed_PID_Init.I_Separate_Threshold,
                              Yaw_Speed_PID_Init.D_First);

    // Pitch 速度 PID：当前输出在循环中被作为 QD4310 内置位置环的角度目标。
    Gimbal.Pitch_Speed_PID.Init(Pitch_Speed_PID_Init.K_P,
                                Pitch_Speed_PID_Init.K_I,
                                Pitch_Speed_PID_Init.K_D,
                                Pitch_Speed_PID_Init.K_F,
                                Pitch_Speed_PID_Init.I_Out_Max,
                                Pitch_Speed_PID_Init.Out_Max,
                                Pitch_Speed_PID_Init.D_T,
                                Pitch_Speed_PID_Init.Dead_Zone,
                                Pitch_Speed_PID_Init.I_Variable_Speed_A,
                                Pitch_Speed_PID_Init.I_Variable_Speed_B,
                                Pitch_Speed_PID_Init.I_Separate_Threshold,
                                Pitch_Speed_PID_Init.D_First);

    // Yaw 角度外环：角度误差 -> 目标角速度。
    Gimbal.Yaw_Angle_PID.Init(Yaw_Angle_PID_Init.K_P,
                              Yaw_Angle_PID_Init.K_I,
                              Yaw_Angle_PID_Init.K_D,
                              Yaw_Angle_PID_Init.K_F,
                              Yaw_Angle_PID_Init.I_Out_Max,
                              Yaw_Angle_PID_Init.Out_Max,
                              Yaw_Angle_PID_Init.D_T,
                              Yaw_Angle_PID_Init.Dead_Zone,
                              Yaw_Angle_PID_Init.I_Variable_Speed_A,
                              Yaw_Angle_PID_Init.I_Variable_Speed_B,
                              Yaw_Angle_PID_Init.I_Separate_Threshold,
                              Yaw_Angle_PID_Init.D_First);

    // Pitch_Angle_PID 暂未整定, 待 pitch 辨识后启用
    // Gimbal.Pitch_Angle_PID.Init(Pitch_Angle_PID_Init.K_P, ...);
    Gimbal.Target_Pitch_Angle = 0.0f;
    Gimbal.Target_Yaw_Angle = 0.0f;
    Gimbal.Target_Pitch_Speed = 0.0f;
    Gimbal.Target_Yaw_Speed = 0.0f;

    // 持续尝试使能两台电机，并通过状态机报告当前未就绪的轴。
RESET:
    if (!Gimbal.Pitch_Motor.enabled)
    {
        Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_PITCH_ERROR);
        QD4310_Enable(&Gimbal.Pitch_Motor);
    }
    else if (!Gimbal.Yaw_Motor.enabled)
    {
        Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_YAW_ERROR);
        QD4310_Enable(&Gimbal.Yaw_Motor);
    }
    else if (Gimbal.Pitch_Motor.enabled && Gimbal.Yaw_Motor.enabled)
    {
        Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_READY);
        return;
    }
    osDelay(20);
    goto RESET;
}

/**
 * @brief 设置两轴目标角度。
 * @param yaw_angle Yaw 目标角度，供 Yaw 角度外环使用。
 * @param pitch_angle Pitch 目标角度；当前循环尚未消费该变量。
 */
void Gimbal_SetTargetAngle(float yaw_angle, float pitch_angle)
{
    Gimbal.Target_Yaw_Angle = yaw_angle;
    Gimbal.Target_Pitch_Angle = pitch_angle;
}

/**
 * @brief 设置两轴目标角速度。
 * @param yaw_speed Yaw 目标角速度；下一次循环会被 Yaw 角度外环输出覆盖。
 * @param pitch_speed Pitch 目标角速度，作为 Pitch 速度 PID 的目标值。
 */
void Gimbal_SetTargetSpeed(float yaw_speed, float pitch_speed)
{
    Gimbal.Target_Yaw_Speed = yaw_speed;
    Gimbal.Target_Pitch_Speed = pitch_speed;
}

/**
 * @brief 执行一次云台控制计算并向两台电机下发命令。
 *
 * 当前控制路径：
 * 1. Yaw：目标角度 -> 角度 PID -> 目标角速度 -> 速度 PID -> 电流指令。
 * 2. Pitch：目标角速度 -> 速度 PID -> QD4310 内置位置环角度指令。
 *
 * 本函数没有读取 Comhub 的视觉报文，也没有把矩形中心误差转换为目标角度；因此
 * 单靠当前文件中的这条控制链不能形成矩形靶视觉追踪闭环。
 */
void Gimbal_Loop(void)
{
    // Yaw 角度外环：使用 BMI088 的 Yaw 欧拉角，计算速度内环目标。
    Gimbal.Yaw_Angle_PID.Set_Target(Gimbal.Target_Yaw_Angle);
    Gimbal.Yaw_Angle_PID.Set_Now(BSP_BMI088.Get_Euler_Angle().Data[0]);
    Gimbal.Yaw_Angle_PID.TIM_Calculate_PeriodElapsedCallback();
    Gimbal.Target_Yaw_Speed = Gimbal.Yaw_Angle_PID.Get_Out();

    // 两轴速度反馈取自 BMI088；Yaw 使用 Z 轴角速度，Pitch 使用 Y 轴角速度。
    Gimbal.Yaw_Speed_PID.Set_Target(Gimbal.Target_Yaw_Speed);
    Gimbal.Pitch_Speed_PID.Set_Target(Gimbal.Target_Pitch_Speed);
    Gimbal.Yaw_Speed_PID.Set_Now(BSP_BMI088.Get_Gyro_Body().Data[2]);
    Gimbal.Pitch_Speed_PID.Set_Now(BSP_BMI088.Get_Gyro_Body().Data[1]);
    Gimbal.Yaw_Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    Gimbal.Pitch_Speed_PID.TIM_Calculate_PeriodElapsedCallback();

    // Yaw 采用电流控制：速度 PID 输出直接作为电机电流指令。
    QD4310_SetCurrent(&Gimbal.Yaw_Motor, Gimbal.Yaw_Speed_PID.Get_Out());
    // Pitch 当前采用电机内置位置环，但传入值仍是 Pitch 速度 PID 的输出。
    // 这表示“速度 PID 输出量”被当成“角度目标量”；该量纲和效果尚需实机验证。
    QD4310_SetAngle(&Gimbal.Pitch_Motor, Gimbal.Pitch_Speed_PID.Get_Out());
}
