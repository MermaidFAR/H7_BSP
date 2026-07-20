#include "Gimbal.h"
#include "BSP_BMI088.h"
#include "Comhub.h"
#include "QD4310.h"
#include "alg_pid.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "sys_timestamp.h"
#include <cmath>
#include <sys/_intsup.h>

QDGimbal_t Gimbal;

namespace
{
/** Yaw 每帧最多修正约 6.9 度，远处目标会通过连续帧逐步追上。 */
constexpr float GIMBAL_VISION_MAX_YAW_STEP_RAD = 0.12f;

/** Pitch 每帧最多修正约 2.3 度，降低内置位置环的瞬时跳变。 */
constexpr float GIMBAL_VISION_MAX_PITCH_STEP_RAD = 0.04f;

/** 新测量时刻至少前进 5 ms，过滤 UART 调度抖动和同一测量的 350 Hz 重复发布。 */
constexpr uint64_t GIMBAL_VISION_NEW_MEASUREMENT_MIN_ADVANCE_US = 5000ULL;

/** 实机验证确认 Yaw 相机误差与云台方向一致，Pitch 方向相反。 */
constexpr float GIMBAL_VISION_YAW_SIGN = 1.0f;
constexpr float GIMBAL_VISION_PITCH_SIGN = -1.0f;

/**
 * Pitch 绝对角限位：约 335.2°～358.1°。
 * 当前机械中位实测约为 6.10 rad，限位避免跨越 0/2π 编码边界和继续向下顶机械结构。
 */
constexpr float GIMBAL_PITCH_MIN_ANGLE_RAD = 5.85f;
constexpr float GIMBAL_PITCH_MAX_ANGLE_RAD = 6.25f;

/** 保存上一次已消费的视觉代数，确保同一帧不会被 1 kHz 控制环重复累加。 */
Struct_Comhub_Message Vision_Message = {};

/** 上一次真正用于更新目标的相机测量时刻代理，单位为 H7 本地微秒。 */
uint64_t Gimbal_Vision_Last_Measurement_Timestamp_Us = 0ULL;
bool Gimbal_Vision_Has_Applied_Measurement = false;

float Gimbal_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/**
 * @brief 消费一帧新的树莓派视觉数据并更新双轴目标。
 *
 * 树莓派虽然以约 350 Hz 发布预测包，但真实 PnP 测量约为 30 Hz。若每个发布包都
 * 执行“当前角度 + 相对误差”，同一个旧误差会在下一张图到来前反复推动目标，形成
 * 延迟、越过和往复振荡。因此这里只对新的 TRACKING 测量更新一次目标；
 * PREDICT_ONLY、重复发布包、LOST 或 DETECTING 均保持最后绝对目标。
 */
void Gimbal_UpdateVisionTarget(void)
{
    if (!Comhub_GetLatest(&Vision_Message))
    {
        return;
    }

    const bool Angle_Valid =
        (Vision_Message.Flags & COMHUB_FLAG_ANGLE_VALID) != 0U &&
        std::isfinite(Vision_Message.Yaw_Error_Rad) &&
        std::isfinite(Vision_Message.Pitch_Error_Rad);
    if (!Angle_Valid ||
        Vision_Message.Track_State == COMHUB_TRACK_LOST ||
        Vision_Message.Track_State == COMHUB_TRACK_DETECTING)
    {
        // 重新捕获后，允许第一帧 TRACKING 测量立即建立新的绝对目标。
        Gimbal_Vision_Last_Measurement_Timestamp_Us = 0ULL;
        Gimbal_Vision_Has_Applied_Measurement = false;
        return;
    }

    if (Vision_Message.Track_State != COMHUB_TRACK_TRACKING)
    {
        // PREDICT_ONLY 只保持目标，禁止用旧图预测值继续重锚相对角度。
        return;
    }

    const uint64_t Measurement_Timestamp_Us =
        Vision_Message.Rx_Timestamp_Us >= Vision_Message.Capture_Age_Us
            ? Vision_Message.Rx_Timestamp_Us - Vision_Message.Capture_Age_Us
            : 0ULL;
    const bool Is_New_Measurement =
        !Gimbal_Vision_Has_Applied_Measurement ||
        Measurement_Timestamp_Us >=
            Gimbal_Vision_Last_Measurement_Timestamp_Us +
                GIMBAL_VISION_NEW_MEASUREMENT_MIN_ADVANCE_US;
    if (!Is_New_Measurement)
    {
        return;
    }

    Gimbal_Vision_Last_Measurement_Timestamp_Us = Measurement_Timestamp_Us;
    Gimbal_Vision_Has_Applied_Measurement = true;

    const float Yaw_Now = BSP_BMI088.Get_Euler_Angle().Data[0];
    if (std::isfinite(Yaw_Now))
    {
        const float Yaw_Step = Gimbal_Clamp(
            GIMBAL_VISION_YAW_SIGN * Vision_Message.Yaw_Error_Rad,
            -GIMBAL_VISION_MAX_YAW_STEP_RAD,
            GIMBAL_VISION_MAX_YAW_STEP_RAD);
        Gimbal.Target_Yaw_Angle = Yaw_Now + Yaw_Step;
    }

    if (std::isfinite(Gimbal.Pitch_Motor.angle))
    {
        const float Pitch_Step = Gimbal_Clamp(
            GIMBAL_VISION_PITCH_SIGN * Vision_Message.Pitch_Error_Rad,
            -GIMBAL_VISION_MAX_PITCH_STEP_RAD,
            GIMBAL_VISION_MAX_PITCH_STEP_RAD);
        Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
            Gimbal.Pitch_Motor.angle + Pitch_Step,
            GIMBAL_PITCH_MIN_ANGLE_RAD,
            GIMBAL_PITCH_MAX_ANGLE_RAD);
    }
}
} // namespace

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
 * 该组参数保留给手动速度控制；矩形追踪时 Pitch 直接使用 QD4310 内置位置环，
 * 不经过这组 PID。
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

    // Pitch 速度 PID：保留给手动速度模式，视觉追踪不使用它。
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
        // 电机刚使能时锁住当前姿态，避免在第一帧视觉数据到达前跳向零点。
        const float Yaw_Now = BSP_BMI088.Get_Euler_Angle().Data[0];
        if (std::isfinite(Yaw_Now))
        {
            Gimbal.Target_Yaw_Angle = Yaw_Now;
        }
        if (std::isfinite(Gimbal.Pitch_Motor.angle))
        {
            Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
                Gimbal.Pitch_Motor.angle,
                GIMBAL_PITCH_MIN_ANGLE_RAD,
                GIMBAL_PITCH_MAX_ANGLE_RAD);
        }
        Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_READY);
        return;
    }
    osDelay(20);
    goto RESET;
}

/**
 * @brief 设置两轴目标角度。
 * @param yaw_angle Yaw 目标角度，供 Yaw 角度外环使用。
 * @param pitch_angle Pitch 电机内置位置环目标角度，单位为弧度。
 */
void Gimbal_SetTargetAngle(float yaw_angle, float pitch_angle)
{
    Gimbal.Target_Yaw_Angle = yaw_angle;
    Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
        pitch_angle,
        GIMBAL_PITCH_MIN_ANGLE_RAD,
        GIMBAL_PITCH_MAX_ANGLE_RAD);
}

/**
 * @brief 设置两轴目标角速度。
 * @param yaw_speed Yaw 目标角速度；下一次循环会被 Yaw 角度外环输出覆盖。
 * @param pitch_speed Pitch 手动速度 PID 目标值；矩形追踪模式不使用。
 */
void Gimbal_SetTargetSpeed(float yaw_speed, float pitch_speed)
{
    Gimbal.Target_Yaw_Speed = yaw_speed;
    Gimbal.Target_Pitch_Speed = pitch_speed;
}

/**
 * @brief 执行一次云台控制计算并向两台电机下发命令。
 *
 * 当前控制路径：Yaw 使用“视觉误差 -> 目标角度 -> 角度 PID -> 目标角速度 ->
 * 速度 PID -> 电流指令”；Pitch 使用“视觉误差 -> 目标角度 -> QD4310 内置位置环”。
 * 视觉丢失时保持最后目标，不发送失能命令。
 */
void Gimbal_Loop(void)
{
    // 视觉线程约 350 Hz 发布，本控制环约 1 kHz；只在消息代数变化时更新目标。
    Gimbal_UpdateVisionTarget();

    // Yaw 角度外环：使用 BMI088 的 Yaw 欧拉角，计算速度内环目标。
    Gimbal.Yaw_Angle_PID.Set_Target(Gimbal.Target_Yaw_Angle);
    Gimbal.Yaw_Angle_PID.Set_Now(BSP_BMI088.Get_Euler_Angle().Data[0]);
    Gimbal.Yaw_Angle_PID.TIM_Calculate_PeriodElapsedCallback();
    Gimbal.Target_Yaw_Speed = Gimbal.Yaw_Angle_PID.Get_Out();

    // Yaw 速度内环使用 BMI088 Z 轴角速度反馈。
    Gimbal.Yaw_Speed_PID.Set_Target(Gimbal.Target_Yaw_Speed);
    Gimbal.Yaw_Speed_PID.Set_Now(BSP_BMI088.Get_Gyro_Body().Data[2]);
    Gimbal.Yaw_Speed_PID.TIM_Calculate_PeriodElapsedCallback();

    // Yaw 采用电流控制：速度 PID 输出直接作为电机电流指令。
    QD4310_SetCurrent(&Gimbal.Yaw_Motor, Gimbal.Yaw_Speed_PID.Get_Out());
    // Pitch 采用 QD4310 内置位置环；下发前再次限幅，手动目标也不能绕过机械范围。
    Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
        Gimbal.Target_Pitch_Angle,
        GIMBAL_PITCH_MIN_ANGLE_RAD,
        GIMBAL_PITCH_MAX_ANGLE_RAD);
    QD4310_SetAngle(&Gimbal.Pitch_Motor, Gimbal.Target_Pitch_Angle);
}
