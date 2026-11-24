/**
 * @file alg_trajectory.h
 * @author zzm
 * @brief 单轴三阶在线轨迹生成器
 */

#ifndef ALG_TRAJECTORY_H
#define ALG_TRAJECTORY_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

enum Enum_Trajectory_Status
{
    TRAJECTORY_UNINITIALIZED = 0,
    TRAJECTORY_RUNNING,
    TRAJECTORY_FINISHED,
    TRAJECTORY_ERROR,
};

/**
 * @brief Reusable, 速度/加速度/jerk 对称限幅的单轴轨迹
 * @details 同一对象由一个控制上下文调用, 周期与 D_T 一致。
 * 位置目标以零速度、零加速度结束; 速度目标到达后保持匀速。
 * 改目标在下一周期从当前规划状态接续, 重复目标不重规划。
 * 不约束位置边界, 允许必要的越过和反向; 不保证时间最优。
 * 接口使用 float, 内部用 double 计算段时间和积分, 减少重规划误差。
 */
class Class_Trajectory
{
  public:
    bool Init(float velocity_max, float acceleration_max, float jerk_max,
              float d_t = 0.001f);
    bool Reset(float position, float velocity = 0.0f, float acceleration = 0.0f);

    bool Set_Target_Position(float position);
    bool Set_Target_Velocity(float velocity);

    Enum_Trajectory_Status TIM_Calculate_PeriodElapsedCallback();

    inline float Get_Position() const;
    inline float Get_Velocity() const;
    inline float Get_Acceleration() const;
    inline Enum_Trajectory_Status Get_Status() const;

  protected:
    struct Struct_State
    {
        double Position;
        double Velocity;
        double Acceleration;
    };

    struct Struct_Segment
    {
        double Duration;
        double Jerk;
    };

    struct Struct_Profile
    {
        Struct_Segment Segment[7];
        double End_Position;
        double End_Velocity;
        uint8_t Length;
    };

    enum Enum_Target
    {
        TARGET_POSITION = 0,
        TARGET_VELOCITY,
    };

    double Velocity_Max = 0.0;
    double Acceleration_Max = 0.0;
    double Jerk_Max = 0.0;
    double D_T = 0.0;

    Struct_State State = {};
    Struct_State Segment_Start = {};
    Struct_Profile Profile = {};
    double Segment_Time = 0.0;
    uint8_t Segment_Index = 0;

    double Target = 0.0;
    Enum_Target Target_Type = TARGET_POSITION;
    bool Target_Changed = false;
    Enum_Trajectory_Status Status = TRAJECTORY_UNINITIALIZED;

    void Build_Velocity(double velocity, double acceleration, double target,
                        Struct_Segment segment[3]) const;
    double Build_Position(double middle_velocity, double cruise_time,
                          Struct_Profile *profile) const;
    bool Plan_Position(Struct_Profile *profile) const;
    bool Plan_Velocity(Struct_Profile *profile) const;
    bool Check_Profile(const Struct_Profile *profile, double distance) const;
    void Advance();
    static Struct_State Integrate(const Struct_State *state, double jerk, double time);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline float Class_Trajectory::Get_Position() const
{
    return ((float) State.Position);
}

inline float Class_Trajectory::Get_Velocity() const
{
    return ((float) State.Velocity);
}

inline float Class_Trajectory::Get_Acceleration() const
{
    return ((float) State.Acceleration);
}

inline Enum_Trajectory_Status Class_Trajectory::Get_Status() const
{
    return (Status);
}

#endif
