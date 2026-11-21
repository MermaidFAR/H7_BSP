/**
 * @file alg_trajectory.cpp
 * @author zzm
 * @brief 通过中间速度求解单轴分段恒 jerk 轨迹
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_trajectory.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

bool Class_Trajectory::Init(float __Velocity_Max, float __Acceleration_Max,
                           float __Jerk_Max, float __D_T)
{
    if (Basic_Math_Is_Invalid_Float(__Velocity_Max) || __Velocity_Max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(__Acceleration_Max) || __Acceleration_Max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(__Jerk_Max) || __Jerk_Max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(__D_T) || __D_T <= 0.0f)
    {
        return (false);
    }

    Velocity_Max = __Velocity_Max;
    Acceleration_Max = __Acceleration_Max;
    Jerk_Max = __Jerk_Max;
    D_T = __D_T;
    Status = TRAJECTORY_FINISHED;
    return (Reset(0.0f));
}

/**
 * @brief 对齐合法初始状态; 运动中 Reset 后默认保持该速度并将加速度平滑收至零
 * @return 失败时保留原状态和轨迹
 */
bool Class_Trajectory::Reset(float __Position, float __Velocity, float __Acceleration)
{
    if (Status == TRAJECTORY_UNINITIALIZED || Basic_Math_Is_Invalid_Float(__Position) ||
        Basic_Math_Is_Invalid_Float(__Velocity) || Basic_Math_Is_Invalid_Float(__Acceleration) ||
        (double)fabsf(__Velocity) > Velocity_Max || (double)fabsf(__Acceleration) > Acceleration_Max)
    {
        return (false);
    }
    double acceleration = __Acceleration;
    double peak_velocity = (double)__Velocity + acceleration * fabs(acceleration) / (2.0 * Jerk_Max);
    if (fabs(peak_velocity) > Velocity_Max)
    {
        return (false);
    }

    Struct_State previous = State;
    State.Position = __Position;
    State.Velocity = __Velocity;
    State.Acceleration = __Acceleration;
    Struct_Profile candidate = {};
    Build_Velocity(State.Velocity, State.Acceleration, State.Velocity, candidate.Segment);
    candidate.Length = 3;
    candidate.End_Velocity = State.Velocity;
    Struct_State end = {0.0, State.Velocity, State.Acceleration};
    for (uint8_t i = 0; i < candidate.Length; i++)
    {
        end = Integrate(&end, candidate.Segment[i].Jerk, candidate.Segment[i].Duration);
    }
    candidate.End_Position = State.Position + end.Position;
    if (!Check_Profile(&candidate, end.Position))
    {
        State = previous;
        return (false);
    }
    Segment_Start = State;
    Profile = candidate;
    Segment_Time = 0.0;
    Segment_Index = 0;
    Target_Type = TARGET_VELOCITY;
    Target = __Velocity;
    Target_Changed = false;
    Status = __Acceleration == 0.0f ? TRAJECTORY_FINISHED : TRAJECTORY_RUNNING;
    return (true);
}

bool Class_Trajectory::Set_Target_Position(float __Position)
{
    if (Status == TRAJECTORY_UNINITIALIZED || Basic_Math_Is_Invalid_Float(__Position))
    {
        return (false);
    }
    if (Target_Type != TARGET_POSITION || Target != (double)__Position || Status == TRAJECTORY_ERROR)
    {
        Target_Type = TARGET_POSITION;
        Target = __Position;
        Target_Changed = true;
        Status = TRAJECTORY_RUNNING;
    }
    return (true);
}

bool Class_Trajectory::Set_Target_Velocity(float __Velocity)
{
    if (Status == TRAJECTORY_UNINITIALIZED || Basic_Math_Is_Invalid_Float(__Velocity) ||
        (double)fabsf(__Velocity) > Velocity_Max)
    {
        return (false);
    }
    if (Target_Type != TARGET_VELOCITY || Target != (double)__Velocity || Status == TRAJECTORY_ERROR)
    {
        Target_Type = TARGET_VELOCITY;
        Target = __Velocity;
        Target_Changed = true;
        Status = TRAJECTORY_RUNNING;
    }
    return (true);
}

Class_Trajectory::Struct_State Class_Trajectory::Integrate(
    const Struct_State *__State, double __Jerk, double __Time)
{
    Struct_State out;
    out.Position = __State->Position + __Time * (__State->Velocity +
        __Time * (__State->Acceleration / 2.0 + __Time * __Jerk / 6.0));
    out.Velocity = __State->Velocity + __Time * (__State->Acceleration + __Time * __Jerk / 2.0);
    out.Acceleration = __State->Acceleration + __Time * __Jerk;
    return (out);
}

/**
 * @brief 从任意合法 v/a 过渡到目标速度及零加速度, 最多三个恒 jerk 段
 */
void Class_Trajectory::Build_Velocity(double __Velocity, double __Acceleration,
                                     double __Target, Struct_Segment __Segment[3]) const
{
    double delta = __Target - __Velocity;
    double natural_delta = __Acceleration * fabs(__Acceleration) / (2.0 * Jerk_Max);
    double direction = delta >= natural_delta ? 1.0 : -1.0;
    double oriented_acceleration = direction * __Acceleration;
    double peak_squared = direction * Jerk_Max * delta + __Acceleration * __Acceleration / 2.0;
    double peak = sqrt(fmax(0.0, peak_squared));
    double hold_time = 0.0;
    if (peak > Acceleration_Max)
    {
        peak = Acceleration_Max;
        hold_time = (direction * delta -
            (peak * peak - __Acceleration * __Acceleration / 2.0) / Jerk_Max) / peak;
    }
    __Segment[0].Duration = fmax(0.0, (peak - oriented_acceleration) / Jerk_Max);
    __Segment[0].Jerk = direction * Jerk_Max;
    __Segment[1].Duration = fmax(0.0, hold_time);
    __Segment[1].Jerk = 0.0;
    __Segment[2].Duration = peak / Jerk_Max;
    __Segment[2].Jerk = -direction * Jerk_Max;
}

double Class_Trajectory::Build_Position(double __Middle_Velocity, double __Cruise_Time,
                                       Struct_Profile *__Profile) const
{
    Build_Velocity(State.Velocity, State.Acceleration, __Middle_Velocity, __Profile->Segment);
    __Profile->Segment[3].Duration = __Cruise_Time;
    __Profile->Segment[3].Jerk = 0.0;
    Build_Velocity(__Middle_Velocity, 0.0, 0.0, &__Profile->Segment[4]);
    __Profile->Length = 7;
    __Profile->End_Velocity = 0.0;
    Struct_State state = {0.0, State.Velocity, State.Acceleration};
    for (uint8_t i = 0; i < __Profile->Length; i++)
    {
        if (i == 3)
        {
            // 第一段速度过渡的终态 a=0, 避免舍入残差在长匀速段累积。
            state.Acceleration = 0.0;
        }
        state = Integrate(&state, __Profile->Segment[i].Jerk, __Profile->Segment[i].Duration);
    }
    return (state.Position);
}

bool Class_Trajectory::Plan_Position(Struct_Profile *__Profile) const
{
    double distance = Target - State.Position;
    double lower = -Velocity_Max;
    double upper = Velocity_Max;
    double upper_distance = Build_Position(upper, 0.0, __Profile);
    if (distance >= upper_distance)
    {
        Build_Position(upper, (distance - upper_distance) / Velocity_Max, __Profile);
    }
    else
    {
        double lower_distance = Build_Position(lower, 0.0, __Profile);
        if (distance <= lower_distance)
        {
            Build_Position(lower, (lower_distance - distance) / Velocity_Max, __Profile);
        }
        else
        {
            double stop_distance = Build_Position(0.0, 0.0, __Profile);
            if (distance != stop_distance)
            {
                if (distance > stop_distance)
                {
                    lower = 0.0;
                }
                else
                {
                    upper = 0.0;
                }
                // 位移关于中间速度连续; 只依赖异号括区, 不假设全局单调。
                for (uint8_t i = 0; i < 64; i++)
                {
                    double middle = (lower + upper) / 2.0;
                    double trial_distance = Build_Position(middle, 0.0, __Profile);
                    if (trial_distance == distance)
                    {
                        break;
                    }
                    if (middle == lower || middle == upper || i == 63)
                    {
                        // 中间速度达到浮点精度时, 从位移不足一侧用匀速段补齐。
                        middle = distance > stop_distance ? lower : upper;
                        trial_distance = Build_Position(middle, 0.0, __Profile);
                        if (middle != 0.0)
                        {
                            Build_Position(middle, (distance - trial_distance) / middle, __Profile);
                        }
                        break;
                    }
                    if (trial_distance < distance)
                    {
                        lower = middle;
                    }
                    else
                    {
                        upper = middle;
                    }
                }
            }
        }
    }
    __Profile->End_Position = Target;
    return (Check_Profile(__Profile, distance));
}

bool Class_Trajectory::Plan_Velocity(Struct_Profile *__Profile) const
{
    Build_Velocity(State.Velocity, State.Acceleration, Target, __Profile->Segment);
    __Profile->Length = 3;
    __Profile->End_Velocity = Target;
    Struct_State state = {0.0, State.Velocity, State.Acceleration};
    for (uint8_t i = 0; i < __Profile->Length; i++)
    {
        state = Integrate(&state, __Profile->Segment[i].Jerk, __Profile->Segment[i].Duration);
    }
    __Profile->End_Position = State.Position + state.Position;
    return (Check_Profile(__Profile, state.Position));
}

bool Class_Trajectory::Check_Profile(const Struct_Profile *__Profile, double __Distance) const
{
    Struct_State state = {0.0, State.Velocity, State.Acceleration};
    double velocity_tolerance = 1.0e-10 * Velocity_Max;
    double acceleration_tolerance = 1.0e-10 * Acceleration_Max;
    for (uint8_t i = 0; i < __Profile->Length; i++)
    {
        double time = __Profile->Segment[i].Duration;
        double jerk = __Profile->Segment[i].Jerk;
        if (__Profile->Length == 7 && i == 3)
        {
            if (fabs(state.Acceleration) > acceleration_tolerance)
            {
                return (false);
            }
            state.Acceleration = 0.0;
        }
        if (!isfinite(time) || time < 0.0 || !isfinite(jerk) || fabs(jerk) > Jerk_Max)
        {
            return (false);
        }
        if (jerk != 0.0)
        {
            double extremum_time = -state.Acceleration / jerk;
            if (extremum_time > 0.0 && extremum_time < time)
            {
                Struct_State extremum = Integrate(&state, jerk, extremum_time);
                if (fabs(extremum.Velocity) > Velocity_Max + velocity_tolerance)
                {
                    return (false);
                }
            }
        }
        state = Integrate(&state, jerk, time);
        if (!isfinite(state.Position) || !isfinite(state.Velocity) || !isfinite(state.Acceleration) ||
            fabs(state.Velocity) > Velocity_Max + velocity_tolerance ||
            fabs(state.Acceleration) > Acceleration_Max + acceleration_tolerance)
        {
            return (false);
        }
    }
    return (fabs(state.Position - __Distance) <= 1.0e-9 * (1.0 + fabs(__Distance)) &&
            fabs(state.Velocity - __Profile->End_Velocity) <= velocity_tolerance &&
            fabs(state.Acceleration) <= acceleration_tolerance);
}

void Class_Trajectory::Advance()
{
    double remaining = D_T;
    while (Segment_Index < Profile.Length)
    {
        const Struct_Segment *segment = &Profile.Segment[Segment_Index];
        double available = segment->Duration - Segment_Time;
        if (remaining < available)
        {
            Segment_Time += remaining;
            State = Integrate(&Segment_Start, segment->Jerk, Segment_Time);
            return;
        }
        remaining -= available;
        State = Integrate(&Segment_Start, segment->Jerk, segment->Duration);
        Segment_Time = 0.0;
        Segment_Index++;
        if (Profile.Length == 7 && Segment_Index == 3)
        {
            State.Acceleration = 0.0;
        }
        Segment_Start = State;
        if (Segment_Index == Profile.Length)
        {
            // 求解时已检查终态误差, 此处仅消除浮点残差。
            State.Position = Profile.End_Position;
            State.Velocity = Profile.End_Velocity;
            State.Acceleration = 0.0;
        }
    }
    State.Position += State.Velocity * remaining;
}

Enum_Trajectory_Status Class_Trajectory::TIM_Calculate_PeriodElapsedCallback()
{
    if (Status == TRAJECTORY_UNINITIALIZED)
    {
        return (Status);
    }
    if (Target_Changed)
    {
        Struct_Profile candidate = {};
        bool valid = Target_Type == TARGET_POSITION ? Plan_Position(&candidate) : Plan_Velocity(&candidate);
        Target_Changed = false;
        if (valid)
        {
            Profile = candidate;
            Segment_Start = State;
            Segment_Time = 0.0;
            Segment_Index = 0;
            Status = TRAJECTORY_RUNNING;
        }
        else
        {
            Status = TRAJECTORY_ERROR;
        }
    }
    // 失败时继续已有轨迹, ERROR 保持至新目标或 Reset; 不发布未验证的候选段。
    Advance();
    if (Status != TRAJECTORY_ERROR)
    {
        Status = Segment_Index == Profile.Length ? TRAJECTORY_FINISHED : TRAJECTORY_RUNNING;
    }
    return (Status);
}
