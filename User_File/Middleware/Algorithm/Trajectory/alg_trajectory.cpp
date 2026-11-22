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

bool Class_Trajectory::Init(float velocity_max, float acceleration_max,
                           float jerk_max, float d_t)
{
    if (Basic_Math_Is_Invalid_Float(velocity_max) || velocity_max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(acceleration_max) || acceleration_max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(jerk_max) || jerk_max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(d_t) || d_t <= 0.0f)
    {
        return (false);
    }

    Velocity_Max = velocity_max;
    Acceleration_Max = acceleration_max;
    Jerk_Max = jerk_max;
    D_T = d_t;
    Status = TRAJECTORY_FINISHED;
    return (Reset(0.0f));
}

/**
 * @brief 对齐合法初始状态; 运动中 Reset 后默认保持该速度并将加速度平滑收至零
 * @return 失败时保留原状态和轨迹
 */
bool Class_Trajectory::Reset(float position, float velocity, float acceleration)
{
    if (Status == TRAJECTORY_UNINITIALIZED || Basic_Math_Is_Invalid_Float(position) ||
        Basic_Math_Is_Invalid_Float(velocity) || Basic_Math_Is_Invalid_Float(acceleration) ||
        (double)fabsf(velocity) > Velocity_Max || (double)fabsf(acceleration) > Acceleration_Max)
    {
        return (false);
    }
    double initial_acceleration = acceleration;
    double peak_velocity = (double)velocity + initial_acceleration * fabs(initial_acceleration) / (2.0 * Jerk_Max);
    if (fabs(peak_velocity) > Velocity_Max)
    {
        return (false);
    }

    Struct_State previous = State;
    State.Position = position;
    State.Velocity = velocity;
    State.Acceleration = acceleration;
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
    Target = velocity;
    Target_Changed = false;
    Status = acceleration == 0.0f ? TRAJECTORY_FINISHED : TRAJECTORY_RUNNING;
    return (true);
}

bool Class_Trajectory::Set_Target_Position(float position)
{
    if (Status == TRAJECTORY_UNINITIALIZED || Basic_Math_Is_Invalid_Float(position))
    {
        return (false);
    }
    if (Target_Type != TARGET_POSITION || Target != (double)position || Status == TRAJECTORY_ERROR)
    {
        Target_Type = TARGET_POSITION;
        Target = position;
        Target_Changed = true;
        Status = TRAJECTORY_RUNNING;
    }
    return (true);
}

bool Class_Trajectory::Set_Target_Velocity(float velocity)
{
    if (Status == TRAJECTORY_UNINITIALIZED || Basic_Math_Is_Invalid_Float(velocity) ||
        (double)fabsf(velocity) > Velocity_Max)
    {
        return (false);
    }
    if (Target_Type != TARGET_VELOCITY || Target != (double)velocity || Status == TRAJECTORY_ERROR)
    {
        Target_Type = TARGET_VELOCITY;
        Target = velocity;
        Target_Changed = true;
        Status = TRAJECTORY_RUNNING;
    }
    return (true);
}

Class_Trajectory::Struct_State Class_Trajectory::Integrate(
    const Struct_State *state, double jerk, double time)
{
    Struct_State out;
    out.Position = state->Position + time * (state->Velocity +
        time * (state->Acceleration / 2.0 + time * jerk / 6.0));
    out.Velocity = state->Velocity + time * (state->Acceleration + time * jerk / 2.0);
    out.Acceleration = state->Acceleration + time * jerk;
    return (out);
}

/**
 * @brief 从任意合法 v/a 过渡到目标速度及零加速度, 最多三个恒 jerk 段
 */
void Class_Trajectory::Build_Velocity(double velocity, double acceleration,
                                     double target, Struct_Segment segment[3]) const
{
    double delta = target - velocity;
    double natural_delta = acceleration * fabs(acceleration) / (2.0 * Jerk_Max);
    double direction = delta >= natural_delta ? 1.0 : -1.0;
    double oriented_acceleration = direction * acceleration;
    double peak_squared = direction * Jerk_Max * delta + acceleration * acceleration / 2.0;
    double peak = sqrt(fmax(0.0, peak_squared));
    double hold_time = 0.0;
    if (peak > Acceleration_Max)
    {
        peak = Acceleration_Max;
        hold_time = (direction * delta -
            (peak * peak - acceleration * acceleration / 2.0) / Jerk_Max) / peak;
    }
    segment[0].Duration = fmax(0.0, (peak - oriented_acceleration) / Jerk_Max);
    segment[0].Jerk = direction * Jerk_Max;
    segment[1].Duration = fmax(0.0, hold_time);
    segment[1].Jerk = 0.0;
    segment[2].Duration = peak / Jerk_Max;
    segment[2].Jerk = -direction * Jerk_Max;
}

double Class_Trajectory::Build_Position(double middle_velocity, double cruise_time,
                                       Struct_Profile *profile) const
{
    Build_Velocity(State.Velocity, State.Acceleration, middle_velocity, profile->Segment);
    profile->Segment[3].Duration = cruise_time;
    profile->Segment[3].Jerk = 0.0;
    Build_Velocity(middle_velocity, 0.0, 0.0, &profile->Segment[4]);
    profile->Length = 7;
    profile->End_Velocity = 0.0;
    Struct_State state = {0.0, State.Velocity, State.Acceleration};
    for (uint8_t i = 0; i < profile->Length; i++)
    {
        if (i == 3)
        {
            // 第一段速度过渡的终态 a=0, 避免舍入残差在长匀速段累积。
            state.Acceleration = 0.0;
        }
        state = Integrate(&state, profile->Segment[i].Jerk, profile->Segment[i].Duration);
    }
    return (state.Position);
}

bool Class_Trajectory::Plan_Position(Struct_Profile *profile) const
{
    double distance = Target - State.Position;
    double lower = -Velocity_Max;
    double upper = Velocity_Max;
    double upper_distance = Build_Position(upper, 0.0, profile);
    if (distance >= upper_distance)
    {
        Build_Position(upper, (distance - upper_distance) / Velocity_Max, profile);
    }
    else
    {
        double lower_distance = Build_Position(lower, 0.0, profile);
        if (distance <= lower_distance)
        {
            Build_Position(lower, (lower_distance - distance) / Velocity_Max, profile);
        }
        else
        {
            double stop_distance = Build_Position(0.0, 0.0, profile);
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
                    double trial_distance = Build_Position(middle, 0.0, profile);
                    if (trial_distance == distance)
                    {
                        break;
                    }
                    if (middle == lower || middle == upper || i == 63)
                    {
                        // 中间速度达到浮点精度时, 从位移不足一侧用匀速段补齐。
                        middle = distance > stop_distance ? lower : upper;
                        trial_distance = Build_Position(middle, 0.0, profile);
                        if (middle != 0.0)
                        {
                            Build_Position(middle, (distance - trial_distance) / middle, profile);
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
    profile->End_Position = Target;
    return (Check_Profile(profile, distance));
}

bool Class_Trajectory::Plan_Velocity(Struct_Profile *profile) const
{
    Build_Velocity(State.Velocity, State.Acceleration, Target, profile->Segment);
    profile->Length = 3;
    profile->End_Velocity = Target;
    Struct_State state = {0.0, State.Velocity, State.Acceleration};
    for (uint8_t i = 0; i < profile->Length; i++)
    {
        state = Integrate(&state, profile->Segment[i].Jerk, profile->Segment[i].Duration);
    }
    profile->End_Position = State.Position + state.Position;
    return (Check_Profile(profile, state.Position));
}

bool Class_Trajectory::Check_Profile(const Struct_Profile *profile, double distance) const
{
    Struct_State state = {0.0, State.Velocity, State.Acceleration};
    double velocity_tolerance = 1.0e-10 * Velocity_Max;
    double acceleration_tolerance = 1.0e-10 * Acceleration_Max;
    for (uint8_t i = 0; i < profile->Length; i++)
    {
        double time = profile->Segment[i].Duration;
        double jerk = profile->Segment[i].Jerk;
        if (profile->Length == 7 && i == 3)
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
    return (fabs(state.Position - distance) <= 1.0e-9 * (1.0 + fabs(distance)) &&
            fabs(state.Velocity - profile->End_Velocity) <= velocity_tolerance &&
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
