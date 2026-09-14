/**
 * @file dji_motor.cpp
 * @author Kylin-6
 * @brief DJI 电机控制与分组发送
 * @date 2026-09-13
 * @note zzm 维护与完善
 */

/* Includes ------------------------------------------------------------------*/

#include "dji_motor.h"
#include "sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

struct Struct_DJIMotor_Tx_Group
{
    Struct_CAN_Tx_Msg message{};
    Class_DJIMotor *slot_owner[4]{};
    Class_DJIMotor_Group *send_owner = nullptr;
    bool initialized = false;
};

struct Struct_DJIMotor_Registration
{
    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint32_t rx_id = 0;
    bool used = false;
};

/* Private variables ---------------------------------------------------------*/

static constexpr float DJI_MOTOR_PI = 3.14159265358979323846f;
static constexpr float DJI_MOTOR_ENCODER_TO_RADIAN = 2.0f * DJI_MOTOR_PI / 8192.0f;
static constexpr float DJI_MOTOR_RPM_TO_RADIAN_PER_SECOND = 2.0f * DJI_MOTOR_PI / 60.0f;
static constexpr float DJI_MOTOR_RADIAN_TO_DEGREE = 180.0f / DJI_MOTOR_PI;
static constexpr float DJI_MOTOR_ROTOR_SPEED_LPF_ALPHA = 0.85f;
static constexpr uint8_t DJI_MOTOR_MAX_GROUPS = 15;
static constexpr uint8_t DJI_MOTOR_MAX_MOTORS = 24;

static Struct_DJIMotor_Tx_Group DJI_Motor_Tx_Groups[DJI_MOTOR_MAX_GROUPS];
static Struct_DJIMotor_Registration DJI_Motor_Registrations[DJI_MOTOR_MAX_MOTORS];

/* Private function declarations ---------------------------------------------*/

static bool DJI_Motor_Check_PID_Config(const PID_InitTypeDef *config);
static float DJI_Motor_Get_Default_Gear_Ratio(Enum_DJIMotor_Type type);
static bool DJI_Motor_Resolve_Protocol(Enum_DJIMotor_Type type,
                                       Enum_DJIMotor_Control_Mode mode,
                                       uint8_t can_id,
                                       uint32_t *rx_id,
                                       uint32_t *tx_id,
                                       uint8_t *slot,
                                       float *command_limit);
static int DJI_Motor_Find_Group(FDCAN_HandleTypeDef *hfdcan, uint32_t tx_id);
static int DJI_Motor_Find_Free_Registration(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_id);
static uint32_t DJI_Motor_Enter_Critical();
static void DJI_Motor_Exit_Critical(uint32_t interrupt_state);

/* Function prototypes -------------------------------------------------------*/

static bool DJI_Motor_Check_PID_Config(const PID_InitTypeDef *config)
{
    return config->D_T > 0.0f && !Basic_Math_Is_Invalid_Float(config->D_T);
}

static float DJI_Motor_Get_Default_Gear_Ratio(Enum_DJIMotor_Type type)
{
    if (type == Enum_DJIMotor_Type::M2006)
    {
        return 36.0f;
    }
    if (type == Enum_DJIMotor_Type::M3508)
    {
        return 19.0f;
    }
    return 1.0f;
}

static bool DJI_Motor_Resolve_Protocol(Enum_DJIMotor_Type type,
                                       Enum_DJIMotor_Control_Mode mode,
                                       uint8_t can_id,
                                       uint32_t *rx_id,
                                       uint32_t *tx_id,
                                       uint8_t *slot,
                                       float *command_limit)
{
    if (can_id < 1 || can_id > 8 ||
        (type == Enum_DJIMotor_Type::GM6020 && can_id > 7) ||
        (type != Enum_DJIMotor_Type::GM6020 && mode != Enum_DJIMotor_Control_Mode::CURRENT))
    {
        return false;
    }

    *slot = (can_id - 1) % 4;
    if (type == Enum_DJIMotor_Type::GM6020)
    {
        *rx_id = 0x204 + can_id;
        if (mode == Enum_DJIMotor_Control_Mode::VOLTAGE)
        {
            *tx_id = can_id <= 4 ? 0x1FF : 0x2FF;
            *command_limit = 25000.0f;
        }
        else
        {
            *tx_id = can_id <= 4 ? 0x1FE : 0x2FE;
            *command_limit = 16384.0f;
        }
    }
    else
    {
        *rx_id = 0x200 + can_id;
        *tx_id = can_id <= 4 ? 0x200 : 0x1FF;
        *command_limit = type == Enum_DJIMotor_Type::M2006 ? 10000.0f : 16384.0f;
    }
    return true;
}

static int DJI_Motor_Find_Group(FDCAN_HandleTypeDef *hfdcan, uint32_t tx_id)
{
    for (uint8_t i = 0; i < DJI_MOTOR_MAX_GROUPS; ++i)
    {
        if (DJI_Motor_Tx_Groups[i].initialized && DJI_Motor_Tx_Groups[i].message.hfdcan == hfdcan &&
            DJI_Motor_Tx_Groups[i].message.id == tx_id)
        {
            return i;
        }
    }
    for (uint8_t i = 0; i < DJI_MOTOR_MAX_GROUPS; ++i)
    {
        if (!DJI_Motor_Tx_Groups[i].initialized)
        {
            return i;
        }
    }
    return -1;
}

static int DJI_Motor_Find_Free_Registration(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_id)
{
    int free_index = -1;
    for (uint8_t i = 0; i < DJI_MOTOR_MAX_MOTORS; ++i)
    {
        if (DJI_Motor_Registrations[i].used && DJI_Motor_Registrations[i].hfdcan == hfdcan &&
            DJI_Motor_Registrations[i].rx_id == rx_id)
        {
            return -1;
        }
        if (!DJI_Motor_Registrations[i].used && free_index < 0)
        {
            free_index = i;
        }
    }
    return free_index;
}

static uint32_t DJI_Motor_Enter_Critical()
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return interrupt_state;
}

static void DJI_Motor_Exit_Critical(uint32_t interrupt_state)
{
    __DMB();
    __set_PRIMASK(interrupt_state);
}

void Class_DJIMotor::PID_Init(Class_PID *pid, const PID_InitTypeDef *config)
{
    pid->Init(config->K_P, config->K_I, config->K_D, config->K_F,
                config->I_Out_Max, config->Out_Max, config->D_T, config->Dead_Zone,
                config->I_Variable_Speed_A, config->I_Variable_Speed_B,
                config->I_Separate_Threshold, config->D_First);
}

bool Class_DJIMotor::Init(const Struct_DJIMotor_Init_Config &config)
{
    if (initialized || config.hfdcan == nullptr ||
        config.feedback_timeout_ms == 0 ||
        ((config.close_loop & DJI_MOTOR_CURRENT_LOOP) != 0 && !DJI_Motor_Check_PID_Config(&config.current_pid)) ||
        ((config.close_loop & DJI_MOTOR_SPEED_LOOP) != 0 && !DJI_Motor_Check_PID_Config(&config.speed_pid)) ||
        ((config.close_loop & DJI_MOTOR_ANGLE_LOOP) != 0 && !DJI_Motor_Check_PID_Config(&config.angle_pid)) ||
        (config.close_loop & ~(DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP | DJI_MOTOR_ANGLE_LOOP)) != 0 ||
        (config.outer_loop != DJI_MOTOR_OPEN_LOOP && (config.close_loop & config.outer_loop) == 0) ||
        (config.angle_feedback == Enum_DJIMotor_Feedback::EXTERNAL && config.external_angle == nullptr) ||
        (config.speed_feedback == Enum_DJIMotor_Feedback::EXTERNAL && config.external_speed == nullptr))
    {
        return false;
    }

    uint32_t resolved_rx_id, tx_id;
    uint8_t resolved_slot;
    float resolved_limit;
    if (!DJI_Motor_Resolve_Protocol(config.motor_type, config.control_mode, config.can_id,
                                    &resolved_rx_id, &tx_id, &resolved_slot, &resolved_limit))
    {
        return false;
    }

    int group_index = DJI_Motor_Find_Group(config.hfdcan, tx_id);
    int registration_index = DJI_Motor_Find_Free_Registration(config.hfdcan, resolved_rx_id);
    if (group_index < 0 || registration_index < 0 ||
        (DJI_Motor_Tx_Groups[group_index].initialized &&
         (DJI_Motor_Tx_Groups[group_index].slot_owner[resolved_slot] != nullptr ||
          DJI_Motor_Tx_Groups[group_index].send_owner != nullptr)))
    {
        return false;
    }
    hfdcan = config.hfdcan;
    rx_id = resolved_rx_id;
    group = (uint8_t)group_index;
    slot = resolved_slot;
    close_loop = config.close_loop;
    outer_loop = config.outer_loop;
    angle_feedback = config.angle_feedback;
    speed_feedback = config.speed_feedback;
    external_angle = config.external_angle;
    external_speed = config.external_speed;
    current_feedforward = config.current_feedforward;
    speed_feedforward = config.speed_feedforward;
    reverse = config.reverse;
    command_limit = resolved_limit;
    gear_ratio = config.gear_ratio > 0.0f && !Basic_Math_Is_Invalid_Float(config.gear_ratio)
                     ? config.gear_ratio : DJI_Motor_Get_Default_Gear_Ratio(config.motor_type);
    feedback_timeout_us = (uint64_t)config.feedback_timeout_ms * 1000;
    has_temperature = config.motor_type != Enum_DJIMotor_Type::M2006;
    PID_Init(&current_pid, &config.current_pid);
    PID_Init(&speed_pid, &config.speed_pid);
    PID_Init(&angle_pid, &config.angle_pid);
    feedback.pid = {};
    for (uint8_t i = 0; i < 3; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            pid_debug_gains[i][j] = 0.0f;
        }
    }
    Apply_PID_Debug_Gains();
    Update_PID_Debug();

    if (!BSP_CAN_RegisterCallback(resolved_rx_id, config.hfdcan, CAN_RxCpltCallback, this))
    {
        return false;
    }

    Struct_DJIMotor_Tx_Group *sender = &DJI_Motor_Tx_Groups[group_index];
    if (!sender->initialized)
    {
        sender->message.hfdcan = config.hfdcan;
        sender->message.id = tx_id;
        sender->message.len = 8;
        sender->initialized = true;
    }
    sender->slot_owner[resolved_slot] = this;
    DJI_Motor_Registrations[registration_index].hfdcan = config.hfdcan;
    DJI_Motor_Registrations[registration_index].rx_id = resolved_rx_id;
    DJI_Motor_Registrations[registration_index].used = true;
    enabled = true;
    initialized = true;
    return true;
}

void Class_DJIMotor::Apply_PID_Debug_Gains()
{
    Class_PID *pids[3] = {&current_pid, &speed_pid, &angle_pid};
    Struct_DJIMotor_PID_Debug *debug[3] = {&feedback.pid.current, &feedback.pid.speed, &feedback.pid.angle};
    for (uint8_t i = 0; i < 3; ++i)
    {
        // 只有调试入口发生修改时才写回，保留原 PID setter 的调参方式。
        if (debug[i]->kp != pid_debug_gains[i][0] && !Basic_Math_Is_Invalid_Float(debug[i]->kp))
        {
            pids[i]->Set_K_P(debug[i]->kp);
        }
        if (debug[i]->ki != pid_debug_gains[i][1] && !Basic_Math_Is_Invalid_Float(debug[i]->ki))
        {
            pids[i]->Set_K_I(debug[i]->ki);
        }
        if (debug[i]->kd != pid_debug_gains[i][2] && !Basic_Math_Is_Invalid_Float(debug[i]->kd))
        {
            pids[i]->Set_K_D(debug[i]->kd);
        }
        debug[i]->kp = pid_debug_gains[i][0] = pids[i]->Get_K_P();
        debug[i]->ki = pid_debug_gains[i][1] = pids[i]->Get_K_I();
        debug[i]->kd = pid_debug_gains[i][2] = pids[i]->Get_K_D();
    }
}

void Class_DJIMotor::Update_PID_Debug()
{
    Class_PID *pids[3] = {&current_pid, &speed_pid, &angle_pid};
    Struct_DJIMotor_PID_Debug *debug[3] = {&feedback.pid.current, &feedback.pid.speed, &feedback.pid.angle};
    for (uint8_t i = 0; i < 3; ++i)
    {
        debug[i]->kf = pids[i]->Get_K_F();
        debug[i]->integral_out_max = pids[i]->Get_I_Out_Max();
        debug[i]->out_max = pids[i]->Get_Out_Max();
        debug[i]->target = pids[i]->Get_Target();
        debug[i]->now = pids[i]->Get_Now();
        debug[i]->error = pids[i]->Get_Error();
        debug[i]->integral_error = pids[i]->Get_Integral_Error();
        debug[i]->out = pids[i]->Get_Out();
    }
}

void Class_DJIMotor::SetRef(float ref)
{
    reference = ref;
}

void Class_DJIMotor::SetRef_Degree(float ref)
{
    reference = ref / DJI_MOTOR_RADIAN_TO_DEGREE;
}

void Class_DJIMotor::Enable()
{
    enabled = true;
}

void Class_DJIMotor::Set_Outer_Loop(Enum_DJIMotor_Loop loop)
{
    if (loop == DJI_MOTOR_OPEN_LOOP || (close_loop & loop) != 0)
    {
        outer_loop = loop;
    }
}

uint64_t Class_DJIMotor::Get_Last_Feedback_Timestamp_Us() const
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    uint64_t timestamp_us = last_feedback_timestamp_us;
    __DMB();
    __set_PRIMASK(interrupt_state);
    return timestamp_us;
}

void Class_DJIMotor::CAN_RxCpltCallback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t id,
                                        uint8_t *data,
                                        uint32_t len,
                                        void *context)
{
    Class_DJIMotor *motor = (Class_DJIMotor *)context;
    if (motor == nullptr || data == nullptr || len != 8 ||
        hfdcan != motor->hfdcan || id != motor->rx_id)
    {
        return;
    }

    uint16_t new_encoder = ((uint16_t)data[0] << 8) | data[1];
    if (!motor->feedback_initialized)
    {
        motor->feedback_initialized = true;
    }
    else
    {
        int32_t delta = (int32_t)new_encoder - motor->last_encoder;
        if (delta > 4096)
        {
            --motor->total_round;
        }
        else if (delta < -4096)
        {
            ++motor->total_round;
        }
    }
    motor->last_encoder = new_encoder;

    float direction = motor->reverse ? -1.0f : 1.0f;
    int16_t rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->feedback.encoder = new_encoder;
    motor->feedback.rotor_angle = direction * new_encoder * DJI_MOTOR_ENCODER_TO_RADIAN;
    motor->feedback.rotor_total_angle = direction *
        (motor->total_round * 2.0f * DJI_MOTOR_PI + new_encoder * DJI_MOTOR_ENCODER_TO_RADIAN);
    float measured_speed = direction * rpm * DJI_MOTOR_RPM_TO_RADIAN_PER_SECOND;
    motor->feedback.rotor_speed = DJI_MOTOR_ROTOR_SPEED_LPF_ALPHA * motor->feedback.rotor_speed +
        (1.0f - DJI_MOTOR_ROTOR_SPEED_LPF_ALPHA) * measured_speed;
    motor->feedback.output_angle = motor->feedback.rotor_angle / motor->gear_ratio;
    motor->feedback.output_total_angle = motor->feedback.rotor_total_angle / motor->gear_ratio;
    motor->feedback.output_speed = motor->feedback.rotor_speed / motor->gear_ratio;
    motor->feedback.rotor_angle_degree = motor->feedback.rotor_angle * DJI_MOTOR_RADIAN_TO_DEGREE;
    motor->feedback.rotor_total_angle_degree = motor->feedback.rotor_total_angle * DJI_MOTOR_RADIAN_TO_DEGREE;
    motor->feedback.rotor_speed_degree_per_second = motor->feedback.rotor_speed * DJI_MOTOR_RADIAN_TO_DEGREE;
    motor->feedback.output_angle_degree = motor->feedback.output_angle * DJI_MOTOR_RADIAN_TO_DEGREE;
    motor->feedback.output_total_angle_degree = motor->feedback.output_total_angle * DJI_MOTOR_RADIAN_TO_DEGREE;
    motor->feedback.output_speed_degree_per_second = motor->feedback.output_speed * DJI_MOTOR_RADIAN_TO_DEGREE;
    motor->feedback.current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    if (motor->has_temperature)
    {
        motor->feedback.temperature = data[6];
    }
    uint32_t interrupt_state = DJI_Motor_Enter_Critical();
    motor->last_feedback_timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
    motor->online = true;
    DJI_Motor_Exit_Critical(interrupt_state);
}

void Class_DJIMotor::Clear_Command()
{
    feedback.pid.current.active = false;
    feedback.pid.speed.active = false;
    feedback.pid.angle.active = false;
    if (!initialized)
    {
        return;
    }
    Struct_DJIMotor_Tx_Group *sender = &DJI_Motor_Tx_Groups[group];
    sender->message.data[2 * slot] = 0;
    sender->message.data[2 * slot + 1] = 0;
}

bool Class_DJIMotor::Check_Feedback_Timeout()
{
    if (!initialized)
    {
        return false;
    }

    uint32_t interrupt_state = DJI_Motor_Enter_Critical();
    uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    if (online && now_us - last_feedback_timestamp_us > feedback_timeout_us)
    {
        online = false;
    }
    bool ready = enabled && online;
    DJI_Motor_Exit_Critical(interrupt_state);
    if (ready)
    {
        return true;
    }

    Clear_Command();
    current_pid.Set_Integral_Error(0.0f);
    speed_pid.Set_Integral_Error(0.0f);
    angle_pid.Set_Integral_Error(0.0f);
    Update_PID_Debug();
    return false;
}

void Class_DJIMotor::Control()
{
    if (!initialized)
    {
        return;
    }
    Apply_PID_Debug_Gains();
    feedback.pid.current.active = false;
    feedback.pid.speed.active = false;
    feedback.pid.angle.active = false;
    if (!Check_Feedback_Timeout())
    {
        return;
    }

    float output = reference;
    if ((close_loop & DJI_MOTOR_ANGLE_LOOP) != 0 && outer_loop == DJI_MOTOR_ANGLE_LOOP)
    {
        angle_pid.Set_Target(output);
        angle_pid.Set_Now(angle_feedback == Enum_DJIMotor_Feedback::EXTERNAL
                              ? *external_angle : feedback.output_total_angle);
        angle_pid.TIM_Calculate_PeriodElapsedCallback();
        feedback.pid.angle.active = true;
        output = angle_pid.Get_Out();
    }
    if ((close_loop & DJI_MOTOR_SPEED_LOOP) != 0 &&
        (outer_loop == DJI_MOTOR_ANGLE_LOOP || outer_loop == DJI_MOTOR_SPEED_LOOP))
    {
        if (speed_feedforward != nullptr)
        {
            output += *speed_feedforward;
        }
        speed_pid.Set_Target(output);
        speed_pid.Set_Now(speed_feedback == Enum_DJIMotor_Feedback::EXTERNAL
                              ? *external_speed : feedback.output_speed);
        speed_pid.TIM_Calculate_PeriodElapsedCallback();
        feedback.pid.speed.active = true;
        output = speed_pid.Get_Out();
    }
    if (current_feedforward != nullptr)
    {
        output += *current_feedforward;
    }
    if ((close_loop & DJI_MOTOR_CURRENT_LOOP) != 0)
    {
        float logical_current = reverse ? -feedback.current_raw : feedback.current_raw;
        current_pid.Set_Target(output);
        current_pid.Set_Now(logical_current);
        current_pid.TIM_Calculate_PeriodElapsedCallback();
        feedback.pid.current.active = true;
        output = current_pid.Get_Out();
    }
    if (reverse)
    {
        output = -output;
    }
    Basic_Math_Constrain(&output, -command_limit, command_limit);

    int16_t command = (int16_t)output;
    uint16_t raw = (uint16_t)command;
    Struct_DJIMotor_Tx_Group *sender = &DJI_Motor_Tx_Groups[group];
    sender->message.data[2 * slot] = (uint8_t)((raw >> 8) & 0xFF);
    sender->message.data[2 * slot + 1] = (uint8_t)(raw & 0xFF);
    Update_PID_Debug();
}

bool Class_DJIMotor::Disable()
{
    enabled = false;
    if (!initialized)
    {
        return false;
    }
    Clear_Command();
    return CAN_Tx_Perform(&DJI_Motor_Tx_Groups[group].message);
}

bool Class_DJIMotor::Set_Feedback_Source(Enum_DJIMotor_Loop loop,
                                         Enum_DJIMotor_Feedback source,
                                         const float *feedback)
{
    if (source == Enum_DJIMotor_Feedback::EXTERNAL && feedback == nullptr)
    {
        return false;
    }
    if (loop == DJI_MOTOR_ANGLE_LOOP)
    {
        angle_feedback = source;
        external_angle = feedback;
        return true;
    }
    if (loop == DJI_MOTOR_SPEED_LOOP)
    {
        speed_feedback = source;
        external_speed = feedback;
        return true;
    }
    return false;
}

bool Class_DJIMotor_Group::Init(Class_DJIMotor *motor1,
                                Class_DJIMotor *motor2,
                                Class_DJIMotor *motor3,
                                Class_DJIMotor *motor4)
{
    Class_DJIMotor *new_motors[4] = {motor1, motor2, motor3, motor4};
    if (initialized || motor1 == nullptr)
    {
        return false;
    }

    uint8_t count = 0;
    bool found_null = false;
    uint8_t target_group = motor1->group;
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (new_motors[i] == nullptr)
        {
            found_null = true;
            continue;
        }
        if (found_null)
        {
            return false;
        }
        if (!new_motors[i]->initialized || new_motors[i]->group != target_group)
        {
            return false;
        }
        for (uint8_t j = 0; j < i; ++j)
        {
            if (new_motors[j] == new_motors[i])
            {
                return false;
            }
        }
        ++count;
    }

    Struct_DJIMotor_Tx_Group *sender = &DJI_Motor_Tx_Groups[target_group];
    if (sender->send_owner != nullptr)
    {
        return false;
    }
    for (uint8_t slot = 0; slot < 4; ++slot)
    {
        if (sender->slot_owner[slot] == nullptr)
        {
            continue;
        }
        bool included = false;
        for (uint8_t i = 0; i < count; ++i)
        {
            if (new_motors[i] == sender->slot_owner[slot])
            {
                included = true;
            }
        }
        if (!included)
        {
            return false;
        }
    }

    for (uint8_t i = 0; i < 4; ++i)
    {
        motors[i] = new_motors[i];
    }
    motor_count = count;
    physical_group = target_group;
    sender->send_owner = this;
    initialized = true;
    return true;
}

void Class_DJIMotor_Group::SetRef(float ref1, float ref2, float ref3, float ref4)
{
    if (!initialized)
    {
        return;
    }
    float targets[4] = {ref1, ref2, ref3, ref4};
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        motors[i]->SetRef(targets[i]);
    }
}

void Class_DJIMotor_Group::SetRef_Degree(float ref1, float ref2, float ref3, float ref4)
{
    if (!initialized)
    {
        return;
    }
    const float refs[4] = {ref1, ref2, ref3, ref4};
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        motors[i]->SetRef_Degree(refs[i]);
    }
}

void Class_DJIMotor_Group::Update(float ref1, float ref2, float ref3, float ref4)
{
    SetRef(ref1, ref2, ref3, ref4);
    Control();
}

void Class_DJIMotor_Group::Control()
{
    if (!initialized)
    {
        return;
    }
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        motors[i]->Control();
    }
}

bool Class_DJIMotor_Group::Control(float ref1, float ref2, float ref3, float ref4)
{
    if (!initialized)
    {
        return false;
    }
    SetRef(ref1, ref2, ref3, ref4);
    Control();
    bool ready = true;
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        if (!motors[i]->online || !motors[i]->enabled)
        {
            ready = false;
        }
    }
    return Send() && ready;
}

bool Class_DJIMotor_Group::Control_Degree(float ref1, float ref2, float ref3, float ref4)
{
    if (!initialized)
    {
        return false;
    }
    SetRef_Degree(ref1, ref2, ref3, ref4);
    Control();
    bool ready = true;
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        if (!motors[i]->online || !motors[i]->enabled)
        {
            ready = false;
        }
    }
    return Send() && ready;
}

bool Class_DJIMotor_Group::Send()
{
    if (!initialized)
    {
        return false;
    }
    Struct_DJIMotor_Tx_Group *sender = &DJI_Motor_Tx_Groups[physical_group];
    for (uint8_t slot = 0; slot < 4; ++slot)
    {
        if (sender->slot_owner[slot] != nullptr)
        {
            sender->slot_owner[slot]->Check_Feedback_Timeout();
        }
    }
    return CAN_Tx_Perform(&sender->message);
}

void Class_DJIMotor_Group::Enable()
{
    if (!initialized)
    {
        return;
    }
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        motors[i]->Enable();
    }
}

bool Class_DJIMotor_Group::Disable()
{
    if (!initialized)
    {
        return false;
    }
    for (uint8_t i = 0; i < motor_count; ++i)
    {
        motors[i]->enabled = false;
        motors[i]->Clear_Command();
    }
    return Send();
}
