/**
 * @file dji_motor.h
 * @author Kylin-6
 * @brief DJI 电机控制与分组发送
 * @date 2026-09-13
 * @note zzm 维护与完善
 */

#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "bsp_can.h"

#include <stdint.h>

/* Exported macros ------------------------------------------------------------*/

/* Exported types -------------------------------------------------------------*/

enum class Enum_DJIMotor_Type : uint8_t
{
    M2006,
    M3508,
    GM6020,
};

enum class Enum_DJIMotor_Control_Mode : uint8_t
{
    CURRENT,
    VOLTAGE,
};

enum Enum_DJIMotor_Loop : uint8_t
{
    DJI_MOTOR_OPEN_LOOP = 0,
    DJI_MOTOR_CURRENT_LOOP = 1 << 0,
    DJI_MOTOR_SPEED_LOOP = 1 << 1,
    DJI_MOTOR_ANGLE_LOOP = 1 << 2,
};

enum class Enum_DJIMotor_Feedback : uint8_t
{
    MOTOR,
    EXTERNAL,
};

struct Struct_DJIMotor_Init_Config
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t can_id;
    Enum_DJIMotor_Type motor_type;
    uint8_t close_loop;
    Enum_DJIMotor_Loop outer_loop;
    PID_InitTypeDef current_pid;
    PID_InitTypeDef speed_pid;
    PID_InitTypeDef angle_pid;
    Enum_DJIMotor_Control_Mode control_mode = Enum_DJIMotor_Control_Mode::CURRENT;
    float gear_ratio = 0.0f;
    uint32_t feedback_timeout_ms = 20;
    bool reverse = false;
    Enum_DJIMotor_Feedback angle_feedback = Enum_DJIMotor_Feedback::MOTOR;
    Enum_DJIMotor_Feedback speed_feedback = Enum_DJIMotor_Feedback::MOTOR;
    const float *external_angle = nullptr; // rad
    const float *external_speed = nullptr; // rad/s
    const float *current_feedforward = nullptr;
    const float *speed_feedforward = nullptr;
};

struct Struct_DJIMotor_PID_Debug
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float kf = 0.0f;
    float integral_out_max = 0.0f;
    float out_max = 0.0f;
    float target = 0.0f;
    float now = 0.0f;
    float error = 0.0f; // 最近一次 PID 计算经过死区处理后的误差
    float integral_error = 0.0f;
    float out = 0.0f; // PID 自身输出，未包含后级环和电机协议限幅
    bool active = false; // 本次控制是否执行该环；停止时为 false
};

struct Struct_DJIMotor_PID_Feedback
{
    Struct_DJIMotor_PID_Debug current;
    Struct_DJIMotor_PID_Debug speed;
    Struct_DJIMotor_PID_Debug angle;
};

struct Struct_DJIMotor_Feedback
{
    uint16_t encoder = 0; // 协议原始编码器值，0~8191
    float rotor_angle = 0.0f; // 转子侧单圈角度，rad，含方向配置
    float rotor_total_angle = 0.0f; // 转子侧累计角度，rad
    float rotor_speed = 0.0f; // 转子侧滤波速度，rad/s
    float output_angle = 0.0f; // 输出侧单圈对应角度，rad
    float output_total_angle = 0.0f; // 输出侧累计角度，rad
    float output_speed = 0.0f; // 输出侧速度，rad/s
    float rotor_angle_degree = 0.0f; // 转子侧单圈角度，deg
    float rotor_total_angle_degree = 0.0f; // 转子侧累计角度，deg
    float rotor_speed_degree_per_second = 0.0f; // 转子侧速度，deg/s
    float output_angle_degree = 0.0f; // 输出侧单圈对应角度，deg
    float output_total_angle_degree = 0.0f; // 输出侧累计角度，deg
    float output_speed_degree_per_second = 0.0f; // 输出侧速度，deg/s
    int16_t current_raw = 0; // 协议原始电流值，不是 A
    uint8_t temperature = 0; // 温度，摄氏度；M2006 不提供
    Struct_DJIMotor_PID_Feedback pid;
};

class Class_DJIMotor
{
public:
    bool Init(const Struct_DJIMotor_Init_Config &config);
    // 角度环为 rad，速度环为 rad/s；开环/电流环仍为协议控制量。
    void SetRef(float ref);
    // 角度环为 deg，速度环为 deg/s；内部转换为弧度制。
    void SetRef_Degree(float ref);
    void Control();
    void Enable();
    bool Disable();
    void Set_Outer_Loop(Enum_DJIMotor_Loop loop);
    bool Set_Feedback_Source(Enum_DJIMotor_Loop loop, Enum_DJIMotor_Feedback source,
                             const float *feedback = nullptr);
    uint64_t Get_Last_Feedback_Timestamp_Us() const;

    Struct_DJIMotor_Feedback feedback;
    // 32 位 MCU 跨上下文读取时使用 Get_Last_Feedback_Timestamp_Us()。
    volatile uint64_t last_feedback_timestamp_us = 0;
    volatile bool online = false;

    Class_PID current_pid;
    Class_PID speed_pid;
    Class_PID angle_pid;

protected:
    friend class Class_DJIMotor_Group;
    static void CAN_RxCpltCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                                   uint8_t *data, uint32_t len, void *context);
    static void PID_Init(Class_PID *pid, const PID_InitTypeDef *config);
    bool Check_Feedback_Timeout();
    void Clear_Command();
    void Apply_PID_Debug_Gains();
    void Update_PID_Debug();

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint32_t rx_id = 0;
    uint8_t group = 0;
    uint8_t slot = 0;
    uint8_t close_loop = DJI_MOTOR_OPEN_LOOP;
    Enum_DJIMotor_Loop outer_loop = DJI_MOTOR_OPEN_LOOP;
    Enum_DJIMotor_Feedback angle_feedback = Enum_DJIMotor_Feedback::MOTOR;
    Enum_DJIMotor_Feedback speed_feedback = Enum_DJIMotor_Feedback::MOTOR;
    const float *external_angle = nullptr;
    const float *external_speed = nullptr;
    const float *current_feedforward = nullptr;
    const float *speed_feedforward = nullptr;
    float pid_debug_gains[3][3]{}; // current/speed/angle 上次同步的 kp/ki/kd
    float reference = 0.0f;
    float command_limit = 0.0f;
    float gear_ratio = 1.0f;
    uint64_t feedback_timeout_us = 20000;
    bool has_temperature = false;
    bool reverse = false;
    bool enabled = false;
    bool initialized = false;
    bool feedback_initialized = false;
    uint16_t last_encoder = 0;
    int32_t total_round = 0;
};

class Class_DJIMotor_Group
{
public:
    bool Init(Class_DJIMotor *motor1,
              Class_DJIMotor *motor2 = nullptr,
              Class_DJIMotor *motor3 = nullptr,
              Class_DJIMotor *motor4 = nullptr);
    void SetRef(float ref1,
                float ref2 = 0.0f,
                float ref3 = 0.0f,
                float ref4 = 0.0f);
    void SetRef_Degree(float ref1,
                       float ref2 = 0.0f,
                       float ref3 = 0.0f,
                       float ref4 = 0.0f);
    void Update(float ref1,
                float ref2 = 0.0f,
                float ref3 = 0.0f,
                float ref4 = 0.0f);
    bool Control(float ref1,
                 float ref2 = 0.0f,
                 float ref3 = 0.0f,
                 float ref4 = 0.0f);
    bool Control_Degree(float ref1,
                        float ref2 = 0.0f,
                        float ref3 = 0.0f,
                        float ref4 = 0.0f);
    void Control();
    bool Send();
    void Enable();
    bool Disable();

protected:
    Class_DJIMotor *motors[4]{};
    uint8_t motor_count = 0;
    uint8_t physical_group = 0;
    bool initialized = false;
};

#endif
