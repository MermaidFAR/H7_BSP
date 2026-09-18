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

/** @brief 电调协议控制方式；VOLTAGE 仅供 GM6020 使用，指令均为协议原始量。 */
enum class Enum_DJIMotor_Control_Mode : uint8_t
{
    CURRENT,
    VOLTAGE,
};

/** @brief close_loop 按位组合启用的环，outer_loop 选择单个控制入口。 */
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

/**
 * @brief 电机配置；外部反馈和前馈仅保存指针，所指变量须在使用期间有效。
 * @note Init 成功后默认使能，收到有效反馈后才允许计算非零指令。
 */
struct Struct_DJIMotor_Init_Config
{
    FDCAN_HandleTypeDef *hfdcan; // 已初始化的 CAN 总线
    uint8_t can_id; // 电机编号，M2006/M3508 为 1~8，GM6020 为 1~7；不是报文 ID
    Enum_DJIMotor_Type motor_type;
    uint8_t close_loop; // 按位或组合 DJI_MOTOR_xxx_LOOP；0 表示无闭环
    Enum_DJIMotor_Loop outer_loop; // 选择目标入口；已启用的电流环仍会执行
    PID_InitTypeDef current_pid; // 电流原始值闭环，各启用环的 D_T 须与 Control 周期一致
    PID_InitTypeDef speed_pid; // 速度闭环，内置反馈为输出侧 rad/s
    PID_InitTypeDef angle_pid; // 角度闭环，内置反馈为输出侧累计 rad
    Enum_DJIMotor_Control_Mode control_mode = Enum_DJIMotor_Control_Mode::CURRENT;
    float gear_ratio = 0.0f; // 转子/输出轴传动比；非正数或无效值使用型号默认值
    uint32_t feedback_timeout_ms = 20; // 反馈超时阈值，须大于 0；Control/Send 检查时清零超时指令
    bool reverse = false; // 同时反转内置运动反馈和输出指令，current_raw 保留报文符号
    Enum_DJIMotor_Feedback angle_feedback = Enum_DJIMotor_Feedback::MOTOR;
    Enum_DJIMotor_Feedback speed_feedback = Enum_DJIMotor_Feedback::MOTOR;
    const float *external_angle = nullptr; // 外部角度反馈，rad，调用者统一零点和正方向
    const float *external_speed = nullptr; // 外部速度反馈，rad/s，调用者统一正方向
    const float *current_feedforward = nullptr; // 电流环目标前馈；无电流环时为最终协议指令前馈
    const float *speed_feedforward = nullptr; // 速度环目标前馈，rad/s；未执行速度环时不使用
};

/** @brief kp/ki/kd 支持调试器修改并在 Control 时同步；其余字段用于观察。 */
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
    // 仅用于角度/速度目标：deg 或 deg/s；函数无条件转换为弧度制。
    void SetRef_Degree(float ref);
    void Control();
    void Enable();
    bool Disable();
    void Set_Outer_Loop(Enum_DJIMotor_Loop loop);
    bool Set_Feedback_Source(Enum_DJIMotor_Loop loop, Enum_DJIMotor_Feedback source,
                             const float *feedback = nullptr);
    uint64_t Get_Last_Feedback_Timestamp_Us() const;

    // 接收中断更新运动反馈，Control 更新 PID 状态；整个结构不是原子快照。
    Struct_DJIMotor_Feedback feedback;
    // 32 位 MCU 跨上下文读取时使用 Get_Last_Feedback_Timestamp_Us()。
    volatile uint64_t last_feedback_timestamp_us = 0;
    volatile bool online = false; // 有效反馈置位，Control/Send 检查超时后清除

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

/**
 * @brief 同一总线、同一发送 ID 的电机组，共用一帧报文。
 * @note 先初始化全部成员，再初始化组；按 Init 参数顺序对应各目标值。
 */
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
