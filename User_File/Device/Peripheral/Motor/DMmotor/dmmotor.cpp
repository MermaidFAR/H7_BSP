/**
 * @file dmmotor.cpp
 * @brief 达妙电机控制帧编码、反馈解析与模式切换。
 * @author Kylin-6
 * @note 原始驱动由 Kylin-6 在 PR #4 贡献，后续适配由 zzm 维护。
 */

/* Includes ------------------------------------------------------------------*/

#include "dmmotor.h"
#include "alg_basic.h"
#include "sys_timestamp.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/** 控制模式通过 CAN ID 偏移区分；参数读写使用独立的 0x7FF 标准帧。 */
static constexpr uint32_t DM_SPEED_MODE_ID_OFFSET = 0x200U;
static constexpr uint32_t DM_POSITION_SPEED_MODE_ID_OFFSET = 0x100U;
static constexpr uint32_t DM_FORCE_POSITION_MODE_ID_OFFSET = 0x300U;
static constexpr uint32_t DM_PARAMETER_ID = 0x7FFU;
static constexpr uint64_t DM_MODE_TIMEOUT_US = 250000; ///< 模式切换应答等待上限，单位 us。
static constexpr uint8_t DM_CMD_ENABLE = 0xFCU;
static constexpr uint8_t DM_CMD_DISABLE = 0xFDU;
static constexpr uint8_t DM_CMD_ZERO_POSITION = 0xFEU;
static constexpr uint8_t DM_CMD_CLEAR_ERROR = 0xFBU;
static constexpr float DM_KP_MIN = 0.0f;
static constexpr float DM_KP_MAX = 500.0f;
static constexpr float DM_KD_MIN = 0.0f;
static constexpr float DM_KD_MAX = 5.0f;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 在 CAN 接收中断中处理模式写入应答或运动反馈。
 * @note master_id 是主控接收 ID，数据首字节低 4 位还需匹配电机 can_id。
 */
void Class_DMMotor::FeedbackCallback(FDCAN_HandleTypeDef *callback_hfdcan,
                                     uint32_t id,
                                     uint8_t *data,
                                     uint32_t len,
                                     void *context)
{
    Class_DMMotor *motor = (Class_DMMotor *)context;
    if (motor == nullptr || data == nullptr || len < 8U ||
        motor->hfdcan != callback_hfdcan || motor->master_id != id ||
        (data[0] & 0x0FU) != motor->can_id)
    {
        return;
    }

    if (motor->mode_pending &&
        SYS_Timestamp.Get_Now_Microsecond() - motor->mode_request_timestamp_us >= DM_MODE_TIMEOUT_US)
    {
        motor->mode_pending = false;
    }

    /** 参数应答与运动反馈共用接收入口，先识别 0x55 写入操作和 0x0A 模式参数。 */
    if (motor->requested_mode != 0 && len == 8 &&
        data[0] == motor->can_id && data[1] == 0 &&
        data[2] == 0x55 && data[3] == 0x0A)
    {
        uint32_t returned_mode;
        memcpy(&returned_mode, &data[4], sizeof(returned_mode));
        if (returned_mode >= 1 && returned_mode <= 4)
        {
            /** 只有未超时且值匹配的应答才更新本地模式；合法的旧应答不作为运动反馈解码。 */
            if (motor->mode_pending && returned_mode == motor->requested_mode)
            {
                motor->mode = (Enum_DMMotor_Mode)returned_mode;
                __DMB();
                motor->mode_pending = false;
            }
            return;
        }
    }

    /** 运动反馈布局：状态/ID 各 4 位，位置 16 位，速度与转矩各 12 位，末尾为两路温度。 */
    motor->feedback.state = (data[0] >> 4) & 0x0FU;
    const float decoded_position =
        Basic_Math_Int_To_Float((data[1] << 8) | data[2], 0, 0xFFFF,
                                -motor->position_max, motor->position_max);
    const float direction = motor->reverse ? -1.0f : 1.0f;

    /**
     * 按位置协议量程展开连续位置：跨界周期为 2 * position_max，不是固定的 2π。
     * 相邻有效反馈间的真实位移需小于 position_max；首帧保留电机上报位置，不自动归零。
     */
    if (!motor->feedback_initialized)
    {
        motor->last_position = decoded_position;
        motor->feedback_initialized = true;
    }
    else if (decoded_position - motor->last_position > motor->position_max)
    {
        --motor->total_round;
    }
    else if (decoded_position - motor->last_position < -motor->position_max)
    {
        ++motor->total_round;
    }

    /** 先在电机原始方向上判断跨界，再统一位置、速度和转矩的逻辑正方向。 */
    motor->last_position = decoded_position;
    motor->feedback.position = direction * decoded_position;
    motor->feedback.total_position = direction *
                            (decoded_position +
                             motor->total_round * 2.0f * motor->position_max);
    motor->feedback.velocity = direction *
                      Basic_Math_Int_To_Float((data[3] << 4) | (data[4] >> 4), 0, 0xFFF,
                                              -motor->velocity_max, motor->velocity_max);
    motor->feedback.torque = direction *
                    Basic_Math_Int_To_Float(((data[4] & 0x0FU) << 8) | data[5], 0, 0xFFF,
                                            -motor->torque_max, motor->torque_max);
    motor->feedback.mos_temperature = data[6];
    motor->feedback.rotor_temperature = data[7];
}

/**
 * @brief 发送使能、失能、清错或置零命令：前 7 字节固定为 0xFF，末字节为命令码。
 * @note 通过插入队列提交一次性命令，避免被周期控制帧覆盖；本接口未向上层返回入队结果。
 */
void Class_DMMotor::SendModeCommand(uint8_t command)
{
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = ControlId();
    message.len = 8U;
    memset(message.data, 0xFF, 7U);
    message.data[7] = command;
    CAN_Tx_Submit(&message);
}

/**
 * @brief 保存通信和量程配置，注册以 master_id 为接收 ID 的反馈回调。
 * @note position_max、velocity_max、torque_max 分别为 rad、rad/s、N*m 的正向量程，
 *       必须与电机端配置一致；motor_mode 只设置本地初始模式，不会向电机发送切换命令。
 * @return true 回调注册成功；false 参数非法或接收 ID 注册失败。
 */
bool Class_DMMotor::Init(FDCAN_HandleTypeDef *motor_hfdcan,
                         uint8_t motor_can_id,
                         uint16_t motor_master_id,
                         Enum_DMMotor_Mode motor_mode,
                         bool motor_reverse,
                         float motor_position_max,
                         float motor_velocity_max,
                         float motor_torque_max)
{
    if (motor_hfdcan == nullptr || motor_can_id > 0x0FU || motor_master_id > 0x7FFU)
    {
        return false;
    }

    if (Basic_Math_Is_Invalid_Float(motor_position_max) || motor_position_max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(motor_velocity_max) || motor_velocity_max <= 0.0f ||
        Basic_Math_Is_Invalid_Float(motor_torque_max) || motor_torque_max <= 0.0f)
    {
        return false;
    }

    hfdcan = motor_hfdcan;
    can_id = motor_can_id;
    master_id = motor_master_id;
    mode = motor_mode;
    reverse = motor_reverse;
    position_max = motor_position_max;
    velocity_max = motor_velocity_max;
    torque_max = motor_torque_max;
    return BSP_CAN_RegisterCallback(master_id, hfdcan, FeedbackCallback, this);
}

/** @brief 按本地记录的模式选择管理命令的发送 ID；切换待应答期间仍使用原模式。 */
uint32_t Class_DMMotor::ControlId() const
{
    switch (mode)
    {
    case Enum_DMMotor_Mode::POSITION_SPEED:
        return DM_POSITION_SPEED_MODE_ID_OFFSET + can_id;
    case Enum_DMMotor_Mode::SPEED:
        return DM_SPEED_MODE_ID_OFFSET + can_id;
    case Enum_DMMotor_Mode::FORCE_POSITION:
        return DM_FORCE_POSITION_MODE_ID_OFFSET + can_id;
    case Enum_DMMotor_Mode::MIT:
    default:
        return can_id;
    }
}

void Class_DMMotor::Enable()
{
    SendModeCommand(DM_CMD_ENABLE);
}

void Class_DMMotor::Disable()
{
    SendModeCommand(DM_CMD_DISABLE);
}

void Class_DMMotor::ClearError()
{
    SendModeCommand(DM_CMD_CLEAR_ERROR);
}

/**
 * @brief 清除本地位置展开状态，并提交电机置零命令。
 * @note 本地状态先于电机处理命令重置，下一帧反馈重新建立起点；调用完成不代表电机已置零。
 */
void Class_DMMotor::SetZeroPosition()
{
    feedback_initialized = false;
    last_position = 0.0f;
    total_round = 0;
    feedback.total_position = 0.0f;
    SendModeCommand(DM_CMD_ZERO_POSITION);
}

/**
 * @brief 向 0x7FF 提交模式参数写入请求，由匹配应答确认后更新本地 mode。
 * @note 待应答期间允许重发同一模式且不延长原超时窗口；不同模式请求需等应答或超时后重试。
 */
void Class_DMMotor::SetMode(Enum_DMMotor_Mode new_mode)
{
    const uint32_t mode_value = (uint32_t)new_mode;
    if (mode_value < 1 || mode_value > 4)
    {
        return;
    }

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_PARAMETER_ID;
    message.len = 8U;
    message.data[0] = can_id;
    message.data[1] = 0U;
    message.data[2] = 0x55U;
    message.data[3] = 0x0AU;
    /** 参数值占后 4 字节，沿用 STM32 的小端内存布局。 */
    memcpy(&message.data[4], &mode_value, sizeof(mode_value));
    /** 将入队与待应答状态更新放在同一临界区，防止接收中断看到尚未登记的请求。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    if (mode_pending && now_us - mode_request_timestamp_us >= DM_MODE_TIMEOUT_US)
    {
        mode_pending = false;
    }
    if ((!mode_pending || requested_mode == mode_value) && CAN_Tx_Submit(&message))
    {
        if (!mode_pending)
        {
            mode_request_timestamp_us = now_us;
        }
        requested_mode = mode_value;
        mode_pending = true;
    }
    __DMB();
    if (primask == 0)
    {
        __enable_irq();
    }
}

/**
 * @brief 更新 CAN 周期发送槽，同一 (总线, ID) 只保留最新控制帧。
 * @note 此处仅发布缓冲，不等待硬件发送，也未向上层返回发布结果。
 */
void Class_DMMotor::Publish(const Struct_CAN_Tx_Msg &message)
{
    CAN_Tx_Perform(&message);
}

/**
 * @brief 将 MIT 位置、速度、增益和转矩目标限幅、量化后打包为 8 字节控制帧。
 * @note 反向配置只改变位置、速度、转矩符号，kp/kd 保持非负；量程需与电机端一致。
 *       本函数直接发布 MIT 格式报文，调用者需保证电机已处于对应模式。
 */
void Class_DMMotor::SetMIT(float position_rad,
                           float velocity_rad_s,
                           float kp,
                           float kd,
                           float torque_nm)
{
    const float direction = reverse ? -1.0f : 1.0f;
    /** 位置量化为 16 位，速度、kp、kd、转矩各量化为 12 位，总计 64 位。 */
    const uint16_t position = (uint16_t)Basic_Math_Float_To_Int(
        Basic_Math_Constrain(direction * position_rad, -position_max, position_max),
        -position_max, position_max, 0, 0xFFFF);
    const uint16_t velocity = (uint16_t)Basic_Math_Float_To_Int(
        Basic_Math_Constrain(direction * velocity_rad_s, -velocity_max, velocity_max),
        -velocity_max, velocity_max, 0, 0xFFF);
    const uint16_t proportional = (uint16_t)Basic_Math_Float_To_Int(
        Basic_Math_Constrain(kp, DM_KP_MIN, DM_KP_MAX), DM_KP_MIN, DM_KP_MAX, 0, 0xFFF);
    const uint16_t derivative = (uint16_t)Basic_Math_Float_To_Int(
        Basic_Math_Constrain(kd, DM_KD_MIN, DM_KD_MAX), DM_KD_MIN, DM_KD_MAX, 0, 0xFFF);
    const uint16_t torque = (uint16_t)Basic_Math_Float_To_Int(
        Basic_Math_Constrain(direction * torque_nm, -torque_max, torque_max),
        -torque_max, torque_max, 0, 0xFFF);

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = can_id;
    message.len = 8U;
    /** 各字段高位在前；data[3] 拼接速度/kp，data[6] 拼接 kd/转矩的半字节。 */
    message.data[0] = (uint8_t)(position >> 8);
    message.data[1] = (uint8_t)position;
    message.data[2] = (uint8_t)(velocity >> 4);
    message.data[3] = (uint8_t)(((velocity & 0x0FU) << 4) | (proportional >> 8));
    message.data[4] = (uint8_t)proportional;
    message.data[5] = (uint8_t)(derivative >> 4);
    message.data[6] = (uint8_t)(((derivative & 0x0FU) << 4) | (torque >> 8));
    message.data[7] = (uint8_t)torque;
    Publish(message);
}

/**
 * @brief 发布位置速度模式目标，前 4 字节为位置 rad，后 4 字节为速度 rad/s。
 * @note 两个 float 直接按 STM32 小端内存布局复制，不经过 MIT 整数量化；不自动切换模式。
 */
void Class_DMMotor::SetPositionSpeed(float position_rad, float velocity_rad_s)
{
    const float direction = reverse ? -1.0f : 1.0f;
    position_rad *= direction;
    velocity_rad_s *= direction;

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_POSITION_SPEED_MODE_ID_OFFSET + can_id;
    message.len = 8U;
    memcpy(&message.data[0], &position_rad, sizeof(position_rad));
    memcpy(&message.data[4], &velocity_rad_s, sizeof(velocity_rad_s));
    Publish(message);
}

/** @brief 发布速度模式目标：4 字节小端 float，单位 rad/s；不自动切换模式。 */
void Class_DMMotor::SetSpeed(float speed_rad_s)
{
    if (reverse)
    {
        speed_rad_s = -speed_rad_s;
    }

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_SPEED_MODE_ID_OFFSET + can_id;
    message.len = sizeof(speed_rad_s);
    memcpy(message.data, &speed_rad_s, sizeof(speed_rad_s));
    Publish(message);
}

/**
 * @brief 发布力位模式目标：位置 float + 速度上限 uint16 + 电流比例上限 uint16，均为小端。
 * @param position_rad 目标位置，rad，随 reverse 改变符号。
 * @param velocity_limit_rad_s 速度幅值上限，限制在 0~100 rad/s 后按 0.01 rad/s 编码。
 * @param current_limit_ratio 电流上限比例，限制在 0~1 后编码为 0~10000。
 * @note 两个上限为非负幅值，不随方向翻转；本函数不自动切换模式。
 */
void Class_DMMotor::SetForcePosition(float position_rad,
                                     float velocity_limit_rad_s,
                                     float current_limit_ratio)
{
    const float direction = reverse ? -1.0f : 1.0f;
    position_rad *= direction;
    const uint16_t velocity_limit = (uint16_t)(
        Basic_Math_Constrain(velocity_limit_rad_s, 0.0f, 100.0f) * 100.0f);
    const uint16_t current_limit = (uint16_t)(
        Basic_Math_Constrain(current_limit_ratio, 0.0f, 1.0f) * 10000.0f);

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_FORCE_POSITION_MODE_ID_OFFSET + can_id;
    message.len = 8U;
    memcpy(&message.data[0], &position_rad, sizeof(position_rad));
    message.data[4] = (uint8_t)velocity_limit;
    message.data[5] = (uint8_t)(velocity_limit >> 8);
    message.data[6] = (uint8_t)current_limit;
    message.data[7] = (uint8_t)(current_limit >> 8);
    Publish(message);
}

/** @brief 复用 MIT 帧实现纯转矩目标：kp/kd 置零，仅保留转矩项，电机需处于 MIT 模式。 */
void Class_DMMotor::SetTorque(float torque_nm)
{
    SetMIT(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
}
