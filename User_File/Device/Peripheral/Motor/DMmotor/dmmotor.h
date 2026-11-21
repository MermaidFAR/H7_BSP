/**
 * @file dmmotor.h
 * @brief 达妙电机配置、控制接口与反馈数据。
 * @author Kylin-6
 * @note 原始驱动由 Kylin-6 在 PR #4 贡献，后续适配由 zzm 维护。
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "bsp_can.h"

#include <stdint.h>

enum Enum_DMMotor_Mode : uint8_t
{
    DM_MODE_MIT = 1U,
    DM_MODE_POSITION_SPEED = 2U,
    DM_MODE_SPEED = 3U,
    DM_MODE_FORCE_POSITION = 4U,
};

struct Struct_DMMotor_Feedback
{
    uint8_t state = 0; // 协议状态码
    float position = 0.0f; // 位置，rad，含方向配置
    float total_position = 0.0f; // 累计位置，rad
    float velocity = 0.0f; // 速度，rad/s
    float torque = 0.0f; // 转矩，N*m
    float mos_temperature = 0.0f; // MOS 温度，摄氏度
    float rotor_temperature = 0.0f; // 转子温度，摄氏度
};

class Class_DMMotor
{
public:
    bool Init(FDCAN_HandleTypeDef *hfdcan,
              uint8_t can_id,
              uint16_t master_id,
              Enum_DMMotor_Mode mode,
              bool reverse = false,
              float position_max = 12.5f,
              float velocity_max = 30.0f,
              float torque_max = 10.0f);
    // true 仅表示命令入队成功，不代表电机已执行；false 时可由上层重试。
    bool Enable();
    bool Disable();
    bool ClearError();
    bool SetZeroPosition();
    // 参数非法、其他模式待应答或入队失败均返回 false。
    bool SetMode(Enum_DMMotor_Mode mode);
    void SetMIT(float position_rad,
                float velocity_rad_s,
                float kp,
                float kd,
                float torque_nm);
    void SetPositionSpeed(float position_rad, float velocity_rad_s);
    void SetSpeed(float speed_rad_s);
    void SetForcePosition(float position_rad,
                          float velocity_limit_rad_s,
                          float current_limit_ratio);
    void SetTorque(float torque_nm);

    Struct_DMMotor_Feedback feedback;

private:
    enum Enum_DM_Mode_Submission : uint8_t
    {
        DM_MODE_SUB_IDLE,
        DM_MODE_SUB_RETRY,
        DM_MODE_SUB_NEW_REQUEST,
        DM_MODE_SUB_CONFIRMED,
    };

    static void FeedbackCallback(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);
    bool SendModeCommand(uint8_t command);
    void Publish(const Struct_CAN_Tx_Msg &message);
    uint32_t ControlId() const;

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint8_t can_id = 0U;
    uint16_t master_id = 0U;
    volatile Enum_DMMotor_Mode mode = DM_MODE_MIT;
    volatile uint32_t requested_mode = 0;
    volatile bool mode_pending = false;
    volatile Enum_DM_Mode_Submission mode_submission = DM_MODE_SUB_IDLE;
    volatile uint64_t mode_request_timestamp_us = 0;
    bool reverse = false;
    float position_max = 12.5f;
    float velocity_max = 30.0f;
    float torque_max = 10.0f;
    bool feedback_initialized = false;
    float last_position = 0.0f;
    int32_t total_round = 0;
};

#endif
