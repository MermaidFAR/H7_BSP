#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "bsp_can.h"

#include <stdint.h>

enum class Enum_DMMotor_Mode : uint8_t
{
    MIT = 1U,
    POSITION_SPEED = 2U,
    SPEED = 3U,
    FORCE_POSITION = 4U,
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
    void Enable();
    void Disable();
    void ClearError();
    void SetZeroPosition();
    void SetMode(Enum_DMMotor_Mode mode);
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

    uint8_t state = 0U;
    float position = 0.0f;
    float total_position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float mos_temperature = 0.0f;
    float rotor_temperature = 0.0f;

private:
    static void FeedbackCallback(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);
    void SendModeCommand(uint8_t command);
    void Publish(const Struct_CAN_Tx_Msg &message);
    uint32_t ControlId() const;

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint8_t can_id = 0U;
    uint16_t master_id = 0U;
    volatile Enum_DMMotor_Mode mode = Enum_DMMotor_Mode::MIT;
    volatile uint32_t requested_mode = 0;
    volatile bool mode_pending = false;
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
