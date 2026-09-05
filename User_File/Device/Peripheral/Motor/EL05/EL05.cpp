/**
 * @file    EL05.cpp
 * @brief   灵足 EL05 扩展帧控制与反馈解析。
 * @author  zzm
 */

/* Includes ------------------------------------------------------------------*/
#include "EL05.h"

#include <math.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define EL05_CMD_MOTION (1)
#define EL05_CMD_FEEDBACK (2)
#define EL05_CMD_ENABLE (3)
#define EL05_CMD_STOP (4)
#define EL05_CMD_ZERO (6)
#define EL05_CMD_READ (17)
#define EL05_CMD_WRITE (18)
#define EL05_CMD_FAULT (21)
#define EL05_CMD_REPORT (24)

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
static void EL05_Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                          uint8_t *data, uint32_t len, void *context);
static bool EL05_Stop(EL05_t *motor, bool clear_error);

/* Function prototypes -------------------------------------------------------*/
static bool EL05_IsValid(const EL05_t *motor)
{
    return motor != NULL && motor->initialized;
}

static bool EL05_ModeIsValid(EL05_Mode_t mode)
{
    return mode == EL05_MODE_MOTION || mode == EL05_MODE_POSITION_PP ||
           mode == EL05_MODE_SPEED || mode == EL05_MODE_CURRENT ||
           mode == EL05_MODE_POSITION_CSP;
}

static uint32_t EL05_SlotKey(const EL05_t *motor)
{
    return 0x05000000 | ((uint32_t)motor->host_id << 8) | motor->id;
}

static float EL05_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint16_t EL05_Encode(float value, float minimum, float maximum)
{
    value = EL05_Clamp(value, minimum, maximum);
    return (uint16_t)((value - minimum) * 65535.0f / (maximum - minimum));
}

static float EL05_Decode(const uint8_t *data, float minimum, float maximum)
{
    uint16_t raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return (float)raw * (maximum - minimum) / 65535.0f + minimum;
}

static void EL05_PutBigEndian16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static uint32_t EL05_GetLittleEndian32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void EL05_MakeMessage(const EL05_t *motor, uint8_t command,
                             Struct_CAN_Tx_Msg *message)
{
    memset(message, 0, sizeof(*message));
    message->hfdcan = motor->hfdcan;
    message->id_type = FDCAN_EXTENDED_ID;
    message->id = ((uint32_t)command << 24) |
                  ((uint32_t)motor->host_id << 8) | motor->id;
    message->len = 8;
}

static void EL05_MakeParameter(const EL05_t *motor, uint8_t command,
                               uint16_t index, uint32_t value,
                               Struct_CAN_Tx_Msg *message)
{
    EL05_MakeMessage(motor, command, message);
    message->data[0] = (uint8_t)index;
    message->data[1] = (uint8_t)(index >> 8);
    message->data[4] = (uint8_t)value;
    message->data[5] = (uint8_t)(value >> 8);
    message->data[6] = (uint8_t)(value >> 16);
    message->data[7] = (uint8_t)(value >> 24);
}

static bool EL05_SetFloatTarget(EL05_t *motor, uint16_t index, float value)
{
    Struct_CAN_Tx_Msg message;
    uint32_t raw;

    if (!EL05_IsValid(motor) || !motor->command_enabled || !isfinite(value))
        return false;

    memcpy(&raw, &value, sizeof(raw));
    EL05_MakeParameter(motor, EL05_CMD_WRITE, index, raw, &message);
    message.slot_key = EL05_SlotKey(motor);
    return CAN_Tx_Perform(&message);
}

bool EL05_Init(EL05_t *motor, uint8_t id, uint8_t host_id,
               FDCAN_HandleTypeDef *hfdcan)
{
    if (motor == NULL || motor->initialized || id == 0 ||
        (hfdcan != &hfdcan1 && hfdcan != &hfdcan2 && hfdcan != &hfdcan3))
        return false;

    memset(motor, 0, sizeof(*motor));
    motor->hfdcan = hfdcan;
    motor->id = id;
    motor->host_id = host_id;
    motor->mode = EL05_MODE_MOTION;
    motor->initialized = true;
    if (!BSP_CAN_RegisterCallbackEx(((uint32_t)id << 8) | host_id,
                                    0xFFFF, FDCAN_EXTENDED_ID, hfdcan,
                                    EL05_Callback, motor))
    {
        motor->initialized = false;
        return false;
    }
    return true;
}

bool EL05_SetMode(EL05_t *motor, EL05_Mode_t mode)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor) || motor->command_enabled || !EL05_ModeIsValid(mode))
        return false;

    if (!EL05_Stop(motor, false)) return false;
    EL05_MakeParameter(motor, EL05_CMD_WRITE, EL05_PARAM_RUN_MODE,
                       (uint32_t)mode, &message);
    if (!CAN_Tx_Submit(&message)) return false;
    motor->mode = mode;
    return true;
}

bool EL05_Enable(EL05_t *motor)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor) || motor->stop_pending) return false;
    EL05_MakeMessage(motor, EL05_CMD_ENABLE, &message);
    if (!CAN_Tx_Submit(&message)) return false;
    motor->command_enabled = true;
    return true;
}

static bool EL05_Stop(EL05_t *motor, bool clear_error)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor)) return false;
    motor->command_enabled = false;
    CAN_Tx_Stop(motor->hfdcan, FDCAN_EXTENDED_ID, EL05_SlotKey(motor));
    EL05_MakeMessage(motor, EL05_CMD_STOP, &message);
    message.data[0] = clear_error ? 1 : 0;
    motor->stop_pending = !CAN_Tx_Submit(&message);
    return !motor->stop_pending;
}

bool EL05_Disable(EL05_t *motor)
{
    return EL05_Stop(motor, false);
}

bool EL05_ClearError(EL05_t *motor)
{
    return EL05_Stop(motor, true);
}

bool EL05_SetZeroAngle(EL05_t *motor)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor) || motor->command_enabled ||
        (motor->mode != EL05_MODE_MOTION && motor->mode != EL05_MODE_POSITION_CSP))
        return false;

    if (!EL05_Stop(motor, false)) return false;
    EL05_MakeMessage(motor, EL05_CMD_ZERO, &message);
    message.data[0] = 1;
    return CAN_Tx_Submit(&message);
}

bool EL05_SetMotion(EL05_t *motor, float angle, float speed,
                    float kp, float kd, float torque)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor) || !motor->command_enabled || motor->mode != EL05_MODE_MOTION ||
        !isfinite(angle) || !isfinite(speed) || !isfinite(kp) || !isfinite(kd) || !isfinite(torque))
        return false;

    EL05_MakeMessage(motor, EL05_CMD_MOTION, &message);
    message.id = ((uint32_t)EL05_CMD_MOTION << 24) |
                 ((uint32_t)EL05_Encode(torque, EL05_MIN_TORQUE, EL05_MAX_TORQUE) << 8) |
                 motor->id;
    EL05_PutBigEndian16(&message.data[0], EL05_Encode(angle, EL05_MIN_ANGLE, EL05_MAX_ANGLE));
    EL05_PutBigEndian16(&message.data[2], EL05_Encode(speed, EL05_MIN_SPEED, EL05_MAX_SPEED));
    EL05_PutBigEndian16(&message.data[4], EL05_Encode(kp, 0.0f, EL05_MAX_KP));
    EL05_PutBigEndian16(&message.data[6], EL05_Encode(kd, 0.0f, EL05_MAX_KD));
    message.slot_key = EL05_SlotKey(motor);
    return CAN_Tx_Perform(&message);
}

bool EL05_SetCurrent(EL05_t *motor, float current)
{
    if (!EL05_IsValid(motor) || motor->mode != EL05_MODE_CURRENT || !isfinite(current))
        return false;
    return EL05_SetFloatTarget(motor, EL05_PARAM_IQ_REF,
                               EL05_Clamp(current, -EL05_MAX_CURRENT, EL05_MAX_CURRENT));
}

bool EL05_SetSpeed(EL05_t *motor, float speed)
{
    if (!EL05_IsValid(motor) || motor->mode != EL05_MODE_SPEED || !isfinite(speed))
        return false;
    return EL05_SetFloatTarget(motor, EL05_PARAM_SPEED_REF,
                               EL05_Clamp(speed, EL05_MIN_SPEED, EL05_MAX_SPEED));
}

bool EL05_SetAngle(EL05_t *motor, float angle)
{
    if (!EL05_IsValid(motor) ||
        (motor->mode != EL05_MODE_POSITION_PP && motor->mode != EL05_MODE_POSITION_CSP))
        return false;
    return EL05_SetFloatTarget(motor, EL05_PARAM_POSITION_REF, angle);
}

bool EL05_ReadParameter(EL05_t *motor, uint16_t index)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor)) return false;
    EL05_MakeParameter(motor, EL05_CMD_READ, index, 0, &message);
    return CAN_Tx_Submit(&message);
}

bool EL05_WriteParameter(EL05_t *motor, uint16_t index, uint32_t value)
{
    Struct_CAN_Tx_Msg message;

    if (!EL05_IsValid(motor)) return false;
    if (index == EL05_PARAM_RUN_MODE)
    {
        if (value > 5) return false;
        return EL05_SetMode(motor, (EL05_Mode_t)value);
    }
    if (index == EL05_PARAM_IQ_REF || index == EL05_PARAM_SPEED_REF ||
        index == EL05_PARAM_POSITION_REF) return false;

    EL05_MakeParameter(motor, EL05_CMD_WRITE, index, value, &message);
    return CAN_Tx_Submit(&message);
}

bool EL05_WriteFloatParameter(EL05_t *motor, uint16_t index, float value)
{
    uint32_t raw;

    if (!isfinite(value) || index == EL05_PARAM_RUN_MODE ||
        index == EL05_PARAM_REPORT_INTERVAL || index == EL05_PARAM_CAN_TIMEOUT ||
        index == EL05_PARAM_ZERO_STATE) return false;
    memcpy(&raw, &value, sizeof(raw));
    return EL05_WriteParameter(motor, index, raw);
}

bool EL05_SetAutoReport(EL05_t *motor, bool enabled)
{
    Struct_CAN_Tx_Msg message;
    uint8_t index;

    if (!EL05_IsValid(motor)) return false;
    EL05_MakeMessage(motor, EL05_CMD_REPORT, &message);
    for (index = 0; index < 6; index++) message.data[index] = (uint8_t)(index + 1);
    message.data[6] = enabled ? 1 : 0;
    return CAN_Tx_Submit(&message);
}

bool EL05_IsOnline(const EL05_t *motor, uint32_t timeout_ms)
{
    uint32_t last_feedback_ms;

    if (!EL05_IsValid(motor) || motor->feedback_count == 0) return false;
    last_feedback_ms = motor->last_feedback_ms;
    return (uint32_t)(HAL_GetTick() - last_feedback_ms) <= timeout_ms;
}

static void EL05_Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                          uint8_t *data, uint32_t len, void *context)
{
    EL05_t *motor = (EL05_t *)context;
    uint8_t command = (uint8_t)(id >> 24);

    if (!EL05_IsValid(motor) || data == NULL || len != 8 || id > 0x1FFFFFFF ||
        hfdcan != motor->hfdcan || (uint8_t)id != motor->host_id ||
        (uint8_t)(id >> 8) != motor->id) return;

    if (command == EL05_CMD_FEEDBACK || command == EL05_CMD_REPORT)
    {
        motor->angle = EL05_Decode(&data[0], EL05_MIN_ANGLE, EL05_MAX_ANGLE);
        motor->speed = EL05_Decode(&data[2], EL05_MIN_SPEED, EL05_MAX_SPEED);
        motor->torque = EL05_Decode(&data[4], EL05_MIN_TORQUE, EL05_MAX_TORQUE);
        motor->temperature = (float)(((uint16_t)data[6] << 8) | data[7]) * 0.1f;
        motor->feedback_fault = (uint8_t)((id >> 16) & 0x3F);
        motor->state = (EL05_State_t)((id >> 22) & 0x03);
        motor->enabled = motor->state == EL05_STATE_RUNNING;
        motor->last_feedback_ms = HAL_GetTick();
        motor->feedback_count++;
    }
    else if (command == EL05_CMD_READ)
    {
        uint16_t index = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        uint32_t raw = EL05_GetLittleEndian32(&data[4]);
        float value;

        motor->parameter_index = index;
        motor->parameter_status = (uint8_t)(id >> 16);
        motor->parameter_value = raw;
        motor->parameter_count++;
        memcpy(&value, &raw, sizeof(value));
        if (motor->parameter_status == 0 && isfinite(value))
        {
            if (index == EL05_PARAM_IQ_FILTERED) motor->current = value;
            if (index == EL05_PARAM_BUS_VOLTAGE) motor->bus_voltage = value;
        }
    }
    else if (command == EL05_CMD_FAULT)
    {
        /* 厂商 RobStride/Python_Sample bus.py 对该帧按 <LL 解码。 */
        motor->fault = EL05_GetLittleEndian32(&data[0]);
        motor->warning = EL05_GetLittleEndian32(&data[4]);
    }
}
