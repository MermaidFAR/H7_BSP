/**
 * @file    qd4310.c
 * @brief   QD4310 FOC 伺服电机驱动实现
 * @details 基于 QDrive 协议, 通过 CAN 总线发送 7 种控制命令 (使能/失能/电流/转速/角度/低速/步进角度),
 *          并解析电机反馈帧获取实时状态 (使能位/电流/转速/角度)。
 *
 *          通信架构:
 *          - MCU → 电机: CAN ID = 0x400 + motor_id, DLC = 3 (cmd + value_lo + value_hi)
 *          - 电机 → MCU: CAN ID = 0x500 + motor_id, DLC = 8 (enable + reserved + current + speed + angle)
 *          - 发送路径: 连续控制命令更新周期槽, 动作命令插入队列, 由 CanTxTask 统一发送
 *          - 接收路径: HAL FDCAN 中断 → BSP_CAN_RegisterCallback 分发 → QDrive_Callback → QD4310_Update
 *
 * @author  zzm
 * @date    2026-06-07
 * @version v1.0
 * @note    使用前需调用 QD4310_Init 初始化电机实例并注册 CAN 反馈回调
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#include "qd4310.h"
#include "sys_critical.h"
#include "sys_timestamp.h"

#include <math.h>
#include <string.h>

static QD4310_t *QD4310_Registrations[QD4310_MAX_MOTORS];
static void QD4310_DecodeFeedback(QD4310_t *motor, const uint8_t feedback[8]);

/**
 * @brief  QD4310 CAN 反馈帧统一回调入口
 * @param  hfdcan 接收帧的 FDCAN 句柄
 * @param  id      接收到的 CAN ID
 * @param  data    反馈数据缓冲区
 * @param  len     数据长度，单位为字节
 * @param  context 注册回调时绑定的电机实例
 * @note   通过 BSP_CAN_RegisterCallback 为每个反馈 ID（0x500 + motor_id）注册。
 */
static void QDrive_Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len, void *context)
{
    QD4310_t *motor = (QD4310_t *)context;
    if (len >= 8 && motor->hfdcan == hfdcan && id == 0x500U + motor->id)
    {
        QD4310_DecodeFeedback(motor, data);
    }
}

/**
 * @brief  初始化 QD4310 电机实例
 * @param  motor  电机结构体指针
 * @param  id     电机 ID (0~7), 决定 CAN 命令 ID (0x400+id) 和反馈 ID (0x500+id)
 * @param  hfdcan FDCAN 句柄指针
 * @return true 已初始化并注册反馈回调；false 参数无效或注册失败
 */
bool QD4310_Init(QD4310_t *motor, uint8_t id, FDCAN_HandleTypeDef *hfdcan)
{
    if (motor == NULL || id > 7)
    {
        return false;
    }

    // 注册成功后再设置实例，并在完整初始化前阻止接收中断访问。
    const uint32_t primask = SYS_Enter_Critical();
    uint8_t index = 0U;
    while (index < QD4310_MAX_MOTORS &&
           QD4310_Registrations[index] != NULL && QD4310_Registrations[index] != motor)
    {
        index++;
    }
    const bool registered = index < QD4310_MAX_MOTORS &&
                            BSP_CAN_RegisterCallback(id + 0x500, hfdcan, QDrive_Callback, motor);
    if (registered)
    {
        memset(motor, 0, sizeof(*motor));
        motor->id = id;
        motor->hfdcan = hfdcan;
        QD4310_Registrations[index] = motor;
    }
    SYS_Exit_Critical(primask);
    return registered;
}

bool QD4310_GetHealth(uint8_t index, QD4310_Health_t *snapshot)
{
    if (snapshot == NULL || index >= QD4310_MAX_MOTORS)
    {
        return false;
    }

    const uint32_t primask = SYS_Enter_Critical();
    const QD4310_t *motor = QD4310_Registrations[index];
    if (motor == NULL)
    {
        SYS_Exit_Critical(primask);
        return false;
    }
    snapshot->hfdcan = motor->hfdcan;
    snapshot->rx_id = 0x500U + motor->id;
    snapshot->last_feedback_us = motor->last_feedback_us;
    snapshot->feedback_timeout_us = QD4310_FEEDBACK_TIMEOUT_MS * 1000ULL;
    snapshot->feedback_received = motor->feedback_received;
    snapshot->enabled = motor->enabled;
    SYS_Exit_Critical(primask);
    return true;
}

/**
 * @brief  数值限幅 (Clamp)
 * @param  value 输入值
 * @param  min   下限
 * @param  max   上限
 * @return float 限幅后的值
 */
static float QD4310_Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief  向电机发送控制命令
 * @param  motor 电机实例指针
 * @param  cmd   命令类型
 * @param  value 命令参数值 (int16_t)
 * @note   电流、速度、角度和低速命令更新周期槽, 其余动作命令进入插入队列
 */
bool QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value) {
    if (motor == NULL || motor->id > 7)
    {
        return false;
    }
    Struct_CAN_Tx_Msg message = {NULL, 0U, {0U}, 0U};

    message.hfdcan = motor->hfdcan;
    message.id = motor->id + 0x400;
    message.len = 3;
    message.data[0] = (uint8_t)cmd;
    const uint16_t bits = (uint16_t)value;
    message.data[1] = (uint8_t)bits;
    message.data[2] = (uint8_t)(bits >> 8);

    if (cmd == QD4310_CMD_CURRENT ||
        cmd == QD4310_CMD_SPEED ||
        cmd == QD4310_CMD_ANGLE ||
        cmd == QD4310_CMD_LOW_SPEED)
    {
        return CAN_Tx_Perform(&message);
    }

    if (cmd == QD4310_CMD_ENABLE || cmd == QD4310_CMD_DISABLE || cmd == QD4310_CMD_RESET)
    {
        BSP_CAN_DiscardPendingSlot(message.hfdcan, message.id);
    }
    if (cmd == QD4310_CMD_ENABLE || cmd == QD4310_CMD_STEP_ANGLE)
    {
        return CAN_Tx_SubmitMotion(&message);
    }
    return CAN_Tx_Submit(&message);
}

/**
 * @brief  解析电机反馈帧并更新状态
 * @param  motor    电机实例指针
 * @param  feedback 反馈数据数组 (8 字节)
 * @note   反馈帧格式:
 *         - [0]: bit0 = enable 状态
 *         - [2-3]: 电流原始值 (int16_t, little-endian), 缩放因子 10A / INT16_MAX
 *         - [4-5]: 转速原始值 (int16_t, little-endian), 缩放因子 1000rpm / INT16_MAX
 *         - [6-7]: 角度原始值 (uint16_t, little-endian), 缩放因子 2π / UINT16_MAX
 */
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8])
{
    if (motor != NULL && feedback != NULL)
    {
        QD4310_DecodeFeedback(motor, feedback);
    }
}

static void QD4310_DecodeFeedback(QD4310_t *motor, const uint8_t feedback[8])
{
    int16_t current_raw = (int16_t)((feedback[3] << 8) | feedback[2]);
    motor->current = (float)current_raw * 10.0f / INT16_MAX;

    int16_t speed_raw = (int16_t)((feedback[5] << 8) | feedback[4]);
    motor->speed = (float)speed_raw * 1000.0f / 32767.0f;

    uint16_t angle_raw = (uint16_t)((feedback[7] << 8) | feedback[6]);
    motor->angle = (float)angle_raw * QD4310_TWO_PI / UINT16_MAX;

    const uint32_t primask = SYS_Enter_Critical();
    motor->enabled = (feedback[0] & 0x01U) != 0U;
    motor->last_feedback_us = SYS_Timestamp_Get_Microsecond();
    motor->feedback_received = true;
    SYS_Exit_Critical(primask);
}

/**
 * @brief  使能电机驱动输出
 * @param  motor 电机实例指针
 */
bool QD4310_Enable(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0x0000);
}

/**
 * @brief  失能电机驱动输出
 * @param  motor 电机实例指针
 */
bool QD4310_Disable(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0x0000);
}

/**
 * @brief  设置电机绝对角度 (位置模式)
 * @param  motor 电机实例指针
 * @param  angle 目标角度, 范围 [0, 2π] rad
 */
bool QD4310_SetAngle(QD4310_t *motor, float angle) {
    if (!isfinite(angle))
    {
        return false;
    }
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    const uint16_t angle_bits = (uint16_t)(angle / QD4310_TWO_PI * UINT16_MAX);
    int16_t angle_value;
    memcpy(&angle_value, &angle_bits, sizeof(angle_value));
    return QD4310_SendCommand(motor, QD4310_CMD_ANGLE, angle_value);
}

/**
 * @brief  设置电机相对步进角度
 * @param  motor      电机实例指针
 * @param  step_angle 步进角度, 范围 [-2π, 2π] rad
 */
bool QD4310_SetStepAngle(QD4310_t *motor, float step_angle) {
    if (!isfinite(step_angle))
    {
        return false;
    }
    step_angle = QD4310_Clamp(step_angle, QD4310_MIN_STEPANGLE, QD4310_MAX_STEPANGLE);
    int16_t step_angle_value = (int16_t)(step_angle / QD4310_MAX_STEPANGLE * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_STEP_ANGLE, step_angle_value);
}

/**
 * @brief  设置电机目标转速 (速度模式)
 * @param  motor 电机实例指针
 * @param  speed 目标转速, 范围 [-1000, 1000] rpm
 */
bool QD4310_SetSpeed(QD4310_t *motor, float speed) {
    if (!isfinite(speed))
    {
        return false;
    }
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_SPEED, speed_value);
}

/**
 * @brief  设置电机低速模式目标转速
 * @param  motor 电机实例指针
 * @param  speed 目标转速, 范围 [-1000, 1000] rpm
 */
bool QD4310_SetLowSpeed(QD4310_t *motor, float speed) {
    if (!isfinite(speed))
    {
        return false;
    }
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED, speed_value);
}

/**
 * @brief  设置电机目标电流 (力矩模式)
 * @param  motor   电机实例指针
 * @param  current 目标电流, 范围 [-10, 10] A
 */
bool QD4310_SetCurrent(QD4310_t *motor, float current) {
    if (!isfinite(current))
    {
        return false;
    }
    current = QD4310_Clamp(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t current_value = (int16_t)(current / QD4310_MAX_CURRENT * INT16_MAX);
    return QD4310_SendCommand(motor, QD4310_CMD_CURRENT, current_value);
}

/**
 * @brief  设置电机0点角度 (校准模式)
 * @param  motor 电机实例指针
 *
 */
bool QD4310_SetZeroAngle(QD4310_t *motor) {
    return QD4310_SendCommand(motor, QD4310_CMD_ZERO_ANGLE, 0x0000);
}
