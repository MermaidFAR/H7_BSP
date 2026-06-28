/**
 * @file    QD4310.c
 * @brief   QD4310 FOC 伺服电机驱动实现
 * @details 基于 QDrive 协议, 通过 CAN 总线发送 7 种控制命令 (使能/失能/电流/转速/角度/低速/步进角度),
 *          并解析电机反馈帧获取实时状态 (使能位/电流/转速/角度)。
 *
 *          通信架构:
 *          - MCU → 电机: CAN ID = 0x400 + motor_id, DLC = 3 (cmd + value_lo + value_hi)
 *          - 电机 → MCU: CAN ID = 0x500 + motor_id, DLC = 8 (enable + reserved + current + speed + angle)
 *          - 发送路径: QD4310_SendCommand → CAN_Tx_Perform (写入 Tx_Msg_Buffer) → CanTxTask 1kHz 周期发送
 *          - 接收路径: HAL FDCAN 中断 → BSP_CAN_RegisterCallback 分发 → QDrive_Callback → QD4310_Update
 *
 * @author  zzm
 * @date    2026-06-07
 * @version v1.0
 * @note    使用前需调用 QD4310_Init 初始化电机实例并注册 CAN 反馈回调
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#include "QD4310.h"
#include "sys_timestamp.h"
#include <string.h>

/**
 * @brief  全局电机实例指针数组
 * @note   CAN 反馈回调通过 motor_id 索引查找对应电机实例
 */
QD4310_t* g_QD4310Instances[MOTOR_IDX] = {0};

/**
 * @brief  全局 CAN 发送消息缓冲
 * @note   所有 QD4310 实例共用, 由 QD4310_SendCommand 填充后通过 CAN_Tx_Perform 写入 Tx_Msg_Buffer
 */
Struct_CAN_Tx_Msg Q_msg = {
    .hfdcan = NULL,
    .id = 0x000,
    .data = {0},
    .len = 0
};

/**
 * @brief  QD4310 CAN 反馈帧统一回调入口
 * @param  hcan FDCAN 句柄
 * @param  id   接收到的 CAN ID
 * @param  data 反馈数据 (8 字节)
 * @param  len  数据长度
 * @note   通过 BSP_CAN_RegisterCallback 为每个电机 ID (0x500 + motor_id) 注册,
 *         内部按 id 低 8 位查找 g_QD4310Instances 并调用 QD4310_Update
 */
static void QDrive_Callback(FDCAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data, uint32_t len)
{
    if (id >= 0x500 && id <= 0x508) {
        uint8_t motor_id = id & 0xFF;
        if (motor_id < MOTOR_IDX && g_QD4310Instances[motor_id] != NULL) {
            QD4310_Update(g_QD4310Instances[motor_id], data);
        }
    }
}

/**
 * @brief  初始化 QD4310 电机实例
 * @param  motor  电机结构体指针
 * @param  id     电机 ID (0~7), 决定 CAN 命令 ID (0x400+id) 和反馈 ID (0x500+id)
 * @param  hfdcan FDCAN 句柄指针
 * @note   调用后自动注册 CAN 反馈回调, 同时将实例写入全局数组供回调查找
 */
void QD4310_Init(QD4310_t *motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan) {
    motor->enabled = false;
    motor->id = id;
    motor->speed = 0.0f;
    motor->angle = 0.0f;
    motor->current = 0.0f;
    motor->hfdcan = hfdcan;
    g_QD4310Instances[motor->id] = motor;
    BSP_CAN_RegisterCallback(motor->id + 0x500, QDrive_Callback);
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
 * @note   通过 CAN_Tx_Perform 写入全局 Tx_Msg_Buffer, 由 CanTxTask 在 1ms 周期内批量发出;
 *         通过 timestamp[0] 标记避免重复发送旧数据
 */
void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value) {
    static uint8_t TxBuffer[3];
    TxBuffer[0] = (uint8_t)cmd;
    TxBuffer[1] = (uint8_t)(value & 0xFF);
    TxBuffer[2] = (uint8_t)((value >> 8) & 0xFF);

    Q_msg.hfdcan = motor->hfdcan;
    Q_msg.id = motor->id + 0x400;
    Q_msg.len = 3;
    memcpy(Q_msg.data, TxBuffer, sizeof(TxBuffer));

    Q_msg.timestamp[0] = SYS_Timestamp_Get_Microsecond();
    CAN_Tx_Perform(&Q_msg);
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
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]) {
    motor->enabled = feedback[0] & 0x01;

    int16_t current_raw = (int16_t)((feedback[3] << 8) | feedback[2]);
    motor->current = (float)current_raw * 10.0f / INT16_MAX;

    int16_t speed_raw = (int16_t)((feedback[5] << 8) | feedback[4]);
    motor->speed = (float)speed_raw * 1000.0f / 32767.0f;

    uint16_t angle_raw = (uint16_t)((feedback[7] << 8) | feedback[6]);
    motor->angle = (float)angle_raw * QD4310_TWO_PI / UINT16_MAX;
}

/**
 * @brief  使能电机驱动输出
 * @param  motor 电机实例指针
 */
void QD4310_Enable(QD4310_t *motor) {
    QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0x0000);
}

/**
 * @brief  失能电机驱动输出
 * @param  motor 电机实例指针
 */
void QD4310_Disable(QD4310_t *motor) {
    QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0x0000);
}

/**
 * @brief  设置电机绝对角度 (位置模式)
 * @param  motor 电机实例指针
 * @param  angle 目标角度, 范围 [0, 2π] rad
 */
void QD4310_SetAngle(QD4310_t *motor, float angle) {
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    int16_t angle_value = (int16_t)(angle / QD4310_TWO_PI * UINT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_ANGLE, angle_value);
}

/**
 * @brief  设置电机相对步进角度
 * @param  motor      电机实例指针
 * @param  step_angle 步进角度, 范围 [-2π, 2π] rad
 */
void QD4310_SetStepAngle(QD4310_t *motor, float step_angle) {
    step_angle = QD4310_Clamp(step_angle, QD4310_MIN_STEPANGLE, QD4310_MAX_STEPANGLE);
    int16_t step_angle_value = (int16_t)(step_angle / QD4310_MAX_STEPANGLE * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_STEP_ANGLE, step_angle_value);
}

/**
 * @brief  设置电机目标转速 (速度模式)
 * @param  motor 电机实例指针
 * @param  speed 目标转速, 范围 [-1000, 1000] rpm
 */
void QD4310_SetSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_SPEED, speed_value);
}

/**
 * @brief  设置电机低速模式目标转速
 * @param  motor 电机实例指针
 * @param  speed 目标转速, 范围 [-1000, 1000] rpm
 */
void QD4310_SetLowSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED, speed_value);
}

/**
 * @brief  设置电机目标电流 (力矩模式)
 * @param  motor   电机实例指针
 * @param  current 目标电流, 范围 [-10, 10] A
 */
void QD4310_SetCurrent(QD4310_t *motor, float current) {
    current = QD4310_Clamp(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t current_value = (int16_t)(current / QD4310_MAX_CURRENT * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_CURRENT, current_value);
}
