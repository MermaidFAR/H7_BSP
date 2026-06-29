//
// Created by jason on 25-10-16.
//

#ifndef QD4310_H
#define QD4310_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#define MOTOR_IDX (16) // 最大支持16个电机实例

// 命令枚举
typedef enum
{
    QD4310_CMD_NOP = 0x00,
    QD4310_CMD_ENABLE = 0x01,
    QD4310_CMD_DISABLE = 0x02,
    QD4310_CMD_CURRENT = 0x03,
    QD4310_CMD_SPEED = 0x04,
    QD4310_CMD_ANGLE = 0x05,
    QD4310_CMD_LOW_SPEED = 0x06,
    QD4310_CMD_STEP_ANGLE = 0x07,
    QD4310_CMD_ZERO_ANGLE = 0xFE,
    QD4310_CMD_RESET = 0xFF,
    QD4310_CMD_CLEAR_ERROR = 0xFB
    
} QD4310_Command_t;

// QD4310电机结构体
typedef struct {
    bool enabled;
    uint8_t id;
    float speed;   // 转速，单位rpm
    float angle;   // 角度，单位弧度
    float current; // 电流，单位A
    FDCAN_HandleTypeDef* hfdcan;
} QD4310_t;

// 函数声明
/**
 * @brief 使能电机
 * @param motor 电机结构体指针
 */
void QD4310_Enable(QD4310_t *motor);
/**
 * @brief 失能电机
 * @param motor 电机结构体指针
 */
void QD4310_Disable(QD4310_t *motor);
/**
 * @brief 更新电机状态
 * @param motor 电机结构体指针
 * @param feedback 来自电机的反馈数据数组
 */
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]);
/**
 * @brief 设置电机角度
 * @param motor 电机结构体指针
 * @param angle 设置的角度,[0,2pi]
 */
void QD4310_SetAngle(QD4310_t *motor, float angle);
/**
 * @brief 设置电机步进角度
 * @param motor 电机结构体指针
 * @param step_angle 设置的角度,[-2pi,2pi]
 */
void QD4310_SetStepAngle(QD4310_t *motor, float step_angle);
/**
 * @brief 设置电机转速
 * @param motor 电机结构体指针
 * @param speed 设置的转速,[-1000,1000]
 */
void QD4310_SetSpeed(QD4310_t *motor, float speed);
/**
 * @brief 设置电机转速
 * @param motor 电机结构体指针
 * @param speed 设置的转速,[-1000,1000]
 */
void QD4310_SetLowSpeed(QD4310_t *motor, float speed);
/**
 * @brief 设置电机电流
 * @param motor 电机结构体指针
 * @param current 设置的转速,[-10,10]
 */
void QD4310_SetCurrent(QD4310_t *motor, float current);

void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value);

void QD4310_Init(QD4310_t* motor, uint8_t id, FDCAN_HandleTypeDef* hfdcan);

// 数学常数定义
#define QD4310_PI (3.14159265358979323846f)
#define QD4310_TWO_PI (2.0f * QD4310_PI)

// 限制值宏定义
#define QD4310_MAX_SPEED (1000.0f)
#define QD4310_MIN_SPEED (-1000.0f)
#define QD4310_MAX_CURRENT (10.0f)
#define QD4310_MIN_CURRENT (-10.0f)
#define QD4310_MAX_STEPANGLE (QD4310_TWO_PI)
#define QD4310_MIN_STEPANGLE (-QD4310_TWO_PI)

#ifdef __cplusplus
}
#endif

#endif //QD4310_H
