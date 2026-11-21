/** @file sys_health.h @author zzm @brief 健康状态与调试快照接口 */
#ifndef SYS_HEALTH_H
#define SYS_HEALTH_H

#include "dji_motor.h"
#include "dm_motor.h"
#include "qd4310.h"
#include <stdint.h>

#define SYS_HEALTH_PERIOD_MS 50
#define SYS_HEALTH_IMU_TIMEOUT_MS 100
#define SYS_HEALTH_STARTUP_TIMEOUT_MS 1000
#define SYS_HEALTH_RECOVERY_CHECKS 2
#define SYS_HEALTH_ABI_VERSION 3U
#define SYS_HEALTH_MAX_MOTORS (DJI_MOTOR_MAX_MOTORS + DM_MOTOR_MAX_MOTORS + QD4310_MAX_MOTORS)

// 24 V 供电系统的默认欠压阈值，按整车供电配置覆盖。
#ifndef SYS_HEALTH_MIN_POWER_V
#define SYS_HEALTH_MIN_POWER_V 18.0f
#endif

enum Enum_Sys_Health_State
{
    HEALTH_DISABLED = 0,
    HEALTH_STARTING,
    HEALTH_OK,
    HEALTH_WARN,
    HEALTH_OFFLINE,
    HEALTH_FAULT,
    HEALTH_UNINIT,
};

enum Enum_Sys_Health_Reason
{
    HEALTH_REASON_NONE = 0,
    HEALTH_REASON_NO_DATA = 1 << 0,
    HEALTH_REASON_STALE = 1 << 1,
    HEALTH_REASON_SPI = 1 << 2,
    HEALTH_REASON_RECOVERY = 1 << 3,
    HEALTH_REASON_SAMPLE_LOSS = 1 << 4,
    HEALTH_REASON_INVALID_SAMPLE = 1 << 5,
    HEALTH_REASON_TIMESTAMP = 1 << 6,
    HEALTH_REASON_FILTER_RESET = 1 << 7,
    HEALTH_REASON_RECOVERING = 1 << 8,
    HEALTH_REASON_ADC = 1 << 9,
    HEALTH_REASON_UNDERVOLTAGE = 1 << 10,
    HEALTH_REASON_MOTOR_FAULT = 1 << 11,
    HEALTH_REASON_INIT_FAILED = 1 << 12,
};

enum Enum_Sys_Health_Motor_Type
{
    HEALTH_MOTOR_NONE = 0,
    HEALTH_MOTOR_DJI,
    HEALTH_MOTOR_DM,
    HEALTH_MOTOR_QD4310,
};

struct Struct_Sys_Health_Item
{
    Enum_Sys_Health_State State;
    uint32_t Reason;
    uint32_t Data_Age_ms;
};

struct Struct_Sys_Health_Motor
{
    Struct_Sys_Health_Item Health;
    FDCAN_HandleTypeDef *Bus;
    uint32_t Feedback_Id;
    bool Enabled;
    Enum_Sys_Health_Motor_Type Type;
};

struct Struct_Sys_Health
{
    uint32_t Sequence;
    uint16_t ABI_Version;
    uint16_t ABI_Size;
    uint64_t Updated_At_us;
    uint32_t Update_Count;
    uint32_t Init_Errors;
    Enum_Sys_Health_State Overall_State;
    uint8_t Safe_To_Run;
    // 启动等待首帧的电机只计入 Motor_Count。
    uint8_t Online_Motor_Count;
    uint8_t Offline_Motor_Count;
    uint32_t Motor_Count;
    float Power_Voltage_V;
    Struct_Sys_Health_Item IMU;
    Struct_Sys_Health_Item Power;
    Struct_Sys_Health_Motor Motor[SYS_HEALTH_MAX_MOTORS];
    Struct_Sys_Health_Item Flash;
    uint32_t Sequence_End;
};

extern "C" volatile Struct_Sys_Health SYS_Health;
extern "C" volatile uint8_t SYS_Health_Buzzer_Mute;

/** @brief 启动时初始化；Init/Update 仅限健康任务调用。 @author zzm */
void Sys_Health_Init(void);
void Sys_Health_Update(void);
/** @brief 任务上下文获取一致副本；包含短暂关中断复制。 @author zzm */
bool Sys_Health_GetSnapshot(Struct_Sys_Health *snapshot);
bool Sys_Health_Is_Safe(void);
Enum_Sys_Health_State Sys_Health_Get_Overall_State(void);
void Sys_Health_Set_Buzzer_Mute(bool mute);

#endif
