/** @file sys_health.h @author zzm @brief 健康状态与调试快照接口 */
#ifndef SYS_HEALTH_H
#define SYS_HEALTH_H

#include "fdcan.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SYS_HEALTH_PERIOD_MS 50
#define SYS_HEALTH_STALE_TIMEOUT_US (6ULL * SYS_HEALTH_PERIOD_MS * 1000ULL)
#define SYS_HEALTH_IMU_TIMEOUT_MS 100
#define SYS_HEALTH_STARTUP_TIMEOUT_MS 1000
#define SYS_HEALTH_RECOVERY_CHECKS 2
#define SYS_HEALTH_ABI_VERSION 4

#ifndef SYS_HEALTH_MAX_MOTORS
#define SYS_HEALTH_MAX_MOTORS 40
#endif

// 24 V 供电系统的默认欠压阈值，按整车供电配置覆盖。
#ifndef SYS_HEALTH_MIN_POWER_V
#define SYS_HEALTH_MIN_POWER_V 18.0f
#endif

typedef enum Enum_Sys_Health_State
{
    HEALTH_DISABLED = 0,
    HEALTH_STARTING,
    HEALTH_OK,
    HEALTH_WARN,
    HEALTH_OFFLINE,
    HEALTH_FAULT,
    HEALTH_UNINIT,
} Enum_Sys_Health_State;

typedef enum Enum_Sys_Health_Reason
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
} Enum_Sys_Health_Reason;

typedef struct Struct_Sys_Health_Item
{
    Enum_Sys_Health_State State;
    uint32_t Reason;
    uint32_t Data_Age_ms; ///< 龄期未知或超过范围时取 UINT32_MAX。
} Struct_Sys_Health_Item;

/**
 * @brief 单个已注册电机/执行器的健康采样快照，由驱动探针回调填充。
 * @note Name 应指向生命周期覆盖运行期的静态字符串（如 "DJI"、"DM"、"QD4310"）。
 */
typedef struct Struct_Sys_Health_Motor_Sample
{
    const char *Name;
    FDCAN_HandleTypeDef *Bus;
    uint32_t Feedback_Id;
    uint64_t Last_Feedback_us;
    uint64_t Feedback_Timeout_us;
    bool Feedback_Received;
    bool Enabled;
    bool Drive_Fault;
} Struct_Sys_Health_Motor_Sample;

/**
 * @brief 电机健康探针回调。
 * @param context 注册时传入的实例上下文。
 * @param sample 输出采样结构体。
 * @return true 采样有效并计入整车健康；false 实例尚未就绪，跳过本周期。
 * @note 回调在 StatusTask 上下文执行，内部不应阻塞或修改电机控制指令。
 */
typedef bool (*Sys_Health_Motor_Probe_t)(void *context, Struct_Sys_Health_Motor_Sample *sample);

typedef struct Struct_Sys_Health_Motor
{
    Struct_Sys_Health_Item Health;
    const char *Name;
    FDCAN_HandleTypeDef *Bus;
    uint32_t Feedback_Id;
    bool Enabled;
} Struct_Sys_Health_Motor;

/**
 * @brief 整车健康快照。
 * @note 异步读取时核对 ABI 与首尾序号，并在读完后复查 Sequence，接受序号一致的偶数版本。
 * @author zzm
 */
typedef struct Struct_Sys_Health
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
} Struct_Sys_Health;

extern volatile Struct_Sys_Health SYS_Health;
extern volatile uint8_t SYS_Health_Buzzer_Mute;

/** @brief 启动时初始化；Init/Update 仅限健康任务调用，不影响已注册的电机探针。 @author zzm */
void Sys_Health_Init(void);
void Sys_Health_Update(void);
/** @brief 注册电机健康探针；参数无效、槽满或重复注册返回 false。 @author zzm */
bool Sys_Health_Register_Motor(Sys_Health_Motor_Probe_t probe, void *context);
/** @brief 注销电机健康探针；未找到匹配项返回 false。 @author zzm */
bool Sys_Health_Unregister_Motor(Sys_Health_Motor_Probe_t probe, void *context);
/** @brief 任务上下文获取一致副本；包含短暂关中断复制。 @author zzm */
bool Sys_Health_GetSnapshot(Struct_Sys_Health *snapshot);
/** @brief 读取运动许可，快照超时或更新时间异常时返回 false。 @author zzm */
bool Sys_Health_Is_Safe(void);
Enum_Sys_Health_State Sys_Health_Get_Overall_State(void);
void Sys_Health_Set_Buzzer_Mute(bool mute);

#ifdef __cplusplus
}
#endif

#endif
