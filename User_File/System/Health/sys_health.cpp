/** @file sys_health.cpp @author zzm @brief 低频设备健康采集与状态发布 */
#include "sys_health.h"
#include "sys_critical.h"
#include "bsp_bmi088.h"
#include "bsp_power.h"
#include "bsp_w25q64jv.h"
#include "Init.h"
#include "sys_debug.h"
#include "sys_timestamp.h"
#include <stddef.h>
#include <string.h>

extern "C" {
volatile Struct_Sys_Health SYS_Health __attribute__((used)) = {};
volatile uint8_t SYS_Health_Buzzer_Mute __attribute__((used)) = 0U;
}

struct Health_Tracking
{
    uint64_t First_Seen_us;
    Struct_Sys_Health_Item Item;
    uint8_t Recovery_Count;
};

struct Health_Motor_Tracking
{
    Health_Tracking Link;
    FDCAN_HandleTypeDef *Bus;
    uint32_t Feedback_Id;
    bool Feedback_Received;
};

static Struct_Sys_Health Health_Next;
static Health_Tracking Health_IMU;
static Health_Tracking Health_Power;
static Health_Motor_Tracking Health_Motors[SYS_HEALTH_MAX_MOTORS];
static uint32_t Previous_IMU_Counters[9];
static bool Previous_IMU_Valid;

static uint32_t Health_Age_ms(uint64_t now, uint64_t timestamp)
{
    uint64_t age = now >= timestamp ? (now - timestamp) / 1000 : 0;
    return age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
}

static void Health_Publish()
{
    Health_Next.Updated_At_us = SYS_Timestamp.Get_Now_Microsecond();
    uint32_t interrupt_state = SYS_Enter_Critical();
    uint32_t sequence = (SYS_Health.Sequence + 1) | 1;
    SYS_Health.Sequence = sequence;
    SYS_Health.Sequence_End = sequence;
    __DMB();
    const size_t begin = offsetof(Struct_Sys_Health, ABI_Version);
    const size_t end = offsetof(Struct_Sys_Health, Sequence_End);
    memcpy((uint8_t *)(void *)&SYS_Health + begin, (const uint8_t *)&Health_Next + begin, end - begin);
    __DMB();
    SYS_Health.Sequence_End = sequence + 1;
    __DMB();
    SYS_Health.Sequence = sequence + 1;
    SYS_Exit_Critical(interrupt_state);
}

static void Health_Set_State(Health_Tracking *tracking, Enum_Sys_Health_State state,
                             uint32_t reason)
{
    if (state == HEALTH_OK && tracking->Item.State >= HEALTH_WARN)
    {
        if (++tracking->Recovery_Count < SYS_HEALTH_RECOVERY_CHECKS)
        {
            tracking->Item.Reason |= HEALTH_REASON_RECOVERING;
            return;
        }
    }
    tracking->Recovery_Count = 0U;
    tracking->Item.State = state;
    tracking->Item.Reason = reason;
}

static Enum_Sys_Health_State Health_Data_State(Health_Tracking *tracking, bool received,
                                              uint64_t timestamp_us, uint64_t timeout_us,
                                              uint64_t now_us, uint32_t *reason)
{
    if (tracking->Item.State == HEALTH_DISABLED)
    {
        tracking->First_Seen_us = now_us;
    }
    if (!received)
    {
        *reason |= HEALTH_REASON_NO_DATA;
        return now_us - tracking->First_Seen_us >= SYS_HEALTH_STARTUP_TIMEOUT_MS * 1000ULL
                   ? HEALTH_OFFLINE : HEALTH_STARTING;
    }
    if (timestamp_us > now_us)
    {
        *reason |= HEALTH_REASON_TIMESTAMP;
        return HEALTH_FAULT;
    }
    if (now_us - timestamp_us > timeout_us)
    {
        *reason |= HEALTH_REASON_STALE;
        return HEALTH_OFFLINE;
    }
    return *reason != 0U ? HEALTH_WARN : HEALTH_OK;
}

static Enum_Sys_Health_State Health_Worse_State(Enum_Sys_Health_State first,
                                               Enum_Sys_Health_State second)
{
    if (first == HEALTH_FAULT || second == HEALTH_FAULT)
    {
        return HEALTH_FAULT;
    }
    if (first == HEALTH_OFFLINE || second == HEALTH_OFFLINE)
    {
        return HEALTH_OFFLINE;
    }
    if (first == HEALTH_UNINIT || second == HEALTH_UNINIT)
    {
        return HEALTH_UNINIT;
    }
    if (first == HEALTH_STARTING || second == HEALTH_STARTING)
    {
        return HEALTH_STARTING;
    }
    return first > second ? first : second;
}

static Struct_Sys_Health_Item Health_Init_Item(uint32_t error_bit, bool initialized,
                                              Enum_Sys_Health_State ready_state)
{
    if ((Health_Next.Init_Errors & error_bit) != 0U)
    {
        return {HEALTH_FAULT, HEALTH_REASON_INIT_FAILED, UINT32_MAX};
    }
    return {initialized ? ready_state : HEALTH_UNINIT, HEALTH_REASON_NONE, UINT32_MAX};
}

static void Health_Update_Board_Init(void)
{
    Health_Next.Init_Errors = System_Get_Init_Errors();
    Health_Next.IMU = Health_Init_Item(SYSTEM_INIT_ERROR_BMI088, BSP_BMI088.Is_Initialized(), HEALTH_STARTING);
    Health_Next.Power = Health_Init_Item(SYSTEM_INIT_ERROR_ADC1, BSP_Power.Is_Initialized(), HEALTH_STARTING);
    Health_Next.Flash = Health_Init_Item(SYSTEM_INIT_ERROR_FLASH, BSP_W25Q64JV.Is_Initialized(), HEALTH_OK);
}

static Enum_Sys_Health_State Health_Board_State(void)
{
    if (Health_Next.Init_Errors != 0U)
    {
        return HEALTH_FAULT;
    }
    return Health_Worse_State(Health_Worse_State(Health_Next.IMU.State, Health_Next.Power.State),
                              Health_Next.Flash.State);
}

static uint32_t Health_IMU_Reasons(const Sys_Debug_IMU_Data_t *imu)
{
    const uint32_t counters[] = {
        imu->SPI2_Error_Counter, imu->BMI088_Transfer_Timeout_Counter,
        imu->BMI088_SPI_Recovery_Counter, imu->Gyro_Queue_Drop_Count,
        imu->Gyro_FIFO_Overrun_Count, imu->Gyro_Outlier_Counter,
        imu->Accel_Update_Rejected_Counter, imu->Timestamp_Anomaly_Counter,
        imu->VQF_Reset_Counter,
    };
    const uint32_t reasons[] = {
        HEALTH_REASON_SPI, HEALTH_REASON_SPI, HEALTH_REASON_RECOVERY,
        HEALTH_REASON_SAMPLE_LOSS, HEALTH_REASON_SAMPLE_LOSS,
        HEALTH_REASON_INVALID_SAMPLE, HEALTH_REASON_INVALID_SAMPLE,
        HEALTH_REASON_TIMESTAMP, HEALTH_REASON_FILTER_RESET,
    };
    uint32_t reason = 0;
    for (uint32_t i = 0; i < sizeof(counters) / sizeof(counters[0]); ++i)
    {
        if (Previous_IMU_Valid && counters[i] != Previous_IMU_Counters[i])
            reason |= reasons[i];
        Previous_IMU_Counters[i] = counters[i];
    }
    Previous_IMU_Valid = true;
    return reason;
}

static void Health_Update_IMU(void)
{
    if (Health_Next.IMU.State != HEALTH_STARTING)
    {
        Health_IMU = {};
        Previous_IMU_Valid = false;
        return;
    }

    Sys_Debug_IMU_Data_t imu;
    const uint32_t primask = SYS_Enter_Critical();
    memcpy(&imu, (const void *)&Debug_IMU_Data, sizeof(imu));
    const uint64_t sample_us = BSP_BMI088.Get_Last_Sample_Timestamp_Us();
    SYS_Exit_Critical(primask);

    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    const bool received = sample_us != 0U && imu.Debug_ABI_Size == sizeof(imu) &&
                          imu.sequence == imu.sequence_end && (imu.sequence & 1U) == 0U;
    const uint64_t timestamp_us = sample_us < imu.timestamp_us ? sample_us : imu.timestamp_us;
    Health_IMU.Item.Data_Age_ms = received ? Health_Age_ms(now_us, timestamp_us) : UINT32_MAX;
    uint32_t reason = received ? Health_IMU_Reasons(&imu) : 0U;
    Enum_Sys_Health_State state = Health_Data_State(&Health_IMU, received, timestamp_us,
        SYS_HEALTH_IMU_TIMEOUT_MS * 1000ULL, now_us, &reason);
    if (received && (sample_us > now_us || imu.timestamp_us > now_us))
    {
        state = HEALTH_FAULT;
        reason |= HEALTH_REASON_TIMESTAMP;
    }
    Health_Set_State(&Health_IMU, state, reason);
    Health_Next.IMU = Health_IMU.Item;
}

static void Health_Update_Power(void)
{
    if (Health_Next.Power.State != HEALTH_STARTING)
    {
        Health_Power = {};
        Health_Next.Power_Voltage_V = 0.0f;
        return;
    }

    uint64_t sample_us;
    const bool valid = BSP_Power.Get_Power_Voltage(&Health_Next.Power_Voltage_V, &sample_us);
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    Health_Power.Item.Data_Age_ms = valid ? Health_Age_ms(now_us, sample_us) : UINT32_MAX;

    uint32_t reason = HEALTH_REASON_NONE;
    Enum_Sys_Health_State state = HEALTH_OK;
    if (valid)
    {
        if (Health_Next.Power_Voltage_V < SYS_HEALTH_MIN_POWER_V)
        {
            reason |= HEALTH_REASON_UNDERVOLTAGE;
            state = HEALTH_FAULT;
        }
    }
    else
    {
        reason = HEALTH_REASON_ADC;
        state = Health_Data_State(&Health_Power, false, 0U, 0U, now_us, &reason);
        if (Health_Power.Item.State >= HEALTH_OK || state == HEALTH_OFFLINE)
        {
            state = HEALTH_FAULT;
        }
    }
    Health_Set_State(&Health_Power, state, reason);
    Health_Next.Power = Health_Power.Item;
}

static void Health_Add_Motor(uint8_t slot, Enum_Sys_Health_Motor_Type type, FDCAN_HandleTypeDef *bus,
                             uint32_t id, bool enabled, bool received, uint64_t sample_us,
                             uint64_t timeout_us, bool drive_fault)
{
    // 用固定注册槽保存历史，发布时压紧有效电机，避免新增电机导致已有历史错位。
    Health_Motor_Tracking *tracking = &Health_Motors[slot];
    if (tracking->Bus != bus || tracking->Feedback_Id != id ||
        (tracking->Feedback_Received && !received))
    {
        tracking->Link = {};
        tracking->Bus = bus;
        tracking->Feedback_Id = id;
    }
    tracking->Feedback_Received = received;
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    tracking->Link.Item.Data_Age_ms = received ? Health_Age_ms(now_us, sample_us) : UINT32_MAX;
    uint32_t reason = HEALTH_REASON_NONE;
    Enum_Sys_Health_State state = Health_Data_State(&tracking->Link, received, sample_us,
                                                    timeout_us, now_us, &reason);
    if (state == HEALTH_OK)
    {
        Health_Next.Online_Motor_Count++;
        if (drive_fault)
        {
            reason |= HEALTH_REASON_MOTOR_FAULT;
            state = HEALTH_FAULT;
        }
    }
    else if (state >= HEALTH_OFFLINE)
    {
        Health_Next.Offline_Motor_Count++;
    }
    Health_Set_State(&tracking->Link, state, reason);

    Struct_Sys_Health_Motor *out = &Health_Next.Motor[Health_Next.Motor_Count++];
    out->Health = tracking->Link.Item;
    out->Bus = bus;
    out->Feedback_Id = id;
    out->Enabled = enabled;
    out->Type = type;
    Health_Next.Overall_State = Health_Worse_State(Health_Next.Overall_State, out->Health.State);
}

static void Health_Update_Motors(void)
{
    const uint32_t previous_count = Health_Next.Motor_Count;
    Health_Next.Motor_Count = 0U;
    Health_Next.Online_Motor_Count = 0U;
    Health_Next.Offline_Motor_Count = 0U;

    for (uint8_t i = 0; i < DJI_MOTOR_MAX_MOTORS; ++i)
    {
        Struct_DJIMotor_Health motor;
        if (Class_DJIMotor::Get_Health(i, &motor))
        {
            Health_Add_Motor(i, HEALTH_MOTOR_DJI, motor.hfdcan, motor.rx_id, motor.enabled,
                             motor.feedback_received, motor.last_feedback_us, motor.feedback_timeout_us, false);
        }
    }
    for (uint8_t i = 0; i < DM_MOTOR_MAX_MOTORS; ++i)
    {
        Struct_DMMotor_Health motor;
        if (Class_DMMotor::Get_Health(i, &motor))
        {
            // 达妙反馈状态 0 为失能，1 为使能，其余按驱动故障处理。
            Health_Add_Motor(DJI_MOTOR_MAX_MOTORS + i, HEALTH_MOTOR_DM, motor.hfdcan, motor.rx_id,
                             motor.enabled, motor.feedback_received, motor.last_feedback_us,
                             motor.feedback_timeout_us, motor.state > 1U);
        }
    }
    for (uint8_t i = 0; i < QD4310_MAX_MOTORS; ++i)
    {
        QD4310_Health_t motor;
        if (QD4310_GetHealth(i, &motor))
        {
            Health_Add_Motor(DJI_MOTOR_MAX_MOTORS + DM_MOTOR_MAX_MOTORS + i, HEALTH_MOTOR_QD4310,
                             motor.hfdcan, motor.rx_id, motor.enabled, motor.feedback_received,
                             motor.last_feedback_us, motor.feedback_timeout_us, false);
        }
    }

    for (uint32_t i = Health_Next.Motor_Count; i < previous_count; ++i)
    {
        Health_Next.Motor[i] = {};
    }
}

void Sys_Health_Init(void)
{
    Health_Next = {};
    Health_Next.ABI_Version = SYS_HEALTH_ABI_VERSION;
    Health_Next.ABI_Size = sizeof(Struct_Sys_Health);
    Health_IMU = {};
    Health_Power = {};
    memset(Health_Motors, 0, sizeof(Health_Motors));
    Previous_IMU_Valid = false;
    Health_Update_Board_Init();
    Health_Next.Overall_State = Health_Board_State();
    Health_Publish();
}

void Sys_Health_Update(void)
{
    Health_Next.Update_Count++;
    Health_Update_Board_Init();
    Health_Update_IMU();
    Health_Update_Power();
    Health_Next.Overall_State = Health_Board_State();
    Health_Update_Motors();
    Health_Next.Safe_To_Run = Health_Next.Power.State == HEALTH_OK &&
                              (Health_Next.Overall_State == HEALTH_OK ||
                               Health_Next.Overall_State == HEALTH_WARN);
    Health_Publish();
}

bool Sys_Health_GetSnapshot(Struct_Sys_Health *snapshot)
{
    if (snapshot == nullptr)
    {
        return false;
    }
    const uint32_t primask = SYS_Enter_Critical();
    memcpy(snapshot, (const void *)&SYS_Health, sizeof(*snapshot));
    SYS_Exit_Critical(primask);
    return snapshot->ABI_Version == SYS_HEALTH_ABI_VERSION && snapshot->ABI_Size == sizeof(*snapshot) &&
           snapshot->Sequence == snapshot->Sequence_End && (snapshot->Sequence & 1U) == 0U;
}

bool Sys_Health_Is_Safe(void)
{
    return SYS_Health.Safe_To_Run != 0U;
}

Enum_Sys_Health_State Sys_Health_Get_Overall_State(void)
{
    return SYS_Health.Overall_State;
}

void Sys_Health_Set_Buzzer_Mute(bool mute)
{
    SYS_Health_Buzzer_Mute = mute;
}
