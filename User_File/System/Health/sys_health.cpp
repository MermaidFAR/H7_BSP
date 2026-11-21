/** @file sys_health.cpp @author zzm @brief 低频设备健康采集与状态发布 */
#include "sys_health.h"
#include "sys_critical.h"
#include "bsp_bmi088.h"
#include "sys_debug.h"
#include "sys_timestamp.h"
#include <stddef.h>
#include <string.h>

extern "C" {
volatile Struct_Sys_Health SYS_Health __attribute__((used)) = {};
}

static Struct_Sys_Health Health_Next;
static uint8_t Recovery_Count[DJI_MOTOR_MAX_MOTORS + 1];
static uint64_t First_Seen_us[DJI_MOTOR_MAX_MOTORS + 1];
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

void Sys_Health_Init(void)
{
    Health_Next = {};
    Health_Next.ABI_Version = 2;
    Health_Next.ABI_Size = sizeof(Struct_Sys_Health);
    Previous_IMU_Valid = false;
    memset(Recovery_Count, 0, sizeof(Recovery_Count));
    memset(First_Seen_us, 0, sizeof(First_Seen_us));
    uint32_t interrupt_state = SYS_Enter_Critical();
    memcpy((void *)&SYS_Health, &Health_Next, sizeof(Health_Next));
    SYS_Exit_Critical(interrupt_state);
}

static void Health_Set_State(uint32_t index, Struct_Sys_Health_Item *item,
                             Enum_Sys_Health_State state, uint32_t reason)
{
    if (state == HEALTH_OK && item->State >= HEALTH_WARN)
    {
        if (++Recovery_Count[index] < SYS_HEALTH_RECOVERY_CHECKS)
        {
            item->Reason = HEALTH_REASON_RECOVERING;
            return;
        }
    }
    Recovery_Count[index] = 0;
    item->State = state;
    item->Reason = reason;
}

static Enum_Sys_Health_State Health_Data_State(uint32_t index, const Struct_Sys_Health_Item *item,
                                              bool received, uint64_t timestamp, uint64_t timeout_us,
                                              uint64_t now, uint32_t *reason)
{
    if (item->State == HEALTH_DISABLED)
        First_Seen_us[index] = now;
    if (!received)
    {
        *reason |= HEALTH_REASON_NO_DATA;
        return now - First_Seen_us[index] > (uint64_t)SYS_HEALTH_STARTUP_TIMEOUT_MS * 1000
                   ? HEALTH_OFFLINE : HEALTH_STARTING;
    }
    if (timestamp > now)
    {
        *reason |= HEALTH_REASON_TIMESTAMP;
        return HEALTH_FAULT;
    }
    if (now - timestamp > timeout_us)
    {
        *reason |= HEALTH_REASON_STALE;
        return HEALTH_OFFLINE;
    }
    return *reason != 0 ? HEALTH_WARN : HEALTH_OK;
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

void Sys_Health_Update(void)
{
    Health_Next.Update_Count++;
    Health_Next.Motor_Count = 0;
    Sys_Debug_IMU_Data_t imu;
    uint32_t interrupt_state = SYS_Enter_Critical();
    memcpy(&imu, (const void *)&Debug_IMU_Data, sizeof(imu));
    uint64_t sample_us = BSP_BMI088.Get_Last_Sample_Timestamp_Us();
    SYS_Exit_Critical(interrupt_state);
    uint64_t now = SYS_Timestamp.Get_Now_Microsecond();
    bool received = sample_us != 0 && imu.Debug_ABI_Size == sizeof(imu) &&
                    imu.sequence == imu.sequence_end && (imu.sequence & 1) == 0;
    uint64_t timestamp = sample_us < imu.timestamp_us ? sample_us : imu.timestamp_us;
    Health_Next.IMU.Data_Age_ms = received ? Health_Age_ms(now, timestamp) : UINT32_MAX;
    uint32_t reason = received ? Health_IMU_Reasons(&imu) : 0;
    Enum_Sys_Health_State state = Health_Data_State(0, &Health_Next.IMU, received, timestamp,
        (uint64_t)SYS_HEALTH_IMU_TIMEOUT_MS * 1000, now, &reason);
    if (received && (sample_us > now || imu.timestamp_us > now))
    {
        state = HEALTH_FAULT;
        reason |= HEALTH_REASON_TIMESTAMP;
    }
    Health_Set_State(0, &Health_Next.IMU, state, reason);
    for (uint32_t i = 0; i < DJI_MOTOR_MAX_MOTORS; ++i)
    {
        Struct_DJIMotor_Health motor;
        if (!Class_DJIMotor::Get_Health(i, &motor))
            continue;
        now = SYS_Timestamp.Get_Now_Microsecond();
        Struct_Sys_Health_Motor *out = &Health_Next.Motor[i];
        out->Bus = motor.hfdcan;
        out->Feedback_Id = motor.rx_id;
        out->Enabled = motor.enabled;
        out->Health.Data_Age_ms = motor.feedback_received ? Health_Age_ms(now, motor.last_feedback_us) : UINT32_MAX;
        Health_Next.Motor_Count++;
        reason = 0;
        state = Health_Data_State(i + 1, &out->Health, motor.feedback_received,
            motor.last_feedback_us, motor.feedback_timeout_us, now, &reason);
        Health_Set_State(i + 1, &out->Health, state, reason);
    }
    Health_Publish();
}

bool Sys_Health_GetSnapshot(Struct_Sys_Health *snapshot)
{
    if (snapshot == nullptr)
        return false;
    uint32_t interrupt_state = SYS_Enter_Critical();
    memcpy(snapshot, (const void *)&SYS_Health, sizeof(*snapshot));
    SYS_Exit_Critical(interrupt_state);
    return snapshot->ABI_Version == 2 && snapshot->Sequence == snapshot->Sequence_End && (snapshot->Sequence & 1) == 0;
}