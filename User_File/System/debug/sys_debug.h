/**
 * @file sys_debug.h
 * @author zzm
 * @brief 系统调试数据接口
 *
 * @details
 * Debug_IMU_Data采用固定ABI供J-Link等外部工具不停核读取。字段的消费者可能不在
 * 固件源码中，不能仅依据C/C++调用关系删除或调整顺序；修改布局时必须同步提升
 * Debug_ABI_Version并更新采样profile。
 */

#ifndef __SYS_DEBUG_H
#define __SYS_DEBUG_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct Struct_Sys_Debug_IMU_Data
{
    // sequence与sequence_end组成seqlock，奇数表示正在写入，偶数表示快照稳定。
    uint32_t sequence;
    uint16_t Debug_ABI_Version;
    uint16_t Debug_ABI_Size;
    uint64_t timestamp_us;
    uint64_t calculate_time_us;

    float Temperature_C;
    float Original_Accel_X_m_s2;
    float Original_Accel_Y_m_s2;
    float Original_Accel_Z_m_s2;
    float Original_Gyro_X_rad_s;
    float Original_Gyro_Y_rad_s;
    float Original_Gyro_Z_rad_s;
    float Corrected_Gyro_X_rad_s;
    float Corrected_Gyro_Y_rad_s;
    float Corrected_Gyro_Z_rad_s;

    float Euler_Yaw_rad;
    float Euler_Pitch_rad;
    float Euler_Roll_rad;
    float Quaternion_W;
    float Quaternion_X;
    float Quaternion_Y;
    float Quaternion_Z;

    float VQF_Gyro_Bias_X_rad_s;
    float VQF_Gyro_Bias_Y_rad_s;
    float VQF_Gyro_Bias_Z_rad_s;
    float VQF_Gyro_Bias_Sigma_rad_s;
    float VQF_Accel_Correction_Rate_rad_s;
    float VQF_Rest_Gyro_Deviation_Ratio;
    float VQF_Rest_Accel_Deviation_Ratio;
    float Accel_Norm_m_s2;
    float D_T_s;

    uint32_t VQF_Reset_Counter;
    uint32_t Accel_Update_Result;
    uint32_t Accel_Update_Attempt_Counter;
    uint32_t Accel_Update_Rejected_Counter;
    uint32_t Gyro_Outlier_Counter;
    uint32_t Temperature_Outlier_Counter;
    uint32_t Temperature_Stale_Counter;
    uint32_t Temperature_Age_Us;
    uint32_t Heater_PWM_Compare;

    uint32_t SPI2_Error_Counter;
    uint32_t SPI2_Transaction_Busy_Counter;
    uint32_t SPI2_Start_Failure_Counter;
    uint32_t SPI2_Callback_Anomaly_Counter;
    uint32_t BMI088_SPI_Recovery_Counter;
    uint32_t BMI088_Transfer_Timeout_Counter;
    uint32_t BMI088_Accel_Timeout_Counter;
    uint32_t BMI088_Gyro_Timeout_Counter;
    uint32_t BMI088_Temperature_Timeout_Counter;
    uint32_t Sensor_Ready_Gap_Counter;
    uint32_t Timestamp_Anomaly_Counter;

    uint32_t Gyro_FIFO_Interrupt_Count;
    uint32_t Gyro_FIFO_Status_Read_Count;
    uint32_t Gyro_FIFO_Frame_Read_Count;
    uint32_t Gyro_FIFO_Overrun_Count;
    uint32_t Gyro_FIFO_Spurious_Interrupt_Count;
    uint32_t Gyro_FIFO_Followup_Request_Count;
    uint32_t Gyro_Queue_Enqueue_Count;
    uint32_t Gyro_Queue_Consume_Count;
    uint32_t Gyro_Queue_Drop_Count;
    uint16_t Gyro_Queue_Depth;
    uint16_t Gyro_Queue_High_Watermark;
    float Gyro_FIFO_Sample_Period_Us;

    uint8_t VQF_Rest_Detected;
    uint8_t Temperature_Data_Valid;
    uint8_t BMI088_SPI_Recovery_Last_Reason;
    uint8_t SPI2_Last_Start_Failure_Status;
    uint8_t SPI2_Transaction_Active;
    uint8_t Reserved[3];
    uint32_t sequence_end;
} Sys_Debug_IMU_Data_t;

/* Exported variables --------------------------------------------------------*/

extern volatile Sys_Debug_IMU_Data_t Debug_IMU_Data;

/* Exported function declarations --------------------------------------------*/

void Sys_Debug_IMU_Update(void);

#ifdef __cplusplus
}
#endif

#endif
