/**
******************************************************************************
* @file sys_debug.h
* @brief 系统调试相关函数声明
*
* 本文件包含系统调试相关函数的声明和相关类型定义。
*
* @author zzm
* @date 2024-06-01
* @version 1.0
* @note 本文件为系统调试相关函数的头文件
*/

#ifndef __SYS_DEBUG_H__
#define __SYS_DEBUG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct Struct_Sys_Debug_IMU_Original_Data {
  float Original_Accel_X_m_s2;
  float Original_Accel_Y_m_s2;
  float Original_Accel_Z_m_s2;

  float Original_Gyro_X_rad_s;
  float Original_Gyro_Y_rad_s;
  float Original_Gyro_Z_rad_s;
} Sys_Debug_IMU_Original_Data_t;

typedef struct Struct_Sys_Debug_IMU_Data {
  uint32_t sequence;
  uint16_t Debug_ABI_Version;
  uint16_t Debug_ABI_Size;

  uint64_t timestamp_us;
  uint64_t calculate_time_us;

  float Tmp;
  float Accel_X_m_s2;
  float Accel_Y_m_s2;
  float Accel_Z_m_s2;

  float Gyro_X_rad_s;
  float Gyro_Y_rad_s;
  float Gyro_Z_rad_s;

  float Euler_Roll_rad;
  float Euler_Pitch_rad;
  float Euler_Yaw_rad;

  float Quaternion_W;
  float Quaternion_X;
  float Quaternion_Y;
  float Quaternion_Z;

  float Accel_Chi_Square_Loss;

  Sys_Debug_IMU_Original_Data_t Original_Data;

  float Accel_Norm_m_s2;
  float D_T_s;
  uint32_t EKF_Reset_Counter;
  uint8_t Accel_Update_Accepted;
  uint8_t Accel_Reject_Reason;
  uint8_t Gyro_Outlier_Counter;
  uint8_t Temperature_Outlier_Counter;
  float Accel_Yaw_Correction_rad;
  float Accel_Yaw_Correction_Accumulated_rad;

  float Fixed_Corrected_Gyro_X_rad_s;
  float Fixed_Corrected_Gyro_Y_rad_s;
  float Fixed_Corrected_Gyro_Z_rad_s;

  float Online_Gyro_Bias_X_rad_s;
  float Online_Gyro_Bias_Y_rad_s;
  float Online_Gyro_Bias_Z_rad_s;

  uint32_t Online_Gyro_Bias_Sample_Count;
  uint8_t Online_Gyro_Bias_Ready;
  uint8_t Online_Gyro_Bias_Reject_Reason;
  uint16_t Online_Gyro_Bias_Reset_Counter;
  float Online_Gyro_Bias_Soak_Elapsed_s;
  float Online_Gyro_Bias_Calibration_Elapsed_s;

  float Persisted_Gyro_Bias_X_rad_s;
  float Persisted_Gyro_Bias_Y_rad_s;
  float Persisted_Gyro_Bias_Z_rad_s;
  float Applied_Gyro_Bias_X_rad_s;
  float Applied_Gyro_Bias_Y_rad_s;
  float Applied_Gyro_Bias_Z_rad_s;
  uint32_t Persisted_Gyro_Bias_Sequence;
  float Gyro_Bias_Validation_Elapsed_s;
  float Gyro_Bias_Validation_Yaw_Delta_rad;
  uint16_t Gyro_Bias_Validation_Failure_Counter;
  uint16_t Gyro_Bias_Commit_Error_Counter;
  uint8_t Gyro_Bias_Applied;
  uint8_t Persisted_Gyro_Bias_Available;
  uint8_t Precision_Ready;
  uint8_t Gyro_Bias_Source;
  uint8_t Gyro_Bias_Validation_Active;
  uint8_t Gyro_Bias_Validation_Result;
  uint8_t Gyro_Bias_Validation_Reject_Reason;
  uint8_t Gyro_Bias_Commit_Pending;

  uint32_t Bias_Store_Active_Sequence;
  uint32_t Bias_Store_Fixed_Offset_CRC32;
  uint32_t Bias_Store_Algorithm_Config_CRC32;
  uint32_t Bias_Store_Commit_Count;
  uint32_t Bias_Store_Error_Count;
  uint8_t Bias_Store_Status;
  uint8_t Bias_Store_Active_Slot;
  uint8_t Bias_Store_Record_Loaded;
  uint8_t Bias_Store_Reserved;

  uint32_t Bias_Store_Promotion_Count;
  uint32_t Bias_Store_Golden_Sequence;
  uint32_t Bias_Store_Candidate_Sequence;
  uint32_t Bias_Store_Highest_Sequence;
  uint32_t Bias_Store_Last_Promotion_Request_Id;
  uint32_t Bias_Promotion_Response_Id;
  uint32_t Bias_Promotion_Result;
  uint32_t Bias_Promotion_Promoted_Sequence;

  uint32_t Gyro_Outlier_Counter_Total;
  uint32_t Temperature_Outlier_Counter_Total;
  uint32_t Accel_Update_Rejected_Counter_Total;
  uint32_t SPI2_Error_Counter_Total;
  uint32_t BMI088_SPI_Recovery_Counter;
  uint32_t Sensor_Ready_Gap_Counter;
  uint32_t Timestamp_Anomaly_Counter;

  uint8_t Bias_Store_Golden_Available;
  uint8_t Bias_Store_Candidate_Available;
  uint8_t Bias_Store_Legacy_Trial_Active;
  uint8_t Bias_Store_Write_Enabled;
  uint8_t Bias_Store_Scan_Degraded;
  uint8_t Bias_Store_Golden_Slot;
  uint8_t Bias_Store_Candidate_Slot;
  uint8_t Bias_Store_Promotion_Result;
  uint8_t Persisted_Gyro_Bias_Is_Golden;
  uint8_t Persisted_Gyro_Bias_Trial_Locked;
  uint8_t Gyro_Bias_Promotion_Candidate_Available;
  uint8_t Temperature_Data_Valid;
  uint32_t Temperature_Stale_Counter_Total;
  uint32_t Temperature_Age_Us;
  uint8_t Gyro_Bias_Candidate_Precheck_Passed;
  uint8_t Gyro_Bias_Maintenance_Mode;
  uint8_t Gyro_Bias_Fixed_Only_Mode;
  uint8_t Gyro_Bias_Validation_Window_Count;
  uint32_t sequence_end;

  uint32_t Accel_Update_Attempt_Counter_Total;
  uint32_t Accel_Reject_Invalid_Counter_Total;
  uint32_t Accel_Reject_Norm_Counter_Total;
  uint32_t Accel_Reject_Chi_Square_Counter_Total;
  uint32_t SPI2_Transaction_Busy_Counter_Total;
  uint32_t SPI2_Start_Failure_Counter_Total;
  uint32_t SPI2_Callback_Anomaly_Counter_Total;
  uint32_t BMI088_Transfer_Timeout_Counter_Total;
  uint32_t BMI088_Accel_Timeout_Counter_Total;
  uint32_t BMI088_Gyro_Timeout_Counter_Total;
  uint32_t BMI088_Temperature_Timeout_Counter_Total;
  uint32_t Heater_PWM_Compare;
  uint8_t BMI088_SPI_Recovery_Last_Reason;
  uint8_t SPI2_Last_Start_Failure_Status;
  uint8_t SPI2_Transaction_Active;
  uint8_t Gyro_Bias_Temperature_Window_Active;

  uint32_t SPI2_Timeout_Snapshot_Count;
  uint32_t SPI2_Timeout_Timestamp_Low32_Us;
  uint32_t SPI2_Timeout_Elapsed_Us;
  uint32_t SPI2_Timeout_SPI_CR1;
  uint32_t SPI2_Timeout_SPI_CR2;
  uint32_t SPI2_Timeout_SPI_CFG1;
  uint32_t SPI2_Timeout_SPI_IER;
  uint32_t SPI2_Timeout_SPI_SR;
  uint32_t SPI2_Timeout_RX_DMA_CR;
  uint32_t SPI2_Timeout_RX_DMA_NDTR;
  uint32_t SPI2_Timeout_TX_DMA_CR;
  uint32_t SPI2_Timeout_TX_DMA_NDTR;
  uint32_t SPI2_Timeout_HAL_Error_Code;
  uint16_t SPI2_Timeout_Manager_Tx_Length;
  uint16_t SPI2_Timeout_Manager_Rx_Length;
  uint8_t SPI2_Timeout_HAL_State;
  uint8_t SPI2_Timeout_HAL_Lock;
  uint8_t SPI2_Timeout_RX_DMA_State;
  uint8_t SPI2_Timeout_TX_DMA_State;
  uint8_t SPI2_Timeout_Transaction_Active;
  uint8_t SPI2_Timeout_NVIC_Pending_Bits;
  uint8_t SPI2_Timeout_NVIC_Active_Bits;
  uint8_t SPI2_Timeout_Reserved;
  uint32_t SPI2_Timeout_DMA1_LISR;
  uint16_t SPI2_Timeout_HAL_Tx_Xfer_Count;
  uint16_t SPI2_Timeout_HAL_Rx_Xfer_Count;

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
} Sys_Debug_IMU_Data_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

extern volatile Sys_Debug_IMU_Data_t Debug_IMU_Data;
void Sys_Debug_IMU_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_DEBUG_H__ */
