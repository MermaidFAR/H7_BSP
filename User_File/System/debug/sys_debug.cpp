/**
******************************************************************************
* @file sys_debug.cpp
* @brief 系统调试相关函数实现
*
* 本文件包含系统调试相关函数的实现。
*
* @author zzm
* @date 2024-06-01
* @version 1.0
* @note 本文件为系统调试相关函数的实现文件
*/

#include "sys_debug.h"
#include "bsp_bmi088.h"
#include "sys_imu_bias_store.h"
#include "sys_timestamp.h"
#include <stddef.h>
#include <type_traits>

static constexpr uint64_t DEBUG_IMU_UPDATE_INTERVAL_US = 5000U;
static constexpr uint16_t DEBUG_IMU_ABI_VERSION = 3U;
static uint64_t Debug_IMU_Pre_Update_Timestamp = 0U;

static_assert(sizeof(Sys_Debug_IMU_Data_t) == 504U, "Debug_IMU_Data ABI size changed");
static_assert(alignof(Sys_Debug_IMU_Data_t) == 8U, "Debug_IMU_Data ABI alignment changed");
static_assert(std::is_standard_layout<Sys_Debug_IMU_Data_t>::value,
              "Debug_IMU_Data must remain standard-layout");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Debug_ABI_Version) == 4U,
              "Debug_IMU_Data ABI version offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, timestamp_us) == 8U,
              "Debug_IMU_Data timestamp offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Bias_Store_Promotion_Count) == 244U,
              "Debug_IMU_Data promotion fields offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Gyro_Outlier_Counter_Total) == 276U,
              "Debug_IMU_Data diagnostics offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Bias_Store_Golden_Available) == 304U,
              "Debug_IMU_Data store flags offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Temperature_Stale_Counter_Total) == 316U,
              "Debug_IMU_Data temperature diagnostics offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Gyro_Bias_Candidate_Precheck_Passed) == 324U,
              "Debug_IMU_Data bias state offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, sequence_end) == 328U,
              "Debug_IMU_Data sequence_end offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Accel_Update_Attempt_Counter_Total) == 332U,
              "Debug_IMU_Data stage A counters offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, SPI2_Transaction_Busy_Counter_Total) == 348U,
              "Debug_IMU_Data SPI counters offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, BMI088_Transfer_Timeout_Counter_Total) == 360U,
              "Debug_IMU_Data timeout counters offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, BMI088_SPI_Recovery_Last_Reason) == 380U,
              "Debug_IMU_Data stage A state offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, SPI2_Timeout_Snapshot_Count) == 384U,
              "Debug_IMU_Data timeout snapshot offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, SPI2_Timeout_Manager_Tx_Length) == 436U,
              "Debug_IMU_Data timeout manager offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, SPI2_Timeout_HAL_State) == 440U,
              "Debug_IMU_Data timeout state offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, SPI2_Timeout_DMA1_LISR) == 448U,
              "Debug_IMU_Data timeout DMA flags offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, SPI2_Timeout_HAL_Tx_Xfer_Count) == 452U,
              "Debug_IMU_Data timeout HAL count offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Gyro_FIFO_Interrupt_Count) == 456U,
              "Debug_IMU_Data gyro FIFO counters offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Gyro_Queue_Depth) == 492U,
              "Debug_IMU_Data gyro queue state offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Gyro_FIFO_Sample_Period_Us) == 496U,
              "Debug_IMU_Data gyro FIFO period offset changed");

extern "C"
{
volatile Sys_Debug_IMU_Data_t Debug_IMU_Data __attribute__((used)) = {};
}

/**
 * @brief 更新IMU调试数据
 *
 * 该函数从BMI088获取最新的IMU数据，并更新到全局调试数据结构中。
 */
extern "C" void Sys_Debug_IMU_Update(void)
{
    const uint64_t timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
    if ((timestamp_us - Debug_IMU_Pre_Update_Timestamp) < DEBUG_IMU_UPDATE_INTERVAL_US)
    {
        return;
    }
    Debug_IMU_Pre_Update_Timestamp = timestamp_us;

    const uint64_t calculate_time_us = BSP_BMI088.Get_Calculating_Time();
    const Class_Matrix_f32<3, 1> accel = BSP_BMI088.Get_Accel();
    const Class_Matrix_f32<3, 1> gyro = BSP_BMI088.Get_Gyro();
    const Class_Matrix_f32<3, 1> euler_angle = BSP_BMI088.Get_Euler_Angle();
    const Class_Quaternion_f32 quaternion = BSP_BMI088.Get_Quaternion();
    const Class_Matrix_f32<3, 1> original_accel = BSP_BMI088.Get_Original_Accel();
    const Class_Matrix_f32<3, 1> original_gyro = BSP_BMI088.Get_Original_Gyro();
    const Class_Matrix_f32<3, 1> fixed_corrected_gyro = BSP_BMI088.Get_Fixed_Corrected_Gyro();
    const Class_Matrix_f32<3, 1> online_gyro_bias = BSP_BMI088.Get_Online_Gyro_Bias();
    const Class_Matrix_f32<3, 1> persisted_gyro_bias = BSP_BMI088.Get_Persisted_Gyro_Bias();
    const Class_Matrix_f32<3, 1> applied_gyro_bias = BSP_BMI088.Get_Applied_Gyro_Bias();
    const Struct_SYS_IMU_Bias_Store_Debug bias_store_debug = SYS_IMU_Bias_Store_Get_Debug();
    const uint32_t accel_update_result = BSP_BMI088.Get_Accel_Update_Result();
    const Struct_BMI088_Accel_Temperature_State temperature_state =
        BSP_BMI088.BMI088_Accel.Get_Temperature_State();
    const float accel_yaw_correction = BSP_BMI088.Get_Accel_Yaw_Correction();
    const float accel_yaw_correction_accumulated = BSP_BMI088.Get_Accel_Yaw_Correction_Accumulated();
    const uint32_t write_sequence = (Debug_IMU_Data.sequence + 1U) | 1U;

    Debug_IMU_Data.sequence = write_sequence;
    __DMB();
    Debug_IMU_Data.sequence_end = write_sequence;
    __DMB();

    Debug_IMU_Data.Debug_ABI_Version = DEBUG_IMU_ABI_VERSION;
    Debug_IMU_Data.Debug_ABI_Size = static_cast<uint16_t>(sizeof(Sys_Debug_IMU_Data_t));
    Debug_IMU_Data.timestamp_us = timestamp_us;
    Debug_IMU_Data.calculate_time_us = calculate_time_us;
    Debug_IMU_Data.Accel_X_m_s2 = accel[0][0];
    Debug_IMU_Data.Accel_Y_m_s2 = accel[1][0];
    Debug_IMU_Data.Accel_Z_m_s2 = accel[2][0];
    Debug_IMU_Data.Gyro_X_rad_s = gyro[0][0];
    Debug_IMU_Data.Gyro_Y_rad_s = gyro[1][0];
    Debug_IMU_Data.Gyro_Z_rad_s = gyro[2][0];

    Debug_IMU_Data.Euler_Yaw_rad = euler_angle[0][0];
    Debug_IMU_Data.Euler_Pitch_rad = euler_angle[1][0];
    Debug_IMU_Data.Euler_Roll_rad = euler_angle[2][0];

    Debug_IMU_Data.Quaternion_W = quaternion.Data[0];
    Debug_IMU_Data.Quaternion_X = quaternion.Data[1];
    Debug_IMU_Data.Quaternion_Y = quaternion.Data[2];
    Debug_IMU_Data.Quaternion_Z = quaternion.Data[3];

    Debug_IMU_Data.Accel_Chi_Square_Loss = BSP_BMI088.Get_Accel_Chi_Square_Loss();

    Debug_IMU_Data.Original_Data.Original_Accel_X_m_s2 = original_accel[0][0];
    Debug_IMU_Data.Original_Data.Original_Accel_Y_m_s2 = original_accel[1][0];
    Debug_IMU_Data.Original_Data.Original_Accel_Z_m_s2 = original_accel[2][0];
    Debug_IMU_Data.Original_Data.Original_Gyro_X_rad_s = original_gyro[0][0];
    Debug_IMU_Data.Original_Data.Original_Gyro_Y_rad_s = original_gyro[1][0];
    Debug_IMU_Data.Original_Data.Original_Gyro_Z_rad_s = original_gyro[2][0];
    Debug_IMU_Data.Tmp = temperature_state.Temperature;

    Debug_IMU_Data.Accel_Norm_m_s2 = BSP_BMI088.Get_Accel_Norm();
    Debug_IMU_Data.D_T_s = BSP_BMI088.Get_D_T();
    Debug_IMU_Data.EKF_Reset_Counter = BSP_BMI088.Get_EKF_Reset_Counter();
    Debug_IMU_Data.Accel_Update_Accepted = static_cast<uint8_t>(accel_update_result & 0x1U);
    Debug_IMU_Data.Accel_Reject_Reason = static_cast<uint8_t>((accel_update_result >> 8U) & 0xffU);
    Debug_IMU_Data.Gyro_Outlier_Counter = static_cast<uint8_t>(BSP_BMI088.BMI088_Gyro.Get_Outlier_Counter());
    Debug_IMU_Data.Temperature_Outlier_Counter = static_cast<uint8_t>(BSP_BMI088.BMI088_Accel.Get_Temperature_Outlier_Counter());
    Debug_IMU_Data.Accel_Yaw_Correction_rad = accel_yaw_correction;
    Debug_IMU_Data.Accel_Yaw_Correction_Accumulated_rad = accel_yaw_correction_accumulated;

    Debug_IMU_Data.Fixed_Corrected_Gyro_X_rad_s = fixed_corrected_gyro[0][0];
    Debug_IMU_Data.Fixed_Corrected_Gyro_Y_rad_s = fixed_corrected_gyro[1][0];
    Debug_IMU_Data.Fixed_Corrected_Gyro_Z_rad_s = fixed_corrected_gyro[2][0];
    Debug_IMU_Data.Online_Gyro_Bias_X_rad_s = online_gyro_bias[0][0];
    Debug_IMU_Data.Online_Gyro_Bias_Y_rad_s = online_gyro_bias[1][0];
    Debug_IMU_Data.Online_Gyro_Bias_Z_rad_s = online_gyro_bias[2][0];
    Debug_IMU_Data.Online_Gyro_Bias_Sample_Count = BSP_BMI088.Get_Online_Gyro_Bias_Sample_Count();
    Debug_IMU_Data.Online_Gyro_Bias_Ready = static_cast<uint8_t>(BSP_BMI088.Get_Online_Gyro_Bias_Ready());
    Debug_IMU_Data.Online_Gyro_Bias_Reject_Reason = BSP_BMI088.Get_Online_Gyro_Bias_Reject_Reason();
    Debug_IMU_Data.Online_Gyro_Bias_Reset_Counter = BSP_BMI088.Get_Online_Gyro_Bias_Reset_Counter();
    Debug_IMU_Data.Online_Gyro_Bias_Soak_Elapsed_s = BSP_BMI088.Get_Online_Gyro_Bias_Soak_Elapsed();
    Debug_IMU_Data.Online_Gyro_Bias_Calibration_Elapsed_s = BSP_BMI088.Get_Online_Gyro_Bias_Calibration_Elapsed();

    Debug_IMU_Data.Persisted_Gyro_Bias_X_rad_s = persisted_gyro_bias[0][0];
    Debug_IMU_Data.Persisted_Gyro_Bias_Y_rad_s = persisted_gyro_bias[1][0];
    Debug_IMU_Data.Persisted_Gyro_Bias_Z_rad_s = persisted_gyro_bias[2][0];
    Debug_IMU_Data.Applied_Gyro_Bias_X_rad_s = applied_gyro_bias[0][0];
    Debug_IMU_Data.Applied_Gyro_Bias_Y_rad_s = applied_gyro_bias[1][0];
    Debug_IMU_Data.Applied_Gyro_Bias_Z_rad_s = applied_gyro_bias[2][0];
    Debug_IMU_Data.Persisted_Gyro_Bias_Sequence = BSP_BMI088.Get_Persisted_Gyro_Bias_Sequence();
    Debug_IMU_Data.Gyro_Bias_Validation_Elapsed_s = BSP_BMI088.Get_Gyro_Bias_Validation_Elapsed();
    Debug_IMU_Data.Gyro_Bias_Validation_Yaw_Delta_rad = BSP_BMI088.Get_Gyro_Bias_Validation_Yaw_Delta();
    Debug_IMU_Data.Gyro_Bias_Validation_Failure_Counter = BSP_BMI088.Get_Gyro_Bias_Validation_Failure_Counter();
    Debug_IMU_Data.Gyro_Bias_Commit_Error_Counter = BSP_BMI088.Get_Gyro_Bias_Commit_Error_Counter();
    Debug_IMU_Data.Gyro_Bias_Applied = static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Applied());
    Debug_IMU_Data.Persisted_Gyro_Bias_Available = static_cast<uint8_t>(BSP_BMI088.Get_Persisted_Gyro_Bias_Available());
    Debug_IMU_Data.Precision_Ready = static_cast<uint8_t>(BSP_BMI088.Get_Precision_Ready());
    Debug_IMU_Data.Gyro_Bias_Source = BSP_BMI088.Get_Gyro_Bias_Source();
    Debug_IMU_Data.Gyro_Bias_Validation_Active = static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Validation_Active());
    Debug_IMU_Data.Gyro_Bias_Validation_Result = BSP_BMI088.Get_Gyro_Bias_Validation_Result();
    Debug_IMU_Data.Gyro_Bias_Validation_Reject_Reason = BSP_BMI088.Get_Gyro_Bias_Validation_Reject_Reason();
    Debug_IMU_Data.Gyro_Bias_Commit_Pending = static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Commit_Pending());

    Debug_IMU_Data.Bias_Store_Active_Sequence = bias_store_debug.Active_Sequence;
    Debug_IMU_Data.Bias_Store_Fixed_Offset_CRC32 = bias_store_debug.Fixed_Offset_CRC32;
    Debug_IMU_Data.Bias_Store_Algorithm_Config_CRC32 = bias_store_debug.Algorithm_Config_CRC32;
    Debug_IMU_Data.Bias_Store_Commit_Count = bias_store_debug.Commit_Count;
    Debug_IMU_Data.Bias_Store_Error_Count = bias_store_debug.Error_Count;
    Debug_IMU_Data.Bias_Store_Status = bias_store_debug.Status;
    Debug_IMU_Data.Bias_Store_Active_Slot = bias_store_debug.Active_Slot;
    Debug_IMU_Data.Bias_Store_Record_Loaded = bias_store_debug.Record_Loaded;
    Debug_IMU_Data.Bias_Store_Reserved = bias_store_debug.Reserved;

    Debug_IMU_Data.Bias_Store_Promotion_Count = bias_store_debug.Promotion_Count;
    Debug_IMU_Data.Bias_Store_Golden_Sequence = bias_store_debug.Golden_Sequence;
    Debug_IMU_Data.Bias_Store_Candidate_Sequence = bias_store_debug.Candidate_Sequence;
    Debug_IMU_Data.Bias_Store_Highest_Sequence = bias_store_debug.Highest_Sequence;
    Debug_IMU_Data.Bias_Store_Last_Promotion_Request_Id = bias_store_debug.Last_Promotion_Request_Id;
    Debug_IMU_Data.Bias_Promotion_Response_Id = IMU_Bias_Promotion_Control.Response_Id;
    Debug_IMU_Data.Bias_Promotion_Result = IMU_Bias_Promotion_Control.Result;
    Debug_IMU_Data.Bias_Promotion_Promoted_Sequence = IMU_Bias_Promotion_Control.Promoted_Sequence;

    Debug_IMU_Data.Gyro_Outlier_Counter_Total = BSP_BMI088.BMI088_Gyro.Get_Outlier_Counter();
    Debug_IMU_Data.Temperature_Outlier_Counter_Total = BSP_BMI088.BMI088_Accel.Get_Temperature_Outlier_Counter();
    Debug_IMU_Data.Accel_Update_Rejected_Counter_Total =
        BSP_BMI088.Get_Accel_Update_Rejected_Counter();
    Debug_IMU_Data.SPI2_Error_Counter_Total = SPI2_Manage_Object.Error_Count;
    Debug_IMU_Data.BMI088_SPI_Recovery_Counter = BSP_BMI088.Get_SPI_Recovery_Counter();
    Debug_IMU_Data.Sensor_Ready_Gap_Counter = BSP_BMI088.Get_Sensor_Ready_Gap_Counter();
    Debug_IMU_Data.Timestamp_Anomaly_Counter = BSP_BMI088.Get_Timestamp_Anomaly_Counter();

    Debug_IMU_Data.Bias_Store_Golden_Available = bias_store_debug.Golden_Available;
    Debug_IMU_Data.Bias_Store_Candidate_Available = bias_store_debug.Candidate_Available;
    Debug_IMU_Data.Bias_Store_Legacy_Trial_Active = bias_store_debug.Legacy_Trial_Active;
    Debug_IMU_Data.Bias_Store_Write_Enabled = bias_store_debug.Write_Enabled;
    Debug_IMU_Data.Bias_Store_Scan_Degraded = bias_store_debug.Scan_Degraded;
    Debug_IMU_Data.Bias_Store_Golden_Slot = bias_store_debug.Golden_Slot;
    Debug_IMU_Data.Bias_Store_Candidate_Slot = bias_store_debug.Candidate_Slot;
    Debug_IMU_Data.Bias_Store_Promotion_Result = bias_store_debug.Promotion_Result;
    Debug_IMU_Data.Persisted_Gyro_Bias_Is_Golden = static_cast<uint8_t>(BSP_BMI088.Get_Persisted_Gyro_Bias_Is_Golden());
    Debug_IMU_Data.Persisted_Gyro_Bias_Trial_Locked = static_cast<uint8_t>(BSP_BMI088.Get_Persisted_Gyro_Bias_Trial_Locked());
    Debug_IMU_Data.Gyro_Bias_Promotion_Candidate_Available =
        static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Promotion_Candidate_Available());
    Debug_IMU_Data.Temperature_Data_Valid =
        static_cast<uint8_t>(temperature_state.Data_Valid);
    Debug_IMU_Data.Temperature_Stale_Counter_Total = temperature_state.Stale_Counter;
    Debug_IMU_Data.Temperature_Age_Us = temperature_state.Age_Us;
    Debug_IMU_Data.Gyro_Bias_Candidate_Precheck_Passed =
        static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Candidate_Precheck_Passed());
    Debug_IMU_Data.Gyro_Bias_Maintenance_Mode =
        static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Maintenance_Mode());
    Debug_IMU_Data.Gyro_Bias_Fixed_Only_Mode =
        static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Fixed_Only_Mode());
    Debug_IMU_Data.Gyro_Bias_Validation_Window_Count =
        BSP_BMI088.Get_Gyro_Bias_Validation_Window_Count();

    Debug_IMU_Data.Accel_Update_Attempt_Counter_Total =
        BSP_BMI088.Get_Accel_Update_Attempt_Counter();
    Debug_IMU_Data.Accel_Reject_Invalid_Counter_Total =
        BSP_BMI088.Get_Accel_Reject_Invalid_Counter();
    Debug_IMU_Data.Accel_Reject_Norm_Counter_Total =
        BSP_BMI088.Get_Accel_Reject_Norm_Counter();
    Debug_IMU_Data.Accel_Reject_Chi_Square_Counter_Total =
        BSP_BMI088.Get_Accel_Reject_Chi_Square_Counter();
    Debug_IMU_Data.SPI2_Transaction_Busy_Counter_Total =
        SPI2_Manage_Object.Transaction_Busy_Count;
    Debug_IMU_Data.SPI2_Start_Failure_Counter_Total =
        SPI2_Manage_Object.Start_Failure_Count;
    Debug_IMU_Data.SPI2_Callback_Anomaly_Counter_Total =
        SPI2_Manage_Object.Callback_Anomaly_Count;
    Debug_IMU_Data.BMI088_Transfer_Timeout_Counter_Total =
        BSP_BMI088.Get_SPI_Transfer_Timeout_Counter();
    Debug_IMU_Data.BMI088_Accel_Timeout_Counter_Total =
        BSP_BMI088.Get_SPI_Accel_Timeout_Counter();
    Debug_IMU_Data.BMI088_Gyro_Timeout_Counter_Total =
        BSP_BMI088.Get_SPI_Gyro_Timeout_Counter();
    Debug_IMU_Data.BMI088_Temperature_Timeout_Counter_Total =
        BSP_BMI088.Get_SPI_Temperature_Timeout_Counter();
    Debug_IMU_Data.Heater_PWM_Compare = temperature_state.Heater_PWM_Compare;
    Debug_IMU_Data.BMI088_SPI_Recovery_Last_Reason =
        BSP_BMI088.Get_SPI_Recovery_Last_Reason();
    Debug_IMU_Data.SPI2_Last_Start_Failure_Status =
        SPI2_Manage_Object.Last_Start_Failure_Status;
    Debug_IMU_Data.SPI2_Transaction_Active =
        static_cast<uint8_t>(SPI2_Manage_Object.Transaction_Active);
    Debug_IMU_Data.Gyro_Bias_Temperature_Window_Active =
        static_cast<uint8_t>(BSP_BMI088.Get_Gyro_Bias_Temperature_Window_Active());

    Debug_IMU_Data.SPI2_Timeout_Snapshot_Count = SPI2_Timeout_Snapshot.Capture_Count;
    Debug_IMU_Data.SPI2_Timeout_Timestamp_Low32_Us =
        SPI2_Timeout_Snapshot.Timestamp_Low32_Us;
    Debug_IMU_Data.SPI2_Timeout_Elapsed_Us = SPI2_Timeout_Snapshot.Elapsed_Us;
    Debug_IMU_Data.SPI2_Timeout_SPI_CR1 = SPI2_Timeout_Snapshot.SPI_CR1;
    Debug_IMU_Data.SPI2_Timeout_SPI_CR2 = SPI2_Timeout_Snapshot.SPI_CR2;
    Debug_IMU_Data.SPI2_Timeout_SPI_CFG1 = SPI2_Timeout_Snapshot.SPI_CFG1;
    Debug_IMU_Data.SPI2_Timeout_SPI_IER = SPI2_Timeout_Snapshot.SPI_IER;
    Debug_IMU_Data.SPI2_Timeout_SPI_SR = SPI2_Timeout_Snapshot.SPI_SR;
    Debug_IMU_Data.SPI2_Timeout_RX_DMA_CR = SPI2_Timeout_Snapshot.RX_DMA_CR;
    Debug_IMU_Data.SPI2_Timeout_RX_DMA_NDTR = SPI2_Timeout_Snapshot.RX_DMA_NDTR;
    Debug_IMU_Data.SPI2_Timeout_TX_DMA_CR = SPI2_Timeout_Snapshot.TX_DMA_CR;
    Debug_IMU_Data.SPI2_Timeout_TX_DMA_NDTR = SPI2_Timeout_Snapshot.TX_DMA_NDTR;
    Debug_IMU_Data.SPI2_Timeout_HAL_Error_Code =
        SPI2_Timeout_Snapshot.HAL_Error_Code;
    Debug_IMU_Data.SPI2_Timeout_Manager_Tx_Length =
        SPI2_Timeout_Snapshot.Manager_Tx_Length;
    Debug_IMU_Data.SPI2_Timeout_Manager_Rx_Length =
        SPI2_Timeout_Snapshot.Manager_Rx_Length;
    Debug_IMU_Data.SPI2_Timeout_HAL_State = SPI2_Timeout_Snapshot.HAL_State;
    Debug_IMU_Data.SPI2_Timeout_HAL_Lock = SPI2_Timeout_Snapshot.HAL_Lock;
    Debug_IMU_Data.SPI2_Timeout_RX_DMA_State = SPI2_Timeout_Snapshot.RX_DMA_State;
    Debug_IMU_Data.SPI2_Timeout_TX_DMA_State = SPI2_Timeout_Snapshot.TX_DMA_State;
    Debug_IMU_Data.SPI2_Timeout_Transaction_Active =
        SPI2_Timeout_Snapshot.Transaction_Active;
    Debug_IMU_Data.SPI2_Timeout_NVIC_Pending_Bits =
        SPI2_Timeout_Snapshot.NVIC_Pending_Bits;
    Debug_IMU_Data.SPI2_Timeout_NVIC_Active_Bits =
        SPI2_Timeout_Snapshot.NVIC_Active_Bits;
    Debug_IMU_Data.SPI2_Timeout_Reserved = 0U;
    Debug_IMU_Data.SPI2_Timeout_DMA1_LISR = SPI2_Timeout_Snapshot.DMA1_LISR;
    Debug_IMU_Data.SPI2_Timeout_HAL_Tx_Xfer_Count =
        SPI2_Timeout_Snapshot.HAL_Tx_Xfer_Count;
    Debug_IMU_Data.SPI2_Timeout_HAL_Rx_Xfer_Count =
        SPI2_Timeout_Snapshot.HAL_Rx_Xfer_Count;

    Debug_IMU_Data.Gyro_FIFO_Interrupt_Count =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Interrupt_Count();
    Debug_IMU_Data.Gyro_FIFO_Status_Read_Count =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Status_Read_Count();
    Debug_IMU_Data.Gyro_FIFO_Frame_Read_Count =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Frame_Read_Count();
    Debug_IMU_Data.Gyro_FIFO_Overrun_Count =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Overrun_Count();
    Debug_IMU_Data.Gyro_FIFO_Spurious_Interrupt_Count =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Spurious_Interrupt_Count();
    Debug_IMU_Data.Gyro_FIFO_Followup_Request_Count =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Followup_Request_Count();
    Debug_IMU_Data.Gyro_Queue_Enqueue_Count =
        BSP_BMI088.BMI088_Gyro.Get_Queue_Enqueue_Count();
    Debug_IMU_Data.Gyro_Queue_Consume_Count =
        BSP_BMI088.BMI088_Gyro.Get_Queue_Consume_Count();
    Debug_IMU_Data.Gyro_Queue_Drop_Count =
        BSP_BMI088.BMI088_Gyro.Get_Queue_Drop_Count();
    Debug_IMU_Data.Gyro_Queue_Depth =
        BSP_BMI088.BMI088_Gyro.Get_Queue_Depth();
    Debug_IMU_Data.Gyro_Queue_High_Watermark =
        BSP_BMI088.BMI088_Gyro.Get_Queue_High_Watermark();
    Debug_IMU_Data.Gyro_FIFO_Sample_Period_Us =
        BSP_BMI088.BMI088_Gyro.Get_FIFO_Sample_Period_Us();

    const uint32_t stable_sequence = write_sequence + 1U;
    __DMB();
    Debug_IMU_Data.sequence_end = stable_sequence;
    __DMB();
    Debug_IMU_Data.sequence = stable_sequence;
}
