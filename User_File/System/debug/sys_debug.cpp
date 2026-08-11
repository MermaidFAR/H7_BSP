/**
 * @file sys_debug.cpp
 * @author zzm
 * @brief 系统调试数据接口
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"

#include "bsp_bmi088.h"
#include "sys_timestamp.h"

#include <stddef.h>
#include <type_traits>

/* Private macros ------------------------------------------------------------*/

static constexpr uint64_t DEBUG_IMU_UPDATE_INTERVAL_US = 5000U;
static constexpr uint16_t DEBUG_IMU_ABI_VERSION = 5U;

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint64_t Debug_IMU_Pre_Update_Timestamp = 0U;

static_assert(sizeof(Sys_Debug_IMU_Data_t) == 264U,
              "Debug_IMU_Data ABI size changed");
static_assert(alignof(Sys_Debug_IMU_Data_t) == 8U,
              "Debug_IMU_Data ABI alignment changed");
static_assert(std::is_standard_layout<Sys_Debug_IMU_Data_t>::value,
              "Debug_IMU_Data must remain standard-layout");
static_assert(offsetof(Sys_Debug_IMU_Data_t, Debug_ABI_Version) == 4U,
              "Debug_IMU_Data ABI version offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, timestamp_us) == 8U,
              "Debug_IMU_Data timestamp offset changed");
static_assert(offsetof(Sys_Debug_IMU_Data_t, sequence_end) == 260U,
              "Debug_IMU_Data sequence_end offset changed");

extern "C" {
volatile Sys_Debug_IMU_Data_t Debug_IMU_Data __attribute__((used)) = {};
}

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Sys_Debug_IMU_Update(void)
{
    const uint64_t timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
    if ((timestamp_us - Debug_IMU_Pre_Update_Timestamp) <
        DEBUG_IMU_UPDATE_INTERVAL_US)
    {
        return;
    }
    Debug_IMU_Pre_Update_Timestamp = timestamp_us;

    const Class_Matrix_f32<3, 1> original_accel =
        BSP_BMI088.Get_Original_Accel();
    const Class_Matrix_f32<3, 1> original_gyro =
        BSP_BMI088.Get_Original_Gyro();
    const Class_Matrix_f32<3, 1> corrected_gyro =
        BSP_BMI088.Get_Gyro_Body();
    const Class_Matrix_f32<3, 1> euler = BSP_BMI088.Get_Euler_Angle();
    const Class_Quaternion_f32 quaternion = BSP_BMI088.Get_Quaternion();
    const Class_Matrix_f32<3, 1> vqf_bias = BSP_BMI088.Get_VQF_Gyro_Bias();
    const Class_Matrix_f32<2, 1> rest_deviation =
        BSP_BMI088.Get_VQF_Relative_Rest_Deviation();
    const Struct_BMI088_Accel_Temperature_State temperature =
        BSP_BMI088.BMI088_Accel.Get_Temperature_State();
    const uint32_t write_sequence = (Debug_IMU_Data.sequence + 1U) | 1U;

    Debug_IMU_Data.sequence = write_sequence;
    Debug_IMU_Data.sequence_end = write_sequence;
    __DMB();

    Debug_IMU_Data.Debug_ABI_Version = DEBUG_IMU_ABI_VERSION;
    Debug_IMU_Data.Debug_ABI_Size = sizeof(Sys_Debug_IMU_Data_t);
    Debug_IMU_Data.timestamp_us = timestamp_us;
    Debug_IMU_Data.calculate_time_us = BSP_BMI088.Get_Calculating_Time();

    Debug_IMU_Data.Temperature_C = temperature.Temperature;
    Debug_IMU_Data.Original_Accel_X_m_s2 = original_accel[0][0];
    Debug_IMU_Data.Original_Accel_Y_m_s2 = original_accel[1][0];
    Debug_IMU_Data.Original_Accel_Z_m_s2 = original_accel[2][0];
    Debug_IMU_Data.Original_Gyro_X_rad_s = original_gyro[0][0];
    Debug_IMU_Data.Original_Gyro_Y_rad_s = original_gyro[1][0];
    Debug_IMU_Data.Original_Gyro_Z_rad_s = original_gyro[2][0];
    Debug_IMU_Data.Corrected_Gyro_X_rad_s = corrected_gyro[0][0];
    Debug_IMU_Data.Corrected_Gyro_Y_rad_s = corrected_gyro[1][0];
    Debug_IMU_Data.Corrected_Gyro_Z_rad_s = corrected_gyro[2][0];

    Debug_IMU_Data.Euler_Yaw_rad = euler[0][0];
    Debug_IMU_Data.Euler_Pitch_rad = euler[1][0];
    Debug_IMU_Data.Euler_Roll_rad = euler[2][0];
    Debug_IMU_Data.Quaternion_W = quaternion.Data[0];
    Debug_IMU_Data.Quaternion_X = quaternion.Data[1];
    Debug_IMU_Data.Quaternion_Y = quaternion.Data[2];
    Debug_IMU_Data.Quaternion_Z = quaternion.Data[3];

    Debug_IMU_Data.VQF_Gyro_Bias_X_rad_s = vqf_bias[0][0];
    Debug_IMU_Data.VQF_Gyro_Bias_Y_rad_s = vqf_bias[1][0];
    Debug_IMU_Data.VQF_Gyro_Bias_Z_rad_s = vqf_bias[2][0];
    Debug_IMU_Data.VQF_Gyro_Bias_Sigma_rad_s =
        BSP_BMI088.Get_VQF_Gyro_Bias_Sigma();
    Debug_IMU_Data.VQF_Accel_Correction_Rate_rad_s =
        BSP_BMI088.Get_VQF_Accel_Correction_Rate();
    Debug_IMU_Data.VQF_Rest_Gyro_Deviation_Ratio = rest_deviation[0][0];
    Debug_IMU_Data.VQF_Rest_Accel_Deviation_Ratio = rest_deviation[1][0];
    Debug_IMU_Data.Accel_Norm_m_s2 = BSP_BMI088.Get_Accel_Norm();
    Debug_IMU_Data.D_T_s = BSP_BMI088.Get_D_T();

    Debug_IMU_Data.VQF_Reset_Counter = BSP_BMI088.Get_VQF_Reset_Counter();
    Debug_IMU_Data.Accel_Update_Result = BSP_BMI088.Get_Accel_Update_Result();
    Debug_IMU_Data.Accel_Update_Attempt_Counter =
        BSP_BMI088.Get_Accel_Update_Attempt_Counter();
    Debug_IMU_Data.Accel_Update_Rejected_Counter =
        BSP_BMI088.Get_Accel_Update_Rejected_Counter();
    Debug_IMU_Data.Gyro_Outlier_Counter =
        BSP_BMI088.BMI088_Gyro.Get_Outlier_Counter();
    Debug_IMU_Data.Temperature_Outlier_Counter =
        BSP_BMI088.BMI088_Accel.Get_Temperature_Outlier_Counter();
    Debug_IMU_Data.Temperature_Stale_Counter = temperature.Stale_Counter;
    Debug_IMU_Data.Temperature_Age_Us = temperature.Age_Us;
    Debug_IMU_Data.Heater_PWM_Compare = temperature.Heater_PWM_Compare;

    Debug_IMU_Data.SPI2_Error_Counter = SPI2_Manage_Object.Error_Count;
    Debug_IMU_Data.SPI2_Transaction_Busy_Counter =
        SPI2_Manage_Object.Transaction_Busy_Count;
    Debug_IMU_Data.SPI2_Start_Failure_Counter =
        SPI2_Manage_Object.Start_Failure_Count;
    Debug_IMU_Data.SPI2_Callback_Anomaly_Counter =
        SPI2_Manage_Object.Callback_Anomaly_Count;
    Debug_IMU_Data.BMI088_SPI_Recovery_Counter =
        BSP_BMI088.Get_SPI_Recovery_Counter();
    Debug_IMU_Data.BMI088_Transfer_Timeout_Counter =
        BSP_BMI088.Get_SPI_Transfer_Timeout_Counter();
    Debug_IMU_Data.BMI088_Accel_Timeout_Counter =
        BSP_BMI088.Get_SPI_Accel_Timeout_Counter();
    Debug_IMU_Data.BMI088_Gyro_Timeout_Counter =
        BSP_BMI088.Get_SPI_Gyro_Timeout_Counter();
    Debug_IMU_Data.BMI088_Temperature_Timeout_Counter =
        BSP_BMI088.Get_SPI_Temperature_Timeout_Counter();
    Debug_IMU_Data.Sensor_Ready_Gap_Counter =
        BSP_BMI088.Get_Sensor_Ready_Gap_Counter();
    Debug_IMU_Data.Timestamp_Anomaly_Counter =
        BSP_BMI088.Get_Timestamp_Anomaly_Counter();

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

    Debug_IMU_Data.VQF_Rest_Detected =
        static_cast<uint8_t>(BSP_BMI088.Get_VQF_Rest_Detected());
    Debug_IMU_Data.Temperature_Data_Valid =
        static_cast<uint8_t>(temperature.Data_Valid);
    Debug_IMU_Data.BMI088_SPI_Recovery_Last_Reason =
        BSP_BMI088.Get_SPI_Recovery_Last_Reason();
    Debug_IMU_Data.SPI2_Last_Start_Failure_Status =
        SPI2_Manage_Object.Last_Start_Failure_Status;
    Debug_IMU_Data.SPI2_Transaction_Active =
        static_cast<uint8_t>(SPI2_Manage_Object.Transaction_Active);

    const uint32_t stable_sequence = write_sequence + 1U;
    __DMB();
    Debug_IMU_Data.sequence_end = stable_sequence;
    Debug_IMU_Data.sequence = stable_sequence;
    __DMB();
}
