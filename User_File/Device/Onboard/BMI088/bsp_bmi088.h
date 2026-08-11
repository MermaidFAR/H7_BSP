/**
 * @file bsp_bmi088.h
 * @author zzm
 * @brief BMI088设备驱动与VQF姿态解算
 */

#ifndef __BSP_BMI088_H
#define __BSP_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "Accel/bsp_bmi088_accel.h"
#include "Gyro/bsp_bmi088_gyro.h"
#include "alg_filter_vqf.h"
#include "alg_quaternion.h"

extern "C" {
#include "cmsis_os2.h"
}

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct Struct_BMI088_Status
{
    bool Ready_Flag = false;
    bool Transfering_Flag = false;
    bool Update_Flag = false;
    uint64_t Ready_Timestamp = 0U;
    uint64_t Transfer_Ready_Timestamp = 0U;
    uint32_t Transfer_Start_Timestamp_Low32 = 0U;
    bool Transfer_Timeout_Armed = false;
    uint64_t Update_Timestamp = 0U;
    uint64_t Update_Ready_Timestamp = 0U;
};

enum Enum_BMI088_Accel_Reject_Reason : uint8_t
{
    BMI088_ACCEL_REJECT_NONE = 0U,
    BMI088_ACCEL_REJECT_INVALID = 1U << 0,
};

enum Enum_BMI088_SPI_Recovery_Reason : uint8_t
{
    BMI088_SPI_RECOVERY_NONE = 0U,
    BMI088_SPI_RECOVERY_ACCEL_TIMEOUT = 1U << 0,
    BMI088_SPI_RECOVERY_GYRO_TIMEOUT = 1U << 1,
    BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT = 1U << 2,
    BMI088_SPI_RECOVERY_HAL_ERROR = 1U << 3,
    BMI088_SPI_RECOVERY_ACCEL_START_FAILURE = 1U << 4,
    BMI088_SPI_RECOVERY_GYRO_START_FAILURE = 1U << 5,
    BMI088_SPI_RECOVERY_TEMPERATURE_START_FAILURE = 1U << 6,
};

struct Struct_BMI088_VQF_Config
{
    ///< VQF姿态修正、零偏估计和静止检测参数。
    Struct_VQF_Parameter Parameter;
    ///< 陀螺仪名义更新周期，单位s；当前2 kHz采样对应0.0005 s。
    float Gyro_D_T = 0.0005f;
    ///< 加速度计名义更新周期，单位s；当前250 Hz采样对应0.004 s。
    float Accel_D_T = 0.004f;
};

class Class_BMI088
{
public:
    Class_BMI088_Accel BMI088_Accel;
    Class_BMI088_Gyro BMI088_Gyro;

    /**
     * @brief 设置BMI088内部VQF的初始化配置
     * @param __Config VQF参数和传感器名义更新周期
     * @note 必须在Init()之前调用；初始化完成后的设置不会生效。
     */
    void Set_VQF_Config(const Struct_BMI088_VQF_Config &__Config);
    void Init();
    void Calculate();

    void SPI_RxCpltCallback();
    void EXTI_Flag_Callback(uint16_t __GPIO_Pin);
    void TIM_128ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Service_PeriodElapsedCallback();

    inline Class_Matrix_f32<3, 1> Get_Original_Accel() const;
    inline Class_Matrix_f32<3, 1> Get_Original_Gyro() const;
    inline Class_Matrix_f32<3, 1> Get_Fixed_Corrected_Gyro() const;
    inline Class_Matrix_f32<3, 1> Get_Fixed_Gyro_Offset() const;
    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const;
    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;
    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const;
    inline Class_Quaternion_f32 Get_Quaternion() const;
    inline Class_Matrix_f32<3, 1> Get_Accel_Body() const;
    inline Class_Matrix_f32<3, 1> Get_Gyro_Body() const;
    inline Class_Matrix_f32<3, 1> Get_Accel() const;
    inline Class_Matrix_f32<3, 1> Get_Gyro() const;

    inline float Get_Accel_Norm() const;
    inline uint32_t Get_Accel_Update_Result() const;
    inline uint32_t Get_Accel_Update_Rejected_Counter() const;
    inline uint32_t Get_Accel_Update_Attempt_Counter() const;
    inline uint32_t Get_SPI_Recovery_Counter() const;
    inline uint32_t Get_SPI_Transfer_Timeout_Counter() const;
    inline uint32_t Get_SPI_Accel_Timeout_Counter() const;
    inline uint32_t Get_SPI_Gyro_Timeout_Counter() const;
    inline uint32_t Get_SPI_Temperature_Timeout_Counter() const;
    inline uint8_t Get_SPI_Recovery_Last_Reason() const;
    inline uint32_t Get_Sensor_Ready_Gap_Counter() const;
    inline uint32_t Get_Timestamp_Anomaly_Counter() const;
    inline float Get_D_T() const;
    inline uint64_t Get_Calculating_Time() const;

    inline uint32_t Get_VQF_Reset_Counter() const;
    inline Class_Matrix_f32<3, 1> Get_VQF_Gyro_Bias() const;
    inline float Get_VQF_Gyro_Bias_Sigma() const;
    inline bool Get_VQF_Rest_Detected() const;
    inline Class_Matrix_f32<2, 1> Get_VQF_Relative_Rest_Deviation() const;
    inline float Get_VQF_Accel_Correction_Rate() const;

protected:
    Struct_SPI_Manage_Object *SPI_Manage_Object = nullptr;

    static constexpr uint32_t TRANSFERING_TIMEOUT = 1000U;
    static constexpr float D_T_TIMEOUT_THRESHOLD = 0.1f;

    const float ACCEL_AFFINE_DATA[9] = {
        0.9813826498493404f, 0.17232440504057203f, 0.027984325323801115f,
        -0.1690535919907899f, 0.9747302115792275f, -0.10336863799715153f,
        -0.046825636945091266f, 0.09953521655990044f, 0.986897809387138f};
    const float ACCEL_BIAS_DATA[3] = {
        0.0038458286072392397f, 0.00647039594993548f,
        0.014968990490337293f};
    const float GYRO_ZERO_OFFSET[3] = {
        -0.009049477102f, 0.000886600098f, 0.001653004116f};

    Struct_BMI088_VQF_Config VQF_Config;
    Class_Filter_VQF Filter_VQF;
    uint32_t VQF_Reset_Counter = 0U;
    uint64_t VQF_Pre_Timestamp = 0U;

    bool Init_Finished_Flag = false;
    Struct_BMI088_Status Accel_Status;
    Struct_BMI088_Status Gyro_Status;
    Struct_BMI088_Status Temperature_Status;
    uint8_t Transfer_Priority_Index = 0U;
    volatile bool Transfer_Service_Active = false;
    uint64_t Gyro_FIFO_Last_Fallback_Poll_Timestamp = 0U;

    Class_Matrix_f32<3, 1> Vector_Pending_Accel;
    uint64_t Pending_Accel_Timestamp = 0U;
    bool Accel_Observation_Pending = false;
    bool Pending_Accel_Valid = false;

    float D_T = 0.0005f;
    Class_Matrix_f32<3, 1> Vector_Original_Accel;
    Class_Matrix_f32<3, 1> Vector_Original_Gyro;
    Class_Matrix_f32<3, 1> Vector_Fixed_Corrected_Gyro;
    Class_Matrix_f32<3, 1> Vector_Euler_Angle;
    Class_Matrix_f32<3, 3> Matrix_Rotation;
    Class_Matrix_f32<4, 1> Vector_Axis_Angle;
    Class_Quaternion_f32 Quarternion;
    Class_Matrix_f32<3, 1> Vector_Accel_Body;
    Class_Matrix_f32<3, 1> Vector_Gyro_Body;
    Class_Matrix_f32<3, 1> Vector_Accel;
    Class_Matrix_f32<3, 1> Vector_Gyro;

    float Accel_Norm = 0.0f;
    uint32_t Accel_Update_Result = 0U;
    uint32_t Accel_Update_Attempt_Counter = 0U;
    uint32_t Accel_Update_Rejected_Counter = 0U;

    uint32_t SPI_Recovery_Counter = 0U;
    uint32_t SPI_Transfer_Timeout_Counter = 0U;
    uint32_t SPI_Accel_Timeout_Counter = 0U;
    uint32_t SPI_Gyro_Timeout_Counter = 0U;
    uint32_t SPI_Temperature_Timeout_Counter = 0U;
    volatile uint8_t SPI_Recovery_Pending_Reason =
        BMI088_SPI_RECOVERY_NONE;
    uint8_t SPI_Recovery_Last_Reason = BMI088_SPI_RECOVERY_NONE;
    uint32_t Sensor_Ready_Gap_Counter = 0U;
    uint32_t Timestamp_Anomaly_Counter = 0U;
    uint64_t Calculating_Time = 0U;

    void BMI088_Recover_SPI(uint8_t __Reason);
    void BMI088_Service_Transfer(const bool &__Allow_Recovery = false);
    void BMI088_Service_Transfer_Locked(const bool &__Allow_Recovery);
    void Set_Accel_Update_Result(const bool &__Accepted,
                                 const uint8_t &__Reject_Reason);
};

extern Class_BMI088 BSP_BMI088;

/* Exported functions --------------------------------------------------------*/

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Accel() const
{
    return Vector_Original_Accel;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Gyro() const
{
    return Vector_Original_Gyro;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Fixed_Corrected_Gyro() const
{
    return Vector_Fixed_Corrected_Gyro;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Fixed_Gyro_Offset() const
{
    return Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Euler_Angle() const
{
    return Vector_Euler_Angle;
}

inline Class_Matrix_f32<3, 3> Class_BMI088::Get_Rotation_Matrix() const
{
    return Matrix_Rotation;
}

inline Class_Matrix_f32<4, 1> Class_BMI088::Get_Axis_Angle() const
{
    return Vector_Axis_Angle;
}

inline Class_Quaternion_f32 Class_BMI088::Get_Quaternion() const
{
    return Quarternion;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel_Body() const
{
    return Vector_Accel_Body;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro_Body() const
{
    return Vector_Gyro_Body;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel() const
{
    return Vector_Accel;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro() const
{
    return Vector_Gyro;
}

inline float Class_BMI088::Get_Accel_Norm() const
{
    return Accel_Norm;
}

inline uint32_t Class_BMI088::Get_Accel_Update_Result() const
{
    return Accel_Update_Result;
}

inline uint32_t Class_BMI088::Get_Accel_Update_Rejected_Counter() const
{
    return Accel_Update_Rejected_Counter;
}

inline uint32_t Class_BMI088::Get_Accel_Update_Attempt_Counter() const
{
    return Accel_Update_Attempt_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Recovery_Counter() const
{
    return SPI_Recovery_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Transfer_Timeout_Counter() const
{
    return SPI_Transfer_Timeout_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Accel_Timeout_Counter() const
{
    return SPI_Accel_Timeout_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Gyro_Timeout_Counter() const
{
    return SPI_Gyro_Timeout_Counter;
}

inline uint32_t Class_BMI088::Get_SPI_Temperature_Timeout_Counter() const
{
    return SPI_Temperature_Timeout_Counter;
}

inline uint8_t Class_BMI088::Get_SPI_Recovery_Last_Reason() const
{
    return SPI_Recovery_Last_Reason;
}

inline uint32_t Class_BMI088::Get_Sensor_Ready_Gap_Counter() const
{
    return Sensor_Ready_Gap_Counter;
}

inline uint32_t Class_BMI088::Get_Timestamp_Anomaly_Counter() const
{
    return Timestamp_Anomaly_Counter;
}

inline float Class_BMI088::Get_D_T() const
{
    return D_T;
}

inline uint64_t Class_BMI088::Get_Calculating_Time() const
{
    return Calculating_Time;
}

inline uint32_t Class_BMI088::Get_VQF_Reset_Counter() const
{
    return VQF_Reset_Counter;
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_VQF_Gyro_Bias() const
{
    return Filter_VQF.Get_Bias_Estimate();
}

inline float Class_BMI088::Get_VQF_Gyro_Bias_Sigma() const
{
    return Filter_VQF.Get_Bias_Sigma();
}

inline bool Class_BMI088::Get_VQF_Rest_Detected() const
{
    return Filter_VQF.Get_Rest_Detected();
}

inline Class_Matrix_f32<2, 1> Class_BMI088::Get_VQF_Relative_Rest_Deviation() const
{
    return Filter_VQF.Get_Relative_Rest_Deviation();
}

inline float Class_BMI088::Get_VQF_Accel_Correction_Rate() const
{
    return Filter_VQF.Get_Last_Accel_Correction_Rate();
}

#ifdef __cplusplus
extern "C" {
#endif

void BMI088_TIM_128ms_Calculate_PeriodElapsedCallback();
void BMI088_TIM_1ms_Service_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
