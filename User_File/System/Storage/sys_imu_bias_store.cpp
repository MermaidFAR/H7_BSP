/**
 * @file sys_imu_bias_store.cpp
 * @author zzm
 * @brief Power-loss-safe BMI088 gyro bias Golden/Candidate store.
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_imu_bias_store.h"

#include "bsp_bmi088.h"
#include "bsp_w25q64jv.h"
#include "sys_flash_layout.h"

#include <cmath>
#include <cstddef>
#include <cstring>

/* Private macros ------------------------------------------------------------*/

static constexpr uint32_t SYS_IMU_BIAS_RECORD_MAGIC = 0x31424d49U;
static constexpr uint16_t SYS_IMU_BIAS_RECORD_FORMAT_V1 = 1U;
static constexpr uint16_t SYS_IMU_BIAS_RECORD_FORMAT_V2 = 2U;
static constexpr uint8_t SYS_IMU_BIAS_RECORD_ROLE_CANDIDATE = 1U;
static constexpr uint32_t SYS_IMU_BIAS_RECORD_COMMIT_MARKER = 0x54494d43U;
static constexpr uint32_t SYS_IMU_BIAS_RECORD_PROMOTION_MARKER = 0x444c4f47U;
static constexpr uint32_t SYS_IMU_BIAS_ERASED_WORD = 0xffffffffU;
static constexpr uint32_t SYS_IMU_BIAS_LEGACY_TRIAL_SEQUENCE = 4U;
static constexpr uint32_t SYS_IMU_BIAS_LEGACY_ALGORITHM_CONFIG_CRC32 = 4116438329U;
static constexpr float SYS_IMU_BIAS_LEGACY_TEMPERATURE_MIN_C = 49.5f;
static constexpr float SYS_IMU_BIAS_LEGACY_TEMPERATURE_MAX_C = 50.5f;
static constexpr uint32_t SYS_IMU_BIAS_LEGACY_MIN_SAMPLE_COUNT = 300000U;
static constexpr float SYS_IMU_BIAS_LEGACY_MAX_YAW_DELTA_RAD = 0.00465421134f;
static constexpr float SYS_IMU_BIAS_MAX_NORM_RAD_S = 0.12f;
static constexpr float SYS_IMU_BIAS_PROMOTION_MAX_OVERALL_DEG_H = 6.0f;
static constexpr float SYS_IMU_BIAS_PROMOTION_MAX_WINDOW_DEG_H = 8.0f;
static constexpr uint32_t SYS_IMU_BIAS_PROMOTION_MIN_DURATION_MS = 599000U;
static constexpr uint32_t SYS_IMU_BIAS_PROMOTION_MIN_SAMPLE_COUNT = 25000U;

/* Private types -------------------------------------------------------------*/

struct Struct_SYS_IMU_Bias_Record_V1
{
    uint32_t Magic;
    uint16_t Format_Version;
    uint16_t Record_Size;
    uint32_t Sequence;
    uint32_t Fixed_Offset_CRC32;
    uint32_t Algorithm_Config_CRC32;
    float Bias_Rad_S[3];
    float Calibration_Temperature_C;
    uint32_t Calibration_Sample_Count;
    float Validation_Yaw_Delta_Rad;
    uint32_t Reserved;
    uint32_t Payload_CRC32;
    uint32_t Commit_Marker;
};

struct Struct_SYS_IMU_Bias_Record_V2
{
    uint32_t Magic;
    uint16_t Format_Version;
    uint16_t Record_Size;
    uint32_t Sequence;
    uint32_t Fixed_Offset_CRC32;
    uint32_t Algorithm_Config_CRC32;
    float Bias_Rad_S[3];
    float Calibration_Temperature_C;
    uint32_t Calibration_Sample_Count;
    uint32_t Source_Sequence;
    uint32_t Validation_Mode_Version;
    uint8_t Record_Role;
    uint8_t Reserved_0[3];
    float Validation_Regression_Slope_Deg_H;
    float Validation_Endpoint_Drift_Deg_H;
    float Validation_Max_Window_Slope_Deg_H;
    uint32_t Validation_Duration_Ms;
    uint32_t Validation_Sample_Count;
    uint16_t EKF_Reset_Delta;
    uint16_t Timestamp_Gap_Count;
    uint16_t Gyro_Outlier_Delta;
    uint16_t Temperature_Outlier_Delta;
    uint16_t Accel_Outlier_Delta;
    uint16_t SPI_Error_Delta;
    uint16_t Flash_Error_Delta;
    uint16_t Reserved_1;
    uint32_t Payload_CRC32;
    uint32_t Commit_Marker;
    uint32_t Promotion_Marker;
};

struct Struct_SYS_IMU_Bias_Slot_State
{
    bool Read_OK;
    bool Legacy_Valid;
    bool Candidate_Valid;
    bool Golden_Valid;
    Struct_SYS_IMU_Bias_Record_V1 Legacy;
    Struct_SYS_IMU_Bias_Record_V2 V2;
};

struct Struct_SYS_IMU_Bias_Promotion_Request
{
    uint32_t Request_Id;
    uint32_t Expected_Candidate_Sequence;
    uint32_t Validation_Mode_Version;
    float Regression_Slope_Deg_Per_Hour;
    float Endpoint_Drift_Deg_Per_Hour;
    float Max_Window_Slope_Deg_Per_Hour;
    uint32_t Validation_Duration_Ms;
    uint32_t Validation_Sample_Count;
    uint16_t EKF_Reset_Delta;
    uint16_t Timestamp_Gap_Count;
    uint16_t Gyro_Outlier_Delta;
    uint16_t Temperature_Outlier_Delta;
    uint16_t Accel_Outlier_Delta;
    uint16_t SPI_Error_Delta;
    uint16_t Flash_Error_Delta;
};

static_assert(sizeof(Struct_SYS_IMU_Bias_Record_V1) == 56U,
              "IMU bias v1 record ABI changed");
static_assert(offsetof(Struct_SYS_IMU_Bias_Record_V1, Payload_CRC32) == 48U,
              "IMU bias v1 payload CRC offset changed");
static_assert(offsetof(Struct_SYS_IMU_Bias_Record_V1, Commit_Marker) == 52U,
              "IMU bias v1 commit marker offset changed");
static_assert(sizeof(Struct_SYS_IMU_Bias_Record_V2) == 100U,
              "IMU bias v2 record ABI changed");
static_assert(offsetof(Struct_SYS_IMU_Bias_Record_V2, Payload_CRC32) == 88U,
              "IMU bias v2 payload CRC offset changed");
static_assert(offsetof(Struct_SYS_IMU_Bias_Record_V2, Commit_Marker) == 92U,
              "IMU bias v2 commit marker offset changed");
static_assert(offsetof(Struct_SYS_IMU_Bias_Record_V2, Promotion_Marker) == 96U,
              "IMU bias v2 promotion marker offset changed");
static_assert(sizeof(Struct_SYS_IMU_Bias_Record_V2) < Namespace_SYS_Flash_Layout::W25Q64_SECTOR_SIZE,
              "IMU bias v2 record must fit in one sector");

/* Private variables ---------------------------------------------------------*/

static Struct_SYS_IMU_Bias_Store_Debug IMU_Bias_Store_Debug;
static Struct_SYS_IMU_Bias_Slot_State Slot_State[2] = {};
static Struct_SYS_IMU_Bias_Record_V1 Legacy_Trial_Record = {};
static Struct_SYS_IMU_Bias_Record_V2 Active_Golden_Record = {};
static Struct_SYS_IMU_Bias_Record_V2 Staged_Candidate_Record = {};
static bool Store_Initialized = false;
static bool Write_Enabled = false;
static bool Legacy_Trial_Valid = false;
static bool Active_Golden_Valid = false;
static bool Staged_Candidate_Valid = false;
static uint8_t Legacy_Trial_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
static uint8_t Active_Golden_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
static uint8_t Staged_Candidate_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
static uint32_t Highest_Sequence = 0U;

volatile Struct_SYS_IMU_Bias_Promotion_Control IMU_Bias_Promotion_Control __attribute__((used)) = {};
volatile uint32_t IMU_Bias_Maintenance_Command __attribute__((used)) = 0U;
volatile uint32_t IMU_Bias_Fixed_Only_Command __attribute__((used)) = 0U;

/* Private function declarations ---------------------------------------------*/

static uint32_t CRC32_Accumulate(uint32_t Crc, const void *Data, size_t Length);
static uint32_t CRC32_Calculate(const void *Data, size_t Length);
static uint32_t Calculate_Fixed_Offset_CRC32();
static uint32_t Calculate_Algorithm_Config_CRC32();
static bool Bias_Is_Sane(const float Bias[3]);
static bool Sequence_Is_Newer(uint32_t Left, uint32_t Right);
static bool Legacy_Record_Is_Valid(const Struct_SYS_IMU_Bias_Record_V1 &Record);
static bool V2_Record_Is_Candidate_Valid(const Struct_SYS_IMU_Bias_Record_V2 &Record);
static bool V2_Record_Is_Golden(const Struct_SYS_IMU_Bias_Record_V2 &Record);
static bool Read_Slot(uint8_t Slot, Struct_SYS_IMU_Bias_Slot_State &State);
static uint32_t Slot_Address(uint8_t Slot);
static bool Erase_Sector(uint32_t Address);
static bool Commit_Candidate(uint32_t Address, const Struct_SYS_IMU_Bias_Record_V2 &Record);
static bool Promote_Candidate(uint32_t Address, const Struct_SYS_IMU_Bias_Record_V2 &Record);
static bool Promotion_Request_Is_Valid(const Struct_SYS_IMU_Bias_Promotion_Request &Request);
static bool Snapshot_Promotion_Request(Struct_SYS_IMU_Bias_Promotion_Request &Request);
static void Set_Promotion_Result(uint32_t Request_Id, uint8_t Result, uint32_t Promoted_Sequence);
static void Register_Error();

/* Function prototypes -------------------------------------------------------*/

static uint32_t CRC32_Accumulate(uint32_t Crc, const void *Data, size_t Length)
{
    const uint8_t *bytes = static_cast<const uint8_t *>(Data);
    for (size_t index = 0U; index < Length; index++)
    {
        Crc ^= bytes[index];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            Crc = (Crc >> 1U) ^ ((Crc & 1U) != 0U ? 0xedb88320U : 0U);
        }
    }
    return Crc;
}

static uint32_t CRC32_Calculate(const void *Data, size_t Length)
{
    return CRC32_Accumulate(0xffffffffU, Data, Length) ^ 0xffffffffU;
}

static uint32_t Calculate_Fixed_Offset_CRC32()
{
    const Class_Matrix_f32<3, 1> fixed_offset = BSP_BMI088.Get_Fixed_Gyro_Offset();
    const float values[3] = {
        fixed_offset[0][0],
        fixed_offset[1][0],
        fixed_offset[2][0],
    };
    return CRC32_Calculate(values, sizeof(values));
}

static uint32_t Calculate_Algorithm_Config_CRC32()
{
    const Struct_BMI088_Gyro_Bias_Algorithm_Config config =
        BSP_BMI088.Get_Gyro_Bias_Algorithm_Config();
    uint32_t crc = 0xffffffffU;

    crc = CRC32_Accumulate(crc, &config.Version, sizeof(config.Version));
    crc = CRC32_Accumulate(crc, &config.Min_Sample_Count, sizeof(config.Min_Sample_Count));
    crc = CRC32_Accumulate(crc, &config.Soak_Time_us, sizeof(config.Soak_Time_us));
    crc = CRC32_Accumulate(crc, &config.Calibration_Time_us, sizeof(config.Calibration_Time_us));
    crc = CRC32_Accumulate(crc, &config.Sample_Gap_Timeout_us, sizeof(config.Sample_Gap_Timeout_us));
    crc = CRC32_Accumulate(crc, &config.Validation_Time_us, sizeof(config.Validation_Time_us));
    crc = CRC32_Accumulate(crc, &config.Validation_Window_Time_us,
                           sizeof(config.Validation_Window_Time_us));
    crc = CRC32_Accumulate(crc, &config.Validation_Window_Count,
                           sizeof(config.Validation_Window_Count));
    crc = CRC32_Accumulate(crc, &config.Temperature_Min_C, sizeof(config.Temperature_Min_C));
    crc = CRC32_Accumulate(crc, &config.Temperature_Max_C, sizeof(config.Temperature_Max_C));
    crc = CRC32_Accumulate(crc, &config.Persisted_Temperature_Hold_Min_C,
                           sizeof(config.Persisted_Temperature_Hold_Min_C));
    crc = CRC32_Accumulate(crc, &config.Persisted_Temperature_Hold_Max_C,
                           sizeof(config.Persisted_Temperature_Hold_Max_C));
    crc = CRC32_Accumulate(crc, &config.Gyro_Norm_Threshold_rad_s, sizeof(config.Gyro_Norm_Threshold_rad_s));
    crc = CRC32_Accumulate(crc, &config.Accel_Norm_Error_Threshold_m_s2, sizeof(config.Accel_Norm_Error_Threshold_m_s2));
    crc = CRC32_Accumulate(crc, &config.Accel_Norm_LPF_Time_Constant_s, sizeof(config.Accel_Norm_LPF_Time_Constant_s));
    crc = CRC32_Accumulate(crc, &config.Validation_Max_Overall_Yaw_Delta_rad,
                           sizeof(config.Validation_Max_Overall_Yaw_Delta_rad));
    crc = CRC32_Accumulate(crc, &config.Validation_Max_Window_Yaw_Delta_rad,
                           sizeof(config.Validation_Max_Window_Yaw_Delta_rad));
    return crc ^ 0xffffffffU;
}

static bool Bias_Is_Sane(const float Bias[3])
{
    float norm_squared = 0.0f;
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        if (!std::isfinite(Bias[axis]))
        {
            return false;
        }
        norm_squared += Bias[axis] * Bias[axis];
    }
    return norm_squared < (SYS_IMU_BIAS_MAX_NORM_RAD_S * SYS_IMU_BIAS_MAX_NORM_RAD_S);
}

static bool Sequence_Is_Newer(uint32_t Left, uint32_t Right)
{
    return static_cast<int32_t>(Left - Right) > 0;
}

static bool Legacy_Record_Is_Valid(const Struct_SYS_IMU_Bias_Record_V1 &Record)
{
    if (Record.Magic != SYS_IMU_BIAS_RECORD_MAGIC ||
        Record.Format_Version != SYS_IMU_BIAS_RECORD_FORMAT_V1 ||
        Record.Record_Size != sizeof(Struct_SYS_IMU_Bias_Record_V1) ||
        Record.Sequence == 0U ||
        Record.Commit_Marker != SYS_IMU_BIAS_RECORD_COMMIT_MARKER ||
        Record.Fixed_Offset_CRC32 != IMU_Bias_Store_Debug.Fixed_Offset_CRC32 ||
        Record.Algorithm_Config_CRC32 != SYS_IMU_BIAS_LEGACY_ALGORITHM_CONFIG_CRC32 ||
        !Bias_Is_Sane(Record.Bias_Rad_S) ||
        !std::isfinite(Record.Calibration_Temperature_C) ||
        Record.Calibration_Temperature_C < SYS_IMU_BIAS_LEGACY_TEMPERATURE_MIN_C ||
        Record.Calibration_Temperature_C > SYS_IMU_BIAS_LEGACY_TEMPERATURE_MAX_C ||
        Record.Calibration_Sample_Count < SYS_IMU_BIAS_LEGACY_MIN_SAMPLE_COUNT ||
        !std::isfinite(Record.Validation_Yaw_Delta_Rad) ||
        !(std::fabs(Record.Validation_Yaw_Delta_Rad) < SYS_IMU_BIAS_LEGACY_MAX_YAW_DELTA_RAD))
    {
        return false;
    }

    return Record.Payload_CRC32 ==
           CRC32_Calculate(&Record, offsetof(Struct_SYS_IMU_Bias_Record_V1, Payload_CRC32));
}

static bool V2_Record_Is_Candidate_Valid(const Struct_SYS_IMU_Bias_Record_V2 &Record)
{
    const Struct_BMI088_Gyro_Bias_Algorithm_Config algorithm_config =
        BSP_BMI088.Get_Gyro_Bias_Algorithm_Config();

    if (Record.Magic != SYS_IMU_BIAS_RECORD_MAGIC ||
        Record.Format_Version != SYS_IMU_BIAS_RECORD_FORMAT_V2 ||
        Record.Record_Size != sizeof(Struct_SYS_IMU_Bias_Record_V2) ||
        Record.Sequence == 0U ||
        Record.Fixed_Offset_CRC32 != IMU_Bias_Store_Debug.Fixed_Offset_CRC32 ||
        Record.Algorithm_Config_CRC32 != IMU_Bias_Store_Debug.Algorithm_Config_CRC32 ||
        Record.Validation_Mode_Version != SYS_IMU_BIAS_VALIDATION_MODE_VERSION ||
        Record.Record_Role != SYS_IMU_BIAS_RECORD_ROLE_CANDIDATE ||
        Record.Commit_Marker != SYS_IMU_BIAS_RECORD_COMMIT_MARKER ||
        (Record.Promotion_Marker != SYS_IMU_BIAS_ERASED_WORD &&
        Record.Promotion_Marker != SYS_IMU_BIAS_RECORD_PROMOTION_MARKER) ||
        !Bias_Is_Sane(Record.Bias_Rad_S) ||
        !std::isfinite(Record.Calibration_Temperature_C) ||
        Record.Calibration_Temperature_C < algorithm_config.Persisted_Temperature_Hold_Min_C ||
        Record.Calibration_Temperature_C > algorithm_config.Persisted_Temperature_Hold_Max_C ||
        !std::isfinite(Record.Validation_Regression_Slope_Deg_H) ||
        !std::isfinite(Record.Validation_Endpoint_Drift_Deg_H) ||
        !std::isfinite(Record.Validation_Max_Window_Slope_Deg_H) ||
        !(std::fabs(Record.Validation_Regression_Slope_Deg_H) < SYS_IMU_BIAS_PROMOTION_MAX_OVERALL_DEG_H) ||
        !(std::fabs(Record.Validation_Endpoint_Drift_Deg_H) < SYS_IMU_BIAS_PROMOTION_MAX_OVERALL_DEG_H) ||
        !(std::fabs(Record.Validation_Max_Window_Slope_Deg_H) < SYS_IMU_BIAS_PROMOTION_MAX_WINDOW_DEG_H) ||
        Record.Validation_Duration_Ms < SYS_IMU_BIAS_PROMOTION_MIN_DURATION_MS ||
        Record.Validation_Sample_Count < SYS_IMU_BIAS_PROMOTION_MIN_SAMPLE_COUNT ||
        Record.EKF_Reset_Delta != 0U ||
        Record.Timestamp_Gap_Count != 0U ||
        Record.Gyro_Outlier_Delta != 0U ||
        Record.Temperature_Outlier_Delta != 0U ||
        Record.Accel_Outlier_Delta != 0U ||
        Record.SPI_Error_Delta != 0U ||
        Record.Flash_Error_Delta != 0U)
    {
        return false;
    }

    return Record.Payload_CRC32 ==
           CRC32_Calculate(&Record, offsetof(Struct_SYS_IMU_Bias_Record_V2, Payload_CRC32));
}

static bool V2_Record_Is_Golden(const Struct_SYS_IMU_Bias_Record_V2 &Record)
{
    return V2_Record_Is_Candidate_Valid(Record) &&
           Record.Promotion_Marker == SYS_IMU_BIAS_RECORD_PROMOTION_MARKER;
}

static uint32_t Slot_Address(uint8_t Slot)
{
    return Slot == SYS_IMU_BIAS_STORE_SLOT_A
               ? Namespace_SYS_Flash_Layout::IMU_BIAS_SLOT_A_ADDRESS
               : Namespace_SYS_Flash_Layout::IMU_BIAS_SLOT_B_ADDRESS;
}

static bool Read_Slot(uint8_t Slot, Struct_SYS_IMU_Bias_Slot_State &State)
{
    std::memset(&State, 0, sizeof(State));
    std::memset(&State.V2, 0xff, sizeof(State.V2));
    State.Read_OK = BSP_W25Q64JV.Read_Data(&State.V2, Slot_Address(Slot), sizeof(State.V2));
    if (!State.Read_OK)
    {
        return false;
    }

    std::memcpy(&State.Legacy, &State.V2, sizeof(State.Legacy));
    State.Legacy_Valid = Legacy_Record_Is_Valid(State.Legacy);
    State.Candidate_Valid = V2_Record_Is_Candidate_Valid(State.V2);
    State.Golden_Valid = State.Candidate_Valid && V2_Record_Is_Golden(State.V2);
    return true;
}

static bool Erase_Sector(uint32_t Address)
{
    const uint32_t error_count = BSP_W25Q64JV.Get_Auto_Polling_Error_Count();

    if (!BSP_W25Q64JV.Set_Write_Enable())
    {
        return false;
    }
    while (!BSP_W25Q64JV.Is_Ready())
    {
        osDelay(1U);
    }
    if (BSP_W25Q64JV.Get_Auto_Polling_Error_Count() != error_count ||
        !BSP_W25Q64JV.Set_Sector_Erased(Address))
    {
        return false;
    }
    while (!BSP_W25Q64JV.Is_Ready())
    {
        osDelay(1U);
    }
    return BSP_W25Q64JV.Get_Auto_Polling_Error_Count() == error_count;
}

static bool Commit_Candidate(uint32_t Address, const Struct_SYS_IMU_Bias_Record_V2 &Record)
{
    if (!V2_Record_Is_Candidate_Valid(Record) ||
        Record.Promotion_Marker != SYS_IMU_BIAS_ERASED_WORD)
    {
        return false;
    }

    if (!Erase_Sector(Address))
    {
        return false;
    }

    constexpr size_t commit_offset = offsetof(Struct_SYS_IMU_Bias_Record_V2, Commit_Marker);
    if (!BSP_W25Q64JV.Write_Data(&Record, Address, commit_offset))
    {
        return false;
    }

    Struct_SYS_IMU_Bias_Record_V2 readback = {};
    if (!BSP_W25Q64JV.Read_Data(&readback, Address, sizeof(readback)) ||
        std::memcmp(&readback, &Record, commit_offset) != 0 ||
        readback.Commit_Marker != SYS_IMU_BIAS_ERASED_WORD ||
        readback.Promotion_Marker != SYS_IMU_BIAS_ERASED_WORD)
    {
        return false;
    }

    if (!BSP_W25Q64JV.Write_Data(&Record.Commit_Marker,
                                  Address + commit_offset,
                                  sizeof(Record.Commit_Marker)) ||
        !BSP_W25Q64JV.Read_Data(&readback, Address, sizeof(readback)))
    {
        return false;
    }

    return V2_Record_Is_Candidate_Valid(readback) &&
           readback.Promotion_Marker == SYS_IMU_BIAS_ERASED_WORD &&
           std::memcmp(&readback, &Record, offsetof(Struct_SYS_IMU_Bias_Record_V2, Promotion_Marker)) == 0;
}

static bool Promote_Candidate(uint32_t Address, const Struct_SYS_IMU_Bias_Record_V2 &Record)
{
    constexpr size_t promotion_offset = offsetof(Struct_SYS_IMU_Bias_Record_V2, Promotion_Marker);
    if (!BSP_W25Q64JV.Write_Data(&Record.Promotion_Marker,
                                  Address + promotion_offset,
                                  sizeof(Record.Promotion_Marker)))
    {
        return false;
    }

    Struct_SYS_IMU_Bias_Record_V2 readback = {};
    return BSP_W25Q64JV.Read_Data(&readback, Address, sizeof(readback)) &&
           V2_Record_Is_Golden(readback) &&
           std::memcmp(&readback, &Record, sizeof(Record)) == 0;
}

static bool Promotion_Request_Is_Valid(const Struct_SYS_IMU_Bias_Promotion_Request &Request)
{
    return Request.Request_Id != 0U &&
           Request.Validation_Mode_Version == SYS_IMU_BIAS_VALIDATION_MODE_VERSION &&
           std::isfinite(Request.Regression_Slope_Deg_Per_Hour) &&
           std::isfinite(Request.Endpoint_Drift_Deg_Per_Hour) &&
           std::isfinite(Request.Max_Window_Slope_Deg_Per_Hour) &&
           std::fabs(Request.Regression_Slope_Deg_Per_Hour) < SYS_IMU_BIAS_PROMOTION_MAX_OVERALL_DEG_H &&
           std::fabs(Request.Endpoint_Drift_Deg_Per_Hour) < SYS_IMU_BIAS_PROMOTION_MAX_OVERALL_DEG_H &&
           std::fabs(Request.Max_Window_Slope_Deg_Per_Hour) < SYS_IMU_BIAS_PROMOTION_MAX_WINDOW_DEG_H &&
           Request.Validation_Duration_Ms >= SYS_IMU_BIAS_PROMOTION_MIN_DURATION_MS &&
           Request.Validation_Sample_Count >= SYS_IMU_BIAS_PROMOTION_MIN_SAMPLE_COUNT &&
           Request.EKF_Reset_Delta == 0U &&
           Request.Timestamp_Gap_Count == 0U &&
           Request.Gyro_Outlier_Delta == 0U &&
           Request.Temperature_Outlier_Delta == 0U &&
           Request.Accel_Outlier_Delta == 0U &&
           Request.SPI_Error_Delta == 0U &&
           Request.Flash_Error_Delta == 0U;
}

static bool Snapshot_Promotion_Request(Struct_SYS_IMU_Bias_Promotion_Request &Request)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool pending = IMU_Bias_Promotion_Control.Command_Magic == SYS_IMU_BIAS_PROMOTION_COMMAND_MAGIC;
    if (pending)
    {
        Request.Request_Id = IMU_Bias_Promotion_Control.Request_Id;
        Request.Expected_Candidate_Sequence = IMU_Bias_Promotion_Control.Expected_Candidate_Sequence;
        Request.Validation_Mode_Version = IMU_Bias_Promotion_Control.Validation_Mode_Version;
        Request.Regression_Slope_Deg_Per_Hour = IMU_Bias_Promotion_Control.Regression_Slope_Deg_Per_Hour;
        Request.Endpoint_Drift_Deg_Per_Hour = IMU_Bias_Promotion_Control.Endpoint_Drift_Deg_Per_Hour;
        Request.Max_Window_Slope_Deg_Per_Hour = IMU_Bias_Promotion_Control.Max_Window_Slope_Deg_Per_Hour;
        Request.Validation_Duration_Ms = IMU_Bias_Promotion_Control.Validation_Duration_Ms;
        Request.Validation_Sample_Count = IMU_Bias_Promotion_Control.Validation_Sample_Count;
        Request.EKF_Reset_Delta = IMU_Bias_Promotion_Control.EKF_Reset_Delta;
        Request.Timestamp_Gap_Count = IMU_Bias_Promotion_Control.Timestamp_Gap_Count;
        Request.Gyro_Outlier_Delta = IMU_Bias_Promotion_Control.Gyro_Outlier_Delta;
        Request.Temperature_Outlier_Delta = IMU_Bias_Promotion_Control.Temperature_Outlier_Delta;
        Request.Accel_Outlier_Delta = IMU_Bias_Promotion_Control.Accel_Outlier_Delta;
        Request.SPI_Error_Delta = IMU_Bias_Promotion_Control.SPI_Error_Delta;
        Request.Flash_Error_Delta = IMU_Bias_Promotion_Control.Flash_Error_Delta;
        IMU_Bias_Promotion_Control.Command_Magic = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return pending;
}

static void Set_Promotion_Result(uint32_t Request_Id, uint8_t Result, uint32_t Promoted_Sequence)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    IMU_Bias_Promotion_Control.Promoted_Sequence = Promoted_Sequence;
    __DMB();
    IMU_Bias_Promotion_Control.Result = Result;
    __DMB();
    IMU_Bias_Promotion_Control.Response_Id = Request_Id;
    if (primask == 0U)
    {
        __enable_irq();
    }
    IMU_Bias_Store_Debug.Last_Promotion_Request_Id = Request_Id;
    IMU_Bias_Store_Debug.Promotion_Result = Result;
}

static void Register_Error()
{
    IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_ERROR;
    IMU_Bias_Store_Debug.Error_Count++;
}

void SYS_IMU_Bias_Store_Init()
{
    IMU_Bias_Store_Debug = {};
    IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_LOADING;
    IMU_Bias_Store_Debug.Active_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    IMU_Bias_Store_Debug.Golden_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    IMU_Bias_Store_Debug.Candidate_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    IMU_Bias_Store_Debug.Fixed_Offset_CRC32 = Calculate_Fixed_Offset_CRC32();
    IMU_Bias_Store_Debug.Algorithm_Config_CRC32 = Calculate_Algorithm_Config_CRC32();

    Store_Initialized = false;
    Write_Enabled = false;
    Legacy_Trial_Valid = false;
    Active_Golden_Valid = false;
    Staged_Candidate_Valid = false;
    Legacy_Trial_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    Active_Golden_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    Staged_Candidate_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    Highest_Sequence = 0U;

    const uint32_t error_count = BSP_W25Q64JV.Get_Auto_Polling_Error_Count();
    BSP_W25Q64JV.Enable_Quad_Mode();
    if (BSP_W25Q64JV.Get_Auto_Polling_Error_Count() != error_count)
    {
        Register_Error();
        Store_Initialized = true;
        return;
    }

    const bool slot_a_read = Read_Slot(SYS_IMU_BIAS_STORE_SLOT_A, Slot_State[SYS_IMU_BIAS_STORE_SLOT_A]);
    const bool slot_b_read = Read_Slot(SYS_IMU_BIAS_STORE_SLOT_B, Slot_State[SYS_IMU_BIAS_STORE_SLOT_B]);
    Write_Enabled = slot_a_read && slot_b_read;
    IMU_Bias_Store_Debug.Write_Enabled = static_cast<uint8_t>(Write_Enabled);
    IMU_Bias_Store_Debug.Scan_Degraded = static_cast<uint8_t>(!Write_Enabled);

    for (uint8_t slot = SYS_IMU_BIAS_STORE_SLOT_A; slot <= SYS_IMU_BIAS_STORE_SLOT_B; slot++)
    {
        const Struct_SYS_IMU_Bias_Slot_State &state = Slot_State[slot];
        if (!state.Read_OK)
        {
            continue;
        }

        if (state.Legacy_Valid &&
            (Highest_Sequence == 0U || Sequence_Is_Newer(state.Legacy.Sequence, Highest_Sequence)))
        {
            Highest_Sequence = state.Legacy.Sequence;
        }
        if (state.Candidate_Valid &&
            (Highest_Sequence == 0U || Sequence_Is_Newer(state.V2.Sequence, Highest_Sequence)))
        {
            Highest_Sequence = state.V2.Sequence;
        }

        if (state.Golden_Valid &&
            (!Active_Golden_Valid || Sequence_Is_Newer(state.V2.Sequence, Active_Golden_Record.Sequence)))
        {
            Active_Golden_Record = state.V2;
            Active_Golden_Valid = true;
            Active_Golden_Slot = slot;
        }
        else if (state.Candidate_Valid && !state.Golden_Valid &&
                 (!Staged_Candidate_Valid || Sequence_Is_Newer(state.V2.Sequence, Staged_Candidate_Record.Sequence)))
        {
            Staged_Candidate_Record = state.V2;
            Staged_Candidate_Valid = true;
            Staged_Candidate_Slot = slot;
        }

        if (state.Legacy_Valid && state.Legacy.Sequence == SYS_IMU_BIAS_LEGACY_TRIAL_SEQUENCE)
        {
            Legacy_Trial_Record = state.Legacy;
            Legacy_Trial_Valid = true;
            Legacy_Trial_Slot = slot;
        }
    }

    IMU_Bias_Store_Debug.Highest_Sequence = Highest_Sequence;
    if (Active_Golden_Valid)
    {
        IMU_Bias_Store_Debug.Golden_Available = 1U;
        IMU_Bias_Store_Debug.Golden_Sequence = Active_Golden_Record.Sequence;
        IMU_Bias_Store_Debug.Golden_Slot = Active_Golden_Slot;
        IMU_Bias_Store_Debug.Active_Sequence = Active_Golden_Record.Sequence;
        IMU_Bias_Store_Debug.Active_Slot = Active_Golden_Slot;
        IMU_Bias_Store_Debug.Record_Loaded = 1U;
        if (BSP_BMI088.Load_Persisted_Gyro_Bias(Active_Golden_Record.Bias_Rad_S,
                                                Active_Golden_Record.Sequence,
                                                true,
                                                false))
        {
            IMU_Bias_Store_Debug.Status = Write_Enabled
                                              ? SYS_IMU_BIAS_STORE_GOLDEN_LOADED
                                              : SYS_IMU_BIAS_STORE_READ_DEGRADED;
        }
        else
        {
            Register_Error();
        }
    }
    else if (Legacy_Trial_Valid)
    {
        IMU_Bias_Store_Debug.Candidate_Available = 1U;
        IMU_Bias_Store_Debug.Candidate_Sequence = Legacy_Trial_Record.Sequence;
        IMU_Bias_Store_Debug.Candidate_Slot = Legacy_Trial_Slot;
        IMU_Bias_Store_Debug.Legacy_Trial_Active = 0U;
        IMU_Bias_Store_Debug.Status = Write_Enabled
                                          ? SYS_IMU_BIAS_STORE_CANDIDATE_STAGED
                                          : SYS_IMU_BIAS_STORE_READ_DEGRADED;
    }
    else
    {
        IMU_Bias_Store_Debug.Status = Write_Enabled
                                          ? SYS_IMU_BIAS_STORE_NO_VALID_RECORD
                                          : SYS_IMU_BIAS_STORE_READ_DEGRADED;
    }

    if (Staged_Candidate_Valid)
    {
        IMU_Bias_Store_Debug.Candidate_Available = 1U;
        IMU_Bias_Store_Debug.Candidate_Sequence = Staged_Candidate_Record.Sequence;
        IMU_Bias_Store_Debug.Candidate_Slot = Staged_Candidate_Slot;
    }
    Store_Initialized = true;
}

void SYS_IMU_Bias_Store_Process()
{
    if (!Store_Initialized)
    {
        return;
    }

    const uint32_t fixed_only_command = IMU_Bias_Fixed_Only_Command;
    if (fixed_only_command == SYS_IMU_BIAS_FIXED_ONLY_ENABLE_MAGIC ||
        fixed_only_command == SYS_IMU_BIAS_FIXED_ONLY_DISABLE_MAGIC)
    {
        IMU_Bias_Fixed_Only_Command = 0U;
        __DMB();
        BSP_BMI088.Set_Gyro_Bias_Fixed_Only_Mode(
            fixed_only_command == SYS_IMU_BIAS_FIXED_ONLY_ENABLE_MAGIC);
    }

    const uint32_t maintenance_command = IMU_Bias_Maintenance_Command;
    if (maintenance_command == SYS_IMU_BIAS_MAINTENANCE_ENABLE_MAGIC ||
        maintenance_command == SYS_IMU_BIAS_MAINTENANCE_DISABLE_MAGIC)
    {
        IMU_Bias_Maintenance_Command = 0U;
        __DMB();
        BSP_BMI088.Set_Gyro_Bias_Maintenance_Mode(
            maintenance_command == SYS_IMU_BIAS_MAINTENANCE_ENABLE_MAGIC);
    }

    Struct_SYS_IMU_Bias_Promotion_Request request = {};
    if (!Snapshot_Promotion_Request(request))
    {
        return;
    }

    if (request.Request_Id == IMU_Bias_Promotion_Control.Response_Id)
    {
        Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_REJECTED_REQUEST, 0U);
        return;
    }
    if (BSP_BMI088.Get_Gyro_Bias_Fixed_Only_Mode())
    {
        Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_REJECTED_REQUEST, 0U);
        return;
    }
    Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_IN_PROGRESS, 0U);

    if (!Write_Enabled)
    {
        Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_REJECTED_STORAGE, 0U);
        return;
    }
    if (!Promotion_Request_Is_Valid(request))
    {
        Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_REJECTED_QUALITY, 0U);
        return;
    }

    if (Staged_Candidate_Valid &&
        request.Expected_Candidate_Sequence == Staged_Candidate_Record.Sequence)
    {
        IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_PROMOTING;
        Struct_SYS_IMU_Bias_Record_V2 promoted_record = Staged_Candidate_Record;
        promoted_record.Promotion_Marker = SYS_IMU_BIAS_RECORD_PROMOTION_MARKER;
        if (!Promote_Candidate(Slot_Address(Staged_Candidate_Slot), promoted_record))
        {
            Register_Error();
            Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_COMMIT_FAILED, 0U);
            return;
        }

        Active_Golden_Record = promoted_record;
        Active_Golden_Valid = true;
        Active_Golden_Slot = Staged_Candidate_Slot;
    }
    else
    {
        Struct_BMI088_Gyro_Bias_Commit_Request candidate = {};
        uint32_t source_sequence = 0U;
        uint8_t source_slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
        bool candidate_available = false;

        if (Legacy_Trial_Valid &&
            request.Expected_Candidate_Sequence == Legacy_Trial_Record.Sequence)
        {
            std::memcpy(candidate.Bias_Rad_S, Legacy_Trial_Record.Bias_Rad_S, sizeof(candidate.Bias_Rad_S));
            candidate.Calibration_Temperature_C = Legacy_Trial_Record.Calibration_Temperature_C;
            candidate.Calibration_Sample_Count = Legacy_Trial_Record.Calibration_Sample_Count;
            candidate.Validation_Yaw_Delta_Rad = Legacy_Trial_Record.Validation_Yaw_Delta_Rad;
            source_sequence = Legacy_Trial_Record.Sequence;
            source_slot = Legacy_Trial_Slot;
            candidate_available = true;
        }
        else if (request.Expected_Candidate_Sequence == 0U)
        {
            candidate_available = BSP_BMI088.Get_Gyro_Bias_Promotion_Candidate(candidate);
        }

        if (!candidate_available || !Bias_Is_Sane(candidate.Bias_Rad_S))
        {
            Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_REJECTED_CANDIDATE, 0U);
            return;
        }

        uint32_t next_sequence = Highest_Sequence + 1U;
        if (next_sequence == 0U)
        {
            next_sequence = 1U;
        }

        Struct_SYS_IMU_Bias_Record_V2 next_record = {};
        next_record.Magic = SYS_IMU_BIAS_RECORD_MAGIC;
        next_record.Format_Version = SYS_IMU_BIAS_RECORD_FORMAT_V2;
        next_record.Record_Size = sizeof(Struct_SYS_IMU_Bias_Record_V2);
        next_record.Sequence = next_sequence;
        next_record.Fixed_Offset_CRC32 = IMU_Bias_Store_Debug.Fixed_Offset_CRC32;
        next_record.Algorithm_Config_CRC32 = IMU_Bias_Store_Debug.Algorithm_Config_CRC32;
        std::memcpy(next_record.Bias_Rad_S, candidate.Bias_Rad_S, sizeof(next_record.Bias_Rad_S));
        next_record.Calibration_Temperature_C = candidate.Calibration_Temperature_C;
        next_record.Calibration_Sample_Count = candidate.Calibration_Sample_Count;
        next_record.Source_Sequence = source_sequence;
        next_record.Validation_Mode_Version = SYS_IMU_BIAS_VALIDATION_MODE_VERSION;
        next_record.Record_Role = SYS_IMU_BIAS_RECORD_ROLE_CANDIDATE;
        next_record.Validation_Regression_Slope_Deg_H = request.Regression_Slope_Deg_Per_Hour;
        next_record.Validation_Endpoint_Drift_Deg_H = request.Endpoint_Drift_Deg_Per_Hour;
        next_record.Validation_Max_Window_Slope_Deg_H = request.Max_Window_Slope_Deg_Per_Hour;
        next_record.Validation_Duration_Ms = request.Validation_Duration_Ms;
        next_record.Validation_Sample_Count = request.Validation_Sample_Count;
        next_record.EKF_Reset_Delta = request.EKF_Reset_Delta;
        next_record.Timestamp_Gap_Count = request.Timestamp_Gap_Count;
        next_record.Gyro_Outlier_Delta = request.Gyro_Outlier_Delta;
        next_record.Temperature_Outlier_Delta = request.Temperature_Outlier_Delta;
        next_record.Accel_Outlier_Delta = request.Accel_Outlier_Delta;
        next_record.SPI_Error_Delta = request.SPI_Error_Delta;
        next_record.Flash_Error_Delta = request.Flash_Error_Delta;
        next_record.Payload_CRC32 =
            CRC32_Calculate(&next_record, offsetof(Struct_SYS_IMU_Bias_Record_V2, Payload_CRC32));
        next_record.Commit_Marker = SYS_IMU_BIAS_RECORD_COMMIT_MARKER;
        next_record.Promotion_Marker = SYS_IMU_BIAS_ERASED_WORD;

        uint8_t target_slot = SYS_IMU_BIAS_STORE_SLOT_A;
        if (Active_Golden_Valid)
        {
            target_slot = Active_Golden_Slot == SYS_IMU_BIAS_STORE_SLOT_A
                              ? SYS_IMU_BIAS_STORE_SLOT_B
                              : SYS_IMU_BIAS_STORE_SLOT_A;
        }
        else if (Legacy_Trial_Valid)
        {
            target_slot = Legacy_Trial_Slot == SYS_IMU_BIAS_STORE_SLOT_A
                              ? SYS_IMU_BIAS_STORE_SLOT_B
                              : SYS_IMU_BIAS_STORE_SLOT_A;
        }
        else if (source_slot != SYS_IMU_BIAS_STORE_SLOT_NONE)
        {
            target_slot = source_slot == SYS_IMU_BIAS_STORE_SLOT_A
                              ? SYS_IMU_BIAS_STORE_SLOT_B
                              : SYS_IMU_BIAS_STORE_SLOT_A;
        }

        IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_CANDIDATE_COMMITTING;
        if (!Commit_Candidate(Slot_Address(target_slot), next_record))
        {
            Register_Error();
            Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_COMMIT_FAILED, 0U);
            return;
        }

        Staged_Candidate_Record = next_record;
        Staged_Candidate_Valid = true;
        Staged_Candidate_Slot = target_slot;
        if (Legacy_Trial_Valid && target_slot == Legacy_Trial_Slot)
        {
            Legacy_Trial_Valid = false;
            IMU_Bias_Store_Debug.Legacy_Trial_Active = 0U;
        }
        Highest_Sequence = next_sequence;
        IMU_Bias_Store_Debug.Commit_Count++;
        IMU_Bias_Store_Debug.Candidate_Available = 1U;
        IMU_Bias_Store_Debug.Candidate_Sequence = next_sequence;
        IMU_Bias_Store_Debug.Candidate_Slot = target_slot;
        IMU_Bias_Store_Debug.Highest_Sequence = Highest_Sequence;
        IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_CANDIDATE_STAGED;

        Struct_SYS_IMU_Bias_Record_V2 promoted_record = next_record;
        promoted_record.Promotion_Marker = SYS_IMU_BIAS_RECORD_PROMOTION_MARKER;
        IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_PROMOTING;
        if (!Promote_Candidate(Slot_Address(target_slot), promoted_record))
        {
            Register_Error();
            Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_COMMIT_FAILED, 0U);
            return;
        }

        Active_Golden_Record = promoted_record;
        Active_Golden_Valid = true;
        Active_Golden_Slot = target_slot;
    }

    if (!BSP_BMI088.Load_Persisted_Gyro_Bias(Active_Golden_Record.Bias_Rad_S,
                                             Active_Golden_Record.Sequence,
                                             true,
                                             false))
    {
        Register_Error();
        Set_Promotion_Result(request.Request_Id, SYS_IMU_BIAS_PROMOTION_COMMIT_FAILED, 0U);
        return;
    }

    Staged_Candidate_Valid = false;
    IMU_Bias_Store_Debug.Active_Sequence = Active_Golden_Record.Sequence;
    IMU_Bias_Store_Debug.Active_Slot = Active_Golden_Slot;
    IMU_Bias_Store_Debug.Record_Loaded = 1U;
    IMU_Bias_Store_Debug.Golden_Available = 1U;
    IMU_Bias_Store_Debug.Golden_Sequence = Active_Golden_Record.Sequence;
    IMU_Bias_Store_Debug.Golden_Slot = Active_Golden_Slot;
    IMU_Bias_Store_Debug.Candidate_Available = 0U;
    IMU_Bias_Store_Debug.Candidate_Sequence = 0U;
    IMU_Bias_Store_Debug.Candidate_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    IMU_Bias_Store_Debug.Legacy_Trial_Active = 0U;
    IMU_Bias_Store_Debug.Promotion_Count++;
    IMU_Bias_Store_Debug.Status = SYS_IMU_BIAS_STORE_PROMOTION_OK;

    Set_Promotion_Result(request.Request_Id,
                         SYS_IMU_BIAS_PROMOTION_OK,
                         Active_Golden_Record.Sequence);
}

Struct_SYS_IMU_Bias_Store_Debug SYS_IMU_Bias_Store_Get_Debug()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const Struct_SYS_IMU_Bias_Store_Debug snapshot = IMU_Bias_Store_Debug;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return snapshot;
}
