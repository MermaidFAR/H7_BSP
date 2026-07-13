/**
 * @file sys_imu_bias_store.h
 * @author zzm
 * @brief Power-loss-safe BMI088 gyro bias Golden/Candidate store.
 */

#ifndef __SYS_IMU_BIAS_STORE_H
#define __SYS_IMU_BIAS_STORE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

static constexpr uint32_t SYS_IMU_BIAS_PROMOTION_COMMAND_MAGIC = 0x4d4f5250U;
static constexpr uint32_t SYS_IMU_BIAS_VALIDATION_MODE_VERSION = 0x00010001U;
static constexpr uint32_t SYS_IMU_BIAS_MAINTENANCE_ENABLE_MAGIC = 0x4e454d49U;
static constexpr uint32_t SYS_IMU_BIAS_MAINTENANCE_DISABLE_MAGIC = 0x53494449U;
static constexpr uint32_t SYS_IMU_BIAS_FIXED_ONLY_ENABLE_MAGIC = 0x4e454f46U;
static constexpr uint32_t SYS_IMU_BIAS_FIXED_ONLY_DISABLE_MAGIC = 0x49444f46U;

/* Exported types ------------------------------------------------------------*/

enum Enum_SYS_IMU_Bias_Store_Status : uint8_t
{
    SYS_IMU_BIAS_STORE_UNINITIALIZED = 0U,
    SYS_IMU_BIAS_STORE_LOADING = 1U,
    SYS_IMU_BIAS_STORE_NO_VALID_RECORD = 2U,
    SYS_IMU_BIAS_STORE_GOLDEN_LOADED = 3U,
    SYS_IMU_BIAS_STORE_LEGACY_TRIAL_LOADED = 4U,
    SYS_IMU_BIAS_STORE_CANDIDATE_COMMITTING = 5U,
    SYS_IMU_BIAS_STORE_CANDIDATE_STAGED = 6U,
    SYS_IMU_BIAS_STORE_PROMOTING = 7U,
    SYS_IMU_BIAS_STORE_PROMOTION_OK = 8U,
    SYS_IMU_BIAS_STORE_READ_DEGRADED = 9U,
    SYS_IMU_BIAS_STORE_ERROR = 10U,
};

enum Enum_SYS_IMU_Bias_Store_Slot : uint8_t
{
    SYS_IMU_BIAS_STORE_SLOT_A = 0U,
    SYS_IMU_BIAS_STORE_SLOT_B = 1U,
    SYS_IMU_BIAS_STORE_SLOT_NONE = 0xffU,
};

enum Enum_SYS_IMU_Bias_Promotion_Result : uint8_t
{
    SYS_IMU_BIAS_PROMOTION_IDLE = 0U,
    SYS_IMU_BIAS_PROMOTION_IN_PROGRESS = 1U,
    SYS_IMU_BIAS_PROMOTION_OK = 2U,
    SYS_IMU_BIAS_PROMOTION_REJECTED_REQUEST = 3U,
    SYS_IMU_BIAS_PROMOTION_REJECTED_CANDIDATE = 4U,
    SYS_IMU_BIAS_PROMOTION_REJECTED_QUALITY = 5U,
    SYS_IMU_BIAS_PROMOTION_REJECTED_STORAGE = 6U,
    SYS_IMU_BIAS_PROMOTION_COMMIT_FAILED = 7U,
};

struct Struct_SYS_IMU_Bias_Promotion_Control
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
    uint16_t Reserved;
    uint32_t Command_Magic;
    uint32_t Response_Id;
    uint32_t Result;
    uint32_t Promoted_Sequence;
};

struct Struct_SYS_IMU_Bias_Store_Debug
{
    uint32_t Active_Sequence = 0U;
    uint32_t Fixed_Offset_CRC32 = 0U;
    uint32_t Algorithm_Config_CRC32 = 0U;
    uint32_t Commit_Count = 0U;
    uint32_t Error_Count = 0U;
    uint32_t Promotion_Count = 0U;
    uint32_t Golden_Sequence = 0U;
    uint32_t Candidate_Sequence = 0U;
    uint32_t Highest_Sequence = 0U;
    uint32_t Last_Promotion_Request_Id = 0U;
    uint8_t Status = SYS_IMU_BIAS_STORE_UNINITIALIZED;
    uint8_t Active_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    uint8_t Record_Loaded = 0U;
    uint8_t Golden_Available = 0U;
    uint8_t Candidate_Available = 0U;
    uint8_t Legacy_Trial_Active = 0U;
    uint8_t Write_Enabled = 0U;
    uint8_t Scan_Degraded = 0U;
    uint8_t Golden_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    uint8_t Candidate_Slot = SYS_IMU_BIAS_STORE_SLOT_NONE;
    uint8_t Promotion_Result = SYS_IMU_BIAS_PROMOTION_IDLE;
    uint8_t Reserved = 0U;
};

static_assert(sizeof(Struct_SYS_IMU_Bias_Promotion_Control) == 64U,
              "IMU bias promotion control ABI changed");

/* Exported variables --------------------------------------------------------*/

extern volatile Struct_SYS_IMU_Bias_Promotion_Control IMU_Bias_Promotion_Control;
extern volatile uint32_t IMU_Bias_Maintenance_Command;
extern volatile uint32_t IMU_Bias_Fixed_Only_Command;

/* Exported function declarations --------------------------------------------*/

void SYS_IMU_Bias_Store_Init();
void SYS_IMU_Bias_Store_Process();
Struct_SYS_IMU_Bias_Store_Debug SYS_IMU_Bias_Store_Get_Debug();

#endif /* __SYS_IMU_BIAS_STORE_H */
