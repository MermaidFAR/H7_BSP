/**
 * @file bsp_bmi088.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-26 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "Accel/bsp_bmi088_accel.h"
#include "Gyro/bsp_bmi088_gyro.h"
#include "alg_quaternion.h"
#include "alg_filter_ekf.h"
extern "C" {
#include "cmsis_os2.h"
}
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief BMI088单路数据状态
 *
 */
struct Struct_BMI088_Status
{
    // 数据准备好标志
    bool Ready_Flag = false;
    // 数据传输标志
    bool Transfering_Flag = false;
    // 数据更新标志
    bool Update_Flag = false;

    // 数据准备好时间戳
    uint64_t Ready_Timestamp = 0;
    // 当前传输对应的数据准备好时间戳
    uint64_t Transfer_Ready_Timestamp = 0;
    // DMA传输实际开始时间戳低32位, 仅用于短超时判断
    uint32_t Transfer_Start_Timestamp_Low32 = 0;
    // HAL 已接受本次传输, 可开始执行超时判断
    bool Transfer_Timeout_Armed = false;
    // 数据更新完成时间戳
    uint64_t Update_Timestamp = 0;
    // 已更新数据对应的数据准备好时间戳
    uint64_t Update_Ready_Timestamp = 0;
};

enum Enum_BMI088_Accel_Reject_Reason : uint8_t
{
    BMI088_ACCEL_REJECT_NONE = 0U,
    BMI088_ACCEL_REJECT_INVALID = 1U << 0,
    BMI088_ACCEL_REJECT_NORM = 1U << 1,
    BMI088_ACCEL_REJECT_CHI_SQUARE = 1U << 2,
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

enum Enum_BMI088_Online_Gyro_Bias_Reject_Reason : uint8_t
{
    BMI088_ONLINE_GYRO_BIAS_REJECT_NONE = 0U,
    BMI088_ONLINE_GYRO_BIAS_REJECT_SENSOR_INVALID = 1U << 0,
    BMI088_ONLINE_GYRO_BIAS_REJECT_TEMPERATURE = 1U << 1,
    BMI088_ONLINE_GYRO_BIAS_REJECT_GYRO = 1U << 2,
    BMI088_ONLINE_GYRO_BIAS_REJECT_ACCEL = 1U << 3,
    BMI088_ONLINE_GYRO_BIAS_REJECT_TIMESTAMP = 1U << 4,
    BMI088_ONLINE_GYRO_BIAS_REJECT_DRIFT = 1U << 5,
};

enum Enum_BMI088_Gyro_Bias_Source : uint8_t
{
    BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY = 0U,
    BMI088_GYRO_BIAS_SOURCE_PERSISTED_PROVISIONAL = 1U,
    BMI088_GYRO_BIAS_SOURCE_LIVE_CANDIDATE = 2U,
    BMI088_GYRO_BIAS_SOURCE_LIVE_PRECHECK = 3U,
    BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN = 4U,
};

enum Enum_BMI088_Gyro_Bias_Validation_Result : uint8_t
{
    BMI088_GYRO_BIAS_VALIDATION_IDLE = 0U,
    BMI088_GYRO_BIAS_VALIDATION_RUNNING = 1U,
    BMI088_GYRO_BIAS_VALIDATION_PASSED = 2U,
    BMI088_GYRO_BIAS_VALIDATION_FAILED = 3U,
};

enum Enum_BMI088_Gyro_Bias_Validation_Reject_Reason : uint8_t
{
    BMI088_GYRO_BIAS_VALIDATION_REJECT_NONE = 0U,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_SENSOR_INVALID = 1U << 0,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_TEMPERATURE = 1U << 1,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_GYRO = 1U << 2,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_ACCEL = 1U << 3,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_TIMESTAMP = 1U << 4,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_DRIFT = 1U << 5,
    BMI088_GYRO_BIAS_VALIDATION_REJECT_WINDOW_DRIFT = 1U << 6,
};

enum Enum_BMI088_Accel_Event_State : uint32_t
{
    BMI088_ACCEL_EVENT_ARMED = 0U,
    BMI088_ACCEL_EVENT_POST_TRIGGER = 1U,
    BMI088_ACCEL_EVENT_FROZEN = 2U,
};

static constexpr uint16_t BMI088_ACCEL_EVENT_ABI_VERSION = 1U;
static constexpr uint32_t BMI088_ACCEL_EVENT_FRAME_CAPACITY = 192U;
static constexpr uint32_t BMI088_ACCEL_EVENT_PRE_TRIGGER_FRAMES = 32U;
static constexpr uint32_t BMI088_ACCEL_EVENT_POST_TRIGGER_FRAMES = 128U;

struct Struct_BMI088_Accel_Event_Frame
{
    uint32_t Frame_Sequence;
    uint32_t Accel_Ready_Timestamp_Low32_Us;
    uint32_t SPI_Rx_Timestamp_Low32_Us;
    uint32_t SPI_SR;
    uint32_t RX_DMA_NDTR;
    uint32_t TX_DMA_NDTR;
    uint32_t HAL_Error_Code;
    float Gyro_X_Rad_S;
    float Gyro_Y_Rad_S;
    float Gyro_Z_Rad_S;
    int16_t Accel_Raw_X;
    int16_t Accel_Raw_Y;
    int16_t Accel_Raw_Z;
    uint8_t SPI_Rx_Bytes[8];
    uint8_t Tx_Address;
    uint8_t Tx_Length;
    uint8_t Rx_Length;
    uint8_t Transaction_Active;
    uint8_t Last_Start_Failure_Status;
    uint8_t Reserved[5];
};

struct Struct_BMI088_Accel_Event_Buffer
{
    uint32_t sequence;
    uint16_t Debug_ABI_Version;
    uint16_t Debug_ABI_Size;
    uint32_t Event_Id;
    uint32_t State;
    uint32_t Write_Index;
    uint32_t Valid_Frame_Count;
    uint32_t Trigger_Index;
    uint32_t Start_Index;
    uint32_t Captured_Frame_Count;
    uint32_t Post_Frames_Remaining;
    uint32_t Dropped_Event_Count;
    uint32_t Host_Ack_Event_Id;
    uint32_t Trigger_Reject_Counter;
    uint32_t Trigger_Accel_Frame_Sequence;
    uint32_t Trigger_Timestamp_Low32_Us;
    float Trigger_Accel_Norm_m_s2;
    float Trigger_Accel_X_m_s2;
    float Trigger_Accel_Y_m_s2;
    float Trigger_Accel_Z_m_s2;
    uint8_t Trigger_Reject_Reason;
    uint8_t Reserved[3];
    Struct_BMI088_Accel_Event_Frame Frames[BMI088_ACCEL_EVENT_FRAME_CAPACITY];
    uint32_t sequence_end;
};

static_assert(sizeof(Struct_BMI088_Accel_Event_Frame) == 64U,
              "BMI088 accel event frame ABI changed");

struct Struct_BMI088_Gyro_Bias_Algorithm_Config
{
    uint32_t Version;
    uint32_t Min_Sample_Count;
    uint64_t Soak_Time_us;
    uint64_t Calibration_Time_us;
    uint64_t Sample_Gap_Timeout_us;
    uint64_t Validation_Time_us;
    uint64_t Validation_Window_Time_us;
    uint32_t Validation_Window_Count;
    float Temperature_Min_C;
    float Temperature_Max_C;
    float Persisted_Temperature_Hold_Min_C;
    float Persisted_Temperature_Hold_Max_C;
    float Gyro_Norm_Threshold_rad_s;
    float Accel_Norm_Error_Threshold_m_s2;
    float Accel_Norm_LPF_Time_Constant_s;
    float Validation_Max_Overall_Yaw_Delta_rad;
    float Validation_Max_Window_Yaw_Delta_rad;
};

struct Struct_BMI088_Gyro_Bias_Commit_Request
{
    float Bias_Rad_S[3];
    float Calibration_Temperature_C;
    uint32_t Calibration_Sample_Count;
    float Validation_Yaw_Delta_Rad;
};

static_assert(sizeof(Struct_BMI088_Gyro_Bias_Algorithm_Config) == 88U,
              "BMI088 gyro bias algorithm config ABI changed");
static_assert(sizeof(Struct_BMI088_Gyro_Bias_Commit_Request) == 24U,
              "BMI088 gyro bias commit request ABI changed");

/**
 * @brief Specialized, 板载AHRS
 *
 */
class Class_BMI088
{
public:
    Class_BMI088_Accel BMI088_Accel;
    Class_BMI088_Gyro BMI088_Gyro;

    Class_Filter_EKF<4, 3, 3> EKF_Quaternion;

    void Init();

    inline Class_Matrix_f32<3, 1> Get_Original_Accel() const;

    inline Class_Matrix_f32<3, 1> Get_Original_Gyro() const;

    inline Class_Matrix_f32<3, 1> Get_Fixed_Corrected_Gyro() const;

    inline Class_Matrix_f32<3, 1> Get_Fixed_Gyro_Offset() const;

    inline Class_Matrix_f32<3, 1> Get_Online_Gyro_Bias() const;

    inline Class_Matrix_f32<3, 1> Get_Persisted_Gyro_Bias() const;

    inline Class_Matrix_f32<3, 1> Get_Applied_Gyro_Bias() const;

    inline uint32_t Get_Online_Gyro_Bias_Sample_Count() const;

    inline bool Get_Online_Gyro_Bias_Ready() const;

    inline uint8_t Get_Online_Gyro_Bias_Reject_Reason() const;

    inline uint16_t Get_Online_Gyro_Bias_Reset_Counter() const;

    inline float Get_Online_Gyro_Bias_Soak_Elapsed() const;

    inline float Get_Online_Gyro_Bias_Calibration_Elapsed() const;

    inline Struct_BMI088_Gyro_Bias_Algorithm_Config Get_Gyro_Bias_Algorithm_Config() const;

    inline bool Get_Persisted_Gyro_Bias_Available() const;

    inline uint32_t Get_Persisted_Gyro_Bias_Sequence() const;

    inline bool Get_Persisted_Gyro_Bias_Is_Golden() const;

    inline bool Get_Persisted_Gyro_Bias_Trial_Locked() const;

    inline bool Get_Gyro_Bias_Applied() const;

    inline bool Get_Precision_Ready() const;

    inline uint8_t Get_Gyro_Bias_Source() const;

    inline bool Get_Gyro_Bias_Validation_Active() const;

    inline uint8_t Get_Gyro_Bias_Validation_Result() const;

    inline uint8_t Get_Gyro_Bias_Validation_Reject_Reason() const;

    inline uint16_t Get_Gyro_Bias_Validation_Failure_Counter() const;

    inline float Get_Gyro_Bias_Validation_Elapsed() const;

    inline float Get_Gyro_Bias_Validation_Yaw_Delta() const;

    inline uint8_t Get_Gyro_Bias_Validation_Window_Count() const;

    inline bool Get_Gyro_Bias_Commit_Pending() const;

    inline uint16_t Get_Gyro_Bias_Commit_Error_Counter() const;

    inline bool Get_Gyro_Bias_Promotion_Candidate_Available() const;

    inline bool Get_Gyro_Bias_Candidate_Precheck_Passed() const;

    inline bool Get_Gyro_Bias_Maintenance_Mode() const;

    void Set_Gyro_Bias_Maintenance_Mode(const bool &__Enable);

    inline bool Get_Gyro_Bias_Fixed_Only_Mode() const;

    void Set_Gyro_Bias_Fixed_Only_Mode(const bool &__Enable);

    bool Load_Persisted_Gyro_Bias(const float __Bias_Rad_S[3],
                                  const uint32_t &__Sequence,
                                  const bool &__Is_Golden,
                                  const bool &__Trial_Locked);

    bool Get_Gyro_Bias_Commit_Request(Struct_BMI088_Gyro_Bias_Commit_Request &__Request) const;

    bool Get_Gyro_Bias_Promotion_Candidate(Struct_BMI088_Gyro_Bias_Commit_Request &__Request) const;

    void Set_Gyro_Bias_Commit_Result(const bool &__Success, const uint32_t &__Sequence);

    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const;

    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;

    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const;

    inline Class_Quaternion_f32 Get_Quaternion() const;

    inline Class_Matrix_f32<3, 1> Get_Accel_Body();

    inline Class_Matrix_f32<3, 1> Get_Gyro_Body();

    inline Class_Matrix_f32<3, 1> Get_Accel();

    inline Class_Matrix_f32<3, 1> Get_Gyro();

    inline float Get_Accel_Chi_Square_Loss() const;

    inline float Get_Accel_Norm() const;

    inline bool Get_Accel_Update_Accepted() const;

    inline uint8_t Get_Accel_Reject_Reason() const;

    inline uint32_t Get_Accel_Update_Result() const;

    inline uint32_t Get_Accel_Update_Rejected_Counter() const;

    inline uint32_t Get_Accel_Update_Attempt_Counter() const;

    inline uint32_t Get_Accel_Reject_Invalid_Counter() const;

    inline uint32_t Get_Accel_Reject_Norm_Counter() const;

    inline uint32_t Get_Accel_Reject_Chi_Square_Counter() const;

    inline float Get_Accel_Yaw_Correction() const;

    inline float Get_Accel_Yaw_Correction_Accumulated() const;

    inline uint32_t Get_EKF_Reset_Counter() const;

    inline uint32_t Get_SPI_Recovery_Counter() const;

    inline uint32_t Get_SPI_Transfer_Timeout_Counter() const;

    inline uint32_t Get_SPI_Accel_Timeout_Counter() const;

    inline uint32_t Get_SPI_Gyro_Timeout_Counter() const;

    inline uint32_t Get_SPI_Temperature_Timeout_Counter() const;

    inline uint8_t Get_SPI_Recovery_Last_Reason() const;

    inline bool Get_Gyro_Bias_Temperature_Window_Active() const;

    inline uint32_t Get_Sensor_Ready_Gap_Counter() const;

    inline uint32_t Get_Timestamp_Anomaly_Counter() const;

    inline float Get_D_T() const;

    inline uint64_t Get_Calculating_Time() const;

    void SPI_RxCpltCallback();

    void EXTI_Flag_Callback(uint16_t GPIO_Pin);

    void TIM_128ms_Calculate_PeriodElapsedCallback();

    void TIM_1ms_Service_PeriodElapsedCallback();

    void EKF_Calculate();

    // void TIM_10us_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;

    // 常量

    // 数据传输超时时间, 单位us
    uint32_t TRANSFERING_TIMEOUT = 1000U;

    // D_T超时时间阈值
    float D_T_TIMEOUT_THRESHOLD = 0.1f;

    // 卡方检验残差阈值
    float ACCEL_CHI_SQUARE_TEST_THRESHOLD = 5.0f;

    // 加速度模长与重力加速度的最大允许偏差, 单位m/s²
    float ACCEL_NORM_ERROR_THRESHOLD = 0.8f;

    // 50℃静止在线陀螺仪零偏校准参数
    float ONLINE_GYRO_BIAS_TEMPERATURE_MIN = 49.5f;
    float ONLINE_GYRO_BIAS_TEMPERATURE_MAX = 50.5f;
    float PERSISTED_GYRO_BIAS_TEMPERATURE_HOLD_MIN = 49.25f;
    float PERSISTED_GYRO_BIAS_TEMPERATURE_HOLD_MAX = 50.75f;
    float ONLINE_GYRO_BIAS_GYRO_NORM_THRESHOLD = 0.12f;
    float ONLINE_GYRO_BIAS_ACCEL_NORM_ERROR_THRESHOLD = 2.0f;
    float ONLINE_GYRO_BIAS_ACCEL_NORM_LPF_TIME_CONSTANT = 0.05f;
    uint64_t ONLINE_GYRO_BIAS_SOAK_TIME = 600000000ULL;
    uint64_t ONLINE_GYRO_BIAS_CALIBRATION_TIME = 180000000ULL;
    uint64_t ONLINE_GYRO_BIAS_SAMPLE_GAP_TIMEOUT = 10000ULL;
    uint32_t ONLINE_GYRO_BIAS_MIN_SAMPLE_COUNT = 300000U;
    uint32_t ONLINE_GYRO_BIAS_ALGORITHM_VERSION = 5U;
    uint64_t ONLINE_GYRO_BIAS_VALIDATION_TIME = 600000000ULL;
    uint64_t ONLINE_GYRO_BIAS_VALIDATION_WINDOW_TIME = 120000000ULL;
    uint32_t ONLINE_GYRO_BIAS_VALIDATION_WINDOW_COUNT = 5U;
    float ONLINE_GYRO_BIAS_VALIDATION_MAX_OVERALL_YAW_DELTA = 0.01745329252f;
    float ONLINE_GYRO_BIAS_VALIDATION_MAX_WINDOW_YAW_DELTA = 0.00465421134f;

    // 校正数据, 与温控有关, 温控在50℃
    // 加速度计仿射矩阵源数据
    const float ACCEL_AFFINE_DATA[9] = {0.9813826498493404f, 0.17232440504057203f, 0.027984325323801115f, -0.1690535919907899f, 0.9747302115792275f, -0.10336863799715153f, -0.046825636945091266f, 0.09953521655990044f, 0.986897809387138f};
    // 加速度计偏置
    const float ACCEL_BIAS_DATA[3] = {0.0038458286072392397, 0.00647039594993548f, 0.014968990490337293f};
    // 陀螺仪偏置
    const float GYRO_ZERO_OFFSET[3] = {-0.009049477102f, 0.000886600098f, 0.001653004116f};

    // 内部变量

    // EKF初始化状态完成标志
    bool EKF_Init_Finished_Flag = false;

    // 初始化完成标志
    bool Init_Finished_Flag = false;

    // 数据状态
    Struct_BMI088_Status Accel_Status;
    Struct_BMI088_Status Gyro_Status;
    Struct_BMI088_Status Temperature_Status;
    uint8_t Transfer_Priority_Index = 0;
    volatile bool Transfer_Service_Active = false;
    uint64_t Gyro_FIFO_Last_Fallback_Poll_Timestamp = 0U;

    // 上一次陀螺仪源数据
    Class_Matrix_f32<3, 1> Vector_Pre_Original_Gyro;
    bool Gyro_Integration_Initialized = false;

    // 加速度计归一化数据
    Class_Matrix_f32<3, 1> Vector_Normalized_Accel;
    Class_Matrix_f32<3, 1> Vector_Pending_Accel;
    uint64_t Pending_Accel_Timestamp = 0U;
    bool Accel_Observation_Pending = false;
    bool Pending_Accel_Valid = false;

    // EKF计算时间戳
    uint64_t EKF_Now_Timestamp = 0;
    // 上次EKF计算时间戳
    uint64_t EKF_Pre_Timestamp = 0;

    // 时间差
    float D_T = 0.000125f;

    // 读变量

    // 加速度计源数据
    Class_Matrix_f32<3, 1> Vector_Original_Accel;
    // 陀螺仪源数据
    Class_Matrix_f32<3, 1> Vector_Original_Gyro;
    // 固定零偏补偿后的陀螺仪数据
    Class_Matrix_f32<3, 1> Vector_Fixed_Corrected_Gyro;
    // 启动静止校准得到的在线陀螺仪零偏
    Class_Matrix_f32<3, 1> Vector_Online_Gyro_Bias;
    // Flash 中上次独立验证通过的陀螺仪零偏
    Class_Matrix_f32<3, 1> Vector_Persisted_Gyro_Bias;
    // 当前真正从固定补偿结果中减去的陀螺仪零偏
    Class_Matrix_f32<3, 1> Vector_Applied_Gyro_Bias;
    Class_Matrix_f32<3, 1> Vector_Previous_Applied_Gyro_Bias;

    uint64_t Online_Gyro_Bias_Soak_Elapsed_Time = 0;
    uint64_t Online_Gyro_Bias_Calibration_Elapsed_Time = 0;
    uint64_t Online_Gyro_Bias_Last_Sample_Timestamp = 0;
    uint32_t Online_Gyro_Bias_Sample_Count = 0;
    uint16_t Online_Gyro_Bias_Reset_Counter = 0;
    uint8_t Online_Gyro_Bias_Reject_Reason = BMI088_ONLINE_GYRO_BIAS_REJECT_NONE;
    bool Online_Gyro_Bias_Ready = false;
    bool Online_Gyro_Bias_Accel_Norm_Filter_Initialized = false;
    float Online_Gyro_Bias_Filtered_Accel_Norm = GRAVITY_ACCELERATION;
    float Online_Gyro_Bias_Calibration_Temperature = 0.0f;

    uint32_t Persisted_Gyro_Bias_Sequence = 0U;
    uint8_t Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
    uint8_t Previous_Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
    bool Persisted_Gyro_Bias_Available = false;
    bool Persisted_Gyro_Bias_Is_Golden = false;
    bool Persisted_Gyro_Bias_Trial_Locked = false;
    bool Gyro_Bias_Applied = false;
    bool Previous_Gyro_Bias_Applied = false;
    bool Precision_Ready = false;

    uint64_t Gyro_Bias_Validation_Elapsed_Time = 0U;
    uint64_t Gyro_Bias_Validation_Window_Elapsed_Time = 0U;
    uint64_t Gyro_Bias_Validation_Last_Timestamp = 0U;
    float Gyro_Bias_Validation_Previous_Yaw = 0.0f;
    float Gyro_Bias_Validation_Yaw_Delta = 0.0f;
    float Gyro_Bias_Validation_Window_Yaw_Delta = 0.0f;
    uint8_t Gyro_Bias_Validation_Window_Count = 0U;
    uint16_t Gyro_Bias_Validation_Failure_Counter = 0U;
    uint8_t Gyro_Bias_Validation_Result = BMI088_GYRO_BIAS_VALIDATION_IDLE;
    uint8_t Gyro_Bias_Validation_Reject_Reason = BMI088_GYRO_BIAS_VALIDATION_REJECT_NONE;
    bool Gyro_Bias_Validation_Active = false;

    Struct_BMI088_Gyro_Bias_Commit_Request Pending_Gyro_Bias_Commit = {};
    uint16_t Gyro_Bias_Commit_Error_Counter = 0U;
    bool Gyro_Bias_Commit_Pending = false;
    bool Gyro_Bias_Promotion_Candidate_Available = false;
    bool Gyro_Bias_Candidate_Precheck_Passed = false;
    bool Gyro_Bias_Maintenance_Mode = false;
    // 诊断时可通过命令切换为只使用编译期固定补偿
    bool Gyro_Bias_Fixed_Only_Mode = false;
    bool Gyro_Bias_Temperature_Window_Active = false;

    // 欧拉角, Yaw-Pitch-Roll顺序
    Class_Matrix_f32<3, 1> Vector_Euler_Angle;
    // 旋转矩阵
    Class_Matrix_f32<3, 3> Matrix_Rotation;
    // 轴角式
    Class_Matrix_f32<4, 1> Vector_Axis_Angle;
    // 四元数
    Class_Quaternion_f32 Quarternion;

    // 机体坐标系下的加速度
    Class_Matrix_f32<3, 1> Vector_Accel_Body;
    // 机体坐标系下的角速度
    Class_Matrix_f32<3, 1> Vector_Gyro_Body;
    // 大地坐标系下的加速度
    Class_Matrix_f32<3, 1> Vector_Accel;
    // 大地坐标系下的角速度
    Class_Matrix_f32<3, 1> Vector_Gyro;

    // 卡方检验值
    float Accel_Chi_Square_Loss = 0.0f;
    // 加速度模长
    float Accel_Norm = 0.0f;
    // 最近一次加速度观测结果, bit0为接受标志, bit8..15为拒绝原因
    uint32_t Accel_Update_Result = 0U;
    uint32_t Accel_Update_Attempt_Counter = 0U;
    uint32_t Accel_Update_Rejected_Counter = 0U;
    uint32_t Accel_Reject_Invalid_Counter = 0U;
    uint32_t Accel_Reject_Norm_Counter = 0U;
    uint32_t Accel_Reject_Chi_Square_Counter = 0U;
    // 最近一次加速度更新产生的yaw修正量
    float Accel_Yaw_Correction = 0.0f;
    // 加速度更新产生的yaw累计修正量
    float Accel_Yaw_Correction_Accumulated = 0.0f;
    // EKF运行时重置次数
    uint32_t EKF_Reset_Counter = 0;
    uint32_t SPI_Recovery_Counter = 0;
    uint32_t SPI_Transfer_Timeout_Counter = 0;
    uint32_t SPI_Accel_Timeout_Counter = 0;
    uint32_t SPI_Gyro_Timeout_Counter = 0;
    uint32_t SPI_Temperature_Timeout_Counter = 0;
    volatile uint8_t SPI_Recovery_Pending_Reason = BMI088_SPI_RECOVERY_NONE;
    uint8_t SPI_Recovery_Last_Reason = BMI088_SPI_RECOVERY_NONE;
    uint32_t Sensor_Ready_Gap_Counter = 0;
    uint32_t Timestamp_Anomaly_Counter = 0;
    // 处理时间
    uint64_t Calculating_Time = 0;

    // 写变量

    // 读写变量

    // 内部函数

    // EKF的相关函数
    // 四元数状态转移函数
    static Class_Matrix_f32<4, 1> EKF_Function_F(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数状态转移函数对状态的雅可比矩阵
    static Class_Matrix_f32<4, 4> EKF_Function_Jacobian_F_X(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数状态转移函数对过程噪声的雅可比矩阵
    static Class_Matrix_f32<4, 3> EKF_Function_Jacobian_F_W(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数测量函数
    static Class_Matrix_f32<3, 1> EKF_Function_H(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    // 四元数测量函数对状态的雅可比矩阵
    static Class_Matrix_f32<3, 4> EKF_Function_Jacobian_H_X(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    // 四元数测量函数对测量噪声的雅可比矩阵
    static Class_Matrix_f32<3, 3> EKF_Function_Jacobian_H_V(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    void Accel_Chi_Square_Calculate();
    bool Accel_Norm_Is_Valid() const;
    void Set_Accel_Update_Result(const bool &__Accepted, const uint8_t &__Reject_Reason);
    void Init_Accel_Event_Buffer();
    void Capture_Accel_Event_Frame(const uint64_t &__Accel_Ready_Timestamp);
    void Trigger_Accel_Event(const uint8_t &__Reject_Reason);
    void Reset_Online_Gyro_Bias_Calibration(const uint8_t &__Reject_Reason);
    void Update_Online_Gyro_Bias_Calibration(const uint64_t &__Gyro_Timestamp);
    bool Online_Gyro_Bias_Temperature_Is_Valid(
        const Struct_BMI088_Accel_Temperature_State &__Temperature_State);
    bool Persisted_Gyro_Bias_Temperature_Is_Valid(
        const Struct_BMI088_Accel_Temperature_State &__Temperature_State);
    void Update_Persisted_Gyro_Bias_Application(
        const Struct_BMI088_Accel_Temperature_State &__Temperature_State);
    void Start_Gyro_Bias_Validation();
    void Update_Gyro_Bias_Validation(const uint64_t &__Gyro_Timestamp);
    void Fail_Gyro_Bias_Validation(const uint8_t &__Reject_Reason);
    void EKF_Reset();
    void BMI088_Recover_SPI(uint8_t __Reason);
    void BMI088_Service_Transfer(const bool &Allow_Recovery = false);
    void BMI088_Service_Transfer_Locked(const bool &Allow_Recovery);
    bool EKF_Predict_To_Timestamp(const uint64_t &Timestamp);
    void EKF_Update_With_Accel();
    void EKF_Output_To_Timestamp(const uint64_t &Timestamp);
};

/* Exported variables --------------------------------------------------------*/

extern Class_BMI088 BSP_BMI088;
extern volatile Struct_BMI088_Accel_Event_Buffer Debug_Accel_Event_Buffer;
extern "C" { extern osThreadId_t BMI088TaskHandle; }

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取加速度计原始数据
 *
 * @return 加速度计原始数据, 单位m/s²
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Accel() const
{
    return (Vector_Original_Accel);
}

/**
 * @brief 获取陀螺仪原始数据
 *
 * @return 陀螺仪原始数据, 单位rad/s
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Gyro() const
{
    return (Vector_Original_Gyro);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Fixed_Corrected_Gyro() const
{
    return (Vector_Fixed_Corrected_Gyro);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Fixed_Gyro_Offset() const
{
    return Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Online_Gyro_Bias() const
{
    return (Vector_Online_Gyro_Bias);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Persisted_Gyro_Bias() const
{
    return (Vector_Persisted_Gyro_Bias);
}

inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Applied_Gyro_Bias() const
{
    return (Vector_Applied_Gyro_Bias);
}

inline uint32_t Class_BMI088::Get_Online_Gyro_Bias_Sample_Count() const
{
    return (Online_Gyro_Bias_Sample_Count);
}

inline bool Class_BMI088::Get_Online_Gyro_Bias_Ready() const
{
    return (Online_Gyro_Bias_Ready);
}

inline uint8_t Class_BMI088::Get_Online_Gyro_Bias_Reject_Reason() const
{
    return (Online_Gyro_Bias_Reject_Reason);
}

inline uint16_t Class_BMI088::Get_Online_Gyro_Bias_Reset_Counter() const
{
    return (Online_Gyro_Bias_Reset_Counter);
}

inline float Class_BMI088::Get_Online_Gyro_Bias_Soak_Elapsed() const
{
    return (Online_Gyro_Bias_Soak_Elapsed_Time / 1000000.0f);
}

inline float Class_BMI088::Get_Online_Gyro_Bias_Calibration_Elapsed() const
{
    return (Online_Gyro_Bias_Calibration_Elapsed_Time / 1000000.0f);
}

inline Struct_BMI088_Gyro_Bias_Algorithm_Config Class_BMI088::Get_Gyro_Bias_Algorithm_Config() const
{
    return {
        ONLINE_GYRO_BIAS_ALGORITHM_VERSION,
        ONLINE_GYRO_BIAS_MIN_SAMPLE_COUNT,
        ONLINE_GYRO_BIAS_SOAK_TIME,
        ONLINE_GYRO_BIAS_CALIBRATION_TIME,
        ONLINE_GYRO_BIAS_SAMPLE_GAP_TIMEOUT,
        ONLINE_GYRO_BIAS_VALIDATION_TIME,
        ONLINE_GYRO_BIAS_VALIDATION_WINDOW_TIME,
        ONLINE_GYRO_BIAS_VALIDATION_WINDOW_COUNT,
        ONLINE_GYRO_BIAS_TEMPERATURE_MIN,
        ONLINE_GYRO_BIAS_TEMPERATURE_MAX,
        PERSISTED_GYRO_BIAS_TEMPERATURE_HOLD_MIN,
        PERSISTED_GYRO_BIAS_TEMPERATURE_HOLD_MAX,
        ONLINE_GYRO_BIAS_GYRO_NORM_THRESHOLD,
        ONLINE_GYRO_BIAS_ACCEL_NORM_ERROR_THRESHOLD,
        ONLINE_GYRO_BIAS_ACCEL_NORM_LPF_TIME_CONSTANT,
        ONLINE_GYRO_BIAS_VALIDATION_MAX_OVERALL_YAW_DELTA,
        ONLINE_GYRO_BIAS_VALIDATION_MAX_WINDOW_YAW_DELTA,
    };
}

inline bool Class_BMI088::Get_Persisted_Gyro_Bias_Available() const
{
    return Persisted_Gyro_Bias_Available;
}

inline uint32_t Class_BMI088::Get_Persisted_Gyro_Bias_Sequence() const
{
    return Persisted_Gyro_Bias_Sequence;
}

inline bool Class_BMI088::Get_Persisted_Gyro_Bias_Is_Golden() const
{
    return Persisted_Gyro_Bias_Is_Golden;
}

inline bool Class_BMI088::Get_Persisted_Gyro_Bias_Trial_Locked() const
{
    return Persisted_Gyro_Bias_Trial_Locked;
}

inline bool Class_BMI088::Get_Gyro_Bias_Applied() const
{
    return Gyro_Bias_Applied;
}

inline bool Class_BMI088::Get_Precision_Ready() const
{
    return Precision_Ready;
}

inline uint8_t Class_BMI088::Get_Gyro_Bias_Source() const
{
    return Gyro_Bias_Source;
}

inline bool Class_BMI088::Get_Gyro_Bias_Validation_Active() const
{
    return Gyro_Bias_Validation_Active;
}

inline uint8_t Class_BMI088::Get_Gyro_Bias_Validation_Result() const
{
    return Gyro_Bias_Validation_Result;
}

inline uint8_t Class_BMI088::Get_Gyro_Bias_Validation_Reject_Reason() const
{
    return Gyro_Bias_Validation_Reject_Reason;
}

inline uint16_t Class_BMI088::Get_Gyro_Bias_Validation_Failure_Counter() const
{
    return Gyro_Bias_Validation_Failure_Counter;
}

inline float Class_BMI088::Get_Gyro_Bias_Validation_Elapsed() const
{
    return Gyro_Bias_Validation_Elapsed_Time / 1000000.0f;
}

inline float Class_BMI088::Get_Gyro_Bias_Validation_Yaw_Delta() const
{
    return Gyro_Bias_Validation_Yaw_Delta;
}

inline uint8_t Class_BMI088::Get_Gyro_Bias_Validation_Window_Count() const
{
    return Gyro_Bias_Validation_Window_Count;
}

inline bool Class_BMI088::Get_Gyro_Bias_Commit_Pending() const
{
    return Gyro_Bias_Commit_Pending;
}

inline uint16_t Class_BMI088::Get_Gyro_Bias_Commit_Error_Counter() const
{
    return Gyro_Bias_Commit_Error_Counter;
}

inline bool Class_BMI088::Get_Gyro_Bias_Promotion_Candidate_Available() const
{
    return Gyro_Bias_Promotion_Candidate_Available;
}

inline bool Class_BMI088::Get_Gyro_Bias_Candidate_Precheck_Passed() const
{
    return Gyro_Bias_Candidate_Precheck_Passed;
}

inline bool Class_BMI088::Get_Gyro_Bias_Maintenance_Mode() const
{
    return Gyro_Bias_Maintenance_Mode;
}

inline bool Class_BMI088::Get_Gyro_Bias_Fixed_Only_Mode() const
{
    return Gyro_Bias_Fixed_Only_Mode;
}

/**
 * @brief 获取Euler角
 *
 * @return Euler角, 单位rad
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Euler_Angle() const
{
    return (Vector_Euler_Angle);
}

/**
 * @brief 获取旋转矩阵
 *
 * @return 旋转矩阵
 */
inline Class_Matrix_f32<3, 3> Class_BMI088::Get_Rotation_Matrix() const
{
    return (Matrix_Rotation);
}

/**
 * @brief 获取轴角式的轴
 *
 * @return 轴角式
 */
inline Class_Matrix_f32<4, 1> Class_BMI088::Get_Axis_Angle() const
{
    return (Vector_Axis_Angle);
}

/**
 * @brief 获取四元数
 *
 */
inline Class_Quaternion_f32 Class_BMI088::Get_Quaternion() const
{
    return (Quarternion);
}

/**
 * @brief 获取机体坐标系下的加速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel_Body()
{
    return (Vector_Accel_Body);
}

/**
 * @brief 获取机体坐标系下的角速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro_Body()
{
    return (Vector_Gyro_Body);
}

/**
 * @brief 获取大地坐标系下的加速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel()
{
    return (Vector_Accel);
}

/**
 * @brief 获取大地坐标系下的角速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro()
{
    return (Vector_Gyro);
}

/**
 * @brief 获取加速度计卡方检验残差
 *
 * @return 加速度计卡方检验残差
 */
inline float Class_BMI088::Get_Accel_Chi_Square_Loss() const
{
    return (Accel_Chi_Square_Loss);
}

inline float Class_BMI088::Get_Accel_Norm() const
{
    return (Accel_Norm);
}

inline bool Class_BMI088::Get_Accel_Update_Accepted() const
{
    return ((Accel_Update_Result & 0x1U) != 0U);
}

inline uint8_t Class_BMI088::Get_Accel_Reject_Reason() const
{
    return (static_cast<uint8_t>((Accel_Update_Result >> 8U) & 0xffU));
}

inline uint32_t Class_BMI088::Get_Accel_Update_Result() const
{
    return (Accel_Update_Result);
}

inline uint32_t Class_BMI088::Get_Accel_Update_Rejected_Counter() const
{
    return (Accel_Update_Rejected_Counter);
}

inline uint32_t Class_BMI088::Get_Accel_Update_Attempt_Counter() const
{
    return (Accel_Update_Attempt_Counter);
}

inline uint32_t Class_BMI088::Get_Accel_Reject_Invalid_Counter() const
{
    return (Accel_Reject_Invalid_Counter);
}

inline uint32_t Class_BMI088::Get_Accel_Reject_Norm_Counter() const
{
    return (Accel_Reject_Norm_Counter);
}

inline uint32_t Class_BMI088::Get_Accel_Reject_Chi_Square_Counter() const
{
    return (Accel_Reject_Chi_Square_Counter);
}

inline float Class_BMI088::Get_Accel_Yaw_Correction() const
{
    return (Accel_Yaw_Correction);
}

inline float Class_BMI088::Get_Accel_Yaw_Correction_Accumulated() const
{
    return (Accel_Yaw_Correction_Accumulated);
}

inline uint32_t Class_BMI088::Get_EKF_Reset_Counter() const
{
    return (EKF_Reset_Counter);
}

inline uint32_t Class_BMI088::Get_SPI_Recovery_Counter() const
{
    return (SPI_Recovery_Counter);
}

inline uint32_t Class_BMI088::Get_SPI_Transfer_Timeout_Counter() const
{
    return (SPI_Transfer_Timeout_Counter);
}

inline uint32_t Class_BMI088::Get_SPI_Accel_Timeout_Counter() const
{
    return (SPI_Accel_Timeout_Counter);
}

inline uint32_t Class_BMI088::Get_SPI_Gyro_Timeout_Counter() const
{
    return (SPI_Gyro_Timeout_Counter);
}

inline uint32_t Class_BMI088::Get_SPI_Temperature_Timeout_Counter() const
{
    return (SPI_Temperature_Timeout_Counter);
}

inline uint8_t Class_BMI088::Get_SPI_Recovery_Last_Reason() const
{
    return (SPI_Recovery_Last_Reason);
}

inline bool Class_BMI088::Get_Gyro_Bias_Temperature_Window_Active() const
{
    return (Gyro_Bias_Temperature_Window_Active);
}

inline uint32_t Class_BMI088::Get_Sensor_Ready_Gap_Counter() const
{
    return (Sensor_Ready_Gap_Counter);
}

inline uint32_t Class_BMI088::Get_Timestamp_Anomaly_Counter() const
{
    return (Timestamp_Anomaly_Counter);
}

inline float Class_BMI088::Get_D_T() const
{
    return (D_T);
}

/**
 * @brief 获取计算时间
 *
 * @return 计算时间, 单位s
 */
inline uint64_t Class_BMI088::Get_Calculating_Time() const
{
    return (Calculating_Time);
}

#endif

#ifdef __cplusplus
extern "C" {
#endif

void BMI088_TIM_128ms_Calculate_PeriodElapsedCallback();

void BMI088_TIM_1ms_Service_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
