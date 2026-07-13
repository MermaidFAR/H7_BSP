/**
 * @file bsp_bmi088.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-26 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"


extern "C" { extern osThreadId_t BMI088TaskHandle; }

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_BMI088 BSP_BMI088;
volatile Struct_BMI088_Accel_Event_Buffer Debug_Accel_Event_Buffer
    __attribute__((used, section(".dma_buffer"), aligned(32))) = {};
static uint32_t BMI088_Accel_Event_Frame_Sequence = 0U;

// Ozone graphing: plain global floats for Timeline Data Plot

/* Private function declarations ---------------------------------------------*/

static bool BMI088_Status_Update_Matches(const Struct_BMI088_Status &Status, const Struct_BMI088_Status &Shadow_Status)
{
    return Status.Update_Flag &&
           Shadow_Status.Update_Flag &&
           (Status.Update_Timestamp == Shadow_Status.Update_Timestamp) &&
           (Status.Update_Ready_Timestamp == Shadow_Status.Update_Ready_Timestamp);
}

static uint64_t BMI088_Status_Get_Update_Ready_Timestamp(const Struct_BMI088_Status &Status)
{
    if (Status.Update_Ready_Timestamp != 0)
    {
        return Status.Update_Ready_Timestamp;
    }
    return Status.Ready_Timestamp;
}

static float BMI088_Wrap_Radian(const float &Angle)
{
    float result = Angle;
    if (result > PI)
    {
        result -= 2.0f * PI;
    }
    else if (result < -PI)
    {
        result += 2.0f * PI;
    }
    return result;
}

static uint32_t BMI088_Accel_Event_Begin_Write()
{
    uint32_t sequence = Debug_Accel_Event_Buffer.sequence;
    if ((sequence & 1U) != 0U)
    {
        sequence++;
    }
    sequence++;
    Debug_Accel_Event_Buffer.sequence = sequence;
    Debug_Accel_Event_Buffer.sequence_end = sequence;
    __DMB();
    return sequence;
}

static void BMI088_Accel_Event_End_Write(const uint32_t &Odd_Sequence)
{
    __DMB();
    const uint32_t even_sequence = Odd_Sequence + 1U;
    Debug_Accel_Event_Buffer.sequence_end = even_sequence;
    Debug_Accel_Event_Buffer.sequence = even_sequence;
    __DMB();
}

static void BMI088_Status_Clear_Update_If_Matches(Struct_BMI088_Status &Status, const Struct_BMI088_Status &Shadow_Status)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (BMI088_Status_Update_Matches(Status, Shadow_Status))
    {
        Status.Update_Flag = false;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static bool BMI088_Status_Begin_Transfer(Struct_BMI088_Status &Status,
                                         uint64_t &Transfer_Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!Status.Ready_Flag || Status.Transfering_Flag)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }

    Transfer_Ready_Timestamp = Status.Ready_Timestamp;
    Status.Transfer_Ready_Timestamp = Transfer_Ready_Timestamp;
    Status.Transfer_Start_Timestamp_Low32 = 0U;
    Status.Transfer_Timeout_Armed = false;
    Status.Transfering_Flag = true;
    Status.Ready_Flag = false;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

static void BMI088_Status_Arm_Transfer_Timeout(
    Struct_BMI088_Status &Status, const uint64_t &Transfer_Ready_Timestamp,
    const uint64_t &Transfer_Start_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Status.Transfering_Flag &&
        Status.Transfer_Ready_Timestamp == Transfer_Ready_Timestamp)
    {
        Status.Transfer_Start_Timestamp_Low32 =
            static_cast<uint32_t>(Transfer_Start_Timestamp);
        __DMB();
        Status.Transfer_Timeout_Armed = true;
        __DMB();
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void BMI088_Status_Mark_Ready_If_Clear(
    Struct_BMI088_Status &Status, const uint64_t &Ready_Timestamp);

static void BMI088_Status_Mark_Ready(Struct_BMI088_Status &Status,
                                     const uint64_t &Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Status.Ready_Timestamp = Ready_Timestamp;
    __DMB();
    Status.Ready_Flag = true;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void BMI088_Status_Restore_Ready_After_Start_Failure(
    Struct_BMI088_Status &Status, const uint64_t &Transfer_Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Status.Transfering_Flag = false;
    Status.Transfer_Start_Timestamp_Low32 = 0U;
    Status.Transfer_Timeout_Armed = false;
    Status.Transfer_Ready_Timestamp = 0U;
    if (!Status.Ready_Flag)
    {
        Status.Ready_Timestamp = Transfer_Ready_Timestamp;
        Status.Ready_Flag = true;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static bool BMI088_Status_Restore_Ready_On_Timeout(Struct_BMI088_Status &Status,
                                                   const uint32_t &Now_Timestamp_Low32,
                                                   const uint32_t &Timeout,
                                                   SPI_HandleTypeDef *SPI_Handler)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t elapsed_us = static_cast<uint32_t>(
        Now_Timestamp_Low32 - Status.Transfer_Start_Timestamp_Low32);
    const bool timed_out = Status.Transfering_Flag && Status.Transfer_Timeout_Armed &&
                           elapsed_us >= Timeout;
    if (timed_out)
    {
        SPI_Capture_Timeout_Snapshot(SPI_Handler, elapsed_us);
        const uint64_t transfer_ready_timestamp = Status.Transfer_Ready_Timestamp;
        Status.Transfering_Flag = false;
        Status.Transfer_Start_Timestamp_Low32 = 0U;
        Status.Transfer_Timeout_Armed = false;
        Status.Transfer_Ready_Timestamp = 0U;
        if (!Status.Ready_Flag)
        {
            Status.Ready_Timestamp = transfer_ready_timestamp;
            Status.Ready_Flag = true;
        }
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return timed_out;
}

static void BMI088_Status_Restore_Ready_On_Recovery(Struct_BMI088_Status &Status)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Status.Transfering_Flag && !Status.Ready_Flag)
    {
        Status.Ready_Timestamp = Status.Transfer_Ready_Timestamp;
        Status.Ready_Flag = true;
    }
    Status.Transfering_Flag = false;
    Status.Transfer_Start_Timestamp_Low32 = 0U;
    Status.Transfer_Timeout_Armed = false;
    Status.Transfer_Ready_Timestamp = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void BMI088_Reset_DMA_Handle(DMA_HandleTypeDef *DMA_Handler)
{
    if (DMA_Handler == nullptr)
    {
        return;
    }

    HAL_DMA_Abort(DMA_Handler);
    DMA_Handler->ErrorCode = HAL_DMA_ERROR_NONE;
    DMA_Handler->State = HAL_DMA_STATE_READY;
    DMA_Handler->Lock = HAL_UNLOCKED;
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化BMI088
 *
 */
void Class_BMI088::Init()
{
    Init_Accel_Event_Buffer();
    SPI_Manage_Object = &SPI2_Manage_Object;

    BMI088_Accel.Init(true);
    BMI088_Gyro.Init();

    // 欧拉角需要辅助初始化EKF, 第一次初始化默认Yaw是0
    Vector_Euler_Angle[0][0] = 0.0f;

    Init_Finished_Flag = true;
}

bool Class_BMI088::Load_Persisted_Gyro_Bias(const float __Bias_Rad_S[3],
                                            const uint32_t &__Sequence,
                                            const bool &__Is_Golden,
                                            const bool &__Trial_Locked)
{
    if (__Bias_Rad_S == nullptr || __Sequence == 0U)
    {
        return false;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        if (Basic_Math_Is_Invalid_Float(__Bias_Rad_S[axis]))
        {
            return false;
        }
    }

    const Class_Matrix_f32<3, 1> persisted_bias(__Bias_Rad_S);
    if (persisted_bias.Get_Modulus() >= ONLINE_GYRO_BIAS_GYRO_NORM_THRESHOLD)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Vector_Persisted_Gyro_Bias = persisted_bias;
    Persisted_Gyro_Bias_Sequence = __Sequence;
    Persisted_Gyro_Bias_Available = true;
    Persisted_Gyro_Bias_Is_Golden = __Is_Golden;
    Persisted_Gyro_Bias_Trial_Locked = __Trial_Locked;
    if (__Is_Golden)
    {
        Vector_Online_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
        Online_Gyro_Bias_Soak_Elapsed_Time = 0U;
        Online_Gyro_Bias_Calibration_Elapsed_Time = 0U;
        Online_Gyro_Bias_Last_Sample_Timestamp = 0U;
        Online_Gyro_Bias_Sample_Count = 0U;
        Online_Gyro_Bias_Ready = false;
        Online_Gyro_Bias_Accel_Norm_Filter_Initialized = false;
        Gyro_Bias_Promotion_Candidate_Available = false;
        Gyro_Bias_Candidate_Precheck_Passed = false;
        Gyro_Bias_Maintenance_Mode = false;
        Gyro_Bias_Commit_Pending = false;
        Gyro_Bias_Validation_Window_Elapsed_Time = 0U;
        Gyro_Bias_Validation_Window_Yaw_Delta = 0.0f;
        Gyro_Bias_Validation_Window_Count = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

bool Class_BMI088::Get_Gyro_Bias_Commit_Request(Struct_BMI088_Gyro_Bias_Commit_Request &__Request) const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool pending = Gyro_Bias_Commit_Pending;
    if (pending)
    {
        __Request = Pending_Gyro_Bias_Commit;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return pending;
}

bool Class_BMI088::Get_Gyro_Bias_Promotion_Candidate(Struct_BMI088_Gyro_Bias_Commit_Request &__Request) const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool available = Gyro_Bias_Promotion_Candidate_Available;
    if (available)
    {
        __Request = Pending_Gyro_Bias_Commit;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return available;
}

void Class_BMI088::Set_Gyro_Bias_Maintenance_Mode(const bool &__Enable)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (Gyro_Bias_Maintenance_Mode == __Enable)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    Gyro_Bias_Maintenance_Mode = __Enable;
    Vector_Online_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
    Online_Gyro_Bias_Soak_Elapsed_Time = 0U;
    Online_Gyro_Bias_Calibration_Elapsed_Time = 0U;
    Online_Gyro_Bias_Last_Sample_Timestamp = 0U;
    Online_Gyro_Bias_Sample_Count = 0U;
    Online_Gyro_Bias_Ready = false;
    Online_Gyro_Bias_Accel_Norm_Filter_Initialized = false;
    Gyro_Bias_Promotion_Candidate_Available = false;
    Gyro_Bias_Candidate_Precheck_Passed = false;
    Gyro_Bias_Validation_Elapsed_Time = 0U;
    Gyro_Bias_Validation_Window_Elapsed_Time = 0U;
    Gyro_Bias_Validation_Last_Timestamp = 0U;
    Gyro_Bias_Validation_Yaw_Delta = 0.0f;
    Gyro_Bias_Validation_Window_Yaw_Delta = 0.0f;
    Gyro_Bias_Validation_Window_Count = 0U;
    Gyro_Bias_Validation_Result = BMI088_GYRO_BIAS_VALIDATION_IDLE;
    Gyro_Bias_Validation_Reject_Reason = BMI088_GYRO_BIAS_VALIDATION_REJECT_NONE;
    Gyro_Bias_Validation_Active = false;

    const Struct_BMI088_Accel_Temperature_State temperature_state =
        BMI088_Accel.Get_Temperature_State();
    if (!Gyro_Bias_Fixed_Only_Mode && Persisted_Gyro_Bias_Available &&
        Persisted_Gyro_Bias_Temperature_Is_Valid(temperature_state))
    {
        Vector_Applied_Gyro_Bias = Vector_Persisted_Gyro_Bias;
        Gyro_Bias_Applied = true;
        Gyro_Bias_Source = Persisted_Gyro_Bias_Is_Golden
                               ? BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN
                               : BMI088_GYRO_BIAS_SOURCE_PERSISTED_PROVISIONAL;
        Precision_Ready = Persisted_Gyro_Bias_Is_Golden;
    }
    else
    {
        Vector_Applied_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
        Gyro_Bias_Applied = false;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
        Precision_Ready = false;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Class_BMI088::Set_Gyro_Bias_Fixed_Only_Mode(const bool &__Enable)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    Gyro_Bias_Fixed_Only_Mode = __Enable;
    Vector_Online_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
    Online_Gyro_Bias_Soak_Elapsed_Time = 0U;
    Online_Gyro_Bias_Calibration_Elapsed_Time = 0U;
    Online_Gyro_Bias_Last_Sample_Timestamp = 0U;
    Online_Gyro_Bias_Sample_Count = 0U;
    Online_Gyro_Bias_Ready = false;
    Online_Gyro_Bias_Accel_Norm_Filter_Initialized = false;
    Gyro_Bias_Promotion_Candidate_Available = false;
    Gyro_Bias_Candidate_Precheck_Passed = false;
    Gyro_Bias_Validation_Elapsed_Time = 0U;
    Gyro_Bias_Validation_Window_Elapsed_Time = 0U;
    Gyro_Bias_Validation_Last_Timestamp = 0U;
    Gyro_Bias_Validation_Yaw_Delta = 0.0f;
    Gyro_Bias_Validation_Window_Yaw_Delta = 0.0f;
    Gyro_Bias_Validation_Window_Count = 0U;
    Gyro_Bias_Validation_Result = BMI088_GYRO_BIAS_VALIDATION_IDLE;
    Gyro_Bias_Validation_Reject_Reason = BMI088_GYRO_BIAS_VALIDATION_REJECT_NONE;
    Gyro_Bias_Validation_Active = false;
    Gyro_Bias_Commit_Pending = false;

    if (__Enable)
    {
        Gyro_Bias_Temperature_Window_Active = false;
        Vector_Applied_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
        Gyro_Bias_Applied = false;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
        Precision_Ready = false;
    }
    else if (Persisted_Gyro_Bias_Available &&
             Persisted_Gyro_Bias_Temperature_Is_Valid(
                 BMI088_Accel.Get_Temperature_State()))
    {
        Vector_Applied_Gyro_Bias = Vector_Persisted_Gyro_Bias;
        Gyro_Bias_Applied = true;
        Gyro_Bias_Source = Persisted_Gyro_Bias_Is_Golden
                               ? BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN
                               : BMI088_GYRO_BIAS_SOURCE_PERSISTED_PROVISIONAL;
        Precision_Ready = Persisted_Gyro_Bias_Is_Golden;
    }
    else
    {
        Vector_Applied_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
        Gyro_Bias_Applied = false;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
        Precision_Ready = false;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Class_BMI088::Set_Gyro_Bias_Commit_Result(const bool &__Success, const uint32_t &__Sequence)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (__Success && Gyro_Bias_Commit_Pending && __Sequence != 0U)
    {
        Vector_Persisted_Gyro_Bias = Class_Matrix_f32<3, 1>(Pending_Gyro_Bias_Commit.Bias_Rad_S);
        Persisted_Gyro_Bias_Sequence = __Sequence;
        Persisted_Gyro_Bias_Available = true;
        Persisted_Gyro_Bias_Is_Golden = true;
        Persisted_Gyro_Bias_Trial_Locked = false;
        Gyro_Bias_Commit_Pending = false;
    }
    else if (!__Success && Gyro_Bias_Commit_Error_Counter < 0xffffU)
    {
        Gyro_Bias_Commit_Error_Counter++;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief SPI接收完成回调函数
 *
 */
void Class_BMI088::SPI_RxCpltCallback()
{
    if (SPI_Manage_Object->Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin)
    {
        if (Init_Finished_Flag)
        {
            Struct_BMI088_Status *completed_status = nullptr;
            if (SPI_Manage_Object->Rx_Buffer_Length == 6U)
            {
                completed_status = &Accel_Status;
            }
            else if (SPI_Manage_Object->Rx_Buffer_Length == 2U)
            {
                completed_status = &Temperature_Status;
            }
            if (completed_status == nullptr || !completed_status->Transfering_Flag ||
                completed_status->Transfer_Ready_Timestamp == 0U)
            {
                SPI_Manage_Object->Callback_Anomaly_Count++;
                return;
            }
        }

        BMI088_Accel.SPI_RxCpltCallback();

        if (Init_Finished_Flag)
        {
            if (SPI_Manage_Object->Rx_Buffer_Length == 6)
            {
                Capture_Accel_Event_Frame(Accel_Status.Transfer_Ready_Timestamp);
                Accel_Status.Transfering_Flag = false;
                Accel_Status.Update_Flag = true;
                Accel_Status.Update_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
                Accel_Status.Update_Ready_Timestamp = Accel_Status.Transfer_Ready_Timestamp;
                Accel_Status.Transfer_Ready_Timestamp = 0U;
                Accel_Status.Transfer_Start_Timestamp_Low32 = 0U;
                Accel_Status.Transfer_Timeout_Armed = false;
            }
            else if (SPI_Manage_Object->Rx_Buffer_Length == 2)
            {
                Temperature_Status.Transfering_Flag = false;
                Temperature_Status.Transfer_Ready_Timestamp = 0U;
                Temperature_Status.Transfer_Start_Timestamp_Low32 = 0U;
                Temperature_Status.Transfer_Timeout_Armed = false;
            }
        }
    }
    else if (SPI_Manage_Object->Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {
        if (Init_Finished_Flag &&
            (!Gyro_Status.Transfering_Flag ||
             Gyro_Status.Transfer_Ready_Timestamp == 0U))
        {
            SPI_Manage_Object->Callback_Anomaly_Count++;
            return;
        }

        const uint64_t gyro_ready_timestamp =
            Gyro_Status.Transfer_Ready_Timestamp;
        const uint8_t gyro_result =
            BMI088_Gyro.SPI_RxCallback(gyro_ready_timestamp);

        if (Init_Finished_Flag)
        {
            Gyro_Status.Transfering_Flag = false;
            Gyro_Status.Transfer_Ready_Timestamp = 0U;
            Gyro_Status.Transfer_Start_Timestamp_Low32 = 0U;
            Gyro_Status.Transfer_Timeout_Armed = false;

            if ((gyro_result &
                 BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED) != 0U)
            {
                BMI088_Status_Mark_Ready_If_Clear(
                    Gyro_Status, SYS_Timestamp.Get_Now_Microsecond());
            }
            if ((gyro_result &
                 BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED) != 0U)
            {
                osThreadFlagsSet(BMI088TaskHandle, 0x0001);
            }
        }
    }

    // 不再从 SPI 回调中发起新传输（DMA-in-DMA 竞态），由 EXTI/慢周期统一发起
}

/**
 * @brief EXTI中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void Class_BMI088::EXTI_Flag_Callback(uint16_t GPIO_Pin)
{
    if (!Init_Finished_Flag) return;

    uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();

    // 记录当前传感器数据就绪
    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin)
    {
        BMI088_Status_Mark_Ready(Accel_Status, now_timestamp);
    }
    else if (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BMI088_Gyro.Notify_FIFO_Interrupt(now_timestamp);
        BMI088_Status_Mark_Ready(Gyro_Status, now_timestamp);
    }

    BMI088_Service_Transfer();
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
void Class_BMI088::TIM_128ms_Calculate_PeriodElapsedCallback()
{
    uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();

    BMI088_Status_Mark_Ready(Temperature_Status, now_timestamp);
    BMI088_Service_Transfer(true);
    BMI088_Accel.TIM_128ms_Heater_PID_PeriodElapsedCallback();
}

void Class_BMI088::TIM_1ms_Service_PeriodElapsedCallback()
{
    const uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    if ((now_timestamp - Gyro_FIFO_Last_Fallback_Poll_Timestamp) >= 4000U)
    {
        Gyro_FIFO_Last_Fallback_Poll_Timestamp = now_timestamp;
        BMI088_Status_Mark_Ready_If_Clear(Gyro_Status, now_timestamp);
    }
    BMI088_Service_Transfer(true);
}

void Class_BMI088::BMI088_Recover_SPI(uint8_t __Reason)
{
    SPI_Recovery_Counter++;
    SPI_Recovery_Last_Reason = __Reason;
    if ((__Reason & (BMI088_SPI_RECOVERY_ACCEL_TIMEOUT |
                     BMI088_SPI_RECOVERY_GYRO_TIMEOUT |
                     BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT)) != 0U)
    {
        SPI_Transfer_Timeout_Counter++;
    }
    if ((__Reason & BMI088_SPI_RECOVERY_ACCEL_TIMEOUT) != 0U)
    {
        SPI_Accel_Timeout_Counter++;
    }
    if ((__Reason & BMI088_SPI_RECOVERY_GYRO_TIMEOUT) != 0U)
    {
        SPI_Gyro_Timeout_Counter++;
    }
    if ((__Reason & BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT) != 0U)
    {
        SPI_Temperature_Timeout_Counter++;
    }
    SPI_Recovery_Pending_Reason = BMI088_SPI_RECOVERY_NONE;

    BMI088_Status_Restore_Ready_On_Recovery(Accel_Status);
    BMI088_Status_Restore_Ready_On_Recovery(Gyro_Status);
    BMI088_Status_Restore_Ready_On_Recovery(Temperature_Status);

    if (SPI_Manage_Object == nullptr || SPI_Manage_Object->SPI_Handler == nullptr)
    {
        return;
    }

    SPI_HandleTypeDef *spi_handler = SPI_Manage_Object->SPI_Handler;

    if (SPI_Manage_Object->Activate_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(SPI_Manage_Object->Activate_GPIOx, SPI_Manage_Object->Activate_GPIO_Pin,
                          SPI_Manage_Object->Activate_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    HAL_SPI_Abort(spi_handler);
    BMI088_Reset_DMA_Handle(spi_handler->hdmatx);
    BMI088_Reset_DMA_Handle(spi_handler->hdmarx);
    spi_handler->ErrorCode = HAL_SPI_ERROR_NONE;
    spi_handler->State = HAL_SPI_STATE_READY;
    spi_handler->Lock = HAL_UNLOCKED;
    spi_handler->TxXferCount = 0;
    spi_handler->RxXferCount = 0;
    SPI_Manage_Object->Activate_GPIOx = nullptr;
    SPI_Manage_Object->Tx_Buffer_Length = 0;
    SPI_Manage_Object->Rx_Buffer_Length = 0;
    __DMB();
    SPI_Manage_Object->Transaction_Active = false;
    __DMB();

}

void Class_BMI088::BMI088_Service_Transfer(const bool &Allow_Recovery)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Transfer_Service_Active)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
    Transfer_Service_Active = true;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    BMI088_Service_Transfer_Locked(Allow_Recovery);

    const uint32_t unlock_primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    Transfer_Service_Active = false;
    __DMB();
    if (unlock_primask == 0U)
    {
        __enable_irq();
    }
}

void Class_BMI088::BMI088_Service_Transfer_Locked(const bool &Allow_Recovery)
{
    if (!Init_Finished_Flag)
    {
        return;
    }

    if (Allow_Recovery)
    {
        uint8_t recovery_reason = SPI_Recovery_Pending_Reason;
        const uint32_t now_timestamp_low32 =
            static_cast<uint32_t>(SYS_Timestamp.Get_Now_Microsecond());
        SPI_HandleTypeDef *spi_handler = SPI_Manage_Object != nullptr
                                             ? SPI_Manage_Object->SPI_Handler
                                             : nullptr;
        if (BMI088_Status_Restore_Ready_On_Timeout(Accel_Status, now_timestamp_low32,
                                                   TRANSFERING_TIMEOUT,
                                                   spi_handler))
        {
            recovery_reason |= BMI088_SPI_RECOVERY_ACCEL_TIMEOUT;
        }
        if (BMI088_Status_Restore_Ready_On_Timeout(Gyro_Status, now_timestamp_low32,
                                                   TRANSFERING_TIMEOUT,
                                                   spi_handler))
        {
            recovery_reason |= BMI088_SPI_RECOVERY_GYRO_TIMEOUT;
        }
        if (BMI088_Status_Restore_Ready_On_Timeout(Temperature_Status, now_timestamp_low32,
                                                   TRANSFERING_TIMEOUT,
                                                   spi_handler))
        {
            recovery_reason |= BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT;
        }

        const bool spi_error = (SPI_Manage_Object != nullptr) &&
                               (SPI_Manage_Object->SPI_Handler != nullptr) &&
                               (SPI_Manage_Object->SPI_Handler->ErrorCode != HAL_SPI_ERROR_NONE);
        if (spi_error)
        {
            recovery_reason |= BMI088_SPI_RECOVERY_HAL_ERROR;
        }
        if (recovery_reason != BMI088_SPI_RECOVERY_NONE)
        {
            BMI088_Recover_SPI(recovery_reason);
            return;
        }
    }
    else if (SPI_Recovery_Pending_Reason != BMI088_SPI_RECOVERY_NONE)
    {
        return;
    }

    if (Accel_Status.Transfering_Flag || Gyro_Status.Transfering_Flag || Temperature_Status.Transfering_Flag)
    {
        return;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t index = (Transfer_Priority_Index + i) % 3;

        if (index == 0 && Accel_Status.Ready_Flag)
        {
            uint64_t transfer_ready_timestamp = 0U;
            if (!BMI088_Status_Begin_Transfer(Accel_Status, transfer_ready_timestamp))
            {
                continue;
            }
            uint8_t status = BMI088_Accel.SPI_Request_Accel();
            if (status == HAL_OK)
            {
                BMI088_Status_Arm_Transfer_Timeout(
                    Accel_Status, transfer_ready_timestamp,
                    SYS_Timestamp.Get_Now_Microsecond());
                Transfer_Priority_Index = 1;
            }
            else
            {
                BMI088_Status_Restore_Ready_After_Start_Failure(
                    Accel_Status, transfer_ready_timestamp);
                SPI_Recovery_Pending_Reason |= BMI088_SPI_RECOVERY_ACCEL_START_FAILURE;
                if (Allow_Recovery)
                {
                    BMI088_Recover_SPI(SPI_Recovery_Pending_Reason);
                }
            }
            return;
        }
        else if (index == 1 && Gyro_Status.Ready_Flag)
        {
            uint64_t transfer_ready_timestamp = 0U;
            if (!BMI088_Status_Begin_Transfer(Gyro_Status, transfer_ready_timestamp))
            {
                continue;
            }
            uint8_t status = BMI088_Gyro.SPI_Request_Gyro();
            if (status == HAL_OK)
            {
                BMI088_Status_Arm_Transfer_Timeout(
                    Gyro_Status, transfer_ready_timestamp,
                    SYS_Timestamp.Get_Now_Microsecond());
                Transfer_Priority_Index = 2;
            }
            else
            {
                BMI088_Status_Restore_Ready_After_Start_Failure(
                    Gyro_Status, transfer_ready_timestamp);
                SPI_Recovery_Pending_Reason |= BMI088_SPI_RECOVERY_GYRO_START_FAILURE;
                if (Allow_Recovery)
                {
                    BMI088_Recover_SPI(SPI_Recovery_Pending_Reason);
                }
            }
            return;
        }
        else if (index == 2 && Temperature_Status.Ready_Flag)
        {
            uint64_t transfer_ready_timestamp = 0U;
            if (!BMI088_Status_Begin_Transfer(Temperature_Status, transfer_ready_timestamp))
            {
                continue;
            }
            uint8_t status = BMI088_Accel.SPI_Request_Temperature();
            if (status == HAL_OK)
            {
                BMI088_Status_Arm_Transfer_Timeout(
                    Temperature_Status, transfer_ready_timestamp,
                    SYS_Timestamp.Get_Now_Microsecond());
                Transfer_Priority_Index = 0;
            }
            else
            {
                BMI088_Status_Restore_Ready_After_Start_Failure(
                    Temperature_Status, transfer_ready_timestamp);
                SPI_Recovery_Pending_Reason |= BMI088_SPI_RECOVERY_TEMPERATURE_START_FAILURE;
                if (Allow_Recovery)
                {
                    BMI088_Recover_SPI(SPI_Recovery_Pending_Reason);
                }
            }
            return;
        }
    }
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
void Class_BMI088::EKF_Calculate()
{
    uint64_t calculate_start_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    Struct_BMI088_Gyro_Sample gyro_sample = {};
    if (!BMI088_Gyro.Pop_Sample(gyro_sample))
    {
        return;
    }

    Struct_BMI088_Status shadow_gyro_status = {};
    Struct_BMI088_Status shadow_accel_status = {};

    if (!Accel_Observation_Pending)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const Struct_BMI088_Status accel_status_snapshot = Accel_Status;
        const Class_Matrix_f32<3, 1> accel_snapshot =
            BMI088_Accel.Get_Raw_Accel();
        const bool accel_valid_snapshot = BMI088_Accel.Get_Valid_Flag();
        if (primask == 0U)
        {
            __enable_irq();
        }

        if (accel_status_snapshot.Update_Flag)
        {
            Vector_Pending_Accel = accel_snapshot;
            Pending_Accel_Timestamp =
                BMI088_Status_Get_Update_Ready_Timestamp(accel_status_snapshot);
            Pending_Accel_Valid = accel_valid_snapshot;
            Accel_Observation_Pending = true;
            BMI088_Status_Clear_Update_If_Matches(
                Accel_Status, accel_status_snapshot);
        }
    }

    if (Accel_Observation_Pending)
    {
        shadow_accel_status.Update_Flag = true;
        shadow_accel_status.Update_Timestamp = Pending_Accel_Timestamp;
        shadow_accel_status.Update_Ready_Timestamp = Pending_Accel_Timestamp;
        Vector_Original_Accel = Vector_Pending_Accel;
    }
    else
    {
        Vector_Original_Accel = BMI088_Accel.Get_Raw_Accel();
    }
    const bool accel_observation_valid =
        Accel_Observation_Pending && Pending_Accel_Valid;

    shadow_gyro_status.Update_Flag = true;
    shadow_gyro_status.Update_Timestamp = gyro_sample.Timestamp_Us;
    shadow_gyro_status.Update_Ready_Timestamp = gyro_sample.Timestamp_Us;

    Vector_Original_Gyro[0][0] = gyro_sample.Gyro_Rad_S[0];
    Vector_Original_Gyro[1][0] = gyro_sample.Gyro_Rad_S[1];
    Vector_Original_Gyro[2][0] = gyro_sample.Gyro_Rad_S[2];
    const bool gyro_valid = gyro_sample.Valid != 0U;

    // 如果加热电阻使能, 则进行零偏修正
    if (BMI088_Accel.Get_Heater_Enable())
    {
        Vector_Original_Accel = (Class_Matrix_f32<3, 3>(ACCEL_AFFINE_DATA) * Vector_Original_Accel / GRAVITY_ACCELERATION + Class_Matrix_f32<3, 1>(ACCEL_BIAS_DATA)) * GRAVITY_ACCELERATION;

        if (gyro_valid)
        {
            Vector_Original_Gyro = Vector_Original_Gyro + Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
        }
    }

    if (gyro_valid)
    {
        Vector_Fixed_Corrected_Gyro = Vector_Original_Gyro;
    }
    else
    {
        Vector_Original_Gyro = Vector_Pre_Original_Gyro;
    }

    Accel_Norm = Vector_Original_Accel.Get_Modulus();

    if (shadow_gyro_status.Update_Flag)
    {
        Update_Online_Gyro_Bias_Calibration(BMI088_Status_Get_Update_Ready_Timestamp(shadow_gyro_status));
    }

    if (gyro_valid)
    {
        Vector_Original_Gyro = Vector_Fixed_Corrected_Gyro;
        if (Gyro_Bias_Applied)
        {
            Vector_Original_Gyro = Vector_Original_Gyro - Vector_Applied_Gyro_Bias;
        }
    }

    // 加速度计归一化数据
    if (shadow_accel_status.Update_Flag && accel_observation_valid)
    {
        Vector_Normalized_Accel = Vector_Original_Accel.Get_Normalization();
    }
    else if (shadow_accel_status.Update_Flag)
    {
        Set_Accel_Update_Result(false, BMI088_ACCEL_REJECT_INVALID);
        Accel_Observation_Pending = false;
        shadow_accel_status.Update_Flag = false;
    }


    // EKF初始化与计算
    if (!EKF_Init_Finished_Flag && shadow_accel_status.Update_Flag &&
        accel_observation_valid)
    {
        if (gyro_sample.Timestamp_Us < Pending_Accel_Timestamp)
        {
            return;
        }
        if (!Accel_Norm_Is_Valid())
        {
            Set_Accel_Update_Result(false, BMI088_ACCEL_REJECT_NORM);
            Accel_Observation_Pending = false;
            return;
        }

        // EKF相关变量与函数

        // 过程噪声协方差矩阵
        float array_q[9] = {0.865f, 0.0f, 0.0f, 0.0f, 0.975f, 0.0f, 0.0f, 0.0f, 1.077f};
        Class_Matrix_f32<3, 3> matrix_q(array_q);
        // 测量噪声协方差矩阵
        float array_r[9] = {0.0446f, 0.0f, 0.0f, 0.0f, 0.0476f, 0.0f, 0.0f, 0.0f, 0.0537f};
        Class_Matrix_f32<3, 3> matrix_r(array_r);
        // 初始状态协方差矩阵
        Class_Matrix_f32<4, 4> matrix_p = Namespace_ALG_Matrix::Identity<4, 4>();
        // 初始状态向量
        Class_Matrix_f32<4, 1> vector_x;
        float half_yaw = Vector_Euler_Angle[0][0] * 0.5f;
        float half_roll = atan2f(Vector_Normalized_Accel[1][0], Vector_Normalized_Accel[2][0]) * 0.5f;
        float half_pitch = asinf(-Vector_Normalized_Accel[0][0]) * 0.5f;
        float cy = cosf(half_yaw);
        float sy = sinf(half_yaw);
        float cp = cosf(half_pitch);
        float sp = sinf(half_pitch);
        float cr = cosf(half_roll);
        float sr = sinf(half_roll);
        vector_x[0][0] = cy * cp * cr + sy * sp * sr;
        vector_x[1][0] = cy * cp * sr - sy * sp * cr;
        vector_x[2][0] = sy * cp * sr + cy * sp * cr;
        vector_x[3][0] = sy * cp * cr - cy * sp * sr;
        vector_x = vector_x.Get_Normalization();

        EKF_Quaternion.Init(matrix_q, matrix_r, matrix_p, vector_x);

        EKF_Quaternion.Config_Nonlinear_State_Model(EKF_Function_F, EKF_Function_Jacobian_F_X, EKF_Function_Jacobian_F_W);
        EKF_Quaternion.Config_Nonlinear_Measurement_Model(EKF_Function_H, EKF_Function_Jacobian_H_X, EKF_Function_Jacobian_H_V);

        EKF_Init_Finished_Flag = true;
        Accel_Yaw_Correction = 0.0f;
        Set_Accel_Update_Result(true, BMI088_ACCEL_REJECT_NONE);
        BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
        Accel_Observation_Pending = false;
        EKF_Pre_Timestamp = gyro_sample.Timestamp_Us;
        EKF_Now_Timestamp = EKF_Pre_Timestamp;

        if (gyro_valid)
        {
            Vector_Pre_Original_Gyro = Vector_Original_Gyro;
            Gyro_Integration_Initialized = true;
        }

        return;
    }

    if (EKF_Init_Finished_Flag)
    {
        uint64_t gyro_update_ready_timestamp = BMI088_Status_Get_Update_Ready_Timestamp(shadow_gyro_status);
        uint64_t accel_update_ready_timestamp = BMI088_Status_Get_Update_Ready_Timestamp(shadow_accel_status);

        if (!shadow_gyro_status.Update_Flag && !shadow_accel_status.Update_Flag)
        {
            // 无传感器新数据, 只输出当前时刻外推姿态
        }
        else if (shadow_gyro_status.Update_Flag && !shadow_accel_status.Update_Flag)
        {
            if (!EKF_Predict_To_Timestamp(gyro_update_ready_timestamp))
            {
                EKF_Reset();
                return;
            }
            BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
        }
        else if (!shadow_gyro_status.Update_Flag && shadow_accel_status.Update_Flag)
        {
            // 只有加速度计更新时暂存, 等下一次陀螺仪更新后按时间戳对齐修正
        }
        else
        {
            if (gyro_update_ready_timestamp == accel_update_ready_timestamp)
            {
                if (!EKF_Predict_To_Timestamp(gyro_update_ready_timestamp))
                {
                    EKF_Reset();
                    return;
                }
                EKF_Update_With_Accel();
                Accel_Observation_Pending = false;
                BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
            }
            else if (gyro_update_ready_timestamp < accel_update_ready_timestamp)
            {
                if (!EKF_Predict_To_Timestamp(gyro_update_ready_timestamp))
                {
                    EKF_Reset();
                    return;
                }
                BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
            }
            else
            {
                if (accel_update_ready_timestamp > EKF_Pre_Timestamp)
                {
                    if (!EKF_Predict_To_Timestamp(accel_update_ready_timestamp))
                    {
                        EKF_Reset();
                        return;
                    }
                    EKF_Update_With_Accel();
                }
                Accel_Observation_Pending = false;

                if (!EKF_Predict_To_Timestamp(gyro_update_ready_timestamp))
                {
                    EKF_Reset();
                    return;
                }
                BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
            }
        }

        EKF_Output_To_Timestamp(calculate_start_timestamp);
        if (shadow_gyro_status.Update_Flag)
        {
            Update_Gyro_Bias_Validation(BMI088_Status_Get_Update_Ready_Timestamp(shadow_gyro_status));
        }
        Calculating_Time = SYS_Timestamp.Get_Now_Microsecond() - calculate_start_timestamp;

        if (gyro_valid)
        {
            Vector_Pre_Original_Gyro = Vector_Original_Gyro;
            Gyro_Integration_Initialized = true;
        }
    }
}

bool Class_BMI088::Accel_Norm_Is_Valid() const
{
    return Accel_Norm >= (GRAVITY_ACCELERATION - ACCEL_NORM_ERROR_THRESHOLD) &&
           Accel_Norm <= (GRAVITY_ACCELERATION + ACCEL_NORM_ERROR_THRESHOLD);
}

void Class_BMI088::Init_Accel_Event_Buffer()
{
    memset((void *) &Debug_Accel_Event_Buffer, 0, sizeof(Debug_Accel_Event_Buffer));
    Debug_Accel_Event_Buffer.Debug_ABI_Version = BMI088_ACCEL_EVENT_ABI_VERSION;
    Debug_Accel_Event_Buffer.Debug_ABI_Size = sizeof(Debug_Accel_Event_Buffer);
    Debug_Accel_Event_Buffer.State = BMI088_ACCEL_EVENT_ARMED;
    BMI088_Accel_Event_Frame_Sequence = 0U;
    __DMB();
}

void Class_BMI088::Capture_Accel_Event_Frame(const uint64_t &__Accel_Ready_Timestamp)
{
    if (__Accel_Ready_Timestamp == 0U)
    {
        return;
    }

    if (Debug_Accel_Event_Buffer.State == BMI088_ACCEL_EVENT_FROZEN)
    {
        if (Debug_Accel_Event_Buffer.Event_Id == 0U ||
            Debug_Accel_Event_Buffer.Host_Ack_Event_Id != Debug_Accel_Event_Buffer.Event_Id)
        {
            return;
        }

        const uint32_t reset_sequence = BMI088_Accel_Event_Begin_Write();
        Debug_Accel_Event_Buffer.State = BMI088_ACCEL_EVENT_ARMED;
        Debug_Accel_Event_Buffer.Write_Index = 0U;
        Debug_Accel_Event_Buffer.Valid_Frame_Count = 0U;
        Debug_Accel_Event_Buffer.Trigger_Index = 0U;
        Debug_Accel_Event_Buffer.Start_Index = 0U;
        Debug_Accel_Event_Buffer.Captured_Frame_Count = 0U;
        Debug_Accel_Event_Buffer.Post_Frames_Remaining = 0U;
        Debug_Accel_Event_Buffer.Dropped_Event_Count = 0U;
        Debug_Accel_Event_Buffer.Host_Ack_Event_Id = 0U;
        BMI088_Accel_Event_End_Write(reset_sequence);
    }

    const uint32_t write_sequence = BMI088_Accel_Event_Begin_Write();
    const uint32_t frame_index = Debug_Accel_Event_Buffer.Write_Index;
    volatile Struct_BMI088_Accel_Event_Frame &frame =
        Debug_Accel_Event_Buffer.Frames[frame_index];
    const uint8_t *rx_buffer = SPI_Manage_Object->Rx_Buffer;
    const Class_Matrix_f32<3, 1> gyro = BMI088_Gyro.Get_Raw_Gyro();

    frame.Frame_Sequence = ++BMI088_Accel_Event_Frame_Sequence;
    frame.Accel_Ready_Timestamp_Low32_Us = static_cast<uint32_t>(__Accel_Ready_Timestamp);
    frame.SPI_Rx_Timestamp_Low32_Us = static_cast<uint32_t>(SPI_Manage_Object->Rx_Timestamp);
    frame.SPI_SR = SPI_Manage_Object->SPI_Handler->Instance->SR;
    frame.RX_DMA_NDTR = SPI_Manage_Object->SPI_Handler->hdmarx != nullptr
                            ? __HAL_DMA_GET_COUNTER(SPI_Manage_Object->SPI_Handler->hdmarx)
                            : 0xffffffffU;
    frame.TX_DMA_NDTR = SPI_Manage_Object->SPI_Handler->hdmatx != nullptr
                            ? __HAL_DMA_GET_COUNTER(SPI_Manage_Object->SPI_Handler->hdmatx)
                            : 0xffffffffU;
    frame.HAL_Error_Code = SPI_Manage_Object->SPI_Handler->ErrorCode;
    frame.Gyro_X_Rad_S = gyro[0][0];
    frame.Gyro_Y_Rad_S = gyro[1][0];
    frame.Gyro_Z_Rad_S = gyro[2][0];
    frame.Accel_Raw_X = static_cast<int16_t>(static_cast<uint16_t>(rx_buffer[2]) |
                                              (static_cast<uint16_t>(rx_buffer[3]) << 8U));
    frame.Accel_Raw_Y = static_cast<int16_t>(static_cast<uint16_t>(rx_buffer[4]) |
                                              (static_cast<uint16_t>(rx_buffer[5]) << 8U));
    frame.Accel_Raw_Z = static_cast<int16_t>(static_cast<uint16_t>(rx_buffer[6]) |
                                              (static_cast<uint16_t>(rx_buffer[7]) << 8U));
    for (uint8_t index = 0U; index < 8U; index++)
    {
        frame.SPI_Rx_Bytes[index] = rx_buffer[index];
    }
    frame.Tx_Address = SPI_Manage_Object->Tx_Buffer[0];
    frame.Tx_Length = static_cast<uint8_t>(SPI_Manage_Object->Tx_Buffer_Length);
    frame.Rx_Length = static_cast<uint8_t>(SPI_Manage_Object->Rx_Buffer_Length);
    frame.Transaction_Active = static_cast<uint8_t>(SPI_Manage_Object->Transaction_Active);
    frame.Last_Start_Failure_Status = SPI_Manage_Object->Last_Start_Failure_Status;
    for (uint8_t index = 0U; index < sizeof(frame.Reserved); index++)
    {
        frame.Reserved[index] = 0U;
    }

    Debug_Accel_Event_Buffer.Write_Index =
        (frame_index + 1U) % BMI088_ACCEL_EVENT_FRAME_CAPACITY;
    if (Debug_Accel_Event_Buffer.Valid_Frame_Count < BMI088_ACCEL_EVENT_FRAME_CAPACITY)
    {
        Debug_Accel_Event_Buffer.Valid_Frame_Count++;
    }

    if (Debug_Accel_Event_Buffer.State == BMI088_ACCEL_EVENT_POST_TRIGGER &&
        Debug_Accel_Event_Buffer.Post_Frames_Remaining > 0U)
    {
        Debug_Accel_Event_Buffer.Post_Frames_Remaining--;
        if (Debug_Accel_Event_Buffer.Captured_Frame_Count < BMI088_ACCEL_EVENT_FRAME_CAPACITY)
        {
            Debug_Accel_Event_Buffer.Captured_Frame_Count++;
        }
        if (Debug_Accel_Event_Buffer.Post_Frames_Remaining == 0U)
        {
            Debug_Accel_Event_Buffer.State = BMI088_ACCEL_EVENT_FROZEN;
        }
    }
    BMI088_Accel_Event_End_Write(write_sequence);
}

void Class_BMI088::Trigger_Accel_Event(const uint8_t &__Reject_Reason)
{
    if (!Gyro_Bias_Temperature_Window_Active ||
        Debug_Accel_Event_Buffer.Valid_Frame_Count == 0U ||
        Debug_Accel_Event_Buffer.State == BMI088_ACCEL_EVENT_FROZEN)
    {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t write_sequence = BMI088_Accel_Event_Begin_Write();
    if (Debug_Accel_Event_Buffer.State == BMI088_ACCEL_EVENT_ARMED)
    {
        uint32_t event_id = Debug_Accel_Event_Buffer.Event_Id + 1U;
        if (event_id == 0U)
        {
            event_id = 1U;
        }
        const uint32_t trigger_index =
            (Debug_Accel_Event_Buffer.Write_Index + BMI088_ACCEL_EVENT_FRAME_CAPACITY - 1U) %
            BMI088_ACCEL_EVENT_FRAME_CAPACITY;
        const uint32_t pre_frames = Debug_Accel_Event_Buffer.Valid_Frame_Count > 1U
                                        ? Debug_Accel_Event_Buffer.Valid_Frame_Count - 1U
                                        : 0U;
        const uint32_t bounded_pre_frames =
            pre_frames < BMI088_ACCEL_EVENT_PRE_TRIGGER_FRAMES
                ? pre_frames
                : BMI088_ACCEL_EVENT_PRE_TRIGGER_FRAMES;

        Debug_Accel_Event_Buffer.Event_Id = event_id;
        Debug_Accel_Event_Buffer.State = BMI088_ACCEL_EVENT_POST_TRIGGER;
        Debug_Accel_Event_Buffer.Trigger_Index = trigger_index;
        Debug_Accel_Event_Buffer.Start_Index =
            (trigger_index + BMI088_ACCEL_EVENT_FRAME_CAPACITY - bounded_pre_frames) %
            BMI088_ACCEL_EVENT_FRAME_CAPACITY;
        Debug_Accel_Event_Buffer.Captured_Frame_Count = bounded_pre_frames + 1U;
        Debug_Accel_Event_Buffer.Post_Frames_Remaining =
            BMI088_ACCEL_EVENT_POST_TRIGGER_FRAMES;
        Debug_Accel_Event_Buffer.Dropped_Event_Count = 0U;
        Debug_Accel_Event_Buffer.Trigger_Reject_Counter = Accel_Update_Rejected_Counter;
        Debug_Accel_Event_Buffer.Trigger_Accel_Frame_Sequence =
            Debug_Accel_Event_Buffer.Frames[trigger_index].Frame_Sequence;
        Debug_Accel_Event_Buffer.Trigger_Timestamp_Low32_Us =
            static_cast<uint32_t>(SYS_Timestamp.Get_Current_Timestamp());
        Debug_Accel_Event_Buffer.Trigger_Accel_Norm_m_s2 = Accel_Norm;
        Debug_Accel_Event_Buffer.Trigger_Accel_X_m_s2 = Vector_Original_Accel[0][0];
        Debug_Accel_Event_Buffer.Trigger_Accel_Y_m_s2 = Vector_Original_Accel[1][0];
        Debug_Accel_Event_Buffer.Trigger_Accel_Z_m_s2 = Vector_Original_Accel[2][0];
        Debug_Accel_Event_Buffer.Trigger_Reject_Reason = __Reject_Reason;
    }
    else
    {
        Debug_Accel_Event_Buffer.Dropped_Event_Count++;
    }
    BMI088_Accel_Event_End_Write(write_sequence);
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void BMI088_Status_Mark_Ready_If_Clear(
    Struct_BMI088_Status &Status, const uint64_t &Ready_Timestamp)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!Status.Ready_Flag)
    {
        Status.Ready_Timestamp = Ready_Timestamp;
        __DMB();
        Status.Ready_Flag = true;
        __DMB();
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Class_BMI088::Set_Accel_Update_Result(const bool &__Accepted, const uint8_t &__Reject_Reason)
{
    Accel_Update_Result = (__Accepted ? 1U : 0U) | (static_cast<uint32_t>(__Reject_Reason) << 8U);
    Accel_Update_Attempt_Counter++;
    if (!__Accepted)
    {
        Accel_Update_Rejected_Counter++;
        if ((__Reject_Reason & BMI088_ACCEL_REJECT_INVALID) != 0U)
        {
            Accel_Reject_Invalid_Counter++;
        }
        if ((__Reject_Reason & BMI088_ACCEL_REJECT_NORM) != 0U)
        {
            Accel_Reject_Norm_Counter++;
        }
        if ((__Reject_Reason & BMI088_ACCEL_REJECT_CHI_SQUARE) != 0U)
        {
            Accel_Reject_Chi_Square_Counter++;
        }
        Trigger_Accel_Event(__Reject_Reason);
        Accel_Yaw_Correction = 0.0f;
    }
}

void Class_BMI088::Reset_Online_Gyro_Bias_Calibration(const uint8_t &__Reject_Reason)
{
    if (Online_Gyro_Bias_Reset_Counter < 0xffffU)
    {
        Online_Gyro_Bias_Reset_Counter++;
    }
    Vector_Online_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
    Online_Gyro_Bias_Soak_Elapsed_Time = 0;
    Online_Gyro_Bias_Calibration_Elapsed_Time = 0;
    Online_Gyro_Bias_Last_Sample_Timestamp = 0;
    Online_Gyro_Bias_Sample_Count = 0;
    Online_Gyro_Bias_Reject_Reason = __Reject_Reason;
    Online_Gyro_Bias_Ready = false;
    Online_Gyro_Bias_Accel_Norm_Filter_Initialized = false;
    Online_Gyro_Bias_Filtered_Accel_Norm = GRAVITY_ACCELERATION;
}

void Class_BMI088::Update_Online_Gyro_Bias_Calibration(const uint64_t &__Gyro_Timestamp)
{
    if (Gyro_Bias_Fixed_Only_Mode)
    {
        return;
    }

    const Struct_BMI088_Accel_Temperature_State temperature_state =
        BMI088_Accel.Get_Temperature_State();
    Update_Persisted_Gyro_Bias_Application(temperature_state);

    if ((Persisted_Gyro_Bias_Is_Golden || Persisted_Gyro_Bias_Trial_Locked) &&
        !Gyro_Bias_Maintenance_Mode)
    {
        return;
    }

    if (Online_Gyro_Bias_Ready)
    {
        return;
    }

    if (__Gyro_Timestamp == 0)
    {
        Online_Gyro_Bias_Reject_Reason = BMI088_ONLINE_GYRO_BIAS_REJECT_TIMESTAMP;
        return;
    }

    if (Online_Gyro_Bias_Last_Sample_Timestamp != 0 &&
        __Gyro_Timestamp == Online_Gyro_Bias_Last_Sample_Timestamp)
    {
        return;
    }

    uint64_t valid_interval = 0;
    bool timestamp_gap = false;
    if (Online_Gyro_Bias_Last_Sample_Timestamp != 0)
    {
        if (__Gyro_Timestamp < Online_Gyro_Bias_Last_Sample_Timestamp)
        {
            timestamp_gap = true;
        }
        else
        {
            valid_interval = __Gyro_Timestamp - Online_Gyro_Bias_Last_Sample_Timestamp;
            if (valid_interval > ONLINE_GYRO_BIAS_SAMPLE_GAP_TIMEOUT)
            {
                valid_interval = 0;
                timestamp_gap = true;
            }
        }
    }
    Online_Gyro_Bias_Last_Sample_Timestamp = __Gyro_Timestamp;

    if (!BMI088_Accel.Get_Valid_Flag() || !BMI088_Gyro.Get_Valid_Flag())
    {
        Online_Gyro_Bias_Reject_Reason = BMI088_ONLINE_GYRO_BIAS_REJECT_SENSOR_INVALID;
        return;
    }
    if (!Online_Gyro_Bias_Temperature_Is_Valid(temperature_state))
    {
        Online_Gyro_Bias_Reject_Reason = BMI088_ONLINE_GYRO_BIAS_REJECT_TEMPERATURE;
        return;
    }
    if (Vector_Fixed_Corrected_Gyro.Get_Modulus() >= ONLINE_GYRO_BIAS_GYRO_NORM_THRESHOLD)
    {
        Reset_Online_Gyro_Bias_Calibration(BMI088_ONLINE_GYRO_BIAS_REJECT_GYRO);
        return;
    }

    if (timestamp_gap)
    {
        Online_Gyro_Bias_Reject_Reason = BMI088_ONLINE_GYRO_BIAS_REJECT_TIMESTAMP;
        return;
    }

    if (!Online_Gyro_Bias_Accel_Norm_Filter_Initialized)
    {
        Online_Gyro_Bias_Filtered_Accel_Norm = Accel_Norm;
        Online_Gyro_Bias_Accel_Norm_Filter_Initialized = true;
    }
    else
    {
        const float sample_interval = valid_interval / 1000000.0f;
        const float filter_alpha = sample_interval / (ONLINE_GYRO_BIAS_ACCEL_NORM_LPF_TIME_CONSTANT + sample_interval);
        Online_Gyro_Bias_Filtered_Accel_Norm += filter_alpha * (Accel_Norm - Online_Gyro_Bias_Filtered_Accel_Norm);
    }

    if (fabsf(Online_Gyro_Bias_Filtered_Accel_Norm - GRAVITY_ACCELERATION) > ONLINE_GYRO_BIAS_ACCEL_NORM_ERROR_THRESHOLD)
    {
        Reset_Online_Gyro_Bias_Calibration(BMI088_ONLINE_GYRO_BIAS_REJECT_ACCEL);
        return;
    }

    Online_Gyro_Bias_Reject_Reason = BMI088_ONLINE_GYRO_BIAS_REJECT_NONE;
    if (Online_Gyro_Bias_Soak_Elapsed_Time < ONLINE_GYRO_BIAS_SOAK_TIME)
    {
        uint64_t soak_increment = valid_interval;
        const uint64_t soak_remaining = ONLINE_GYRO_BIAS_SOAK_TIME - Online_Gyro_Bias_Soak_Elapsed_Time;
        if (soak_increment > soak_remaining)
        {
            soak_increment = soak_remaining;
        }
        Online_Gyro_Bias_Soak_Elapsed_Time += soak_increment;
        valid_interval -= soak_increment;

        if (Online_Gyro_Bias_Soak_Elapsed_Time < ONLINE_GYRO_BIAS_SOAK_TIME)
        {
            return;
        }
    }

    if (Online_Gyro_Bias_Sample_Count == 0)
    {
        Online_Gyro_Bias_Sample_Count = 1;
        Vector_Online_Gyro_Bias = Vector_Fixed_Corrected_Gyro;
    }
    else
    {
        Online_Gyro_Bias_Sample_Count++;
        const float sample_weight = 1.0f / static_cast<float>(Online_Gyro_Bias_Sample_Count);
        Vector_Online_Gyro_Bias = Vector_Online_Gyro_Bias +
                                  (Vector_Fixed_Corrected_Gyro - Vector_Online_Gyro_Bias) * sample_weight;
    }

    const uint64_t calibration_remaining = ONLINE_GYRO_BIAS_CALIBRATION_TIME - Online_Gyro_Bias_Calibration_Elapsed_Time;
    Online_Gyro_Bias_Calibration_Elapsed_Time += valid_interval < calibration_remaining ? valid_interval : calibration_remaining;

    if (Online_Gyro_Bias_Calibration_Elapsed_Time >= ONLINE_GYRO_BIAS_CALIBRATION_TIME &&
        Online_Gyro_Bias_Sample_Count >= ONLINE_GYRO_BIAS_MIN_SAMPLE_COUNT)
    {
        Online_Gyro_Bias_Ready = true;
        Online_Gyro_Bias_Calibration_Temperature = temperature_state.Temperature;
        Start_Gyro_Bias_Validation();
    }
}

bool Class_BMI088::Online_Gyro_Bias_Temperature_Is_Valid(
    const Struct_BMI088_Accel_Temperature_State &__Temperature_State)
{
    if (!__Temperature_State.Data_Valid ||
        Basic_Math_Is_Invalid_Float(__Temperature_State.Temperature))
    {
        Gyro_Bias_Temperature_Window_Active = false;
        return false;
    }

    if (Gyro_Bias_Temperature_Window_Active)
    {
        if (__Temperature_State.Temperature < PERSISTED_GYRO_BIAS_TEMPERATURE_HOLD_MIN ||
            __Temperature_State.Temperature > PERSISTED_GYRO_BIAS_TEMPERATURE_HOLD_MAX)
        {
            Gyro_Bias_Temperature_Window_Active = false;
        }
    }
    else if (__Temperature_State.Temperature >= ONLINE_GYRO_BIAS_TEMPERATURE_MIN &&
             __Temperature_State.Temperature <= ONLINE_GYRO_BIAS_TEMPERATURE_MAX)
    {
        Gyro_Bias_Temperature_Window_Active = true;
    }

    return Gyro_Bias_Temperature_Window_Active;
}

bool Class_BMI088::Persisted_Gyro_Bias_Temperature_Is_Valid(
    const Struct_BMI088_Accel_Temperature_State &__Temperature_State)
{
    return Online_Gyro_Bias_Temperature_Is_Valid(__Temperature_State);
}

void Class_BMI088::Update_Persisted_Gyro_Bias_Application(
    const Struct_BMI088_Accel_Temperature_State &__Temperature_State)
{
    if (Gyro_Bias_Fixed_Only_Mode)
    {
        Gyro_Bias_Temperature_Window_Active = false;
        if (Gyro_Bias_Applied ||
            Gyro_Bias_Source != BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY || Precision_Ready)
        {
            Vector_Applied_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
            Gyro_Bias_Applied = false;
            Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
            Precision_Ready = false;
        }
        return;
    }

    const bool temperature_valid =
        Persisted_Gyro_Bias_Temperature_Is_Valid(__Temperature_State);
    const bool learned_source = Gyro_Bias_Source != BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
    if (!temperature_valid && learned_source)
    {
        Vector_Applied_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
        Gyro_Bias_Applied = false;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
        Precision_Ready = false;
        return;
    }

    if (temperature_valid && Persisted_Gyro_Bias_Is_Golden && !Gyro_Bias_Maintenance_Mode &&
        Gyro_Bias_Applied &&
        Gyro_Bias_Source != BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN)
    {
        Vector_Applied_Gyro_Bias = Vector_Persisted_Gyro_Bias;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN;
        Precision_Ready = true;
        return;
    }

    const bool live_precheck_can_reapply = Online_Gyro_Bias_Ready &&
                                           Gyro_Bias_Candidate_Precheck_Passed &&
                                           Gyro_Bias_Promotion_Candidate_Available;
    if (temperature_valid && live_precheck_can_reapply && !Gyro_Bias_Applied &&
        !Gyro_Bias_Validation_Active && !Precision_Ready)
    {
        Vector_Applied_Gyro_Bias = Vector_Online_Gyro_Bias;
        Gyro_Bias_Applied = true;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_LIVE_PRECHECK;
        return;
    }

    const bool persisted_can_reapply = Persisted_Gyro_Bias_Is_Golden || !Online_Gyro_Bias_Ready;
    if (temperature_valid && Persisted_Gyro_Bias_Available && !Gyro_Bias_Applied &&
        !Gyro_Bias_Validation_Active && persisted_can_reapply && !Precision_Ready)
    {
        Vector_Applied_Gyro_Bias = Vector_Persisted_Gyro_Bias;
        Gyro_Bias_Applied = true;
        Gyro_Bias_Source = Persisted_Gyro_Bias_Is_Golden
                               ? BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN
                               : BMI088_GYRO_BIAS_SOURCE_PERSISTED_PROVISIONAL;
        Precision_Ready = Persisted_Gyro_Bias_Is_Golden;
    }
}

void Class_BMI088::Start_Gyro_Bias_Validation()
{
    Vector_Previous_Applied_Gyro_Bias = Vector_Applied_Gyro_Bias;
    Previous_Gyro_Bias_Applied = Gyro_Bias_Applied;
    Previous_Gyro_Bias_Source = Gyro_Bias_Source;

    Vector_Applied_Gyro_Bias = Vector_Online_Gyro_Bias;
    Gyro_Bias_Applied = true;
    Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_LIVE_CANDIDATE;
    Precision_Ready = false;

    Gyro_Bias_Validation_Elapsed_Time = 0U;
    Gyro_Bias_Validation_Window_Elapsed_Time = 0U;
    Gyro_Bias_Validation_Last_Timestamp = 0U;
    Gyro_Bias_Validation_Previous_Yaw = Vector_Euler_Angle[0][0];
    Gyro_Bias_Validation_Yaw_Delta = 0.0f;
    Gyro_Bias_Validation_Window_Yaw_Delta = 0.0f;
    Gyro_Bias_Validation_Window_Count = 0U;
    Gyro_Bias_Validation_Result = BMI088_GYRO_BIAS_VALIDATION_RUNNING;
    Gyro_Bias_Validation_Reject_Reason = BMI088_GYRO_BIAS_VALIDATION_REJECT_NONE;
    Gyro_Bias_Validation_Active = true;
    Gyro_Bias_Candidate_Precheck_Passed = false;
}

void Class_BMI088::Update_Gyro_Bias_Validation(const uint64_t &__Gyro_Timestamp)
{
    if (!Gyro_Bias_Validation_Active)
    {
        return;
    }

    if (!BMI088_Accel.Get_Valid_Flag() || !BMI088_Gyro.Get_Valid_Flag())
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_SENSOR_INVALID);
        return;
    }
    const Struct_BMI088_Accel_Temperature_State temperature_state =
        BMI088_Accel.Get_Temperature_State();
    if (!Online_Gyro_Bias_Temperature_Is_Valid(temperature_state))
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_TEMPERATURE);
        return;
    }
    if (Vector_Original_Gyro.Get_Modulus() >= ONLINE_GYRO_BIAS_GYRO_NORM_THRESHOLD)
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_GYRO);
        return;
    }
    if (fabsf(Accel_Norm - GRAVITY_ACCELERATION) > ONLINE_GYRO_BIAS_ACCEL_NORM_ERROR_THRESHOLD)
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_ACCEL);
        return;
    }
    if (__Gyro_Timestamp == 0U)
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_TIMESTAMP);
        return;
    }

    const float yaw = Vector_Euler_Angle[0][0];
    if (Gyro_Bias_Validation_Last_Timestamp == 0U)
    {
        Gyro_Bias_Validation_Last_Timestamp = __Gyro_Timestamp;
        Gyro_Bias_Validation_Previous_Yaw = yaw;
        return;
    }
    if (__Gyro_Timestamp <= Gyro_Bias_Validation_Last_Timestamp)
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_TIMESTAMP);
        return;
    }

    const uint64_t interval = __Gyro_Timestamp - Gyro_Bias_Validation_Last_Timestamp;
    if (interval > ONLINE_GYRO_BIAS_SAMPLE_GAP_TIMEOUT)
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_TIMESTAMP);
        return;
    }

    Gyro_Bias_Validation_Last_Timestamp = __Gyro_Timestamp;
    const float yaw_delta = BMI088_Wrap_Radian(yaw - Gyro_Bias_Validation_Previous_Yaw);
    Gyro_Bias_Validation_Elapsed_Time += interval;
    Gyro_Bias_Validation_Window_Elapsed_Time += interval;
    Gyro_Bias_Validation_Yaw_Delta += yaw_delta;
    Gyro_Bias_Validation_Window_Yaw_Delta += yaw_delta;
    Gyro_Bias_Validation_Previous_Yaw = yaw;

    if (Gyro_Bias_Validation_Window_Elapsed_Time >= ONLINE_GYRO_BIAS_VALIDATION_WINDOW_TIME)
    {
        if (!(fabsf(Gyro_Bias_Validation_Window_Yaw_Delta) <
              ONLINE_GYRO_BIAS_VALIDATION_MAX_WINDOW_YAW_DELTA))
        {
            Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_WINDOW_DRIFT);
            return;
        }
        Gyro_Bias_Validation_Window_Elapsed_Time -= ONLINE_GYRO_BIAS_VALIDATION_WINDOW_TIME;
        Gyro_Bias_Validation_Window_Yaw_Delta = 0.0f;
        if (Gyro_Bias_Validation_Window_Count < 0xffU)
        {
            Gyro_Bias_Validation_Window_Count++;
        }
    }

    if (Gyro_Bias_Validation_Elapsed_Time < ONLINE_GYRO_BIAS_VALIDATION_TIME)
    {
        return;
    }

    if (Gyro_Bias_Validation_Window_Count != ONLINE_GYRO_BIAS_VALIDATION_WINDOW_COUNT)
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_TIMESTAMP);
        return;
    }

    if (!(fabsf(Gyro_Bias_Validation_Yaw_Delta) <
          ONLINE_GYRO_BIAS_VALIDATION_MAX_OVERALL_YAW_DELTA))
    {
        Fail_Gyro_Bias_Validation(BMI088_GYRO_BIAS_VALIDATION_REJECT_DRIFT);
        return;
    }

    Gyro_Bias_Validation_Active = false;
    Gyro_Bias_Validation_Result = BMI088_GYRO_BIAS_VALIDATION_PASSED;
    Gyro_Bias_Validation_Reject_Reason = BMI088_GYRO_BIAS_VALIDATION_REJECT_NONE;
    Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_LIVE_PRECHECK;
    Precision_Ready = false;
    Gyro_Bias_Candidate_Precheck_Passed = true;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Pending_Gyro_Bias_Commit.Bias_Rad_S[0] = Vector_Online_Gyro_Bias[0][0];
    Pending_Gyro_Bias_Commit.Bias_Rad_S[1] = Vector_Online_Gyro_Bias[1][0];
    Pending_Gyro_Bias_Commit.Bias_Rad_S[2] = Vector_Online_Gyro_Bias[2][0];
    Pending_Gyro_Bias_Commit.Calibration_Temperature_C = Online_Gyro_Bias_Calibration_Temperature;
    Pending_Gyro_Bias_Commit.Calibration_Sample_Count = Online_Gyro_Bias_Sample_Count;
    Pending_Gyro_Bias_Commit.Validation_Yaw_Delta_Rad = Gyro_Bias_Validation_Yaw_Delta;
    Gyro_Bias_Commit_Pending = false;
    Gyro_Bias_Promotion_Candidate_Available = true;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Class_BMI088::Fail_Gyro_Bias_Validation(const uint8_t &__Reject_Reason)
{
    Gyro_Bias_Validation_Active = false;
    Gyro_Bias_Validation_Result = BMI088_GYRO_BIAS_VALIDATION_FAILED;
    Gyro_Bias_Validation_Reject_Reason = __Reject_Reason;
    Gyro_Bias_Candidate_Precheck_Passed = false;
    if (Gyro_Bias_Validation_Failure_Counter < 0xffffU)
    {
        Gyro_Bias_Validation_Failure_Counter++;
    }

    const bool previous_persisted = Previous_Gyro_Bias_Source == BMI088_GYRO_BIAS_SOURCE_PERSISTED_PROVISIONAL ||
                                    Previous_Gyro_Bias_Source == BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN;
    const Struct_BMI088_Accel_Temperature_State temperature_state =
        BMI088_Accel.Get_Temperature_State();
    const bool persisted_temperature_valid = previous_persisted &&
        Persisted_Gyro_Bias_Temperature_Is_Valid(temperature_state);
    const bool restore_persisted = Previous_Gyro_Bias_Applied && previous_persisted &&
                                    persisted_temperature_valid;
    if (Previous_Gyro_Bias_Applied &&
        (!previous_persisted || restore_persisted))
    {
        Vector_Applied_Gyro_Bias = Vector_Previous_Applied_Gyro_Bias;
        Gyro_Bias_Applied = true;
        Gyro_Bias_Source = Previous_Gyro_Bias_Source;
    }
    else
    {
        Vector_Applied_Gyro_Bias = Namespace_ALG_Matrix::Zero<3, 1>();
        Gyro_Bias_Applied = false;
        Gyro_Bias_Source = BMI088_GYRO_BIAS_SOURCE_FIXED_ONLY;
    }
    Precision_Ready = Gyro_Bias_Applied &&
                      Gyro_Bias_Source == BMI088_GYRO_BIAS_SOURCE_PERSISTED_GOLDEN &&
                      persisted_temperature_valid;
    Reset_Online_Gyro_Bias_Calibration(__Reject_Reason);
}

void Class_BMI088::EKF_Reset()
{
    EKF_Init_Finished_Flag = false;
    Gyro_Integration_Initialized = false;
    EKF_Reset_Counter++;
}

bool Class_BMI088::EKF_Predict_To_Timestamp(const uint64_t &Timestamp)
{
    if (Timestamp <= EKF_Pre_Timestamp)
    {
        Timestamp_Anomaly_Counter++;
        EKF_Now_Timestamp = EKF_Pre_Timestamp;
        D_T = 0.0f;
        return true;
    }

    D_T = (Timestamp - EKF_Pre_Timestamp) / 1000000.0f;
    if (D_T <= 0.0f || D_T > D_T_TIMEOUT_THRESHOLD)
    {
        if (D_T > D_T_TIMEOUT_THRESHOLD)
        {
            Sensor_Ready_Gap_Counter++;
        }
        return false;
    }

    EKF_Quaternion.Set_D_T(D_T);
    EKF_Quaternion.Vector_U = Gyro_Integration_Initialized
                                  ? (Vector_Pre_Original_Gyro +
                                     Vector_Original_Gyro) * 0.5f
                                  : Vector_Original_Gyro;
    EKF_Quaternion.TIM_Predict_PeriodElapsedCallback();
    EKF_Quaternion.Vector_X = EKF_Quaternion.Vector_X.Get_Normalization();

    EKF_Pre_Timestamp = Timestamp;
    EKF_Now_Timestamp = Timestamp;

    return true;
}

void Class_BMI088::EKF_Update_With_Accel()
{
    uint8_t accel_reject_reason = BMI088_ACCEL_REJECT_NONE;
    Accel_Chi_Square_Calculate();
    if (!Accel_Norm_Is_Valid())
    {
        accel_reject_reason |= BMI088_ACCEL_REJECT_NORM;
    }
    if (!(Accel_Chi_Square_Loss <= ACCEL_CHI_SQUARE_TEST_THRESHOLD))
    {
        accel_reject_reason |= BMI088_ACCEL_REJECT_CHI_SQUARE;
    }
    if (accel_reject_reason != BMI088_ACCEL_REJECT_NONE)
    {
        Set_Accel_Update_Result(false, accel_reject_reason);
        return;
    }

    const Class_Quaternion_f32 quaternion_predicted(EKF_Quaternion.Vector_X);
    const float predicted_yaw = quaternion_predicted.Get_Euler_Angle()[0][0];

    EKF_Quaternion.Vector_Z = Vector_Normalized_Accel;
    EKF_Quaternion.TIM_Update_PeriodElapsedCallback();
    EKF_Quaternion.Vector_X = EKF_Quaternion.Vector_X.Get_Normalization();

    const Class_Quaternion_f32 quaternion_updated(EKF_Quaternion.Vector_X);
    const float updated_yaw = quaternion_updated.Get_Euler_Angle()[0][0];
    Accel_Yaw_Correction = BMI088_Wrap_Radian(updated_yaw - predicted_yaw);
    Accel_Yaw_Correction_Accumulated += Accel_Yaw_Correction;

    const Class_Quaternion_f32 yaw_correction_quaternion = Namespace_ALG_Quaternion::From_Axis_Angle(
        Namespace_ALG_Matrix::Axis_Z_3d(), -Accel_Yaw_Correction);
    Class_Matrix_f32<4, 4> matrix_yaw_correction = yaw_correction_quaternion.Get_Self_Matrix();
    EKF_Quaternion.Vector_X = (yaw_correction_quaternion * quaternion_updated).Get_Normalization();
    EKF_Quaternion.Matrix_P = matrix_yaw_correction * EKF_Quaternion.Matrix_P * matrix_yaw_correction.Get_Transpose();

    Set_Accel_Update_Result(true, BMI088_ACCEL_REJECT_NONE);
}

void Class_BMI088::EKF_Output_To_Timestamp(const uint64_t &Timestamp)
{
    float output_d_t = 0.0f;
    if (Timestamp > EKF_Pre_Timestamp)
    {
        output_d_t = (Timestamp - EKF_Pre_Timestamp) / 1000000.0f;
        if (output_d_t > D_T_TIMEOUT_THRESHOLD)
        {
            output_d_t = 0.0f;
        }
    }

    if (output_d_t > 0.0f)
    {
        Quarternion = EKF_Function_F(EKF_Quaternion.Vector_X, Vector_Original_Gyro, output_d_t).Get_Normalization();
    }
    else
    {
        Quarternion = EKF_Quaternion.Vector_X.Get_Normalization();
    }

    Vector_Euler_Angle = Quarternion.Get_Euler_Angle();
    Matrix_Rotation = Quarternion.Get_Rotation_Matrix();
    Vector_Axis_Angle = Quarternion.Get_Axis_Angle();

    Class_Matrix_f32<3, 1> vector_gravity_body = Matrix_Rotation.Get_Transpose() * (-Namespace_ALG_Matrix::Axis_Z_3d() * GRAVITY_ACCELERATION);

    Vector_Accel_Body = Vector_Original_Accel + vector_gravity_body;
    Vector_Accel = Matrix_Rotation * Vector_Accel_Body;
    Vector_Gyro_Body = Vector_Original_Gyro;
    Vector_Gyro = Matrix_Rotation * Vector_Gyro_Body;
}

/**
 * @brief 定时器周期中断回调函数
 * @note 已废弃,原因过高频率中断打断MCU影响RTOS使用,实际实现极为优雅,仅是不兼容RTOS
 */
// void Class_BMI088::TIM_10us_Calculate_PeriodElapsedCallback()
// {
//     if (Init_Finished_Flag && Accel_Ready_Flag && !Accel_Transfering_Flag && !Gyro_Transfering_Flag && !Temperature_Transfering_Flag)
//     {
//         // 数据准备好, 读取加速度计
//         Accel_Transfering_Flag = true;
//         Accel_Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
//         BMI088_Accel.SPI_Request_Accel();
//         Accel_Ready_Flag = false;
//         return;
//     }
//
//     if (Init_Finished_Flag && Gyro_Ready_Flag && !Accel_Transfering_Flag && !Gyro_Transfering_Flag && !Temperature_Transfering_Flag)
//     {
//         // 数据准备好, 读取陀螺仪
//         Gyro_Transfering_Flag = true;
//         Gyro_Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
//         BMI088_Gyro.SPI_Request_Gyro();
//         Gyro_Ready_Flag = false;
//         return;
//     }
//
//     if (Init_Finished_Flag && Temperature_Ready_Flag && !Accel_Transfering_Flag && !Gyro_Transfering_Flag && !Temperature_Transfering_Flag)
//     {
//         // 温度准备好, 读取温度
//         Temperature_Transfering_Flag = true;
//         Temperature_Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
//         BMI088_Accel.SPI_Request_Temperature();
//         Temperature_Ready_Flag = false;
//         return;
//     }
//
//     if (Init_Finished_Flag && Accel_Transfering_Flag && (SYS_Timestamp.Get_Now_Microsecond() - Accel_Transfering_Timestamp) >= TRANSFERING_TIMEOUT)
//     {
//         // 加速度计传输超时
//         Accel_Transfering_Flag = true;
//         Accel_Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
//         BMI088_Accel.SPI_Request_Accel();
//         Accel_Ready_Flag = false;
//         return;
//     }
//
//     if (Init_Finished_Flag && Gyro_Transfering_Flag && (SYS_Timestamp.Get_Now_Microsecond() - Gyro_Transfering_Timestamp) >= TRANSFERING_TIMEOUT)
//     {
//         // 陀螺仪传输超时
//         Gyro_Transfering_Flag = true;
//         Gyro_Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
//         BMI088_Gyro.SPI_Request_Gyro();
//         Gyro_Ready_Flag = false;
//         return;
//     }
//
//     if (Init_Finished_Flag && Temperature_Transfering_Flag && (SYS_Timestamp.Get_Now_Microsecond() - Temperature_Transfering_Timestamp) >= TRANSFERING_TIMEOUT)
//     {
//         // 温度传输超时
//         Temperature_Transfering_Flag = true;
//         Temperature_Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
//         BMI088_Accel.SPI_Request_Temperature();
//         Temperature_Ready_Flag = false;
//         return;
//     }
// }

/**
 * @brief 四元数状态转移函数
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 1> Class_BMI088::EKF_Function_F(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 1> matrix_result;

    // 角速度矩阵
    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];
    matrix_omega[3][3] = 0.0f;

    const float omega_norm = Vector_U.Get_Modulus();
    const float half_angle = 0.5f * omega_norm * D_T;
    const float scalar_coefficient = cosf(half_angle);
    const float omega_coefficient = omega_norm > 1.0e-6f
                                        ? sinf(half_angle) / omega_norm
                                        : 0.5f * D_T;
    matrix_result = scalar_coefficient * Vector_X +
                    omega_coefficient * matrix_omega * Vector_X;

    return matrix_result;
}

/**
 * @brief 四元数状态转移函数对状态的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 4> Class_BMI088::EKF_Function_Jacobian_F_X(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 4> matrix_result;

    // 角速度矩阵
    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];

    const float omega_norm = Vector_U.Get_Modulus();
    const float half_angle = 0.5f * omega_norm * D_T;
    const float scalar_coefficient = cosf(half_angle);
    const float omega_coefficient = omega_norm > 1.0e-6f
                                        ? sinf(half_angle) / omega_norm
                                        : 0.5f * D_T;
    matrix_result = scalar_coefficient * Namespace_ALG_Matrix::Identity<4, 4>() +
                    omega_coefficient * matrix_omega;

    return matrix_result;
}

/**
 * @brief 四元数状态转移函数对过程噪声的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 3> Class_BMI088::EKF_Function_Jacobian_F_W(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 3> matrix_result;

    // 四元数矩阵
    Class_Matrix_f32<4, 3> matrix_q;
    matrix_q[0][0] = -Vector_X[1][0];
    matrix_q[0][1] = -Vector_X[2][0];
    matrix_q[0][2] = -Vector_X[3][0];
    matrix_q[1][0] = Vector_X[0][0];
    matrix_q[1][1] = -Vector_X[3][0];
    matrix_q[1][2] = Vector_X[2][0];
    matrix_q[2][0] = Vector_X[3][0];
    matrix_q[2][1] = Vector_X[0][0];
    matrix_q[2][2] = -Vector_X[1][0];
    matrix_q[3][0] = -Vector_X[2][0];
    matrix_q[3][1] = Vector_X[1][0];
    matrix_q[3][2] = Vector_X[0][0];

    matrix_result = 0.5f * D_T * matrix_q;

    return matrix_result;
}

/**
 * @brief 四元数测量函数
 *
 * @param Vector_X 状态向量
 */
Class_Matrix_f32<3, 1> Class_BMI088::EKF_Function_H(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    Class_Matrix_f32<3, 1> matrix_result;

    matrix_result[0][0] = 2.0f * (Vector_X[1][0] * Vector_X[3][0] - Vector_X[0][0] * Vector_X[2][0]);
    matrix_result[1][0] = 2.0f * (Vector_X[2][0] * Vector_X[3][0] + Vector_X[0][0] * Vector_X[1][0]);
    matrix_result[2][0] = Vector_X[0][0] * Vector_X[0][0] - Vector_X[1][0] * Vector_X[1][0] - Vector_X[2][0] * Vector_X[2][0] + Vector_X[3][0] * Vector_X[3][0];

    return matrix_result;
}

/**
 * @brief 四元数测量函数对状态的雅可比矩阵
 *
 * @param Vector_X 状态向量
 */
Class_Matrix_f32<3, 4> Class_BMI088::EKF_Function_Jacobian_H_X(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    Class_Matrix_f32<3, 4> matrix_result;

    matrix_result[0][0] = -2.0f * Vector_X[2][0];
    matrix_result[0][1] = 2.0f * Vector_X[3][0];
    matrix_result[0][2] = -2.0f * Vector_X[0][0];
    matrix_result[0][3] = 2.0f * Vector_X[1][0];

    matrix_result[1][0] = 2.0f * Vector_X[1][0];
    matrix_result[1][1] = 2.0f * Vector_X[0][0];
    matrix_result[1][2] = 2.0f * Vector_X[3][0];
    matrix_result[1][3] = 2.0f * Vector_X[2][0];

    matrix_result[2][0] = 2.0f * Vector_X[0][0];
    matrix_result[2][1] = -2.0f * Vector_X[1][0];
    matrix_result[2][2] = -2.0f * Vector_X[2][0];
    matrix_result[2][3] = 2.0f * Vector_X[3][0];

    return matrix_result;
}

/**
 * @brief 四元数测量函数对测量噪声的雅可比矩阵
 *
 * @param Vector_X 状态向量
 */
Class_Matrix_f32<3, 3> Class_BMI088::EKF_Function_Jacobian_H_V(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    return Namespace_ALG_Matrix::Identity<3, 3>();
}

/**
 * @brief 卡方检验
 *
 */
void Class_BMI088::Accel_Chi_Square_Calculate()
{
    Class_Matrix_f32<3, 1> vector_error;
    Class_Matrix_f32<3, 4> matrix_h_x;
    Class_Matrix_f32<3, 3> matrix_d;

    vector_error = Vector_Normalized_Accel - EKF_Quaternion.Function_H(EKF_Quaternion.Vector_X, D_T);

    matrix_h_x = EKF_Quaternion.Function_Jacobian_H_X(EKF_Quaternion.Vector_X, D_T);

    matrix_d = matrix_h_x * EKF_Quaternion.Matrix_P_Prior * matrix_h_x.Get_Transpose() + EKF_Quaternion.Matrix_R;

    Accel_Chi_Square_Loss = (vector_error.Get_Transpose() * matrix_d.Get_Inverse() * vector_error)[0][0];
}

#ifdef __cplusplus
extern "C" {
#endif

void BMI088_TIM_128ms_Calculate_PeriodElapsedCallback()
{
    BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
}

void BMI088_TIM_1ms_Service_PeriodElapsedCallback()
{
    BSP_BMI088.TIM_1ms_Service_PeriodElapsedCallback();
}

#ifdef __cplusplus
}
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
