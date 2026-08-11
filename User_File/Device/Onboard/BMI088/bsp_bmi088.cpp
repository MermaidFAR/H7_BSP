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
 * @brief 设置BMI088内部VQF的初始化参数
 *
 * @param __Config VQF配置
 * @note 只接受初始化前的配置，避免运行中修改采样周期或滤波参数破坏内部状态。
 */
void Class_BMI088::Set_VQF_Config(const Struct_BMI088_VQF_Config &__Config)
{
    if (Init_Finished_Flag)
    {
        return;
    }
    VQF_Config = __Config;
}

/**
 * @brief 初始化BMI088
 *
 */
void Class_BMI088::Init()
{
    SPI_Manage_Object = &SPI2_Manage_Object;

    BMI088_Accel.Init(true);
    BMI088_Gyro.Init();
    Filter_VQF.Init(VQF_Config.Parameter,
                    VQF_Config.Gyro_D_T,
                    VQF_Config.Accel_D_T);

    // 第一次姿态输出默认Yaw为0
    Vector_Euler_Angle[0][0] = 0.0f;

    Init_Finished_Flag = true;
}

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
 * @brief 使用VQF处理一帧BMI088数据
 */
void Class_BMI088::Calculate()
{
    const uint64_t calculate_start_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    Struct_BMI088_Gyro_Sample gyro_sample = {};
    if (!BMI088_Gyro.Pop_Sample(gyro_sample))
    {
        return;
    }

    if (!Accel_Observation_Pending)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const Struct_BMI088_Status accel_status_snapshot = Accel_Status;
        const Class_Matrix_f32<3, 1> accel_snapshot = BMI088_Accel.Get_Raw_Accel();
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
            BMI088_Status_Clear_Update_If_Matches(Accel_Status, accel_status_snapshot);
        }
    }

    Vector_Original_Accel = Accel_Observation_Pending
                                ? Vector_Pending_Accel
                                : BMI088_Accel.Get_Raw_Accel();
    Vector_Original_Gyro[0][0] = gyro_sample.Gyro_Rad_S[0];
    Vector_Original_Gyro[1][0] = gyro_sample.Gyro_Rad_S[1];
    Vector_Original_Gyro[2][0] = gyro_sample.Gyro_Rad_S[2];
    Vector_Fixed_Corrected_Gyro = Vector_Original_Gyro;

    const bool gyro_valid = gyro_sample.Valid != 0U;
    if (BMI088_Accel.Get_Heater_Enable())
    {
        Vector_Original_Accel =
            (Class_Matrix_f32<3, 3>(ACCEL_AFFINE_DATA) *
                 Vector_Original_Accel / GRAVITY_ACCELERATION +
             Class_Matrix_f32<3, 1>(ACCEL_BIAS_DATA)) *
            GRAVITY_ACCELERATION;
        if (gyro_valid)
        {
            Vector_Fixed_Corrected_Gyro +=
                Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
        }
    }
    Accel_Norm = Vector_Original_Accel.Get_Modulus();

    if (VQF_Pre_Timestamp == 0U)
    {
        D_T = VQF_Config.Gyro_D_T;
        VQF_Pre_Timestamp = gyro_sample.Timestamp_Us;
    }
    else if (gyro_sample.Timestamp_Us <= VQF_Pre_Timestamp)
    {
        Timestamp_Anomaly_Counter++;
        return;
    }
    else
    {
        D_T = static_cast<float>(gyro_sample.Timestamp_Us - VQF_Pre_Timestamp) /
              1000000.0f;
        VQF_Pre_Timestamp = gyro_sample.Timestamp_Us;
        if (D_T > D_T_TIMEOUT_THRESHOLD)
        {
            Sensor_Ready_Gap_Counter++;
            Filter_VQF.Reset();
            VQF_Reset_Counter++;
            D_T = VQF_Config.Gyro_D_T;
        }
    }

    if (gyro_valid)
    {
        Filter_VQF.Update_Gyro(Vector_Fixed_Corrected_Gyro);
    }

    if (Accel_Observation_Pending &&
        Pending_Accel_Timestamp <= gyro_sample.Timestamp_Us)
    {
        if (Pending_Accel_Valid &&
            !Basic_Math_Is_Invalid_Float(Accel_Norm) &&
            Accel_Norm > 1.0e-6f)
        {
            Filter_VQF.Update_Accel(Vector_Original_Accel);
            Set_Accel_Update_Result(true, BMI088_ACCEL_REJECT_NONE);
        }
        else
        {
            Set_Accel_Update_Result(false, BMI088_ACCEL_REJECT_INVALID);
        }
        Accel_Observation_Pending = false;
    }

    Quarternion = Filter_VQF.Get_Quaternion_6D();
    Vector_Euler_Angle = Quarternion.Get_Euler_Angle();
    Matrix_Rotation = Quarternion.Get_Rotation_Matrix();
    Vector_Axis_Angle = Quarternion.Get_Axis_Angle();

    const Class_Matrix_f32<3, 1> vector_gravity_body =
        Matrix_Rotation.Get_Transpose() *
        (-Namespace_ALG_Matrix::Axis_Z_3d() * GRAVITY_ACCELERATION);
    Vector_Accel_Body = Vector_Original_Accel + vector_gravity_body;
    Vector_Accel = Matrix_Rotation * Vector_Accel_Body;
    Vector_Gyro_Body = Filter_VQF.Get_Last_Corrected_Gyro();
    Vector_Gyro = Matrix_Rotation * Vector_Gyro_Body;
    Calculating_Time = SYS_Timestamp.Get_Now_Microsecond() -
                       calculate_start_timestamp;
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
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
    }
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
