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
 * @param config VQF配置
 * @note 只接受初始化前的配置，避免运行中修改采样周期或滤波参数破坏内部状态。
 */
void Class_BMI088::Set_VQF_Config(const Struct_BMI088_VQF_Config &config)
{
    const float accel_interval_us = config.Accel_D_T * 1000000.0f;
    if (Init_Finished_Flag || !isfinite(config.Gyro_D_T) || config.Gyro_D_T <= 0.0f ||
        !(accel_interval_us >= 1.0f && accel_interval_us < 0x1p64f))
    {
        return;
    }
    VQF_Config = config;
    Accel_Interval_Us = (uint64_t)accel_interval_us;
}

/**
 * @brief 初始化BMI088
 *
 */
bool Class_BMI088::Init()
{
    Init_Finished_Flag = false;
    SPI_Manage_Object = &SPI2_Manage_Object;

    const bool accel_ready = BMI088_Accel.Init(true);
    const bool gyro_ready = BMI088_Gyro.Init();
    if (!accel_ready || !gyro_ready)
    {
        return false;
    }
    Filter_VQF.Init(VQF_Config.Parameter,
                    VQF_Config.Gyro_D_T,
                    VQF_Config.Accel_D_T);

    // 第一次姿态输出默认Yaw为0
    Vector_Euler_Angle[0][0] = 0.0f;

    Init_Finished_Flag = true;
    return true;
}

void Class_BMI088::SPI_RxCpltCallback()
{
    const bool accel_chip = SPI_Manage_Object->Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port &&
                            SPI_Manage_Object->Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin;
    if (!Init_Finished_Flag)
    {
        if (accel_chip)
        {
            BMI088_Accel.SPI_RxCpltCallback();
        }
        else
        {
            BMI088_Gyro.SPI_RxCallback(0U);
        }
        return;
    }

    // EXTI、SPI2 和 SPI2 DMA 同优先级，回调处理期间不会互相抢占。
    const Enum_BMI088_Sensor sensor = Active_Sensor;
    const bool valid_transfer = SPI_State == BMI088_SPI_STATE_STARTING ||
                                SPI_State == BMI088_SPI_STATE_IN_FLIGHT;
    const bool valid_frame = sensor == BMI088_SENSOR_GYRO
        ? !accel_chip
        : accel_chip && SPI_Manage_Object->Rx_Buffer_Length ==
                            (sensor == BMI088_SENSOR_ACCEL ? 6U : 2U);
    if (!valid_transfer || !valid_frame)
    {
        SPI_Manage_Object->Callback_Anomaly_Count++;
        return;
    }

    uint32_t flags = 0U;
    if (sensor == BMI088_SENSOR_GYRO)
    {
        const uint8_t result = BMI088_Gyro.SPI_RxCallback(Active_Ready_Timestamp);
        if ((result & BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED) != 0U)
        {
            Mark_Ready(BMI088_SENSOR_GYRO, SYS_Timestamp.Get_Now_Microsecond(), false);
            flags |= 0x0002;
        }
        if ((result & BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED) != 0U)
        {
            flags |= 0x0001;
        }
    }
    else
    {
        BMI088_Accel.SPI_RxCpltCallback();
        if (sensor == BMI088_SENSOR_ACCEL)
        {
            Accel_Update_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
            Accel_Update_Ready_Timestamp = Active_Ready_Timestamp;
        }
    }

    SPI_State = SPI_State == BMI088_SPI_STATE_STARTING ? BMI088_SPI_STATE_COMPLETED_DURING_START
                                                       : BMI088_SPI_STATE_IDLE;
    if (flags != 0U)
    {
        osThreadFlagsSet(BMI088TaskHandle, flags);
    }
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
        Mark_Ready(BMI088_SENSOR_ACCEL, now_timestamp);
    }
    else if (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BMI088_Gyro.Notify_FIFO_Interrupt(now_timestamp);
        Mark_Ready(BMI088_SENSOR_GYRO, now_timestamp);
    }

    BMI088_Service_Transfer();
}

/**
 * @brief 每 128 ms 请求一次温度读取并更新加热控制。
 */
void Class_BMI088::TIM_128ms_Calculate_PeriodElapsedCallback()
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();

    Mark_Ready(BMI088_SENSOR_TEMPERATURE, now_timestamp);
    BMI088_Service_Transfer(true);
    BMI088_Accel.TIM_128ms_Heater_PID_PeriodElapsedCallback();
}

void Class_BMI088::TIM_1ms_Service_PeriodElapsedCallback()
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    const uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    if ((now_timestamp - BMI088_Gyro.Get_FIFO_Last_Interrupt_Timestamp_Us()) >= 2000U &&
        (now_timestamp - Gyro_FIFO_Last_Fallback_Poll_Timestamp) >= 1000U)
    {
        Gyro_FIFO_Last_Fallback_Poll_Timestamp = now_timestamp;
        Mark_Ready(BMI088_SENSOR_GYRO, now_timestamp, false);
    }
    BMI088_Service_Transfer(true);
}

void Class_BMI088::BMI088_Recover_SPI(uint8_t reason)
{
    SPI_Recovery_Counter++;
    SPI_Recovery_Last_Reason = reason;
    if ((reason & (BMI088_SPI_RECOVERY_ACCEL_TIMEOUT |
                     BMI088_SPI_RECOVERY_GYRO_TIMEOUT |
                     BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT)) != 0U)
    {
        SPI_Transfer_Timeout_Counter++;
    }
    if ((reason & BMI088_SPI_RECOVERY_ACCEL_TIMEOUT) != 0U)
    {
        SPI_Accel_Timeout_Counter++;
    }
    if ((reason & BMI088_SPI_RECOVERY_GYRO_TIMEOUT) != 0U)
    {
        SPI_Gyro_Timeout_Counter++;
    }
    if ((reason & BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT) != 0U)
    {
        SPI_Temperature_Timeout_Counter++;
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
    const uint32_t primask = SYS_Enter_Critical();
    SPI_Manage_Object->Transaction_Active = false;
    SPI_State = BMI088_SPI_STATE_IDLE;
    SYS_Exit_Critical(primask);
}

void Class_BMI088::BMI088_Service_Transfer(bool Allow_Recovery)
{
    if (!Init_Finished_Flag)
    {
        return;
    }

    static constexpr uint8_t timeout_reasons[BMI088_SENSOR_COUNT] = {
        BMI088_SPI_RECOVERY_ACCEL_TIMEOUT, BMI088_SPI_RECOVERY_GYRO_TIMEOUT,
        BMI088_SPI_RECOVERY_TEMPERATURE_TIMEOUT
    };
    static constexpr uint8_t start_failure_reasons[BMI088_SENSOR_COUNT] = {
        BMI088_SPI_RECOVERY_ACCEL_START_FAILURE, BMI088_SPI_RECOVERY_GYRO_START_FAILURE,
        BMI088_SPI_RECOVERY_TEMPERATURE_START_FAILURE
    };

    uint32_t primask = SYS_Enter_Critical();
    if (SPI_State == BMI088_SPI_STATE_STARTING ||
        SPI_State == BMI088_SPI_STATE_COMPLETED_DURING_START ||
        SPI_State == BMI088_SPI_STATE_RECOVERING)
    {
        SYS_Exit_Critical(primask);
        return;
    }

    uint8_t recovery = SPI_Recovery_Pending_Reason;
    if (Allow_Recovery)
    {
        SPI_HandleTypeDef *spi_handler = SPI_Manage_Object->SPI_Handler;
        if (SPI_State == BMI088_SPI_STATE_IN_FLIGHT)
        {
            const uint32_t now = (uint32_t)SYS_Timestamp.Get_Now_Microsecond();
            const uint32_t elapsed = now - Transfer_Start_Timestamp_Low32;
            if (elapsed >= TRANSFERING_TIMEOUT)
            {
                SPI_Capture_Timeout_Snapshot(spi_handler, elapsed);
                recovery |= timeout_reasons[Active_Sensor];
            }
        }
        if (spi_handler->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            recovery |= BMI088_SPI_RECOVERY_HAL_ERROR;
        }
        if (recovery != BMI088_SPI_RECOVERY_NONE)
        {
            if (SPI_State == BMI088_SPI_STATE_IN_FLIGHT)
            {
                uint64_t &ready = Ready_Timestamps[Active_Sensor];
                if (ready == 0U)
                {
                    ready = Active_Ready_Timestamp;
                }
            }
            SPI_Recovery_Pending_Reason = BMI088_SPI_RECOVERY_NONE;
            SPI_State = BMI088_SPI_STATE_RECOVERING;
            SYS_Exit_Critical(primask);

            BMI088_Recover_SPI(recovery);
            return;
        }
    }

    if (SPI_State != BMI088_SPI_STATE_IDLE || recovery != BMI088_SPI_RECOVERY_NONE)
    {
        SYS_Exit_Critical(primask);
        return;
    }

    uint8_t index = Next_Sensor;
    for (uint8_t scanned = 0; scanned < BMI088_SENSOR_COUNT; scanned++, index = (index + 1) % BMI088_SENSOR_COUNT)
    {
        if (Ready_Timestamps[index] == 0U)
        {
            continue;
        }

        const Enum_BMI088_Sensor sensor = (Enum_BMI088_Sensor)index;
        Active_Sensor = sensor;
        Active_Ready_Timestamp = Ready_Timestamps[index];
        Ready_Timestamps[index] = 0U;
        SPI_State = BMI088_SPI_STATE_STARTING;
        SYS_Exit_Critical(primask);

        uint8_t status;
        switch (sensor)
        {
        case BMI088_SENSOR_ACCEL:
            status = BMI088_Accel.SPI_Request_Accel();
            break;
        case BMI088_SENSOR_GYRO:
            status = BMI088_Gyro.SPI_Request_Gyro();
            break;
        default:
            status = BMI088_Accel.SPI_Request_Temperature();
            break;
        }

        primask = SYS_Enter_Critical();
        if (status == HAL_OK)
        {
            Next_Sensor = (index + 1) % BMI088_SENSOR_COUNT;
            if (SPI_State == BMI088_SPI_STATE_COMPLETED_DURING_START)
            {
                SPI_State = BMI088_SPI_STATE_IDLE;
            }
            else
            {
                Transfer_Start_Timestamp_Low32 = (uint32_t)SYS_Timestamp.Get_Now_Microsecond();
                SPI_State = BMI088_SPI_STATE_IN_FLIGHT;
            }
        }
        else
        {
            if (Ready_Timestamps[index] == 0U)
            {
                Ready_Timestamps[index] = Active_Ready_Timestamp;
            }
            const uint8_t reason = start_failure_reasons[index];
            SPI_State = Allow_Recovery ? BMI088_SPI_STATE_RECOVERING : BMI088_SPI_STATE_IDLE;
            if (Allow_Recovery)
            {
                SYS_Exit_Critical(primask);
                BMI088_Recover_SPI(reason);
                return;
            }
            else
            {
                SPI_Recovery_Pending_Reason = reason;
            }
        }
        SYS_Exit_Critical(primask);
        return;
    }
    SYS_Exit_Critical(primask);
}

/**
 * @brief 使用VQF处理一帧BMI088数据
 */
void Class_BMI088::Calculate()
{
    if (!Init_Finished_Flag)
    {
        return;
    }
    const uint64_t calculate_start_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    Struct_BMI088_Gyro_Sample gyro_sample = {};
    if (!BMI088_Gyro.Pop_Sample(gyro_sample))
    {
        return;
    }

    if (!Accel_Observation_Pending)
    {
        const uint32_t primask = SYS_Enter_Critical();
        const uint64_t accel_update = Accel_Update_Timestamp;
        const uint64_t accel_timestamp = Accel_Update_Ready_Timestamp;
        const Class_Matrix_f32<3, 1> accel_snapshot = BMI088_Accel.Get_Raw_Accel();
        const bool accel_valid_snapshot = BMI088_Accel.Get_Valid_Flag();
        SYS_Exit_Critical(primask);

        if (accel_update != 0U)
        {
            if (Pending_Accel_Timestamp == 0U ||
                accel_timestamp / Accel_Interval_Us !=
                    Pending_Accel_Timestamp / Accel_Interval_Us)
            {
                Vector_Pending_Accel = accel_snapshot;
                Pending_Accel_Timestamp = accel_timestamp;
                Pending_Accel_Valid = accel_valid_snapshot;
                Accel_Observation_Pending = true;
            }
            const uint32_t clear_primask = SYS_Enter_Critical();
            if (Accel_Update_Timestamp == accel_update && Accel_Update_Ready_Timestamp == accel_timestamp)
            {
                Accel_Update_Timestamp = 0U;
            }
            SYS_Exit_Critical(clear_primask);
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
        D_T = (float)(gyro_sample.Timestamp_Us - VQF_Pre_Timestamp) /
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

void Class_BMI088::Mark_Ready(Enum_BMI088_Sensor sensor, uint64_t timestamp, bool replace)
{
    const uint32_t primask = SYS_Enter_Critical();
    uint64_t &ready = Ready_Timestamps[sensor];
    if (replace || ready == 0U)
    {
        ready = timestamp;
    }
    SYS_Exit_Critical(primask);
}

void Class_BMI088::Set_Accel_Update_Result(bool accepted, uint8_t reject_reason)
{
    Accel_Update_Result = (accepted ? 1U : 0U) | ((uint32_t)reject_reason << 8U);
    Accel_Update_Attempt_Counter++;
    if (!accepted)
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
