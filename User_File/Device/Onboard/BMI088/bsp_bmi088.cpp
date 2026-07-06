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


extern "C" { extern osThreadId_t InsTaskHandle; }

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_BMI088 BSP_BMI088;

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

static bool BMI088_Status_Restore_Ready_On_Timeout(Struct_BMI088_Status &Status, const uint64_t &Now_Timestamp, const uint64_t &Timeout)
{
    if (Status.Transfering_Flag && (Now_Timestamp - Status.Transfering_Timestamp) >= Timeout)
    {
        Status.Transfering_Flag = false;
        if (!Status.Ready_Flag)
        {
            Status.Ready_Flag = true;
            Status.Ready_Timestamp = Status.Transfering_Timestamp;
        }
        return true;
    }
    return false;
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
    SPI_Manage_Object = &SPI2_Manage_Object;

    BMI088_Accel.Init(false);
    BMI088_Gyro.Init();

    // 欧拉角需要辅助初始化EKF, 第一次初始化默认Yaw是0
    Vector_Euler_Angle[0][0] = 0.0f;

    Init_Finished_Flag = true;
}

/**
 * @brief SPI接收完成回调函数
 *
 */
void Class_BMI088::SPI_RxCpltCallback()
{
    if (SPI_Manage_Object->Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin)
    {
        BMI088_Accel.SPI_RxCpltCallback();

        if (Init_Finished_Flag)
        {
            if (SPI_Manage_Object->Rx_Buffer_Length == 6)
            {
                Accel_Status.Transfering_Flag = false;
                Accel_Status.Update_Flag = true;
                Accel_Status.Update_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
                Accel_Status.Update_Ready_Timestamp = Accel_Status.Transfering_Timestamp;
            }
            else if (SPI_Manage_Object->Rx_Buffer_Length == 2)
            {
                Temperature_Status.Transfering_Flag = false;
            }
        }
    }
    else if (SPI_Manage_Object->Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {
        BMI088_Gyro.SPI_RxCallback();

        if (Init_Finished_Flag)
        {
            Gyro_Status.Transfering_Flag = false;
            Gyro_Status.Update_Flag = true;
            Gyro_Status.Update_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
            Gyro_Status.Update_Ready_Timestamp = Gyro_Status.Transfering_Timestamp;

            osThreadFlagsSet(InsTaskHandle, 0x0001);
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
        Accel_Status.Ready_Timestamp = now_timestamp;
        Accel_Status.Ready_Flag = true;
    }
    else if (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        Gyro_Status.Ready_Timestamp = now_timestamp;
        Gyro_Status.Ready_Flag = true;
    }

    BMI088_Service_Transfer(now_timestamp);
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
void Class_BMI088::TIM_128ms_Calculate_PeriodElapsedCallback()
{
    uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();

    Temperature_Status.Ready_Flag = true;
    Temperature_Status.Ready_Timestamp = now_timestamp;
    BMI088_Service_Transfer(now_timestamp, true);
    BMI088_Accel.TIM_128ms_Heater_PID_PeriodElapsedCallback();
}

void Class_BMI088::BMI088_Recover_SPI()
{
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

    Accel_Status.Transfering_Flag = false;
    Gyro_Status.Transfering_Flag = false;
    Temperature_Status.Transfering_Flag = false;
}

void Class_BMI088::BMI088_Service_Transfer(const uint64_t &Now_Timestamp, const bool &Allow_Recovery)
{
    if (!Init_Finished_Flag)
    {
        return;
    }

    bool transfer_timeout = false;
    transfer_timeout |= BMI088_Status_Restore_Ready_On_Timeout(Accel_Status, Now_Timestamp, TRANSFERING_TIMEOUT);
    transfer_timeout |= BMI088_Status_Restore_Ready_On_Timeout(Gyro_Status, Now_Timestamp, TRANSFERING_TIMEOUT);
    transfer_timeout |= BMI088_Status_Restore_Ready_On_Timeout(Temperature_Status, Now_Timestamp, TRANSFERING_TIMEOUT);

    bool spi_error = (SPI_Manage_Object != nullptr) &&
                     (SPI_Manage_Object->SPI_Handler != nullptr) &&
                     (SPI_Manage_Object->SPI_Handler->ErrorCode != HAL_SPI_ERROR_NONE);

    if (Allow_Recovery && (transfer_timeout || spi_error))
    {
        BMI088_Recover_SPI();
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
            uint8_t status = BMI088_Accel.SPI_Request_Accel();
            if (status == HAL_OK)
            {
                Accel_Status.Transfering_Flag = true;
                Accel_Status.Transfering_Timestamp = Accel_Status.Ready_Timestamp;
                Accel_Status.Ready_Flag = false;
                Transfer_Priority_Index = 1;
            }
            else if (Allow_Recovery)
            {
                BMI088_Recover_SPI();
            }
            return;
        }
        else if (index == 1 && Gyro_Status.Ready_Flag)
        {
            uint8_t status = BMI088_Gyro.SPI_Request_Gyro();
            if (status == HAL_OK)
            {
                Gyro_Status.Transfering_Flag = true;
                Gyro_Status.Transfering_Timestamp = Gyro_Status.Ready_Timestamp;
                Gyro_Status.Ready_Flag = false;
                Transfer_Priority_Index = 2;
            }
            else if (Allow_Recovery)
            {
                BMI088_Recover_SPI();
            }
            return;
        }
        else if (index == 2 && Temperature_Status.Ready_Flag)
        {
            uint8_t status = BMI088_Accel.SPI_Request_Temperature();
            if (status == HAL_OK)
            {
                Temperature_Status.Transfering_Flag = true;
                Temperature_Status.Transfering_Timestamp = Now_Timestamp;
                Temperature_Status.Ready_Flag = false;
                Transfer_Priority_Index = 0;
            }
            else if (Allow_Recovery)
            {
                BMI088_Recover_SPI();
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
    Struct_BMI088_Status shadow_gyro_status;
    Struct_BMI088_Status shadow_accel_status;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    shadow_gyro_status = Gyro_Status;
    shadow_accel_status = Accel_Status;
    if (primask == 0U)
    {
        __enable_irq();
    }

    Vector_Original_Accel = BMI088_Accel.Get_Raw_Accel();
    Vector_Original_Gyro = BMI088_Gyro.Get_Raw_Gyro();

    // 角速度合法则保存, 不合法则使用上次的数据
    if (!BMI088_Gyro.Get_Valid_Flag())
    {
        Vector_Original_Gyro = Vector_Pre_Original_Gyro;
    }

    // 如果加热电阻使能, 则进行零偏修正
    if (BMI088_Accel.Get_Heater_Enable())
    {
        Vector_Original_Accel = (Class_Matrix_f32<3, 3>(ACCEL_AFFINE_DATA) * Vector_Original_Accel / GRAVITY_ACCELERATION + Class_Matrix_f32<3, 1>(ACCEL_BIAS_DATA)) * GRAVITY_ACCELERATION;

        Vector_Original_Gyro = Vector_Original_Gyro + Class_Matrix_f32<3, 1>(GYRO_ZERO_OFFSET);
    }

    // 加速度计归一化数据
    if (BMI088_Accel.Get_Valid_Flag())
    {
        Vector_Normalized_Accel = Vector_Original_Accel.Get_Normalization();
    }
    else
    {
        BMI088_Status_Clear_Update_If_Matches(Accel_Status, shadow_accel_status);
        shadow_accel_status.Update_Flag = false;
    }


    // EKF初始化与计算
    if (!EKF_Init_Finished_Flag && shadow_accel_status.Update_Flag && BMI088_Accel.Get_Valid_Flag())
    {
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
        BMI088_Status_Clear_Update_If_Matches(Accel_Status, shadow_accel_status);
        BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
        EKF_Pre_Timestamp = BMI088_Status_Get_Update_Ready_Timestamp(shadow_accel_status);
        EKF_Now_Timestamp = EKF_Pre_Timestamp;

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
                EKF_Init_Finished_Flag = false;
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
                    EKF_Init_Finished_Flag = false;
                    return;
                }
                EKF_Update_With_Accel();
                BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
                BMI088_Status_Clear_Update_If_Matches(Accel_Status, shadow_accel_status);
            }
            else if (gyro_update_ready_timestamp < accel_update_ready_timestamp)
            {
                if (!EKF_Predict_To_Timestamp(gyro_update_ready_timestamp))
                {
                    EKF_Init_Finished_Flag = false;
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
                        EKF_Init_Finished_Flag = false;
                        return;
                    }
                    EKF_Update_With_Accel();
                }
                BMI088_Status_Clear_Update_If_Matches(Accel_Status, shadow_accel_status);

                if (!EKF_Predict_To_Timestamp(gyro_update_ready_timestamp))
                {
                    EKF_Init_Finished_Flag = false;
                    return;
                }
                BMI088_Status_Clear_Update_If_Matches(Gyro_Status, shadow_gyro_status);
            }
        }

        EKF_Output_To_Timestamp(calculate_start_timestamp);
        Calculating_Time = SYS_Timestamp.Get_Now_Microsecond() - calculate_start_timestamp;

        if (BMI088_Gyro.Get_Valid_Flag())
        {
            Vector_Pre_Original_Gyro = Vector_Original_Gyro;
        }
    }
}

bool Class_BMI088::EKF_Predict_To_Timestamp(const uint64_t &Timestamp)
{
    if (Timestamp <= EKF_Pre_Timestamp)
    {
        EKF_Now_Timestamp = EKF_Pre_Timestamp;
        D_T = 0.0f;
        return true;
    }

    D_T = (Timestamp - EKF_Pre_Timestamp) / 1000000.0f;
    if (D_T <= 0.0f || D_T > D_T_TIMEOUT_THRESHOLD)
    {
        return false;
    }

    EKF_Quaternion.Set_D_T(D_T);
    EKF_Quaternion.Vector_U = Vector_Original_Gyro;
    EKF_Quaternion.TIM_Predict_PeriodElapsedCallback();
    EKF_Quaternion.Vector_X = EKF_Quaternion.Vector_X.Get_Normalization();

    EKF_Pre_Timestamp = Timestamp;
    EKF_Now_Timestamp = Timestamp;

    return true;
}

void Class_BMI088::EKF_Update_With_Accel()
{
    if (!BMI088_Accel.Get_Valid_Flag())
    {
        return;
    }

    Accel_Chi_Square_Calculate();
    if (Accel_Chi_Square_Loss <= ACCEL_CHI_SQUARE_TEST_THRESHOLD)
    {
        EKF_Quaternion.Vector_Z = Vector_Normalized_Accel;
        EKF_Quaternion.TIM_Update_PeriodElapsedCallback();
        EKF_Quaternion.Vector_X = EKF_Quaternion.Vector_X.Get_Normalization();
    }
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

    matrix_result = Vector_X + 0.5f * D_T * matrix_omega * Vector_X;

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

    matrix_result = Namespace_ALG_Matrix::Identity<4, 4>() + 0.5f * D_T * matrix_omega;

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

#ifdef __cplusplus
}
#endif
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
