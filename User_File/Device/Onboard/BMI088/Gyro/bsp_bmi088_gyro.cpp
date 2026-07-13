/**
 * @file bsp_bmi088_gyro.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之陀螺仪
 * @version 0.1
 * @date 2025-08-19 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static constexpr float BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD = 32.0f;
static uint32_t BMI088_Gyro_Outlier_Counter = 0U;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

uint32_t Class_BMI088_Gyro::Get_Outlier_Counter() const
{
    return (BMI088_Gyro_Outlier_Counter);
}

void Class_BMI088_Gyro::Notify_FIFO_Interrupt(const uint64_t &__Timestamp_Us)
{
    FIFO_Last_Interrupt_Timestamp_Us = __Timestamp_Us;
    FIFO_Interrupt_Count++;
}

bool Class_BMI088_Gyro::Pop_Sample(Struct_BMI088_Gyro_Sample &__Sample)
{
    const uint16_t tail = Sample_Queue_Tail;
    __DMB();
    if (tail == Sample_Queue_Head)
    {
        return false;
    }

    __Sample = Sample_Queue[tail];
    __DMB();
    Sample_Queue_Tail = static_cast<uint16_t>((tail + 1U) &
                                               BMI088_GYRO_SAMPLE_QUEUE_MASK);
    Sample_Queue_Consume_Count++;
    return true;
}

/**
 * @brief 初始化BMI088加速度计
 *
 * @param Heater_Enable 是否使能加热电阻
 */
void Class_BMI088_Gyro::Init()
{
    // 绑定SPI
    SPI_Manage_Object = &SPI2_Manage_Object;

    // 绑定片选
    CS_GPIO_Port = BMI088_GYRO__SPI_CS_GPIO_Port;
    CS_Pin = BMI088_GYRO__SPI_CS_Pin;
    Activate_Pin_State = GPIO_PIN_RESET;

    Sample_Queue_Head = 0U;
    Sample_Queue_Tail = 0U;
    Sample_Queue_High_Watermark = 0U;
    FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
    FIFO_Pending_Frame_Count = 0U;
    FIFO_Requested_Frame_Count = 0U;
    FIFO_Batch_Total_Frame_Count = 0U;
    FIFO_Batch_Processed_Frame_Count = 0U;
    FIFO_Previous_Batch_Frame_Count = 0U;
    FIFO_Batch_Anchor_Timestamp_Us = 0U;
    FIFO_Previous_Anchor_Timestamp_Us = 0U;
    FIFO_Last_Interrupt_Timestamp_Us = 0U;
    FIFO_Last_Handled_Interrupt_Timestamp_Us = 0U;
    FIFO_Last_Enqueued_Timestamp_Us = 0U;
    FIFO_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    FIFO_Batch_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    FIFO_Batch_From_Interrupt = false;
    FIFO_Overrun_Latched = false;
    FIFO_Sample_Sequence = 0U;
    FIFO_Interrupt_Count = 0U;
    FIFO_Status_Read_Count = 0U;
    FIFO_Frame_Read_Count = 0U;
    FIFO_Overrun_Count = 0U;
    FIFO_Spurious_Interrupt_Count = 0U;
    FIFO_Followup_Request_Count = 0U;
    Sample_Queue_Enqueue_Count = 0U;
    Sample_Queue_Consume_Count = 0U;
    Sample_Queue_Drop_Count = 0U;

    uint8_t res;

    // 检测通信是否正常
    Register.GYRO_CHIP_ID_RO = 0x00;
    while (Register.GYRO_CHIP_ID_RO != 0x0f)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Gyro_Register, GYRO_CHIP_ID_RO));
        Namespace_SYS_Timestamp::Delay_Millisecond(100);
    }

    // 软重启
    res = 0xb6;
    Write_Single_Register(offsetof(Struct_BMI088_Gyro_Register, GYRO_SOFTRESET_WO), &res);
    Namespace_SYS_Timestamp::Delay_Millisecond(100);

    // 检测通信是否正常
    Register.GYRO_CHIP_ID_RO = 0x00;
    while (Register.GYRO_CHIP_ID_RO != 0x0f)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Gyro_Register, GYRO_CHIP_ID_RO));
        Namespace_SYS_Timestamp::Delay_Millisecond(100);
    }

    for (uint8_t i = 0; i < BMI088_GYRO_INIT_INSTRUCTION_NUM; i++)
    {
        ((uint8_t *) (&Register))[BMI088_GYRO_REGISTER_CONFIG[i][0]] = 0x00;
        while (((uint8_t *) (&Register))[BMI088_GYRO_REGISTER_CONFIG[i][0]] != BMI088_GYRO_REGISTER_CONFIG[i][1])
        {
            // 写入寄存器
            Write_Single_Register(BMI088_GYRO_REGISTER_CONFIG[i][0], &BMI088_GYRO_REGISTER_CONFIG[i][1]);
            Namespace_SYS_Timestamp::Delay_Millisecond(100);

            // 读取寄存器
            Read_Single_Register(BMI088_GYRO_REGISTER_CONFIG[i][0]);
            Namespace_SYS_Timestamp::Delay_Millisecond(100);
        }
    }

}

void Class_BMI088_Gyro::Start_FIFO_Acquisition()
{
    // 全局初始化结束时清空启动阶段积累的旧数据。
    const uint8_t fifo_stream_mode = 0x80U;
    Write_Single_Register(
        offsetof(Struct_BMI088_Gyro_Register, FIFO_CONFIG_1_RW),
        &fifo_stream_mode);
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

/**
 * @brief SPI接收回调函数, 处理加速度计数据
 *
 */
uint8_t Class_BMI088_Gyro::SPI_RxCallback(const uint64_t &__Ready_Timestamp_Us)
{
    const uint8_t spi_init_address =
        SPI_Manage_Object->Tx_Buffer[0] & ~BMI088_GYRO_READ_MASK;

    if (spi_init_address == offsetof(Struct_BMI088_Gyro_Register, FIFO_STATUS_RO))
    {
        if (SPI_Manage_Object->Rx_Buffer_Length != 1U)
        {
            return BMI088_GYRO_SPI_RESULT_NONE;
        }

        Register.FIFO_STATUS_RO = SPI_Manage_Object->Rx_Buffer[1];
        FIFO_Status_Read_Count++;
        const bool overrun = (Register.FIFO_STATUS_RO & 0x80U) != 0U;
        const uint8_t frame_count = Register.FIFO_STATUS_RO & 0x7fU;
        if (overrun && !FIFO_Overrun_Latched)
        {
            FIFO_Overrun_Latched = true;
            FIFO_Overrun_Count++;
        }

        const bool from_interrupt =
            __Ready_Timestamp_Us != 0U &&
            __Ready_Timestamp_Us == FIFO_Last_Interrupt_Timestamp_Us &&
            __Ready_Timestamp_Us != FIFO_Last_Handled_Interrupt_Timestamp_Us;
        if (from_interrupt)
        {
            FIFO_Last_Handled_Interrupt_Timestamp_Us = __Ready_Timestamp_Us;
        }

        if (frame_count < BMI088_GYRO_FIFO_WATERMARK_FRAME_COUNT && !overrun)
        {
            if (from_interrupt)
            {
                FIFO_Spurious_Interrupt_Count++;
            }
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
            return BMI088_GYRO_SPI_RESULT_NONE;
        }
        if (frame_count == 0U)
        {
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
            return BMI088_GYRO_SPI_RESULT_NONE;
        }

        if (FIFO_Previous_Anchor_Timestamp_Us != 0U &&
            FIFO_Previous_Batch_Frame_Count != 0U &&
            __Ready_Timestamp_Us > FIFO_Previous_Anchor_Timestamp_Us)
        {
            const float estimated_period_us =
                static_cast<float>(__Ready_Timestamp_Us -
                                   FIFO_Previous_Anchor_Timestamp_Us) /
                static_cast<float>(FIFO_Previous_Batch_Frame_Count);
            if (estimated_period_us >= BMI088_GYRO_MIN_SAMPLE_PERIOD_US &&
                estimated_period_us <= BMI088_GYRO_MAX_SAMPLE_PERIOD_US)
            {
                FIFO_Sample_Period_Us =
                    0.875f * FIFO_Sample_Period_Us +
                    0.125f * estimated_period_us;
            }
        }

        FIFO_Batch_Total_Frame_Count = frame_count;
        FIFO_Batch_Processed_Frame_Count = 0U;
        FIFO_Batch_Anchor_Timestamp_Us = __Ready_Timestamp_Us;
        FIFO_Batch_From_Interrupt = from_interrupt;
        FIFO_Batch_Sample_Period_Us = FIFO_Sample_Period_Us;
        if (!from_interrupt && FIFO_Last_Enqueued_Timestamp_Us != 0U &&
            __Ready_Timestamp_Us > FIFO_Last_Enqueued_Timestamp_Us)
        {
            const float batch_period_us =
                static_cast<float>(__Ready_Timestamp_Us -
                                   FIFO_Last_Enqueued_Timestamp_Us) /
                static_cast<float>(frame_count);
            if (batch_period_us >= 250.0f && batch_period_us <= 750.0f)
            {
                FIFO_Batch_Sample_Period_Us = batch_period_us;
            }
        }
        FIFO_Pending_Frame_Count =
            frame_count > BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                ? BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                : frame_count;
        FIFO_Request = BMI088_GYRO_FIFO_REQUEST_DATA;
        FIFO_Followup_Request_Count++;
        return BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED;
    }

    if (spi_init_address == offsetof(Struct_BMI088_Gyro_Register, FIFO_DATA_RO))
    {
        const uint16_t expected_length =
            static_cast<uint16_t>(FIFO_Requested_Frame_Count) *
            BMI088_GYRO_FIFO_FRAME_SIZE;
        if (FIFO_Requested_Frame_Count == 0U ||
            SPI_Manage_Object->Rx_Buffer_Length != expected_length)
        {
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
            return BMI088_GYRO_SPI_RESULT_NONE;
        }

        uint8_t result = BMI088_GYRO_SPI_RESULT_NONE;
        for (uint8_t i = 0U; i < FIFO_Requested_Frame_Count; i++)
        {
            const uint8_t *frame =
                &SPI_Manage_Object->Rx_Buffer[1U +
                                              static_cast<uint16_t>(i) *
                                                  BMI088_GYRO_FIFO_FRAME_SIZE];
            const Class_Matrix_f32<3, 1> gyro = Decode_Gyro_Frame(frame);
            const bool invalid_float =
                Basic_Math_Is_Invalid_Float(gyro[0][0]) ||
                Basic_Math_Is_Invalid_Float(gyro[1][0]) ||
                Basic_Math_Is_Invalid_Float(gyro[2][0]);
            const bool outlier =
                fabsf(gyro[0][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD ||
                fabsf(gyro[1][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD ||
                fabsf(gyro[2][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD;
            const bool valid = !invalid_float && !outlier;
            if (outlier)
            {
                BMI088_Gyro_Outlier_Counter++;
            }

            Vector_Raw_Gyro = gyro;
            Valid_Flag = valid;

            const uint8_t anchor_frame_index =
                FIFO_Batch_From_Interrupt
                    ? BMI088_GYRO_FIFO_WATERMARK_FRAME_COUNT - 1U
                    : FIFO_Batch_Total_Frame_Count - 1U;
            const int32_t frame_offset =
                static_cast<int32_t>(FIFO_Batch_Processed_Frame_Count) + i -
                static_cast<int32_t>(anchor_frame_index);
            const float timestamp_offset_f =
                static_cast<float>(frame_offset) *
                FIFO_Batch_Sample_Period_Us;
            const int64_t timestamp_offset_us = static_cast<int64_t>(
                timestamp_offset_f + (timestamp_offset_f >= 0.0f ? 0.5f : -0.5f));
            uint64_t sample_timestamp_us = FIFO_Batch_Anchor_Timestamp_Us;
            if (timestamp_offset_us >= 0)
            {
                sample_timestamp_us += static_cast<uint64_t>(timestamp_offset_us);
            }
            else
            {
                const uint64_t offset = static_cast<uint64_t>(-timestamp_offset_us);
                sample_timestamp_us = sample_timestamp_us > offset
                                          ? sample_timestamp_us - offset
                                          : 1U;
            }

            if (FIFO_Last_Enqueued_Timestamp_Us != 0U &&
                sample_timestamp_us <= FIFO_Last_Enqueued_Timestamp_Us)
            {
                const uint64_t sample_step_us = static_cast<uint64_t>(
                    FIFO_Batch_Sample_Period_Us + 0.5f);
                sample_timestamp_us = FIFO_Last_Enqueued_Timestamp_Us +
                                      (sample_step_us != 0U ? sample_step_us : 1U);
            }

            if (Enqueue_Sample(gyro, sample_timestamp_us, valid))
            {
                FIFO_Last_Enqueued_Timestamp_Us = sample_timestamp_us;
                result |= BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED;
            }
        }

        FIFO_Frame_Read_Count += FIFO_Requested_Frame_Count;
        FIFO_Batch_Processed_Frame_Count += FIFO_Requested_Frame_Count;
        const uint8_t remaining =
            FIFO_Batch_Total_Frame_Count - FIFO_Batch_Processed_Frame_Count;
        if (remaining != 0U)
        {
            FIFO_Pending_Frame_Count =
                remaining > BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                    ? BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT
                    : remaining;
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_DATA;
        }
        else
        {
            FIFO_Previous_Anchor_Timestamp_Us = FIFO_Batch_Anchor_Timestamp_Us;
            FIFO_Previous_Batch_Frame_Count = FIFO_Batch_Total_Frame_Count;
            FIFO_Batch_Total_Frame_Count = 0U;
            FIFO_Batch_Processed_Frame_Count = 0U;
            FIFO_Batch_From_Interrupt = false;
            FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
        }

        FIFO_Followup_Request_Count++;
        result |= BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED;
        return result;
    }

    memcpy((uint8_t *) (&Register) + spi_init_address, &SPI_Manage_Object->Rx_Buffer[1], SPI_Manage_Object->Rx_Buffer_Length);

    // 处理数据
    if (spi_init_address == offsetof(Struct_BMI088_Gyro_Register, RATE_X_RO))
    {
        // 读取陀螺仪数据完成

        Vector_Raw_Gyro = Decode_Gyro_Frame(
            reinterpret_cast<const uint8_t *>(&Register.RATE_X_RO));

        const bool invalid_float = Basic_Math_Is_Invalid_Float(Vector_Raw_Gyro[0][0]) ||
                                   Basic_Math_Is_Invalid_Float(Vector_Raw_Gyro[1][0]) ||
                                   Basic_Math_Is_Invalid_Float(Vector_Raw_Gyro[2][0]);
        const bool outlier = fabsf(Vector_Raw_Gyro[0][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD ||
                             fabsf(Vector_Raw_Gyro[1][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD ||
                             fabsf(Vector_Raw_Gyro[2][0]) >= BMI088_GYRO_OUTLIER_ABSOLUTE_THRESHOLD;
        if (invalid_float || outlier)
        {
            Valid_Flag = false;
            if (outlier)
            {
                BMI088_Gyro_Outlier_Counter++;
            }
        }
        else
        {
            Valid_Flag = true;
        }
    }
    return BMI088_GYRO_SPI_RESULT_NONE;
}

/**
 * @brief TIM定时器中断回调函数, 100us周期
 *
 */
uint8_t Class_BMI088_Gyro::SPI_Request_Gyro()
{
    uint8_t register_address =
        offsetof(Struct_BMI088_Gyro_Register, FIFO_STATUS_RO);
    uint16_t rx_length = 1U;
    if (FIFO_Request == BMI088_GYRO_FIFO_REQUEST_DATA)
    {
        register_address = offsetof(Struct_BMI088_Gyro_Register, FIFO_DATA_RO);
        FIFO_Requested_Frame_Count = FIFO_Pending_Frame_Count;
        rx_length = static_cast<uint16_t>(FIFO_Requested_Frame_Count) *
                    BMI088_GYRO_FIFO_FRAME_SIZE;
    }
    const uint8_t tx_data[1] = {
        static_cast<uint8_t>(register_address | BMI088_GYRO_READ_MASK)};

    const uint8_t status = SPI_Transmit_Receive_Data(
        SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
        Activate_Pin_State, tx_data, 1U, rx_length);
    if (status != HAL_OK && FIFO_Request == BMI088_GYRO_FIFO_REQUEST_DATA)
    {
        FIFO_Requested_Frame_Count = 0U;
    }
    return status;
}

Class_Matrix_f32<3, 1> Class_BMI088_Gyro::Decode_Gyro_Frame(
    const uint8_t *__Frame) const
{
    Class_Matrix_f32<3, 1> gyro;
    if (__Frame == nullptr)
    {
        return gyro;
    }

    const int16_t raw_x = static_cast<int16_t>(
        static_cast<uint16_t>(__Frame[0]) |
        (static_cast<uint16_t>(__Frame[1]) << 8U));
    const int16_t raw_y = static_cast<int16_t>(
        static_cast<uint16_t>(__Frame[2]) |
        (static_cast<uint16_t>(__Frame[3]) << 8U));
    const int16_t raw_z = static_cast<int16_t>(
        static_cast<uint16_t>(__Frame[4]) |
        (static_cast<uint16_t>(__Frame[5]) << 8U));
    const float scale = static_cast<float>(1U << (4U - BMI088_GYRO_RANGE)) *
                        125.0f * BASIC_MATH_DEG_TO_RAD / 32768.0f;
    gyro[0][0] = static_cast<float>(raw_x) * scale;
    gyro[1][0] = static_cast<float>(raw_y) * scale;
    gyro[2][0] = static_cast<float>(raw_z) * scale;
    return gyro;
}

bool Class_BMI088_Gyro::Enqueue_Sample(
    const Class_Matrix_f32<3, 1> &__Gyro,
    const uint64_t &__Timestamp_Us,
    const bool &__Valid)
{
    const uint32_t sequence = ++FIFO_Sample_Sequence;
    const uint16_t head = Sample_Queue_Head;
    const uint16_t next_head = static_cast<uint16_t>(
        (head + 1U) & BMI088_GYRO_SAMPLE_QUEUE_MASK);
    if (next_head == Sample_Queue_Tail)
    {
        Sample_Queue_Drop_Count++;
        return false;
    }

    Struct_BMI088_Gyro_Sample &sample = Sample_Queue[head];
    sample.Timestamp_Us = __Timestamp_Us;
    sample.Gyro_Rad_S[0] = __Gyro[0][0];
    sample.Gyro_Rad_S[1] = __Gyro[1][0];
    sample.Gyro_Rad_S[2] = __Gyro[2][0];
    sample.Sequence = sequence;
    sample.Valid = __Valid ? 1U : 0U;
    sample.Reserved[0] = 0U;
    sample.Reserved[1] = 0U;
    sample.Reserved[2] = 0U;
    __DMB();
    Sample_Queue_Head = next_head;
    Sample_Queue_Enqueue_Count++;

    const uint16_t depth = Get_Queue_Depth();
    if (depth > Sample_Queue_High_Watermark)
    {
        Sample_Queue_High_Watermark = depth;
    }
    return true;
}

/**
 * @brief 读取单个寄存器, 数据不会立即返回, 而是在SPI接收回调函数中处理
 *
 * @param Register_Address 寄存器地址
 */
void Class_BMI088_Gyro::Read_Single_Register(const uint8_t &Register_Address) const
{
    const uint8_t tx_data[1] = {static_cast<uint8_t>(Register_Address | BMI088_GYRO_READ_MASK)};

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, 1, 1);
}

/** * @brief 读取多个寄存器, 数据不会立即返回, 而是在SPI接收回调函数中处理
 *
 * @param Register_Address 寄存器地址
 * @param Rx_Length 接收数据长度
 */
void Class_BMI088_Gyro::Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const
{
    if (Rx_Length > (SPI_BUFFER_SIZE - 1U))
    {
        return;
    }
    const uint8_t tx_data[1] = {static_cast<uint8_t>(Register_Address | BMI088_GYRO_READ_MASK)};

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, 1, static_cast<uint16_t>(Rx_Length));
}

/**
 * @brief 写入单个寄存器
 *
 * @param Register_Address 寄存器地址
 * @param Tx_Data_Buffer 发送数据缓冲区
 */
void Class_BMI088_Gyro::Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const
{
    const uint8_t tx_data[2] = {Register_Address, Tx_Data_Buffer[0]};

    SPI_Transmit_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                      Activate_Pin_State, tx_data, sizeof(tx_data));
}

/**
 * @brief 写入多个寄存器
 *
 * @param Register_Address 寄存器地址
 * @param Tx_Data_Buffer 发送数据缓冲区
 * @param Tx_Length 发送数据长度
 */
void Class_BMI088_Gyro::Write_Multi_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer, const uint32_t &Tx_Length) const
{
    if (Tx_Length > (SPI_BUFFER_SIZE - 1U))
    {
        return;
    }
    uint8_t tx_data[SPI_BUFFER_SIZE] = {};
    tx_data[0] = Register_Address;
    memcpy(&tx_data[1], Tx_Data_Buffer, Tx_Length);

    SPI_Transmit_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin, Activate_Pin_State,
                      tx_data, static_cast<uint16_t>(Tx_Length + 1U));
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
