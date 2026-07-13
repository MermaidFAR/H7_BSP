/**
 * @file bsp_bmi088_gyro.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之陀螺仪
 * @version 0.1
 * @date 2025-08-19 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_BMI088_GYRO_H
#define BSP_BMI088_GYRO_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro_register.h"
#include "bsp_spi.h"
#include "alg_matrix.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 陀螺仪量程枚举类型
 *
 */
enum Enum_BSP_BMI088_Gyro_Range : uint8_t
{
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS,
    BMI088_GYRO_RANGE_500DPS,
    BMI088_GYRO_RANGE_250DPS,
    BMI088_GYRO_RANGE_125DPS,
};

enum Enum_BSP_BMI088_Gyro_SPI_Result : uint8_t
{
    BMI088_GYRO_SPI_RESULT_NONE = 0U,
    BMI088_GYRO_SPI_RESULT_SAMPLES_QUEUED = 1U << 0,
    BMI088_GYRO_SPI_RESULT_FOLLOWUP_REQUIRED = 1U << 1,
};

struct Struct_BMI088_Gyro_Sample
{
    uint64_t Timestamp_Us;
    float Gyro_Rad_S[3];
    uint32_t Sequence;
    uint8_t Valid;
    uint8_t Reserved[3];
};

static_assert(sizeof(Struct_BMI088_Gyro_Sample) == 32U,
              "BMI088 gyro sample ABI changed");

/**
 * @brief Specialized, 陀螺仪
 *
 */
class Class_BMI088_Gyro
{
public:
    void Init();

    void Start_FIFO_Acquisition();

    inline bool Get_Valid_Flag() const;

    inline Class_Matrix_f32<3, 1> Get_Raw_Gyro() const;

    uint32_t Get_Outlier_Counter() const;

    void Notify_FIFO_Interrupt(const uint64_t &__Timestamp_Us);

    bool Pop_Sample(Struct_BMI088_Gyro_Sample &__Sample);

    inline uint16_t Get_Queue_Depth() const;

    inline uint16_t Get_Queue_High_Watermark() const;

    inline uint32_t Get_FIFO_Interrupt_Count() const;

    inline uint32_t Get_FIFO_Status_Read_Count() const;

    inline uint32_t Get_FIFO_Frame_Read_Count() const;

    inline uint32_t Get_FIFO_Overrun_Count() const;

    inline uint32_t Get_FIFO_Spurious_Interrupt_Count() const;

    inline uint32_t Get_FIFO_Followup_Request_Count() const;

    inline uint32_t Get_Queue_Enqueue_Count() const;

    inline uint32_t Get_Queue_Consume_Count() const;

    inline uint32_t Get_Queue_Drop_Count() const;

    inline float Get_FIFO_Sample_Period_Us() const;

    inline uint64_t Get_FIFO_Last_Interrupt_Timestamp_Us() const;

    inline Class_Matrix_f32<3, 1> Get_Callibrated_Gyro() const;

    uint8_t SPI_RxCallback(const uint64_t &__Ready_Timestamp_Us);

    uint8_t SPI_Request_Gyro();

protected:
    // 初始化相关常量

    // 绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;
    // 片选绑定的GPIO
    GPIO_TypeDef *CS_GPIO_Port;
    uint16_t CS_Pin;
    GPIO_PinState Activate_Pin_State;

    // 常量

    // 读取寄存器时需要设置的掩码
    const uint8_t BMI088_GYRO_READ_MASK = 0x80;
    // 初始化指令数
    const uint8_t BMI088_GYRO_INIT_INSTRUCTION_NUM = 8;
    // 陀螺仪量程, 默认2000°/s
    const Enum_BSP_BMI088_Gyro_Range BMI088_GYRO_RANGE = BMI088_GYRO_RANGE_2000DPS;
    // 寄存器配置相关
    const uint8_t BMI088_GYRO_REGISTER_CONFIG[8][2] = {
        // 设置陀螺仪量程
        {offsetof(Struct_BMI088_Gyro_Register, GYRO_RANGE_RW), BMI088_GYRO_RANGE},
        // 设置陀螺仪反馈频率为2000Hz, 带宽为230Hz, bit7只读且恒为1
        {offsetof(Struct_BMI088_Gyro_Register, GYRO_BANDWODTH_RW), 0x01 | 0x80},
        // INT3推挽高有效, 与MCU上升沿EXTI配置一致
        {offsetof(Struct_BMI088_Gyro_Register, INT3_INT4_IO_CONF_RW), 0x0d},
        // FIFO watermark中断映射到INT3
        {offsetof(Struct_BMI088_Gyro_Register, INT3_INT4_IO_MAP_RW), 0x04},
        // FIFO超过7帧时产生watermark中断, 即每8帧触发
        {offsetof(Struct_BMI088_Gyro_Register, FIFO_CONFIG_0_RW), 0x07},
        // FIFO stream模式, 满时保留最新99帧
        {offsetof(Struct_BMI088_Gyro_Register, FIFO_CONFIG_1_RW), 0x80},
        // 使能FIFO watermark中断
        {offsetof(Struct_BMI088_Gyro_Register, FIFO_WM_EN_RW), 0x88},
        // 由1ms服务任务每4ms轮询FIFO, 不依赖锁存中断边沿
        {offsetof(Struct_BMI088_Gyro_Register, GYRO_INT_CTRL_RW), 0x00},
    };

    enum Enum_BMI088_Gyro_FIFO_Request : uint8_t
    {
        BMI088_GYRO_FIFO_REQUEST_STATUS = 0U,
        BMI088_GYRO_FIFO_REQUEST_DATA,
    };

    static constexpr uint8_t BMI088_GYRO_FIFO_WATERMARK_FRAME_COUNT = 8U;
    static constexpr uint8_t BMI088_GYRO_FIFO_FRAME_SIZE = 6U;
    static constexpr uint8_t BMI088_GYRO_FIFO_MAX_READ_FRAME_COUNT =
        (SPI_BUFFER_SIZE - 1U) / BMI088_GYRO_FIFO_FRAME_SIZE;
    static constexpr uint16_t BMI088_GYRO_SAMPLE_QUEUE_CAPACITY = 128U;
    static constexpr uint16_t BMI088_GYRO_SAMPLE_QUEUE_MASK =
        BMI088_GYRO_SAMPLE_QUEUE_CAPACITY - 1U;
    static constexpr float BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US = 500.0f;
    static constexpr float BMI088_GYRO_MIN_SAMPLE_PERIOD_US = 450.0f;
    static constexpr float BMI088_GYRO_MAX_SAMPLE_PERIOD_US = 550.0f;

    // 内部变量

    // 寄存器结构体
    Struct_BMI088_Gyro_Register Register;

    // 读变量

    // 当前陀螺仪是否有效
    bool Valid_Flag = true;
    // 当前角速度
    Class_Matrix_f32<3, 1> Vector_Raw_Gyro;

    Struct_BMI088_Gyro_Sample Sample_Queue[BMI088_GYRO_SAMPLE_QUEUE_CAPACITY] = {};
    volatile uint16_t Sample_Queue_Head = 0U;
    volatile uint16_t Sample_Queue_Tail = 0U;
    uint16_t Sample_Queue_High_Watermark = 0U;

    Enum_BMI088_Gyro_FIFO_Request FIFO_Request = BMI088_GYRO_FIFO_REQUEST_STATUS;
    uint8_t FIFO_Pending_Frame_Count = 0U;
    uint8_t FIFO_Requested_Frame_Count = 0U;
    uint8_t FIFO_Batch_Total_Frame_Count = 0U;
    uint8_t FIFO_Batch_Processed_Frame_Count = 0U;
    uint8_t FIFO_Previous_Batch_Frame_Count = 0U;
    uint64_t FIFO_Batch_Anchor_Timestamp_Us = 0U;
    uint64_t FIFO_Previous_Anchor_Timestamp_Us = 0U;
    uint64_t FIFO_Last_Interrupt_Timestamp_Us = 0U;
    uint64_t FIFO_Last_Handled_Interrupt_Timestamp_Us = 0U;
    uint64_t FIFO_Last_Enqueued_Timestamp_Us = 0U;
    float FIFO_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    float FIFO_Batch_Sample_Period_Us = BMI088_GYRO_NOMINAL_SAMPLE_PERIOD_US;
    bool FIFO_Batch_From_Interrupt = false;
    bool FIFO_Overrun_Latched = false;
    uint32_t FIFO_Sample_Sequence = 0U;
    uint32_t FIFO_Interrupt_Count = 0U;
    uint32_t FIFO_Status_Read_Count = 0U;
    uint32_t FIFO_Frame_Read_Count = 0U;
    uint32_t FIFO_Overrun_Count = 0U;
    uint32_t FIFO_Spurious_Interrupt_Count = 0U;
    uint32_t FIFO_Followup_Request_Count = 0U;
    uint32_t Sample_Queue_Enqueue_Count = 0U;
    uint32_t Sample_Queue_Consume_Count = 0U;
    uint32_t Sample_Queue_Drop_Count = 0U;

    // 写变量

    // 读写变量

    // 内部函数

    void Read_Single_Register(const uint8_t &Register_Address) const;

    void Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const;

    void Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const;

    void Write_Multi_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer, const uint32_t &Tx_Length) const;

    Class_Matrix_f32<3, 1> Decode_Gyro_Frame(const uint8_t *__Frame) const;

    bool Enqueue_Sample(const Class_Matrix_f32<3, 1> &__Gyro,
                        const uint64_t &__Timestamp_Us,
                        const bool &__Valid);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前陀螺仪是否有效
 *
 * @return 当前陀螺仪是否有效
 */
inline bool Class_BMI088_Gyro::Get_Valid_Flag() const
{
    return (Valid_Flag);
}

/**
 * @brief 获取当当前陀螺仪原始数据
 *
 * @return 当前陀螺仪原始数据
 */
inline Class_Matrix_f32<3, 1> Class_BMI088_Gyro::Get_Raw_Gyro() const
{
    return (Vector_Raw_Gyro);
}

inline uint16_t Class_BMI088_Gyro::Get_Queue_Depth() const
{
    return static_cast<uint16_t>((Sample_Queue_Head - Sample_Queue_Tail) &
                                 BMI088_GYRO_SAMPLE_QUEUE_MASK);
}

inline uint16_t Class_BMI088_Gyro::Get_Queue_High_Watermark() const
{
    return Sample_Queue_High_Watermark;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Interrupt_Count() const
{
    return FIFO_Interrupt_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Status_Read_Count() const
{
    return FIFO_Status_Read_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Frame_Read_Count() const
{
    return FIFO_Frame_Read_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Overrun_Count() const
{
    return FIFO_Overrun_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Spurious_Interrupt_Count() const
{
    return FIFO_Spurious_Interrupt_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_FIFO_Followup_Request_Count() const
{
    return FIFO_Followup_Request_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_Queue_Enqueue_Count() const
{
    return Sample_Queue_Enqueue_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_Queue_Consume_Count() const
{
    return Sample_Queue_Consume_Count;
}

inline uint32_t Class_BMI088_Gyro::Get_Queue_Drop_Count() const
{
    return Sample_Queue_Drop_Count;
}

inline float Class_BMI088_Gyro::Get_FIFO_Sample_Period_Us() const
{
    return FIFO_Sample_Period_Us;
}

inline uint64_t Class_BMI088_Gyro::Get_FIFO_Last_Interrupt_Timestamp_Us() const
{
    return FIFO_Last_Interrupt_Timestamp_Us;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
