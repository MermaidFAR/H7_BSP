/**
 * @file bsp_bmi088_accel.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-14 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 合肥当地重力加速度，供BMI088加速度换算和有效性判断使用。
const float GRAVITY_ACCELERATION = 9.7947f;

static constexpr float BMI088_TEMPERATURE_MIN = -40.0f;
static constexpr float BMI088_TEMPERATURE_MAX = 85.0f;
static constexpr float BMI088_TEMPERATURE_MAX_STEP = 5.0f;
static uint32_t BMI088_Temperature_Outlier_Counter = 0U;
static uint32_t BMI088_Temperature_Stale_Counter = 0U;
static uint32_t BMI088_Accel_Invalid_Counter = 0U;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

uint32_t Class_BMI088_Accel::Get_Temperature_Outlier_Counter() const
{
    return (BMI088_Temperature_Outlier_Counter);
}

uint32_t Class_BMI088_Accel::Get_Accel_Invalid_Counter() const
{
    return (BMI088_Accel_Invalid_Counter);
}

uint32_t Class_BMI088_Accel::Get_Temperature_Stale_Counter() const
{
    return (BMI088_Temperature_Stale_Counter);
}

bool Class_BMI088_Accel::Get_Temperature_Valid_Flag() const
{
    return Get_Temperature_State().Data_Valid;
}

uint32_t Class_BMI088_Accel::Get_Temperature_Age_Us() const
{
    return Get_Temperature_State().Age_Us;
}

Struct_BMI088_Accel_Temperature_State Class_BMI088_Accel::Get_Temperature_State() const
{
    Struct_BMI088_Accel_Temperature_State state = {};
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    state.Now_Timestamp_Us = SYS_Timestamp.Get_Current_Timestamp();
    state.Last_Valid_Timestamp_Us = Temperature_Last_Valid_Timestamp;
    state.Temperature = Now_Temperature;
    state.Stale_Counter = BMI088_Temperature_Stale_Counter;
    state.Heater_PWM_Compare = Heater_PWM_Compare;
    const bool raw_valid = Temperature_Valid_Flag;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (raw_valid)
    {
        const uint64_t age_us = state.Now_Timestamp_Us >= state.Last_Valid_Timestamp_Us
                                    ? state.Now_Timestamp_Us - state.Last_Valid_Timestamp_Us
                                    : UINT64_MAX;
        state.Age_Us = age_us <= UINT32_MAX ? static_cast<uint32_t>(age_us)
                                            : UINT32_MAX;
        state.Data_Valid = age_us <= TEMPERATURE_STALE_TIMEOUT_US;
    }
    return state;
}

/**
 * @brief 初始化BMI088加速度计
 *
 * @param Heater_Enable 是否使能加热电阻
 */
void Class_BMI088_Accel::Init(const bool &__Heater_Enable)
{
    // 绑定SPI
    SPI_Manage_Object = &SPI2_Manage_Object;

    // 绑定片选
    CS_GPIO_Port = BMI088_ACCEL__SPI_CS_GPIO_Port;
    CS_Pin = BMI088_ACCEL__SPI_CS_Pin;
    Activate_Pin_State = GPIO_PIN_RESET;

    // 绑定加热电阻定时器
    htim = &htim3;
    TIM_Channel = TIM_CHANNEL_4;

    Heater_Enable = __Heater_Enable;

    // 初始化PID
    PID_Temperature.Init(100.0f, 10.0f, 0.0f, 0.0f, 300.0f, 500.0f, 0.128f);

    // 启动PWM
    if (Heater_Enable)
    {
        HAL_TIM_PWM_Start(htim, TIM_Channel);
        __HAL_TIM_SET_COMPARE(htim, TIM_Channel, 0);
    }

    uint8_t res;

    // 检测通信是否正常
    Register.ACC_CHIP_ID_RO = 0x00;
    while (Register.ACC_CHIP_ID_RO != 0x1e)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_CHIP_ID_RO));
        Namespace_SYS_Timestamp::Delay_Millisecond(100);
    }

    // 软重启
    res = 0xb6;
    Write_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_PWR_CTRL_RW), &res);
    Namespace_SYS_Timestamp::Delay_Millisecond(100);

    // 检测通信是否正常
    Register.ACC_CHIP_ID_RO = 0x00;
    while (Register.ACC_CHIP_ID_RO != 0x1e)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_CHIP_ID_RO));
        Namespace_SYS_Timestamp::Delay_Millisecond(100);
    }

    for (uint8_t i = 0; i < BMI088_ACCEL_INIT_INSTRUCTION_NUM; i++)
    {
        ((uint8_t *) (&Register))[BMI088_ACCEL_REGISTER_CONFIG[i][0]] = 0x00;
        while (((uint8_t *) (&Register))[BMI088_ACCEL_REGISTER_CONFIG[i][0]] != BMI088_ACCEL_REGISTER_CONFIG[i][1])
        {
            // 写入寄存器
            Write_Single_Register(BMI088_ACCEL_REGISTER_CONFIG[i][0], &BMI088_ACCEL_REGISTER_CONFIG[i][1]);
            Namespace_SYS_Timestamp::Delay_Millisecond(100);

            // 读取寄存器
            Read_Single_Register(BMI088_ACCEL_REGISTER_CONFIG[i][0]);
            Namespace_SYS_Timestamp::Delay_Millisecond(100);
        }
    }

    // 预读取一次加速度计数据
    Read_Multi_Register(offsetof(Struct_BMI088_Accel_Register, ACC_X_RO), 6);
    Namespace_SYS_Timestamp::Delay_Millisecond(100);
}

/**
 * @brief SPI接收回调函数, 处理加速度计数据
 *
 */
void Class_BMI088_Accel::SPI_RxCpltCallback()
{
    uint8_t spi_init_address = SPI_Manage_Object->Tx_Buffer[0] & ~BMI088_ACCEL_READ_MASK;

    memcpy((uint8_t *) (&Register) + spi_init_address, &SPI_Manage_Object->Rx_Buffer[1 + BMI088_ACCEL_SPI_RX_RESERVED], SPI_Manage_Object->Rx_Buffer_Length);

    // 处理数据
    if (spi_init_address == offsetof(Struct_BMI088_Accel_Register, ACC_X_RO))
    {
        // 读取加速度计数据完成

        Vector_Raw_Accel[0][0] = (float) (Register.ACC_X_RO) / 32768.0f * (1 << (BMI088_ACCEL_RANGE + 1)) * 1.5f * GRAVITY_ACCELERATION;
        Vector_Raw_Accel[1][0] = (float) (Register.ACC_Y_RO) / 32768.0f * (1 << (BMI088_ACCEL_RANGE + 1)) * 1.5f * GRAVITY_ACCELERATION;
        Vector_Raw_Accel[2][0] = (float) (Register.ACC_Z_RO) / 32768.0f * (1 << (BMI088_ACCEL_RANGE + 1)) * 1.5f * GRAVITY_ACCELERATION;

        if (Basic_Math_Is_Invalid_Float(Vector_Raw_Accel[0][0]) || Basic_Math_Is_Invalid_Float(Vector_Raw_Accel[1][0]) || Basic_Math_Is_Invalid_Float(Vector_Raw_Accel[2][0]))
        {
            Valid_Flag = false;
            BMI088_Accel_Invalid_Counter++;
        }
        else
        {
            Valid_Flag = true;
        }
    }
    else if (spi_init_address == offsetof(Struct_BMI088_Accel_Register, TEMP_MSB_RO))
    {
        // 读取温度计数据完成
        int16_t raw_temperature = static_cast<int16_t>(Register.TEMP_MSB_RO << 3 | Register.TEMP_LSB_RO >> 5);
        if ((raw_temperature & 0x0400) != 0)
        {
            raw_temperature -= 1 << 11;
        }
        const float temperature = 23.0f + static_cast<float>(raw_temperature) * 0.125f;
        const bool physical_outlier = temperature < BMI088_TEMPERATURE_MIN || temperature > BMI088_TEMPERATURE_MAX;
        const bool step_outlier = Temperature_Valid_Flag &&
                                  fabsf(temperature - Now_Temperature) > BMI088_TEMPERATURE_MAX_STEP;
        if (physical_outlier || step_outlier)
        {
            BMI088_Temperature_Outlier_Counter++;
            Temperature_Valid_Flag = false;
            Temperature_Rebase_Count = 0U;
        }
        else if (Temperature_Valid_Flag)
        {
            Now_Temperature = temperature;
            Temperature_Last_Valid_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
        }
        else
        {
            if (Temperature_Rebase_Count == 0U ||
                fabsf(temperature - Temperature_Rebase_Candidate) > BMI088_TEMPERATURE_MAX_STEP)
            {
                Temperature_Rebase_Candidate = temperature;
                Temperature_Rebase_Count = 1U;
            }
            else
            {
                Temperature_Rebase_Candidate = temperature;
                Temperature_Rebase_Count++;
            }

            if (Temperature_Rebase_Count >= TEMPERATURE_REBASE_SAMPLE_COUNT)
            {
                Now_Temperature = Temperature_Rebase_Candidate;
                Temperature_Last_Valid_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
                Temperature_Rebase_Count = 0U;
                __DMB();
                Temperature_Valid_Flag = true;
                Temperature_Stale_Latched = false;
            }
        }
    }
}

/**
 * @brief SPI请求加速度计数据
 *
 */
uint8_t Class_BMI088_Accel::SPI_Request_Accel()
{
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(offsetof(Struct_BMI088_Accel_Register, ACC_X_RO) | BMI088_ACCEL_READ_MASK);

    return SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                                     Activate_Pin_State, tx_data, sizeof(tx_data), 6);
}

/**
 * @brief SPI请求温度计数据
 *
 */
uint8_t Class_BMI088_Accel::SPI_Request_Temperature()
{
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(offsetof(Struct_BMI088_Accel_Register, TEMP_MSB_RO) | BMI088_ACCEL_READ_MASK);

    return SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                                     Activate_Pin_State, tx_data, sizeof(tx_data), 2);
}

/**
 * @brief TIM定时器中断回调函数, 100ms周期
 *
 */
void Class_BMI088_Accel::TIM_128ms_Heater_PID_PeriodElapsedCallback()
{
    if (Heater_Enable)
    {
        const Struct_BMI088_Accel_Temperature_State temperature_state =
            Get_Temperature_State();
        if (!temperature_state.Data_Valid ||
            Basic_Math_Is_Invalid_Float(temperature_state.Temperature))
        {
            if (!Temperature_Stale_Latched)
            {
                BMI088_Temperature_Stale_Counter++;
                Temperature_Stale_Latched = true;
            }
            Heater_PWM_Compare = 0U;
            __HAL_TIM_SET_COMPARE(htim, TIM_Channel, 0);
            return;
        }
        Temperature_Stale_Latched = false;

        // 是否开启预热
        if (temperature_state.Temperature < HEATER_PREHEAT_BASE_TEMPERATURE)
        {
            // 需要预热
            Heater_Preheat_Finished_Flag = false;
        }
        else
        {
            // 不需要预热
            Heater_Preheat_Finished_Flag = true;
        }

        float tmp;
        if (!Heater_Preheat_Finished_Flag)
        {
            tmp = HEATER_PREHEAT_POWER;
        }
        else
        {
            PID_Temperature.Set_Now(temperature_state.Temperature);
            PID_Temperature.Set_Target(HEATER_TARGET_TEMPERATURE);
            PID_Temperature.TIM_Calculate_PeriodElapsedCallback();
            tmp = PID_Temperature.Get_Out();
        }
        float output = tmp / (BSP_Power.Get_Power_Voltage() * BSP_Power.Get_Power_Voltage()) * HEATER_NOMINAL_VOLTAGE * HEATER_NOMINAL_VOLTAGE;
        Basic_Math_Constrain(&output, 0.0f, 10000.0f);
        Heater_PWM_Compare = static_cast<uint32_t>(output);
        __HAL_TIM_SET_COMPARE(htim, TIM_Channel, Heater_PWM_Compare);
    }
    else
    {
        Heater_PWM_Compare = 0U;
        __HAL_TIM_SET_COMPARE(htim, TIM_Channel, 0);
    }
}

/**
 * @brief 读取单个寄存器, 数据不会立即返回, 而是在SPI接收回调函数中处理
 *
 * @param Register_Address 寄存器地址
 */
void Class_BMI088_Accel::Read_Single_Register(const uint8_t &Register_Address) const
{
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(Register_Address | BMI088_ACCEL_READ_MASK);

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, sizeof(tx_data), 1);
}

/** * @brief 读取多个寄存器, 数据不会立即返回, 而是在SPI接收回调函数中处理
 *
 * @param Register_Address 寄存器地址
 * @param Rx_Length 接收数据长度
 */
void Class_BMI088_Accel::Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const
{
    constexpr uint32_t tx_length = 2U;
    if (Rx_Length > (SPI_BUFFER_SIZE - tx_length))
    {
        return;
    }
    uint8_t tx_data[2] = {};
    tx_data[0] = static_cast<uint8_t>(Register_Address | BMI088_ACCEL_READ_MASK);

    SPI_Transmit_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin,
                              Activate_Pin_State, tx_data, sizeof(tx_data), static_cast<uint16_t>(Rx_Length));
}

/**
 * @brief 写入单个寄存器
 *
 * @param Register_Address 寄存器地址
 * @param Tx_Data_Buffer 发送数据缓冲区
 */
void Class_BMI088_Accel::Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const
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
void Class_BMI088_Accel::Write_Multi_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer, const uint32_t &Tx_Length) const
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
