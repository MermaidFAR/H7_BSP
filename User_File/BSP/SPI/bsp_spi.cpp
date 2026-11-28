/**
 * @file bsp_spi.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的SPI通信初始化与配置流程
 * @version 3.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-13 1.1 加入SPI配置操作
 * @date 2023-11-19 1.2 修改成cpp, 24赛季定稿
 * @date 2024-08-22 2.1 新增SPI回调函数空指针判定
 * @date 2025-08-13 3.1 适配达妙MC02板, 准备着手开发陀螺仪
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_spi.h"
#include "sys_critical.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_SPI_Manage_Object SPI1_Manage_Object = {};

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_SPI_Manage_Object SPI2_Manage_Object = {};

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_SPI_Manage_Object SPI3_Manage_Object = {};

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_SPI_Manage_Object SPI4_Manage_Object = {};

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_SPI_Manage_Object SPI5_Manage_Object = {};

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_SPI_Manage_Object SPI6_Manage_Object = {};

Struct_SPI_Timeout_Snapshot SPI2_Timeout_Snapshot = {};

/* Private function declarations ---------------------------------------------*/

static Struct_SPI_Manage_Object *SPI_Get_Manage_Object(const SPI_HandleTypeDef *hspi)
{
    if (hspi == nullptr)
    {
        return nullptr;
    }

    if (hspi->Instance == SPI1)
        return &SPI1_Manage_Object;
    if (hspi->Instance == SPI2)
        return &SPI2_Manage_Object;
    if (hspi->Instance == SPI3)
        return &SPI3_Manage_Object;
    if (hspi->Instance == SPI4)
        return &SPI4_Manage_Object;
    if (hspi->Instance == SPI5)
        return &SPI5_Manage_Object;
    if (hspi->Instance == SPI6)
        return &SPI6_Manage_Object;
    return nullptr;
}

static bool SPI_Recover_DMA(DMA_HandleTypeDef *dma_handler)
{
    if (dma_handler->State == HAL_DMA_STATE_BUSY)
    {
        const uint32_t previous_error = dma_handler->ErrorCode;
        if (HAL_DMA_Abort(dma_handler) != HAL_OK)
        {
            if (dma_handler->ErrorCode != HAL_DMA_ERROR_NO_XFER)
            {
                return false;
            }
            dma_handler->ErrorCode = previous_error;
        }
    }

    // HAL 中止超时后软件可已是 ERROR，只有硬件 EN 清零后才能复用该通道。
    const bool enabled = IS_DMA_STREAM_INSTANCE(dma_handler->Instance)
                             ? (((DMA_Stream_TypeDef *) dma_handler->Instance)->CR & DMA_SxCR_EN) != 0U
                             : (((BDMA_Channel_TypeDef *) dma_handler->Instance)->CCR & BDMA_CCR_EN) != 0U;
    if (enabled)
    {
        __HAL_DMA_DISABLE(dma_handler);
        return false;
    }
    if (dma_handler->State != HAL_DMA_STATE_READY || dma_handler->Lock != HAL_UNLOCKED ||
        (dma_handler->ErrorCode & ~HAL_DMA_ERROR_NO_XFER) != HAL_DMA_ERROR_NONE)
    {
        return HAL_DMA_Init(dma_handler) == HAL_OK;
    }
    // HAL_SPI_Abort 可能对已空闲的 DMA 调用 Abort，留下 HAL_DMA_ERROR_NO_XFER。
    dma_handler->ErrorCode = HAL_DMA_ERROR_NONE;
    return true;
}

void SPI_Capture_Timeout_Snapshot(SPI_HandleTypeDef *hspi, uint32_t Elapsed_Us)
{
    if (hspi == nullptr || hspi->Instance != SPI2)
    {
        return;
    }

    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    DMA_HandleTypeDef *rx_dma = hspi->hdmarx;
    DMA_HandleTypeDef *tx_dma = hspi->hdmatx;
    DMA_Stream_TypeDef *rx_stream = rx_dma != nullptr
                                        ? (DMA_Stream_TypeDef *) rx_dma->Instance
                                        : nullptr;
    DMA_Stream_TypeDef *tx_stream = tx_dma != nullptr
                                        ? (DMA_Stream_TypeDef *) tx_dma->Instance
                                        : nullptr;
    const uint32_t next_capture_count = SPI2_Timeout_Snapshot.Capture_Count + 1U;

    uint8_t pending_bits = 0U;
    pending_bits |= NVIC_GetPendingIRQ(SPI2_IRQn) != 0U ? 0x01U : 0U;
    pending_bits |= NVIC_GetPendingIRQ(DMA1_Stream0_IRQn) != 0U ? 0x02U : 0U;
    pending_bits |= NVIC_GetPendingIRQ(DMA1_Stream1_IRQn) != 0U ? 0x04U : 0U;

    uint8_t active_bits = 0U;
    active_bits |= NVIC_GetActive(SPI2_IRQn) != 0U ? 0x01U : 0U;
    active_bits |= NVIC_GetActive(DMA1_Stream0_IRQn) != 0U ? 0x02U : 0U;
    active_bits |= NVIC_GetActive(DMA1_Stream1_IRQn) != 0U ? 0x04U : 0U;

    SPI2_Timeout_Snapshot.Timestamp_Low32_Us =
        (uint32_t) SYS_Timestamp.Get_Now_Microsecond();
    SPI2_Timeout_Snapshot.Elapsed_Us = Elapsed_Us;
    SPI2_Timeout_Snapshot.SPI_CR1 = hspi->Instance->CR1;
    SPI2_Timeout_Snapshot.SPI_CR2 = hspi->Instance->CR2;
    SPI2_Timeout_Snapshot.SPI_CFG1 = hspi->Instance->CFG1;
    SPI2_Timeout_Snapshot.SPI_IER = hspi->Instance->IER;
    SPI2_Timeout_Snapshot.SPI_SR = hspi->Instance->SR;
    SPI2_Timeout_Snapshot.DMA1_LISR = DMA1->LISR;
    SPI2_Timeout_Snapshot.RX_DMA_CR =
        rx_stream != nullptr ? rx_stream->CR : 0U;
    SPI2_Timeout_Snapshot.RX_DMA_NDTR =
        rx_stream != nullptr ? rx_stream->NDTR : 0U;
    SPI2_Timeout_Snapshot.TX_DMA_CR =
        tx_stream != nullptr ? tx_stream->CR : 0U;
    SPI2_Timeout_Snapshot.TX_DMA_NDTR =
        tx_stream != nullptr ? tx_stream->NDTR : 0U;
    SPI2_Timeout_Snapshot.HAL_Error_Code = hspi->ErrorCode;
    SPI2_Timeout_Snapshot.HAL_Tx_Xfer_Count = hspi->TxXferCount;
    SPI2_Timeout_Snapshot.HAL_Rx_Xfer_Count = hspi->RxXferCount;
    SPI2_Timeout_Snapshot.Manager_Tx_Length =
        manage_object != nullptr ? manage_object->Tx_Buffer_Length : 0U;
    SPI2_Timeout_Snapshot.Manager_Rx_Length =
        manage_object != nullptr ? manage_object->Rx_Buffer_Length : 0U;
    SPI2_Timeout_Snapshot.HAL_State = (uint8_t) hspi->State;
    SPI2_Timeout_Snapshot.HAL_Lock = (uint8_t) hspi->Lock;
    SPI2_Timeout_Snapshot.RX_DMA_State =
        rx_dma != nullptr ? (uint8_t) rx_dma->State : 0U;
    SPI2_Timeout_Snapshot.TX_DMA_State =
        tx_dma != nullptr ? (uint8_t) tx_dma->State : 0U;
    SPI2_Timeout_Snapshot.Transaction_Active =
        manage_object != nullptr && manage_object->Transaction_State != SPI_TRANSACTION_IDLE ? 1U : 0U;
    SPI2_Timeout_Snapshot.NVIC_Pending_Bits = pending_bits;
    SPI2_Timeout_Snapshot.NVIC_Active_Bits = active_bits;
    SPI2_Timeout_Snapshot.Reserved = 0U;
    __DMB();
    SPI2_Timeout_Snapshot.Capture_Count = next_capture_count;
    __DMB();
}

static bool SPI_Try_Acquire_Transaction(Struct_SPI_Manage_Object *manage_object)
{
    const uint32_t primask = SYS_Enter_Critical();
    const bool acquired = manage_object->Transaction_State == SPI_TRANSACTION_IDLE;
    if (acquired)
    {
        manage_object->Transaction_State = SPI_TRANSACTION_ACTIVE;
    }
    else
    {
        manage_object->Transaction_Busy_Count++;
    }
    SYS_Exit_Critical(primask);
    return acquired;
}

static void SPI_Deactivate_Chip_Select(Struct_SPI_Manage_Object *manage_object)
{
    if (manage_object != nullptr && manage_object->Activate_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(manage_object->Activate_GPIOx, manage_object->Activate_GPIO_Pin,
                          manage_object->Activate_Level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

static void SPI_Release_Transaction(Struct_SPI_Manage_Object *manage_object)
{
    if (manage_object == nullptr)
    {
        return;
    }

    manage_object->Activate_GPIOx = nullptr;
    manage_object->Activate_GPIO_Pin = 0;
    manage_object->Tx_Buffer_Length = 0;
    manage_object->Rx_Buffer_Length = 0;
    __DMB();
    manage_object->Transaction_State = SPI_TRANSACTION_IDLE;
    __DMB();
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化SPI
 *
 * @param hspi SPI编号
 * @param Callback_Function 处理回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }

    memset(manage_object, 0, sizeof(Struct_SPI_Manage_Object));
    manage_object->SPI_Handler = hspi;
    manage_object->Callback_Function = Callback_Function;
}

/**
 * @brief 发送数据
 *
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Activate_Level 片选GPIO引脚电平
 * @param Tx_Data 待复制到DMA缓冲区的数据
 * @param Tx_Length 长度
 * @return uint8_t 执行状态
 */
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                          GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length, bool Blocking)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr || Tx_Data == nullptr || Tx_Length == 0 || Tx_Length > SPI_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }
    if (!SPI_Try_Acquire_Transaction(manage_object))
    {
        return HAL_BUSY;
    }

    memcpy(manage_object->Tx_Buffer, Tx_Data, Tx_Length);
    manage_object->Activate_GPIOx = GPIOx;
    manage_object->Activate_GPIO_Pin = GPIO_Pin;
    manage_object->Activate_Level = Activate_Level;
    manage_object->Tx_Buffer_Length = Tx_Length;
    manage_object->Rx_Buffer_Length = 0;

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, Activate_Level);
    }

    if (Blocking || hspi->Instance == SPI6)
    {
        const HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, manage_object->Tx_Buffer, Tx_Length, 1);
        if (status != HAL_OK)
        {
            manage_object->Start_Failure_Count++;
            manage_object->Last_Start_Failure_Status = (uint8_t) status;
        }
        SPI_Deactivate_Chip_Select(manage_object);
        SPI_Release_Transaction(manage_object);
        return (uint8_t) status;
    }

    const HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(hspi, manage_object->Tx_Buffer, Tx_Length);
    if (status != HAL_OK)
    {
        manage_object->Start_Failure_Count++;
        manage_object->Last_Start_Failure_Status = (uint8_t) status;
        SPI_Deactivate_Chip_Select(manage_object);
        SPI_Release_Transaction(manage_object);
    }
    return (uint8_t) status;
}

/**
 * @brief 发送并接收数据
 *
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Activate_Level 片选GPIO引脚电平
 * @param Tx_Data 待复制到DMA缓冲区的数据
 * @param Tx_Length 发送长度
 * @param Rx_Length 接收长度
 * @return uint8_t 执行状态
 */
uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                                  GPIO_PinState Activate_Level, const uint8_t *Tx_Data,
                                  uint16_t Tx_Length, uint16_t Rx_Length, bool Blocking)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    const uint32_t transfer_length = (uint32_t) Tx_Length + Rx_Length;
    if (manage_object == nullptr || Tx_Data == nullptr || Tx_Length == 0 || Rx_Length == 0 ||
        transfer_length > SPI_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }
    if (!SPI_Try_Acquire_Transaction(manage_object))
    {
        return HAL_BUSY;
    }

    memset(manage_object->Tx_Buffer, 0, transfer_length);
    memset(manage_object->Rx_Buffer, 0, transfer_length);
    memcpy(manage_object->Tx_Buffer, Tx_Data, Tx_Length);
    manage_object->Activate_GPIOx = GPIOx;
    manage_object->Activate_GPIO_Pin = GPIO_Pin;
    manage_object->Activate_Level = Activate_Level;
    manage_object->Tx_Buffer_Length = Tx_Length;
    manage_object->Rx_Buffer_Length = Rx_Length;

    if (GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, Activate_Level);
    }

    if (Blocking || hspi->Instance == SPI6)
    {
        const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
            hspi, manage_object->Tx_Buffer, manage_object->Rx_Buffer,
            (uint16_t) transfer_length, 1);
        if (status != HAL_OK)
        {
            manage_object->Start_Failure_Count++;
            manage_object->Last_Start_Failure_Status = (uint8_t) status;
        }
        SPI_Deactivate_Chip_Select(manage_object);
        if (status == HAL_OK)
        {
            manage_object->Rx_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
            if (manage_object->Callback_Function != nullptr)
            {
                manage_object->Callback_Function(manage_object->Tx_Buffer, manage_object->Rx_Buffer,
                                                 manage_object->Tx_Buffer_Length,
                                                 manage_object->Rx_Buffer_Length);
            }
        }
        SPI_Release_Transaction(manage_object);
        return (uint8_t) status;
    }

    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        hspi, manage_object->Tx_Buffer, manage_object->Rx_Buffer, (uint16_t) transfer_length);
    if (status != HAL_OK)
    {
        manage_object->Start_Failure_Count++;
        manage_object->Last_Start_Failure_Status = (uint8_t) status;
        SPI_Deactivate_Chip_Select(manage_object);
        SPI_Release_Transaction(manage_object);
    }
    return (uint8_t) status;
}

/**
 * @brief HAL库SPI仅发送回调函数
 *
 * @param hspi SPI编号
 */
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }
    if (manage_object->Transaction_State != SPI_TRANSACTION_ACTIVE)
    {
        manage_object->Callback_Anomaly_Count++;
        return;
    }

    SPI_Deactivate_Chip_Select(manage_object);
    SPI_Release_Transaction(manage_object);
}

/**
 * @brief HAL库SPI全双工DMA中断
 *
 * @param hspi SPI编号
 */
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }
    if (manage_object->Transaction_State != SPI_TRANSACTION_ACTIVE)
    {
        manage_object->Callback_Anomaly_Count++;
        return;
    }

    SPI_Deactivate_Chip_Select(manage_object);
    manage_object->Rx_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (manage_object->Callback_Function != nullptr)
    {
        manage_object->Callback_Function(manage_object->Tx_Buffer, manage_object->Rx_Buffer,
                                         manage_object->Tx_Buffer_Length, manage_object->Rx_Buffer_Length);
    }

    SPI_Release_Transaction(manage_object);
}

/**
 * @brief HAL库SPI错误回调函数
 *
 * @param hspi SPI编号
 */
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return;
    }
    if (manage_object->Transaction_State != SPI_TRANSACTION_ACTIVE)
    {
        manage_object->Callback_Anomaly_Count++;
        return;
    }

    manage_object->Error_Count++;
    SPI_Deactivate_Chip_Select(manage_object);
    SPI_Release_Transaction(manage_object);
}

bool SPI_Recover(SPI_HandleTypeDef *hspi)
{
    Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    if (manage_object == nullptr)
    {
        return false;
    }

    uint32_t primask = SYS_Enter_Critical();
    if (manage_object->Transaction_State == SPI_TRANSACTION_RECOVERING)
    {
        SYS_Exit_Critical(primask);
        return false;
    }
    if (manage_object->Transaction_State == SPI_TRANSACTION_RECOVERY_PENDING && hspi->Lock == HAL_UNLOCKED)
    {
        // HAL_SPI_Abort 用此位判定本次中止结果，重试前移除上次锁存的中止错误。
        hspi->ErrorCode &= ~HAL_SPI_ERROR_ABORT;
    }
    manage_object->Transaction_State = SPI_TRANSACTION_RECOVERING;
    SYS_Exit_Critical(primask);

    SPI_Deactivate_Chip_Select(manage_object);
    DMA_HandleTypeDef *const channels[] = {hspi->hdmatx, hspi->hdmarx};
    const uint32_t previous_errors[] = {
        hspi->hdmatx != nullptr ? hspi->hdmatx->ErrorCode : HAL_DMA_ERROR_NONE,
        hspi->hdmarx != nullptr ? hspi->hdmarx->ErrorCode : HAL_DMA_ERROR_NONE};
    const bool spi_stopped = HAL_SPI_Abort(hspi) == HAL_OK &&
                             (hspi->Instance->CR1 & (SPI_CR1_SPE | SPI_CR1_CSTART)) == 0U &&
                             (hspi->Instance->CFG1 & (SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN)) == 0U;
    bool stopped = spi_stopped;
    for (unsigned i = 0; i < 2U; ++i)
    {
        DMA_HandleTypeDef *dma = channels[i];
        if (dma == nullptr)
        {
            continue;
        }
        // HAL_SPI_Abort 中止已空闲的 DMA 时会覆盖原错误，包括上次 Init 留下的 PARAM。
        if (dma->ErrorCode == HAL_DMA_ERROR_NO_XFER)
        {
            dma->ErrorCode = previous_errors[i];
        }
        if (spi_stopped && !SPI_Recover_DMA(dma))
        {
            stopped = false;
        }
    }

    primask = SYS_Enter_Critical();
    if (stopped)
    {
        SPI_Release_Transaction(manage_object);
    }
    else
    {
        manage_object->Recovery_Failure_Count++;
        manage_object->Transaction_State = SPI_TRANSACTION_RECOVERY_PENDING;
    }
    SYS_Exit_Critical(primask);
    return stopped;
}

bool SPI_Has_Error(const SPI_HandleTypeDef *hspi)
{
    return hspi != nullptr && hspi->ErrorCode != HAL_SPI_ERROR_NONE;
}

uint32_t SPI_Get_Error_Count(const SPI_HandleTypeDef *hspi)
{
    const Struct_SPI_Manage_Object *manage_object = SPI_Get_Manage_Object(hspi);
    return manage_object != nullptr ? manage_object->Error_Count : 0;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
