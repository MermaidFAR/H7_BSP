/**
 * @file    bsp_uart.cpp
 * @brief   板级支持包：UART 通信初始化与配置流程（基于 STM32H7 + DMA 双缓冲）
 * @details 仿照 SCUT-Robotlab / 达妙 drv_uart 范式改写，适配 H7_BSP 工程。
 *          - 管理对象（含 DMA 收发缓冲区）放入 .dma_buffer 段（RAM_D1，DMA1/DMA2 可访问）
 *          - HAL 回调以 extern "C" 定义以正确覆写 HAL 弱符号
 * @note    仅接管具备 RX DMA 的 7 路：USART1/2/3、UART5、USART6、UART7、USART10。
 *          UART4、UART8、UART9 无 DMA（H7 DMA stream 已占满），不在此驱动接管。
 *          UART5 仅有 RX DMA、无 TX DMA，发送时自动回退为阻塞发送。
 *
 * @author  zzm（仿 yssickjgd / USTC-RoboWalker drv_uart）
 * @version 1.0
 * @date    2026-06-01
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"

/* Private macros ------------------------------------------------------------*/

// 阻塞发送超时（ms），用于无 TX DMA 的 UART5
#define UART_TX_BLOCKING_TIMEOUT 10

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 管理对象含 DMA 缓冲区，必须放入 .dma_buffer 段（链接到 RAM_D1，DMA1/DMA2 可访问）
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object USART1_Manage_Object;
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object USART2_Manage_Object;
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object USART3_Manage_Object;
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object UART5_Manage_Object;
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object USART6_Manage_Object;
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object UART7_Manage_Object;
__attribute__((section(".dma_buffer"), aligned(32))) Struct_UART_Manage_Object USART10_Manage_Object;

/* Private function declarations ---------------------------------------------*/

static Struct_UART_Manage_Object *UART_Get_Manage_Object(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_Start_Receive_ToIdle(Struct_UART_Manage_Object *manage);
static bool UART_Restart_Receive(Struct_UART_Manage_Object *manage);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 根据外设实例取得对应管理对象
 *
 * @param huart UART 句柄
 * @return Struct_UART_Manage_Object* 对应管理对象，未接管的实例返回 nullptr
 */
static Struct_UART_Manage_Object *UART_Get_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        return (&USART1_Manage_Object);
    }
    else if (huart->Instance == USART2)
    {
        return (&USART2_Manage_Object);
    }
    else if (huart->Instance == USART3)
    {
        return (&USART3_Manage_Object);
    }
    else if (huart->Instance == UART5)
    {
        return (&UART5_Manage_Object);
    }
    else if (huart->Instance == USART6)
    {
        return (&USART6_Manage_Object);
    }
    else if (huart->Instance == UART7)
    {
        return (&UART7_Manage_Object);
    }
    else if (huart->Instance == USART10)
    {
        return (&USART10_Manage_Object);
    }

    // UART4 / UART8 / UART9 无 DMA，未接管
    return (nullptr);
}

/**
 * @brief 启动空闲 DMA 接收并关闭不需要的半传输中断。
 * @details 半传输事件发生时 DMA 仍是 BUSY，不能切换缓冲并重新启动；本驱动只在
 *          IDLE 或整缓冲完成后消费数据，因此关闭 HT 可避免偶发 BUSY 和字节缺口。
 */
static HAL_StatusTypeDef UART_Start_Receive_ToIdle(Struct_UART_Manage_Object *manage)
{
    if (manage == nullptr || manage->UART_Handler == nullptr ||
        manage->UART_Handler->hdmarx == nullptr || manage->Rx_Buffer_Active == nullptr)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef Status = HAL_UARTEx_ReceiveToIdle_DMA(
        manage->UART_Handler,
        manage->Rx_Buffer_Active,
        UART_BUFFER_SIZE);
    if (Status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(manage->UART_Handler->hdmarx, DMA_IT_HT);
    }
    return Status;
}

/**
 * @brief 在任务上下文清理 UART/DMA 状态并重新启动空闲 DMA 接收。
 * @note  HAL_UART_AbortReceive 会关闭旧 DMA、清除 ORE/FE/NE/PE 并冲掉 RDR。
 */
static bool UART_Restart_Receive(Struct_UART_Manage_Object *manage)
{
    if (manage == nullptr || manage->UART_Handler == nullptr ||
        manage->UART_Handler->hdmarx == nullptr)
    {
        return false;
    }

    UART_HandleTypeDef *huart = manage->UART_Handler;
    manage->Rx_Buffer_Active = nullptr;

    if (HAL_UART_AbortReceive(huart) != HAL_OK)
    {
        manage->Rx_Restart_Pending = true;
        manage->Rx_Restart_Failure_Count++;
        return false;
    }

    // AbortReceive 已清除错误位和 RDR；这里再清 IDLE，避免旧空闲标志抢先中断。
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    if (UART_Start_Receive_ToIdle(manage) != HAL_OK)
    {
        manage->Rx_Buffer_Active = nullptr;
        manage->Rx_Restart_Pending = true;
        manage->Rx_Restart_Failure_Count++;
        return false;
    }

    manage->Rx_Restart_Pending = false;
    manage->Rx_Restart_Count++;
    return true;
}

/**
 * @brief 初始化 UART，绑定回调并启动 DMA 接收
 *
 * @param huart UART 句柄
 * @param Callback_Function 接收处理回调函数
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback_Function)
{
    Struct_UART_Manage_Object *manage = UART_Get_Manage_Object(huart);
    if (manage == nullptr)
    {
        return;
    }

    memset(manage, 0, sizeof(Struct_UART_Manage_Object));

    manage->UART_Handler = huart;
    manage->Callback_Function = Callback_Function;

    manage->Rx_Buffer_Ready = manage->Rx_Buffer_1;

    // Some managed UARTs are TX-only in the current CubeMX DMA allocation.
    // Starting the HAL DMA receive path without hdmarx still sets DMAR/IDLEIE,
    // then the first IDLE interrupt dereferences a null DMA handle.
    if (huart->hdmarx == nullptr)
    {
        return;
    }

    // 即使线路上电时已经持续发送，也先清除溢出和残留字节后再接管 DMA。
    UART_Restart_Receive(manage);
}

/**
 * @brief 重新初始化 UART（错误恢复时重启 DMA 接收）
 *
 * @param huart UART 句柄
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *manage = UART_Get_Manage_Object(huart);
    if (manage == nullptr || huart->hdmarx == nullptr)
    {
        return;
    }

    UART_Restart_Receive(manage);
}

/**
 * @brief UART 接收恢复服务，每 1 ms 由系统任务调用，内部每 10 ms 重试一次。
 * @details 错误中断只设置 Rx_Restart_Pending，本函数在任务上下文持续重试，
 *          避免在中断里递归调用 HAL 或一次失败后永久停止接收。
 */
void UART_TIM_1ms_Recover_PeriodElapsedCallback(void)
{
    static uint8_t Retry_Divider = 0U;
    Retry_Divider++;
    if (Retry_Divider < 10U)
    {
        return;
    }
    Retry_Divider = 0U;

    Struct_UART_Manage_Object *const Manage_List[] = {
        &USART1_Manage_Object,
        &USART2_Manage_Object,
        &USART3_Manage_Object,
        &UART5_Manage_Object,
        &USART6_Manage_Object,
        &UART7_Manage_Object,
        &USART10_Manage_Object,
    };

    for (Struct_UART_Manage_Object *manage : Manage_List)
    {
        if (manage->Rx_Restart_Pending)
        {
            UART_Restart_Receive(manage);
        }
    }
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART 句柄
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态（HAL_StatusTypeDef）
 * @note  有 TX DMA 的走 DMA 发送；无 TX DMA（如 UART5）回退为阻塞发送
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    if (huart->hdmatx != nullptr)
    {
        return (HAL_UART_Transmit_DMA(huart, Data, Length));
    }

    return (HAL_UART_Transmit(huart, Data, Length, UART_TX_BLOCKING_TIMEOUT));
}

/**
 * @brief HAL 库 UART 接收 DMA 空闲中断回调
 *
 * @param huart UART 句柄
 * @param Size 本帧接收到的字节长度
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Struct_UART_Manage_Object *manage = UART_Get_Manage_Object(huart);
    if (manage == nullptr || huart->hdmarx == nullptr || manage->Rx_Buffer_Active == nullptr)
    {
        return;
    }

    // 程序未初始化完成时，仅重启接收不分发回调
    if (!init_finished)
    {
        if (UART_Start_Receive_ToIdle(manage) != HAL_OK)
        {
            manage->Rx_Buffer_Active = nullptr;
            manage->Rx_Restart_Pending = true;
            manage->Rx_Restart_Failure_Count++;
        }
        return;
    }

    // 双缓冲切换：刚收满的设为 Ready，另一块设为 Active
    manage->Rx_Buffer_Ready = manage->Rx_Buffer_Active;
    if (manage->Rx_Buffer_Active == manage->Rx_Buffer_0)
    {
        manage->Rx_Buffer_Active = manage->Rx_Buffer_1;
    }
    else
    {
        manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    }

    manage->Rx_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (UART_Start_Receive_ToIdle(manage) != HAL_OK)
    {
        manage->Rx_Buffer_Active = nullptr;
        manage->Rx_Restart_Pending = true;
        manage->Rx_Restart_Failure_Count++;
    }

    if (manage->Callback_Function != nullptr)
    {
        manage->Callback_Function(manage->Rx_Buffer_Ready, Size);
    }
}

/**
 * @brief HAL 库 UART 错误中断回调
 *
 * @param huart UART 句柄
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *manage = UART_Get_Manage_Object(huart);
    if (manage == nullptr || huart->hdmarx == nullptr)
    {
        return;
    }

    // 中断上下文只记录故障并请求任务恢复，避免递归调用 HAL 和二次竞态。
    manage->Rx_Buffer_Active = nullptr;
    manage->Rx_Restart_Pending = true;
    manage->Rx_Error_Count++;
}
