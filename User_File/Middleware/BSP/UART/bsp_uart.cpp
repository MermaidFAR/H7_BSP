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

    manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    manage->Rx_Buffer_Ready = manage->Rx_Buffer_1;

    HAL_UARTEx_ReceiveToIdle_DMA(huart, manage->Rx_Buffer_Active, UART_BUFFER_SIZE);
}

/**
 * @brief 重新初始化 UART（错误恢复时重启 DMA 接收）
 *
 * @param huart UART 句柄
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *manage = UART_Get_Manage_Object(huart);
    if (manage == nullptr)
    {
        return;
    }

    manage->Rx_Buffer_Active = manage->Rx_Buffer_0;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, manage->Rx_Buffer_Active, UART_BUFFER_SIZE);
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
    if (manage == nullptr)
    {
        return;
    }

    // 程序未初始化完成时，仅重启接收不分发回调
    if (!init_finished)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, manage->Rx_Buffer_Active, UART_BUFFER_SIZE);
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

    HAL_UARTEx_ReceiveToIdle_DMA(huart, manage->Rx_Buffer_Active, UART_BUFFER_SIZE);

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
    UART_Reinit(huart);
}