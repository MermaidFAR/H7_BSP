/**
 * @file bsp_ospi.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的OSPI通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 新建文档
 * @date 2026-06-01 0.2 迁移至 H7_BSP: 改 bsp_ 命名、HAL 回调加 extern "C"、缓冲入 .dma_buffer
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_ospi.h"

/* Private macros ------------------------------------------------------------*/

#define OSPI_COMMAND_TIMEOUT 1

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_OSPI_Manage_Object OSPI1_Manage_Object = {nullptr};

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_OSPI_Manage_Object OSPI2_Manage_Object = {nullptr};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

static Struct_OSPI_Manage_Object *OSPI_Get_Manage_Object(OSPI_HandleTypeDef *hospi)
{
    if (hospi == nullptr)
    {
        return nullptr;
    }
    if (hospi->Instance == OCTOSPI1)
    {
        return &OSPI1_Manage_Object;
    }
    if (hospi->Instance == OCTOSPI2)
    {
        return &OSPI2_Manage_Object;
    }
    return nullptr;
}

/**
 * @brief 初始化OSPI
 *
 * @param hospi OSPI编号
 * @param Auto_Polling_Callback_Function 自动轮询完成回调函数
 * @param Rx_Callback_Function 接收完成回调函数
 * @param Tx_Callback_Function 发送完成回调函数
 */
void OSPI_Init(OSPI_HandleTypeDef *hospi, OSPI_Status_Match_Callback Auto_Polling_Callback_Function, OSPI_Rx_Callback Rx_Callback_Function, OSPI_Tx_Callback Tx_Callback_Function)
{
    Struct_OSPI_Manage_Object *manage = OSPI_Get_Manage_Object(hospi);
    if (manage == nullptr)
    {
        return;
    }
    memset(manage, 0, sizeof(*manage));
    manage->OSPI_Handler = hospi;
    manage->Status_Match_Callback_Function = Auto_Polling_Callback_Function;
    manage->Rx_Callback_Function = Rx_Callback_Function;
    manage->Tx_Callback_Function = Tx_Callback_Function;
    HAL_OSPI_SetTimeout(hospi, OSPI_COMMAND_TIMEOUT);
}

/**
 * @brief 自动轮询
 *
 */
HAL_StatusTypeDef OSPI_Auto_Polling(OSPI_HandleTypeDef *hospi, OSPI_AutoPollingTypeDef *Config)
{
    if (OSPI_Get_Manage_Object(hospi) == nullptr || Config == nullptr)
    {
        return HAL_ERROR;
    }
    return HAL_OSPI_AutoPolling_IT(hospi, Config);
}

/**
 * @brief 发送指令
 *
 * @param hospi OSPI编号
 */
HAL_StatusTypeDef OSPI_Command(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *Command)
{
    if (OSPI_Get_Manage_Object(hospi) == nullptr || Command == nullptr)
    {
        return HAL_ERROR;
    }
    return HAL_OSPI_Command(hospi, Command, OSPI_COMMAND_TIMEOUT);
}

/**
 * @brief 发送数据
 *
 * @param hospi OSPI编号
 */
HAL_StatusTypeDef OSPI_Command_Transmit_Data(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *Command)
{
    Struct_OSPI_Manage_Object *manage = OSPI_Get_Manage_Object(hospi);
    if (manage == nullptr || Command == nullptr || Command->DataMode == HAL_OSPI_DATA_NONE ||
        Command->NbData == 0 || Command->NbData > OSPI_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef status = OSPI_Command(hospi, Command);
    if (status != HAL_OK)
    {
        return status;
    }
    return HAL_OSPI_Transmit_DMA(hospi, manage->Tx_Buffer);
}

/**
 * @brief 接收数据
 *
 * @param hospi OSPI编号
 */
HAL_StatusTypeDef OSPI_Command_Receive_Data(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *Command)
{
    Struct_OSPI_Manage_Object *manage = OSPI_Get_Manage_Object(hospi);
    if (manage == nullptr || Command == nullptr || Command->DataMode == HAL_OSPI_DATA_NONE ||
        Command->NbData == 0 || Command->NbData > OSPI_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef status = OSPI_Command(hospi, Command);
    if (status != HAL_OK)
    {
        return status;
    }
    return HAL_OSPI_Receive_DMA(hospi, manage->Rx_Buffer);
}

/**
 * @brief 交互数据
 *
 * @param hospi OSPI编号
 */
HAL_StatusTypeDef OSPI_Command_Transmit_Receive_Data(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *Command)
{
    // 不能在同一指令上紧接着启动 TX DMA 和 RX DMA；后者会遇到 BUSY_TX。
    // 调用者必须分别提交写/读指令，并在完成回调后切换方向。
    (void)hospi;
    (void)Command;
    return HAL_ERROR;
}

/**
 * @brief HAL库OSPI自动轮询回调函数
 *
 */
extern "C" void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef *hospi)
{
    if (hospi->Instance == OCTOSPI1)
    {
        OSPI1_Manage_Object.Auto_Polling_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

        if (OSPI1_Manage_Object.Status_Match_Callback_Function != nullptr)
        {
            OSPI1_Manage_Object.Status_Match_Callback_Function();
        }
    }
    else if (hospi->Instance == OCTOSPI2)
    {
        OSPI2_Manage_Object.Auto_Polling_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

        if (OSPI2_Manage_Object.Status_Match_Callback_Function != nullptr)
        {
            OSPI2_Manage_Object.Status_Match_Callback_Function();
        }
    }
}

/**
 * @brief HAL库OSPI接收回调函数
 *
 * @param hospi OSPI编号
 */
extern "C" void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi)
{
    if (hospi->Instance == OCTOSPI1)
    {
        OSPI1_Manage_Object.Rx_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

        if (OSPI1_Manage_Object.Rx_Callback_Function != nullptr)
        {
            OSPI1_Manage_Object.Rx_Callback_Function(OSPI1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hospi->Instance == OCTOSPI2)
    {
        OSPI2_Manage_Object.Rx_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

        if (OSPI2_Manage_Object.Rx_Callback_Function != nullptr)
        {
            OSPI2_Manage_Object.Rx_Callback_Function(OSPI2_Manage_Object.Rx_Buffer);
        }
    }
}

/**
 * @brief HAL库OSPI发送回调函数
 *
 * @param hospi OSPI编号
 */
extern "C" void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi)
{
    if (hospi->Instance == OCTOSPI1)
    {
        if (OSPI1_Manage_Object.Tx_Callback_Function != nullptr)
        {
            OSPI1_Manage_Object.Tx_Callback_Function(OSPI1_Manage_Object.Tx_Buffer);
        }
    }
    else if (hospi->Instance == OCTOSPI2)
    {
        if (OSPI2_Manage_Object.Tx_Callback_Function != nullptr)
        {
            OSPI2_Manage_Object.Tx_Callback_Function(OSPI2_Manage_Object.Tx_Buffer);
        }
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
