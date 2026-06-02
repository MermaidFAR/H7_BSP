/**
 * @file bsp_w25q64jv.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief W25Q64JV 外部 Flash 器件层驱动
 * @version 0.1
 * @date 2025-10-15 0.1 新建文档
 * @date 2026-06-02 0.2 迁移至 H7_BSP: 改短名 include
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_w25q64jv.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_W25Q64JV BSP_W25Q64JV;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化 W25Q64JV，轮询 JEDEC ID 直到芯片就绪
 *
 * @param __Flash_Mode 工作模式（Normal / MemoryMapped）
 */
void Class_W25Q64JV::Init(const Enum_W25Q64JV_Mode &__Flash_Mode)
{
    OSPI_Manage_Object = &OSPI2_Manage_Object;
    Flash_Mode = __Flash_Mode;

    // 读取 JEDEC ID（EF 40 17），确认芯片存活
    Command = COMMAND_DEFAULT_CONFIG;
    Command.NbData = 3;
    while (*reinterpret_cast<uint32_t *>(OSPI_Manage_Object->Rx_Buffer) != 0x001740EF)
    {
        OSPI_Command_Receive_Data(OSPI_Manage_Object->OSPI_Handler, &Command);
        Namespace_SYS_Timestamp::Delay_Millisecond(100);
    }

    if (__Flash_Mode == W25Q64JV_Mode_MemoryMapped)
    {
        // 配置内存映射模式（Quad I/O 快速读）
        Command = COMMAND_DEFAULT_CONFIG;
        Command.Instruction = W25Q64JV_Command_FAST_READ_QUAD_IO;
        Command.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
        Command.DataMode = HAL_OSPI_DATA_4_LINES;
        Command.DummyCycles = 6;
        OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);
        Namespace_SYS_Timestamp::Delay_Millisecond(100);

        OSPI_MemoryMappedTypeDef tmp_config = {0};
        HAL_OSPI_MemoryMapped(OSPI_Manage_Object->OSPI_Handler, &tmp_config);
    }
}

/**
 * @brief OSPI 自动轮询完成回调（WIP bit 清零，操作已结束）
 *
 */
void Class_W25Q64JV::OSPI_StatusMatchCallback()
{
    Busy_Flag = false;
    Write_Enable_Activated_Flag = false;

    if (Current_Instruction == W25Q64JV_Command_WRITE_ENABLE)
    {
        Write_Enable_Activated_Flag = true;
    }
}

/**
 * @brief OSPI 接收完成回调，接收后发起 WIP 轮询
 *
 */
void Class_W25Q64JV::OSPI_RxCallback()
{
    Auto_Polling_With_Timeout();
}

/**
 * @brief OSPI 发送完成回调，发送后发起 WIP 轮询
 *
 */
void Class_W25Q64JV::OSPI_TxCallback()
{
    Auto_Polling_With_Timeout();
}

/**
 * @brief 1ms 定时器回调，用于检测自动轮询超时
 *
 */
void Class_W25Q64JV::TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback()
{
    if (Busy_Flag && (SYS_Timestamp.Get_Current_Timestamp() - OSPI_Manage_Object->Auto_Polling_Timestamp > Current_Auto_Polling_Timeout))
    {
        Busy_Flag = false;
        Auto_Polling_Error_Count++;
    }
}

/**
 * @brief 带超时保护的自动轮询（读 SR1，等待 WIP=0）
 *
 */
void Class_W25Q64JV::Auto_Polling_With_Timeout()
{
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_READ_STATUS_REGISTER_1;
    Command.NbData = 1;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    OSPI_AutoPollingTypeDef tmp_config = AUTO_POLLING_DEFAULT_CONFIG;
    OSPI_Auto_Polling(OSPI_Manage_Object->OSPI_Handler, &tmp_config);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
