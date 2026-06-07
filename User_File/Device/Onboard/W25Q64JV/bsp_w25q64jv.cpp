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

#include <SEGGER_RTT.h>

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

    // 重置 OSPI 状态机（避免之前操作残留的错误状态）
    OSPI_Manage_Object->OSPI_Handler->State = HAL_OSPI_STATE_READY;
    __HAL_OSPI_CLEAR_FLAG(OSPI_Manage_Object->OSPI_Handler, HAL_OSPI_FLAG_SM | HAL_OSPI_FLAG_TC | HAL_OSPI_FLAG_TE);

    // 读取 JEDEC ID（EF 40 17），确认芯片存活
    Command = COMMAND_DEFAULT_CONFIG;
    Command.DataMode = HAL_OSPI_DATA_1_LINE;
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
 * @brief 在 RTOS 中启用 Quad 模式（设置 SR2 QE 位）
 *        走标准 DMA + 回调链，调用前确保 RTOS 已启动
 *
 */
void Class_W25Q64JV::Enable_Quad_Mode()
{
    SEGGER_RTT_printf(0, "QE start\n");

    // 硬件复位 Flash（确保干净状态）
    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_ENABLE_RESET;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);
    osDelay(1);
    Command.Instruction = W25Q64JV_Command_RESET_DEVICE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);
    osDelay(50);
    SEGGER_RTT_printf(0, "Reset done\n");

    // 发送 Write Enable
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
    Current_Instruction = W25Q64JV_Command_WRITE_ENABLE;
    Current_Auto_Polling_Timeout = AUTOPOLLING_DEFAULT_TIMEOUT;

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_WRITE_ENABLE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    Auto_Polling_With_Timeout();
    while (Is_Busy())
    {
        osDelay(1);
    }

    SEGGER_RTT_printf(0, "QE WE done\n");

    // 验证 WEL 是否真的置位
    Suppress_AutoPolling = true;
    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_READ_STATUS_REGISTER_1;
    Command.DataMode = HAL_OSPI_DATA_1_LINE;
    Command.NbData = 1;
    OSPI_Command_Receive_Data(OSPI_Manage_Object->OSPI_Handler, &Command);
    osDelay(5);
    uint8_t wel_check = OSPI_Manage_Object->Rx_Buffer[0];
    SEGGER_RTT_printf(0, "WEL check: SR1=%02X (WEL=%d WIP=%d)\n",
                      wel_check, (wel_check >> 1) & 1, wel_check & 1);

    // 写 SR2 = 0x02（QE = 1），抑制回调中的 AutoPolling，手动轮询 WIP
    OSPI_Manage_Object->Tx_Buffer[0] = 0x02;

    Suppress_AutoPolling = true;

    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
    Current_Instruction = W25Q64JV_Command_WRITE_STATUS_REGISTER_2;
    Current_Auto_Polling_Timeout = AUTOPOLLING_DEFAULT_TIMEOUT;

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_WRITE_STATUS_REGISTER_2;
    Command.DataMode = HAL_OSPI_DATA_1_LINE;
    Command.NbData = 1;

    OSPI_Command_Transmit_Data(OSPI_Manage_Object->OSPI_Handler, &Command);
    osDelay(10);

    // 手动轮询 WIP，回调链已被 Suppress_AutoPolling 抑制，无冲突
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();
    while (1)
    {
        Command = COMMAND_DEFAULT_CONFIG;
        Command.Instruction = W25Q64JV_Command_READ_STATUS_REGISTER_1;
        Command.NbData = 1;
        OSPI_Command_Receive_Data(OSPI_Manage_Object->OSPI_Handler, &Command);
        osDelay(1);

        if ((OSPI_Manage_Object->Rx_Buffer[0] & 0x01) == 0)
            break;

        if (SYS_Timestamp.Get_Current_Timestamp() - Busy_Timestamp > Current_Auto_Polling_Timeout)
        {
            Auto_Polling_Error_Count++;
            SEGGER_RTT_printf(0, "QE WIP timeout\n");
            break;
        }
    }

    // 回读 SR1/SR2 确认芯片状态
    Suppress_AutoPolling = true;

    uint8_t sr1, sr2;

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_READ_STATUS_REGISTER_1;
    Command.DataMode = HAL_OSPI_DATA_1_LINE;
    Command.NbData = 1;
    OSPI_Command_Receive_Data(OSPI_Manage_Object->OSPI_Handler, &Command);
    osDelay(5);
    sr1 = OSPI_Manage_Object->Rx_Buffer[0];

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_READ_STATUS_REGISTER_2;
    Command.DataMode = HAL_OSPI_DATA_1_LINE;
    Command.NbData = 1;
    OSPI_Command_Receive_Data(OSPI_Manage_Object->OSPI_Handler, &Command);
    osDelay(5);
    sr2 = OSPI_Manage_Object->Rx_Buffer[0];

    SEGGER_RTT_printf(0, "SR1=%02X SR2=%02X (WEL=%d BP=%d QE=%d)\n",
                      sr1, sr2,
                      sr1 & 1, (sr1 >> 2) & 0x1F, (sr2 >> 1) & 1);

    Suppress_AutoPolling = false;
    Busy_Flag = false;

    SEGGER_RTT_printf(0, "QE done err=%d\n", Auto_Polling_Error_Count);
}

/**
 * @brief OSPI 自动轮询完成回调（WIP bit 清零，操作已结束）
 *
 */
void Class_W25Q64JV::OSPI_StatusMatchCallback()
{
    SEGGER_RTT_printf(0, "StatMatch Busy=%d Instr=%02X\n", Busy_Flag, Current_Instruction);

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
    if (!Suppress_AutoPolling)
        Auto_Polling_With_Timeout();
}

/**
 * @brief OSPI 发送完成回调，发送后发起 WIP 轮询
 *
 */
void Class_W25Q64JV::OSPI_TxCallback()
{
    if (!Suppress_AutoPolling)
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
    SEGGER_RTT_printf(0, "AP start\n");
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_READ_STATUS_REGISTER_1;
    Command.DataMode = HAL_OSPI_DATA_1_LINE;
    Command.NbData = 1;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    OSPI_AutoPollingTypeDef tmp_config = AUTO_POLLING_DEFAULT_CONFIG;
    OSPI_Auto_Polling(OSPI_Manage_Object->OSPI_Handler, &tmp_config);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
