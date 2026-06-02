/**
 * @file bsp_w25q64jv.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief W25Q64JV 外部 Flash 器件层驱动
 * @version 0.1
 * @date 2025-10-15 0.1 新建文档
 * @date 2026-06-02 0.2 迁移至 H7_BSP: 改短名 include, 去除多余 HAL include
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef __BSP_W25Q64JV_H
#define __BSP_W25Q64JV_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_w25q64jv_register.h"
#include "bsp_ospi.h"
#include "cmsis_os.h"
#include <SEGGER_RTT.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

enum Enum_W25Q64JV_Mode
{
    W25Q64JV_Mode_Normal = 0,
    W25Q64JV_Mode_MemoryMapped,
};

/**
 * @brief Specialized, W25Q64JV 外部 Flash 芯片驱动
 *
 */
class Class_W25Q64JV
{
public:
    void Init(const Enum_W25Q64JV_Mode &__Flash_Mode = W25Q64JV_Mode_Normal);

    void Enable_Quad_Mode();

    inline void Get_Buffer(const uint32_t &Address, const uint8_t &Length);

    inline void Set_Write_Enable();

    inline void Set_Sector_Erased(const uint32_t &Address);

    inline void Set_Bolck_Erased_32K(const uint32_t &Address);

    inline void Set_Bolck_Erased_64K(const uint32_t &Address);

    inline void Set_Chip_Erased();

    inline void Set_Buffer(const uint8_t *Buffer, const uint32_t &Address, const uint8_t &Length);

    bool Is_Ready() { return !Is_Busy(); }

    inline void Read_Data(void *Dest, const uint32_t &Address, const uint32_t &Length);

    inline void Write_Data(const void *Src, const uint32_t &Address, const uint32_t &Length);

    void OSPI_StatusMatchCallback();

    void OSPI_RxCallback();

    void OSPI_TxCallback();

    void TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();

  protected:
    // 绑定的OSPI管理对象
    Struct_OSPI_Manage_Object *OSPI_Manage_Object;
    // Flash工作模式
    Enum_W25Q64JV_Mode Flash_Mode;

    // W25Q64JV默认命令配置（JEDEC ID读取）
    const OSPI_RegularCmdTypeDef COMMAND_DEFAULT_CONFIG = {
        .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
        .FlashId = HAL_OSPI_FLASH_ID_1,
        .Instruction = W25Q64JV_Command_JEDEC_ID,
        .InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
        .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
        .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
        .Address = 0,
        .AddressMode = HAL_OSPI_ADDRESS_NONE,
        .AddressSize = HAL_OSPI_ADDRESS_24_BITS,
        .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
        .AlternateBytes = 0,
        .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
        .AlternateBytesSize = 0,
        .AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE,
        .DataMode = HAL_OSPI_DATA_NONE,
        .NbData = 0,
        .DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
        .DummyCycles = 0,
        .DQSMode = HAL_OSPI_DQS_DISABLE,
        .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
    };

    // W25Q64JV默认自动轮询配置（轮询 WIP bit = SR1[0]）
    const OSPI_AutoPollingTypeDef AUTO_POLLING_DEFAULT_CONFIG = {
        .Match = 0x00,
        .Mask = 0x01,
        .MatchMode = HAL_OSPI_MATCH_MODE_AND,
        .AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
        .Interval = 5,
    };

    // 轮询超时时间（微秒）
    const uint64_t AUTOPOLLING_DEFAULT_TIMEOUT = 5000000;           // 5000ms
    const uint64_t AUTOPOLLING_SECTOR_ERASED_TIMEOUT = 400000;      // 400ms
    const uint64_t AUTOPOLLING_BLOCK_ERASED_32K_TIMEOUT = 1600000;  // 1600ms
    const uint64_t AUTOPOLLING_BLOCK_ERASED_64K_TIMEOUT = 2000000;  // 2000ms
    const uint64_t AUTOPOLLING_CHIP_ERASED_TIMEOUT = 100000000;     // 100000ms

    // 写使能激活标志
    bool Write_Enable_Activated_Flag = false;
    // 忙标志（发出指令/收发数据后置1，StatusMatch回调中清零）
    bool Busy_Flag = false;
    // 置忙时间戳（微秒）
    uint64_t Busy_Timestamp = 0;

    // 当前命令配置
    OSPI_RegularCmdTypeDef Command = COMMAND_DEFAULT_CONFIG;
    uint32_t Current_Instruction = 0;

    // 当前指令对应的轮询超时
    uint64_t Current_Auto_Polling_Timeout = AUTOPOLLING_DEFAULT_TIMEOUT;
    // 轮询超时错误计数
    uint32_t Auto_Polling_Error_Count = 0;
    // 抑制 Tx/Rx 回调中的自动轮询（Enable_Quad_Mode 手动轮询 WIP 时使用）
    bool Suppress_AutoPolling = false;

    bool Is_Busy()
    {
        if (Busy_Flag)
        {
            if (SYS_Timestamp.Get_Current_Timestamp() - Busy_Timestamp > Current_Auto_Polling_Timeout)
            {
                SEGGER_RTT_printf(0, "TIMEOUT! Instr=%02X\n", Current_Instruction);
                Busy_Flag = false;
                Auto_Polling_Error_Count++;
                return false;
            }
            return true;
        }
        return false;
    }

    void Auto_Polling_With_Timeout();
};

/* Exported variables ---------------------------------------------------------*/

extern Class_W25Q64JV BSP_W25Q64JV;

/* Exported function prototypes -----------------------------------------------*/

/**
 * @brief 读数据（1-4-4 Quad I/O 快速读）
 *
 * @param Address 起始地址
 * @param Length  读取字节数（≤ OSPI_BUFFER_SIZE）
 */
inline void Class_W25Q64JV::Get_Buffer(const uint32_t &Address, const uint8_t &Length)
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_FAST_READ_QUAD_IO;
    Command.Address = Address;
    Command.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    Command.DataMode = HAL_OSPI_DATA_4_LINES;
    Command.DummyCycles = 6;
    Command.NbData = Length;
    OSPI_Command_Receive_Data(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_FAST_READ_QUAD_IO;
}

/**
 * @brief 发送写使能指令（06h），后续写/擦操作必须先调用
 *
 */
inline void Class_W25Q64JV::Set_Write_Enable()
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_WRITE_ENABLE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_WRITE_ENABLE;
    Current_Auto_Polling_Timeout = AUTOPOLLING_DEFAULT_TIMEOUT;
    Auto_Polling_With_Timeout();
}

/**
 * @brief 擦除单个 Sector（4KB），地址必须 4096 字节对齐
 *
 */
inline void Class_W25Q64JV::Set_Sector_Erased(const uint32_t &Address)
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (!Write_Enable_Activated_Flag)
    {
        Busy_Flag = false;
        return;
    }

    if (Address % 4096)
    {
        Busy_Flag = false;
        return;
    }

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_SECTOR_ERASE;
    Command.Address = Address;
    Command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_SECTOR_ERASE;
    Current_Auto_Polling_Timeout = AUTOPOLLING_SECTOR_ERASED_TIMEOUT;
    Auto_Polling_With_Timeout();
}

/**
 * @brief 擦除单个 32K Block，地址必须 32768 字节对齐
 *
 */
inline void Class_W25Q64JV::Set_Bolck_Erased_32K(const uint32_t &Address)
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (!Write_Enable_Activated_Flag)
    {
        Busy_Flag = false;
        return;
    }

    if (Address % 32768)
    {
        Busy_Flag = false;
        return;
    }

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_BLOCK_ERASE_32K;
    Command.Address = Address;
    Command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_BLOCK_ERASE_32K;
    Current_Auto_Polling_Timeout = AUTOPOLLING_BLOCK_ERASED_32K_TIMEOUT;
    Auto_Polling_With_Timeout();
}

/**
 * @brief 擦除单个 64K Block，地址必须 65536 字节对齐
 *
 */
inline void Class_W25Q64JV::Set_Bolck_Erased_64K(const uint32_t &Address)
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (!Write_Enable_Activated_Flag)
    {
        Busy_Flag = false;
        return;
    }

    if (Address % 65536)
    {
        Busy_Flag = false;
        return;
    }

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_BLOCK_ERASE_64K;
    Command.Address = Address;
    Command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_BLOCK_ERASE_64K;
    Current_Auto_Polling_Timeout = AUTOPOLLING_BLOCK_ERASED_64K_TIMEOUT;
    Auto_Polling_With_Timeout();
}

/**
 * @brief 擦除整片 Chip（耗时约 100s）
 *
 */
inline void Class_W25Q64JV::Set_Chip_Erased()
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (!Write_Enable_Activated_Flag)
    {
        Busy_Flag = false;
        return;
    }

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_CHIP_ERASE;
    OSPI_Command(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_CHIP_ERASE;
    Current_Auto_Polling_Timeout = AUTOPOLLING_CHIP_ERASED_TIMEOUT;
    Auto_Polling_With_Timeout();
}

/**
 * @brief 页写数据（1-1-4 Quad 页编程）
 *        写前必须调用 Set_Write_Enable()；写完在 TxCallback 中自动发起轮询
 *
 * @param Buffer  源数据指针
 * @param Address 目标地址（不跨 Page，即 Address%256 + Length ≤ 256）
 * @param Length  写入字节数
 */
inline void Class_W25Q64JV::Set_Buffer(const uint8_t *Buffer, const uint32_t &Address, const uint8_t &Length)
{
    if (Is_Busy())
    {
        return;
    }
    Busy_Flag = true;
    Busy_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    if (!Write_Enable_Activated_Flag)
    {
        Busy_Flag = false;
        return;
    }

    // 不允许跨 Page（256B 边界）写
    if (Address % 256 + Length > 256)
    {
        Busy_Flag = false;
        return;
    }

    Command = COMMAND_DEFAULT_CONFIG;
    Command.Instruction = W25Q64JV_Command_QUAD_INPUT_PAGE_PROGRAM;
    Command.Address = Address;
    Command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    Command.DataMode = HAL_OSPI_DATA_4_LINES;
    Command.NbData = Length;
    memcpy(OSPI_Manage_Object->Tx_Buffer, Buffer, Length);
    OSPI_Command_Transmit_Data(OSPI_Manage_Object->OSPI_Handler, &Command);

    Current_Instruction = W25Q64JV_Command_QUAD_INPUT_PAGE_PROGRAM;
    Current_Auto_Polling_Timeout = AUTOPOLLING_DEFAULT_TIMEOUT;
}

/**
 * @brief 同步读取任意长度数据（分次 DMA 接收，自动等待）
 *
 */
inline void Class_W25Q64JV::Read_Data(void *Dest, const uint32_t &Address, const uint32_t &Length)
{
    uint8_t *dst = static_cast<uint8_t *>(Dest);
    uint32_t addr = Address;
    uint32_t remaining = Length;

    while (remaining > 0)
    {
        uint32_t chunk = (remaining > OSPI_BUFFER_SIZE) ? OSPI_BUFFER_SIZE : remaining;

        Get_Buffer(addr, chunk);
        while (!Is_Ready()) { osDelay(1); }

        memcpy(dst, OSPI_Manage_Object->Rx_Buffer, chunk);

        dst += chunk;
        addr += chunk;
        remaining -= chunk;
    }
}

/**
 * @brief 同步写入任意长度数据（自动处理页边界和写使能）
 *
 */
inline void Class_W25Q64JV::Write_Data(const void *Src, const uint32_t &Address, const uint32_t &Length)
{
    const uint8_t *src = static_cast<const uint8_t *>(Src);
    uint32_t addr = Address;
    uint32_t remaining = Length;

    while (remaining > 0)
    {
        uint32_t page_remain = 256 - (addr % 256);
        uint32_t chunk = (remaining > page_remain) ? page_remain : remaining;

        Set_Write_Enable();
        while (!Is_Ready()) { osDelay(1); }

        Set_Buffer(src, addr, chunk);
        while (!Is_Ready()) { osDelay(1); }

        src += chunk;
        addr += chunk;
        remaining -= chunk;
    }
}

#endif /* __BSP_W25Q64JV_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
