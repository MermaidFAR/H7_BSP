/**
 * @file bsp_w25q64jv_register.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief W25Q64JV 指令寄存器定义
 * @version 0.1
 * @date 2025-10-13 0.1 新建文档
 * @date 2026-06-02 0.2 迁移至 H7_BSP
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef __BSP_W25Q64JV_REGISTER_H
#define __BSP_W25Q64JV_REGISTER_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

enum Enum_W25Q64JV_Command : uint8_t
{
    // 标准SPI命令

    // 读写状态寄存器
    W25Q64JV_Command_READ_STATUS_REGISTER_1 = 0x05,
    W25Q64JV_Command_READ_STATUS_REGISTER_2 = 0x35,
    W25Q64JV_Command_READ_STATUS_REGISTER_3 = 0x15,
    W25Q64JV_Command_WRITE_STATUS_REGISTER_1 = 0x01,
    W25Q64JV_Command_WRITE_STATUS_REGISTER_2 = 0x31,
    W25Q64JV_Command_WRITE_STATUS_REGISTER_3 = 0x11,

    // 读数据, 仅在标准SPI模式下使用, 默认50MHz
    W25Q64JV_Command_READ_DATA = 0x03,
    // 快速读数据, 比普通读数据多了个dummy字节, 给芯片反应时间, 可达104或133MHz
    W25Q64JV_Command_FAST_READ = 0x0b,

    // 擦除命令
    W25Q64JV_Command_SECTOR_ERASE = 0x20,
    W25Q64JV_Command_BLOCK_ERASE_32K = 0x52,
    W25Q64JV_Command_BLOCK_ERASE_64K = 0xd8,
    W25Q64JV_Command_CHIP_ERASE = 0xc7,

    // 擦除/写入暂停与恢复
    W25Q64JV_Command_ERASE_PROGRAM_SUSPEND = 0x75,
    W25Q64JV_Command_ERASE_PROGRAM_RESUME = 0x7a,

    // 写使能 / 失能
    W25Q64JV_Command_WRITE_ENABLE = 0x06,
    W25Q64JV_Command_WRITE_DISABLE = 0x04,
    // 易失性SR写入使能
    W25Q64JV_Command_VOLATILE_SR_WRITE_ENABLE = 0x50,

    // 页编程, 每次最多256B
    W25Q64JV_Command_PAGE_PROGRAM = 0x02,

    // 安全寄存器
    W25Q64JV_Command_READ_SECURITY_REGISTER = 0x48,
    W25Q64JV_Command_ERASE_SECURITY_REGISTER = 0x44,
    W25Q64JV_Command_PROGRAM_SECURITY_REGISTER = 0x42,

    // 块锁定
    W25Q64JV_Command_READ_BLOCK_LOCK = 0x3d,
    W25Q64JV_Command_INDIVIDUAL_BLOCK_LOCK = 0x36,
    W25Q64JV_Command_INDIVIDUAL_BLOCK_UNLOCK = 0x39,
    W25Q64JV_Command_GLOBAL_BLOCK_LOCK = 0x7e,
    W25Q64JV_Command_GLOBAL_BLOCK_UNLOCK = 0x98,

    // 复位命令, 按顺序发送 66, 99 才可复位
    W25Q64JV_Command_ENABLE_RESET = 0x66,
    W25Q64JV_Command_RESET_DEVICE = 0x99,

    // 掉电模式
    W25Q64JV_Command_POWER_DOWN = 0xb9,
    W25Q64JV_Command_RELEASE_POWER_DOWN = 0xab,

    // ID 读取
    W25Q64JV_Command_MFTR_DEVICE_ID = 0x90,
    W25Q64JV_Command_READ_UNIQUE_ID = 0x4b,
    W25Q64JV_Command_JEDEC_ID = 0x9f,
    W25Q64JV_Command_READ_SFDP_REGISTER = 0x5a,

    // 多线SPI命令
    W25Q64JV_Command_FAST_READ_DUAL_OUTPUT = 0x3b,
    W25Q64JV_Command_FAST_READ_DUAL_IO = 0xbb,
    W25Q64JV_Command_FAST_READ_QUAD_OUTPUT = 0x6b,
    // 1-4-4 Quad I/O 快速读
    W25Q64JV_Command_FAST_READ_QUAD_IO = 0xeb,

    // 1-1-4 Quad 页编程, 每次最多256B
    W25Q64JV_Command_QUAD_INPUT_PAGE_PROGRAM = 0x32,

    W25Q64JV_Command_SET_BURST_WITH_WRAP = 0x77,
    W25Q64JV_Command_MFTR_DEVICE_ID_DUAL_IO = 0x92,
    W25Q64JV_Command_MFTR_DEVICE_ID_QUAD_IO = 0x94,
};

#endif /* __BSP_W25Q64JV_REGISTER_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
