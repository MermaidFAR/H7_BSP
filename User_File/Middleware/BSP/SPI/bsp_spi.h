/**
 * @file bsp_spi.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的SPI通信初始化与配置流程
 * @version 3.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-13 1.1 加入SPI配置操作
 * @date 2023-11-19 1.2 修改成cpp, 24赛季定稿
 * @date 2024-08-22 2.1 新增回调函数空指针判定
 * @date 2025-08-13 3.1 适配达妙MC02板, 准备着手开发陀螺仪
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

#ifndef DRV_SPI_H
#define DRV_SPI_H

/* Includes ------------------------------------------------------------------*/

#include "sys_timestamp.h"
#include "spi.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define SPI_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SPI通信接收回调函数数据类型
 *
 */
typedef void (*SPI_Callback)(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);

/**
 * @brief CAN通信处理结构体
 *
 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    SPI_Callback Callback_Function;

    // 当前是否存在尚未完成的事务
    volatile bool Transaction_Active;

    // 片选信号的GPIO与电平
    GPIO_TypeDef *Activate_GPIOx;
    uint16_t Activate_GPIO_Pin;
    GPIO_PinState Activate_Level;

    // 一次收发对应的数据长度
    uint8_t Tx_Buffer[SPI_BUFFER_SIZE];
    uint8_t Rx_Buffer[SPI_BUFFER_SIZE];
    uint16_t Tx_Buffer_Length;
    uint16_t Rx_Buffer_Length;

    // 接收时间戳
    uint64_t Rx_Timestamp;

    // HAL SPI error callback count
    volatile uint32_t Error_Count;
    volatile uint32_t Transaction_Busy_Count;
    volatile uint32_t Start_Failure_Count;
    volatile uint32_t Callback_Anomaly_Count;
    volatile uint8_t Last_Start_Failure_Status;
};

/**
 * @brief SPI timeout 发生前的硬件与 HAL 现场
 *
 * @note Capture_Count 在其余字段写完后递增，快照会一直保留到下一次 timeout。
 */
struct Struct_SPI_Timeout_Snapshot
{
    volatile uint32_t Capture_Count;
    volatile uint32_t Timestamp_Low32_Us;
    volatile uint32_t Elapsed_Us;
    volatile uint32_t SPI_CR1;
    volatile uint32_t SPI_CR2;
    volatile uint32_t SPI_CFG1;
    volatile uint32_t SPI_IER;
    volatile uint32_t SPI_SR;
    volatile uint32_t DMA1_LISR;
    volatile uint32_t RX_DMA_CR;
    volatile uint32_t RX_DMA_NDTR;
    volatile uint32_t TX_DMA_CR;
    volatile uint32_t TX_DMA_NDTR;
    volatile uint32_t HAL_Error_Code;
    volatile uint16_t HAL_Tx_Xfer_Count;
    volatile uint16_t HAL_Rx_Xfer_Count;
    volatile uint16_t Manager_Tx_Length;
    volatile uint16_t Manager_Rx_Length;
    volatile uint8_t HAL_State;
    volatile uint8_t HAL_Lock;
    volatile uint8_t RX_DMA_State;
    volatile uint8_t TX_DMA_State;
    volatile uint8_t Transaction_Active;
    volatile uint8_t NVIC_Pending_Bits;
    volatile uint8_t NVIC_Active_Bits;
    volatile uint8_t Reserved;
};

/* Exported variables ---------------------------------------------------------*/

extern volatile bool init_finished;

extern struct Struct_SPI_Manage_Object SPI1_Manage_Object;
extern struct Struct_SPI_Manage_Object SPI2_Manage_Object;
extern struct Struct_SPI_Manage_Object SPI3_Manage_Object;
extern struct Struct_SPI_Manage_Object SPI4_Manage_Object;
extern struct Struct_SPI_Manage_Object SPI5_Manage_Object;
extern struct Struct_SPI_Manage_Object SPI6_Manage_Object;
extern struct Struct_SPI_Timeout_Snapshot SPI2_Timeout_Snapshot;

/* Exported function declarations ---------------------------------------------*/

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function);

uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length);

uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length, uint16_t Rx_Length);

void SPI_Capture_Timeout_Snapshot(SPI_HandleTypeDef *hspi, uint32_t Elapsed_Us);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
