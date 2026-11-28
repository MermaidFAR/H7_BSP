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

#include "spi.h"
#include "stm32h7xx_hal.h"
#include "sys_timestamp.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define SPI_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SPI 全双工收发完成回调。
 * @param Tx_Buffer 本次事务的发送缓冲区。
 * @param Rx_Buffer 全双工接收缓冲区，读取段从偏移 Tx_Length 起取 Rx_Length 字节。
 * @param Tx_Length 读取段前发送的字节数。
 * @param Rx_Length 读取段的字节数。
 * @note 轮询完成时在调用者上下文执行，DMA 完成时在中断上下文执行。
 * @note 两个缓冲区借用至回调返回，此后 SPI 事务释放。
 */
typedef void (*SPI_Callback)(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);

enum Enum_SPI_Transaction_State : uint8_t
{
    SPI_TRANSACTION_IDLE,
    SPI_TRANSACTION_ACTIVE,
    SPI_TRANSACTION_RECOVERING,
    SPI_TRANSACTION_RECOVERY_PENDING,
};

/**
 * @brief 单路 SPI 的收发缓冲区、片选状态与诊断计数。
 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    SPI_Callback Callback_Function;

    volatile Enum_SPI_Transaction_State Transaction_State;

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
    uint32_t Error_Count;
    uint32_t Transaction_Busy_Count;
    uint32_t Start_Failure_Count;
    uint32_t Recovery_Failure_Count;
    uint32_t Callback_Anomaly_Count;
    uint8_t Last_Start_Failure_Status;
};

/**
 * @brief SPI timeout 发生前的硬件与 HAL 现场
 *
 * @note Capture_Count 在其余字段写完后递增，快照会一直保留到下一次 timeout。
 */
struct Struct_SPI_Timeout_Snapshot
{
    uint32_t Capture_Count;
    uint32_t Timestamp_Low32_Us;
    uint32_t Elapsed_Us;
    uint32_t SPI_CR1;
    uint32_t SPI_CR2;
    uint32_t SPI_CFG1;
    uint32_t SPI_IER;
    uint32_t SPI_SR;
    uint32_t DMA1_LISR;
    uint32_t RX_DMA_CR;
    uint32_t RX_DMA_NDTR;
    uint32_t TX_DMA_CR;
    uint32_t TX_DMA_NDTR;
    uint32_t HAL_Error_Code;
    uint16_t HAL_Tx_Xfer_Count;
    uint16_t HAL_Rx_Xfer_Count;
    uint16_t Manager_Tx_Length;
    uint16_t Manager_Rx_Length;
    uint8_t HAL_State;
    uint8_t HAL_Lock;
    uint8_t RX_DMA_State;
    uint8_t TX_DMA_State;
    uint8_t Transaction_Active;
    uint8_t NVIC_Pending_Bits;
    uint8_t NVIC_Active_Bits;
    uint8_t Reserved;
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

// Blocking 用于启动配置，SPI6 始终使用轮询。
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length, bool Blocking = false);

uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState Activate_Level, const uint8_t *Tx_Data, uint16_t Tx_Length, uint16_t Rx_Length, bool Blocking = false);

void SPI_Capture_Timeout_Snapshot(SPI_HandleTypeDef *hspi, uint32_t Elapsed_Us);

/**
 * @brief 在任务上下文中止异常 SPI/DMA 事务并复位总线管理状态。
 * @param hspi 需要恢复的 SPI 句柄，由事务所有者在提交返回后调用。
 * @return true 表示 SPI 与关联 DMA 已停止，缓冲区可复用。false 时由任务稍后重试。
 */
bool SPI_Recover(SPI_HandleTypeDef *hspi);

/**
 * @brief 查询 HAL SPI 句柄是否存在未清除的错误标志。
 * @param hspi SPI 句柄。
 */
bool SPI_Has_Error(const SPI_HandleTypeDef *hspi);

/**
 * @brief 读取对应 SPI 总线累计的 HAL 错误回调次数。
 * @param hspi SPI 句柄。
 */
uint32_t SPI_Get_Error_Count(const SPI_HandleTypeDef *hspi);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
