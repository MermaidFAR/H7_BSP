/**
 * @file    bsp_uart.h
 * @brief   板级支持包：UART 通信初始化与配置流程（基于 STM32H7 + DMA 双缓冲）
 * @details 仿照 SCUT-Robotlab / 达妙 drv_uart 范式改写，适配 H7_BSP 工程：
 *          - 仅接管具备 RX DMA 的 7 路 UART：UART5、UART7、USART1、USART2、USART3、USART6、USART10
 *          - 接收采用 HAL_UARTEx_ReceiveToIdle_DMA + IDLE 中断，双缓冲交替，收不定长帧
 *          - 管理对象（含 DMA 收发缓冲区）放入 .dma_buffer 段（链接到 RAM_D1，DMA1/DMA2 可访问）
 * @note    UART4、UART8、UART9 因 STM32H7 DMA1+DMA2 共 16 条 stream 已被占满，
 *          无法分配 DMA，暂不在本驱动接管。如需使用请改用中断/阻塞方式单独实现。
 *
 * @author  zzm（仿 yssickjgd / USTC-RoboWalker drv_uart）
 * @version 1.0
 * @date    2026-06-01
 */

#ifndef BSP_UART_H
#define BSP_UART_H

/* Includes ------------------------------------------------------------------*/

#include "sys_timestamp.h"
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 接收缓冲区字节长度
#define UART_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 通信接收回调函数数据类型
 *
 * @param Buffer 接收完毕的缓冲区指针
 * @param Length 本帧接收到的字节长度
 */
typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief UART 通信处理结构体
 * @note  含 DMA 双缓冲区，整体放入 .dma_buffer 段以保证 DMA 可访问
 */
struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    UART_Callback Callback_Function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t Rx_Buffer_0[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[UART_BUFFER_SIZE];
    // 正在接收的缓冲区
    uint8_t *Rx_Buffer_Active;
    // 接收完毕的缓冲区
    uint8_t *Rx_Buffer_Ready;

    // 接收时间戳
    uint64_t Rx_Timestamp;
};

/* Exported variables --------------------------------------------------------*/

extern volatile bool init_finished;

// 仅接管具备 RX DMA 的 7 路 UART，命名与外设实例一致
extern struct Struct_UART_Manage_Object USART1_Manage_Object;
extern struct Struct_UART_Manage_Object USART2_Manage_Object;
extern struct Struct_UART_Manage_Object USART3_Manage_Object;
extern struct Struct_UART_Manage_Object UART5_Manage_Object;
extern struct Struct_UART_Manage_Object USART6_Manage_Object;
extern struct Struct_UART_Manage_Object UART7_Manage_Object;
extern struct Struct_UART_Manage_Object USART10_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback_Function);

void UART_Reinit(UART_HandleTypeDef *huart);

uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

#endif // !BSP_UART_H