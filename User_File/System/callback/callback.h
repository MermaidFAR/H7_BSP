/**
 ******************************************************************************
 * @file callback.h
 * @brief 定时器回调函数声明
 * 
 * 本文件回调函数的声明和相关类型定义。
 * 
 * @author zzm
 * @date 
 * @version 1.0
 * 
 * @copyright Copyright (c) 2024
 * 
 * @note 此文件为定时器回调函数的头文件，需要在使用定时器中断时包含
 ******************************************************************************
 */

#ifndef __TIM_CALLBACK_H
#define __TIM_CALLBACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);

void OSPI2_Polling_Callback();
void OSPI2_Rx_Callback(uint8_t *Buffer);
void OSPI2_Tx_Callback(uint8_t *Buffer);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_CALLBACK_H */