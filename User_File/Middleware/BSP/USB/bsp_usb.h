/**
 * @file bsp_usb.h
 * @brief BSP: USB CDC virtual COM transport
 * @version 0.1
 * @date 2026-06-03
 */

#ifndef BSP_USB_H
#define BSP_USB_H

/* Includes ------------------------------------------------------------------*/

#include "sys_timestamp.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

typedef void (*USB_Callback)(uint8_t *Buffer, uint16_t Length);

struct Struct_USB_Manage_Object
{
    USB_Callback Callback_Function;

    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    uint64_t Rx_Timestamp;
};

/* Exported variables --------------------------------------------------------*/

extern struct Struct_USB_Manage_Object USB0_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void USB_Init(USB_Callback Callback_Function);

uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length);

void USB_ReceiveCallback(uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USB_H */

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
