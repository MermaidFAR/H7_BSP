/**
 * @file bsp_usb.cpp
 * @brief BSP: USB CDC virtual COM transport
 * @version 0.1
 * @date 2026-06-03
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_USB_Manage_Object USB0_Manage_Object = {nullptr};

extern volatile bool init_finished;
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Initialize USB CDC BSP callback routing
 *
 * @param Callback_Function receive callback, nullable
 */
void USB_Init(USB_Callback Callback_Function)
{
    USB0_Manage_Object.Callback_Function = Callback_Function;
    USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
    USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_1;

    if (hUsbDeviceHS.pClassData != nullptr)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
    }
    else
    {
        USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
    }
}

/**
 * @brief Transmit one USB CDC frame
 *
 * @param Data transmit buffer
 * @param Length transmit length
 * @return USB device result
 */
uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length)
{
    if (hUsbDeviceHS.pClassData == nullptr)
    {
        return USBD_FAIL;
    }

    return CDC_Transmit_HS(Data, Length);
}

/**
 * @brief USB CDC receive hook called from usbd_cdc_if.c
 *
 * @param Size received data size
 */
void USB_ReceiveCallback(uint16_t Size)
{
    if (!init_finished || USB0_Manage_Object.Rx_Buffer_Active == nullptr)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }

    USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_Active;
    if (USB0_Manage_Object.Rx_Buffer_Active == USB0_Manage_Object.Rx_Buffer_0)
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_1;
    }
    else
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
    }

    USB0_Manage_Object.Rx_Timestamp = SYS_Timestamp.Get_Current_Timestamp();

    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);

    if (USB0_Manage_Object.Callback_Function != nullptr)
    {
        USB0_Manage_Object.Callback_Function(USB0_Manage_Object.Rx_Buffer_Ready, Size);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
