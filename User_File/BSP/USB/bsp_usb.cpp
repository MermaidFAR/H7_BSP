/**
 * @file bsp_usb.cpp
 * @brief BSP: USB CDC virtual COM transport
 * @version 0.1
 * @date 2026-06-03
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"
#include "sys_critical.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__attribute__((section(".dma_buffer"), aligned(32)))
Struct_USB_Manage_Object USB0_Manage_Object;

static bool USB_Tx_Submitting;

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
    USB0_Manage_Object.Rx_Timestamp = 0;
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
    if (Data == nullptr || Length == 0 || Length > USB_BUFFER_SIZE)
    {
        return USBD_FAIL;
    }

    uint32_t primask = SYS_Enter_Critical();
    const USBD_CDC_HandleTypeDef *hcdc = (const USBD_CDC_HandleTypeDef *) hUsbDeviceHS.pClassData;
    if (hcdc == nullptr || hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        SYS_Exit_Critical(primask);
        return USBD_FAIL;
    }
    if (USB_Tx_Submitting || hcdc->TxState != 0)
    {
        SYS_Exit_Critical(primask);
        return USBD_BUSY;
    }
    USB_Tx_Submitting = true;
    SYS_Exit_Critical(primask);

    memcpy(USB0_Manage_Object.Tx_Buffer, Data, Length);
    const uint8_t result = CDC_Transmit_HS(USB0_Manage_Object.Tx_Buffer, Length);

    primask = SYS_Enter_Critical();
    USB_Tx_Submitting = false;
    SYS_Exit_Critical(primask);
    return result;
}

/**
 * @brief USB CDC receive hook called from usbd_cdc_if.c
 *
 * @param Buffer actual packet buffer supplied by the CDC stack
 * @param Size received data size
 */
void USB_ReceiveCallback(uint8_t *Buffer, uint16_t Size)
{
    if (!init_finished || Buffer == nullptr || Size > USB_BUFFER_SIZE)
    {
        USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }

    // 重新枚举时 CDC_Init_HS 会换回 UserRxBufferHS，不能依赖旧 Active 指针。
    USB0_Manage_Object.Rx_Buffer_Ready = Buffer;
    if (Buffer == USB0_Manage_Object.Rx_Buffer_0)
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
