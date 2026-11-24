#ifndef COM_H
#define COM_H

#include <stdbool.h>

#include "bsp_uart.h"
#include "cmsis_os2.h"

typedef struct
{
    uint8_t Header;
    uint16_t Length;
    uint8_t *Data;
    uint16_t Checksum;
} Frame_t;

void Communication_Callback(uint8_t *Buffer, uint16_t Length);

#endif // COM_H
