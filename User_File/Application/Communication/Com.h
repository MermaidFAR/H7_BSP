#ifndef COM_H
#define COM_H

#include <stdbool.h>

#include "cmsis_os2.h"
#include "bsp_uart.h"

typedef struct
{
    uint8_t Header;
    uint16_t Length;
    uint8_t* Data;
    uint16_t Checksum;
} Frame_t;

extern Frame_t RDK_Msg;

extern volatile float RDK_Position_Cm;
extern volatile float RDK_Speed_Cm_S;
extern volatile int8_t RDK_Target_Control;

void Communication_Callback(uint8_t* Buffer, uint16_t Length);
bool Com_RegisterFrame(Frame_t* frame);

#endif // COM_H
