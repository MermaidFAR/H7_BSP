#include "Com.h"

#include <cstdint>
#include <string.h>

alignas(4) static uint8_t RDK_Buffer[4];

volatile float RDK_Position_Cm = 0.0f;

Frame_t RDK_Msg = {
    .Header = 0xA5,
    .Length = 0x04,
    .Data = RDK_Buffer,
    .Checksum = 0

};

void Communication_Callback(uint8_t* Buffer, uint16_t Length)
{
    if (Buffer == nullptr ||
        Length != RDK_Msg.Length + 2U ||
        Buffer[0] != RDK_Msg.Header ||
        Buffer[1] != static_cast<uint8_t>(RDK_Msg.Length))
    {
        return;
    }

    for (uint16_t index = 0U; index < RDK_Msg.Length; ++index)
    {
        RDK_Buffer[index] = Buffer[index + 2U];
    }

    float position_cm;
    memcpy(&position_cm, RDK_Buffer, sizeof(position_cm));
    RDK_Position_Cm = position_cm;
}
