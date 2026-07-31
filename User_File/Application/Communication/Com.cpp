#include "Com.h"

#include <cstdint>
#include <string.h>

alignas(4) static uint8_t RDK_Buffer[9];

volatile float RDK_Position_Cm = 0.0f;
volatile float RDK_Speed_Cm_S = 0.0f;
volatile int8_t RDK_Target_Control = 0;

// 帧格式: A5 09 + float32位置(cm) + float32速度(cm/s) + int8控制字节，数值为小端序。
Frame_t RDK_Msg = {
    .Header = 0xA5,
    .Length = 0x09,
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
    float speed_cm_s;
    memcpy(&position_cm, RDK_Buffer, sizeof(position_cm));
    memcpy(&speed_cm_s,
           RDK_Buffer + sizeof(position_cm),
           sizeof(speed_cm_s));
    RDK_Position_Cm = position_cm;
    RDK_Speed_Cm_S = speed_cm_s;
    RDK_Target_Control = static_cast<int8_t>(RDK_Buffer[sizeof(position_cm) +
                                                        sizeof(speed_cm_s)]);
}
