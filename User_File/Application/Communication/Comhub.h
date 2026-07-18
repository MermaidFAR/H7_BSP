#ifndef __COMHUB_H
#define __COMHUB_H

#include <cstdint>

#define COMHUB_SOF_0 0x5AU
#define COMHUB_SOF_1 0xA5U
#define COMHUB_PROTOCOL_VERSION 0x01U
#define COMHUB_MESSAGE_TYPE_AIM 0x01U
#define COMHUB_FRAME_LENGTH 28U

#define COMHUB_FLAG_ANGLE_VALID (1U << 0U)
#define COMHUB_FLAG_RATE_VALID (1U << 1U)
#define COMHUB_FLAG_PREDICTION_VALID (1U << 2U)
#define COMHUB_FLAG_CAMERA_CALIBRATED (1U << 3U)
#define COMHUB_FLAG_SATURATED (1U << 4U)
#define COMHUB_KNOWN_FLAGS                                                                                           \
    (COMHUB_FLAG_ANGLE_VALID | COMHUB_FLAG_RATE_VALID | COMHUB_FLAG_PREDICTION_VALID |                             \
     COMHUB_FLAG_CAMERA_CALIBRATED | COMHUB_FLAG_SATURATED)

#define COMHUB_ANGLE_RAD_PER_LSB 0.0001f
#define COMHUB_RATE_RAD_S_PER_LSB 0.001f

enum Enum_Comhub_Track_State : uint8_t
{
    COMHUB_TRACK_LOST = 0U,
    COMHUB_TRACK_DETECTING = 1U,
    COMHUB_TRACK_TRACKING = 2U,
    COMHUB_TRACK_PREDICT_ONLY = 3U,
};

struct Struct_Comhub_Message
{
    uint32_t Generation;
    uint16_t Frame;
    Enum_Comhub_Track_State Track_State;
    uint8_t Quality;
    uint8_t Flags;
    uint32_t Capture_Age_Us;
    uint32_t Prediction_Horizon_Us;
    float Yaw_Error_Rad;
    float Pitch_Error_Rad;
    float Yaw_Rate_FF_Rad_S;
    float Pitch_Rate_FF_Rad_S;
    uint64_t Rx_Timestamp_Us;
};

struct Struct_Comhub_Diagnostics
{
    uint32_t Valid_Frame_Count;
    uint32_t CRC_Error_Count;
    uint32_t Format_Error_Count;
    uint32_t Duplicate_Frame_Count;
    uint32_t Out_Of_Order_Frame_Count;
    uint32_t Sequence_Gap_Count;
    uint32_t Sequence_Resync_Count;
    uint32_t Dropped_Byte_Count;
    uint16_t Last_Frame;
    uint64_t Last_Rx_Timestamp_Us;
};

void Comhub_Init();
void Comhub_Callback(uint8_t *Buffer, uint16_t Length);
bool Comhub_GetLatest(Struct_Comhub_Message *Message);
void Comhub_GetDiagnostics(Struct_Comhub_Diagnostics *Diagnostics);
uint16_t Comhub_CRC16_CCITT_FALSE(const uint8_t *Data, uint16_t Length);

#endif
