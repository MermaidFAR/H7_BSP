/* Includes ------------------------------------------------------------------*/

#include "Comhub.h"
#include "main.h"
#include "sys_timestamp.h"
#include <cstring>

/* Private macros ------------------------------------------------------------*/

#define COMHUB_SEQUENCE_RESET_TIMEOUT_US 500000ULL
#define COMHUB_SEQUENCE_OFFSET 8U
#define COMHUB_CAPTURE_AGE_OFFSET 10U
#define COMHUB_PREDICTION_HORIZON_OFFSET 14U
#define COMHUB_YAW_ERROR_OFFSET 18U
#define COMHUB_PITCH_ERROR_OFFSET 20U
#define COMHUB_YAW_RATE_OFFSET 22U
#define COMHUB_PITCH_RATE_OFFSET 24U
#define COMHUB_CRC_OFFSET 26U
#define COMHUB_CRC_INPUT_LENGTH COMHUB_CRC_OFFSET

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static Struct_Comhub_Message Message_Buffer[2] = {};
static volatile uint8_t Published_Index = 0U;
static volatile uint32_t Published_Generation = 0U;
static uint8_t Parser_Buffer[COMHUB_FRAME_LENGTH] = {};
static uint16_t Parser_Length = 0U;
static bool Has_Last_Frame = false;
static Struct_Comhub_Diagnostics Diagnostics = {};

/* Private function declarations ---------------------------------------------*/

static uint16_t Comhub_Read_U16_LE(const uint8_t *Data);
static int16_t Comhub_Read_I16_LE(const uint8_t *Data);
static uint32_t Comhub_Read_U32_LE(const uint8_t *Data);
static constexpr uint16_t Comhub_CRC16_CCITT_FALSE_Impl(const uint8_t *Data, uint16_t Length);
static bool Comhub_Validate_And_Publish(const uint8_t *Frame);
static void Comhub_Resync_After_Invalid_Frame();
static void Comhub_Feed_Byte(uint8_t Byte);

/* Function prototypes -------------------------------------------------------*/

static constexpr uint16_t Comhub_CRC16_CCITT_FALSE_Impl(const uint8_t *Data, uint16_t Length)
{
    uint16_t CRC_Value = 0xFFFFU;
    for (uint16_t Index = 0U; Index < Length; Index++)
    {
        CRC_Value ^= static_cast<uint16_t>(Data[Index]) << 8U;
        for (uint8_t Bit = 0U; Bit < 8U; Bit++)
        {
            CRC_Value = (CRC_Value & 0x8000U) != 0U
                            ? static_cast<uint16_t>((CRC_Value << 1U) ^ 0x1021U)
                            : static_cast<uint16_t>(CRC_Value << 1U);
        }
    }
    return CRC_Value;
}

static_assert(COMHUB_FRAME_LENGTH == COMHUB_CRC_OFFSET + 2U, "Comhub v1 frame layout changed");

uint16_t Comhub_CRC16_CCITT_FALSE(const uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr)
    {
        return 0U;
    }

    return Comhub_CRC16_CCITT_FALSE_Impl(Data, Length);
}

void Comhub_Init()
{
    const uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    std::memset(Message_Buffer, 0, sizeof(Message_Buffer));
    std::memset(Parser_Buffer, 0, sizeof(Parser_Buffer));
    std::memset(&Diagnostics, 0, sizeof(Diagnostics));
    Published_Index = 0U;
    Published_Generation = 0U;
    Parser_Length = 0U;
    Has_Last_Frame = false;
    __set_PRIMASK(Primask);
}

void Comhub_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr || Length == 0U)
    {
        return;
    }

    for (uint16_t Index = 0U; Index < Length; Index++)
    {
        Comhub_Feed_Byte(Buffer[Index]);
    }
}

bool Comhub_GetLatest(Struct_Comhub_Message *Message)
{
    if (Message == nullptr)
    {
        return false;
    }

    const uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t Generation = Published_Generation;
    if (Generation == 0U || Message->Generation == Generation)
    {
        __set_PRIMASK(Primask);
        return false;
    }

    *Message = Message_Buffer[Published_Index];
    Message->Generation = Generation;
    __set_PRIMASK(Primask);
    return true;
}

void Comhub_GetDiagnostics(Struct_Comhub_Diagnostics *Output)
{
    if (Output == nullptr)
    {
        return;
    }

    const uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    *Output = Diagnostics;
    __set_PRIMASK(Primask);
}

static uint16_t Comhub_Read_U16_LE(const uint8_t *Data)
{
    return static_cast<uint16_t>(Data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(Data[1]) << 8U);
}

static int16_t Comhub_Read_I16_LE(const uint8_t *Data)
{
    return static_cast<int16_t>(Comhub_Read_U16_LE(Data));
}

static uint32_t Comhub_Read_U32_LE(const uint8_t *Data)
{
    return static_cast<uint32_t>(Data[0]) |
           (static_cast<uint32_t>(Data[1]) << 8U) |
           (static_cast<uint32_t>(Data[2]) << 16U) |
           (static_cast<uint32_t>(Data[3]) << 24U);
}

static bool Comhub_Validate_And_Publish(const uint8_t *Frame)
{
    if (Frame[0] != COMHUB_SOF_0 || Frame[1] != COMHUB_SOF_1 ||
        Frame[2] != COMHUB_PROTOCOL_VERSION || Frame[3] != COMHUB_MESSAGE_TYPE_AIM ||
        Frame[4] != COMHUB_FRAME_LENGTH || Frame[5] > COMHUB_TRACK_PREDICT_ONLY ||
        Frame[6] > 100U ||
        (Frame[7] & static_cast<uint8_t>(~COMHUB_KNOWN_FLAGS)) != 0U)
    {
        Diagnostics.Format_Error_Count++;
        return false;
    }

    const uint16_t Expected_CRC = Comhub_Read_U16_LE(Frame + COMHUB_CRC_OFFSET);
    const uint16_t Actual_CRC = Comhub_CRC16_CCITT_FALSE(Frame, COMHUB_CRC_INPUT_LENGTH);
    if (Actual_CRC != Expected_CRC)
    {
        Diagnostics.CRC_Error_Count++;
        return false;
    }

    const uint16_t Frame_Sequence = Comhub_Read_U16_LE(Frame + COMHUB_SEQUENCE_OFFSET);
    const uint64_t Now_Us = SYS_Timestamp_Get_Microsecond();
    if (Has_Last_Frame)
    {
        const uint16_t Delta = static_cast<uint16_t>(Frame_Sequence - Diagnostics.Last_Frame);
        if (Delta == 0U)
        {
            Diagnostics.Duplicate_Frame_Count++;
            return true;
        }

        if ((Now_Us - Diagnostics.Last_Rx_Timestamp_Us) <= COMHUB_SEQUENCE_RESET_TIMEOUT_US)
        {
            if (Delta >= 0x8000U)
            {
                Diagnostics.Out_Of_Order_Frame_Count++;
                return true;
            }
            if (Delta > 1U)
            {
                Diagnostics.Sequence_Gap_Count += static_cast<uint32_t>(Delta - 1U);
            }
        }
        else
        {
            Diagnostics.Sequence_Resync_Count++;
        }
    }

    const uint8_t Write_Index = Published_Index ^ 1U;
    Struct_Comhub_Message *Message = &Message_Buffer[Write_Index];
    Message->Frame = Frame_Sequence;
    Message->Track_State = static_cast<Enum_Comhub_Track_State>(Frame[5]);
    Message->Quality = Frame[6];
    Message->Flags = Frame[7];
    Message->Capture_Age_Us = Comhub_Read_U32_LE(Frame + COMHUB_CAPTURE_AGE_OFFSET);
    Message->Prediction_Horizon_Us = Comhub_Read_U32_LE(Frame + COMHUB_PREDICTION_HORIZON_OFFSET);
    Message->Yaw_Error_Rad =
        static_cast<float>(Comhub_Read_I16_LE(Frame + COMHUB_YAW_ERROR_OFFSET)) * COMHUB_ANGLE_RAD_PER_LSB;
    Message->Pitch_Error_Rad =
        static_cast<float>(Comhub_Read_I16_LE(Frame + COMHUB_PITCH_ERROR_OFFSET)) * COMHUB_ANGLE_RAD_PER_LSB;
    Message->Yaw_Rate_FF_Rad_S =
        static_cast<float>(Comhub_Read_I16_LE(Frame + COMHUB_YAW_RATE_OFFSET)) * COMHUB_RATE_RAD_S_PER_LSB;
    Message->Pitch_Rate_FF_Rad_S =
        static_cast<float>(Comhub_Read_I16_LE(Frame + COMHUB_PITCH_RATE_OFFSET)) * COMHUB_RATE_RAD_S_PER_LSB;
    Message->Rx_Timestamp_Us = Now_Us;

    const uint32_t Next_Generation = Published_Generation + 1U;
    Message->Generation = Next_Generation;
    __DMB();
    Published_Index = Write_Index;
    Published_Generation = Next_Generation;

    Diagnostics.Valid_Frame_Count++;
    Diagnostics.Last_Frame = Frame_Sequence;
    Diagnostics.Last_Rx_Timestamp_Us = Now_Us;
    Has_Last_Frame = true;
    return true;
}

static void Comhub_Resync_After_Invalid_Frame()
{
    uint16_t Start = COMHUB_FRAME_LENGTH;
    for (uint16_t Index = 1U; Index + 1U < COMHUB_FRAME_LENGTH; Index++)
    {
        if (Parser_Buffer[Index] == COMHUB_SOF_0 && Parser_Buffer[Index + 1U] == COMHUB_SOF_1)
        {
            Start = Index;
            break;
        }
    }

    if (Start < COMHUB_FRAME_LENGTH)
    {
        Parser_Length = static_cast<uint16_t>(COMHUB_FRAME_LENGTH - Start);
        std::memmove(Parser_Buffer, Parser_Buffer + Start, Parser_Length);
    }
    else if (Parser_Buffer[COMHUB_FRAME_LENGTH - 1U] == COMHUB_SOF_0)
    {
        Parser_Buffer[0] = COMHUB_SOF_0;
        Parser_Length = 1U;
    }
    else
    {
        Parser_Length = 0U;
    }
}

static void Comhub_Feed_Byte(uint8_t Byte)
{
    if (Parser_Length == 0U)
    {
        if (Byte == COMHUB_SOF_0)
        {
            Parser_Buffer[0] = Byte;
            Parser_Length = 1U;
        }
        else
        {
            Diagnostics.Dropped_Byte_Count++;
        }
        return;
    }

    if (Parser_Length == 1U)
    {
        if (Byte == COMHUB_SOF_1)
        {
            Parser_Buffer[1] = Byte;
            Parser_Length = 2U;
        }
        else if (Byte != COMHUB_SOF_0)
        {
            Parser_Length = 0U;
            Diagnostics.Dropped_Byte_Count++;
        }
        return;
    }

    Parser_Buffer[Parser_Length++] = Byte;
    if (Parser_Length < COMHUB_FRAME_LENGTH)
    {
        return;
    }

    if (Comhub_Validate_And_Publish(Parser_Buffer))
    {
        Parser_Length = 0U;
    }
    else
    {
        Comhub_Resync_After_Invalid_Frame();
    }
}
