/**
 * @file dvc_erictool.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief EricTool justfloat 调试工具（UART / USB CDC）
 * @version 0.2
 * @date 2025-09-22 0.1 新建（dm02_test）
 * @date 2026-06-01 0.2 适配 H7_BSP：UART 管理对象重命名，bzero→memset，重命名为 EricTool
 * @date 2026-06-03 0.3 恢复 USB CDC 版本
 *
 * @copyright USTC-RoboWalker (c) 2025-2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_erictool.h"
#include <math.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
Class_EricTool_USB EricTool_USB;
Class_EricTool_UART EricTool_UART;
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 解析一条 variable:value# 指令；失败输出 index=-1、value=0。
 * @note 只读取 Length 范围，变量名最多 99 字节；支持负数、小数，不支持指数形式。
 */
static void EricTool_ParseCommand(const uint8_t *data, uint16_t length,
                                  const char **names, uint8_t count,
                                  int32_t &variable_index, float &variable_value)
{
    variable_index = -1;
    variable_value = 0.0f;
    if (data == nullptr || names == nullptr)
    {
        return;
    }

    uint16_t name_length = 0;
    while (name_length < length && name_length < ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH &&
           data[name_length] != ':' && data[name_length] != 0)
    {
        ++name_length;
    }
    if (name_length == 0 || name_length >= length ||
        name_length >= ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH || data[name_length] != ':')
    {
        return;
    }

    int32_t index = -1;
    for (int i = 0; i < count; ++i)
    {
        if (names[i] != nullptr && strlen(names[i]) == name_length &&
            memcmp(data, names[i], name_length) == 0)
        {
            index = i;
            break;
        }
    }
    if (index == -1)
    {
        return;
    }

    uint16_t cursor = name_length + 1;
    float sign = 1.0f;
    if (cursor < length && data[cursor] == '-')
    {
        sign = -1.0f;
        ++cursor;
    }
    float value = 0.0f;
    float decimal_scale = 1.0f;
    bool has_dot = false;
    bool has_digit = false;
    while (cursor < length && data[cursor] != '#')
    {
        const uint8_t ch = data[cursor++];
        if (ch == '.' && !has_dot)
        {
            has_dot = true;
        }
        else if (ch >= '0' && ch <= '9')
        {
            has_digit = true;
            if (has_dot)
            {
                decimal_scale *= 0.1f;
                value += (ch - '0') * decimal_scale;
            }
            else
            {
                value = value * 10.0f + (ch - '0');
            }
            if (Basic_Math_Is_Invalid_Float(value))
            {
                return;
            }
        }
        else
        {
            return;
        }
    }
    if (cursor == length || !has_digit)
    {
        return;
    }
    variable_index = index;
    variable_value = sign * value;
}

/**
 * @brief Vofa+ 初始化
 *
 * @param huart                        绑定的 UART 外设句柄
 * @param __Rx_Variable_Assignment_Num 接收指令字典数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表指针（字符串指针数组）
 * @param __Frame_Tail                 帧尾（justfloat 默认 0x7f800000）
 */
void Class_EricTool_UART::Init(const UART_HandleTypeDef *huart, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    UART_Manage_Object = UART_Get_Manage_Object(huart);
    if (UART_Manage_Object == nullptr)
    {
        return;
    }

    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = __Rx_Variable_Assignment_List;
    Variable_Index = -1;
    Variable_Value = 0.0f;
    Frame_Tail = __Frame_Tail;
}

/**
 * @brief UART 接收完成回调（注册到 UART 回调链后由 bsp_uart 触发）
 *
 * @param Rx_Data 接收完毕的缓冲区指针
 * @param Length  本帧字节长度
 */
void Class_EricTool_UART::UART_RxCpltCallback(const uint8_t *Rx_Data, const uint16_t &Length)
{
    EricTool_ParseCommand(Rx_Data, Length, Rx_Variable_List, Rx_Variable_Num, Variable_Index, Variable_Value);
}

/**
 * @brief TIM 1ms 定时中断：打包并发送 justfloat 帧
 * @return HAL 发送状态；忙或失败时不排队，下次调用发送最新数据。
 */
uint8_t Class_EricTool_UART::TIM_1ms_Write_PeriodElapsedCallback()
{
  if (UART_Manage_Object == nullptr) return HAL_ERROR;
  Output();
  return UART_Transmit_Data(UART_Manage_Object->UART_Handler, Tx_Buffer, Data_Number * sizeof(float) + sizeof(uint32_t));
}

/**
 * @brief 将 Data[] 中的变量打包成 justfloat 帧写入 Tx_Buffer
 *
 */
void Class_EricTool_UART::Output()
{
    uint8_t *tmp_buffer = Tx_Buffer;

    memset(tmp_buffer, 0, UART_BUFFER_SIZE);

    for (int i = 0; i < Data_Number; i++)
    {
        memcpy(tmp_buffer + i * sizeof(uint32_t), Data[i], sizeof(uint32_t));
    }

    memcpy(tmp_buffer + Data_Number * sizeof(uint32_t), &Frame_Tail, sizeof(uint32_t));
}

/**
 * @brief EricTool USB CDC 初始化
 *
 * @param __Rx_Variable_Assignment_Num 接收指令字典数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表指针（字符串指针数组）
 * @param __Frame_Tail                 帧尾（justfloat 默认 0x7f800000）
 */
void Class_EricTool_USB::Init(const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    USB_Manage_Object = &USB0_Manage_Object;
    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = __Rx_Variable_Assignment_List;
    Variable_Index = -1;
    Variable_Value = 0.0f;
    Frame_Tail = __Frame_Tail;
}

/**
 * @brief USB CDC 接收完成回调（注册到 USB 回调链后由 bsp_usb 触发）
 *
 * @param Rx_Data 接收完毕的缓冲区指针
 * @param Length  本帧字节长度
 */
void Class_EricTool_USB::USB_RxCallback(const uint8_t *Rx_Data, const uint16_t &Length)
{
    EricTool_ParseCommand(Rx_Data, Length, Rx_Variable_List, Rx_Variable_Num, Variable_Index, Variable_Value);
}

/**
 * @brief TIM 1ms 定时中断：打包并通过 USB CDC 发送 justfloat 帧
 */
void Class_EricTool_USB::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();
    USB_Transmit_Data(Tx_Buffer, Data_Number * sizeof(float) + sizeof(uint32_t));
}

/**
 * @brief 将 Data[] 中的变量打包成 justfloat 帧写入 Tx_Buffer
 */
void Class_EricTool_USB::Output()
{
    uint8_t *tmp_buffer = Tx_Buffer;

    memset(tmp_buffer, 0, USB_BUFFER_SIZE);

    for (int i = 0; i < Data_Number; i++)
    {
        const float value = *Data[i];
        memcpy(tmp_buffer + i * sizeof(value), &value, sizeof(value));
    }

    memcpy(tmp_buffer + Data_Number * sizeof(uint32_t), &Frame_Tail, sizeof(uint32_t));
}

void EricTool_Send_Telemetry(void) {
  EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();  // USB 遥测
  EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback(); // UART justfloat
}

    /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
