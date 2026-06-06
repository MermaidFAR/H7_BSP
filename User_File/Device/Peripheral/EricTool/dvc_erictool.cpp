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
 * @brief Vofa+ 初始化
 *
 * @param huart                        绑定的 UART 外设句柄
 * @param __Rx_Variable_Assignment_Num 接收指令字典数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表指针（字符串数组，每项 100 字节）
 * @param __Frame_Tail                 帧尾（justfloat 默认 0x7f800000）
 */
void Class_EricTool_UART::Init(const UART_HandleTypeDef *huart, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    // H7_BSP bsp_uart 接管 7 路：USART1/2/3, UART5, USART6, UART7, USART10
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &USART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &USART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &USART3_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &USART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == USART10)
    {
        UART_Manage_Object = &USART10_Manage_Object;
    }

    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = const_cast<char **>(__Rx_Variable_Assignment_List);
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
    Data_Process(Length);
}

/**
 * @brief TIM 1ms 定时中断：打包并发送 justfloat 帧
 *
 */
void Class_EricTool_UART::TIM_1ms_Write_PeriodElapsedCallback()
{
  if (UART_Manage_Object == nullptr) return;
  Output();
  UART_Transmit_Data(UART_Manage_Object->UART_Handler, Tx_Buffer, Data_Number * sizeof(float) + sizeof(uint32_t));
}

/**
 * @brief 处理接收到的文本指令（格式："variable=value#"）
 *
 * @param Length 帧长度
 */
void Class_EricTool_UART::Data_Process(const uint16_t &Length)
{
    int flag = _Judge_Variable_Name(Length);
    _Judge_Variable_Value(Length, flag);
}

/**
 * @brief 解析指令变量名，返回等号后第一个字符的下标
 *
 * @param Length 帧长度
 * @return uint8_t 等号后第一个字符的下标
 */
uint8_t Class_EricTool_UART::_Judge_Variable_Name(const uint16_t &Length)
{
    char tmp_variable_name[ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];
    int flag;

    for (flag = 0; UART_Manage_Object->Rx_Buffer_Ready[flag] != '=' && flag < Length && UART_Manage_Object->Rx_Buffer_Ready[flag] != 0; flag++)
    {
        tmp_variable_name[flag] = UART_Manage_Object->Rx_Buffer_Ready[flag];
    }
    tmp_variable_name[flag] = 0;

    for (int i = 0; i < Rx_Variable_Num; i++)
    {
        if (strcmp(tmp_variable_name, (char *) ((int) Rx_Variable_List + ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH * i)) == 0)
        {
            Variable_Index = i;
            return (flag + 1);
        }
    }
    Variable_Index = -1;
    return (flag + 1);
}

/**
 * @brief 解析指令数值（支持负数和小数）
 *
 * @param Length 帧长度
 * @param flag   等号后第一个字符的下标
 */
void Class_EricTool_UART::_Judge_Variable_Value(const uint16_t &Length, int flag)
{
    int tmp_dot_flag, tmp_sign_coefficient, i;

    tmp_dot_flag = 0;
    tmp_sign_coefficient = 1;
    Variable_Value = 0.0f;

    if (Variable_Index == -1)
    {
        return;
    }

    if (UART_Manage_Object->Rx_Buffer_Ready[flag] == '-')
    {
        tmp_sign_coefficient = -1;
        flag++;
    }

    for (i = flag; UART_Manage_Object->Rx_Buffer_Ready[i] != '#' && i < Length && UART_Manage_Object->Rx_Buffer_Ready[flag] != 0; i++)
    {
        if (UART_Manage_Object->Rx_Buffer_Ready[i] == '.')
        {
            tmp_dot_flag = i;
        }
        else
        {
            Variable_Value = Variable_Value * 10.0f + (float) (UART_Manage_Object->Rx_Buffer_Ready[i] - '0');
        }
    }

    if (tmp_dot_flag != 0)
    {
        Variable_Value /= powf(10.0f, (float) (i - tmp_dot_flag) - 1.0f);
    }

    Variable_Value *= (float) (tmp_sign_coefficient);
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
 * @param __Rx_Variable_Assignment_List 接收指令字典列表指针（字符串数组，每项 100 字节）
 * @param __Frame_Tail                 帧尾（justfloat 默认 0x7f800000）
 */
void Class_EricTool_USB::Init(const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    USB_Manage_Object = &USB0_Manage_Object;
    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = const_cast<char **>(__Rx_Variable_Assignment_List);
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
    (void)Rx_Data;
    Data_Process(Length);
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
 * @brief 处理接收到的文本指令（格式："variable=value#"）
 *
 * @param Length 帧长度
 */
void Class_EricTool_USB::Data_Process(const uint16_t &Length)
{
    int flag = _Judge_Variable_Name(Length);
    _Judge_Variable_Value(Length, flag);
}

/**
 * @brief 解析指令变量名，返回等号后第一个字符的下标
 *
 * @param Length 帧长度
 * @return uint8_t 等号后第一个字符的下标
 */
uint8_t Class_EricTool_USB::_Judge_Variable_Name(const uint16_t &Length)
{
    char tmp_variable_name[ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];
    int flag;

    for (flag = 0; USB_Manage_Object->Rx_Buffer_Ready[flag] != '=' && flag < Length && USB_Manage_Object->Rx_Buffer_Ready[flag] != 0; flag++)
    {
        tmp_variable_name[flag] = USB_Manage_Object->Rx_Buffer_Ready[flag];
    }
    tmp_variable_name[flag] = 0;

    for (int i = 0; i < Rx_Variable_Num; i++)
    {
        if (strcmp(tmp_variable_name, (char *) ((int) Rx_Variable_List + ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH * i)) == 0)
        {
            Variable_Index = i;
            return (flag + 1);
        }
    }
    Variable_Index = -1;
    return (flag + 1);
}

/**
 * @brief 解析指令数值（支持负数和小数）
 *
 * @param Length 帧长度
 * @param flag   等号后第一个字符的下标
 */
void Class_EricTool_USB::_Judge_Variable_Value(const uint16_t &Length, int flag)
{
    int tmp_dot_flag, tmp_sign_coefficient, i;

    tmp_dot_flag = 0;
    tmp_sign_coefficient = 1;
    Variable_Value = 0.0f;

    if (Variable_Index == -1)
    {
        return;
    }

    if (USB_Manage_Object->Rx_Buffer_Ready[flag] == '-')
    {
        tmp_sign_coefficient = -1;
        flag++;
    }

    for (i = flag; USB_Manage_Object->Rx_Buffer_Ready[i] != '#' && i < Length && USB_Manage_Object->Rx_Buffer_Ready[flag] != 0; i++)
    {
        if (USB_Manage_Object->Rx_Buffer_Ready[i] == '.')
        {
            tmp_dot_flag = i;
        }
        else
        {
            Variable_Value = Variable_Value * 10.0f + (float) (USB_Manage_Object->Rx_Buffer_Ready[i] - '0');
        }
    }

    if (tmp_dot_flag != 0)
    {
        Variable_Value /= powf(10.0f, (float) (i - tmp_dot_flag) - 1.0f);
    }

    Variable_Value *= (float) (tmp_sign_coefficient);
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
        memcpy(tmp_buffer + i * sizeof(uint32_t), Data[i], sizeof(uint32_t));
    }

    memcpy(tmp_buffer + Data_Number * sizeof(uint32_t), &Frame_Tail, sizeof(uint32_t));
}

void EricTool_Send_Telemetry(void) {
  EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();  // USB 遥测
  EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback(); // UART justfloat
}

    /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
