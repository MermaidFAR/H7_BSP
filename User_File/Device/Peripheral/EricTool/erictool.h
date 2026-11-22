/**
 * @file erictool.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief EricTool justfloat 调试工具（UART / USB CDC）
 * @version 0.2
 * @date 2025-09-22 0.1 新建（dm02_test）
 * @date 2026-06-01 0.2 适配 H7_BSP：UART 管理对象命名，重命名为 EricTool
 * @date 2026-06-03 0.3 恢复 USB CDC 版本
 *
 * @copyright USTC-RoboWalker (c) 2025-2026
 *
 */

#ifndef ERICTOOL_H
#define ERICTOOL_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "stm32_hal_legacy.h"
#include <stdarg.h>

/* Exported macros -----------------------------------------------------------*/

#define ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH (100)

    /* Exported types ------------------------------------------------------------*/

    /**
     * @brief Reusable, EricTool justfloat 串口调试工具（UART 版本）
     * @note  上行：justfloat 帧（N×float + 4B 帧尾 0x7f800000）
     *        下行：文本指令 "variable:value#"，成功后可读取字典索引和值；非法帧返回索引 -1、值 0
     */
    class Class_EricTool_UART {
public:
    void Init(const UART_HandleTypeDef *huart, uint8_t rx_variable_assignment_num = 0, const char **rx_variable_assignment_list = nullptr, uint32_t frame_tail = 0x7f800000);

    inline int32_t Get_Variable_Index() const;

    inline float Get_Variable_Value() const;

    void Set_Data(const volatile float *const *Channels, uint8_t Count);
    inline void Set_Data(int Number, ...);

    void UART_RxCpltCallback(const uint8_t *Rx_Data, uint16_t Length);

    uint8_t TIM_1ms_Write_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的 UART 管理对象
  Struct_UART_Manage_Object *UART_Manage_Object = nullptr;
  // 接收指令字典数量
  uint8_t Rx_Variable_Num = 0;
  // 接收指令字典列表指针
  const char **Rx_Variable_List = nullptr;
  // 数据包尾标（justfloat 默认 0x7f800000）
  uint32_t Frame_Tail;

  // 常量

  // 内部变量

  // 发送缓冲区
  uint8_t Tx_Buffer[UART_BUFFER_SIZE];

  // 需要绘图的各个变量数据地址（最多 24 个通道）
  const volatile float *Data[24];
  // 当前发送的数据数量
  uint8_t Data_Number = 0;
  // 当前接收的指令在指令字典中的编号（-1 表示未匹配）
  int32_t Variable_Index = -1;
  // 当前接收的指令值
  float Variable_Value = 0.0f;

  // 读变量

  // 写变量

  // 读写变量

  // 内部函数

  void Output();
};

/**
 * @brief Reusable, EricTool justfloat 调试工具（USB CDC 版本）
 * @note  上行：justfloat 帧（N×float + 4B 帧尾 0x7f800000）
 *        下行：文本指令 "variable:value#"，成功后可读取字典索引和值；非法帧返回索引 -1、值 0
 */
class Class_EricTool_USB
{
public:
    void Init(uint8_t rx_variable_assignment_num = 0, const char **rx_variable_assignment_list = nullptr, uint32_t frame_tail = 0x7f800000);

    inline int32_t Get_Variable_Index() const;

    inline float Get_Variable_Value() const;

    void Set_Data(const volatile float *const *Channels, uint8_t Count);
    inline void Set_Data(int Number, ...);

    void USB_RxCallback(const uint8_t *Rx_Data, uint16_t Length);

    void TIM_1ms_Write_PeriodElapsedCallback();

protected:
    // 绑定的 USB 管理对象
    Struct_USB_Manage_Object *USB_Manage_Object;
    // 接收指令字典数量
    uint8_t Rx_Variable_Num = 0;
    // 接收指令字典列表指针
    const char **Rx_Variable_List = nullptr;
    // 数据包尾标（justfloat 默认 0x7f800000）
    uint32_t Frame_Tail;

    // 发送缓冲区
    uint8_t Tx_Buffer[USB_BUFFER_SIZE];

    // 需要绘图的各个变量数据地址（最多 24 个通道）
    const volatile float *Data[24];
    // 当前发送的数据数量
    uint8_t Data_Number = 0;
    // 当前接收的指令在指令字典中的编号（-1 表示未匹配）
    int32_t Variable_Index = -1;
    // 当前接收的指令值
    float Variable_Value = 0.0f;

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前接收的指令在指令字典中的编号
 *
 * @return int32_t 指令编号，-1 表示未匹配
 */
inline int32_t Class_EricTool_UART::Get_Variable_Index() const
{
    return (Variable_Index);
}

/**
 * @brief 获取当前接收的指令值
 *
 * @return float 指令数值
 */
inline float Class_EricTool_UART::Get_Variable_Value() const
{
    return (Variable_Value);
}

/**
 * @brief 设置需要发送的变量数据（可变参数，传指针）
 *
 * @param Number 数据数量
 * @param ...    各变量的指针（const volatile float *）
 */
inline void Class_EricTool_UART::Set_Data(int Number, ...)
{
    if (Number < 0 || (unsigned int)Number > sizeof(Data) / sizeof(Data[0]))
    {
        return;
    }
    va_list data_ptr;
    va_start(data_ptr, Number);
    for (int i = 0; i < Number; i++)
    {
        Data[i] = va_arg(data_ptr, const volatile float *);
    }
    va_end(data_ptr);
    Data_Number = (uint8_t)Number;
}

/**
 * @brief 获取当前接收的指令在指令字典中的编号
 *
 * @return int32_t 指令编号，-1 表示未匹配
 */
inline int32_t Class_EricTool_USB::Get_Variable_Index() const
{
    return (Variable_Index);
}

/**
 * @brief 获取当前接收的指令值
 *
 * @return float 指令数值
 */
inline float Class_EricTool_USB::Get_Variable_Value() const
{
    return (Variable_Value);
}

// 变参必须传入 const volatile float *，也可使用数组重载。
inline void Class_EricTool_USB::Set_Data(int Number, ...)
{
    if (Number < 0 || (unsigned int)Number > sizeof(Data) / sizeof(Data[0]))
    {
        return;
    }
    va_list data_ptr;
    va_start(data_ptr, Number);
    for (int i = 0; i < Number; i++)
    {
        Data[i] = va_arg(data_ptr, const volatile float *);
    }
    va_end(data_ptr);
    Data_Number = (uint8_t)Number;
}

extern Class_EricTool_USB EricTool_USB;
extern Class_EricTool_UART EricTool_UART;
void EricTool_Send_Telemetry(void);
#endif // !ERICTOOL_H

    /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
