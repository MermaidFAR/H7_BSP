

#ifndef __INIT_H
#define __INIT_H

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART7 下行指令字典下标
 * @note  顺序必须与 Init.cpp 中 EricTool_UART_Rx_Variable_List 的初始化列表一致，
 *        项数由 ERICTOOL_UART_VARIABLE_NUM 自动给出，增删参数只需改这两处。
 */
typedef enum
{
    ERICTOOL_UART_VARIABLE_YAW = 0,     // 转向指令，归一化 [-1, 1]，正 = 右转
    ERICTOOL_UART_VARIABLE_MOVE,        // 前进/后退指令，归一化 [-1, 1]，正 = 前进
    ERICTOOL_UART_VARIABLE_MAX_TURN,    // 转向指令上限 [rad/s]
    ERICTOOL_UART_VARIABLE_YAW_KP,      // 偏航环 Kp
    ERICTOOL_UART_VARIABLE_YAW_KI,      // 偏航环 Ki
    ERICTOOL_UART_VARIABLE_NUM,         // 字典项数，必须放在最后
} Enum_EricTool_UART_Variable;



#ifdef __cplusplus
extern "C" {
#endif

void System_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_H */
