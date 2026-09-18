#include "Init.h"

#include "Com.h"
#include "SEGGER_SYSVIEW.h"
#include "bsp_adc.h"
#include "bsp_bmi088.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_key.h"
#include "bsp_ospi.h"
#include "bsp_power.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "bsp_w25q64jv.h"
#include "bsp_ws2812.h"
#include "callback.h"
#include "dvc_erictool.h"
#include "sys_debug.h"
#include "sys_imu.h"
#include "sys_timestamp.h"
#include "usart.h"

// 全局初始化完成标志位
volatile bool init_finished = false;

namespace
{
/**
 * @brief UART7 转向指令字典（下行文本指令格式："变量名:数值#"，例如 "yaw:0.5#"）
 * @note  每项固定 ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH 字节，
 *        下标顺序必须与 Init.h 的 Enum_EricTool_UART_Variable 保持一致。
 */
char EricTool_UART_Rx_Variable_List[ERICTOOL_UART_VARIABLE_NUM][ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] =
{
    "yaw",      // ERICTOOL_UART_VARIABLE_YAW
    "move",     // ERICTOOL_UART_VARIABLE_MOVE
    "maxturn",  // ERICTOOL_UART_VARIABLE_MAX_TURN
    "kpyaw",    // ERICTOOL_UART_VARIABLE_YAW_KP
    "kiyaw",    // ERICTOOL_UART_VARIABLE_YAW_KI
};

/**
 * @brief UART7 接收完成回调（DMA 空闲中断上下文）
 * @note  本函数只做文本解析，结果暂存在 EricTool_UART 内部；
 *        真正的动作分发给 Control_Task 在任务上下文完成，中断里不做控制。
 */
void EricTool_UART_RxCallback(uint8_t *Buffer, uint16_t Length)
{
    EricTool_UART.UART_RxCpltCallback(Buffer, Length);
}
} // namespace

extern "C" void System_Init(void)
{
    SEGGER_SYSVIEW_Conf();
    // SEGGER_SYSVIEW_Stop();
    SYS_Timestamp.Init(&htim5);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  

    UART_Init(&huart1, nullptr);
    UART_Init(&huart2, nullptr);
    UART_Init(&huart3, nullptr);
    UART_Init(&huart4, nullptr);
    UART_Init(&huart5, nullptr);
    UART_Init(&huart6, nullptr);
    UART_Init(&huart7, EricTool_UART_RxCallback);
    UART_Init(&huart8, nullptr);
    UART_Init(&huart9, nullptr);
    UART_Init(&huart10, nullptr);

    // 陀螺仪的SPI
    SPI_Init(&hspi2, SPI2_Callback);

    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);

    // Flash 的 OSPI
    OSPI_Init(&hospi2, OSPI2_Polling_Callback, OSPI2_Rx_Callback, OSPI2_Tx_Callback);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    System_IMU_Configure();
    BSP_BMI088.Init();
    BSP_WS2812.Init();
    BSP_Buzzer.Init();
    BSP_Key.Init();
    BSP_W25Q64JV.Init();
    ADC_Init(&hadc1, 1);
    BSP_Power.Init(true, true, true);
    EricTool_USB.Init();
    // UART7 转向指令接收：蓝牙 DX-BT04-E，115200 8N1
    EricTool_UART.Init(&huart7, ERICTOOL_UART_VARIABLE_NUM, (const char **) EricTool_UART_Rx_Variable_List);
    BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();
    
    init_finished = true;
}
