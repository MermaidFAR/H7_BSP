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
static unsigned int init_errors = SYSTEM_INIT_ERROR_NONE;

extern "C" unsigned int System_Get_Init_Errors(void)
{
    return init_errors;
}

extern "C" void System_Init(void)
{
    init_finished = false;
    init_errors = SYSTEM_INIT_ERROR_NONE;
    SEGGER_SYSVIEW_Conf();
    // SEGGER_SYSVIEW_Stop();
    SYS_Timestamp.Init(&htim5);

    UART_Init(&huart1, nullptr);
    UART_Init(&huart2, nullptr);
    UART_Init(&huart3, nullptr);
    UART_Init(&huart5, nullptr);
    UART_Init(&huart6, nullptr);
    UART_Init(&huart7, nullptr);
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
    if (!BSP_BMI088.Init())
    {
        init_errors |= SYSTEM_INIT_ERROR_BMI088;
    }
    BSP_WS2812.Init();
    BSP_Buzzer.Init();
    BSP_Key.Init();
    if (!BSP_W25Q64JV.Init())
    {
        init_errors |= SYSTEM_INIT_ERROR_FLASH;
    }
    if (!ADC_Init(&hadc1, 1))
    {
        init_errors |= SYSTEM_INIT_ERROR_ADC1;
    }
    BSP_Power.Init(true, true, true);
    EricTool_USB.Init();
    if (BSP_BMI088.Is_Initialized())
    {
        BSP_BMI088.BMI088_Gyro.Start_FIFO_Acquisition();
    }
    
    init_finished = true;
}
