#include "Init.h"
#include "SEGGER_SYSVIEW.h"
#include "bsp_bmi088.h"
#include "bsp_spi.h"
#include "callback.h"
#include "sys_timestamp.h"
#include "bsp_can.h"

// 全局初始化完成标志位
volatile bool init_finished = false;

extern "C" void System_Init(void)
{
    SEGGER_SYSVIEW_Conf();
    SYS_Timestamp.Init(&htim5); 

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);

    // 陀螺仪的SPI
    SPI_Init(&hspi2, SPI2_Callback);

    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);
    
    HAL_TIM_Base_Start_IT(&htim5);
    BSP_BMI088.Init();
    

    
    init_finished = true;

}