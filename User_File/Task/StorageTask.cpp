/**
 * @file StorageTask.cpp
 * @author zzm
 * @brief Low-priority external Flash owner task.
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_imu_bias_store.h"

extern "C"
{
#include "cmsis_os2.h"
}

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Storage_Task(void *argument)
{
    (void)argument;
    SYS_IMU_Bias_Store_Init();

    for (;;)
    {
        SYS_IMU_Bias_Store_Process();
        osDelay(100U);
    }
}
