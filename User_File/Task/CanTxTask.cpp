/**
 * @file    CanTxTask.cpp
 * @brief   CAN 发送任务骨架
 * @author  zzm
 * @version 1.0
 * @date    2026-05-16
 */

/* Includes ------------------------------------------------------------------*/

#include "user_task.h"

#include "bsp_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Can_Tx_Task(void *argument)
{
  uint32_t xLastWakeTime = osKernelGetTickCount();
  for (;;) {
    xLastWakeTime += 1;
    osDelayUntil(xLastWakeTime);
    BSP_CAN_SendAsync();
    BSP_CAN_SendPer();
  }
}
