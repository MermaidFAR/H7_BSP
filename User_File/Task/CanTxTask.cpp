/**
 * @file    CanTxTask.cpp
 * @brief   CAN 发送任务骨架
 * @author  zzm
 * @version 1.0
 * @date    2026-05-16
 */

/* Includes ------------------------------------------------------------------*/

#include "user_task.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
}

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Can_Tx_Task(void *argument)
{

  for (;;) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime,
                    pdMS_TO_TICKS(1)); // 每1ms发送一次CAN消息,根据需要调整频率
    BSP_CAN_SendPer();
  }
}