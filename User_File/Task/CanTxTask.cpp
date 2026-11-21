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
    for (;;)
    {
        BSP_CAN_Recover();
        BSP_CAN_SendAsync();
        BSP_CAN_SendPer();
        // 提交时立即唤醒；1 kHz 内核节拍提供 FIFO 满与总线故障的重试兜底。
        osThreadFlagsWait(BSP_CAN_TX_READY_FLAG, osFlagsWaitAny, 1U);
    }
}
