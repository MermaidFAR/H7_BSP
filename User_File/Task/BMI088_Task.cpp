/**
 * @file BMI088_Task.cpp
 * @brief BMI088 高优先级姿态解算任务。
 * @author zzm
 *
 * @details
 * 陀螺仪 SPI 接收回调在 FIFO 状态机需要续传或样本入队后置位线程标志。本任务
 * 被唤醒后在任务上下文继续传输并逐帧调用 Calculate() 清空队列，避免约 2 kHz
 * 的陀螺仪数据积压；完成本批解算后更新调试遥测数据。任务使用 CMSIS-RTOS v2
 * 接口，不直接依赖 FreeRTOS 原生 API。
 */

/* Includes ------------------------------------------------------------------*/
#include "user_task.h"
#include "bsp_bmi088.h"
#include "sys_debug.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief 执行 BMI088 FIFO 样本的高优先级姿态解算任务。
 * @param argument CMSIS-RTOS 任务入口参数，当前未使用。
 * @details
 * 任务阻塞等待 SPI 接收回调发出的续传或样本就绪标志。每次唤醒后先继续传输，
 * 再持续解算直到陀螺仪单生产者/单消费者队列为空；由此将中断中的短时数据接收
 * 与任务中的VQF运算分离。
 */
extern "C" void BMI088_Task(void *argument) {
  // 姿态解算依赖高频陀螺仪数据，提升任务优先级以降低 FIFO 排队延迟。
  osThreadSetPriority(osThreadGetId(), osPriorityHigh2);

  for (;;) {
    const uint32_t flags =
        osThreadFlagsWait(0x0003, osFlagsWaitAny, osWaitForever);

    if ((flags & 0x0002) != 0U) {
      BSP_BMI088.BMI088_Service_Transfer(true);
    }

    if (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U) {
      do {
        BSP_BMI088.Calculate();
      } while (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U);

      Sys_Debug_IMU_Update();
    }
  }
}
