/**
 * @file BMI088_Task.cpp
 * @brief BMI088 高优先级姿态解算任务。
 * @author zzm
 *
 * @details
 * 陀螺仪 SPI 接收回调在 FIFO 样本入队后置位线程标志。本任务被唤醒后逐帧调用
 * Calculate() 清空队列，避免约 2 kHz 的陀螺仪数据积压；完成本批解算后更新
 * 调试遥测数据。任务使用 CMSIS-RTOS v2 接口，不直接依赖 FreeRTOS 原生 API。
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
 * 任务阻塞等待 SPI 接收回调发出的样本就绪标志。每次唤醒后持续解算，直到陀螺仪
 * 单生产者/单消费者队列为空，再发布调试数据；由此将中断中的短时数据接收与任务中
 * 的VQF运算分离。
 */
extern "C" void BMI088_Task(void *argument) {
  // 姿态解算依赖高频陀螺仪数据，提升任务优先级以降低 FIFO 排队延迟。
  osThreadSetPriority(osThreadGetId(), osPriorityHigh2);

  for (;;) {
    // SPI 接收回调仅在样本入队后置位该标志，任务保持阻塞而非周期轮询。
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    do {
      // Calculate() 每次从陀螺仪队列取出一帧；循环排空可避免高频数据积压。
      BSP_BMI088.Calculate();
    } while (BSP_BMI088.BMI088_Gyro.Get_Queue_Depth() != 0U);

    // 发布最新姿态与诊断数据，供上位机调试和运行状态监控。
    Sys_Debug_IMU_Update();
  }
}
