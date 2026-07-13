/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t StorageTaskHandle;
const osThreadAttr_t StorageTask_attributes = {
  .name = "StorageTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE END Variables */
/* Definitions for TransportTask */
osThreadId_t TransportTaskHandle;
const osThreadAttr_t TransportTask_attributes = {
  .name = "TransportTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InsTask */
osThreadId_t InsTaskHandle;
const osThreadAttr_t InsTask_attributes = {
  .name = "InsTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for CanTxTask */
osThreadId_t CanTxTaskHandle;
const osThreadAttr_t CanTxTask_attributes = {
  .name = "CanTxTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TIM_1ms_Task */
osThreadId_t TIM_1ms_TaskHandle;
const osThreadAttr_t TIM_1ms_Task_attributes = {
  .name = "TIM_1ms_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BMI088Task */
osThreadId_t BMI088TaskHandle;
const osThreadAttr_t BMI088Task_attributes = {
  .name = "BMI088Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Storage_Task(void *argument);

/* USER CODE END FunctionPrototypes */

void Transport_Task(void *argument);
void Ins_Task(void *argument);
void Can_Tx_Task(void *argument);
void TIM1msTask(void *argument);
void BMI088_Task(void *argument);
void Gimbal_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  BSP_CAN_ConfigInit();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TransportTask */
  TransportTaskHandle = osThreadNew(Transport_Task, NULL, &TransportTask_attributes);

  /* creation of InsTask */
  InsTaskHandle = osThreadNew(Ins_Task, NULL, &InsTask_attributes);

  /* creation of CanTxTask */
  CanTxTaskHandle = osThreadNew(Can_Tx_Task, NULL, &CanTxTask_attributes);

  /* creation of TIM_1ms_Task */
  TIM_1ms_TaskHandle = osThreadNew(TIM1msTask, NULL, &TIM_1ms_Task_attributes);

  /* creation of BMI088Task */
  BMI088TaskHandle = osThreadNew(BMI088_Task, NULL, &BMI088Task_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(Gimbal_Task, NULL, &GimbalTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  StorageTaskHandle = osThreadNew(Storage_Task, NULL, &StorageTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Transport_Task */
/**
  * @brief  Function implementing the TransportTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Transport_Task */
__weak void Transport_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Transport_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Transport_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the InsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void *argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_Can_Tx_Task */
/**
* @brief Function implementing the CanTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_Tx_Task */
__weak void Can_Tx_Task(void *argument)
{
  /* USER CODE BEGIN Can_Tx_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Can_Tx_Task */
}

/* USER CODE BEGIN Header_TIM1msTask */
/**
* @brief Function implementing the TIM_1ms_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TIM1msTask */
__weak void TIM1msTask(void *argument)
{
  /* USER CODE BEGIN TIM1msTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TIM1msTask */
}

/* USER CODE BEGIN Header_BMI088_Task */
/**
* @brief Function implementing the BMI088Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMI088_Task */
__weak void BMI088_Task(void *argument)
{
  /* USER CODE BEGIN BMI088_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BMI088_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
volatile char *dbg_overflow_task_name = NULL;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  dbg_overflow_task_name = (volatile char *)pcTaskName;
  __asm volatile("bkpt #0");
  for (;;) {}
}
/* USER CODE END Application */

