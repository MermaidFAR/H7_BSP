/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    octospi.c
  * @brief   This file provides code for the configuration
  *          of the OCTOSPI instances.
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
#include "octospi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

OSPI_HandleTypeDef hospi2;
MDMA_HandleTypeDef hmdma_octospi2_fifo_th;

/* OCTOSPI2 init function */
void MX_OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */

  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */

  /* USER CODE END OCTOSPI2_Init 1 */
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 1;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi2.Init.DeviceSize = 23;
  hospi2.Init.ChipSelectHighTime = 1;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
  hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi2.Init.ClockPrescaler = 2;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi2.Init.ChipSelectBoundary = 0;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi2.Init.MaxTran = 0;
  hospi2.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */

  /* USER CODE END OCTOSPI2_Init 2 */

}

void HAL_OSPI_MspInit(OSPI_HandleTypeDef* ospiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(ospiHandle->Instance==OCTOSPI2)
  {
  /* USER CODE BEGIN OCTOSPI2_MspInit 0 */

  /* USER CODE END OCTOSPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
    PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* OCTOSPI2 clock enable */
    __HAL_RCC_OCTOSPIM_CLK_ENABLE();
    __HAL_RCC_OSPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**OCTOSPI2 GPIO Configuration
    PC3_C     ------> OCTOSPIM_P1_IO0
    PA3     ------> OCTOSPIM_P1_IO2
    PA6     ------> OCTOSPIM_P1_IO3
    PB0     ------> OCTOSPIM_P1_IO1
    PB2     ------> OCTOSPIM_P1_CLK
    PE11     ------> OCTOSPIM_P1_NCS
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_OCTOSPIM_P1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_OCTOSPIM_P1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_OCTOSPIM_P1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* OCTOSPI2 MDMA Init */
    /* OCTOSPI2_FIFO_TH Init */
    hmdma_octospi2_fifo_th.Instance = MDMA_Channel0;
    hmdma_octospi2_fifo_th.Init.Request = MDMA_REQUEST_OCTOSPI2_FIFO_TH;
    hmdma_octospi2_fifo_th.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    hmdma_octospi2_fifo_th.Init.Priority = MDMA_PRIORITY_LOW;
    hmdma_octospi2_fifo_th.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_octospi2_fifo_th.Init.SourceInc = MDMA_SRC_INC_BYTE;
    hmdma_octospi2_fifo_th.Init.DestinationInc = MDMA_DEST_INC_BYTE;
    hmdma_octospi2_fifo_th.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
    hmdma_octospi2_fifo_th.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
    hmdma_octospi2_fifo_th.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_octospi2_fifo_th.Init.BufferTransferLength = 1;
    hmdma_octospi2_fifo_th.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma_octospi2_fifo_th.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_octospi2_fifo_th.Init.SourceBlockAddressOffset = 0;
    hmdma_octospi2_fifo_th.Init.DestBlockAddressOffset = 0;
    if (HAL_MDMA_Init(&hmdma_octospi2_fifo_th) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_MDMA_ConfigPostRequestMask(&hmdma_octospi2_fifo_th, 0, 0) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(ospiHandle,hmdma,hmdma_octospi2_fifo_th);

    /* OCTOSPI2 interrupt Init */
    HAL_NVIC_SetPriority(OCTOSPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OCTOSPI2_IRQn);
  /* USER CODE BEGIN OCTOSPI2_MspInit 1 */

  /* USER CODE END OCTOSPI2_MspInit 1 */
  }
}

void HAL_OSPI_MspDeInit(OSPI_HandleTypeDef* ospiHandle)
{

  if(ospiHandle->Instance==OCTOSPI2)
  {
  /* USER CODE BEGIN OCTOSPI2_MspDeInit 0 */

  /* USER CODE END OCTOSPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_OCTOSPIM_CLK_DISABLE();
    __HAL_RCC_OSPI2_CLK_DISABLE();

    /**OCTOSPI2 GPIO Configuration
    PC3_C     ------> OCTOSPIM_P1_IO0
    PA3     ------> OCTOSPIM_P1_IO2
    PA6     ------> OCTOSPIM_P1_IO3
    PB0     ------> OCTOSPIM_P1_IO1
    PB2     ------> OCTOSPIM_P1_CLK
    PE11     ------> OCTOSPIM_P1_NCS
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3|GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);

    /* OCTOSPI2 MDMA DeInit */
    HAL_MDMA_DeInit(ospiHandle->hmdma);

    /* OCTOSPI2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(OCTOSPI2_IRQn);
  /* USER CODE BEGIN OCTOSPI2_MspDeInit 1 */

  /* USER CODE END OCTOSPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
