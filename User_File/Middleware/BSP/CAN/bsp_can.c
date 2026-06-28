/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   板级支持包：CAN总线驱动实现 (基于 STM32H7 FDCAN)
 * @details 包含CAN过滤器的配置、中断接收回调的分发管理以及线程安全的发送函数。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.0
 ******************************************************************************
 */
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "stm32h723xx.h"
#include "sys_timestamp.h"
#include <stdbool.h>
#include <string.h>

/* 外部句柄引用 */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief 全局 CAN 回调函数注册表
 */
static CAN_CallbackEntry_t g_CanCallbacks[MAX_CAN_CALLBACKS];
/* 发送互斥锁定义 */
static osMutexId_t Can1_TxMutex;
static osMutexId_t Can2_TxMutex;
static osMutexId_t Can3_TxMutex;

/* 发送互斥锁属性定义 */
static const osMutexAttr_t Can1_TxMutex_Attr = {
    .name = "Can1_TxMutex",
};

static const osMutexAttr_t Can2_TxMutex_Attr = {
    .name = "Can2_TxMutex",
};

static const osMutexAttr_t Can3_TxMutex_Attr = {
    .name = "Can3_TxMutex",
};

// 预设的 CAN 消息缓冲区，用于 CAN_Tx_Perform 和 BSP_CAN_SendPer 函数
static Struct_CAN_Tx_Msg Tx_Msg_Buffer[3];

static osMessageQueueId_t Can_Tx_Queue = NULL;// CAN 发送消息队列

static void BSP_CAN_Init_Msg(void) {
  //
    Tx_Msg_Buffer[0].hfdcan = &hfdcan1;
    Tx_Msg_Buffer[0].id = 0x000;
    Tx_Msg_Buffer[0].len = 0;
    memset(Tx_Msg_Buffer[0].data, 0, sizeof(Tx_Msg_Buffer[0].data));

    Tx_Msg_Buffer[1].hfdcan = &hfdcan2;
    Tx_Msg_Buffer[1].id = 0x000;
    Tx_Msg_Buffer[1].len = 0;
    memset(Tx_Msg_Buffer[1].data, 0, sizeof(Tx_Msg_Buffer[1].data));

    Tx_Msg_Buffer[2].hfdcan = &hfdcan3;
    Tx_Msg_Buffer[2].id = 0x000;
    Tx_Msg_Buffer[2].len = 0;
    memset(Tx_Msg_Buffer[2].data, 0, sizeof(Tx_Msg_Buffer[2].data));
}

/**
 * @brief  初始化发送互斥锁
 * @note   防止多任务同时写入同一个 CAN TX FIFO 导致竞争
 */
static void BSP_CAN_Init_Locks(void)
{
    if (Can1_TxMutex == NULL)
    {
        Can1_TxMutex = osMutexNew(&Can1_TxMutex_Attr);
    }

    if (Can2_TxMutex == NULL)
    {
        Can2_TxMutex = osMutexNew(&Can2_TxMutex_Attr);
    }

    if (Can3_TxMutex == NULL)
    {
        Can3_TxMutex = osMutexNew(&Can3_TxMutex_Attr);
    }
}
/**
 * @brief  配置并启动所有 FDCAN 实例
 * @note   由于作者不同该can库函数需要在RTOS启动后调用，以确保互斥锁和消息队列的正确创建。
 */
void BSP_CAN_ConfigInit(void)
{
    FDCAN_FilterTypeDef FDCAN_FilterConfig;

    /* --------------------------------------------------------- */
    /* 公共过滤器配置：标准帧，掩码模式，允许所有 ID 通过             */
    /* --------------------------------------------------------- */
    FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN_FilterConfig.FilterIndex = 0;
    FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 指定导向 FIFO0
    FDCAN_FilterConfig.FilterID1 = 0x000; // ID 匹配值
    FDCAN_FilterConfig.FilterID2 = 0x000; // 掩码值 (0表示不关心，即接收所有)

    /* ========================= FDCAN 1 ========================= */
    // 1. 配置过滤器
    if ((HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_FilterConfig)) != HAL_OK)
    {
        Error_Handler();
    }
    // 2. 配置全局过滤器：拒绝不匹配的
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    // 3. 开启 FIFO0 新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    // 4. 启动 FDCAN1
    HAL_FDCAN_Start(&hfdcan1);


    /* ========================= FDCAN 2 ========================= */
    // 复用 FDCAN_FilterConfig 结构体，参数一致
    if ((HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_FilterConfig)) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);


    /* ========================= FDCAN 3 ========================= */
    // 复用 FDCAN_FilterConfig 结构体，参数一致
    if ((HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_FilterConfig)) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);

    BSP_CAN_Init_Locks();
    BSP_CAN_Init_Msg();
    // 创建 CAN 发送消息队列
    Can_Tx_Queue = osMessageQueueNew(16, sizeof(Struct_CAN_Tx_Msg), NULL);
}

/**
 * @brief  注册 CAN 接收回调函数
 * @details 在回调列表中查找空闲位置或匹配的 ID 进行注册。
 * @param  can_id    需要监听的 CAN ID (标准帧)
 * @param  pCallback 回调函数指针
 * @return bool      
 * - true: 注册成功
 * - false: 注册失败 (回调列表已满，请增大 MAX_CAN_CALLBACKS 宏定义)
 * @note   调用此函数后务必检查返回值，确保注册成功。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id, CAN_RxCallback_t pCallback)
{
    for (int i = 0; i < MAX_CAN_CALLBACKS; i++)
    {
        // 找到一个空位，或者覆盖已有的相同 ID
        if ((g_CanCallbacks[i].is_used == 0) || (g_CanCallbacks[i].ID == can_id))
        {
            g_CanCallbacks[i].ID = can_id;
            g_CanCallbacks[i].func = pCallback;
            g_CanCallbacks[i].is_used = 1;
            return true;
        }
    }
    return false; 
}

/**
 * @brief  HAL库 FDCAN FIFO0 接收中断回调
 * @note   此函数在中断上下文中执行，应避免耗时操作
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // 1. 读取数据
        if ((HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData)) == HAL_OK)
        {
            // 2. 遍历注册表
            for (int i = 0; i < MAX_CAN_CALLBACKS; i++)
            {
                if ((g_CanCallbacks[i].is_used) && (g_CanCallbacks[i].ID == rxHeader.Identifier))
                {
                    // 3. 调用注册进来的函数
                    g_CanCallbacks[i].func(hfdcan, rxHeader.Identifier, rxData, rxHeader.DataLength >> 16);
                    break; // 找到后退出
                }
            }
        }
    }
}

/**
  * @brief  通用 CAN 发送函数 (线程安全)
  * @param  hfdcan: CAN 句柄
  * @param  id: 目标 ID (标准帧)
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @return 0: 失败/超时, 1: 成功
  */
static bool BSP_CAN_SendMsg(Struct_CAN_Tx_Msg *TxMsg) {
   FDCAN_HandleTypeDef* hfdcan = TxMsg->hfdcan;
   uint32_t id = TxMsg->id;
   uint8_t* data = TxMsg->data;
   uint32_t len = TxMsg->len;

   if (hfdcan == NULL || data == NULL || len == 0 || len > FDCAN_MAX_PAYLOAD)
   {
    TxMsg->status = CAN_NULL;
    return false;
  }
  FDCAN_TxHeaderTypeDef TxHeader;
  osMutexId_t pMutex = NULL;

  // 1. 根据句柄选择对应的锁
  if (hfdcan == &hfdcan1)
    pMutex = Can1_TxMutex;
  else if (hfdcan == &hfdcan2)
    pMutex = Can2_TxMutex;
  else if (hfdcan == &hfdcan3)
    pMutex = Can3_TxMutex;

  if (pMutex == NULL)
  {
      TxMsg->status = CAN_LOCK;
    return false;
  }

  // 2. 获取锁 (等待 2ms，如果总线太忙拿不到锁就放弃，防止卡死控制环)
  if (osMutexAcquire(pMutex, 2U) != osOK)
  {
      TxMsg->status = CAN_BUSY;
    return false; // 获取锁失败（总线拥堵）
  }

  // 3. 配置发送头 (针对 H7 FDCAN)
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = len;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 关闭波特率切换 (Classic CAN)
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 经典模式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  // 4. 检查 FIFO 是否有空位
  if ((HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)) > 0) {
    // 放入发送队列
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);

    TxMsg->timestamp[1] = SYS_Timestamp_Get_Microsecond();
    // 5. 释放锁
    osMutexRelease(pMutex);
    TxMsg->status = (status == HAL_OK) ? CAN_OK : CAN_ERROR;
    return (status == HAL_OK);
  }

  // FIFO 满了，释放锁并返回失败
  osMutexRelease(pMutex);
  TxMsg->status = CAN_BUSY;
  return false;
}

/**
 * @brief  提交预设的 CAN 消息进行发送
 * @param  TxHeader: 包含 CAN 句柄、ID、数据和长度的结构体指针
 * @return true: 消息成功提交到发送队列, false: 提交失败 (如无效参数或总线拥堵)
 * @note   此函数操作的是异步发送,再使用非实时性要求的设备时调用。
 */
bool CAN_Tx_Submit(Struct_CAN_Tx_Msg *TxHeader) {
    if (TxHeader == NULL || TxHeader->hfdcan == NULL) {
        if (TxHeader != NULL) {
            TxHeader->status = CAN_NULL;
        }
        return false;
    }
    return (osMessageQueuePut(Can_Tx_Queue, TxHeader, 0, 0) == osOK);
}

/**
 * @brief  更新预设的 CAN 消息并提交发送
 * @param  TxMsg: 包含 CAN 句柄、ID、数据和长度的结构体指针
 * @return true: 消息成功提交, false: 提交失败 (如无效参数、总线拥堵或 timestamp[0] 早于上次发送时间)
 * @note   timestamp 语义: [0]=本次提交时间, [1]=上一次 BSP_CAN_SendMsg 实际发送完成时间。
 *         若 timestamp[0] < timestamp[1] 说明这是旧数据, 拒绝覆盖, 返回 false。
 *         提交前先从 Tx_Msg_Buffer 回写 timestamp[1], 使调用者下次可通过更新 timestamp[0] 触发重新提交。
 */
bool CAN_Tx_Perform(Struct_CAN_Tx_Msg *TxMsg) {

    if (TxMsg == NULL || TxMsg->hfdcan == NULL || TxMsg->timestamp[0] < TxMsg->timestamp[1])
    {
    return false;
  }
  if (TxMsg->hfdcan == &hfdcan1) {
    TxMsg->timestamp[1] = Tx_Msg_Buffer[0].timestamp[1];  // 回写上次实际发送时间
    Tx_Msg_Buffer[0] = *TxMsg;
  }
  else if (TxMsg->hfdcan == &hfdcan2) {
    TxMsg->timestamp[1] = Tx_Msg_Buffer[1].timestamp[1];
    Tx_Msg_Buffer[1] = *TxMsg;
  }
  else if (TxMsg->hfdcan == &hfdcan3) {
    TxMsg->timestamp[1] = Tx_Msg_Buffer[2].timestamp[1];
    Tx_Msg_Buffer[2] = *TxMsg;
  }
  else
  {
      TxMsg->status = CAN_NULL;
    return false; // 无效的 CAN 句柄
  }
  TxMsg->status = CAN_OK;
  return true;
}
/**
 * @brief  批量发送预设的 CAN 消息
 * @return true: 全部发送成功, false: 至少有一条发送失败
 * @note   此函数会尝试发送 Tx_Msg_Buffer 中的三条消息，并返回整体结果。
 *         适用于需要同时更新多条消息的场景，减少调用次数。
 *         
 */
bool BSP_CAN_SendPer(void) {
  if (!BSP_CAN_SendMsg(&Tx_Msg_Buffer[0])) return Tx_Msg_Buffer[0].status == CAN_OK;
  if (!BSP_CAN_SendMsg(&Tx_Msg_Buffer[1])) return Tx_Msg_Buffer[1].status == CAN_OK;
  if (!BSP_CAN_SendMsg(&Tx_Msg_Buffer[2])) return Tx_Msg_Buffer[2].status == CAN_OK;
  return true;
}

/**
 * @brief  CAN 发送任务的消息处理函数
 * @note   此函数应在一个独立的 FreeRTOS 任务中运行，持续监听发送队列并处理消息。
 */
void BSP_CAN_SendAsync(void) {
  Struct_CAN_Tx_Msg msg;
  while (osMessageQueueGet(Can_Tx_Queue, &msg, NULL, 0) == osOK) {
    BSP_CAN_SendMsg(&msg);
  }
}