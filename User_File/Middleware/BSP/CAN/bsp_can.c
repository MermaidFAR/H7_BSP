/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   板级支持包：CAN 总线驱动实现
 * @details 提供接收回调分发、插入队列和多 ID 周期缓冲。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.1
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "cmsis_os2.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

/** 最多允许注册的接收回调数量。 */
#define MAX_CAN_CALLBACKS  (16)

/** 周期发送槽总数；每个不同的 (FDCAN, CAN ID) 占用一个槽。 */
#define CAN_TX_SLOT_COUNT  (32)

/** 插入发送队列能够缓存的消息数量。 */
#define CAN_TX_QUEUE_DEPTH (16)

/* Private types -------------------------------------------------------------*/

/**
 * @brief 一项 CAN 接收回调注册记录。
 * @note hfdcan 和 ID 共同组成唯一键。
 */
typedef struct
{
    uint32_t ID;                 /*!< 要监听的标准帧 ID。 */
    FDCAN_HandleTypeDef *hfdcan; /*!< 要监听的 FDCAN 总线。 */
    CAN_RxCallback_t func;       /*!< 收到匹配报文时调用的函数。 */
    uint8_t is_used;             /*!< 该注册项是否已经启用。 */
    void *context;               /*!< 原样传回回调的设备实例或用户数据。 */
} CAN_CallbackEntry_t;

/**
 * @brief 一个周期发送槽。
 * @note 一个槽只属于一个固定的 (FDCAN, CAN ID)。
 */
typedef struct
{
    Struct_CAN_Tx_Msg message; /*!< 当前键最近一次发布的完整消息。 */
    uint32_t version;          /*!< 发布版本；每次更新消息时递增。 */
    uint32_t sent_version;     /*!< 最近一次成功写入硬件 FIFO 的版本。 */
    uint8_t is_used;           /*!< 该槽是否已经分配给某个发送键。 */
} Struct_CAN_Tx_Slot;

/* Private variables ---------------------------------------------------------*/

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

static CAN_CallbackEntry_t Can_RxCallbacks[MAX_CAN_CALLBACKS];
static Struct_CAN_Tx_Slot Can_TxSlots[CAN_TX_SLOT_COUNT];
static uint16_t Can_TxRoundRobin;

static osMessageQueueId_t Can_TxQueue;

/* Private function declarations ---------------------------------------------*/

static bool BSP_CAN_HandleIsValid(FDCAN_HandleTypeDef *hfdcan);
static bool BSP_CAN_MessageIsValid(const Struct_CAN_Tx_Msg *message);
static uint32_t BSP_CAN_EnterCritical(void);
static void BSP_CAN_ExitCritical(uint32_t primask);
static void BSP_CAN_ConfigBus(FDCAN_HandleTypeDef *hfdcan);
static bool BSP_CAN_SendMsg(const Struct_CAN_Tx_Msg *message);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 判断 FDCAN 句柄是否属于 BSP 管理的三条总线。
 * @param hfdcan 待检查的 FDCAN 句柄。
 * @return true 是有效句柄；false 不是有效句柄。
 */
static bool BSP_CAN_HandleIsValid(FDCAN_HandleTypeDef *hfdcan)
{
    return hfdcan == &hfdcan1 ||
           hfdcan == &hfdcan2 ||
           hfdcan == &hfdcan3;
}

/**
 * @brief 检查发送消息是否满足当前 Classic CAN 发送接口的要求。
 * @param message 待检查的发送消息。
 * @return true 消息有效；false 消息为空、总线无效、ID 越界或长度无效。
 */
static bool BSP_CAN_MessageIsValid(const Struct_CAN_Tx_Msg *message)
{
    return message != NULL &&
           BSP_CAN_HandleIsValid(message->hfdcan) &&
           message->id <= 0x7FF &&
           message->len > 0 &&
           message->len <= FDCAN_MAX_PAYLOAD;
}

/**
 * @brief 关闭中断并进入用于保护 CAN 共享数据的临界区。
 * @return 进入临界区前的 PRIMASK，用于恢复原来的中断状态。
 * @note 必须与 BSP_CAN_ExitCritical 成对调用。
 */
static uint32_t BSP_CAN_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

/**
 * @brief 退出临界区，并恢复进入前的中断状态。
 * @param primask BSP_CAN_EnterCritical 返回的原始 PRIMASK。
 */
static void BSP_CAN_ExitCritical(uint32_t primask)
{
    __DMB();
    if (primask == 0)
    {
        __enable_irq();
    }
}

/**
 * @brief 配置并启动一条 FDCAN 总线的接收功能。
 * @param hfdcan 要配置的 FDCAN 总线句柄。
 * @details 标准帧全部进入 RX FIFO0，扩展帧和未匹配帧被拒绝，
 *          随后启用 FIFO0 新消息中断并启动外设。
 */
static void BSP_CAN_ConfigBus(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0) != HAL_OK ||
        HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 初始化 CAN BSP 使用的软件资源并启动三条 FDCAN 总线。
 * @details 清空周期发送槽，复位轮询位置，并在首次调用时创建插入发送队列。
 * @note 消息队列创建失败时调用 Error_Handler。
 */
void BSP_CAN_ConfigInit(void)
{
    memset(Can_TxSlots, 0, sizeof(Can_TxSlots));
    Can_TxRoundRobin = 0;

    if (Can_TxQueue == NULL)
    {
        Can_TxQueue = osMessageQueueNew(CAN_TX_QUEUE_DEPTH,
                                        sizeof(Struct_CAN_Tx_Msg),
                                        NULL);
    }

    if (Can_TxQueue == NULL)
    {
        Error_Handler();
    }

    BSP_CAN_ConfigBus(&hfdcan1);
    BSP_CAN_ConfigBus(&hfdcan2);
    BSP_CAN_ConfigBus(&hfdcan3);
}

/**
 * @brief 按 (FDCAN 句柄, CAN ID) 注册接收回调。
 * @param can_id 要监听的标准帧 ID。
 * @param hfdcan 要监听的 FDCAN 总线句柄。
 * @param callback 收到匹配报文时调用的函数。
 * @param context 原样传回 callback 的设备实例或用户数据。
 * @return true 注册成功；false 注册表已满或相同键已经存在。
 * @note 注册过程在临界区内完成，可以避免接收中断看到尚未填写完整的表项。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context)
{
    uint32_t primask;
    uint16_t index;
    int16_t free_index = -1;

    primask = BSP_CAN_EnterCritical();
    for (index = 0; index < MAX_CAN_CALLBACKS; index++)
    {
        if (Can_RxCallbacks[index].is_used != 0)
        {
            if (Can_RxCallbacks[index].hfdcan == hfdcan &&
                Can_RxCallbacks[index].ID == can_id)
            {
                BSP_CAN_ExitCritical(primask);
                return false;
            }
        }
        else if (free_index < 0)
        {
            free_index = (int16_t)index;
        }
    }

    if (free_index < 0)
    {
        BSP_CAN_ExitCritical(primask);
        return false;
    }

    index = (uint16_t)free_index;
    Can_RxCallbacks[index].ID = can_id;
    Can_RxCallbacks[index].hfdcan = hfdcan;
    Can_RxCallbacks[index].func = callback;
    Can_RxCallbacks[index].context = context;

    /* 最后启用表项，保证接收中断只会读取已经填写完整的内容。 */
    Can_RxCallbacks[index].is_used = 1;

    BSP_CAN_ExitCritical(primask);
    return true;
}

/**
 * @brief HAL 的 FDCAN RX FIFO0 中断回调入口。
 * @param hfdcan 触发本次中断的 FDCAN 总线句柄。
 * @param RxFifo0ITs FIFO0 中断事件标志。
 * @details 函数会取空 FIFO0，并将每帧数据分发给键完全匹配的已注册回调。
 * @note 用户回调在中断上下文中执行，不应阻塞。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[FDCAN_MAX_PAYLOAD];
    uint16_t index;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                   FDCAN_RX_FIFO0,
                                   &rx_header,
                                   rx_data) != HAL_OK)
        {
            break;
        }

        for (index = 0; index < MAX_CAN_CALLBACKS; index++)
        {
            if (Can_RxCallbacks[index].is_used != 0 &&
                Can_RxCallbacks[index].hfdcan == hfdcan &&
                Can_RxCallbacks[index].ID == rx_header.Identifier)
            {
                Can_RxCallbacks[index].func(hfdcan,
                                            rx_header.Identifier,
                                            rx_data,
                                            rx_header.DataLength,
                                            Can_RxCallbacks[index].context);
                break;
            }
        }
    }
}

/**
 * @brief 将一帧消息复制到插入发送队列。
 * @param tx_msg 要提交的完整 CAN 消息。
 * @return true 入队成功；false 消息无效、队列未创建或队列已满。
 * @note 超时时间为零，本函数不会等待空闲队列位置。
 */
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *tx_msg)
{
    if (!BSP_CAN_MessageIsValid(tx_msg) || Can_TxQueue == NULL)
    {
        return false;
    }

    return osMessageQueuePut(Can_TxQueue, tx_msg, 0, 0) == osOK;
}

/**
 * @brief 发布某个 (FDCAN 句柄, CAN ID) 的最新周期发送数据。
 * @param tx_msg 要发布的完整 CAN 消息。
 * @return true 已更新已有槽或分配新槽；false 消息无效或周期槽已满。
 * @details 先查找键完全相同的槽；没有匹配项时自动使用第一个空槽。
 *          对同一键再次调用会覆盖旧数据并递增版本号，因此发送任务取到的
 *          总是该键尚未发送的最新完整消息，不需要设备层管理槽号。
 */
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *tx_msg)
{
    Struct_CAN_Tx_Slot *slot;
    uint32_t primask;
    uint16_t index;
    int16_t slot_index = -1;
    int16_t free_index = -1;

    if (!BSP_CAN_MessageIsValid(tx_msg))
    {
        return false;
    }

    primask = BSP_CAN_EnterCritical();

    /* 同时寻找精确匹配槽和第一个可分配的空槽。 */
    for (index = 0; index < CAN_TX_SLOT_COUNT; index++)
    {
        if (Can_TxSlots[index].is_used != 0)
        {
            if (Can_TxSlots[index].message.hfdcan == tx_msg->hfdcan &&
                Can_TxSlots[index].message.id == tx_msg->id)
            {
                slot_index = (int16_t)index;
                break;
            }
        }
        else if (free_index < 0)
        {
            free_index = (int16_t)index;
        }
    }

    /* 尚未为该键分配槽时，自动占用第一个空槽。 */
    if (slot_index < 0)
    {
        slot_index = free_index;
    }

    if (slot_index < 0)
    {
        BSP_CAN_ExitCritical(primask);
        return false;
    }

    slot = &Can_TxSlots[slot_index];
    if (slot->is_used == 0)
    {
        memset(slot, 0, sizeof(*slot));
        slot->message.hfdcan = tx_msg->hfdcan;
        slot->message.id = tx_msg->id;
    }

    /* 在临界区内复制整帧，发送任务不会读到一半新、一半旧的数据。 */
    memcpy(slot->message.data, tx_msg->data, tx_msg->len);
    if (tx_msg->len < FDCAN_MAX_PAYLOAD)
    {
        memset(&slot->message.data[tx_msg->len],
               0,
               FDCAN_MAX_PAYLOAD - tx_msg->len);
    }
    slot->message.len = tx_msg->len;
    slot->version++;
    /* 新槽最后再标记为可用，避免消费者看到尚未初始化完整的槽。 */
    slot->is_used = 1;

    BSP_CAN_ExitCritical(primask);
    return true;
}

/**
 * @brief 尝试把一帧消息写入对应 FDCAN 的硬件发送 FIFO。
 * @param message 要发送的完整 CAN 消息。
 * @return true HAL 已接受该消息；false 消息无效、FIFO 已满或 HAL 写入失败。
 * @note 返回 true 表示消息已经进入硬件发送 FIFO，不表示对端已经收到。
 */
static bool BSP_CAN_SendMsg(const Struct_CAN_Tx_Msg *message)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if (!BSP_CAN_MessageIsValid(message) ||
        HAL_FDCAN_GetTxFifoFreeLevel(message->hfdcan) == 0)
    {
        return false;
    }

    tx_header.Identifier = message->id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = message->len;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(message->hfdcan,
                                         &tx_header,
                                         message->data) == HAL_OK;
}

/**
 * @brief 按先进先出顺序处理当前插入发送队列中的全部消息。
 * @details 每次先从软件队列取出一帧，再尝试写入对应总线的硬件发送 FIFO。
 * @note 当前实现不会把写入失败的消息重新放回软件队列。
 */
void BSP_CAN_SendAsync(void)
{
    Struct_CAN_Tx_Msg message;

    while (osMessageQueueGet(Can_TxQueue,
                             &message,
                             NULL,
                             0) == osOK)
    {
        BSP_CAN_SendMsg(&message);
    }
}

/**
 * @brief 轮询周期槽，并发送每个槽尚未处理的最新版本。
 * @return true 所有待处理槽均已成功写入硬件 FIFO，或没有待处理数据；
 *         false 至少有一个待处理槽本轮发送失败。
 * @details 每轮从上次结束位置继续扫描，避免低下标槽长期优先。发送前在临界区
 *          内复制消息和版本，实际 HAL 发送在临界区外完成。发送成功后只确认本次
 *          快照的版本；若生产者在发送期间发布了新版本，新版本会保留到下一轮。
 */
bool BSP_CAN_SendPer(void)
{
    Struct_CAN_Tx_Msg message;
    Struct_CAN_Tx_Slot *slot;
    uint32_t sending_version = 0;
    uint32_t primask;
    uint16_t start_index = Can_TxRoundRobin;
    uint16_t scan_count;
    uint16_t slot_index;
    uint8_t pending;
    bool result = true;

    for (scan_count = 0; scan_count < CAN_TX_SLOT_COUNT; scan_count++)
    {
        slot_index = (uint16_t)((start_index + scan_count) % CAN_TX_SLOT_COUNT);

        /* 只在临界区内读取共享槽，耗时的 HAL 调用留在临界区外。 */
        primask = BSP_CAN_EnterCritical();
        slot = &Can_TxSlots[slot_index];
        pending = slot->is_used != 0 &&
                  slot->version != slot->sent_version;

        if (pending != 0)
        {
            message = slot->message;
            sending_version = slot->version;
        }
        BSP_CAN_ExitCritical(primask);

        if (pending == 0)
        {
            continue;
        }

        if (!BSP_CAN_SendMsg(&message))
        {
            result = false;
            continue;
        }

        primask = BSP_CAN_EnterCritical();

        /* 只确认快照版本；发送期间产生的新版本仍保持待发送状态。 */
        Can_TxSlots[slot_index].sent_version = sending_version;
        BSP_CAN_ExitCritical(primask);

        Can_TxRoundRobin =
            (uint16_t)((slot_index + 1) % CAN_TX_SLOT_COUNT);
    }

    return result;
}
