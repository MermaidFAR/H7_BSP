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
#include "sys_critical.h"
#include "sys_timestamp.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

/** 周期发送槽总数；每个不同的 (FDCAN, CAN ID) 占用一个槽。 */
#define CAN_TX_SLOT_COUNT  (32)

/** 每条总线的插入发送队列深度。 */
#define CAN_TX_QUEUE_DEPTH (16)

/* Private types -------------------------------------------------------------*/

/**
 * @brief 一项 CAN 接收回调注册记录。
 */
typedef struct
{
    uint32_t ID;                 /*!< 要监听的标准帧 ID。 */
    FDCAN_HandleTypeDef *hfdcan; /*!< 要监听的 FDCAN 总线。 */
    CAN_RxCallback_t func;       /*!< 收到匹配报文时调用的函数。 */
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
} Struct_CAN_Tx_Slot;

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    osMessageQueueId_t tx_queue;
    Struct_CAN_Tx_Msg pending_message;
    bool has_pending;
    uint64_t last_recovery;
} CAN_Bus_t;

/* Private variables ---------------------------------------------------------*/

extern osThreadId_t CanTxTaskHandle;

static CAN_Bus_t Can_Buses[] = {
    {.handle = &hfdcan1},
    {.handle = &hfdcan2},
    {.handle = &hfdcan3},
};
#define CAN_BUS_COUNT (sizeof(Can_Buses) / sizeof(Can_Buses[0]))

static CAN_CallbackEntry_t Can_RxCallbacks[MAX_CAN_CALLBACKS];
static volatile uint32_t Can_RxCallbackCount;
static Struct_CAN_Tx_Slot Can_TxSlots[CAN_TX_SLOT_COUNT];
static volatile uint16_t Can_TxSlotCount;
static uint16_t Can_TxRoundRobin;

static CAN_Bus_t *BSP_CAN_GetBus(FDCAN_HandleTypeDef *hfdcan)
{
    for (size_t i = 0; i < CAN_BUS_COUNT; i++)
    {
        if (Can_Buses[i].handle == hfdcan)
        {
            return &Can_Buses[i];
        }
    }
    return NULL;
}

static bool BSP_CAN_MessageIsValid(const Struct_CAN_Tx_Msg *message)
{
    return message != NULL && message->id <= 0x7FF &&
           message->len > 0 && message->len <= FDCAN_MAX_PAYLOAD;
}

static Struct_CAN_Tx_Msg BSP_CAN_CopyMessage(const Struct_CAN_Tx_Msg *message)
{
    /* HAL 以 32 位为单位读数据，补齐 DLC 之外的字节。 */
    Struct_CAN_Tx_Msg copy = {.hfdcan = message->hfdcan, .id = message->id, .len = message->len};
    memcpy(copy.data, message->data, copy.len);
    return copy;
}

static void BSP_CAN_WakeTxTask(void)
{
    if (CanTxTaskHandle != NULL && __get_PRIMASK() == 0U)
    {
        osThreadFlagsSet(CanTxTaskHandle, BSP_CAN_TX_READY_FLAG);
    }
}

/**
 * @brief 配置并启动一条 FDCAN 总线的接收功能。
 * @param hfdcan 要配置的 FDCAN 总线句柄。
 * @details 标准数据帧进入 RX FIFO0，远程帧、扩展帧和未匹配帧被拒绝，
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
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
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
 * @details 清空周期发送槽与重试状态，为每条总线创建独立发送队列。
 * @note 消息队列创建失败时调用 Error_Handler。
 */
void BSP_CAN_ConfigInit(void)
{
    Can_TxSlotCount = 0;
    Can_TxRoundRobin = 0;

    for (size_t i = 0; i < CAN_BUS_COUNT; i++)
    {
        CAN_Bus_t *bus = &Can_Buses[i];
        bus->has_pending = false;
        bus->last_recovery = 0;
        if (bus->tx_queue == NULL)
        {
            bus->tx_queue = osMessageQueueNew(CAN_TX_QUEUE_DEPTH, sizeof(Struct_CAN_Tx_Msg), NULL);
            if (bus->tx_queue == NULL)
            {
                Error_Handler();
            }
        }
        BSP_CAN_ConfigBus(bus->handle);
    }
}

/**
 * @brief 按 (FDCAN 句柄, CAN ID) 注册接收回调。
 * @param can_id 要监听的标准帧 ID。
 * @param hfdcan 要监听的 FDCAN 总线句柄。
 * @param callback 收到匹配报文时调用的函数。
 * @param context 原样传回 callback 的设备实例或用户数据。
 * @return true 注册成功；false 参数无效、注册表已满或同一回调实例已注册。
 * @note 注册过程在临界区内完成，可以避免接收中断看到尚未填写完整的表项。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context)
{
    if (BSP_CAN_GetBus(hfdcan) == NULL || can_id > 0x7FF || callback == NULL)
    {
        return false;
    }

    const uint32_t primask = SYS_Enter_Critical();
    const uint32_t count = Can_RxCallbackCount;
    for (uint32_t index = 0; index < count; index++)
    {
        const CAN_CallbackEntry_t *entry = &Can_RxCallbacks[index];
        if (entry->hfdcan == hfdcan && entry->ID == can_id &&
            entry->func == callback && entry->context == context)
        {
            SYS_Exit_Critical(primask);
            return false;
        }
    }

    if (count == MAX_CAN_CALLBACKS)
    {
        SYS_Exit_Critical(primask);
        return false;
    }

    Can_RxCallbacks[count] = (CAN_CallbackEntry_t){
        .ID = can_id, .hfdcan = hfdcan, .func = callback, .context = context
    };
    __DMB();
    Can_RxCallbackCount = count + 1;

    SYS_Exit_Critical(primask);
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
    /* 当前 HAL 按 DLC 表复制，Classic DLC 9..15 也可能复制 12..64 字节。
     * 必须先容纳 HAL 的最大写入，再按 Classic 语义向设备层提供最多 8 字节。 */
    uint8_t rx_data[64];
    uint32_t rx_length;

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

        if (rx_header.IdType != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME ||
            rx_header.FDFormat != FDCAN_CLASSIC_CAN)
        {
            continue;
        }
        rx_length = rx_header.DataLength <= FDCAN_MAX_PAYLOAD ?
                    rx_header.DataLength : FDCAN_MAX_PAYLOAD;

        const uint32_t count = Can_RxCallbackCount;
        __DMB();
        for (uint32_t index = 0; index < count; index++)
        {
            const CAN_CallbackEntry_t *entry = &Can_RxCallbacks[index];
            if (entry->hfdcan == hfdcan && entry->ID == rx_header.Identifier)
            {
                entry->func(hfdcan, rx_header.Identifier, rx_data, rx_length, entry->context);
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
    if (!BSP_CAN_MessageIsValid(tx_msg) || __get_PRIMASK() != 0U)
    {
        return false;
    }
    CAN_Bus_t *bus = BSP_CAN_GetBus(tx_msg->hfdcan);
    if (bus == NULL || bus->tx_queue == NULL)
    {
        return false;
    }
    const Struct_CAN_Tx_Msg message = BSP_CAN_CopyMessage(tx_msg);
    if (osMessageQueuePut(bus->tx_queue, &message, 0, 0) != osOK)
    {
        return false;
    }

    BSP_CAN_WakeTxTask();
    return true;
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
    if (!BSP_CAN_MessageIsValid(tx_msg) || BSP_CAN_GetBus(tx_msg->hfdcan) == NULL)
    {
        return false;
    }

    const uint32_t primask = SYS_Enter_Critical();
    const uint16_t count = Can_TxSlotCount;
    uint16_t index = 0;
    for (; index < count; index++)
    {
        const Struct_CAN_Tx_Msg *existing = &Can_TxSlots[index].message;
        if (existing->hfdcan == tx_msg->hfdcan && existing->id == tx_msg->id)
        {
            break;
        }
    }

    if (index == CAN_TX_SLOT_COUNT)
    {
        SYS_Exit_Critical(primask);
        return false;
    }

    Struct_CAN_Tx_Slot *slot = &Can_TxSlots[index];
    if (index == count)
    {
        memset(slot, 0, sizeof(*slot));
    }

    slot->message = BSP_CAN_CopyMessage(tx_msg);
    slot->version++;
    if (index == count)
    {
        Can_TxSlotCount = count + 1;
    }

    SYS_Exit_Critical(primask);
    BSP_CAN_WakeTxTask();
    return true;
}

/**
 * @brief 尝试把一帧消息写入对应 FDCAN 的硬件发送 FIFO。
 * @param message 要发送的完整 CAN 消息。
 * @return true HAL 已接受该消息；false 总线不可用、FIFO 已满或 HAL 写入失败。
 * @note 返回 true 表示消息已经进入硬件发送 FIFO，不表示对端已经收到。
 */
static bool BSP_CAN_SendMsg(const Struct_CAN_Tx_Msg *message)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((message->hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0 ||
        (message->hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0 ||
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

void BSP_CAN_Recover(void)
{
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    for (size_t i = 0; i < CAN_BUS_COUNT; i++)
    {
        CAN_Bus_t *bus = &Can_Buses[i];
        FDCAN_GlobalTypeDef *instance = bus->handle->Instance;
        if ((instance->PSR & FDCAN_PSR_BO) != 0 &&
            (instance->CCCR & FDCAN_CCCR_INIT) != 0 &&
            now - bus->last_recovery >= 100000)
        {
            bus->last_recovery = now;
            CLEAR_BIT(instance->CCCR, FDCAN_CCCR_INIT);
        }
    }
}

/** 按各总线的入队顺序发送，失败帧保留在该总线等待重试。 */
void BSP_CAN_SendAsync(void)
{
    for (size_t i = 0; i < CAN_BUS_COUNT; i++)
    {
        CAN_Bus_t *bus = &Can_Buses[i];
        // 限制单轮工作量，持续提交时其他总线和周期槽仍有机会处理。
        for (uint32_t sent = 0; sent <= CAN_TX_QUEUE_DEPTH; sent++)
        {
            if (!bus->has_pending)
            {
                if (osMessageQueueGet(bus->tx_queue, &bus->pending_message, NULL, 0) != osOK)
                {
                    break;
                }
                bus->has_pending = true;
            }

            if (!BSP_CAN_SendMsg(&bus->pending_message))
            {
                break;
            }
            bus->has_pending = false;
        }
    }
}

/**
 * @brief 轮询周期槽，并发送每个槽尚未处理的最新版本。
 * @return true 所有待处理槽均已成功写入硬件 FIFO，或没有待处理数据；
 *         false 至少有一个待处理槽本轮发送失败。
 * @details 每轮轮换起始槽，只扫描已分配的槽。发送前在临界区
 *          内复制消息和版本，实际 HAL 发送在临界区外完成。发送成功后只确认本次
 *          快照的版本；若生产者在发送期间发布了新版本，新版本会保留到下一轮。
 */
bool BSP_CAN_SendPer(void)
{
    const uint16_t count = Can_TxSlotCount;
    if (count == 0)
    {
        return true;
    }

    uint16_t index = Can_TxRoundRobin;
    Can_TxRoundRobin = index + 1 == count ? 0 : index + 1;
    bool result = true;
    for (uint16_t scanned = 0; scanned < count; scanned++)
    {
        Struct_CAN_Tx_Slot *slot = &Can_TxSlots[index];
        index = index + 1 == count ? 0 : index + 1;

        const uint32_t primask = SYS_Enter_Critical();
        const uint32_t version = slot->version;
        if (version == slot->sent_version)
        {
            SYS_Exit_Critical(primask);
            continue;
        }
        const Struct_CAN_Tx_Msg message = slot->message;
        SYS_Exit_Critical(primask);

        if (BSP_CAN_SendMsg(&message))
        {
            // 只有发送任务更新 sent_version；发送期间发布的新版本继续等待。
            slot->sent_version = version;
        }
        else
        {
            result = false;
        }
    }

    return result;
}
