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
#define CAN_TX_SLOT_COUNT (32)

/** 每条总线的插入发送队列深度。 */
#define CAN_TX_QUEUE_DEPTH (16)
#define CAN_TX_HW_BUFFER_COUNT (32U)
#define CAN_BUS_RECOVERY_INTERVAL_US (100000U)
#define CAN_ALL_IDS (0x800U)

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
    Struct_CAN_Tx_Msg message;  /*!< 当前键最近一次发布的完整消息。 */
    uint64_t timestamp_us;      /*!< 最近一次发布时间。 */
    uint32_t version;           /*!< 发布版本；每次更新消息时递增。 */
    uint32_t sent_version;      /*!< 最近一次已发送或已作废的版本。 */
    uint32_t motion_generation; /*!< 指定 ID 被停用时递增。 */
    bool safe_stop;             /*!< 当前版本为全零停机帧。 */
} Struct_CAN_Tx_Slot;

typedef struct
{
    Struct_CAN_Tx_Msg message;
    uint64_t timestamp_us;
    uint32_t bus_generation;
    uint32_t stop_generation;
    const Struct_CAN_Tx_Slot *motion_slot;
    uint32_t motion_generation;
} CAN_Queued_Tx_t;

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    osMessageQueueId_t tx_queue;
    CAN_Queued_Tx_t pending_message;
    bool has_pending;
    uint32_t generation;
    uint64_t last_recovery;
    uint32_t motion_buffers;
    uint64_t tx_timestamp_us[CAN_TX_HW_BUFFER_COUNT];
    uint16_t motion_id[CAN_TX_HW_BUFFER_COUNT];
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
static uint32_t Can_StopGeneration;
static uint64_t Can_StopTimestampUs;
static bool Can_MotionAllowed = true;

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
        bus->generation++;
        bus->last_recovery = 0;
        bus->motion_buffers = 0;
        if (bus->tx_queue == NULL)
        {
            bus->tx_queue = osMessageQueueNew(CAN_TX_QUEUE_DEPTH, sizeof(CAN_Queued_Tx_t), NULL);
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

    Can_RxCallbacks[count] = (CAN_CallbackEntry_t) {
        .ID = can_id, .hfdcan = hfdcan, .func = callback, .context = context};
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
        rx_length = rx_header.DataLength <= FDCAN_MAX_PAYLOAD ? rx_header.DataLength : FDCAN_MAX_PAYLOAD;

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

/* 调用方持有临界区，槽地址在 CAN BSP 初始化后保持稳定。 */
static Struct_CAN_Tx_Slot *BSP_CAN_GetTxSlot(const Struct_CAN_Tx_Msg *message)
{
    for (uint16_t i = 0; i < Can_TxSlotCount; i++)
    {
        Struct_CAN_Tx_Slot *slot = &Can_TxSlots[i];
        if (slot->message.hfdcan == message->hfdcan && slot->message.id == message->id)
        {
            return slot;
        }
    }
    if (Can_TxSlotCount == CAN_TX_SLOT_COUNT)
    {
        return NULL;
    }

    Struct_CAN_Tx_Slot *slot = &Can_TxSlots[Can_TxSlotCount];
    memset(slot, 0, sizeof(*slot));
    slot->message.hfdcan = message->hfdcan;
    slot->message.id = message->id;
    Can_TxSlotCount++;
    return slot;
}

static bool BSP_CAN_Submit(const Struct_CAN_Tx_Msg *tx_msg, bool motion)
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
    const uint32_t primask = SYS_Enter_Critical();
    if (motion && !Can_MotionAllowed)
    {
        SYS_Exit_Critical(primask);
        return false;
    }
    const Struct_CAN_Tx_Slot *slot = motion ? BSP_CAN_GetTxSlot(tx_msg) : NULL;
    if (motion && slot == NULL)
    {
        SYS_Exit_Critical(primask);
        return false;
    }
    const CAN_Queued_Tx_t queued = {
        .message = BSP_CAN_CopyMessage(tx_msg),
        .timestamp_us = SYS_Timestamp_Get_Microsecond(),
        .bus_generation = bus->generation,
        .stop_generation = Can_StopGeneration,
        .motion_slot = slot,
        .motion_generation = motion ? slot->motion_generation : 0U,
    };
    /* RTOS 入队在临界区外。发送前按代次作废跨总线恢复的帧及跨停机的运动帧。 */
    SYS_Exit_Critical(primask);
    if (osMessageQueuePut(bus->tx_queue, &queued, 0, 0) != osOK)
    {
        return false;
    }

    BSP_CAN_WakeTxTask();
    return true;
}

bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *tx_msg)
{
    return BSP_CAN_Submit(tx_msg, false);
}

bool CAN_Tx_SubmitMotion(const Struct_CAN_Tx_Msg *tx_msg)
{
    return BSP_CAN_Submit(tx_msg, true);
}

/**
 * @brief 发布某个 (FDCAN 句柄, CAN ID) 的最新周期发送数据。
 * @param tx_msg 要发布的完整 CAN 消息。
 * @return true 已更新已有槽或分配新槽；false 消息无效或周期槽已满。
 * @details 先查找键完全相同的槽；没有匹配项时自动使用第一个空槽。
 *          对同一键再次调用会覆盖旧数据并递增版本号，因此发送任务取到的
 *          总是该键尚未发送的最新完整消息，不需要设备层管理槽号。
 */
static bool BSP_CAN_Publish(const Struct_CAN_Tx_Msg *tx_msg, bool safe_stop,
                            bool stamped, uint64_t timestamp_us)
{
    if (!BSP_CAN_MessageIsValid(tx_msg))
    {
        return false;
    }
    const CAN_Bus_t *bus = BSP_CAN_GetBus(tx_msg->hfdcan);
    if (bus == NULL)
    {
        return false;
    }

    const uint32_t primask = SYS_Enter_Critical();
    if ((!safe_stop && !Can_MotionAllowed) ||
        (stamped && ((Can_StopGeneration != 0U && timestamp_us <= Can_StopTimestampUs) ||
                     (bus->last_recovery != 0U && timestamp_us <= bus->last_recovery))))
    {
        SYS_Exit_Critical(primask);
        return false;
    }
    const Struct_CAN_Tx_Msg message = BSP_CAN_CopyMessage(tx_msg);
    if (safe_stop)
    {
        for (uint8_t i = 0; i < message.len; i++)
        {
            if (message.data[i] != 0U)
            {
                SYS_Exit_Critical(primask);
                return false;
            }
        }
    }
    Struct_CAN_Tx_Slot *slot = BSP_CAN_GetTxSlot(tx_msg);
    if (slot == NULL)
    {
        SYS_Exit_Critical(primask);
        return false;
    }

    slot->message = message;
    slot->safe_stop = safe_stop;
    slot->timestamp_us = stamped ? timestamp_us : SYS_Timestamp_Get_Microsecond();
    slot->version++;

    SYS_Exit_Critical(primask);
    BSP_CAN_WakeTxTask();
    return true;
}

bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *tx_msg)
{
    return BSP_CAN_Publish(tx_msg, false, false, 0U);
}

bool CAN_Tx_PerformStamped(const Struct_CAN_Tx_Msg *tx_msg, uint64_t timestamp_us)
{
    return BSP_CAN_Publish(tx_msg, false, true, timestamp_us);
}

bool CAN_Tx_PerformSafe(const Struct_CAN_Tx_Msg *tx_msg)
{
    return BSP_CAN_Publish(tx_msg, true, false, 0U);
}

/**
 * @brief 尝试把一帧消息写入对应 FDCAN 的硬件发送 FIFO。
 * @param message 要发送的完整 CAN 消息。
 * @return true HAL 已接受该消息；false 总线不可用、FIFO 已满或 HAL 写入失败。
 * @note 返回 true 表示消息已经进入硬件发送 FIFO，不表示对端已经收到。
 * @note 调用方持有临界区，使门控检查、硬件入队和缓冲区登记保持原子性。
 */
static bool BSP_CAN_SendMsg(CAN_Bus_t *bus, const Struct_CAN_Tx_Msg *message,
                            uint64_t timestamp_us, bool motion)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((motion && !Can_MotionAllowed) ||
        (message->hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0 ||
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

    if (HAL_FDCAN_AddMessageToTxFifoQ(message->hfdcan, &tx_header, message->data) != HAL_OK)
    {
        return false;
    }

    const uint32_t buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(bus->handle);
    if (buffer != 0U)
    {
        const uint32_t index = POSITION_VAL(buffer);
        if (index < CAN_TX_HW_BUFFER_COUNT)
        {
            bus->tx_timestamp_us[index] = timestamp_us;
            bus->motion_buffers &= ~buffer;
            if (motion)
            {
                bus->motion_id[index] = (uint16_t) message->id;
                bus->motion_buffers |= buffer;
            }
        }
    }
    return true;
}

/* 调用方持有临界区，作废版本与写硬件 FIFO 使用同一把锁。 */
static void BSP_CAN_DiscardSlots(FDCAN_HandleTypeDef *hfdcan, uint32_t id, bool motion_only)
{
    for (uint16_t i = 0; i < Can_TxSlotCount; i++)
    {
        Struct_CAN_Tx_Slot *slot = &Can_TxSlots[i];
        if (motion_only && slot->safe_stop)
        {
            continue;
        }
        if (hfdcan == NULL || (slot->message.hfdcan == hfdcan &&
                               (id == CAN_ALL_IDS || slot->message.id == id)))
        {
            slot->sent_version = slot->version;
            if (id != CAN_ALL_IDS)
            {
                slot->motion_generation++;
            }
        }
    }
}

static uint32_t BSP_CAN_GetHardwareRequests(const FDCAN_GlobalTypeDef *instance)
{
    // 扫描期间，新写入的请求可能仍在 TXBAR，随后才进入 TXBRP。
    const uint32_t added = instance->TXBAR;
    return added | instance->TXBRP;
}

/* 调用方持有临界区，保留已发布的全零停机帧。 */
static void BSP_CAN_DiscardMotion(void)
{
    BSP_CAN_DiscardSlots(NULL, CAN_ALL_IDS, true);
    Can_StopGeneration++;
    Can_StopTimestampUs = SYS_Timestamp_Get_Microsecond();
    for (size_t i = 0; i < CAN_BUS_COUNT; i++)
    {
        CAN_Bus_t *bus = &Can_Buses[i];
        const uint32_t pending = BSP_CAN_GetHardwareRequests(bus->handle->Instance) & bus->motion_buffers;
        if (pending != 0U)
        {
            bus->handle->Instance->TXBCR = pending;
        }
    }
}

void BSP_CAN_SetMotionAllowed(bool allowed)
{
    const uint32_t primask = SYS_Enter_Critical();
    if (Can_MotionAllowed && !allowed)
    {
        BSP_CAN_DiscardMotion();
    }
    Can_MotionAllowed = allowed;
    SYS_Exit_Critical(primask);
}

void BSP_CAN_DiscardPendingSlots(void)
{
    const uint32_t primask = SYS_Enter_Critical();
    BSP_CAN_DiscardMotion();
    SYS_Exit_Critical(primask);
}

void BSP_CAN_DiscardPendingSlot(FDCAN_HandleTypeDef *hfdcan, uint32_t id)
{
    CAN_Bus_t *bus = BSP_CAN_GetBus(hfdcan);
    if (bus == NULL || id > 0x7FFU)
    {
        return;
    }

    const uint32_t primask = SYS_Enter_Critical();
    BSP_CAN_DiscardSlots(hfdcan, id, false);
    uint32_t pending = BSP_CAN_GetHardwareRequests(hfdcan->Instance) & bus->motion_buffers;
    uint32_t cancel = 0U;
    while (pending != 0U)
    {
        const uint32_t index = POSITION_VAL(pending);
        const uint32_t buffer = 1UL << index;
        if (bus->motion_id[index] == id)
        {
            cancel |= buffer;
        }
        pending &= ~buffer;
    }
    if (cancel != 0U)
    {
        hfdcan->Instance->TXBCR = cancel;
    }
    SYS_Exit_Critical(primask);
}

static void BSP_CAN_ExpireHardwareRequests(CAN_Bus_t *bus, uint64_t now)
{
    /* 总线繁忙时帧可能仍在硬件中等待仲裁，入队后也要按原始时间检查龄期。 */
    FDCAN_GlobalTypeDef *instance = bus->handle->Instance;
    uint32_t pending = BSP_CAN_GetHardwareRequests(instance);
    bus->motion_buffers &= pending;
    uint32_t cancel = 0U;
    while (pending != 0U)
    {
        const uint32_t index = POSITION_VAL(pending);
        const uint32_t buffer = 1UL << index;
        if (now - bus->tx_timestamp_us[index] > CAN_TX_SLOT_MAX_AGE_US)
        {
            cancel |= buffer;
        }
        pending &= ~buffer;
    }
    if (cancel != 0U)
    {
        instance->TXBCR = cancel;
    }
}

void BSP_CAN_Recover(void)
{
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    for (size_t i = 0; i < CAN_BUS_COUNT; i++)
    {
        CAN_Bus_t *bus = &Can_Buses[i];
        FDCAN_GlobalTypeDef *instance = bus->handle->Instance;
        const uint32_t primask = SYS_Enter_Critical();
        if ((instance->PSR & FDCAN_PSR_BO) != 0 &&
            (instance->CCCR & FDCAN_CCCR_INIT) != 0 &&
            now - bus->last_recovery >= CAN_BUS_RECOVERY_INTERVAL_US)
        {
            const uint32_t pending = BSP_CAN_GetHardwareRequests(instance);
            if (pending != 0U)
            {
                // 等下次任务轮询确认取消完成，再允许硬件恢复发送。
                instance->TXBCR = pending;
            }
            else
            {
                BSP_CAN_DiscardSlots(bus->handle, CAN_ALL_IDS, false);
                bus->generation++;
                bus->has_pending = false;
                bus->motion_buffers = 0U;
                bus->last_recovery = now;
                CLEAR_BIT(instance->CCCR, FDCAN_CCCR_INIT);
            }
        }
        else
        {
            BSP_CAN_ExpireHardwareRequests(bus, now);
        }
        SYS_Exit_Critical(primask);
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

            const uint32_t primask = SYS_Enter_Critical();
            const CAN_Queued_Tx_t *pending = &bus->pending_message;
            const Struct_CAN_Tx_Slot *slot = pending->motion_slot;
            if (pending->bus_generation != bus->generation ||
                (slot != NULL && (!Can_MotionAllowed || pending->stop_generation != Can_StopGeneration ||
                                  pending->motion_generation != slot->motion_generation)) ||
                SYS_Timestamp_Get_Microsecond() - pending->timestamp_us > CAN_TX_SLOT_MAX_AGE_US ||
                BSP_CAN_SendMsg(bus, &pending->message, pending->timestamp_us, slot != NULL))
            {
                bus->has_pending = false;
            }
            SYS_Exit_Critical(primask);
            if (bus->has_pending)
            {
                break;
            }
        }
    }
}

/**
 * @brief 轮询周期槽，并发送每个槽尚未处理的最新版本。
 * @return true 所有待处理槽均已成功写入硬件 FIFO，或没有待处理数据；
 *         false 至少有一个待处理槽本轮发送失败。
 * @details 每轮轮换起始槽，只扫描已分配的槽。版本、数据龄期和有界的 HAL
 *          FIFO 写入在同一临界区内处理，防止清槽后仍发出旧快照。
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
        CAN_Bus_t *bus = BSP_CAN_GetBus(slot->message.hfdcan);
        const bool motion = !slot->safe_stop;
        if ((motion && !Can_MotionAllowed) ||
            SYS_Timestamp_Get_Microsecond() - slot->timestamp_us > CAN_TX_SLOT_MAX_AGE_US ||
            BSP_CAN_SendMsg(bus, &slot->message, slot->timestamp_us, motion))
        {
            slot->sent_version = version;
        }
        else
        {
            result = false;
        }
        SYS_Exit_Critical(primask);
    }

    return result;
}
