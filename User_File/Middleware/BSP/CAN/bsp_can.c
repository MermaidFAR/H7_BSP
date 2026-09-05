/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   板级支持包：CAN 总线驱动实现
 * @details 提供接收回调分发、插入队列和多 ID 周期缓冲。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.2
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "cmsis_os2.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

/** 最多允许注册的接收回调数量。 */
#define MAX_CAN_CALLBACKS  (16)

/** 周期发送槽总数；每个不同的总线、帧类型和发送键占用一个槽。 */
#define CAN_TX_SLOT_COUNT  (32)

/** BSP 管理的独立 CAN 总线数量。 */
#define CAN_BUS_COUNT      (3)

/** 每条总线的插入发送队列能够缓存的消息数量。 */
#define CAN_TX_QUEUE_DEPTH (16)

/* Private types -------------------------------------------------------------*/

/**
 * @brief 一项 CAN 接收回调注册记录。
 * @note 同一总线和帧类型的 ID 匹配范围不能重叠。
 */
typedef struct
{
    uint32_t ID;                 /*!< 要监听的 CAN ID，已与掩码相与。 */
    uint32_t mask;               /*!< ID 匹配掩码。 */
    uint32_t id_type;            /*!< 标准帧或扩展帧类型。 */
    FDCAN_HandleTypeDef *hfdcan; /*!< 要监听的 FDCAN 总线。 */
    CAN_RxCallback_t func;       /*!< 收到匹配报文时调用的函数。 */
    uint8_t is_used;             /*!< 该注册项是否已经启用。 */
    void *context;               /*!< 原样传回回调的设备实例或用户数据。 */
} CAN_CallbackEntry_t;

/**
 * @brief 一个周期发送槽。
 * @note 一个槽只属于固定总线、帧类型和默认 ID 键或非零逻辑键。
 */
typedef struct
{
    Struct_CAN_Tx_Msg message; /*!< 当前键最近一次发布的完整消息。 */
    uint32_t version;          /*!< 发布版本；每次更新消息时递增。 */
    uint32_t sent_version;     /*!< 最近一次成功写入硬件 FIFO 的版本。 */
    uint8_t is_used;           /*!< 该槽是否已经分配给某个发送键。 */
} Struct_CAN_Tx_Slot;

/**
 * @brief 单条总线的事件队列和队首重试状态。
 */
typedef struct
{
    osMessageQueueId_t queue;
    Struct_CAN_Tx_Msg pending;
    uint8_t pending_valid;
} Struct_CAN_Tx_Bus;

/* Private variables ---------------------------------------------------------*/

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

static CAN_CallbackEntry_t Can_RxCallbacks[MAX_CAN_CALLBACKS];
static Struct_CAN_Tx_Slot Can_TxSlots[CAN_TX_SLOT_COUNT];
static uint16_t Can_TxRoundRobin;

static Struct_CAN_Tx_Bus Can_TxBuses[CAN_BUS_COUNT];

/* Private function declarations ---------------------------------------------*/

static int16_t BSP_CAN_GetBusIndex(FDCAN_HandleTypeDef *hfdcan);
static bool BSP_CAN_HandleIsValid(FDCAN_HandleTypeDef *hfdcan);
static bool BSP_CAN_IdentifierIsValid(uint32_t id, uint32_t id_type);
static bool BSP_CAN_MessageIsValid(const Struct_CAN_Tx_Msg *message);
static uint32_t BSP_CAN_EnterCritical(void);
static void BSP_CAN_ExitCritical(uint32_t primask);
static void BSP_CAN_ConfigBus(FDCAN_HandleTypeDef *hfdcan);
static bool BSP_CAN_SendMsg(const Struct_CAN_Tx_Msg *message);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 获取句柄对应的总线下标，无效句柄返回 -1。
 */
static int16_t BSP_CAN_GetBusIndex(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return 0;
    if (hfdcan == &hfdcan2) return 1;
    if (hfdcan == &hfdcan3) return 2;
    return -1;
}

/**
 * @brief 判断 FDCAN 句柄是否属于 BSP 管理的三条总线。
 * @param hfdcan 待检查的 FDCAN 句柄。
 * @return true 是有效句柄；false 不是有效句柄。
 */
static bool BSP_CAN_HandleIsValid(FDCAN_HandleTypeDef *hfdcan)
{
    return BSP_CAN_GetBusIndex(hfdcan) >= 0;
}

/**
 * @brief 检查帧类型及对应的 11 位或 29 位标识符范围。
 */
static bool BSP_CAN_IdentifierIsValid(uint32_t id, uint32_t id_type)
{
    return (id_type == FDCAN_STANDARD_ID && id <= 0x7FF) ||
           (id_type == FDCAN_EXTENDED_ID && id <= 0x1FFFFFFF);
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
           BSP_CAN_IdentifierIsValid(message->id, message->id_type) &&
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
 * @details 标准帧全部进入 RX FIFO0，未匹配扩展帧也进入 FIFO0，远程帧拒绝，
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
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
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
 * @details 清空周期发送槽，复位轮询位置，并在首次调用时创建每条总线的事件队列。
 * @note 消息队列创建失败时调用 Error_Handler。
 */
void BSP_CAN_ConfigInit(void)
{
    uint16_t index;

    memset(Can_TxSlots, 0, sizeof(Can_TxSlots));
    Can_TxRoundRobin = 0;
    for (index = 0; index < CAN_BUS_COUNT; index++)
    {
        Can_TxBuses[index].pending_valid = 0;
        if (Can_TxBuses[index].queue == NULL)
        {
            Can_TxBuses[index].queue = osMessageQueueNew(CAN_TX_QUEUE_DEPTH,
                                                        sizeof(Struct_CAN_Tx_Msg),
                                                        NULL);
        }

        if (Can_TxBuses[index].queue == NULL)
        {
            Error_Handler();
        }
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
 * @return true 注册成功；false 参数无效、注册表已满或匹配范围已经存在。
 * @note 注册过程在临界区内完成，可以避免接收中断看到尚未填写完整的表项。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context)
{
    return BSP_CAN_RegisterCallbackEx(can_id,
                                      0x7FF,
                                      FDCAN_STANDARD_ID,
                                      hfdcan,
                                      callback,
                                      context);
}

/**
 * @brief 注册一个标准帧或扩展帧掩码回调，拒绝同总线、同类型的重叠范围。
 */
bool BSP_CAN_RegisterCallbackEx(uint32_t can_id,
                                uint32_t can_mask,
                                uint32_t id_type,
                                FDCAN_HandleTypeDef *hfdcan,
                                CAN_RxCallback_t callback,
                                void *context)
{
    uint32_t primask;
    uint16_t index;
    int16_t free_index = -1;

    if (!BSP_CAN_HandleIsValid(hfdcan) || callback == NULL ||
        !BSP_CAN_IdentifierIsValid(can_id, id_type) ||
        !BSP_CAN_IdentifierIsValid(can_mask, id_type))
    {
        return false;
    }

    primask = BSP_CAN_EnterCritical();
    for (index = 0; index < MAX_CAN_CALLBACKS; index++)
    {
        if (Can_RxCallbacks[index].is_used != 0)
        {
            if (Can_RxCallbacks[index].hfdcan == hfdcan &&
                Can_RxCallbacks[index].id_type == id_type &&
                ((Can_RxCallbacks[index].ID ^ can_id) &
                 Can_RxCallbacks[index].mask & can_mask) == 0)
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
    Can_RxCallbacks[index].ID = can_id & can_mask;
    Can_RxCallbacks[index].mask = can_mask;
    Can_RxCallbacks[index].id_type = id_type;
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
 * @details 函数会取空 FIFO0，只将 Classic CAN 数据帧分发给掩码匹配的回调。
 * @note 用户回调在中断上下文中执行，不应阻塞。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    /* HAL 按 DLC 拷贝数据，先容纳最大 FD 帧，再检查是否允许分发。 */
    uint8_t rx_data[64];
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

        if (rx_header.FDFormat != FDCAN_CLASSIC_CAN ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME ||
            rx_header.DataLength > FDCAN_DLC_BYTES_8 ||
            !BSP_CAN_IdentifierIsValid(rx_header.Identifier, rx_header.IdType))
        {
            continue;
        }

        for (index = 0; index < MAX_CAN_CALLBACKS; index++)
        {
            if (Can_RxCallbacks[index].is_used != 0 &&
                Can_RxCallbacks[index].hfdcan == hfdcan &&
                Can_RxCallbacks[index].id_type == rx_header.IdType &&
                Can_RxCallbacks[index].ID ==
                    (rx_header.Identifier & Can_RxCallbacks[index].mask))
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
    int16_t bus_index;

    if (!BSP_CAN_MessageIsValid(tx_msg))
    {
        return false;
    }

    bus_index = BSP_CAN_GetBusIndex(tx_msg->hfdcan);
    if (Can_TxBuses[bus_index].queue == NULL)
    {
        return false;
    }

    return osMessageQueuePut(Can_TxBuses[bus_index].queue, tx_msg, 0, 0) == osOK;
}

/**
 * @brief 发布某个总线、帧类型和发送键的最新周期数据。
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
                Can_TxSlots[index].message.id_type == tx_msg->id_type &&
                Can_TxSlots[index].message.slot_key == tx_msg->slot_key &&
                (tx_msg->slot_key != 0 ||
                 Can_TxSlots[index].message.id == tx_msg->id))
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
    }

    /* 在临界区内复制整帧，发送任务不会读到一半新、一半旧的数据。 */
    slot->message = *tx_msg;
    if (tx_msg->len < FDCAN_MAX_PAYLOAD)
    {
        memset(&slot->message.data[tx_msg->len],
               0,
               FDCAN_MAX_PAYLOAD - tx_msg->len);
    }
    slot->version++;
    /* 新槽最后再标记为可用，避免消费者看到尚未初始化完整的槽。 */
    slot->is_used = 1;

    BSP_CAN_ExitCritical(primask);
    return true;
}

/**
 * @brief 停止并释放非零逻辑键的周期槽；与发送投递使用同一个临界区。
 */
bool CAN_Tx_Stop(FDCAN_HandleTypeDef *hfdcan,
                 uint32_t id_type,
                 uint32_t slot_key)
{
    uint32_t primask;
    uint16_t index;

    if (!BSP_CAN_HandleIsValid(hfdcan) || slot_key == 0 ||
        !BSP_CAN_IdentifierIsValid(0, id_type))
    {
        return false;
    }

    primask = BSP_CAN_EnterCritical();
    for (index = 0; index < CAN_TX_SLOT_COUNT; index++)
    {
        if (Can_TxSlots[index].is_used != 0 &&
            Can_TxSlots[index].message.hfdcan == hfdcan &&
            Can_TxSlots[index].message.id_type == id_type &&
            Can_TxSlots[index].message.slot_key == slot_key)
        {
            memset(&Can_TxSlots[index], 0, sizeof(Can_TxSlots[index]));
            break;
        }
    }
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
    tx_header.IdType = message->id_type;
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
 * @brief 按各总线的先进先出顺序处理事件队列。
 * @details 每路每轮最多投递 CAN_TX_QUEUE_DEPTH 帧，失败时保留队首并继续下一路。
 * @note 仅由单个 CAN 发送任务调用，保证队列消费顺序。
 */
void BSP_CAN_SendAsync(void)
{
    Struct_CAN_Tx_Bus *bus;
    uint16_t bus_index;
    uint16_t send_count;

    for (bus_index = 0; bus_index < CAN_BUS_COUNT; bus_index++)
    {
        bus = &Can_TxBuses[bus_index];
        if (bus->queue == NULL)
        {
            continue;
        }

        for (send_count = 0; send_count < CAN_TX_QUEUE_DEPTH; send_count++)
        {
            if (bus->pending_valid == 0)
            {
                if (osMessageQueueGet(bus->queue,
                                      &bus->pending,
                                      NULL,
                                      0) != osOK)
                {
                    break;
                }
                bus->pending_valid = 1;
            }

            if (!BSP_CAN_SendMsg(&bus->pending))
            {
                break;
            }
            bus->pending_valid = 0;
        }
    }
}

/**
 * @brief 轮询周期槽，并发送每个槽尚未处理的最新版本。
 * @return true 所有待处理槽均已成功写入硬件 FIFO，或没有待处理数据；
 *         false 至少有一个周期槽因同总线事件待发或硬件忙而推迟发送。
 * @details 同总线事件全部投递后才发送周期数据。槽检查、非阻塞 HAL 投递和确认
 *          在同一个短临界区内完成，确保 Stop 返回后不会再发送旧快照。
 */
bool BSP_CAN_SendPer(void)
{
    Struct_CAN_Tx_Slot *slot;
    Struct_CAN_Tx_Bus *bus;
    uint32_t primask;
    uint16_t start_index = Can_TxRoundRobin;
    uint16_t scan_count;
    uint16_t slot_index;
    int16_t bus_index;
    bool result = true;

    for (scan_count = 0; scan_count < CAN_TX_SLOT_COUNT; scan_count++)
    {
        slot_index = (uint16_t)((start_index + scan_count) % CAN_TX_SLOT_COUNT);

        primask = BSP_CAN_EnterCritical();
        slot = &Can_TxSlots[slot_index];
        if (slot->is_used != 0 && slot->version != slot->sent_version)
        {
            bus_index = BSP_CAN_GetBusIndex(slot->message.hfdcan);
            bus = &Can_TxBuses[bus_index];
            if (bus->pending_valid != 0 ||
                (bus->queue != NULL && osMessageQueueGetCount(bus->queue) != 0))
            {
                result = false;
            }
            else if (BSP_CAN_SendMsg(&slot->message))
            {
                slot->sent_version = slot->version;
                Can_TxRoundRobin =
                    (uint16_t)((slot_index + 1) % CAN_TX_SLOT_COUNT);
            }
            else
            {
                result = false;
            }
        }
        BSP_CAN_ExitCritical(primask);
    }

    return result;
}
