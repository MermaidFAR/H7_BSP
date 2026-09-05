/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   板级支持包：CAN 总线驱动
 * @details
 * 接收方向使用总线、帧类型和 ID 掩码匹配回调。
 * 发送方向保留两条通道：
 * - CAN_Tx_Submit：每条总线独立的插入队列，同总线按顺序发送。
 * - CAN_Tx_Perform：周期缓冲，同一总线、帧类型和发送键只保留最新数据。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.2
 ******************************************************************************
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Classic CAN 单帧允许的最大数据长度。 */
#define FDCAN_MAX_PAYLOAD (8)

/**
 * @brief CAN 接收回调类型。
 * @param hfdcan 实际收到该帧的 FDCAN 句柄。
 * @param id 接收到的标准帧或扩展帧 ID。
 * @param data 接收数据，只在本次回调执行期间有效。
 * @param len 接收数据长度。
 * @param context 注册时保存的设备实例或用户数据。
 * @note 回调运行在 FDCAN 接收中断中，不应阻塞。
 */
typedef void (*CAN_RxCallback_t)(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);

/**
 * @brief 一帧完整的 Classic CAN 发送数据。
 * @note 队列和周期槽都会复制结构体内容，调用者可以使用局部变量。
 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;     /*!< 目标 FDCAN 总线句柄。 */
    uint32_t id;                     /*!< 标准帧 11 位或扩展帧 29 位 ID。 */
    uint8_t data[FDCAN_MAX_PAYLOAD]; /*!< 待发送的数据。 */
    uint8_t len;                     /*!< 有效数据长度，取值范围为 1~8。 */
    uint32_t id_type;                /*!< FDCAN_STANDARD_ID（零默认）或 FDCAN_EXTENDED_ID。 */
    uint32_t slot_key;               /*!< 周期逻辑键；零表示直接以 id 为键。 */
} Struct_CAN_Tx_Msg;

/**
 * @brief 初始化三条 FDCAN 总线、插入队列和周期发送槽。
 * @note 应在 RTOS 内核初始化完成后、创建 CAN 发送任务前调用一次。
 */
void BSP_CAN_ConfigInit(void);

/**
 * @brief 注册一个接收回调。
 * @param can_id 要监听的标准帧 ID。
 * @param hfdcan 要监听的 FDCAN 总线句柄。
 * @param callback 收到匹配报文时调用的函数。
 * @param context 回调对应的设备实例或用户数据，BSP 不解析其内容。
 * @return true 注册成功；false 参数无效、注册表已满或匹配范围已经注册。
 * @note 注册键是 (hfdcan, can_id)，因此不同总线可以注册相同 ID。
 * @note callback 在 FDCAN 接收中断中执行，不应进行阻塞操作。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context);

/**
 * @brief 按总线、帧类型和 ID 掩码注册接收回调。
 * @param can_id 要监听的 CAN ID，掩码以外的位不参与匹配。
 * @param can_mask ID 匹配掩码，位为 1 表示该位必须相同。
 * @param id_type FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID。
 * @param hfdcan 要监听的 FDCAN 总线句柄。
 * @param callback 收到匹配的 Classic CAN 数据帧时调用的函数。
 * @param context 原样传回回调的设备实例或用户数据。
 * @return true 注册成功；false 参数无效、注册表已满或匹配范围重叠。
 * @note 同一总线和帧类型的匹配范围不能重叠，回调运行在接收中断中。
 */
bool BSP_CAN_RegisterCallbackEx(uint32_t can_id,
                                uint32_t can_mask,
                                uint32_t id_type,
                                FDCAN_HandleTypeDef *hfdcan,
                                CAN_RxCallback_t callback,
                                void *context);

/**
 * @brief 把一帧消息插入发送队列。
 * @param tx_msg 要插入队列的完整 CAN 消息。
 * @return true 已成功复制到队列；false 参数无效、队列未创建或队列已满。
 * @note 同总线每次提交都会按队列顺序处理，适合使能、失能、复位、回零等命令。
 * @note 三路队列独立，每路最多排队 16 帧；某一路拥塞不阻塞其他总线。
 * @note 函数会复制消息内容，返回后调用者可以继续修改或释放原变量。
 */
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *tx_msg);

/**
 * @brief 更新周期发送缓冲。
 * @param tx_msg 要发布的最新周期消息。
 * @return true 已更新或分配周期槽；false 参数无效或周期槽已满。
 * @note BSP 按总线、帧类型和键分配槽；slot_key 为零时使用帧 ID。
 * @note 非零 slot_key 使用独立逻辑键空间，允许更新时改变帧 ID。
 * @note 本函数只更新内存中的周期槽，不直接调用 HAL 发送。
 */
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *tx_msg);

/**
 * @brief 停止并释放一个非零逻辑键对应的周期发送槽。
 * @param hfdcan 周期槽所属总线。
 * @param id_type FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID。
 * @param slot_key 非零周期逻辑键。
 * @return true 已停止或槽原本不存在；false 参数无效。
 * @note 返回后该槽的旧数据不再投递；后续 Perform 可以重新分配该键。
 * @note 不能撤回已经写入硬件 FIFO 的帧，不处理插入发送队列。
 */
bool CAN_Tx_Stop(FDCAN_HandleTypeDef *hfdcan,
                 uint32_t id_type,
                 uint32_t slot_key);

/**
 * @brief 按各总线的先进先出顺序处理独立事件队列，每路每轮最多投递 16 帧。
 * @note 仅由同一个 CAN 发送任务周期调用，投递失败的队首消息下次重试。
 */
void BSP_CAN_SendAsync(void);

/**
 * @brief 周期检查所有发送槽，并发送尚未处理的最新数据。
 * @return true 所有待处理槽均已写入硬件 FIFO，或当前没有待处理数据；
 *         false 至少有一个周期槽因同总线事件待发或硬件忙而推迟发送。
 * @note 仅由同一个 CAN 发送任务周期调用；同总线事件待发时暂停该路周期发送。
 */
bool BSP_CAN_SendPer(void);

#ifdef __cplusplus
}
#endif

#endif
