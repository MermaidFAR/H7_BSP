/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   板级支持包：CAN 总线驱动
 * @details
 * 接收方向使用 (FDCAN 句柄, CAN ID) 作为回调注册键。
 * 发送方向保留两条通道：
 * - CAN_Tx_Submit：插入队列，每次提交都按顺序发送。
 * - CAN_Tx_Perform：周期缓冲，同一 (FDCAN, CAN ID) 只保留最新数据。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.1
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
 * @param id 接收到的标准帧 ID。
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
    uint32_t id;                     /*!< 标准帧 ID，取值范围为 0x000~0x7FF。 */
    uint8_t data[FDCAN_MAX_PAYLOAD]; /*!< 待发送的数据。 */
    uint8_t len;                     /*!< 有效数据长度，取值范围为 1~8。 */
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
 * @return true 注册成功；false 注册表已满或相同键已经注册。
 * @note 注册键是 (hfdcan, can_id)，因此不同总线可以注册相同 ID。
 * @note callback 在 FDCAN 接收中断中执行，不应进行阻塞操作。
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id,
                              FDCAN_HandleTypeDef *hfdcan,
                              CAN_RxCallback_t callback,
                              void *context);

/**
 * @brief 把一帧消息插入发送队列。
 * @param tx_msg 要插入队列的完整 CAN 消息。
 * @return true 已成功复制到队列；false 参数无效、队列未创建或队列已满。
 * @note 每次提交都会按队列顺序处理，适合使能、失能、复位、回零等命令。
 * @note 函数会复制消息内容，返回后调用者可以继续修改或释放原变量。
 */
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *tx_msg);

/**
 * @brief 更新周期发送缓冲。
 * @param tx_msg 要发布的最新周期消息。
 * @return true 已更新或分配周期槽；false 参数无效或周期槽已满。
 * @note BSP 自动按 (hfdcan, id) 查找或分配槽，不需要设备层保存槽号。
 * @note 相同键只保留最新数据；不同总线或不同 ID 使用不同槽。
 * @note 本函数只更新内存中的周期槽，不直接调用 HAL 发送。
 */
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *tx_msg);

/**
 * @brief 按先进先出顺序处理插入发送队列。
 * @note 通常由 CAN 发送任务周期调用。
 */
void BSP_CAN_SendAsync(void);

/**
 * @brief 周期检查所有发送槽，并发送尚未处理的最新数据。
 * @return true 所有待处理槽均已写入硬件 FIFO，或当前没有待处理数据；
 *         false 至少有一个待处理槽暂时未能写入硬件 FIFO。
 * @note 通常由 CAN 发送任务周期调用。
 */
bool BSP_CAN_SendPer(void);

#ifdef __cplusplus
}
#endif

#endif
