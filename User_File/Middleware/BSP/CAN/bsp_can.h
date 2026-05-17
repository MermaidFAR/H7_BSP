/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   板级支持包：CAN总线驱动 (基于 STM32H7 FDCAN)
 * @details 包含CAN过滤器的配置、中断接收回调的分发管理以及线程安全的发送函数。
 * @author  zzm
 * @date    2025-12-24
 * @version v1.1
 ******************************************************************************
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <string.h>
#include <stdbool.h>
#include "fdcan.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  FDCAN 最大负载长度 (Classic CAN模式)
 * @note 更改负载长度的，默认没有FDCAN电机,一切配置都是默认普通CAN，
 * @note 如果需要动态长度请严格思考自己是否需要该功能，缩短帧长度控制同时可能提高帧频率增加总线负载
 * @note 可能降低代码稳定性
 */
#define FDCAN_MAX_PAYLOAD  8

/**
 * @brief  最大支持的 CAN 回调函数数量
 * @note   如果注册失败，请增大此值
 */
#define MAX_CAN_CALLBACKS 16

 /**
 * @brief  CAN 接收回调函数指针类型定义
 * @param  hfdcan FDCAN 句柄
 * @param  id     接收到的 CAN ID
 * @param  data   接收到的数据指针
 * @param  len    接收到的数据长度
 */
typedef void (*CAN_RxCallback_t)(FDCAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data, uint32_t len);

/**
 * @brief  CAN 回调注册表条目结构体
 */
typedef struct {
    uint32_t ID;              /*!< 监听的 CAN ID */
    CAN_RxCallback_t func;    /*!< 对应的处理回调函数 */
    uint8_t is_used;          /*!< 标记该条目是否已被占用 (1:占用, 0:空闲) */
} CAN_CallbackEntry_t;

/**
 * @brief  CAN 发送消息结构体
 */
typedef struct 
{
    FDCAN_HandleTypeDef* hfdcan;
    uint32_t id;
    uint8_t data[FDCAN_MAX_PAYLOAD];
    uint32_t len;
} Struct_CAN_Tx_Msg;

/**
 * @brief  初始化 CAN 总线硬件及过滤器
 * @note   配置为接收所有标准帧 ID，并初始化互斥锁
 */
void BSP_CAN_ConfigInit(void);

/**
 * @brief  注册 CAN 接收回调函数
 * @param  can_id    需要监听的 CAN ID
 * @param  pCallback 回调函数指针
 */
bool BSP_CAN_RegisterCallback(uint32_t can_id, CAN_RxCallback_t pCallback);

/**
 * @brief  发送 CAN 消息 (线程安全)
 * @param  hfdcan FDCAN 句柄 (&hfdcan1, &hfdcan2, &hfdcan3)
 * @param  id     目标 CAN ID (标准帧)
 * @param  data   要发送的数据指针
 * @param  len    数据长度 (通常为8)
 * @return true: 发送成功 (写入 FIFO), false: 发送失败 (超时或 FIFO 满)
 */
bool BSP_CAN_SendMsg(FDCAN_HandleTypeDef* hfdcan, uint32_t id, uint8_t* data, uint32_t len);

/**
 * @brief  批量发送预设的 CAN 消息
 * @return true: 全部发送成功, false: 至少有一条发送失败
 * @note   此函数会尝试发送 Tx_Msg_Buffer 中的三条消息，并返回整体结果。
 *         适用于需要同时更新多条消息的场景，减少调用次数。
 */
bool BSP_CAN_SendPer(void);

#ifdef __cplusplus
}
#endif

#endif // !BSP_CAN_H
