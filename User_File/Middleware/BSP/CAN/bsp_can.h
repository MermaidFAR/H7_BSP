/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   板级支持包：CAN总线驱动 (基于 STM32H7 FDCAN)
 * @details 包含CAN过滤器的配置、中断接收回调的分发管理以及线程安全的发送函数。
 * @author  zzm
 * @date    2026-05-18
 * @version v2.0
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

//=====================宏定义========================

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

//=====================结构体定义========================

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

//=====================对外接口========================

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
 * @brief  批量发送预设的 CAN 消息
 * @return true: 全部发送成功, false: 至少有一条发送失败
 * @note   此函数会尝试发送 Tx_Msg_Buffer 中的三条消息，并返回整体结果。
 *         适用于需要同时更新多条消息的场景，减少调用次数。
 */
bool BSP_CAN_SendPer(void);

/**
 * @brief  异步发送 CAN 消息任务处理函数
 * @note   此函数应在一个独立的 FreeRTOS 任务中运行，持续监听发送队列并处理消息。
 */
void BSP_CAN_SendAsync(void);

/**
 * @brief  通用 CAN 发送函数 (线程安全)
 * @param  hfdcan: CAN 句柄
 * @param  id: 目标 ID (标准帧)
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @return true: 消息成功提交到发送队列, false: 提交失败 (如无效参数或总线拥堵)
 * @note   此函数会将消息封装成 Struct_CAN_Tx_Msg 结构体，并尝试提交到发送队列。
 *         适用于需要即时发送单条消息的场景，且不要求严格的实时性。
 */

bool CAN_Tx_Submit(Struct_CAN_Tx_Msg *TxHeader);

/**
 * @brief  更新预设的 CAN 消息并发送
 * @param  TxHeader: 包含 CAN 句柄、ID、数据和长度的结构体指针
 * @return true: 消息成功提交到发送队列, false: 提交失败 (如无效参数或总线拥堵)
 * @note   此函数会更新 Tx_Msg_Buffer 中对应 CAN 句柄的消息内容，并尝试发送。
 *         适用于需要频繁更新同一条消息内容的场景，减少调用 BSP_CAN_SendMsg 的次数。
 */
bool CAN_Tx_Perform(Struct_CAN_Tx_Msg *TxHeader);

//=====================淘汰接口========================

/**
 * @note 接口转为静态不对外开放, 通过BSP_CAN_SendPer批量发送预设消息，重构了发送架构，但是新架构不兼容裸机
 *       裸机开发请重启该函数。
 *       发送函数不再直接接受消息参数，而是通过全局缓冲区 Tx_Msg_Buffer 来存储待发送的消息。
 */
// bool BSP_CAN_SendMsg(FDCAN_HandleTypeDef* hfdcan, uint32_t id, uint8_t* data, uint32_t len);



#ifdef __cplusplus
}
#endif

#endif // !BSP_CAN_H
