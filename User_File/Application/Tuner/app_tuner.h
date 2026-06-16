/**
 * @file app_tuner.h
 * @author zzm
 * @brief PID 在线调参模块 (appTuner)
 * @version 2.1
 * @date 2026-06-06 2.0 多实例支持, Flash 异步任务, 独立写入隔离
 * @date 2026-06-16 2.1 Flash 版本机制: version 字段替代 NaN 检测, 代码默认值变更自动覆盖旧数据
 *
 * @details
 * 数据分区:
 *   - Flash 持久化: K_P/K_I/K_D/K_F/Out_Max/I_Out_Max/变速积分/积分分离/D_First/D_T
 *   - RAM 实时注入: Target (EricTool 可写) / Now (电机等外部写入)
 *   - 只读遥测:    Out/I_Out (PID 计算输出, 仅 TX)
 *
 * 工作流:
 *   - EricTool RX → 改 active 实例的 RAM 缓冲 → 立即推入 PID → 标记 dirty
 *   - Flash Task (独立 RTOS 任务) → Erase Sector → Write 全量 → 清 dirty
 *   - 上电 Init → 逐个读 Flash → 版本校验 → 不匹配则从 PID 拉默认值写入 Flash → 匹配则推入 PID
 *   - App_Tuner_TX 只调 Set_Data, EricTool_Send_Telemetry 由 StatusTask 调
 *
 * 版本机制 (FLASH_PID_PARAMS_VERSION):
 *   - 每个 Flash Sector 首位存 version 字段
 *   - 上电读回 version 与宏对比, 不匹配则视为无效, 从 PID 对象回读当前值重新写入 Flash
 *   - 以下情况需将 FLASH_PID_PARAMS_VERSION +1:
 *     * 修改了 Class_PID::Init() 的默认参数
 *     * 增删或重排 Flash_PID_Params_t 的字段
 *   - 不需要改 version 的情况: 只改逻辑代码不变默认值、增删非持久化的 RAM 字段
 *
 * @note  每个 PID 实例独占一个 Flash Sector (4KB)
 * @note  Now 不被 EricTool 写入, 由电机控制等外部代码回填
 * @note  D_T 可变不可调: 代码设, 不在 EricTool 字典中
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#ifndef APP_TUNER_H
#define APP_TUNER_H

#include "alg_pid.h"
#include "bsp_usb.h"
#include "bsp_w25q64jv.h"
#include "dvc_erictool.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 最大 PID 实例数 */
#define PID_TUNER_MAX 4

/**
 * @brief Flash 参数结构版本号
 *
 * @note  改 Class_PID::Init() 默认参数 或 改 Flash_PID_Params_t 字段 → +1
 * @note  上电初版 version 不匹配 → 自动从 PID 回读默认值写入 Flash
 * @note  不需改 version: 只改逻辑代码不变默认值、增删非持久化字段
 */
#define FLASH_PID_PARAMS_VERSION 2

/**
 * @brief Flash 持久化的 PID 参数
 *
 * @note  version 字段位于首位, 上电时与 FLASH_PID_PARAMS_VERSION 对比
 * @note  不匹配 → 视为无效数据 → 从 PID 对象回读并重写整个 Sector
 * @note  一个 Sector (4KB) 可容纳约 100 个此结构体
 * @note  多实例各占独立 Sector, 无需 RMW
 */
typedef struct {
    uint32_t version;               ///< 结构版本 (FLASH_PID_PARAMS_VERSION)
    float K_P;                      ///< 比例系数
    float K_I;                      ///< 积分系数
    float K_D;                      ///< 微分系数
    float K_F;                      ///< 前馈系数
    float I_Out_Max;                ///< 积分输出限幅, 0 为不限制
    float Out_Max;                  ///< 输出限幅, 0 为不限制
    float I_Variable_Speed_A;       ///< 变速积分: 定速内段阈值, 0 为关闭
    float I_Variable_Speed_B;       ///< 变速积分: 变速区间上限
    float I_Separate_Threshold;     ///< 积分分离阈值, 0 为关闭
    Enum_PID_D_First D_First;       ///< 微分先行使能
    float D_T;                      ///< PID 计算周期 (s), 代码设不可调
} Flash_PID_Params_t;

/**
 * @brief 单个 PID Tuner 实例
 */
typedef struct {
    Flash_PID_Params_t flash;       ///< Flash 参数镜像
    float Target;                   ///< 设定值 (EricTool 可写, 不持久)
    float Now;                      ///< 当前值 (外部写入, 不持久, 不在 RX 字典)
    float Out;                      ///< PID 输出 (只读遥测)
    float I_Out;                    ///< 积分项输出 (只读遥测)
    Class_PID *pid;                 ///< 绑定的 PID 对象指针
    uint32_t flash_addr;            ///< Flash 存储基地址 (需 Sector 对齐)
    bool dirty;                     ///< 待提交到 Flash 标记
} PID_Tuner_Instance_t;

/**
 * @brief PID Tuner 管理器
 *
 * @note  active_index 决定当前 EricTool RX/TX 操作的目标实例
 */
typedef struct {
    PID_Tuner_Instance_t instance[PID_TUNER_MAX];   ///< 实例数组
    uint8_t active_index;                           ///< 当前活跃实例索引
    uint8_t count;                                  ///< 已注册实例数
} PID_Tuner_Manager_t;

extern PID_Tuner_Manager_t PID_Tuner;

/**
 * @brief 注册一个 PID 实例
 *
 * @param index      实例索引 (0 ~ PID_TUNER_MAX-1)
 * @param pid        已初始化 D_T/Dead_Zone 的 Class_PID 实例指针
 * @param flash_addr W25Q64JV 地址, 需 4KB Sector 对齐
 *
 * @note  必须在 App_Tuner_Init() 之前调用
 */
void App_Tuner_Register_PID(uint8_t index, Class_PID *pid, uint32_t flash_addr);

/**
 * @brief 模块初始化
 *
 * @note  遍历所有已注册实例, 逐个读 Flash 做版本校验
 * @note  version != FLASH_PID_PARAMS_VERSION → 视为无效 → 从 PID 回读默认值写入 Flash (Pull_PID_To_Flash)
 * @note  version 匹配 → 从 Flash 恢复参数到 PID (Push_Flash_Params_To_PID)
 */
void App_Tuner_Init(void);

/**
 * @brief 遥测指针注册 (不调用 EricTool_Send_Telemetry)
 *
 * @note  只调 EricTool_USB.Set_Data(), 由 StatusTask 周期性调用 EricTool_Send_Telemetry()
 * @note  使用 active_index 指向当前活跃实例
 * @note  通道: K_P/K_I/K_D/K_F/I_Out_Max/Out_Max/Aspeed/Bspeed/SepThr/Target/Now/Out/I_Out (13 通道)
 */
void App_Tuner_TX(void);

/**
 * @brief EricTool 下行指令处理
 *
 * @note  Flash 字段 (case 0~9): 改 active 实例 RAM → 推入 PID → 标 dirty
 * @note  Target    (case 10):  改 active 实例 Target → 推入 PID, 不标 dirty
 * @note  Now 不在 RX 字典中, 由外部代码直接写 PID_Tuner.instance[i].Now
 */
void App_Tuner_RX(void);

/**
 * @brief Flash 异步提交任务 (独立 RTOS 任务入口)
 *
 * @note  遍历所有实例, 对 dirty 的实例执行: Set_Write_Enable → Erase → Write → 清 dirty
 * @note  建议 100ms 周期, 低优先级
 * @note  Erase 耗时 ~400ms, 期间 osDelay 让出 CPU
 */
void App_Tuner_Flash_Task(void);

#ifdef __cplusplus
}
#endif

#endif
