/**
 * @file    EL05.h
 * @brief   灵足 EL05 默认私有协议驱动，CAN 2.0 扩展帧，1 Mbps。
 * @author  zzm
 * @note    依据 EL05使用说明书2600428，第 29~44 页。
 */
#ifndef __EL05_H
#define __EL05_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Private macros ------------------------------------------------------------*/
#define EL05_MIN_ANGLE (-12.57f)
#define EL05_MAX_ANGLE (12.57f)
#define EL05_MIN_SPEED (-50.0f)
#define EL05_MAX_SPEED (50.0f)
#define EL05_MIN_TORQUE (-6.0f)
#define EL05_MAX_TORQUE (6.0f)
#define EL05_MAX_CURRENT (11.0f)
#define EL05_MAX_KP (500.0f)
#define EL05_MAX_KD (5.0f)

/* Private types -------------------------------------------------------------*/
/** @brief 电机控制模式，对应 run_mode 参数（0x7005）；切换前需停止电机。 */
typedef enum
{
    EL05_MODE_MOTION = 0,       /**< 运控模式：同时给定位置、速度、Kp、Kd 和前馈力矩。 */
    EL05_MODE_POSITION_PP = 1,  /**< 轮廓位置模式：给定目标位置，电机按配置的速度、加速度规划运动。 */
    EL05_MODE_SPEED = 2,        /**< 速度模式：给定目标角速度（rad/s），可配置加速度和电流限制。 */
    EL05_MODE_CURRENT = 3,      /**< 电流模式：给定目标 Iq 电流（A），由电机内部电流环控制。 */
    EL05_MODE_POSITION_CSP = 5  /**< 周期同步位置模式：主控周期更新目标位置（rad），可配置速度限制。 */
} EL05_Mode_t;

typedef enum
{
    EL05_STATE_RESET = 0,
    EL05_STATE_CALIBRATION = 1,
    EL05_STATE_RUNNING = 2
} EL05_State_t;

typedef enum
{
    EL05_PARAM_RUN_MODE = 0x7005,
    EL05_PARAM_IQ_REF = 0x7006,
    EL05_PARAM_SPEED_REF = 0x700A,
    EL05_PARAM_LIMIT_TORQUE = 0x700B,
    EL05_PARAM_CURRENT_KP = 0x7010,
    EL05_PARAM_CURRENT_KI = 0x7011,
    EL05_PARAM_CURRENT_FILTER = 0x7014,
    EL05_PARAM_POSITION_REF = 0x7016,
    EL05_PARAM_LIMIT_SPEED = 0x7017,
    EL05_PARAM_LIMIT_CURRENT = 0x7018,
    EL05_PARAM_MECH_POSITION = 0x7019,
    EL05_PARAM_IQ_FILTERED = 0x701A,
    EL05_PARAM_MECH_SPEED = 0x701B,
    EL05_PARAM_BUS_VOLTAGE = 0x701C,
    EL05_PARAM_POSITION_KP = 0x701E,
    EL05_PARAM_SPEED_KP = 0x701F,
    EL05_PARAM_SPEED_KI = 0x7020,
    EL05_PARAM_SPEED_FILTER = 0x7021,
    EL05_PARAM_SPEED_ACCELERATION = 0x7022,
    EL05_PARAM_PP_SPEED = 0x7024,
    EL05_PARAM_PP_ACCELERATION = 0x7025,
    EL05_PARAM_REPORT_INTERVAL = 0x7026,
    EL05_PARAM_CAN_TIMEOUT = 0x7028,
    EL05_PARAM_ZERO_STATE = 0x7029,
    EL05_PARAM_ZERO_OFFSET = 0x702B
} EL05_Parameter_t;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t id;
    uint8_t host_id;
    bool initialized;
    bool command_enabled;          /**< 使能命令已入队，不代表电机已使能。 */
    bool stop_pending;             /**< 停止帧尚未入队，需重试停止后才允许使能。 */
    EL05_Mode_t mode;              /**< 本机最近成功提交的控制模式。 */
    volatile bool enabled;        /**< 由反馈帧的运行状态更新。 */
    volatile EL05_State_t state;
    volatile uint8_t feedback_fault;
    volatile uint32_t fault;
    volatile uint32_t warning;
    volatile float angle;         /**< rad，反馈按约 8π 周期回绕。 */
    volatile float speed;         /**< rad/s。 */
    volatile float torque;        /**< Nm，不能当成电流。 */
    volatile float temperature;   /**< 摄氏度。 */
    volatile float current;       /**< A，仅由 0x701A 参数读取更新。 */
    volatile float bus_voltage;   /**< V，仅由 0x701C 参数读取更新。 */
    volatile uint32_t last_feedback_ms; /**< 最近一次位置、速度、力矩反馈的 HAL 时间戳（ms），用于超时判断。 */
    volatile uint32_t feedback_count;   /**< 位置、速度、力矩反馈帧累计接收次数，每收到一帧加 1。 */
    volatile uint16_t parameter_index;  /**< 最近一次参数读取应答的参数编号，如 0x701C 表示母线电压。 */
    volatile uint32_t parameter_value;  /**< 最近一次参数读取应答的原始 32 位数据；float 参数需按位解释，不能强制数值转换。 */
    volatile uint8_t parameter_status;  /**< 最近一次参数读取的结果：0 成功，1 失败。 */
    volatile uint32_t parameter_count;  /**< 参数读取应答累计接收次数，成功或失败均加 1，用于识别新应答。 */
} EL05_t;

/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 绑定电机并注册反馈；对象应静态分配、零初始化，且在回调存续期内有效。
 * @note 在 BSP_CAN_ConfigInit 后调用一次，不发送使能或配置命令。
 *       id 使用 1~255，host_id 使用 0~255（官方示例默认 0xFF）。
 *       控制接口由同一任务串行调用；中断只更新反馈，读取多个反馈字段时需快照保护。
 *       默认 mode 为运控；若电机已配置其他模式，应先显式 SetMode。
 */
bool EL05_Init(EL05_t *motor, uint8_t id, uint8_t host_id,
               FDCAN_HandleTypeDef *hfdcan);

/** @brief 本机禁用时先排队停止再切换模式；成功仅表示两帧已入发送队列。 */
bool EL05_SetMode(EL05_t *motor, EL05_Mode_t mode);
bool EL05_Enable(EL05_t *motor);

/** @brief 关闭本机控制发布并取消周期槽；若停止帧入队失败，调用者需重试。 */
bool EL05_Disable(EL05_t *motor);
bool EL05_ClearError(EL05_t *motor);

/** @brief 本机禁用时先排队停止再标零，仅适用于运控/CSP；PP 模式不支持。 */
bool EL05_SetZeroAngle(EL05_t *motor);

/** @brief 运控五参数：rad、rad/s、Kp、Kd、Nm；有限值按协议范围限幅。 */
bool EL05_SetMotion(EL05_t *motor, float angle, float speed,
                    float kp, float kd, float torque);
bool EL05_SetCurrent(EL05_t *motor, float current);
bool EL05_SetSpeed(EL05_t *motor, float speed);

/** @brief PP/CSP 目标角度(rad)，支持多圈浮点目标，不使用运控帧的 ±12.57 限幅。 */
bool EL05_SetAngle(EL05_t *motor, float angle);

/**
 * @brief 提交单个参数读取请求（通信类型 17），不等待电机应答。
 * @param motor 已初始化的电机实例。
 * @param index 参数编号，如 EL05_PARAM_BUS_VOLTAGE（0x701C）。
 * @return true 请求已入队；false 实例无效或发送队列不可用、已满。
 * @note 收到应答后才更新 parameter_index/value/status/count；仅保存最近一条应答。
 *       读取电流或母线电压成功且数值有效时，还会更新 current 或 bus_voltage。
 */
bool EL05_ReadParameter(EL05_t *motor, uint16_t index);

/**
 * @brief 按原始位值写入单个参数（通信类型 18），配置按 FIFO 排队。
 * @param motor 已初始化的电机实例。
 * @param index 参数编号，参数类型、取值范围和写权限由调用者按手册保证。
 * @param value 参数原始位值；uint8/uint16 参数的未使用高位须清零，浮点参数建议使用 WriteFloatParameter。
 * @return true 所需命令已入队；false 实例无效、操作被拒绝或入队失败。
 * @note run_mode（0x7005）转交 SetMode，先排队停止再切换模式。
 *       禁止通过本接口写入电流、速度、位置目标，请使用对应的 SetCurrent/SetSpeed/SetAngle。
 *       普通写入掉电丢失；返回 true 仅表示 BSP 接受命令，不代表电机已执行或参数已保存。
 */
bool EL05_WriteParameter(EL05_t *motor, uint16_t index, uint32_t value);

/**
 * @brief 将 float 按 IEEE 754 原始位模式编码，再通过 WriteParameter 写入。
 * @param motor 已初始化的电机实例。
 * @param index 浮点参数编号，如 EL05_PARAM_LIMIT_CURRENT（0x7018）。
 * @param value 参数实际浮点值，单位和允许范围由 index 决定。
 * @return true 写入命令已入队；false 数值为 NaN/Inf、操作被拒绝或入队失败。
 * @note 本接口不进行限幅，调用者需保证参数为可写 float 类型且取值符合手册。
 *       run_mode、上报间隔、CAN 超时和零点标志等整数参数应使用 WriteParameter。
 */
bool EL05_WriteFloatParameter(EL05_t *motor, uint16_t index, float value);

/**
 * @brief 提交主动上报开关命令（通信类型 24）。
 * @param motor 已初始化的电机实例。
 * @param enabled true 开启主动上报；false 关闭主动上报。
 * @return true 命令已入队；false 实例无效或发送队列不可用、已满。
 * @note 电机默认关闭主动上报，默认上报间隔为 10 ms；本函数不修改上报间隔。
 *       可通过 WriteParameter 写入 EL05_PARAM_REPORT_INTERVAL：1 表示 10 ms，每加 1 增加 5 ms。
 *       上报反馈由接收回调更新角度、速度、力矩、温度和运行状态，不会使能电机。
 */
bool EL05_SetAutoReport(EL05_t *motor, bool enabled);

/** @brief 判断最近一次位置/速度/力矩反馈是否在 timeout_ms 内，未收到反馈返回 false。 */
bool EL05_IsOnline(const EL05_t *motor, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
