#ifndef SPIN_MODE_H
#define SPIN_MODE_H

#include "alg_fsm.h"

typedef enum
{
    /** @brief 云台跟随底盘：上层保持相对朝向，本模块不补偿底盘转速。 */
    SpinMode_GIMBAL_FOLLOW = 0,
    /** @brief 云台锁定：上层保持世界 Yaw，本模块补偿底盘转速；非零 vw 即小陀螺。 */
    SpinMode_GIMBAL_LOCK,
    /** @brief 底盘跟随云台：上层生成车头对齐所需的 vw，本模块补偿底盘转速。 */
    SpinMode_CHASSIS_FOLLOW
} SpinMode_e;

/** @brief 最近一次计算结果；速度均为指令，未经过执行器反馈修正或限幅。 */
typedef struct
{
    /** @brief m/s；底盘坐标系 X 轴平移速度，向前为正。 */
    float x;
    /** @brief m/s；底盘坐标系 Y 轴平移速度，向左为正。 */
    float y;
    /** @brief rad/s；底盘 Yaw 角速度，逆时针为正；三个模式均原样输出 vw。 */
    float w;
    /**
     * @brief rad/s；云台相对底盘的角速度前馈，与 w 使用相同正方向。
     * @note 云台跟随模式为 0；云台锁定、底盘跟随模式为 -w。
     *       串级 PID 接法：速度环目标 = 角度环输出(rad/s) + forward。
     *       此接法要求速度环反馈为云台相对底盘的角速度；若直接闭环 IMU
     *       世界角速度，不应在世界速度目标中重复叠加此相对运动补偿。
     *       forward 不是电流/力矩前馈，不加到速度 PID 的控制输出端。
     */
    float forward; // 希望底盘要做好哦,做不好就不要用这个值了
} SpinOutput_t;

typedef struct
{
    /** @brief rad；云台与底盘正向对齐时 theta 的读数，用于扣除相对角零偏。 */
    float zero_point;
    /** @brief 初始模式；仅支持 SpinMode_e 中的三种模式。 */
    SpinMode_e mode;
} SpinConfig_t;

/**
 * @author zzm
 * @brief 云台系运动指令到车体系速度及相对速度前馈的计算接口。
 * @note 先 Init 再计算；调用方在同一上下文更新有限输入并读取结果。
 *       X 向前、Y 向左、逆时针为正；角度 rad，角速度 rad/s，平移速度 m/s。
 *       theta 减去 zero_point 后表示云台朝向减底盘朝向；已归零时 zero_point 为 0。
 *       各模式的 w 均由调用方的 vw 提供。
 *       上层负责云台角度目标和底盘跟随控制；本模块不锁存目标或生成跟随角速度。
 *       world_angle 仅供上层暂存/读取世界 Yaw，不参与四项速度输出的计算。
 *       锁定与底盘跟随使用相同的 -vw 前馈，区别在于上层生成 vw 的控制策略。
 *       forward 依据指令而非实测角速度，叠加到云台相对底盘的速度目标。
 *       模块不做单位换算、减速比换算或速度限幅，由调用方在接入控制环时处理。
 */
class SpinMode
{
  public:
    /**
     * @brief 初始化并清空输入/输出与模式状态；应在第一次计算前调用。
     * @param config 配置指针，仅在本次调用中读取；NULL 使用零偏移、云台跟随模式。
     * @return true 初始化成功；零偏非有限值或模式非法时返回 false 并保留原状态。
     */
    bool Init(const SpinConfig_t *config = NULL);

    /**
     * @brief 暂存上层云台世界 Yaw 目标，通过 Get_WorldTarget() 读取。
     * @param target_angle 单位为 rad，逆时针为正，可由上层按 IMU 姿态设置并保持。
     * @note 原样保留连续角度；不参与速度计算，不自动采集、锁存或修正目标。
     */
    void Set_WorldTarget(float target_angle);

    /**
     * @brief 更新当前云台与底盘的相对夹角，计算时扣除 config.zero_point。
     * @param measured_theta 单位为 rad，逆时针为正。measured_theta - zero_point = 云台 Yaw - 底盘 Yaw。
     * @note 可输入连续角度，计算时归一化为最短相对角；已归零时 zero_point 设为 0。
     */
    void Set_theta(float measured_theta);

    /**
     * @brief 设置运动指令；模块仅转换平移坐标系并计算输出，不执行底盘控制。
     * @param vx m/s；云台坐标系 X 轴速度，沿云台朝向为正。
     * @param vy m/s；云台坐标系 Y 轴速度，向云台左侧为正。
     * @param vw rad/s；底盘 Yaw 角速度指令，逆时针为正；输出 w = vw。
     * @note 底盘跟随时，vw 应由上层根据当前云台与底盘的夹角生成。
     *       锁定模式下 vw 可为小陀螺转速；不能将角度误差(rad)直接作为 vw。
     */
    void Set_MoveTarget(float vx, float vy, float vw);

    /** @brief 切换模式；非法值或重复设置当前模式不生效，切换不自动采集世界目标。 */
    void Set_Spin_Mode(SpinMode_e mode);

    /**
     * @brief 使用当前输入更新输出；由调用方在更新输入后主动调用，不创建定时器。
     * @note 无采样周期积分或目标锁存；相同模式及运动输入产生相同速度输出。
     */
    void TIM_Calculate_PeriodElapsedCallback();

    /** @brief 返回最近一次计算的结果副本；仅调用 Set 函数不会刷新输出。 */
    SpinOutput_t Get_Output() const;
    /** @brief 返回当前三态模式。 */
    SpinMode_e Get_Spin_Mode() const;
    /** @brief rad；返回保存的世界 Yaw 目标，未归一化，也不是当前姿态反馈。 */
    float Get_WorldTarget() const;

  protected:
    Class_FSM<3> SpinMode_FSM;
    SpinOutput_t Output = {};
    float zero_point = 0.0f;  /**< rad；初始化时归一化的相对角零偏。 */
    float world_angle = 0.0f; /**< rad；上层保持的连续世界 Yaw 目标。 */
    float theta = 0.0f;       /**< rad；尚未扣除 zero_point 的相对角反馈。 */
    float x_target = 0.0f;    /**< m/s；云台坐标系前向速度指令。 */
    float y_target = 0.0f;    /**< m/s；云台坐标系左向速度指令。 */
    float w_target = 0.0f;    /**< rad/s；底盘逆时针角速度指令。 */
};

#endif
