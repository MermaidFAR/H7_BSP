#if BALANCE

#ifndef __BALANCE_H
#define __BALANCE_H

#include "QD4310.h"
#include "alg_pid.h"
#include "alg_fsm.h"

#define Left_ID     1
#define Right_ID    2

enum Enum_Balance_Status
{
    BALANCE_Status_DISABLE = 0,
    BALANCE_Status_READY,
    BALANCE_Status_LEFT_ERROR,
    BALANCE_Status_RIGHT_ERROR,
};

/**
 * @brief 平衡车控制对象
 *
 * 上电自动进入平衡。纵向 LQR 输出共同电流，轮速差 PID 输出差动电流。
 * 位置误差更新暂被注释，位置输出项仍启用，当前尚不形成正常的位置闭环。
 * 每轮最终限流±1.5 A。
 */
typedef struct
{
    QD4310_t Left_Motor;  /* 左轮 QD4310 电机对象，保存使能状态和反馈值。 */
    QD4310_t Right_Motor; /* 右轮 QD4310 电机对象，保存使能状态和反馈值。 */
    Class_FSM<4> Balance_FSM; /* 平衡车状态机：失能、就绪、左轮错误、右轮错误。 */

    /* 保留原对象名；直立/速度 PID 不再参与控制，转向 PID 改用轮速差反馈。 */
    Class_PID Turn_PID;  /* 转向环：目标转向角速度误差 -> 左右轮差动量。 */

    /* 外部目标值。 */
    float Target_Angle;      /* 已标定直立零点，rad；LQR 不覆盖。 */
    float Target_Speed;      /* 目标前进速度，单位 rpm；正负表示前进或后退。 */
    float Target_Turn_Angle; /* 目标转向角，单位 rad/s；正负表示左右转。 */
    
    float Speed_Feedback;    /* 原始合成轮速rpm；LQR使用修正后的Speed_M_S。 */
    /* 四状态增益G=-K；四项输出均启用，但位置误差更新暂被注释。 */
    float LQR_K[4];          /* 输入单位依次 rad、rad/s、m/s、m，输出共同电流 A。 */
    float LQR_Out;           /* 限幅前的每轮共同电流，正值对应物理前进。 */
    float Angle_Error;       /* Roll 减 Target_Angle，rad。 */
    float Angle_Rate;        /* BMI088 车体 X 轴角速度，rad/s。 */
    float Angle_Rate_Deg_S;  /* 角速度显示值，deg/s。 */
    float Speed_M_S;         /* 合成轮速加车体绕轴转动补偿后的车速，m/s。 */
    float Position;          /* 累计轮角和车体转角得到的位置，m。 */
    float Position_Target;   /* 停车点目标，m；行驶时跟随当前位置。 */
    float Position_Error;    /* Position - Position_Target，m。 */

    /* VOFA 遥测快照：均为本次 Balance_Control() 实际使用或计算得到的数据。 */
    float Angle_Target;      /* 直立工作点，等于标定零点，单位 rad。 */
    float Angle_Feedback;    /* 直立环实际反馈 Roll 角，单位 rad。 */
    float Turn_Feedback;     /* 右轮减左轮物理速度，经轮距换算的转向角速度，rad/s。 */
    float Turn_PID_Out;      /* 转向环原始 PID 输出，单位 A。 */
    float Left_Current_Command;  /* 左轮限幅后电流指令，单位 A，保留电机原始方向。 */
    float Right_Current_Command; /* 右轮限幅后电流指令，单位 A，保留电机原始方向。 */

    /* VOFA 显示快照：仅用于单位换算，不参与控制计算。 */
    float Angle_Target_Deg;       /* 最终直立目标角，单位 deg。 */
    float Angle_Feedback_Deg;     /* 直立实际 Roll 角，单位 deg。 */
    float Target_Turn_Deg_S;      /* 转向目标角速度，单位 deg/s。 */
    float Turn_Feedback_Deg_S;    /* 转向实际角速度，单位 deg/s。 */
} QDBalance_t;

extern QDBalance_t Balance;

void Balance_Init(void);

/** @brief 1 ms 控制入口：采集反馈，依次调用 BalanceLQR/BalancePID，混合并发送电流。 */
void Balance_Control(void);

/** @brief 由 Balance_Control 调用，使用已更新的状态计算 LQR_Out（A），不发送电流。 */
void BalanceLQR(void);

/** @brief 由 Balance_Control 调用，使用 Turn_Feedback 计算转向 PID 输出（A），不发送电流。 */
void BalancePID(void);
void VOFA_Init(void);
void VOFA_Control(void);

/*
 * USART10下行按#分帧，支持分包与多条连发；USB仅由TransportTask发送遥测。
 * speed:目标rpm#；turn:目标rad/s#（内部由左右轮速差闭环，不使用偏航角）。
 * VOFA两个MaterialJoystick分别绑定以下文本命令（参数顺序均为X、Y）：
 * 前后摇杆：speed_xy:%.3f,%.3f#，只取Y，单位rpm；上推前进、下推后退。
 * 转向摇杆：turn_xy:%.3f,%.3f#，只取-X，单位rad/s；右推右转、左推左转。
 * 两摇杆均开启自动回弹与数值更新发送。前后回中只归零速度，转向回中只归零
 * 轮速差对应的转向目标，不锁偏航角，不覆盖另一摇杆的目标。
 * 起步调试可设前后范围-15..15、步进1；转向范围-0.5..0.5、步进0.01。
 * 范围是上位机手感设置，非新增固件限速；0目标不等于电机失能。
 * lqr_angle/lqr_rate/lqr_speed/lqr_pos:数值#，分别设置 LQR_K[0..3]；
 * 当前lqr_pos只保存增益值，不会重新启用被注释的位置反馈。
 * turn_kp/turn_ki/turn_kd:数值# 调整转向PID。旧 angle_*、speed_* 增益键已移除，
 * 避免将旧PID单位的参数误写入LQR。在线改增益不清位置，也不改标定零点。
 * speed:0# 设置零速目标；停车点仍记录，位置误差更新仍被注释。上电自动启动不变。
 * 17路 JustFloat、100 Hz，当前通道含义：
 * I0直立零点(deg), I1实际Roll(deg), I2共同电流(A), I3目标rpm, I4原始合成rpm,
 * I5停车点目标(m), I6当前位置(m), I7转向目标(deg/s), I8轮速差换算反馈(deg/s),
 * I9差动PID输出(A), I10修正车速(m/s), I11位置误差(m), I12前后角速度(deg/s),
 * I13/I14左右最终指令(A), I15/I16左右反馈电流(A)。
 */

#endif
#endif
