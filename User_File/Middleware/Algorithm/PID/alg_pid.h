/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "alg_filter_iir.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行：对测量值微分以避开目标阶跃；与可选 D 低通独立。
 *
 */
enum Enum_PID_D_First
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
};

/**
 * @brief PID初始化结构体
 *
 */
typedef struct
{
    float K_P;
    float K_I;
    float K_D;
    float K_F;
    float I_Out_Max;
    float Out_Max;
    float D_T;
    float Dead_Zone;
    float I_Variable_Speed_A;
    float I_Variable_Speed_B;
    float I_Separate_Threshold;
    Enum_PID_D_First D_First;
    float D_Filter_Cutoff = 0.0f; // D 支路一阶 IIR 截止频率，Hz；0 关闭。
} PID_InitTypeDef;

/**
 * @brief Reusable, PID算法
 * @note 先 Init 后计算；参数须为有限值，D_T > 0 且与实际计算周期一致。
 *       Dead_Zone、Out_Max 为非负值；Init 保留 PID 历史，D 滤波启停规则见 Init。
 *       死区只归零有效误差，不保证总输出为零；已有积分、微分和前馈仍可输出。
 */
class Class_PID
{
  public:
    // D_Filter_Cutoff: Hz，0 关闭；非正/非有限值按关闭处理，上限由 IIR 限为采样频率的一半。
    // 首次启用或切换 D_First 时滤波状态从 0 开始；持续启用且 D_First 不变时保留滤波值。
    // 低通不对齐 Pre_Now/Pre_Error，不能替代停机恢复时的历史对齐。
    void Init(float k_p, float k_i, float k_d, float k_f = 0.0f, float i_out_max = 0.0f, float out_max = 0.0f, float d_t = 0.001f, float dead_zone = 0.0f, float i_variable_speed_a = 0.0f, float i_variable_speed_b = 0.0f, float i_separate_threshold = 0.0f, Enum_PID_D_First d_first = PID_D_First_DISABLE, float d_filter_cutoff = 0.0f);

    float Get_Target() const;
    float Get_Now() const;
    float Get_Error() const; // 最近一次计算经过死区处理后的误差

    inline float Get_Integral_Error() const;

    inline float Get_Out() const;

    inline float Get_K_P() const;

    inline float Get_K_I() const;

    inline float Get_K_D() const;

    inline float Get_K_F() const;

    inline float Get_I_Out_Max() const;

    inline float Get_Out_Max() const;

    inline float Get_I_Variable_Speed_A() const;

    inline float Get_I_Variable_Speed_B() const;

    inline float Get_I_Separate_Threshold() const;

    inline float Get_D_T() const;

    inline Enum_PID_D_First Get_D_First() const;

    inline float Get_D_Filter_Cutoff() const;

    inline void Set_K_P(float k_p);

    inline void Set_K_I(float k_i);

    inline void Set_K_D(float k_d);

    inline void Set_K_F(float k_f);

    inline void Set_I_Out_Max(float i_out_max);

    inline void Set_Out_Max(float out_max);

    inline void Set_I_Variable_Speed_A(float i_variable_speed_a);

    inline void Set_I_Variable_Speed_B(float i_variable_speed_b);

    inline void Set_I_Separate_Threshold(float i_separate_threshold);

    inline void Set_Target(float target);

    inline void Set_Now(float now);

    inline void Set_Integral_Error(float integral_error);

    void TIM_Calculate_PeriodElapsedCallback();

  protected:
    // 初始化相关常量

    // PID计时器周期, s
    float D_T;
    // 非负死区阈值：含边界的有效误差归零，区外扣除死区宽度；不修改 Target。
    float Dead_Zone;
    // 微分先行
    Enum_PID_D_First D_First = PID_D_First_DISABLE;
    // D 支路低通请求截止频率，Hz；0 关闭。滤波对象静态持有，无动态分配。
    float D_Filter_Cutoff = 0.0f;

    // 常量

    // 内部变量

    // 之前的当前值
    float Pre_Now = 0.0f;
    // 之前的目标值
    float Pre_Target = 0.0f;
    // 之前的输出值
    float Pre_Out = 0.0f;
    // 前向误差
    float Pre_Error = 0.0f;
    // 滤波差分速率，之后再乘 K_D；K_D 在线改变不混入滤波历史。
    Class_Filter_IIR_First_Order D_Filter;

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // PID的P
    float K_P = 0.0f;
    // PID的I，设为零时在下一次计算中清空积分
    float K_I = 0.0f;
    // PID的D
    float K_D = 0.0f;
    // 每周期目标增量前馈系数：K_F * (Target - Pre_Target)，不除以 D_T。
    float K_F = 0.0f;

    // 积分输出幅值上限，每次累加后限幅，0为不限制
    float I_Out_Max = 0;
    // 输出限幅, 0为不限制
    float Out_Max = 0;

    // 变速积分内段阈值 A；A、B 均为零时关闭，启用线性段时要求 0 <= A < B。
    float I_Variable_Speed_A = 0.0f;
    // 变速积分外段阈值 B（不是区间宽度）；阈值按死区外的原始误差幅值判断。
    float I_Variable_Speed_B = 0.0f;
    // 积分分离阈值，正数启用、零关闭；死区外原始误差幅值达到阈值时清空积分。
    float I_Separate_Threshold = 0.0f;

    // 目标值
    float Target = 0.0f;
    // 当前值
    float Now = 0.0f;

    // 读写变量

    // 积分值
    float Integral_Error = 0.0f;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_PID::Get_Integral_Error() const
{
    return (Integral_Error);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Class_PID::Get_Out() const
{
    return (Out);
}

/**
 * @brief 设定PID的P
 *
 * @param k_p PID的P
 */
inline void Class_PID::Set_K_P(float k_p)
{
    K_P = k_p;
}

/**
 * @brief 设定PID的I
 *
 * @param k_i PID的I
 */
inline void Class_PID::Set_K_I(float k_i)
{
    K_I = k_i;
}

/**
 * @brief 设定PID的D
 *
 * @param k_d PID的D
 */
inline void Class_PID::Set_K_D(float k_d)
{
    K_D = k_d;
}

/**
 * @brief 设定每周期目标增量前馈系数
 *
 * @param k_f 前馈系数，不除以 D_T
 */
inline void Class_PID::Set_K_F(float k_f)
{
    K_F = k_f;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param i_out_max 积分限幅, 0为不限制
 */
inline void Class_PID::Set_I_Out_Max(float i_out_max)
{
    I_Out_Max = i_out_max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param out_max 输出限幅, 0为不限制
 */
inline void Class_PID::Set_Out_Max(float out_max)
{
    Out_Max = out_max;
}

/**
 * @brief 设定变速积分内段阈值 A
 *
 * @param i_variable_speed_a 与 B 均为零时关闭；启用线性段时要求 0 <= A < B
 */
inline void Class_PID::Set_I_Variable_Speed_A(float i_variable_speed_a)
{
    I_Variable_Speed_A = i_variable_speed_a;
}

/**
 * @brief 设定变速积分外段阈值 B，不是区间宽度
 *
 * @param i_variable_speed_b 与 A 均为零时关闭；启用线性段时要求 0 <= A < B
 */
inline void Class_PID::Set_I_Variable_Speed_B(float i_variable_speed_b)
{
    I_Variable_Speed_B = i_variable_speed_b;
}

/**
 * @brief 设定积分分离阈值，达到时清空积分，不是冻结积分
 *
 * @param i_separate_threshold 积分分离阈值，需为正数, 0为不限制
 */
inline void Class_PID::Set_I_Separate_Threshold(float i_separate_threshold)
{
    I_Separate_Threshold = i_separate_threshold;
}

/**
 * @brief 设定目标值
 *
 * @param target 目标值
 */
inline void Class_PID::Set_Target(float target)
{
    Target = target;
}

/**
 * @brief 设定当前值
 *
 * @param now 当前值
 */
inline void Class_PID::Set_Now(float now)
{
    Now = now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param integral_error 积分值
 */
inline void Class_PID::Set_Integral_Error(float integral_error)
{
    Integral_Error = integral_error;
}

inline float Class_PID::Get_K_P() const
{
    return K_P;
}

inline float Class_PID::Get_K_I() const
{
    return K_I;
}

inline float Class_PID::Get_K_D() const
{
    return K_D;
}

inline float Class_PID::Get_K_F() const
{
    return K_F;
}

inline float Class_PID::Get_I_Out_Max() const
{
    return I_Out_Max;
}

inline float Class_PID::Get_Out_Max() const
{
    return Out_Max;
}

inline float Class_PID::Get_I_Variable_Speed_A() const
{
    return I_Variable_Speed_A;
}

inline float Class_PID::Get_I_Variable_Speed_B() const
{
    return I_Variable_Speed_B;
}

inline float Class_PID::Get_I_Separate_Threshold() const
{
    return I_Separate_Threshold;
}

inline float Class_PID::Get_D_T() const
{
    return D_T;
}

inline Enum_PID_D_First Class_PID::Get_D_First() const
{
    return D_First;
}

inline float Class_PID::Get_D_Filter_Cutoff() const
{
    return D_Filter_Cutoff;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
