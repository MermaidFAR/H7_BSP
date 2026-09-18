/**
 * @file alg_smc.h
 * @author zzm
 * @brief 线性滑模面与饱和边界层滑模控制
 */

#ifndef __ALG_SMC_H
#define __ALG_SMC_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 单轴二阶对象滑模控制器
 * @details 模型为 d2y/dt2 = Model + Gain * u + d, Gain为名义输入增益, d为匹配扰动。
 * 误差e = Target - Now, 滑模面s = de/dt + Lambda * e。
 * 名义趋近律为ds/dt = -K * s - Eta * sat(s / Boundary), 以边界层减轻抖振。
 * @note Init成功后使用; 调用方提供同一时刻的有限输入, 不在内部差分或积分。
 * 角度需由调用方展开为连续量; 输出饱和及采样延迟会影响趋近条件。
 * 参数Setter不重复检查, 调用方保证满足Init约束, 在同一控制上下文的两次计算之间更新。
 */
class Class_SMC
{
public:
    bool Init(float __Lambda, float __K, float __Eta, float __Boundary, float __Gain, float __Out_Max = 0.0f);

    inline float Get_Out() const;

    inline float Get_Surface() const;

    inline float Get_Lambda() const;

    inline float Get_K() const;

    inline float Get_Eta() const;

    inline float Get_Boundary() const;

    inline float Get_Gain() const;

    inline float Get_Out_Max() const;

    inline void Set_Lambda(float __Lambda);

    inline void Set_K(float __K);

    inline void Set_Eta(float __Eta);

    inline void Set_Boundary(float __Boundary);

    inline void Set_Gain(float __Gain);

    inline void Set_Out_Max(float __Out_Max);

    inline void Set_Target(float __Target, float __Target_Dot = 0.0f, float __Target_DDot = 0.0f);

    inline void Set_Now(float __Now, float __Now_Dot);

    inline void Set_Model(float __Model);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    float Lambda = 0.0f;            ///< 滑模面系数, 1/s
    float K = 0.0f;                 ///< 线性趋近增益, 1/s
    float Eta = 0.0f;               ///< 切换趋近增益, 与y的二阶导数同单位
    float Boundary = 0.0f;          ///< 边界层半宽, 与y的一阶导数同单位
    float Gain = 0.0f;              ///< 名义输入增益
    float Gain_Inverse = 0.0f;      ///< 名义输入增益的倒数
    float Out_Max = 0.0f;           ///< 对称输出限幅, 0为不限制

    float Target = 0.0f;            ///< 目标值
    float Target_Dot = 0.0f;        ///< 目标一阶导数
    float Target_DDot = 0.0f;       ///< 目标二阶导数
    float Now = 0.0f;               ///< 当前测量值
    float Now_Dot = 0.0f;           ///< 当前测量一阶导数
    float Model = 0.0f;             ///< 名义模型中不含控制输入的二阶导数项

    float Surface = 0.0f;           ///< 最近一次计算的滑模面值
    float Out = 0.0f;               ///< 最近一次计算的限幅后输出
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取控制输出
 * @return 与模型输入u同单位的限幅后输出
 */
inline float Class_SMC::Get_Out() const
{
    return (Out);
}

/**
 * @brief 获取滑模面值
 * @return 最近一次计算的s, 与测量一阶导数同单位
 */
inline float Class_SMC::Get_Surface() const
{
    return (Surface);
}

/**
 * @brief 获取滑模面系数
 * @return 滑模面系数, 1/s
 */
inline float Class_SMC::Get_Lambda() const
{
    return (Lambda);
}

/**
 * @brief 获取线性趋近增益
 * @return 线性趋近增益, 1/s
 */
inline float Class_SMC::Get_K() const
{
    return (K);
}

/**
 * @brief 获取切换趋近增益
 * @return 切换趋近增益, 与测量二阶导数同单位
 */
inline float Class_SMC::Get_Eta() const
{
    return (Eta);
}

/**
 * @brief 获取边界层半宽
 * @return 边界层半宽, 与测量一阶导数同单位
 */
inline float Class_SMC::Get_Boundary() const
{
    return (Boundary);
}

/**
 * @brief 获取名义输入增益
 * @return 名义输入增益, 单位为测量二阶导数/控制输入
 */
inline float Class_SMC::Get_Gain() const
{
    return (Gain);
}

/**
 * @brief 获取输出限幅
 * @return 输出绝对值上限, 0为不限制
 */
inline float Class_SMC::Get_Out_Max() const
{
    return (Out_Max);
}

/**
 * @brief 设定滑模面系数
 * @param __Lambda 滑模面系数, 1/s, 大于0
 */
inline void Class_SMC::Set_Lambda(float __Lambda)
{
    Lambda = __Lambda;
}

/**
 * @brief 设定线性趋近增益
 * @param __K 线性趋近增益, 1/s, 不小于0
 */
inline void Class_SMC::Set_K(float __K)
{
    K = __K;
}

/**
 * @brief 设定切换趋近增益
 * @param __Eta 切换趋近增益, 与测量二阶导数同单位, 大于0
 */
inline void Class_SMC::Set_Eta(float __Eta)
{
    Eta = __Eta;
}

/**
 * @brief 设定边界层半宽
 * @param __Boundary 边界层半宽, 与测量一阶导数同单位, 大于0
 */
inline void Class_SMC::Set_Boundary(float __Boundary)
{
    Boundary = __Boundary;
}

/**
 * @brief 设定名义输入增益并更新倒数
 * @param __Gain 名义输入增益, 非零且倒数有效, 允许为负
 */
inline void Class_SMC::Set_Gain(float __Gain)
{
    Gain = __Gain;
    Gain_Inverse = 1.0f / __Gain;
}

/**
 * @brief 设定输出限幅
 * @param __Out_Max 输出绝对值上限, 不小于0, 0为不限制
 */
inline void Class_SMC::Set_Out_Max(float __Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief 设置当前时刻的目标轨迹
 * @param __Target 目标值
 * @param __Target_Dot 目标一阶导数, 定点控制时为0
 * @param __Target_DDot 目标二阶导数, 定点控制时为0
 */
inline void Class_SMC::Set_Target(float __Target, float __Target_Dot, float __Target_DDot)
{
    Target = __Target;
    Target_Dot = __Target_Dot;
    Target_DDot = __Target_DDot;
}

/**
 * @brief 设置同一时刻的测量状态
 * @param __Now 当前测量值
 * @param __Now_Dot 当前测量一阶导数, 由传感器或外部估计器提供
 */
inline void Class_SMC::Set_Now(float __Now, float __Now_Dot)
{
    Now = __Now;
    Now_Dot = __Now_Dot;
}

/**
 * @brief 设置当前名义模型项
 * @param __Model 模型中的f(y, dy/dt, t), 与测量二阶导数同单位
 * @note 例如含粘性阻尼的转动模型, 此项可取-阻尼系数/惯量*角速度。
 */
inline void Class_SMC::Set_Model(float __Model)
{
    Model = __Model;
}

#endif
