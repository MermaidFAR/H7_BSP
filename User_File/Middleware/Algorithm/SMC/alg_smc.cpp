/**
 * @file alg_smc.cpp
 * @author zzm
 * @brief 线性滑模面与饱和边界层滑模控制
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_smc.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化滑模控制器, 成功后清零输入与输出
 * @param __Lambda 滑模面系数, 1/s, 大于0
 * @param __K 线性趋近增益, 1/s, 不小于0; 取0时为等速趋近律
 * @param __Eta 切换趋近增益, 与测量二阶导数同单位, 大于0
 * @param __Boundary 边界层半宽, 与测量一阶导数同单位, 大于0
 * @param __Gain 名义输入增益, 单位为测量二阶导数/控制输入, 非零, 允许为负
 * @param __Out_Max 输出绝对值上限, 不小于0, 0为不限制
 * @return 参数有效时返回true, 否则保留原配置和输入输出并返回false
 * @note 理想模型且未饱和时, Eta大于匹配扰动上界可保证边界层外趋近。
 * 边界层内允许存在稳态误差; 采样周期需足够小, 连续趋近条件不等于离散稳定保证。
 */
bool Class_SMC::Init(float __Lambda, float __K, float __Eta, float __Boundary, float __Gain, float __Out_Max)
{
    if (Basic_Math_Is_Invalid_Float(__Lambda) ||
        Basic_Math_Is_Invalid_Float(__K) ||
        Basic_Math_Is_Invalid_Float(__Eta) ||
        Basic_Math_Is_Invalid_Float(__Boundary) ||
        Basic_Math_Is_Invalid_Float(__Gain) ||
        Basic_Math_Is_Invalid_Float(__Out_Max) ||
        __Lambda <= 0.0f || __K < 0.0f || __Eta <= 0.0f ||
        __Boundary <= 0.0f || __Gain == 0.0f || __Out_Max < 0.0f)
    {
        return (false);
    }

    float gain_inverse = 1.0f / __Gain;
    if (Basic_Math_Is_Invalid_Float(gain_inverse))
    {
        return (false);
    }

    Lambda = __Lambda;
    K = __K;
    Eta = __Eta;
    Boundary = __Boundary;
    Gain = __Gain;
    Gain_Inverse = gain_inverse;
    Out_Max = __Out_Max;

    Target = 0.0f;
    Target_Dot = 0.0f;
    Target_DDot = 0.0f;
    Now = 0.0f;
    Now_Dot = 0.0f;
    Model = 0.0f;
    Surface = 0.0f;
    Out = 0.0f;
    return (true);
}

/**
 * @brief 使用当前状态计算一次控制输出
 * @details u = (目标二阶导数 - Model + Lambda*de/dt + K*s + Eta*sat(s/Boundary)) / Gain。
 * @note 调用前更新目标、测量及模型项; 本函数无时间历史, 由调用方周期调度。
 */
void Class_SMC::TIM_Calculate_PeriodElapsedCallback()
{
    float error = Target - Now;
    float error_dot = Target_Dot - Now_Dot;
    Surface = error_dot + Lambda * error;
    float switching = Basic_Math_Constrain(Surface / Boundary, -1.0f, 1.0f);

    Out = (Target_DDot - Model + Lambda * error_dot + K * Surface + Eta * switching) * Gain_Inverse;
    if (Out_Max > 0.0f)
    {
        Out = Basic_Math_Constrain(Out, -Out_Max, Out_Max);
    }
}
