/**
 * @file alg_dob.cpp
 * @author zzm
 * @brief 一阶名义模型的Q滤波扰动观测器
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_dob.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化DOB, 成功后按零初值复位
 * @param __Gain 名义模型静态增益, 非零
 * @param __Time_Constant 名义模型时间常数, s, 大于0
 * @param __Filter_Frequency Q滤波器的双重极点频率, Hz, 大于0且小于采样频率的一半
 * @param __D_T 固定采样周期, s, 大于0
 * @return 参数及系数有效时返回true, 否则保留原配置和状态并返回false
 * @note 极点频率不是整个Q的-3dB截止频率; 非零初始测量需另行Reset对齐。
 */
bool Class_DOB_First_Order::Init(float __Gain, float __Time_Constant, float __Filter_Frequency, float __D_T)
{
    if (Basic_Math_Is_Invalid_Float(__Gain) ||
        Basic_Math_Is_Invalid_Float(__Time_Constant) ||
        Basic_Math_Is_Invalid_Float(__Filter_Frequency) ||
        Basic_Math_Is_Invalid_Float(__D_T) ||
        __Gain == 0.0f || __Time_Constant <= 0.0f ||
        __Filter_Frequency <= 0.0f || __D_T <= 0.0f ||
        __Filter_Frequency * __D_T >= 0.5f)
    {
        return (false);
    }

    // beta = 1-exp(-Ts/tau), 名义离散模型为y[k]=(1-beta)*y[k-1]+Gain*beta*(u+d)[k-1]
    // expm1f避免指数接近1时, 直接相减损失精度
    float model_step = -expm1f(-__D_T / __Time_Constant);
    float gain_inverse = 1.0f / __Gain;
    float filter_alpha = -expm1f(-2.0f * PI * __Filter_Frequency * __D_T);
    // 将逆模型的1/(Gain*beta)与第一级Q的alpha合并
    float difference_gain = filter_alpha / (__Gain * model_step);
    if (Basic_Math_Is_Invalid_Float(gain_inverse) ||
        Basic_Math_Is_Invalid_Float(difference_gain) || filter_alpha <= 0.0f)
    {
        return (false);
    }

    Gain_Inverse = gain_inverse;
    Filter_Alpha = filter_alpha;
    Difference_Gain = difference_gain;

    Reset();
    return (true);
}

/**
 * @brief 以当前测量值为起点清空观测状态
 * @param __Now 复位时刻的测量y[0], 与后续Set_Now同单位
 * @details 下一次计算应传入一个采样周期后的测量值及该周期实际施加的输入。
 * 保留模型和滤波系数, 将扰动估计及两级滤波状态清零。
 */
void Class_DOB_First_Order::Reset(float __Now)
{
    Now = __Now;
    Pre_Now = __Now;
    Input = 0.0f;
    Filter_Out = 0.0f;
    Out = 0.0f;
}

/**
 * @brief 按配置采样周期执行一次扰动观测
 * @details 逆模型残差为(y[k]-(1-beta)*y[k-1])/(Gain*beta)-u[k-1]。
 * 第一级将残差与低通合并计算, 再串联第二级低通, 不先生成未滤波的差分商。
 * @note 每次调用前更新Now和Input; 一个采样周期计算一次。
 */
void Class_DOB_First_Order::TIM_Calculate_PeriodElapsedCallback()
{
    Filter_Out += Difference_Gain * (Now - Pre_Now) +
                  Filter_Alpha * (Gain_Inverse * Pre_Now - Input - Filter_Out);
    Out += Filter_Alpha * (Filter_Out - Out);
    Pre_Now = Now;
}
