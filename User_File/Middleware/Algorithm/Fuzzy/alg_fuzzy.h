/**
 * @file alg_fuzzy.h
 * @author zzm
 * @brief 双输入零阶Sugeno模糊推理
 */

#ifndef ALG_FUZZY_H
#define ALG_FUZZY_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct Struct_Fuzzy_Sugeno_Config
{
    const float *Input_1_Nodes = NULL;
    const float *Input_2_Nodes = NULL;
    const float *Rule_Table = NULL;
    uint16_t Input_1_Count = 0;
    uint16_t Input_2_Count = 0;
    uint16_t Output_Count = 0;
    uint32_t Rule_Table_Length = 0; ///< float元素数, 等于两个节点数与输出数的乘积
};

/**
 * @brief Reusable, 双输入、多输出的零阶Sugeno推理器
 * @details 相邻节点间为线性三角隶属函数, 两端为肩形, 每个输入的隶属度和为1。
 * 采用乘积AND、常数后件、完整规则表及单位规则权重, 等价于分片双线性插值。
 * 规则按[input1节点][input2节点][输出]展开为一维数组, 输出下标变化最快。
 * 节点及规则由调用方持有, 使用期间保持有效且只读; Init与Calculate不可并发。
 * 输入与节点使用相同单位, 超出节点范围时保持边界值; 缩放及微分由调用方处理。
 */
class Class_Fuzzy_Sugeno
{
  public:
    bool Init(const Struct_Fuzzy_Sugeno_Config &config);

    bool Calculate(float input_1, float input_2, float *out) const;

    inline uint16_t Get_Output_Count() const;

  protected:
    Struct_Fuzzy_Sugeno_Config Config;

    static bool Check_Nodes(const float *nodes, uint16_t count);

    static uint16_t Find_Interval(const float *nodes, uint16_t count,
                                  float input, float *ratio);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline uint16_t Class_Fuzzy_Sugeno::Get_Output_Count() const
{
    return (Config.Output_Count);
}

#endif
