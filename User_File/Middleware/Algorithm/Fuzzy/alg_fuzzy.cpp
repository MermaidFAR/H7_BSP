/**
 * @file alg_fuzzy.cpp
 * @author zzm
 * @brief 双输入零阶Sugeno模糊推理
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_fuzzy.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 绑定严格递增的节点及完整规则表, 不复制数组、不分配内存
 * @return 配置有效时返回true; 失败保留原配置
 */
bool Class_Fuzzy_Sugeno::Init(const Struct_Fuzzy_Sugeno_Config &config)
{
    if (config.Rule_Table == NULL || config.Output_Count == 0 ||
        !Check_Nodes(config.Input_1_Nodes, config.Input_1_Count) ||
        !Check_Nodes(config.Input_2_Nodes, config.Input_2_Count))
    {
        return (false);
    }

    uint64_t length = (uint64_t) config.Input_1_Count *
                      config.Input_2_Count * config.Output_Count;
    if (length != config.Rule_Table_Length)
    {
        return (false);
    }
    for (uint32_t i = 0; i < config.Rule_Table_Length; i++)
    {
        if (Basic_Math_Is_Invalid_Float(config.Rule_Table[i]))
        {
            return (false);
        }
    }

    Config = config;
    return (true);
}

/**
 * @brief 对两个输入执行一次推理
 * @param out 调用方提供的Output_Count个float, 不得与节点或规则表重叠
 * @return 未初始化、输出指针为空或输入无效时返回false且不写输出
 */
bool Class_Fuzzy_Sugeno::Calculate(float input_1, float input_2, float *out) const
{
    if (Config.Rule_Table == NULL || out == NULL ||
        Basic_Math_Is_Invalid_Float(input_1) ||
        Basic_Math_Is_Invalid_Float(input_2))
    {
        return (false);
    }

    float ratio_1;
    float ratio_2;
    uint16_t index_1 = Find_Interval(Config.Input_1_Nodes, Config.Input_1_Count,
                                     input_1, &ratio_1);
    uint16_t index_2 = Find_Interval(Config.Input_2_Nodes, Config.Input_2_Count,
                                     input_2, &ratio_2);
    uint32_t offset = ((uint32_t) index_1 * Config.Input_2_Count + index_2) *
                      Config.Output_Count;
    uint32_t stride = (uint32_t) Config.Input_2_Count * Config.Output_Count;
    const float *rule_00 = Config.Rule_Table + offset;
    const float *rule_01 = rule_00 + Config.Output_Count;
    const float *rule_10 = rule_00 + stride;
    const float *rule_11 = rule_10 + Config.Output_Count;

    for (uint16_t i = 0; i < Config.Output_Count; i++)
    {
        float low = (1.0f - ratio_1) * rule_00[i] + ratio_1 * rule_10[i];
        float high = (1.0f - ratio_1) * rule_01[i] + ratio_1 * rule_11[i];
        out[i] = (1.0f - ratio_2) * low + ratio_2 * high;
    }
    return (true);
}

bool Class_Fuzzy_Sugeno::Check_Nodes(const float *nodes, uint16_t count)
{
    if (nodes == NULL || count < 2)
    {
        return (false);
    }
    for (uint16_t i = 0; i < count; i++)
    {
        if (Basic_Math_Is_Invalid_Float(nodes[i]))
        {
            return (false);
        }
        if (i > 0)
        {
            float width = nodes[i] - nodes[i - 1];
            if (Basic_Math_Is_Invalid_Float(width) || width <= 0.0f)
            {
                return (false);
            }
        }
    }
    return (true);
}

uint16_t Class_Fuzzy_Sugeno::Find_Interval(const float *nodes, uint16_t count,
                                           float input, float *ratio)
{
    if (input <= nodes[0])
    {
        *ratio = 0.0f;
        return (0);
    }
    if (input >= nodes[count - 1])
    {
        *ratio = 1.0f;
        return (count - 2);
    }

    uint16_t low = 0;
    uint16_t high = count - 1;
    while (high - low > 1)
    {
        uint16_t mid = low + (high - low) / 2;
        if (input < nodes[mid])
        {
            high = mid;
        }
        else
        {
            low = mid;
        }
    }
    *ratio = (input - nodes[low]) / (nodes[high] - nodes[low]);
    return (low);
}
