/**
 * @file bsp_buzzer.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 板载蜂鸣器
 * @version 0.1
 * @date 2025-08-15 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/**
 * Cube中, 主频240M, psc240, arr250, 对应freq4000Hz
 * 原则上满足, arr * freq = 1MHz
 * 因此, freq = 1MHz / arr
 *
 * 此外, 该蜂鸣器有效音域为G3~ A7
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "tim.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

extern const float BUZZER_FREQUENCY_A0;
extern const float BUZZER_FREQUENCY_A0_SHARP;
extern const float BUZZER_FREQUENCY_B0_FLAT;
extern const float BUZZER_FREQUENCY_B0;
extern const float BUZZER_FREQUENCY_C1;
extern const float BUZZER_FREQUENCY_C1_SHARP;
extern const float BUZZER_FREQUENCY_D1_FLAT;
extern const float BUZZER_FREQUENCY_D1;
extern const float BUZZER_FREQUENCY_D1_SHARP;
extern const float BUZZER_FREQUENCY_E1_FLAT;
extern const float BUZZER_FREQUENCY_E1;
extern const float BUZZER_FREQUENCY_F1;
extern const float BUZZER_FREQUENCY_F1_SHARP;
extern const float BUZZER_FREQUENCY_G1_FLAT;
extern const float BUZZER_FREQUENCY_G1;
extern const float BUZZER_FREQUENCY_G1_SHARP;
extern const float BUZZER_FREQUENCY_A1_FLAT;
extern const float BUZZER_FREQUENCY_A1;
extern const float BUZZER_FREQUENCY_A1_SHARP;
extern const float BUZZER_FREQUENCY_B1_FLAT;
extern const float BUZZER_FREQUENCY_B1;
extern const float BUZZER_FREQUENCY_C2;
extern const float BUZZER_FREQUENCY_C2_SHARP;
extern const float BUZZER_FREQUENCY_D2_FLAT;
extern const float BUZZER_FREQUENCY_D2;
extern const float BUZZER_FREQUENCY_D2_SHARP;
extern const float BUZZER_FREQUENCY_E2_FLAT;
extern const float BUZZER_FREQUENCY_E2;
extern const float BUZZER_FREQUENCY_F2;
extern const float BUZZER_FREQUENCY_F2_SHARP;
extern const float BUZZER_FREQUENCY_G2_FLAT;
extern const float BUZZER_FREQUENCY_G2;
extern const float BUZZER_FREQUENCY_G2_SHARP;
extern const float BUZZER_FREQUENCY_A2_FLAT;
extern const float BUZZER_FREQUENCY_A2;
extern const float BUZZER_FREQUENCY_A2_SHARP;
extern const float BUZZER_FREQUENCY_B2_FLAT;
extern const float BUZZER_FREQUENCY_B2;
extern const float BUZZER_FREQUENCY_C3;
extern const float BUZZER_FREQUENCY_C3_SHARP;
extern const float BUZZER_FREQUENCY_D3_FLAT;
extern const float BUZZER_FREQUENCY_D3;
extern const float BUZZER_FREQUENCY_D3_SHARP;
extern const float BUZZER_FREQUENCY_E3_FLAT;
extern const float BUZZER_FREQUENCY_E3;
extern const float BUZZER_FREQUENCY_F3;
extern const float BUZZER_FREQUENCY_F3_SHARP;
extern const float BUZZER_FREQUENCY_G3_FLAT;
extern const float BUZZER_FREQUENCY_G3;
extern const float BUZZER_FREQUENCY_G3_SHARP;
extern const float BUZZER_FREQUENCY_A3_FLAT;
extern const float BUZZER_FREQUENCY_A3;
extern const float BUZZER_FREQUENCY_A3_SHARP;
extern const float BUZZER_FREQUENCY_B3_FLAT;
extern const float BUZZER_FREQUENCY_B3;
extern const float BUZZER_FREQUENCY_C4;
extern const float BUZZER_FREQUENCY_C4_SHARP;
extern const float BUZZER_FREQUENCY_D4_FLAT;
extern const float BUZZER_FREQUENCY_D4;
extern const float BUZZER_FREQUENCY_D4_SHARP;
extern const float BUZZER_FREQUENCY_E4_FLAT;
extern const float BUZZER_FREQUENCY_E4;
extern const float BUZZER_FREQUENCY_F4;
extern const float BUZZER_FREQUENCY_F4_SHARP;
extern const float BUZZER_FREQUENCY_G4_FLAT;
extern const float BUZZER_FREQUENCY_G4;
extern const float BUZZER_FREQUENCY_G4_SHARP;
extern const float BUZZER_FREQUENCY_A4_FLAT;
extern const float BUZZER_FREQUENCY_A4;
extern const float BUZZER_FREQUENCY_A4_SHARP;
extern const float BUZZER_FREQUENCY_B4_FLAT;
extern const float BUZZER_FREQUENCY_B4;
extern const float BUZZER_FREQUENCY_C5;
extern const float BUZZER_FREQUENCY_C5_SHARP;
extern const float BUZZER_FREQUENCY_D5_FLAT;
extern const float BUZZER_FREQUENCY_D5;
extern const float BUZZER_FREQUENCY_D5_SHARP;
extern const float BUZZER_FREQUENCY_E5_FLAT;
extern const float BUZZER_FREQUENCY_E5;
extern const float BUZZER_FREQUENCY_F5;
extern const float BUZZER_FREQUENCY_F5_SHARP;
extern const float BUZZER_FREQUENCY_G5_FLAT;
extern const float BUZZER_FREQUENCY_G5;
extern const float BUZZER_FREQUENCY_G5_SHARP;
extern const float BUZZER_FREQUENCY_A5_FLAT;
extern const float BUZZER_FREQUENCY_A5;
extern const float BUZZER_FREQUENCY_A5_SHARP;
extern const float BUZZER_FREQUENCY_B5_FLAT;
extern const float BUZZER_FREQUENCY_B5;
extern const float BUZZER_FREQUENCY_C6;
extern const float BUZZER_FREQUENCY_C6_SHARP;
extern const float BUZZER_FREQUENCY_D6_FLAT;
extern const float BUZZER_FREQUENCY_D6;
extern const float BUZZER_FREQUENCY_D6_SHARP;
extern const float BUZZER_FREQUENCY_E6_FLAT;
extern const float BUZZER_FREQUENCY_E6;
extern const float BUZZER_FREQUENCY_F6;
extern const float BUZZER_FREQUENCY_F6_SHARP;
extern const float BUZZER_FREQUENCY_G6_FLAT;
extern const float BUZZER_FREQUENCY_G6;
extern const float BUZZER_FREQUENCY_G6_SHARP;
extern const float BUZZER_FREQUENCY_A6_FLAT;
extern const float BUZZER_FREQUENCY_A6;
extern const float BUZZER_FREQUENCY_A6_SHARP;
extern const float BUZZER_FREQUENCY_B6_FLAT;
extern const float BUZZER_FREQUENCY_B6;
extern const float BUZZER_FREQUENCY_C7;
extern const float BUZZER_FREQUENCY_C7_SHARP;
extern const float BUZZER_FREQUENCY_D7_FLAT;
extern const float BUZZER_FREQUENCY_D7;
extern const float BUZZER_FREQUENCY_D7_SHARP;
extern const float BUZZER_FREQUENCY_E7_FLAT;
extern const float BUZZER_FREQUENCY_E7;
extern const float BUZZER_FREQUENCY_F7;
extern const float BUZZER_FREQUENCY_F7_SHARP;
extern const float BUZZER_FREQUENCY_G7_FLAT;
extern const float BUZZER_FREQUENCY_G7;
extern const float BUZZER_FREQUENCY_G7_SHARP;
extern const float BUZZER_FREQUENCY_A7_FLAT;
extern const float BUZZER_FREQUENCY_A7;
extern const float BUZZER_FREQUENCY_A7_SHARP;
extern const float BUZZER_FREQUENCY_B7_FLAT;
extern const float BUZZER_FREQUENCY_B7;
extern const float BUZZER_FREQUENCY_C8;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 板载蜂鸣器
 *
 */
class Class_Buzzer
{
public:
    void Init(float frequency = 4000.0f, float loudness = 0.0f);

    /** @brief 设置频率 (Hz)，无效值保留当前设置。 */
    void Set_Frequency(float frequency);

    /** @brief 设置响度 [0, 1]，无效值保留当前设置。 */
    void Set_Loudness(float loudness);

    /** @brief 同时设置频率 (Hz) 与响度 [0, 1]。 */
    void Set_Sound(float frequency, float loudness);

private:
    TIM_HandleTypeDef *htim = nullptr;
    uint32_t TIM_Channel = TIM_CHANNEL_2;
    float Frequency = 4000.0f;
    float Loudness = 0.0f;

    void Apply_PWM();
};

/* Exported variables --------------------------------------------------------*/

extern Class_Buzzer BSP_Buzzer;

#endif
