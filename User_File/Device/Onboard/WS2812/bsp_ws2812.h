/**
 * @file bsp_ws2812.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 板载LED灯珠
 * @version 0.1
 * @date 2025-08-14 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_WS2812_H
#define BSP_WS2812_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_spi.h"
#include "sys_critical.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief WS2812单个灯珠颜色值
 *
 */
struct Struct_WS2812_Color
{
    uint8_t Red;
    uint8_t Green;
    uint8_t Blue;
};

/**
 * @brief Specialized, WS2812单个灯珠
 *
 */
class Class_WS2812
{
public:
    void Init(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0);

    inline void Set_Red(uint8_t red);

    inline void Set_Green(uint8_t green);

    inline void Set_Blue(uint8_t blue);

    inline void Set_RGB(uint8_t red, uint8_t green, uint8_t blue);

    inline void Set_Color(const Struct_WS2812_Color &color, float brightness = 1.0f);

    void TIM_10ms_Write_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;

    // 常量

    // 灯珠的逻辑码
    const uint8_t LEVEL_0 = 0x60;
    const uint8_t LEVEL_1 = 0x78;

    // 内部变量

    // 读变量

    // 写变量

    // RGB颜色值
    // WS2812的RGB顺序是GRB, 为方便使用, 仍采用RGB顺序
    Struct_WS2812_Color Color;
    Struct_WS2812_Color Last_Written_Color;
    bool Has_Written_Color = false;

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

extern const Struct_WS2812_Color WS2812_COLOR_BLACK;
extern const Struct_WS2812_Color WS2812_COLOR_WHITE;
extern const Struct_WS2812_Color WS2812_COLOR_RED;
extern const Struct_WS2812_Color WS2812_COLOR_GREEN;
extern const Struct_WS2812_Color WS2812_COLOR_BLUE;
extern const Struct_WS2812_Color WS2812_COLOR_YELLOW;
extern const Struct_WS2812_Color WS2812_COLOR_CYAN;
extern const Struct_WS2812_Color WS2812_COLOR_MAGENTA;

extern Class_WS2812 BSP_WS2812;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置红色值
 *
 * @param red 红色分量
 */
inline void Class_WS2812::Set_Red(uint8_t red)
{
    Color.Red = red;
    Color.Green = 0;
    Color.Blue = 0;
}

/**
 * @brief 设置绿色值
 *
 * @param green 绿色分量
 */
inline void Class_WS2812::Set_Green(uint8_t green)
{
    Color.Red = 0;
    Color.Green = green;
    Color.Blue = 0;
}

/**
 * @brief 设置蓝色值
 *
 * @param blue 蓝色分量
 */
inline void Class_WS2812::Set_Blue(uint8_t blue)
{
    Color.Red = 0;
    Color.Green = 0;
    Color.Blue = blue;
}

/**
 * @brief 设置RGB颜色值
 *
 * @param red 红色分量
 * @param green 绿色分量
 * @param blue 蓝色分量
 */
inline void Class_WS2812::Set_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    Color.Red = red;
    Color.Green = green;
    Color.Blue = blue;
}

/**
 * @brief 设置颜色值
 *
 * @param color 颜色值
 * @param brightness 各颜色通道的亮度比例，范围 [0, 1]。
 */
inline void Class_WS2812::Set_Color(const Struct_WS2812_Color &color, float brightness)
{
    const Struct_WS2812_Color scaled_color = {
        (uint8_t)((float)color.Red * brightness),
        (uint8_t)((float)color.Green * brightness),
        (uint8_t)((float)color.Blue * brightness),
    };
    const uint32_t primask = SYS_Enter_Critical();
    Color = scaled_color;
    SYS_Exit_Critical(primask);
}

#endif

#ifdef __cplusplus
extern "C" {
#endif

void BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback();

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
