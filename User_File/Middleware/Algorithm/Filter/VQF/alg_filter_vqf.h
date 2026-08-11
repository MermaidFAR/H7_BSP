/**
 * @file alg_filter_vqf.h
 * @author zzm
 * @brief VQF姿态滤波器
 * @version 1.0
 * @date 2026-08-12
 *
 * SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
 * SPDX-FileCopyrightText: 2026 zzm
 * SPDX-License-Identifier: MIT
 */

#ifndef __ALG_FILTER_VQF_H
#define __ALG_FILTER_VQF_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "alg_quaternion.h"

#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VQF参数
 */
struct Struct_VQF_Parameter
{
    ///< 加速度重力方向修正的时间常数，单位s；越大越平滑、收敛越慢。
    float Tau_Accel = 3.0f;
    ///< 是否在运动状态下利用重力方向约束估计陀螺仪零偏。
    bool Motion_Bias_Estimation_Enable = true;
    ///< 是否在静止确认后利用低通角速度估计陀螺仪零偏。
    bool Rest_Bias_Estimation_Enable = true;
    ///< 启动时零偏估计的标准差，单位deg/s；越大允许初期调整越快。
    float Bias_Sigma_Init_Deg_S = 0.5f;
    ///< 零偏协方差的遗忘时间，单位s；越大则长期估计越稳定。
    float Bias_Forgetting_Time = 100.0f;
    ///< 零偏估计和观测残差的绝对限幅，单位deg/s。
    float Bias_Clip_Deg_S = 2.0f;
    ///< 运动状态下零偏观测的标准差，单位deg/s；越小权重越高。
    float Bias_Sigma_Motion_Deg_S = 0.1f;
    ///< 运动时重力轴方向的观测权重因子；越小越不信任该方向的估计。
    float Bias_Vertical_Forgetting_Factor = 0.0001f;
    ///< 静止状态下零偏观测的标准差，单位deg/s；越小权重越高。
    float Bias_Sigma_Rest_Deg_S = 0.03f;
    ///< 连续满足门限后判定静止的最短时间，单位s。
    float Rest_Min_Time = 1.5f;
    ///< 静止检测陀螺仪与加速度计低通滤波的时间常数，单位s。
    float Rest_Filter_Tau = 0.5f;
    ///< 静止检测允许的陀螺仪高频残差，单位deg/s。
    float Rest_Threshold_Gyro_Deg_S = 2.0f;
    ///< 静止检测允许的加速度高频残差，单位m/s^2。
    float Rest_Threshold_Accel = 0.5f;
};

/**
 * @brief Reusable, 6D VQF姿态滤波器
 */
class Class_Filter_VQF
{
public:
    /**
     * @brief 初始化VQF参数和滤波状态
     * @param __Parameter 姿态修正、零偏估计和静止检测参数
     * @param __Gyro_D_T 陀螺仪更新周期，单位s
     * @param __Accel_D_T 加速度计更新周期，单位s
     */
    void Init(const Struct_VQF_Parameter &__Parameter,
              const float &__Gyro_D_T,
              const float &__Accel_D_T);

    void Reset();

    void Update_Gyro(const Class_Matrix_f32<3, 1> &__Gyro);

    void Update_Accel(const Class_Matrix_f32<3, 1> &__Accel);

    /**
     * @brief 设置外部提供的零偏初值
     * @param __Bias 陀螺仪零偏，单位rad/s
     * @param __Sigma 估计标准差，单位rad/s；负值表示保持当前协方差
     */
    void Set_Bias_Estimate(const Class_Matrix_f32<3, 1> &__Bias,
                           const float &__Sigma = -1.0f);

    /**
     * @brief 独立开关运动状态和静止状态的在线零偏估计
     */
    void Set_Bias_Estimation_Enable(const bool &__Motion_Enable,
                                    const bool &__Rest_Enable);

    inline bool Get_Initialized_Flag() const;

    inline Class_Quaternion_f32 Get_Quaternion_3D() const;

    inline Class_Quaternion_f32 Get_Quaternion_6D() const;

    inline Class_Matrix_f32<3, 1> Get_Bias_Estimate() const;

    float Get_Bias_Sigma() const;

    inline bool Get_Rest_Detected() const;

    inline Class_Matrix_f32<2, 1> Get_Relative_Rest_Deviation() const;

    inline float Get_Last_Accel_Correction_Rate() const;

    inline Class_Matrix_f32<3, 1> Get_Last_Corrected_Gyro() const;

protected:
    /* 初始化相关常量 --------------------------------------------------------*/

    Struct_VQF_Parameter Parameter;
    float Gyro_D_T = 0.0005f;
    float Accel_D_T = 0.004f;

    double Accel_Low_Pass_B[3] = {};
    double Accel_Low_Pass_A[2] = {};
    double Rest_Gyro_Low_Pass_B[3] = {};
    double Rest_Gyro_Low_Pass_A[2] = {};
    double Rest_Accel_Low_Pass_B[3] = {};
    double Rest_Accel_Low_Pass_A[2] = {};

    float Bias_Covariance_Init = 0.0f;
    float Bias_Process_Noise = 0.0f;
    float Bias_Motion_Noise = 0.0f;
    float Bias_Vertical_Noise = 0.0f;
    float Bias_Rest_Noise = 0.0f;

    /* 内部变量 --------------------------------------------------------------*/

    bool Initialized_Flag = false;
    Class_Quaternion_f32 Gyro_Quaternion;
    Class_Quaternion_f32 Accel_Quaternion;

    Class_Matrix_f32<3, 1> Last_Accel_Low_Pass;
    double Accel_Low_Pass_State[6] = {};
    float Last_Accel_Correction_Rate = 0.0f;

    Class_Matrix_f32<3, 1> Bias_Estimate;
    Class_Matrix_f32<3, 3> Bias_Covariance;
    double Motion_Rotation_Low_Pass_State[18] = {};
    double Motion_Bias_Low_Pass_State[4] = {};

    bool Rest_Detected = false;
    float Rest_Time = 0.0f;
    float Rest_Gyro_Squared_Deviation = 0.0f;
    float Rest_Accel_Squared_Deviation = 0.0f;
    Class_Matrix_f32<3, 1> Rest_Last_Gyro_Low_Pass;
    Class_Matrix_f32<3, 1> Rest_Last_Accel_Low_Pass;
    double Rest_Gyro_Low_Pass_State[6] = {};
    double Rest_Accel_Low_Pass_State[6] = {};

    Class_Matrix_f32<3, 1> Last_Corrected_Gyro;

    /* 内部函数 --------------------------------------------------------------*/

    void Calculate_Coefficient();

    void Update_Bias(const Class_Matrix_f32<3, 1> &__Accel_Earth);

    static bool Vector_Is_Valid(const Class_Matrix_f32<3, 1> &__Vector);

    static Class_Quaternion_f32 Normalize_Quaternion(
        const Class_Quaternion_f32 &__Quaternion);

    static void Calculate_Filter_Coefficient(const float &__Tau,
                                             const float &__D_T,
                                             double __B[3],
                                             double __A[2]);

    static void Reset_Filter_State(const uint32_t &__Length,
                                   double __State[]);

    static void Filter_Vector(const float __Input[],
                              const uint32_t &__Length,
                              const float &__Tau,
                              const float &__D_T,
                              const double __B[3],
                              const double __A[2],
                              double __State[],
                              float __Output[]);

    static bool Inverse_Matrix_3x3(const Class_Matrix_f32<3, 3> &__Input,
                                   Class_Matrix_f32<3, 3> &__Output);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline bool Class_Filter_VQF::Get_Initialized_Flag() const
{
    return (Initialized_Flag);
}

inline Class_Quaternion_f32 Class_Filter_VQF::Get_Quaternion_3D() const
{
    return (Gyro_Quaternion);
}

inline Class_Quaternion_f32 Class_Filter_VQF::Get_Quaternion_6D() const
{
    return (Normalize_Quaternion(Accel_Quaternion * Gyro_Quaternion));
}

inline Class_Matrix_f32<3, 1> Class_Filter_VQF::Get_Bias_Estimate() const
{
    return (Bias_Estimate);
}

inline bool Class_Filter_VQF::Get_Rest_Detected() const
{
    return (Rest_Detected);
}

inline Class_Matrix_f32<2, 1> Class_Filter_VQF::Get_Relative_Rest_Deviation() const
{
    Class_Matrix_f32<2, 1> result;
    const float gyro_threshold = Parameter.Rest_Threshold_Gyro_Deg_S * PI / 180.0f;
    result[0][0] = gyro_threshold > 0.0f
                       ? sqrtf(Rest_Gyro_Squared_Deviation) / gyro_threshold
                       : 0.0f;
    result[1][0] = Parameter.Rest_Threshold_Accel > 0.0f
                       ? sqrtf(Rest_Accel_Squared_Deviation) /
                             Parameter.Rest_Threshold_Accel
                       : 0.0f;
    return (result);
}

inline float Class_Filter_VQF::Get_Last_Accel_Correction_Rate() const
{
    return (Last_Accel_Correction_Rate);
}

inline Class_Matrix_f32<3, 1> Class_Filter_VQF::Get_Last_Corrected_Gyro() const
{
    return (Last_Corrected_Gyro);
}

#endif
