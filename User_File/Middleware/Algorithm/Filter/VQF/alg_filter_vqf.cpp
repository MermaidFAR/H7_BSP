/**
 * @file alg_filter_vqf.cpp
 * @author zzm
 * @brief VQF姿态滤波器
 * @version 1.0
 * @date 2026-08-12
 *
 * SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
 * SPDX-FileCopyrightText: 2026 zzm
 * SPDX-License-Identifier: MIT
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_filter_vqf.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化VQF
 *
 * @param parameter VQF参数
 * @param gyro_d_t 陀螺仪采样周期, 单位s
 * @param accel_d_t 加速度计采样周期, 单位s
 */
void Class_Filter_VQF::Init(const Struct_VQF_Parameter &parameter,
                            float gyro_d_t,
                            float accel_d_t)
{
    Parameter = parameter;
    Gyro_D_T = gyro_d_t;
    Accel_D_T = accel_d_t;

    if (Parameter.Tau_Accel <= 0.0f)
    {
        Parameter.Tau_Accel = 3.0f;
    }
    if (Parameter.Rest_Filter_Tau <= 0.0f)
    {
        Parameter.Rest_Filter_Tau = 0.5f;
    }
    if (Parameter.Bias_Forgetting_Time <= 0.0f)
    {
        Parameter.Bias_Forgetting_Time = 100.0f;
    }
    if (Gyro_D_T <= 0.0f)
    {
        Gyro_D_T = 0.0005f;
    }
    if (Accel_D_T <= 0.0f)
    {
        Accel_D_T = Gyro_D_T;
    }

    Calculate_Coefficient();
    Reset();
    Initialized_Flag = true;
}

/**
 * @brief 复位VQF状态
 */
void Class_Filter_VQF::Reset()
{
    Gyro_Quaternion = Namespace_ALG_Quaternion::Unit_Real();
    Accel_Quaternion = Namespace_ALG_Quaternion::Unit_Real();

    Last_Accel_Low_Pass = Namespace_ALG_Matrix::Zero<3, 1>();
    Reset_Filter_State(3U, Accel_Low_Pass_State);
    Last_Accel_Correction_Rate = 0.0f;

    Bias_Estimate = Namespace_ALG_Matrix::Zero<3, 1>();
    Bias_Covariance = Bias_Covariance_Init * Namespace_ALG_Matrix::Identity<3, 3>();
    Reset_Filter_State(9U, Motion_Rotation_Low_Pass_State);
    Reset_Filter_State(2U, Motion_Bias_Low_Pass_State);

    Rest_Detected = false;
    Rest_Time = 0.0f;
    Rest_Gyro_Squared_Deviation = 0.0f;
    Rest_Accel_Squared_Deviation = 0.0f;
    Rest_Last_Gyro_Low_Pass = Namespace_ALG_Matrix::Zero<3, 1>();
    Rest_Last_Accel_Low_Pass = Namespace_ALG_Matrix::Zero<3, 1>();
    Reset_Filter_State(3U, Rest_Gyro_Low_Pass_State);
    Reset_Filter_State(3U, Rest_Accel_Low_Pass_State);

    Last_Corrected_Gyro = Namespace_ALG_Matrix::Zero<3, 1>();
}

/**
 * @brief 更新陀螺仪数据
 *
 * @param gyro 固定误差校正后的角速度, 单位rad/s
 */
void Class_Filter_VQF::Update_Gyro(const Class_Matrix_f32<3, 1> &gyro)
{
    if (!Initialized_Flag || !Vector_Is_Valid(gyro))
    {
        return;
    }

    if (Parameter.Rest_Bias_Estimation_Enable)
    {
        float gyro_sample[3] = {gyro[0][0], gyro[1][0], gyro[2][0]};
        float gyro_low_pass[3] = {};
        Filter_Vector(gyro_sample,
                      3U,
                      Parameter.Rest_Filter_Tau,
                      Gyro_D_T,
                      Rest_Gyro_Low_Pass_B,
                      Rest_Gyro_Low_Pass_A,
                      Rest_Gyro_Low_Pass_State,
                      gyro_low_pass);

        for (uint32_t i = 0U; i < 3U; i++)
        {
            Rest_Last_Gyro_Low_Pass[i][0] = gyro_low_pass[i];
        }

        const Class_Matrix_f32<3, 1> deviation = gyro - Rest_Last_Gyro_Low_Pass;
        Rest_Gyro_Squared_Deviation = deviation[0][0] * deviation[0][0] +
                                      deviation[1][0] * deviation[1][0] +
                                      deviation[2][0] * deviation[2][0];

        const float gyro_threshold = Parameter.Rest_Threshold_Gyro_Deg_S * PI / 180.0f;
        const float bias_clip = Parameter.Bias_Clip_Deg_S * PI / 180.0f;
        if (Rest_Gyro_Squared_Deviation >= gyro_threshold * gyro_threshold ||
            fabsf(Rest_Last_Gyro_Low_Pass[0][0]) > bias_clip ||
            fabsf(Rest_Last_Gyro_Low_Pass[1][0]) > bias_clip ||
            fabsf(Rest_Last_Gyro_Low_Pass[2][0]) > bias_clip)
        {
            Rest_Time = 0.0f;
            Rest_Detected = false;
        }
    }

    Last_Corrected_Gyro = gyro - Bias_Estimate;
    const float gyro_norm = Last_Corrected_Gyro.Get_Modulus();
    const float angle = gyro_norm * Gyro_D_T;
    if (gyro_norm > 1.0e-9f)
    {
        const float scalar = cosf(0.5f * angle);
        const float vector_scale = sinf(0.5f * angle) / gyro_norm;
        const Class_Quaternion_f32 gyro_step(
            scalar,
            vector_scale * Last_Corrected_Gyro[0][0],
            vector_scale * Last_Corrected_Gyro[1][0],
            vector_scale * Last_Corrected_Gyro[2][0]);
        Gyro_Quaternion = Normalize_Quaternion(Gyro_Quaternion * gyro_step);
    }
}

/**
 * @brief 更新加速度计数据
 *
 * @param accel 加速度, 单位m/s^2
 */
void Class_Filter_VQF::Update_Accel(const Class_Matrix_f32<3, 1> &accel)
{
    if (!Initialized_Flag || !Vector_Is_Valid(accel) || accel.Get_Modulus() <= 1.0e-9f)
    {
        return;
    }

    if (Parameter.Rest_Bias_Estimation_Enable)
    {
        float accel_sample[3] = {accel[0][0], accel[1][0], accel[2][0]};
        float accel_low_pass[3] = {};
        Filter_Vector(accel_sample,
                      3U,
                      Parameter.Rest_Filter_Tau,
                      Accel_D_T,
                      Rest_Accel_Low_Pass_B,
                      Rest_Accel_Low_Pass_A,
                      Rest_Accel_Low_Pass_State,
                      accel_low_pass);

        for (uint32_t i = 0U; i < 3U; i++)
        {
            Rest_Last_Accel_Low_Pass[i][0] = accel_low_pass[i];
        }

        const Class_Matrix_f32<3, 1> deviation = accel - Rest_Last_Accel_Low_Pass;
        Rest_Accel_Squared_Deviation = deviation[0][0] * deviation[0][0] +
                                       deviation[1][0] * deviation[1][0] +
                                       deviation[2][0] * deviation[2][0];

        if (Rest_Accel_Squared_Deviation >=
            Parameter.Rest_Threshold_Accel * Parameter.Rest_Threshold_Accel)
        {
            Rest_Time = 0.0f;
            Rest_Detected = false;
        }
        else
        {
            Rest_Time += Accel_D_T;
            if (Rest_Time >= Parameter.Rest_Min_Time)
            {
                Rest_Detected = true;
            }
        }
    }

    const Class_Matrix_f32<3, 1> accel_gyro_earth =
        Gyro_Quaternion.Get_Rotation_Matrix() * accel;
    float accel_earth_input[3] = {
        accel_gyro_earth[0][0],
        accel_gyro_earth[1][0],
        accel_gyro_earth[2][0]};
    float accel_earth_low_pass[3] = {};
    Filter_Vector(accel_earth_input,
                  3U,
                  Parameter.Tau_Accel,
                  Accel_D_T,
                  Accel_Low_Pass_B,
                  Accel_Low_Pass_A,
                  Accel_Low_Pass_State,
                  accel_earth_low_pass);
    for (uint32_t i = 0U; i < 3U; i++)
    {
        Last_Accel_Low_Pass[i][0] = accel_earth_low_pass[i];
    }

    Class_Matrix_f32<3, 1> accel_earth =
        Accel_Quaternion.Get_Rotation_Matrix() * Last_Accel_Low_Pass;
    accel_earth = accel_earth.Get_Normalization();

    const float quaternion_real = sqrtf(Basic_Math_Constrain(
        0.5f * (accel_earth[2][0] + 1.0f), 0.0f, 1.0f));
    Class_Quaternion_f32 accel_correction;
    if (quaternion_real > 1.0e-6f)
    {
        accel_correction[0] = quaternion_real;
        accel_correction[1] = 0.5f * accel_earth[1][0] / quaternion_real;
        accel_correction[2] = -0.5f * accel_earth[0][0] / quaternion_real;
        accel_correction[3] = 0.0f;
    }
    else
    {
        accel_correction = Class_Quaternion_f32(0.0f, 1.0f, 0.0f, 0.0f);
    }
    Accel_Quaternion = Normalize_Quaternion(accel_correction * Accel_Quaternion);

    Last_Accel_Correction_Rate =
        acosf(Basic_Math_Constrain(accel_earth[2][0], -1.0f, 1.0f)) / Accel_D_T;

    Update_Bias(accel_earth);
}

/**
 * @brief 设置陀螺仪零偏估计
 *
 * @param bias 零偏, 单位rad/s
 * @param sigma 零偏标准差, 单位rad/s, 小于等于0时保持原协方差
 */
void Class_Filter_VQF::Set_Bias_Estimate(const Class_Matrix_f32<3, 1> &bias,
                                         float sigma)
{
    if (!Vector_Is_Valid(bias))
    {
        return;
    }

    const float bias_clip = Parameter.Bias_Clip_Deg_S * PI / 180.0f;
    for (uint32_t i = 0U; i < 3U; i++)
    {
        Bias_Estimate[i][0] = Basic_Math_Constrain(bias[i][0], -bias_clip, bias_clip);
    }

    if (sigma > 0.0f && !Basic_Math_Is_Invalid_Float(sigma))
    {
        const float sigma_internal = sigma * 180.0f * 100.0f / PI;
        Bias_Covariance = sigma_internal * sigma_internal *
                          Namespace_ALG_Matrix::Identity<3, 3>();
    }
}

/**
 * @brief 设置运动和静止零偏估计开关
 */
void Class_Filter_VQF::Set_Bias_Estimation_Enable(bool motion_enable,
                                                  bool rest_enable)
{
    const bool motion_changed =
        Parameter.Motion_Bias_Estimation_Enable != motion_enable;
    const bool rest_changed =
        Parameter.Rest_Bias_Estimation_Enable != rest_enable;
    Parameter.Motion_Bias_Estimation_Enable = motion_enable;
    Parameter.Rest_Bias_Estimation_Enable = rest_enable;

    if (motion_changed)
    {
        Reset_Filter_State(9U, Motion_Rotation_Low_Pass_State);
        Reset_Filter_State(2U, Motion_Bias_Low_Pass_State);
    }
    if (rest_changed)
    {
        Rest_Detected = false;
        Rest_Time = 0.0f;
        Rest_Gyro_Squared_Deviation = 0.0f;
        Rest_Accel_Squared_Deviation = 0.0f;
        Reset_Filter_State(3U, Rest_Gyro_Low_Pass_State);
        Reset_Filter_State(3U, Rest_Accel_Low_Pass_State);
    }
}

/**
 * @brief 获取陀螺仪零偏标准差上界
 *
 * @return float 标准差, 单位rad/s
 */
float Class_Filter_VQF::Get_Bias_Sigma() const
{
    const float row_sum_0 = fabsf(Bias_Covariance[0][0]) +
                            fabsf(Bias_Covariance[0][1]) +
                            fabsf(Bias_Covariance[0][2]);
    const float row_sum_1 = fabsf(Bias_Covariance[1][0]) +
                            fabsf(Bias_Covariance[1][1]) +
                            fabsf(Bias_Covariance[1][2]);
    const float row_sum_2 = fabsf(Bias_Covariance[2][0]) +
                            fabsf(Bias_Covariance[2][1]) +
                            fabsf(Bias_Covariance[2][2]);
    const float covariance = Basic_Math_Constrain(
        fmaxf(row_sum_0, fmaxf(row_sum_1, row_sum_2)),
        0.0f,
        Bias_Covariance_Init);
    return (sqrtf(covariance) * PI / (100.0f * 180.0f));
}

/**
 * @brief 计算VQF固定系数
 */
void Class_Filter_VQF::Calculate_Coefficient()
{
    Calculate_Filter_Coefficient(Parameter.Tau_Accel,
                                 Accel_D_T,
                                 Accel_Low_Pass_B,
                                 Accel_Low_Pass_A);
    Calculate_Filter_Coefficient(Parameter.Rest_Filter_Tau,
                                 Gyro_D_T,
                                 Rest_Gyro_Low_Pass_B,
                                 Rest_Gyro_Low_Pass_A);
    Calculate_Filter_Coefficient(Parameter.Rest_Filter_Tau,
                                 Accel_D_T,
                                 Rest_Accel_Low_Pass_B,
                                 Rest_Accel_Low_Pass_A);

    const float bias_sigma_init = Parameter.Bias_Sigma_Init_Deg_S * 100.0f;
    Bias_Covariance_Init = bias_sigma_init * bias_sigma_init;
    Bias_Process_Noise = 100.0f * Accel_D_T / Parameter.Bias_Forgetting_Time;

    const float motion_covariance =
        Parameter.Bias_Sigma_Motion_Deg_S * Parameter.Bias_Sigma_Motion_Deg_S * 10000.0f;
    Bias_Motion_Noise = motion_covariance * motion_covariance / Bias_Process_Noise +
                        motion_covariance;
    Bias_Vertical_Noise = Bias_Motion_Noise /
                          fmaxf(Parameter.Bias_Vertical_Forgetting_Factor, 1.0e-10f);

    const float rest_covariance =
        Parameter.Bias_Sigma_Rest_Deg_S * Parameter.Bias_Sigma_Rest_Deg_S * 10000.0f;
    Bias_Rest_Noise = rest_covariance * rest_covariance / Bias_Process_Noise +
                      rest_covariance;
}

/**
 * @brief 更新陀螺仪零偏估计
 *
 * @param accel_earth 归一化后的地理系加速度
 */
void Class_Filter_VQF::Update_Bias(const Class_Matrix_f32<3, 1> &accel_earth)
{
    if (!Parameter.Motion_Bias_Estimation_Enable &&
        !Parameter.Rest_Bias_Estimation_Enable)
    {
        return;
    }

    const Class_Matrix_f32<3, 3> rotation_now =
        Get_Quaternion_6D().Get_Rotation_Matrix();
    Class_Matrix_f32<2, 1> bias_earth_xy;
    bias_earth_xy[0][0] = rotation_now[0][0] * Bias_Estimate[0][0] +
                          rotation_now[0][1] * Bias_Estimate[1][0] +
                          rotation_now[0][2] * Bias_Estimate[2][0];
    bias_earth_xy[1][0] = rotation_now[1][0] * Bias_Estimate[0][0] +
                          rotation_now[1][1] * Bias_Estimate[1][0] +
                          rotation_now[1][2] * Bias_Estimate[2][0];

    float rotation_input[9] = {};
    float rotation_low_pass[9] = {};
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            rotation_input[row * 3U + column] = rotation_now[row][column];
        }
    }
    Filter_Vector(rotation_input,
                  9U,
                  Parameter.Tau_Accel,
                  Accel_D_T,
                  Accel_Low_Pass_B,
                  Accel_Low_Pass_A,
                  Motion_Rotation_Low_Pass_State,
                  rotation_low_pass);

    float bias_input[2] = {bias_earth_xy[0][0], bias_earth_xy[1][0]};
    float bias_low_pass[2] = {};
    Filter_Vector(bias_input,
                  2U,
                  Parameter.Tau_Accel,
                  Accel_D_T,
                  Accel_Low_Pass_B,
                  Accel_Low_Pass_A,
                  Motion_Bias_Low_Pass_State,
                  bias_low_pass);

    Class_Matrix_f32<3, 3> measurement_matrix;
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            measurement_matrix[row][column] = rotation_low_pass[row * 3U + column];
        }
    }

    Class_Matrix_f32<3, 1> error;
    Class_Matrix_f32<3, 1> measurement_noise;
    bool update_enable = false;
    if (Rest_Detected && Parameter.Rest_Bias_Estimation_Enable)
    {
        measurement_matrix = Namespace_ALG_Matrix::Identity<3, 3>();
        error = Rest_Last_Gyro_Low_Pass - Bias_Estimate;
        measurement_noise[0][0] = Bias_Rest_Noise;
        measurement_noise[1][0] = Bias_Rest_Noise;
        measurement_noise[2][0] = Bias_Rest_Noise;
        update_enable = true;
    }
    else if (Parameter.Motion_Bias_Estimation_Enable)
    {
        const Class_Matrix_f32<3, 1> predicted_bias = measurement_matrix * Bias_Estimate;
        error[0][0] = -accel_earth[1][0] / Accel_D_T +
                      bias_low_pass[0] - predicted_bias[0][0];
        error[1][0] = accel_earth[0][0] / Accel_D_T +
                      bias_low_pass[1] - predicted_bias[1][0];
        error[2][0] = -predicted_bias[2][0];
        measurement_noise[0][0] = Bias_Motion_Noise;
        measurement_noise[1][0] = Bias_Motion_Noise;
        measurement_noise[2][0] = Bias_Vertical_Noise;
        update_enable = true;
    }

    for (uint32_t i = 0U; i < 3U; i++)
    {
        if (Bias_Covariance[i][i] < Bias_Covariance_Init)
        {
            Bias_Covariance[i][i] += Bias_Process_Noise;
        }
    }

    if (!update_enable)
    {
        return;
    }

    const float bias_clip = Parameter.Bias_Clip_Deg_S * PI / 180.0f;
    for (uint32_t i = 0U; i < 3U; i++)
    {
        error[i][0] = Basic_Math_Constrain(error[i][0], -bias_clip, bias_clip);
    }

    Class_Matrix_f32<3, 3> covariance_measurement_transpose =
        Bias_Covariance * measurement_matrix.Get_Transpose();
    Class_Matrix_f32<3, 3> innovation_covariance =
        measurement_matrix * covariance_measurement_transpose;
    innovation_covariance[0][0] += measurement_noise[0][0];
    innovation_covariance[1][1] += measurement_noise[1][0];
    innovation_covariance[2][2] += measurement_noise[2][0];

    Class_Matrix_f32<3, 3> innovation_covariance_inverse;
    if (!Inverse_Matrix_3x3(innovation_covariance, innovation_covariance_inverse))
    {
        return;
    }

    Class_Matrix_f32<3, 3> gain =
        covariance_measurement_transpose * innovation_covariance_inverse;
    Bias_Estimate += gain * error;
    Bias_Covariance -= gain * measurement_matrix * Bias_Covariance;

    for (uint32_t i = 0U; i < 3U; i++)
    {
        Bias_Estimate[i][0] = Basic_Math_Constrain(
            Bias_Estimate[i][0], -bias_clip, bias_clip);
    }
}

/**
 * @brief 检查三维向量是否有效
 */
bool Class_Filter_VQF::Vector_Is_Valid(const Class_Matrix_f32<3, 1> &vector)
{
    return (!Basic_Math_Is_Invalid_Float(vector[0][0]) &&
            !Basic_Math_Is_Invalid_Float(vector[1][0]) &&
            !Basic_Math_Is_Invalid_Float(vector[2][0]));
}

/**
 * @brief 归一化四元数
 */
Class_Quaternion_f32 Class_Filter_VQF::Normalize_Quaternion(
    const Class_Quaternion_f32 &quaternion)
{
    const float modulus = quaternion.Get_Modulus();
    if (modulus <= 1.0e-9f || Basic_Math_Is_Invalid_Float(modulus))
    {
        return (Namespace_ALG_Quaternion::Unit_Real());
    }
    return (Class_Quaternion_f32(quaternion / modulus));
}

/**
 * @brief 计算二阶Butterworth低通滤波器系数
 */
void Class_Filter_VQF::Calculate_Filter_Coefficient(float tau,
                                                    float d_t,
                                                    double b[3],
                                                    double a[2])
{
    if (tau < 0.5f * d_t)
    {
        b[0] = 1.0;
        b[1] = 0.0;
        b[2] = 0.0;
        a[0] = 0.0;
        a[1] = 0.0;
        return;
    }

    const double cutoff_frequency =
        sqrt(2.0) / (2.0 * (double) PI * (double) tau);
    const double coefficient =
        tan((double) PI * cutoff_frequency * (double) d_t);
    const double denominator = coefficient * coefficient +
                               sqrt(2.0) * coefficient + 1.0;
    const double b0 = coefficient * coefficient / denominator;
    b[0] = b0;
    b[1] = 2.0 * b0;
    b[2] = b0;
    a[0] = 2.0 * (coefficient * coefficient - 1.0) / denominator;
    a[1] = (1.0 - sqrt(2.0) * coefficient +
            coefficient * coefficient) /
           denominator;
}

/**
 * @brief 复位向量滤波器状态
 */
void Class_Filter_VQF::Reset_Filter_State(uint32_t length,
                                          double state[])
{
    const double invalid = (double) NAN;
    for (uint32_t i = 0U; i < 2U * length; i++)
    {
        state[i] = invalid;
    }
}

/**
 * @brief 对向量各分量执行VQF二阶低通
 */
void Class_Filter_VQF::Filter_Vector(const float input[],
                                     uint32_t length,
                                     float tau,
                                     float d_t,
                                     const double b[3],
                                     const double a[2],
                                     double state[],
                                     float output[])
{
    if (isnan(state[0]))
    {
        if (isnan(state[1]))
        {
            state[1] = 0.0;
            for (uint32_t i = 0U; i < length; i++)
            {
                state[2U + i] = 0.0;
            }
        }

        state[1] += 1.0;
        for (uint32_t i = 0U; i < length; i++)
        {
            state[2U + i] += (double) input[i];
            output[i] = (float) (state[2U + i] / state[1]);
        }

        if ((float) state[1] * d_t >= tau)
        {
            for (uint32_t i = 0U; i < length; i++)
            {
                state[2U * i] =
                    (double) output[i] * (1.0 - b[0]);
                state[2U * i + 1U] =
                    (double) output[i] * (b[2] - a[1]);
            }
        }
        return;
    }

    for (uint32_t i = 0U; i < length; i++)
    {
        const double input_value = (double) input[i];
        const double output_value = b[0] * input_value + state[2U * i];
        state[2U * i] = b[1] * input_value - a[0] * output_value +
                        state[2U * i + 1U];
        state[2U * i + 1U] = b[2] * input_value - a[1] * output_value;
        output[i] = (float) output_value;
    }
}

/**
 * @brief 计算三阶方阵的逆矩阵
 */
bool Class_Filter_VQF::Inverse_Matrix_3x3(
    const Class_Matrix_f32<3, 3> &input,
    Class_Matrix_f32<3, 3> &output)
{
    const double a = input[0][0];
    const double b = input[0][1];
    const double c = input[0][2];
    const double d = input[1][0];
    const double e = input[1][1];
    const double f = input[1][2];
    const double g = input[2][0];
    const double h = input[2][1];
    const double i = input[2][2];

    const double cofactor_a = e * i - f * h;
    const double cofactor_b = c * h - b * i;
    const double cofactor_c = b * f - c * e;
    const double cofactor_d = f * g - d * i;
    const double cofactor_e = a * i - c * g;
    const double cofactor_f = c * d - a * f;
    const double cofactor_g = d * h - e * g;
    const double cofactor_h = b * g - a * h;
    const double cofactor_i = a * e - b * d;
    const double determinant =
        a * cofactor_a + b * cofactor_d + c * cofactor_g;

    if (fabs(determinant) <= 1.0e-12)
    {
        output = Namespace_ALG_Matrix::Zero<3, 3>();
        return (false);
    }

    const double inverse_determinant = 1.0 / determinant;
    output[0][0] = (float) (cofactor_a * inverse_determinant);
    output[0][1] = (float) (cofactor_b * inverse_determinant);
    output[0][2] = (float) (cofactor_c * inverse_determinant);
    output[1][0] = (float) (cofactor_d * inverse_determinant);
    output[1][1] = (float) (cofactor_e * inverse_determinant);
    output[1][2] = (float) (cofactor_f * inverse_determinant);
    output[2][0] = (float) (cofactor_g * inverse_determinant);
    output[2][1] = (float) (cofactor_h * inverse_determinant);
    output[2][2] = (float) (cofactor_i * inverse_determinant);
    return (true);
}
