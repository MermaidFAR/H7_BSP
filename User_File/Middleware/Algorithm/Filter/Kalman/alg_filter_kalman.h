/**
 * @file alg_filter_kalman.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief Kalman滤波器
 * @version 0.1
 * @date 2025-09-07 0.1 新增
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef ALG_FILTER_KALMAN_H
#define ALG_FILTER_KALMAN_H

/* Includes ------------------------------------------------------------------*/

#include "alg_matrix.h"
#include <math.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, Kalman滤波器算法
 *
 * 系统模型
 * x_k = A * x_{k-1} + B * u_k + w_k
 * z_k = H * x_k + v_k
 * 其中, w_k ~ N(0, Q), v_k ~ N(0, R)
 *
 * 每轮调用预测和更新函数前, 需要先更新输入向量Vector_U和测量向量Vector_Z
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 */
template<uint32_t State_Dimension = 2, uint32_t Input_Dimension = 1, uint32_t Measurement_Dimension = 2>
class Class_Filter_Kalman
{
public:
    // 随着系统确定的
    // 系统状态矩阵
    Class_Matrix_f32<State_Dimension, State_Dimension> Matrix_A;
    Class_Matrix_f32<State_Dimension, Input_Dimension> Matrix_B;
    // 测量矩阵
    Class_Matrix_f32<Measurement_Dimension, State_Dimension> Matrix_H;

    // 需要自己调参的
    // 过程噪声协方差矩阵
    Class_Matrix_f32<State_Dimension, State_Dimension> Matrix_Q;
    // 测量噪声协方差矩阵
    Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> Matrix_R;

    // 内部运算需要的
    // 误差协方差矩阵
    Class_Matrix_f32<State_Dimension, State_Dimension> Matrix_P;
    // 状态向量
    Class_Matrix_f32<State_Dimension, 1> Vector_X;
    // 输入向量
    Class_Matrix_f32<Input_Dimension, 1> Vector_U;
    // 先验估计状态向量
    Class_Matrix_f32<State_Dimension, 1> Vector_X_Prior;
    // 先验误差协方差矩阵
    Class_Matrix_f32<State_Dimension, State_Dimension> Matrix_P_Prior;
    // Kalman增益矩阵
    Class_Matrix_f32<State_Dimension, Measurement_Dimension> Matrix_K;

    // 传感器拿到的
    // 测量向量
    Class_Matrix_f32<Measurement_Dimension, 1> Vector_Z;

    void Init(const Class_Matrix_f32<State_Dimension, State_Dimension> &__Matrix_A, const Class_Matrix_f32<State_Dimension, Input_Dimension> &__Matrix_B, const Class_Matrix_f32<Measurement_Dimension, State_Dimension> &__Matrix_H, const Class_Matrix_f32<State_Dimension, State_Dimension> &__Matrix_Q, const Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> &__Matrix_R, const Class_Matrix_f32<State_Dimension, State_Dimension> &__Matrix_P = Namespace_ALG_Matrix::Identity<State_Dimension, State_Dimension>(), const Class_Matrix_f32<State_Dimension, 1> &__Vector_X = Namespace_ALG_Matrix::Zero<State_Dimension, 1>(), const Class_Matrix_f32<Input_Dimension, 1> &__Matrix_U = Namespace_ALG_Matrix::Zero<Input_Dimension, 1>());

    void TIM_Predict_PeriodElapsedCallback();

    bool TIM_Update_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 常量

    // 单位矩阵
    const Class_Matrix_f32<State_Dimension, State_Dimension> MATRIX_I_STATE = Namespace_ALG_Matrix::Identity<State_Dimension, State_Dimension>();

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化Kalman滤波器
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 * @param __Matrix_A 系统状态矩阵
 * @param __Matrix_B 控制矩阵
 * @param __Matrix_H 测量矩阵
 * @param __Matrix_Q 过程噪声协方差矩阵
 * @param __Matrix_R 测量噪声协方差矩阵
 * @param __Vector_X 初始状态向量
 * @param __Matrix_U 初始输入向量
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
void Class_Filter_Kalman<State_Dimension, Input_Dimension, Measurement_Dimension>::Init(const Class_Matrix_f32<State_Dimension, State_Dimension> &__Matrix_A, const Class_Matrix_f32<State_Dimension, Input_Dimension> &__Matrix_B, const Class_Matrix_f32<Measurement_Dimension, State_Dimension> &__Matrix_H, const Class_Matrix_f32<State_Dimension, State_Dimension> &__Matrix_Q, const Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> &__Matrix_R, const Class_Matrix_f32<State_Dimension, State_Dimension> &__Matrix_P, const Class_Matrix_f32<State_Dimension, 1> &__Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &__Matrix_U)
{
    Matrix_A = __Matrix_A;
    Matrix_B = __Matrix_B;
    Matrix_H = __Matrix_H;

    Matrix_Q = __Matrix_Q;
    Matrix_R = __Matrix_R;

    Matrix_P = __Matrix_P;
    Vector_X = __Vector_X;
    Vector_U = __Matrix_U;
}

/**
 * @brief Kalman滤波器预测步骤, 周期与采样周期相同
 * @note 预测后 X/P 即为当前估计；缺测时跳过更新，下一次预测继续推进状态与协方差。
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
void Class_Filter_Kalman<State_Dimension, Input_Dimension, Measurement_Dimension>::TIM_Predict_PeriodElapsedCallback()
{
    // 预测状态向量
    Vector_X_Prior = Matrix_A * Vector_X + Matrix_B * Vector_U;

    // 预测误差协方差矩阵
    Matrix_P_Prior = Matrix_A * Matrix_P * Matrix_A.Get_Transpose() + Matrix_Q;

    Vector_X = Vector_X_Prior;
    Matrix_P = Matrix_P_Prior;
}

/**
 * @brief Kalman滤波器更新步骤, 周期与采样周期相同
 * @note 每周期先预测；仅在本周期有有效测量时更新一次。
 * @note 求逆或计算失败时保留当前 X/P，清零 K，下一周期可继续预测。
 * @return true 测量修正成功；false 求逆失败或计算结果非有限。
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
bool Class_Filter_Kalman<State_Dimension, Input_Dimension, Measurement_Dimension>::TIM_Update_PeriodElapsedCallback()
{
    bool success = false;
    Class_Matrix_f32<State_Dimension, Measurement_Dimension> matrix_h_t = Matrix_H.Get_Transpose();
    Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> matrix_s_inverse =
        (Matrix_H * Matrix_P_Prior * matrix_h_t + Matrix_R).Get_Inverse(&success);
    if (!success)
    {
        Matrix_K = Namespace_ALG_Matrix::Zero<State_Dimension, Measurement_Dimension>();
        return false;
    }

    Class_Matrix_f32<State_Dimension, Measurement_Dimension> matrix_k =
        Matrix_P_Prior * matrix_h_t * matrix_s_inverse;
    Class_Matrix_f32<State_Dimension, 1> vector_x =
        Vector_X_Prior + matrix_k * (Vector_Z - Matrix_H * Vector_X_Prior);
    Class_Matrix_f32<State_Dimension, State_Dimension> matrix_tmp = MATRIX_I_STATE - matrix_k * Matrix_H;
    Class_Matrix_f32<State_Dimension, State_Dimension> matrix_p =
        matrix_tmp * Matrix_P_Prior * matrix_tmp.Get_Transpose() + matrix_k * Matrix_R * matrix_k.Get_Transpose();

    if (!vector_x.Is_Finite() || !matrix_p.Is_Finite())
    {
        Matrix_K = Namespace_ALG_Matrix::Zero<State_Dimension, Measurement_Dimension>();
        return false;
    }

    Matrix_K = matrix_k;
    Vector_X = vector_x;
    Matrix_P = matrix_p;
    return true;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
