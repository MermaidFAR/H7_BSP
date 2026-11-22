/**
 * @file alg_filter_ekf.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 扩展Kalman滤波器
 * @version 0.1
 * @date 2025-09-15 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef ALG_FILTER_EKF_H
#define ALG_FILTER_EKF_H

/* Includes ------------------------------------------------------------------*/

#include "alg_matrix.h"
#include <math.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 扩展Kalman滤波器算法
 *
 * 系统模型
 * x_k = f(x_{k-1}, u_k, w_k)
 * z_k = h(x_k, v_k)
 * 其中, w_k ~ N(0, Q), v_k ~ N(0, R)
 *
 * 每轮调用预测和更新函数前, 需要先更新输入向量Vector_U和测量向量Vector_Z
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 */
template<uint32_t State_Dimension = 2, uint32_t Input_Dimension = 1, uint32_t Measurement_Dimension = 2>
class Class_Filter_EKF
{
public:
    // 随着系统确定的
    // 系统状态函数
    Class_Matrix_f32<State_Dimension, 1> (*Function_F)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T) = nullptr;

    // 系统状态函数对状态的雅可比矩阵
    Class_Matrix_f32<State_Dimension, State_Dimension> (*Function_Jacobian_F_X)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T) = nullptr;

    // 系统状态函数对过程噪声的雅可比矩阵
    Class_Matrix_f32<State_Dimension, Input_Dimension> (*Function_Jacobian_F_W)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T) = nullptr;

    // 系统测量函数
    Class_Matrix_f32<Measurement_Dimension, 1> (*Function_H)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, float D_T) = nullptr;

    // 系统测量函数对状态的雅可比矩阵
    Class_Matrix_f32<Measurement_Dimension, State_Dimension> (*Function_Jacobian_H_X)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, float D_T) = nullptr;

    // 系统测量函数对测量噪声的雅可比矩阵
    Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> (*Function_Jacobian_H_V)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, float D_T) = nullptr;

    // 需要自己调参的
    // 过程噪声协方差矩阵
    Class_Matrix_f32<Input_Dimension, Input_Dimension> Matrix_Q;
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

    void Init(const Class_Matrix_f32<Input_Dimension, Input_Dimension> &matrix_q, const Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> &matrix_r, const Class_Matrix_f32<State_Dimension, State_Dimension> &matrix_p = Namespace_ALG_Matrix::Identity<State_Dimension, State_Dimension>(), const Class_Matrix_f32<State_Dimension, 1> &vector_x = Namespace_ALG_Matrix::Zero<State_Dimension, 1>(), const Class_Matrix_f32<Input_Dimension, 1> &matrix_u = Namespace_ALG_Matrix::Zero<Input_Dimension, 1>());

    inline void Config_Nonlinear_State_Model(Class_Matrix_f32<State_Dimension, 1> (*function_f)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T), Class_Matrix_f32<State_Dimension, State_Dimension> (*function_jacobian_f_x)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T), Class_Matrix_f32<State_Dimension, Input_Dimension> (*function_jacobian_f_w)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T));

    inline void Config_Nonlinear_Measurement_Model(Class_Matrix_f32<Measurement_Dimension, 1> (*function_h)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, float D_T), Class_Matrix_f32<Measurement_Dimension, State_Dimension> (*function_jacobian_h_x)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, float D_T), Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> (*function_jacobian_h_v)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, float D_T));

    void Set_D_T(float d_t);

    void TIM_Predict_PeriodElapsedCallback();

    bool TIM_Update_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 常量

    // 单位矩阵
    const Class_Matrix_f32<State_Dimension, State_Dimension> MATRIX_I_STATE = Namespace_ALG_Matrix::Identity<State_Dimension, State_Dimension>();
    // 零矩阵
    const Class_Matrix_f32<State_Dimension, State_Dimension> MATRIX_ZERO_STATE = Namespace_ALG_Matrix::Zero<State_Dimension, State_Dimension>();

    // 内部变量

    // 读变量

    // 写变量

    // 时间差, 单位s
    float D_T = 0.001f;

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化扩展Kalman滤波器
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 * @param matrix_q 过程噪声协方差矩阵
 * @param matrix_r 测量噪声协方差矩阵
 * @param vector_x 初始状态向量
 * @param matrix_u 初始输入向量
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
void Class_Filter_EKF<State_Dimension, Input_Dimension, Measurement_Dimension>::Init(const Class_Matrix_f32<Input_Dimension, Input_Dimension> &matrix_q, const Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> &matrix_r, const Class_Matrix_f32<State_Dimension, State_Dimension> &matrix_p, const Class_Matrix_f32<State_Dimension, 1> &vector_x, const Class_Matrix_f32<Input_Dimension, 1> &matrix_u)
{
    Matrix_Q = matrix_q;
    Matrix_R = matrix_r;

    Matrix_P = matrix_p;
    Vector_X = vector_x;
    Vector_U = matrix_u;
}

/**
 * @brief 设置非线性系统状态模型
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 * @param function_f 系统状态函数
 * @param function_jacobian_f_x 系统状态函数对状态的雅可比矩阵
 * @param function_jacobian_f_w 系统状态函数对过程噪声的雅可比矩阵
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
inline void Class_Filter_EKF<State_Dimension, Input_Dimension, Measurement_Dimension>::Config_Nonlinear_State_Model(Class_Matrix_f32 < State_Dimension, 1 > (*function_f)(const Class_Matrix_f32 < State_Dimension, 1 > &Vector_X,
const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, 
float D_T
)
,
Class_Matrix_f32<State_Dimension, State_Dimension> (*function_jacobian_f_x)(const Class_Matrix_f32<State_Dimension, 1> &Vector_X, const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, float D_T), Class_Matrix_f32<State_Dimension, Input_Dimension>(*function_jacobian_f_w)(
const Class_Matrix_f32<State_Dimension, 1> &Vector_X, 
const Class_Matrix_f32<Input_Dimension, 1> &Vector_U, 
float D_T
)
)
{
    Function_F = function_f;
    Function_Jacobian_F_X = function_jacobian_f_x;
    Function_Jacobian_F_W = function_jacobian_f_w;
}

/**
 * @brief 设置非线性系统测量模型
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 * @param function_h 系统测量函数
 * @param function_jacobian_h_x 系统测量函数对状态的雅可比矩阵
 * @param function_jacobian_h_v 系统测量函数对测量噪声的雅可比矩阵
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
inline void Class_Filter_EKF<State_Dimension, Input_Dimension, Measurement_Dimension>::Config_Nonlinear_Measurement_Model(Class_Matrix_f32 < Measurement_Dimension, 1 > (*function_h)(const Class_Matrix_f32 < State_Dimension, 1 > &Vector_X, float D_T), Class_Matrix_f32 < Measurement_Dimension, State_Dimension > (*function_jacobian_h_x)(const Class_Matrix_f32 < State_Dimension, 1 > &Vector_X, float D_T), Class_Matrix_f32 < Measurement_Dimension, Measurement_Dimension > (*function_jacobian_h_v)(const Class_Matrix_f32 < State_Dimension, 1 > &Vector_X, float D_T))
{
    Function_H = function_h;
    Function_Jacobian_H_X = function_jacobian_h_x;
    Function_Jacobian_H_V = function_jacobian_h_v;
}

/**
 * @brief 设置计算周期
 *
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
void Class_Filter_EKF<State_Dimension, Input_Dimension, Measurement_Dimension>::Set_D_T(float d_t)
{
    D_T = d_t;
}

/**
 * @brief 扩展Kalman滤波器预测步骤, 周期与采样周期相同
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
void Class_Filter_EKF<State_Dimension, Input_Dimension, Measurement_Dimension>::TIM_Predict_PeriodElapsedCallback()
{
    if (Function_F == nullptr || Function_Jacobian_F_X == nullptr ||
        Function_Jacobian_F_W == nullptr)
    {
        return;
    }

    // 计算Jacobi矩阵
    Class_Matrix_f32 < State_Dimension, State_Dimension > matrix_f_x = Function_Jacobian_F_X(Vector_X, Vector_U, D_T);
    Class_Matrix_f32 < State_Dimension, Input_Dimension > matrix_f_w = Function_Jacobian_F_W(Vector_X, Vector_U, D_T);

    // 预测状态向量
    Vector_X_Prior = Function_F(Vector_X, Vector_U, D_T);

    // 预测误差协方差矩阵
    Matrix_P_Prior = matrix_f_x * Matrix_P * matrix_f_x.Get_Transpose() + matrix_f_w * Matrix_Q * matrix_f_w.Get_Transpose();

    // 在未更新观测时, 也能有后验
    Vector_X = Vector_X_Prior;
    Matrix_P = Matrix_P_Prior;
}

/**
 * @brief 扩展Kalman滤波器更新步骤, 周期与采样周期相同
 * @note 每周期先预测；仅在本周期有有效测量时更新一次。
 * @note 求逆或计算失败时保留当前 X/P，清零 K，下一周期可继续预测。
 * @return true 测量修正成功；false 测量模型未配置、求逆失败或计算结果非有限。
 *
 * @tparam State_Dimension 状态维度
 * @tparam Input_Dimension 输入维度
 * @tparam Measurement_Dimension 测量维度
 */
template<uint32_t State_Dimension, uint32_t Input_Dimension, uint32_t Measurement_Dimension>
bool Class_Filter_EKF<State_Dimension, Input_Dimension, Measurement_Dimension>::TIM_Update_PeriodElapsedCallback()
{
    if (Function_H == nullptr || Function_Jacobian_H_X == nullptr ||
        Function_Jacobian_H_V == nullptr)
    {
        Matrix_K = Namespace_ALG_Matrix::Zero<State_Dimension, Measurement_Dimension>();
        return false;
    }

    // 计算Jacobi矩阵
    Class_Matrix_f32<Measurement_Dimension, State_Dimension> matrix_h_x = Function_Jacobian_H_X(Vector_X_Prior, D_T);
    Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> matrix_h_v = Function_Jacobian_H_V(Vector_X_Prior, D_T);
    Class_Matrix_f32<State_Dimension, Measurement_Dimension> matrix_h_x_t = matrix_h_x.Get_Transpose();
    Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> matrix_r =
        matrix_h_v * Matrix_R * matrix_h_v.Get_Transpose();

    // 计算Kalman增益矩阵
    bool success = false;
    Class_Matrix_f32<Measurement_Dimension, Measurement_Dimension> matrix_s_inverse =
        (matrix_h_x * Matrix_P_Prior * matrix_h_x_t + matrix_r).Get_Inverse(&success);
    if (!success)
    {
        Matrix_K = Namespace_ALG_Matrix::Zero<State_Dimension, Measurement_Dimension>();
        return false;
    }

    Class_Matrix_f32<State_Dimension, Measurement_Dimension> matrix_k =
        Matrix_P_Prior * matrix_h_x_t * matrix_s_inverse;

    // 更新状态向量
    Class_Matrix_f32<State_Dimension, 1> vector_x =
        Vector_X_Prior + matrix_k * (Vector_Z - Function_H(Vector_X_Prior, D_T));

    // 更新误差协方差矩阵, 为获得更好的数值稳定性, 采用约瑟夫形式
    Class_Matrix_f32<State_Dimension, State_Dimension> matrix_tmp = MATRIX_I_STATE - matrix_k * matrix_h_x;
    Class_Matrix_f32<State_Dimension, State_Dimension> matrix_p =
        matrix_tmp * Matrix_P_Prior * matrix_tmp.Get_Transpose() + matrix_k * matrix_r * matrix_k.Get_Transpose();
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
