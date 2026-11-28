/**
 * @file alg_rls.h
 * @brief 定长递推最小二乘辨识
 */

#ifndef ALG_RLS_H
#define ALG_RLS_H

#include <stddef.h>
#include <stdint.h>

#define RLS_MAX_DIMENSION 6

/** @brief 对线性模型 y = φᵀθ 作定长递推最小二乘估计。 @author zzm */
class Class_RLS
{
  public:
    /**
     * @brief 配置遗忘因子和初始协方差 P = initial_covariance*I。
     * @param dimension 特征和参数个数，范围为 1 到 RLS_MAX_DIMENSION。
     * @param forgetting_factor 取值 (0, 1]，越小越重视近期样本。
     * @param initial_covariance 初始协方差的正对角值。
     * @param initial_parameters 初始参数数组，长度为 dimension。传 NULL 时从零开始。
     * @author zzm
     */
    bool Init(uint8_t dimension, float forgetting_factor, float initial_covariance,
              const float *initial_parameters = NULL);
    /**
     * @brief 用一组时间对齐的特征和测量更新参数与协方差。
     * @param features 特征数组，长度为初始化时的 dimension。
     * @param measurement 对应的标量观测值。
     * @return true 已接纳该样本。false 时保留上次成功更新的状态。
     * @author zzm
     */
    bool Update(const float *features, float measurement);
    /**
     * @brief 标量观测的 SIFt-RLS 更新，仅在本帧特征提供信息的方向遗忘。
     * @note P0 * ‖φ‖² 小于 1 - λ 时只执行测量更新，以免微弱输入持续放大方差。
     *       P0 为初始协方差标量。λ = 1 时与 Update 相同。
     * @see https://arxiv.org/abs/2404.10844
     */
    bool Update_Directional(const float *features, float measurement);
    void Reset();
    bool Get_Parameters(float *out, uint8_t capacity) const;
    float Get_Parameter(uint8_t index) const;
    uint8_t Get_Dimension() const;
    bool Get_Initialized_Flag() const;

  protected:
    double Parameters[RLS_MAX_DIMENSION] = {};
    double Initial_Parameters[RLS_MAX_DIMENSION] = {};
    double Covariance[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION] = {};
    double Initial_Covariance = 0.0;
    double Forgetting_Factor = 1.0;
    uint8_t Dimension = 0;
    bool Initialized_Flag = false;

  private:
    bool Update_Sample(const float *features, float measurement, bool directional);
};

#endif
