/**
 * @file alg_rls.cpp
 * @brief 定长递推最小二乘辨识
 */

#include "alg_rls.h"

#include <float.h>
#include <math.h>

// 对称矩阵的 Cholesky 分解 P = L Lᵀ。
static bool RLS_Cholesky(const double matrix[][RLS_MAX_DIMENSION],
                         double lower[][RLS_MAX_DIMENSION], uint8_t dimension)
{
    for (uint8_t i = 0; i < dimension; i++)
    {
        for (uint8_t j = 0; j <= i; j++)
        {
            double value = matrix[j][i];
            for (uint8_t k = 0; k < j; k++)
            {
                value -= lower[i][k] * lower[j][k];
            }
            if (i == j)
            {
                if (!(value > 0.0) || !isfinite(value))
                {
                    return false;
                }
                lower[i][j] = sqrt(value);
            }
            else
            {
                lower[i][j] = value / lower[j][j];
            }
        }
    }
    return true;
}

bool Class_RLS::Init(uint8_t dimension, float forgetting_factor, float initial_covariance,
                     const float *initial_parameters)
{
    if (dimension == 0 || dimension > RLS_MAX_DIMENSION ||
        !isfinite(forgetting_factor) || forgetting_factor <= 0.0f || forgetting_factor > 1.0f ||
        !isfinite(initial_covariance) || initial_covariance <= 0.0f)
    {
        return (false);
    }

    for (uint8_t i = 0; i < dimension; i++)
    {
        if (initial_parameters != NULL && !isfinite(initial_parameters[i]))
        {
            return (false);
        }
    }

    Dimension = dimension;
    Forgetting_Factor = forgetting_factor;
    Initial_Covariance = initial_covariance;
    for (uint8_t i = 0; i < RLS_MAX_DIMENSION; i++)
    {
        Initial_Parameters[i] = i < dimension && initial_parameters != NULL ? initial_parameters[i] : 0.0f;
    }
    Initialized_Flag = true;
    Reset();
    return (true);
}

bool Class_RLS::Update(const float *features, float measurement)
{
    return Update_Sample(features, measurement, false);
}

bool Class_RLS::Update_Directional(const float *features, float measurement)
{
    return Update_Sample(features, measurement, true);
}

bool Class_RLS::Update_Sample(const float *features, float measurement, bool directional)
{
    if (!Initialized_Flag || features == NULL || !isfinite(measurement))
    {
        return (false);
    }

    double feature_norm_squared = 0.0;
    for (uint8_t i = 0; i < Dimension; i++)
    {
        if (!isfinite(features[i]))
        {
            return false;
        }
        feature_norm_squared += (double) features[i] * (double) features[i];
    }

    double covariance_feature[RLS_MAX_DIMENSION] = {};
    double new_parameters[RLS_MAX_DIMENSION];
    double new_covariance[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION];
    double lower[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION];
    const double (*prior)[RLS_MAX_DIMENSION] = Covariance;
    // φᵀφ 是该观测沿自身方向增加的精度，(1-λ)/P0 是初始精度的一次遗忘量。
    if (directional && Forgetting_Factor < 1.0 &&
        Initial_Covariance * feature_norm_squared >= 1.0 - Forgetting_Factor)
    {
        if (!RLS_Cholesky(Covariance, lower, Dimension))
        {
            return false;
        }
        // 解 L z = φ，得到 φᵀ P⁻¹ φ = zᵀz。观测只有一行，无需求逆或 SVD。
        double whitened[RLS_MAX_DIMENSION];
        double information = 0.0;
        for (uint8_t i = 0; i < Dimension; i++)
        {
            double value = features[i];
            for (uint8_t j = 0; j < i; j++)
            {
                value -= lower[i][j] * whitened[j];
            }
            whitened[i] = value / lower[i][i];
            information += whitened[i] * whitened[i];
        }
        if (!(information > 0.0) || !isfinite(information))
        {
            return false;
        }
        // SIFt-RLS 式 (28)：测量前只沿 φ 增加协方差。
        const double direction_scale = ((1.0 - Forgetting_Factor) / Forgetting_Factor) / information;
        for (uint8_t i = 0; i < Dimension; i++)
        {
            for (uint8_t j = i; j < Dimension; j++)
            {
                const double value = Covariance[i][j] + direction_scale * (double) features[i] * (double) features[j];
                new_covariance[i][j] = value;
                new_covariance[j][i] = value;
            }
        }
        prior = new_covariance;
    }

    double prediction = 0.0;
    const double uniform_forgetting = directional ? 1.0 : Forgetting_Factor;
    double denominator = uniform_forgetting;
    for (uint8_t i = 0; i < Dimension; i++)
    {
        prediction += (double) features[i] * Parameters[i];
        for (uint8_t j = 0; j < Dimension; j++)
        {
            covariance_feature[i] += prior[i][j] * (double) features[j];
        }
        denominator += (double) features[i] * covariance_feature[i];
    }
    if (!isfinite(denominator) || denominator <= 0.0)
    {
        return false;
    }

    const double correction = ((double) measurement - prediction) / denominator;
    const double inverse_forgetting = 1.0 / uniform_forgetting;
    for (uint8_t i = 0; i < Dimension; i++)
    {
        new_parameters[i] = Parameters[i] + covariance_feature[i] * correction;
        const double gain = covariance_feature[i] / denominator;
        for (uint8_t j = i; j < Dimension; j++)
        {
            // 取更新矩阵及其转置的平均，只计算一次对称的秩一修正。
            const double symmetric = 0.5 * prior[i][j] + 0.5 * prior[j][i];
            const double value = (symmetric - gain * covariance_feature[j]) * inverse_forgetting;
            new_covariance[i][j] = value;
            new_covariance[j][i] = value;
        }
    }

    // 候选协方差在舍入后仍须正定，参数须能由 float 接口返回。
    if (!RLS_Cholesky(new_covariance, lower, Dimension))
    {
        return false;
    }
    for (uint8_t i = 0; i < Dimension; i++)
    {
        if (!(fabs(new_parameters[i]) <= (double) FLT_MAX))
        {
            return false;
        }
    }

    for (uint8_t i = 0; i < Dimension; i++)
    {
        Parameters[i] = new_parameters[i];
        for (uint8_t j = 0; j < Dimension; j++)
        {
            Covariance[i][j] = new_covariance[i][j];
        }
    }
    return (true);
}

void Class_RLS::Reset()
{
    if (!Initialized_Flag)
    {
        return;
    }

    for (uint8_t i = 0; i < RLS_MAX_DIMENSION; i++)
    {
        Parameters[i] = Initial_Parameters[i];
        for (uint8_t j = 0; j < RLS_MAX_DIMENSION; j++)
        {
            Covariance[i][j] = i == j && i < Dimension ? Initial_Covariance : 0.0;
        }
    }
}

bool Class_RLS::Get_Parameters(float *out, uint8_t capacity) const
{
    if (!Initialized_Flag || out == NULL || capacity < Dimension)
    {
        return (false);
    }
    for (uint8_t i = 0; i < Dimension; i++)
    {
        out[i] = (float) Parameters[i];
    }
    return (true);
}

float Class_RLS::Get_Parameter(uint8_t index) const
{
    if (!Initialized_Flag || index >= Dimension)
    {
        return (0.0f);
    }
    return ((float) Parameters[index]);
}

uint8_t Class_RLS::Get_Dimension() const
{
    return (Dimension);
}

bool Class_RLS::Get_Initialized_Flag() const
{
    return (Initialized_Flag);
}
