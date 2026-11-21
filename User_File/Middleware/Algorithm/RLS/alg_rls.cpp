/**
 * @file alg_rls.cpp
 * @brief 定长递推最小二乘辨识
 */

#include "alg_rls.h"

#include <float.h>
#include <math.h>

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
    if (!Initialized_Flag || features == NULL || !isfinite(measurement))
    {
        return (false);
    }

    for (uint8_t i = 0; i < Dimension; i++)
    {
        if (!isfinite(features[i]))
        {
            return false;
        }
    }

    double covariance_feature[RLS_MAX_DIMENSION] = {};
    double new_parameters[RLS_MAX_DIMENSION];
    double new_covariance[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION];
    double prediction = 0.0;
    double denominator = Forgetting_Factor;
    for (uint8_t i = 0; i < Dimension; i++)
    {
        prediction += (double)features[i] * Parameters[i];
        for (uint8_t j = 0; j < Dimension; j++)
        {
            covariance_feature[i] += Covariance[i][j] * (double)features[j];
        }
        denominator += (double)features[i] * covariance_feature[i];
    }
    if (!isfinite(denominator) || denominator <= 0.0)
    {
        return false;
    }

    const double correction = ((double)measurement - prediction) / denominator;
    const double inverse_forgetting = 1.0 / Forgetting_Factor;
    for (uint8_t i = 0; i < Dimension; i++)
    {
        new_parameters[i] = Parameters[i] + covariance_feature[i] * correction;
        const double gain = covariance_feature[i] / denominator;
        for (uint8_t j = i; j < Dimension; j++)
        {
            // 取更新矩阵及其转置的平均，只计算一次对称的秩一修正。
            const double symmetric = 0.5 * Covariance[i][j] + 0.5 * Covariance[j][i];
            const double value = (symmetric - gain * covariance_feature[j]) * inverse_forgetting;
            new_covariance[i][j] = value;
            new_covariance[j][i] = value;
        }
    }

    // 候选状态完整校验后再提交；参数必须能由 float 接口返回。
    for (uint8_t i = 0; i < Dimension; i++)
    {
        if (!(fabs(new_parameters[i]) <= (double)FLT_MAX) || new_covariance[i][i] <= 0.0)
        {
            return false;
        }
        for (uint8_t j = i; j < Dimension; j++)
        {
            if (!isfinite(new_covariance[i][j]))
            {
                return false;
            }
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
        out[i] = (float)Parameters[i];
    }
    return (true);
}

float Class_RLS::Get_Parameter(uint8_t index) const
{
    if (!Initialized_Flag || index >= Dimension)
    {
        return (0.0f);
    }
    return ((float)Parameters[index]);
}

uint8_t Class_RLS::Get_Dimension() const
{
    return (Dimension);
}

bool Class_RLS::Get_Initialized_Flag() const
{
    return (Initialized_Flag);
}
