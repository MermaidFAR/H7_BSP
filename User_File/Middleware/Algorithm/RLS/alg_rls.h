/**
 * @file alg_rls.h
 * @brief 定长递推最小二乘辨识
 */

#ifndef ALG_RLS_H
#define ALG_RLS_H

#include <stdint.h>
#include <stddef.h>

#define RLS_MAX_DIMENSION 6

class Class_RLS
{
public:
    bool Init(uint8_t dimension, float forgetting_factor, float initial_covariance,
              const float *initial_parameters = NULL);
    bool Update(const float *features, float measurement);
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
};

#endif
