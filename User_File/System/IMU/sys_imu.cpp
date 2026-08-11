/**
 * @file sys_imu.cpp
 * @author zzm
 * @brief IMU系统级参数配置
 * @version 1.0
 * @date 2026-08-12
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_imu.h"

#include "bsp_bmi088.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 设置BMI088使用的VQF姿态与零偏估计参数
 *
 * @details
 * 此处只选择整机参数并交给BMI088，不包含传感器操作或算法实现。
 * 参数在BSP_BMI088.Init()中用于初始化VQF，因此必须先配置、后初始化。
 */
void System_IMU_Configure()
{
    Struct_BMI088_VQF_Config config;

    // VQF更新周期。陀螺仪按2 kHz逐帧积分，加速度计按250 Hz修正重力方向。
    config.Gyro_D_T = 0.0005f;
    config.Accel_D_T = 0.004f;

    // 加速度修正时间常数越大，Pitch/Roll越平滑，但收敛到重力方向越慢。
    config.Parameter.Tau_Accel = 3.0f;

    // 运动和静止零偏估计同时启用：运动时缓慢跟踪，静止确认后提高可信度。
    config.Parameter.Motion_Bias_Estimation_Enable = true;
    config.Parameter.Rest_Bias_Estimation_Enable = true;

    // 初始零偏不确定度。数值越大，启动阶段允许零偏估计更快调整。
    config.Parameter.Bias_Sigma_Init_Deg_S = 0.5f;

    // 零偏遗忘时间越长，长期估计越稳定，但温漂变化后的重新收敛越慢。
    config.Parameter.Bias_Forgetting_Time = 100.0f;

    // 限制零偏估计和残差的最大幅度，避免真实转动被误当作零偏。
    config.Parameter.Bias_Clip_Deg_S = 2.0f;

    // 运动状态下的零偏观测噪声；越小越信任运动过程中的估计。
    config.Parameter.Bias_Sigma_Motion_Deg_S = 0.1f;

    // 降低运动时重力轴方向零偏的观测权重，越小越保守。
    config.Parameter.Bias_Vertical_Forgetting_Factor = 0.0001f;

    // 静止状态下的零偏观测噪声；小于运动值，使静止零偏更新更可信。
    config.Parameter.Bias_Sigma_Rest_Deg_S = 0.03f;

    // 陀螺仪和加速度计连续满足静止条件1.5 s后，才进入静止零偏估计。
    config.Parameter.Rest_Min_Time = 1.5f;

    // 静止检测低通时间常数，抑制瞬时振动导致的状态反复切换。
    config.Parameter.Rest_Filter_Tau = 0.5f;

    // 静止门限：角速度残差不超过3.5 deg/s，加速度残差不超过0.5 m/s^2。
    config.Parameter.Rest_Threshold_Gyro_Deg_S = 3.5f;
    config.Parameter.Rest_Threshold_Accel = 0.5f;

    BSP_BMI088.Set_VQF_Config(config);
}
