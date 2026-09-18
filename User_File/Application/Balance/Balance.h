#if BALANCE

#ifndef __BALANCE_H
#define __BALANCE_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "alg_lqr.h"
#include "alg_pid.h"
#include "sys_timestamp.h"

/* Exported macros -----------------------------------------------------------*/

// ---- 控制周期 ----

// 控制周期 [s]，与 Control_Task 的 1 kHz 调度一致（位置参考斜坡的积分步长）
#define BALANCE_CONTROL_PERIOD_S (0.001f)

// ---- 位置参考防饱和 ----

// 位置参考允许领先实际位置的最大距离 [m]
#define BALANCE_POSITION_REF_ERROR_MAX (0.3f)

// ---- 电机安装符号 ----
// 命令与反馈共用同一份定义，保证符号约定一致。
// 左 = FDCAN1（id=1）；右 = FDCAN2（id=0，镜像安装，实测正电流转向相反）。
#define BALANCE_MOTOR_LEFT_SIGN (+1.0f)
#define BALANCE_MOTOR_RIGHT_SIGN (-1.0f)

// ---- 在线调参的安全区间 ----
// 不信任串口来源：非法值拒绝，越界值限幅。上限留出试错余量但挡掉危险值。
#define BALANCE_MAX_YAW_SPEED_MIN (0.0f)
#define BALANCE_MAX_YAW_SPEED_MAX (15.0f)   // rad/s，约 2.4 转/秒
#define BALANCE_YAW_K_P_MIN (0.0f)
#define BALANCE_YAW_K_P_MAX (0.5f)          // 整定值 0.05 的 10 倍
#define BALANCE_YAW_K_I_MIN (0.0f)
#define BALANCE_YAW_K_I_MAX (0.5f)          // 整定值 0.05 的 10 倍

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 轮组多圈累计器
 * @note  电机反馈角度是 0~2π 单圈值，每转一圈跳回 0，必须展开成连续角。
 */
typedef struct
{
    float Last_Raw_Angle;   // 上一次原始角 [rad]，范围 0~2π
    float Accumulated;      // 多圈累计角 [rad]
    bool Initialized;       // 首次收到有效反馈时只记基准，不累计
} Struct_Wheel_Accumulator;

/**
 * @brief 状态量调试镜像
 * @note  成员取地址即可直接喂给 EricTool/USB 遥测，也可在 Ozone 里 Watch。
 */
typedef struct
{
    float Theta;                // rad，前倾为正
    float Theta_Dot;            // rad/s
    float Position;             // m，前进为正
    float Velocity;             // m/s
    float Out;                  // A，LQR 输出的共模电流，正 = 车前进
    float Current_Diff;         // A，差模电流（转向）
    float Yaw_Speed;            // rad/s，实测偏航角速度
    float Target_Yaw_Speed;     // rad/s，目标偏航角速度
    float Yaw_Input;            // 归一化转向指令 [-1, 1]，超时后归零
    float Max_Yaw_Speed;        // rad/s，当前生效的转向上限
    float Yaw_K_P;              // 当前生效的偏航环 Kp
    float Yaw_K_I;              // 当前生效的偏航环 Ki
    float Move_Input;           // 归一化前进指令 [-1, 1]，正 = 前进，超时后归零
    float Position_Reference;   // m，位置参考（与 Position 一起看就知道跟没跟上）
} Struct_Balance_Debug;

/**
 * @brief Specialized, 平衡车状态解算
 *
 * @note  状态量严格遵循建模约定：SI 单位，θ 前倾为正、p 前进为正。
 * @note  输出按【共模 / 差模】分解，由 Control_Task 合成左右轮电流：
 *        i_common 正 = 车前进（LQR 输出），i_diff 正 = 车右转（偏航环输出）。
 */
class Class_Balance
{
public:
    void Init();

    // 输入：由 Control_Task 把电机原始反馈喂进来
    inline void Set_Wheel_Left(const float &__Raw_Angle_Rad, const float &__Speed_Rpm, const bool &__Valid);
    inline void Set_Wheel_Right(const float &__Raw_Angle_Rad, const float &__Speed_Rpm, const bool &__Valid);

    // 遥控输入（归一化 [-1, 1]），时间戳为该指令所属帧的收帧时刻，用于超时判定
    inline void Set_Yaw_Input(const float &__Yaw_Input, const uint64_t &__Timestamp);
    inline void Set_Move_Input(const float &__Move_Input, const uint64_t &__Timestamp);

    // 在线调参（下行指令调用；内部拒绝非法值、限幅越界值）
    inline void Set_Max_Yaw_Speed(const float &__Max_Yaw_Speed);
    inline void Set_Yaw_K_P(const float &__K_P);
    inline void Set_Yaw_K_I(const float &__K_I);

    // 输出：供 Control_Task 合成左右轮电流
    inline float Get_Current() const;
    inline float Get_Current_Diff() const;

    // 1ms 周期解算
    void TIM_1ms_Calculate_PeriodElapsedCallback();

    // 状态量调试镜像
    Struct_Balance_Debug Debug;

protected:
    // 初始化相关常量

    // 轮半径 [m]，用于弧度 → 米
    float Wheel_Radius = 0.035f;
    // 平衡姿态下的 IMU 俯仰读数 [rad]，实测 -0.08
    float Theta_Zero_Offset = -0.08f;

    // ---- 状态量 x = [θ, θ̇, p, ṗ] ----

    float Theta = 0.0f;         // rad，前倾为正
    float Theta_Dot = 0.0f;     // rad/s
    float Position = 0.0f;      // m，前进为正
    float Velocity = 0.0f;      // m/s

    // ---- 电机原始反馈 ----

    float Raw_Angle_Left = 0.0f;
    float Speed_Rpm_Left = 0.0f;
    bool Valid_Left = false;

    float Raw_Angle_Right = 0.0f;
    float Speed_Rpm_Right = 0.0f;
    bool Valid_Right = false;

    // ---- 平衡（LQR）----

    // 状态顺序 [θ(rad), θ̇(rad/s), p(m), ṗ(m/s)]
    Class_LQR<4> LQR;
    // 归一化前进/后退输入 [-1, 1]，正 = 前进
    float Move_Input = 0.0f;
    // 前进/后退指令的输入比例尺 [m/s]：v_ref = Move_Input × Max_Velocity
    float Max_Velocity = 1.0f;
    // 线速度参考 [m/s]
    float Velocity_Reference = 0.0f;
    // 位置参考 [m]：每毫秒沿 v_ref 累加，形成匀速移动的参考点
    float Position_Reference = 0.0f;
    // 最近一次"有效前进指令"的到达时刻 [us]
    uint64_t Move_Input_Timestamp = 0;

    // ---- 转向（偏航角速度环）----

    // 被控对象自带积分器 → P 起主要作用，Ki 消除摩擦造成的稳态误差
    Class_PID Yaw_Speed_PID;
    // 归一化转向输入 [-1, 1]，正 = 右转
    float Yaw_Input = 0.0f;
    // 目标偏航角速度上限 [rad/s]
    float Max_Yaw_Speed = 1.5f;
    // 目标偏航角速度 [rad/s]
    float Target_Yaw_Speed = 0.0f;
    // 差模电流输出 [A]，正 = 右转
    float Current_Diff = 0.0f;
    // 最近一次"有效转向指令"的到达时刻 [us]
    uint64_t Yaw_Input_Timestamp = 0;

    // ---- 遥控指令有效期 ----

    // [us]，超时未刷新即视为链路失联，对应输入归零
    uint64_t Command_Timeout_Us = 1000000ULL;

    // ---- 内部变量 ----

    Struct_Wheel_Accumulator Accum_Left = {};
    Struct_Wheel_Accumulator Accum_Right = {};

    // ---- 内部函数：1ms 回调按职责拆分为下列步骤 ----

    void Update_States();
    void Update_Command_Timeout();
    void Update_Yaw_Loop();
    void Update_Lqr_Reference();
    void Update_Debug();

    /**
     * @brief 归一化遥控指令的统一入口
     *
     * @param __Input           被写入的输入量引用
     * @param __Input_Timestamp 被写入的时间戳引用
     * @param __Value           外部传入的值（不可信）
     * @param __Value_Timestamp 该值所属帧的收帧时刻 [us]
     *
     * @note 非法浮点按"停机"处理；非法时不刷新时间戳，保持"最后一条有效指令"的语义，
     *       避免一路 NaN 让旧指令永不超时。
     */
    static inline void Apply_Normalized_Input(float &__Input, uint64_t &__Input_Timestamp,
                                             const float &__Value, const uint64_t &__Value_Timestamp);

    /**
     * @brief 在线调参的统一校验：非法浮点拒绝，越界值限幅
     *
     * @param __Value 引用，就地修正
     * @param __Min   允许下限
     * @param __Max   允许上限
     * @return bool true = 值合法（可能已被限幅），false = 值非法（未修改）
     */
    static inline bool Sanitize_Tunable(float &__Value, const float &__Min, const float &__Max);
};

/* Exported variables --------------------------------------------------------*/

extern Class_Balance Balance;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置左侧（FDCAN1, id=1）电机原始反馈
 *
 * @param __Raw_Angle_Rad 电机反馈角 [rad]，0~2π 单圈值
 * @param __Speed_Rpm     电机反馈转速 [rpm]
 * @param __Valid         反馈是否有效（电机已使能即视为有效）
 */
inline void Class_Balance::Set_Wheel_Left(const float &__Raw_Angle_Rad, const float &__Speed_Rpm, const bool &__Valid)
{
    Raw_Angle_Left = __Raw_Angle_Rad;
    Speed_Rpm_Left = __Speed_Rpm;
    Valid_Left = __Valid;
}

/**
 * @brief 设置右侧（FDCAN2, id=0）电机原始反馈
 */
inline void Class_Balance::Set_Wheel_Right(const float &__Raw_Angle_Rad, const float &__Speed_Rpm, const bool &__Valid)
{
    Raw_Angle_Right = __Raw_Angle_Rad;
    Speed_Rpm_Right = __Speed_Rpm;
    Valid_Right = __Valid;
}

/**
 * @brief 获取 LQR 输出的共模电流 [A]，正 = 车前进
 */
inline float Class_Balance::Get_Current() const
{
    return (LQR.Get_Out());
}

/**
 * @brief 获取差模电流（转向输出）[A]，正 = 右转
 */
inline float Class_Balance::Get_Current_Diff() const
{
    return (Current_Diff);
}

/**
 * @brief 设置归一化转向输入
 *
 * @param __Yaw_Input 归一化转向输入，正 = 右转
 * @param __Timestamp 本条指令的到达时刻 [us]，由串口收帧时间戳给出
 */
inline void Class_Balance::Set_Yaw_Input(const float &__Yaw_Input, const uint64_t &__Timestamp)
{
    Apply_Normalized_Input(Yaw_Input, Yaw_Input_Timestamp, __Yaw_Input, __Timestamp);
}

/**
 * @brief 设置归一化前进/后退输入
 *
 * @param __Move_Input 归一化前进输入，正 = 前进
 * @param __Timestamp  本条指令的到达时刻 [us]，由串口收帧时间戳给出
 */
inline void Class_Balance::Set_Move_Input(const float &__Move_Input, const uint64_t &__Timestamp)
{
    Apply_Normalized_Input(Move_Input, Move_Input_Timestamp, __Move_Input, __Timestamp);
}

/**
 * @brief 归一化遥控指令的统一入口
 */
inline void Class_Balance::Apply_Normalized_Input(float &__Input, uint64_t &__Input_Timestamp,
                                                  const float &__Value, const uint64_t &__Value_Timestamp)
{
    // 非法浮点（NaN/Inf/次正规）一律按"停机"处理，且不刷新时间戳
    if (Basic_Math_Is_Invalid_Float(__Value))
    {
        __Input = 0.0f;
        return;
    }

    // 归一化输入限幅到 [-1, 1]
    float input = __Value;
    Basic_Math_Constrain(&input, -1.0f, 1.0f);

    __Input = input;
    __Input_Timestamp = __Value_Timestamp;
}

/**
 * @brief 在线调参的统一校验：非法浮点拒绝，越界值限幅
 */
inline bool Class_Balance::Sanitize_Tunable(float &__Value, const float &__Min, const float &__Max)
{
    if (Basic_Math_Is_Invalid_Float(__Value))
    {
        return (false);
    }

    Basic_Math_Constrain(&__Value, __Min, __Max);
    return (true);
}

/**
 * @brief 在线设置转向指令上限 [rad/s]
 */
inline void Class_Balance::Set_Max_Yaw_Speed(const float &__Max_Yaw_Speed)
{
    // 非法值直接丢弃，保持上一组可用参数（fail-safe：宁可不变，不可乱变）
    float value = __Max_Yaw_Speed;
    if (Sanitize_Tunable(value, BALANCE_MAX_YAW_SPEED_MIN, BALANCE_MAX_YAW_SPEED_MAX))
    {
        Max_Yaw_Speed = value;
    }
}

/**
 * @brief 在线设置偏航环 Kp
 */
inline void Class_Balance::Set_Yaw_K_P(const float &__K_P)
{
    float value = __K_P;
    if (Sanitize_Tunable(value, BALANCE_YAW_K_P_MIN, BALANCE_YAW_K_P_MAX))
    {
        Yaw_Speed_PID.Set_K_P(value);
    }
}

/**
 * @brief 在线设置偏航环 Ki
 */
inline void Class_Balance::Set_Yaw_K_I(const float &__K_I)
{
    float value = __K_I;
    if (Sanitize_Tunable(value, BALANCE_YAW_K_I_MIN, BALANCE_YAW_K_I_MAX))
    {
        Yaw_Speed_PID.Set_K_I(value);
    }
}

#endif // !__BALANCE_H

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
