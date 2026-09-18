/* Includes ------------------------------------------------------------------*/

#include "Balance.h"
#include "bsp_bmi088.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_Balance Balance;

// LQR 电流增益，单位 [A/状态]，由 MATLAB 离线设计给出：
//   Q = diag([300, 1, 30, 1/0.3^2]), R = 3000, kt = 0.27
//   状态顺序 [θ(rad), θ̇(rad/s), p(m), ṗ(m/s)]
static const float Balance_LQR_K_Data[4] = {-1.3019f, -0.0878f, -0.3704f, -0.9333f};

/* Private function declarations ---------------------------------------------*/

namespace
{
/**
 * @brief 把 0~2π 单圈角展开成连续角
 *
 * @param accum     多圈累计器，就地更新
 * @param raw_angle 本次收到的原始角 [rad]，范围 0~2π
 */
void Accumulate_Wheel(Struct_Wheel_Accumulator &accum, const float raw_angle)
{
    if (!accum.Initialized)
    {
        // 首次有效反馈只记基准，避免凭空多出一个 2π
        accum.Last_Raw_Angle = raw_angle;
        accum.Initialized = true;
        return;
    }

    float delta = raw_angle - accum.Last_Raw_Angle;
    if (delta > PI)
    {
        delta -= 2.0f * PI;     // 从接近 2π 跳回接近 0：其实是向前转了
    }
    if (delta < -PI)
    {
        delta += 2.0f * PI;     // 反向
    }

    accum.Accumulated += delta;
    accum.Last_Raw_Angle = raw_angle;
}
} // namespace

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 */
void Class_Balance::Init()
{
    // ---- 状态量与反馈 ----
    Theta = 0.0f;
    Theta_Dot = 0.0f;
    Position = 0.0f;
    Velocity = 0.0f;

    Valid_Left = false;
    Valid_Right = false;
    Accum_Left = {};
    Accum_Right = {};

    // ---- 遥控输入：全部归零，避免复位后沿用旧指令 ----
    Yaw_Input = 0.0f;
    Yaw_Input_Timestamp = 0;
    Current_Diff = 0.0f;
    Target_Yaw_Speed = 0.0f;

    Move_Input = 0.0f;
    Move_Input_Timestamp = 0;
    Velocity_Reference = 0.0f;
    // 位置参考与位置一起从 0 起步，避免上电瞬间凭空产生位置误差
    Position_Reference = 0.0f;

    Debug = {};

    // ---- 控制器 ----
    // LQR：1.0 A 输出限幅 = 电机额定电流
    const Class_Matrix_f32<1, 4> lqr_k(Balance_LQR_K_Data);
    LQR.Init(lqr_k, 1.0f);

    // 偏航角速度环（转向）
    //   被控对象自带积分器 → P 即可得响应；但转向摩擦会造成约 15% 稳态误差，
    //   由 Ki 消除（实测：加 Ki 后跟踪误差 14.6% → 0.08%）。
    //   左右对称性实测差 1.9%，无需额外补偿。
    //   实测基准：目标 0.75 rad/s 时转 3 圈用 25.15s（理论 25.13s）。
    //   符号：执行器 i_diff>0 = 右转；反馈陀螺 Z 轴需取反（见 Update_Yaw_Loop）。
    //   参数：Kp, Ki, Kd, Kf, I_Out_Max=0.15A(积分限幅), Out_Max=0.30A(差模上限), D_T
    //   电流预算：i_common 上限 1.0 A、i_diff 上限 0.30 A，两者同时顶满时单轮
    //   为 1.30 A，已超电机额定 1 A 但未到峰值 2 A。正常行驶不会同时顶满，
    //   若要收紧，应下调 Out_Max 而不是 LQR 的 Out_Max。
    Yaw_Speed_PID.Init(0.050f, 0.050f, 0.0f, 0.0f, 0.15f, 0.30f, 0.001f);
}

/**
 * @brief 1ms 周期回调：按职责依次推进各环节
 *
 * @note 顺序不可调换：先解算状态，再判定遥控超时，然后才轮到使用这些输入的环节。
 */
void Class_Balance::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Update_States();
    Update_Command_Timeout();
    Update_Yaw_Loop();
    Update_Lqr_Reference();

    // 组装状态向量 x = [θ, θ̇, p, ṗ]，目标 x_ref = [0, 0, p_ref, v_ref]，解算 LQR
    Class_Matrix_f32<4, 1> state;
    state[0][0] = Theta;
    state[1][0] = Theta_Dot;
    state[2][0] = Position;
    state[3][0] = Velocity;

    Class_Matrix_f32<4, 1> reference;
    reference[0][0] = 0.0f;
    reference[1][0] = 0.0f;
    reference[2][0] = Position_Reference;
    reference[3][0] = Velocity_Reference;

    LQR.Set_Target(reference);
    LQR.Set_Now(state);
    LQR.TIM_Calculate_PeriodElapsedCallback();

    Update_Debug();
}

/**
 * @brief 解算四个状态量
 */
void Class_Balance::Update_States()
{
    // ---- 俯仰角：减掉平衡点偏移，使模型零点对准真实平衡位置 ----
    // 实测：前倾时 Euler_Roll(Data[2]) 变正；平衡姿态读数约 -0.08 rad
    Theta = BSP_BMI088.Get_Euler_Angle().Data[2] - Theta_Zero_Offset;

    // ---- 俯仰角速度：陀螺 X 轴（绕传感器 X 转动即俯仰，符号已实测同号）----
    Theta_Dot = BSP_BMI088.Get_Gyro_Body().Data[0];

    // ---- 轮组多圈累计：把 0~2π 单圈角展开成连续角 ----
    if (Valid_Left)
    {
        Accumulate_Wheel(Accum_Left, Raw_Angle_Left);
    }
    if (Valid_Right)
    {
        Accumulate_Wheel(Accum_Right, Raw_Angle_Right);
    }

    // ---- 位置：两轮各自换算到"车前进"方向后取平均，再乘轮半径 ----
    const float forward_angle_left = BALANCE_MOTOR_LEFT_SIGN * Accum_Left.Accumulated;
    const float forward_angle_right = BALANCE_MOTOR_RIGHT_SIGN * Accum_Right.Accumulated;
    Position = 0.5f * (forward_angle_left + forward_angle_right) * Wheel_Radius;

    // ---- 速度：直接读转速反馈，不对位置做差分（避免放大噪声）----
    const float forward_speed_left = BALANCE_MOTOR_LEFT_SIGN * Speed_Rpm_Left;
    const float forward_speed_right = BALANCE_MOTOR_RIGHT_SIGN * Speed_Rpm_Right;
    Velocity = 0.5f * (forward_speed_left + forward_speed_right) * BASIC_MATH_RPM_TO_RADPS * Wheel_Radius;
}

/**
 * @brief 遥控指令超时保护
 *
 * @note 蓝牙断开或手机停发时，指令不再刷新，此处归零避免车持续运动。
 *       时间戳由串口收帧时刻驱动，所以"停发"和"断连"走同一条失效路径。
 */
void Class_Balance::Update_Command_Timeout()
{
    const uint64_t now_us = SYS_Timestamp.Get_Current_Timestamp();

    if (now_us - Yaw_Input_Timestamp > Command_Timeout_Us)
    {
        Yaw_Input = 0.0f;
    }

    if (now_us - Move_Input_Timestamp > Command_Timeout_Us)
    {
        // 只在"刚刚过期"这一刻做一次就地重锚（Move_Input 非零 → 归零）。
        // 如果每毫秒都重锚，位置参考会永远贴着实际位置，"位置保持"就失效了；
        // 如果完全不重锚，参考点会停在过期前冲出去的地方，车会一直追过去。
        if (Move_Input != 0.0f)
        {
            Move_Input = 0.0f;
            Position_Reference = Position;
        }
    }
}

/**
 * @brief 偏航角速度环（转向）
 */
void Class_Balance::Update_Yaw_Loop()
{
    // 陀螺 Z 轴 = 偏航角速度（传感器仅绕竖直轴旋转，Z 轴仍指竖直）。
    // 注意：Z 轴向上，右手定则下正读数 = 逆时针 = 车左转，
    //       与本工程"右转为正"的约定相反，故取反（2026-09-18 手感实测确认）。
    const float yaw_speed = -BSP_BMI088.Get_Gyro_Body().Data[2];
    Debug.Yaw_Speed = yaw_speed;

    if (Basic_Math_Is_Invalid_Float(yaw_speed))
    {
        // 反馈非法：清积分并归零输出（fail-safe，避免车乱转）
        Yaw_Speed_PID.Set_Integral_Error(0.0f);
        Current_Diff = 0.0f;
        return;
    }

    Target_Yaw_Speed = Yaw_Input * Max_Yaw_Speed;
    Yaw_Speed_PID.Set_Target(Target_Yaw_Speed);
    Yaw_Speed_PID.Set_Now(yaw_speed);
    Yaw_Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    Current_Diff = Yaw_Speed_PID.Get_Out();
}

/**
 * @brief 前进/后退：把位置与速度参考一起推成一条匀速斜坡
 *
 * @note 只给速度参考而不给位置参考的话，-K3·p 会把车拽回原点，车根本跑不出去。
 */
void Class_Balance::Update_Lqr_Reference()
{
    Velocity_Reference = Move_Input * Max_Velocity;
    Position_Reference += Velocity_Reference * BALANCE_CONTROL_PERIOD_S;

    // 防积分饱和：参考点最多领先实际位置 BALANCE_POSITION_REF_ERROR_MAX。
    // 车轮被挡住或打滑时若放任参考点无限前移，松手后车会冲出去追它。
    Basic_Math_Constrain(&Position_Reference,
                         Position - BALANCE_POSITION_REF_ERROR_MAX,
                         Position + BALANCE_POSITION_REF_ERROR_MAX);
}

/**
 * @brief 同步调试镜像
 */
void Class_Balance::Update_Debug()
{
    // 此处逐个赋值而非聚合初始化：成员顺序变动时不会静默错位。
    Debug.Theta = Theta;
    Debug.Theta_Dot = Theta_Dot;
    Debug.Position = Position;
    Debug.Velocity = Velocity;
    Debug.Out = LQR.Get_Out();
    Debug.Current_Diff = Current_Diff;
    Debug.Target_Yaw_Speed = Target_Yaw_Speed;
    Debug.Yaw_Input = Yaw_Input;
    Debug.Max_Yaw_Speed = Max_Yaw_Speed;
    Debug.Yaw_K_P = Yaw_Speed_PID.Get_K_P();
    Debug.Yaw_K_I = Yaw_Speed_PID.Get_K_I();
    Debug.Move_Input = Move_Input;
    Debug.Position_Reference = Position_Reference;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
