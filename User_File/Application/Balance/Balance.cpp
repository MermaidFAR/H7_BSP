#include "Balance.h"
#include "bsp_bmi088.h"
#include "bsp_uart.h"
#include "dvc_erictool.h"
#include "fdcan.h"
#include "usart.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#define TurnPIDPolarity     -1          //转向环PID极性修改

#define MOTOR_CURRENT_LIMIT     (1.5f)   /* 单台电机电流上限，单位 A；共同量与转向量混合后限制在 ±此值。 */
#define RIGHT_MOTOR_DIRECTION   (-1.0f)  /* 右电机方向系数；将车轮物理前进方向的电流转换为右电机指令方向。 */

/* 仅用于 VOFA 显示，将弧度转化成角度，控制内部仍使用 rad/rad/s。 */
static constexpr float VOFA_RAD_TO_DEG = 57.295779513f;

QDBalance_t Balance;

/* VOFA两个独立摇杆分别使用speed_xy:X,Y#和turn_xy:X,Y#。 */
static char VOFA_Command_List[11][ERICTOOL_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] =
{
    "speed",
    "turn",
    "lqr_angle",
    "lqr_rate",
    "lqr_speed",
    "lqr_pos",
    "turn_kp",
    "turn_ki",
    "turn_kd",
    "speed_xy",
    "turn_xy"
};

static constexpr float BALANCE_D_T = 0.001f;            //控制周期1ms
static constexpr float WHEEL_RADIUS_M = 0.035f;         //车轮半径为35mm
static constexpr float WHEEL_TRACK_M = 0.170f;          //左右车轮接触点间距
static constexpr float RPM_TO_RAD_S = 0.1047197551f;    //RPM 到 rad/s 的转换系数

/* 轮角为0..2pi单圈值，在任务中展开；这些状态不放在中断里累计。 */
static bool Odometry_Ready = false;         /* 里程基准是否已建立；首次有效采样记录角度基准，避免初始位移跳变。 */
static bool Was_Moving = false;             /* 上一周期速度目标是否非零；目标刚归零时用于锁存停车位置。 */
static float Previous_Left_Angle = 0.0f;    /* 上次左电机单圈角度，rad；用于计算跨越 0/2pi 时的角度增量。 */
static float Previous_Right_Angle = 0.0f;   /* 上次右电机单圈角度，rad；用于计算跨越 0/2pi 时的角度增量。 */
static float Previous_Roll = 0.0f;          /* 上次车体 Roll，rad；用于计算本周期车体绕轮轴的转角增量。 */
static float Wheel_Travel_Rad = 0.0f;       /* 累计双轮相对车体的平均滚动角，rad；按左右电机方向取增量差的一半。 */
static float Body_Travel_Rad = 0.0f;        /* 自里程基准起累计的 Roll 变化，rad；与轮角相加后乘轮半径估算地面位移。 */

static float Balance_Clamp(float Value, float Limit)
{
    return std::fmax(-Limit, std::fmin(Limit, Value));
}

/* 一次采样间隔内转角小于pi；保留正反向经过0/2pi时的真实小增量。 */
static float Balance_AngleDelta(float Now, float Previous)
{
    return std::remainder(Now - Previous, QD4310_TWO_PI);
}

/* USART10 回调只按 # 收齐 VOFA 文本；任务解析数值并更新控制目标。 */
static constexpr uint8_t VOFA_COMMAND_LENGTH = 64;
static constexpr uint8_t VOFA_COMMAND_QUEUE_SIZE = 8;
static char VOFA_Command_Queue[VOFA_COMMAND_QUEUE_SIZE][VOFA_COMMAND_LENGTH];
static volatile uint8_t VOFA_Queue_Write = 0;
static volatile uint8_t VOFA_Queue_Read = 0;
static char VOFA_Rx_Line[VOFA_COMMAND_LENGTH];
static uint8_t VOFA_Rx_Length = 0;
static bool VOFA_Rx_Discard = false;

/* UART7 接收江协手机摇杆帧：[joystick,LH,LV,RH,RV]。 */
static constexpr uint8_t BLUETOOTH_PACKET_LENGTH = 64;
static char Bluetooth_Rx_Line[BLUETOOTH_PACKET_LENGTH];
static char Bluetooth_Rx_Packet[BLUETOOTH_PACKET_LENGTH];
static uint8_t Bluetooth_Rx_Length = 0;
static bool Bluetooth_Rx_Receiving = false;
static volatile bool Bluetooth_Packet_Ready = false;

static void VOFA_UART_Rx_Callback(uint8_t *Buffer, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
    {
        const char Byte = static_cast<char>(Buffer[i]);
        if (Byte == '#')
        {
            const uint8_t Next = (VOFA_Queue_Write + 1U) % VOFA_COMMAND_QUEUE_SIZE;
            if (!VOFA_Rx_Discard && VOFA_Rx_Length > 0 && Next != VOFA_Queue_Read)
            {
                std::memcpy(VOFA_Command_Queue[VOFA_Queue_Write], VOFA_Rx_Line, VOFA_Rx_Length);
                VOFA_Command_Queue[VOFA_Queue_Write][VOFA_Rx_Length] = '\0';
                __DMB();
                VOFA_Queue_Write = Next;
            }
            /* 超长、无效或队列满时丢弃整条，不将残缺命令作为目标。 */
            VOFA_Rx_Length = 0;
            VOFA_Rx_Discard = false;
        }
        else if ((Byte == '\r' || Byte == '\n') && VOFA_Rx_Length == 0)
        {
            continue;
        }
        else if (!VOFA_Rx_Discard)
        {
            if (Byte == '\0' || VOFA_Rx_Length >= VOFA_COMMAND_LENGTH - 1U)
            {
                VOFA_Rx_Discard = true;
            }
            else
            {
                VOFA_Rx_Line[VOFA_Rx_Length++] = Byte;
            }
        }
    }
}

static void Bluetooth_UART_Rx_Callback(uint8_t *Buffer, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
    {
        const char Byte = static_cast<char>(Buffer[i]);
        if (Byte == '[')
        {
            Bluetooth_Rx_Length = 0;
            Bluetooth_Rx_Receiving = true;
        }
        else if (Byte == ']' && Bluetooth_Rx_Receiving)
        {
            if (!Bluetooth_Packet_Ready && Bluetooth_Rx_Length > 0)
            {
                std::memcpy(Bluetooth_Rx_Packet, Bluetooth_Rx_Line, Bluetooth_Rx_Length);
                Bluetooth_Rx_Packet[Bluetooth_Rx_Length] = '\0';
                __DMB();
                Bluetooth_Packet_Ready = true;
            }
            Bluetooth_Rx_Length = 0;
            Bluetooth_Rx_Receiving = false;
        }
        else if (Bluetooth_Rx_Receiving)
        {
            if (Bluetooth_Rx_Length < BLUETOOTH_PACKET_LENGTH - 1U)
            {
                Bluetooth_Rx_Line[Bluetooth_Rx_Length++] = Byte;
            }
            else
            {
                Bluetooth_Rx_Length = 0;
                Bluetooth_Rx_Receiving = false;
            }
        }
    }
}

static bool VOFA_ReadCommand(char *Command)
{
    if (VOFA_Queue_Read == VOFA_Queue_Write)
    {
        return false;
    }
    __DMB();
    std::memcpy(Command, VOFA_Command_Queue[VOFA_Queue_Read], VOFA_COMMAND_LENGTH);
    __DMB();
    VOFA_Queue_Read = (VOFA_Queue_Read + 1U) % VOFA_COMMAND_QUEUE_SIZE;
    return true;
}

/**
 * @brief 转向环参数
 * @note 轮速差换算为转向角速度(rad/s)，PID输出为左右轮差动电流(A)。
 */
PID_InitTypeDef Turn_PID = {
    .K_P = 0.030f,       /* 初版差速P控制，A/(rad/s)，后续可用turn_kp调整。 */
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.10f,  /* 仅Ki非零时启用，默认仍为纯P。 */
    .Out_Max = 1.0f,
    .D_T = BALANCE_D_T,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE
};

void Balance_Init(void)
{
    /* 初始化平衡车状态机，初始状态为失能。 */
    Balance.Balance_FSM.Init(BALANCE_Status_DISABLE);

    /* 注册两个电机实例和反馈回调；QD4310_Init 不会发送使能命令。 */
    QD4310_Init(&Balance.Left_Motor, Left_ID, &hfdcan1);
    QD4310_Init(&Balance.Right_Motor, Right_ID, &hfdcan2);

    /* 四个正数增益保存G=-K；实际闭环含电机/车体反作用，见上方模型。 */
    Balance.LQR_K[0] = 0.74353550f;
    Balance.LQR_K[1] = 0.04714548f;
    Balance.LQR_K[2] = 0.66946370f;     //0.32946370
    Balance.LQR_K[3] = 0.29427677f;

    Balance.Turn_PID.Init(Turn_PID.K_P,
                          Turn_PID.K_I,
                          Turn_PID.K_D,
                          Turn_PID.K_F,
                          (Turn_PID.K_I > 0.0f) ? Turn_PID.I_Out_Max : 0.0f,
                          Turn_PID.Out_Max,
                          Turn_PID.D_T,
                          Turn_PID.Dead_Zone,
                          Turn_PID.I_Variable_Speed_A,
                          Turn_PID.I_Variable_Speed_B,
                          Turn_PID.I_Separate_Threshold,
                          Turn_PID.D_First
                        );

    /* 当前车体静止且轮速为 0 时测得的 Roll 直立零点，单位 rad。 */
    Balance.Target_Angle = -0.1263665f;
    Balance.Target_Speed = 0.0f;
    Balance.Target_Turn_Angle = 0.0f;
    Balance.LQR_Out = 0.0f;
    Balance.Position = 0.0f;
    Balance.Position_Target = 0.0f;
    Balance.Position_Error = 0.0f;
    Wheel_Travel_Rad = 0.0f;
    Body_Travel_Rad = 0.0f;
    Odometry_Ready = false;
    Was_Moving = false;

RESET:
    if (Balance.Left_Motor.enabled == 0)
    {
        Balance.Balance_FSM.Set_Status(BALANCE_Status_LEFT_ERROR);
        QD4310_Enable(&Balance.Left_Motor);
    }
    else if (Balance.Right_Motor.enabled == 0)
    {
        Balance.Balance_FSM.Set_Status(BALANCE_Status_RIGHT_ERROR);
        QD4310_Enable(&Balance.Right_Motor);
    }
    else if (Balance.Left_Motor.enabled == 1 &&
             Balance.Right_Motor.enabled == 1)
    {
        /* 两个电机都反馈使能后，切换为就绪状态并结束初始化。 */
        Balance.Balance_FSM.Set_Status(BALANCE_Status_READY);
        return;
    }
    osDelay(20);
    goto RESET;

}

/*
 * LQR离线设计，采用当前CAD质量分配的条件模型，尚不是实车最优增益。
 * M=0.6055 kg，mb=0.442890989 kg，h=0.026694287 m，Jb=0.000407244782 kg*m^2，
 * 单轮含转子Jw=0.00004921995 kg*m^2，r=0.035 m，Kt=0.27 Nm/A。
 * 转子约45.00 g/台是CAD等密度分配假设；重心/惯量修改后须重新求K。
 * 状态x=[theta(rad),omega(rad/s),v(m/s),p(m)]，theta=Roll-Target_Angle。
 * 输入u为每轮物理前进方向的共同电流A，含轮力矩对车体的反作用：
 * [M+2Jw/r^2, mb*h; mb*h, Jb+mb*h^2]*[v_dot; omega_dot]
 *     = [2Kt*u/r; mb*g*h*theta-2Kt*u]，g=9.80665 m/s^2。
 * 连续模型：omega_dot=223.372504*theta-1552.758583*u，
 *           v_dot=-3.850437*theta+49.261294*u；theta_dot=omega，p_dot=v。
 * 按1ms零阶保持离散化，Q=diag(25,0.04,4,4)，R=100，离散Riccati方程得到
 * K=[-0.73353550,-0.04714548,-0.32946370,-0.19427677]，u=-K*(x-x_ref)。
 * LQR_K保存G=-K，不能再对其电流输出重复套用直立环极性。
 */
/* 使用本周期已更新的状态计算纵向共同电流（A），并更新姿态显示量。 */
void BalanceLQR(void)
{
    const float Speed_Target_M_S = Balance.Target_Speed * RPM_TO_RAD_S * WHEEL_RADIUS_M;

    /* 保持四项输出公式；Position_Error 当前未随里程更新。 */
    Balance.LQR_Out = Balance.LQR_K[0] * Balance.Angle_Error +
                      Balance.LQR_K[1] * Balance.Angle_Rate +
                      Balance.LQR_K[2] * (Balance.Speed_M_S - Speed_Target_M_S);
    Balance.LQR_Out += Balance.LQR_K[3] * Balance.Position_Error;
    if (!std::isfinite(Balance.LQR_Out))
    {
        Balance.LQR_Out = 0.0f;
    }
    Balance.Angle_Target = Balance.Target_Angle;
    Balance.Angle_Target_Deg = Balance.Target_Angle * VOFA_RAD_TO_DEG;
    Balance.Angle_Feedback_Deg = Balance.Angle_Feedback * VOFA_RAD_TO_DEG;
    Balance.Angle_Rate_Deg_S = Balance.Angle_Rate * VOFA_RAD_TO_DEG;
}

/* 使用本周期转向反馈计算差动电流（A），并更新转向显示量。 */
void BalancePID(void)
{
    Balance.Turn_PID.Set_Target(Balance.Target_Turn_Angle);
    Balance.Turn_PID.Set_Now(Balance.Turn_Feedback);
    if (Turn_PID.K_I == 0.0f)
    {
        Balance.Turn_PID.Set_Integral_Error(0.0f);
    }
    Balance.Turn_PID.TIM_Calculate_PeriodElapsedCallback();
    Balance.Turn_PID_Out = Balance.Turn_PID.Get_Out();
    if (!std::isfinite(Balance.Turn_PID_Out))
    {
        Balance.Turn_PID_Out = 0.0f;
    }
    Balance.Target_Turn_Deg_S = Balance.Target_Turn_Angle * VOFA_RAD_TO_DEG;
    Balance.Turn_Feedback_Deg_S = Balance.Turn_Feedback * VOFA_RAD_TO_DEG;
}

/* 每 1 ms 执行一次：采集反馈、更新里程、计算 LQR/转向 PID，再混合发送电流。 */
void Balance_Control(void)
{
    /* Roll为前后倾角，车体X轴陀螺为前后倾角速度。 */
    Balance.Angle_Feedback = BSP_BMI088.Get_Euler_Angle().Data[2];
    Balance.Angle_Rate = BSP_BMI088.Get_Gyro_Body().Data[0];
    const uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    const float Left_Angle = Balance.Left_Motor.angle;
    const float Right_Angle = Balance.Right_Motor.angle;
    const float Left_Speed = Balance.Left_Motor.speed;
    const float Right_Speed = Balance.Right_Motor.speed;
    __set_PRIMASK(Primask);

    /* 只拒绝无效浮点数据，避免NaN参与电流到整数的转换；不新增倾角失能阈值。 */
    if (!std::isfinite(Balance.Angle_Feedback) || !std::isfinite(Balance.Angle_Rate) ||
        !std::isfinite(Left_Angle) || !std::isfinite(Right_Angle) ||
        !std::isfinite(Left_Speed) || !std::isfinite(Right_Speed))
    {
        Balance.LQR_Out = 0.0f;
        Balance.Left_Current_Command = 0.0f;
        Balance.Right_Current_Command = 0.0f;
        QD4310_SetCurrent(&Balance.Left_Motor, 0.0f);
        QD4310_SetCurrent(&Balance.Right_Motor, 0.0f);
        return;
    }

    if (!Odometry_Ready)
    {
        Previous_Left_Angle = Left_Angle;
        Previous_Right_Angle = Right_Angle;
        Previous_Roll = Balance.Angle_Feedback;
        Odometry_Ready = true;
    }

    Wheel_Travel_Rad += 0.5f *
        (Balance_AngleDelta(Left_Angle, Previous_Left_Angle) -
         Balance_AngleDelta(Right_Angle, Previous_Right_Angle));
    Body_Travel_Rad += Balance_AngleDelta(Balance.Angle_Feedback, Previous_Roll);
    Previous_Left_Angle = Left_Angle;
    Previous_Right_Angle = Right_Angle;
    Previous_Roll = Balance.Angle_Feedback;

    Balance.Angle_Error = Balance_AngleDelta(Balance.Angle_Feedback, Balance.Target_Angle);
    Balance.Speed_Feedback = (Left_Speed - Right_Speed) * 0.5f;
    /*
     * QD反馈为转子相对定子的角/速度。以原直立环“Roll误差正 -> 左电流正、右负”
     * 为纵向正方向，轮的地面转角=相对转角+车体转角。位移与速度使用同一补偿。
     * 不能只把rpm换成m/s却让位移仍采用未补偿单圈角。
     */
    Balance.Speed_M_S = WHEEL_RADIUS_M *
        (Balance.Speed_Feedback * RPM_TO_RAD_S + Balance.Angle_Rate);
    Balance.Position = WHEEL_RADIUS_M * (Wheel_Travel_Rad + Body_Travel_Rad);

    const bool Moving = (Balance.Target_Speed != 0.0f);
    if (Moving || Was_Moving)
    {
        /* 行驶时不追旧停车点；切回零速目标时只锁存一次当前停车点。 */
        Balance.Position_Target = Balance.Position;
    }
    Was_Moving = Moving;
    /* 位置误差更新；LQR 位置输出项仍使用该字段的现有值。 */
    Balance.Position_Error = Balance.Position - Balance.Position_Target;
    BalanceLQR();

    /* 右轮物理速度为-Right_Speed；正转向目标要求右轮比左轮快。 */
    Balance.Turn_Feedback = (-Right_Speed - Left_Speed) *
        RPM_TO_RAD_S * WHEEL_RADIUS_M / WHEEL_TRACK_M;
    BalancePID();

    const float Common_Current = Balance_Clamp(Balance.LQR_Out, MOTOR_CURRENT_LIMIT);
    /* 共同量优先，差动量只占剩余电流空间；仍保持每台电机±1.5 A。 */
    const float Turn_Current = Balance_Clamp(
        TurnPIDPolarity * Balance.Turn_PID_Out,
        MOTOR_CURRENT_LIMIT - std::fabs(Common_Current));
    Balance.Left_Current_Command = Common_Current + Turn_Current;
    Balance.Right_Current_Command = RIGHT_MOTOR_DIRECTION * (Common_Current - Turn_Current);
    QD4310_SetCurrent(&Balance.Left_Motor, Balance.Left_Current_Command);
    QD4310_SetCurrent(&Balance.Right_Motor, Balance.Right_Current_Command);
}

/* 处理 USART10 在线增益设置；不复位轮角/停车点，避免在线调整时跳变位置。 */
static void Balance_SetGain(int32_t Index, float Value)
{
    if (!std::isfinite(Value) || Value < 0.0f)
    {
        return;
    }
    if (Index >= 0 && Index < 4)
    {
        Balance.LQR_K[Index] = Value;
        return;
    }
    switch (Index)
    {
        case 4:
            Turn_PID.K_P = Value;
            Balance.Turn_PID.Set_K_P(Value);
            break;
        case 5:
            Turn_PID.K_I = Value;
            Balance.Turn_PID.Set_K_I(Value);
            Balance.Turn_PID.Set_I_Out_Max((Value > 0.0f) ? Turn_PID.I_Out_Max : 0.0f);
            Balance.Turn_PID.Set_Integral_Error(0.0f);
            break;
        case 6:
            Turn_PID.K_D = Value;
            Balance.Turn_PID.Set_K_D(Value);
            break;
        default:
            break;
    }
}

/*
 * MaterialJoystick按X、Y顺序输出，右/上为正。
 * 前进摇杆只取Y(rpm)；转向摇杆只取-X(rad/s)，使右推对应右转。
 * 两条命令各自只改一个目标，回中不会覆盖另一个摇杆的目标。
 */
static void VOFA_ApplyCommand(char *Command)
{
    char *Separator = std::strchr(Command, ':');
    if (Separator == nullptr)
    {
        return;
    }
    *Separator = '\0';

    int32_t Index = -1;
    for (uint8_t i = 0; i < sizeof(VOFA_Command_List) / sizeof(VOFA_Command_List[0]); i++)
    {
        if (std::strcmp(Command, VOFA_Command_List[i]) == 0)
        {
            Index = i;
            break;
        }
    }
    if (Index < 0)
    {
        return;
    }

    char *Value_Start = Separator + 1;
    char *End = nullptr;
    const float X = std::strtof(Value_Start, &End);
    if (End == Value_Start || !std::isfinite(X))
    {
        return;
    }
    while (*End == ' ' || *End == '\t')
    {
        End++;
    }

    if (Index == 9 || Index == 10)
    {
        if (*End != ',')
        {
            return;
        }
        Value_Start = End + 1;
        const float Y = std::strtof(Value_Start, &End);
        if (End == Value_Start || !std::isfinite(Y))
        {
            return;
        }
        while (*End == ' ' || *End == '\t')
        {
            End++;
        }
        if (*End != '\0')
        {
            return;
        }
        if (Index == 9)
        {
            Balance.Target_Speed = Y;
        }
        else
        {
            Balance.Target_Turn_Angle = -X;
        }
        return;
    }

    if (*End != '\0')
    {
        return;
    }
    if (Index == 0)
    {
        Balance.Target_Speed = X;
    }
    else if (Index == 1)
    {
        Balance.Target_Turn_Angle = X;
    }
    else
    {
        Balance_SetGain(Index - 2, X);
    }
}

/*
 * 通过蓝牙模块HC05进行单片机与手机的连接
 * HC05的波特率设置为9600，数据格式为8N1
 * 两条命令各自只改一个目标，回中不会覆盖另一个摇杆的目标
 */
void Bluetooth_Control(void)
{
    if (!Bluetooth_Packet_Ready)
    {
        return;
    }

    char Packet[BLUETOOTH_PACKET_LENGTH];
    const uint32_t Primask = __get_PRIMASK();
    __disable_irq();
    std::memcpy(Packet, Bluetooth_Rx_Packet, BLUETOOTH_PACKET_LENGTH);
    Bluetooth_Packet_Ready = false;
    __set_PRIMASK(Primask);

    char *Tag = std::strtok(Packet, ",");
    if (Tag == nullptr || std::strcmp(Tag, "joystick") != 0)
    {
        return;
    }

    long Joystick_Value[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        char *Value = std::strtok(nullptr, ",");
        if (Value == nullptr)
        {
            return;
        }
        Joystick_Value[i] = std::strtol(Value, nullptr, 10);
    }

    /* 江协字段顺序为 LH、LV、RH、RV；当前只使用 LV 和 RH，数值不做缩放。 */
    Balance.Target_Speed = static_cast<float>(Joystick_Value[1]);
    Balance.Target_Turn_Angle = static_cast<float>(Joystick_Value[2]);
}

//VOFA控制
void VOFA_Init(void)
{
    VOFA_Queue_Write = 0;
    VOFA_Queue_Read = 0;
    VOFA_Rx_Length = 0;
    VOFA_Rx_Discard = false;
    Bluetooth_Rx_Length = 0;
    Bluetooth_Rx_Receiving = false;
    Bluetooth_Packet_Ready = false;
    /* 绑定 USART10 接收回调；USB 遥测由 TransportTask 独立发送。 */
    UART_Init(&huart10, VOFA_UART_Rx_Callback);
    /* UART7 专门接收手机蓝牙发送的江协摇杆协议。 */
    UART_Init(&huart7, Bluetooth_UART_Rx_Callback);
    EricTool_UART.Init(&huart10,
                       11,
                       reinterpret_cast<const char **>(VOFA_Command_List));

    /* 17路通道含义见Balance.h；保持100Hz/JustFloat，USB遥测入口不变。 */
    EricTool_UART.Set_Data(
        17,
        (int) &Balance.Angle_Target_Deg,
        (int) &Balance.Angle_Feedback_Deg,
        (int) &Balance.LQR_Out,
        (int) &Balance.Target_Speed,
        (int) &Balance.Speed_Feedback,
        (int) &Balance.Position_Target,
        (int) &Balance.Position,
        (int) &Balance.Target_Turn_Deg_S,
        (int) &Balance.Turn_Feedback_Deg_S,
        (int) &Balance.Turn_PID_Out,
        (int) &Balance.Speed_M_S,
        (int) &Balance.Position_Error,
        (int) &Balance.Angle_Rate_Deg_S,
        (int) &Balance.Left_Current_Command,
        (int) &Balance.Right_Current_Command,
        (int) &Balance.Left_Motor.current,
        (int) &Balance.Right_Motor.current);
}

void VOFA_Control(void)
{
    Bluetooth_Control();

    /* 在任务中处理完整命令；每次最多处理一个队列容量，避免连续输入占满控制周期。 */
    char Command[VOFA_COMMAND_LENGTH];
    for (uint8_t i = 0; i < VOFA_COMMAND_QUEUE_SIZE; i++)
    {
        if (!VOFA_ReadCommand(Command))
        {
            break;
        }
        VOFA_ApplyCommand(Command);
    }

    /* 17路72字节，115200 8N1下发送约6.25ms，每10个1ms控制周期发送一次。 */
    static uint8_t Telemetry_Divider = 0;
    if (++Telemetry_Divider >= 10)
    {
        Telemetry_Divider = 0;
        /* 发送缓存属于EricTool；DMA尚忙时不重写它。 */
        if (huart10.gState == HAL_UART_STATE_READY)
        {
            EricTool_UART.TIM_1ms_Write_PeriodElapsedCallback();
        }
    }
}
