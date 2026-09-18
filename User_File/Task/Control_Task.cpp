/**
 * @file Control_Task.cpp
 * @brief 两台 QD4310 驱动轮的力矩（电流）控制任务。
 * @details TIM4 每 1 ms 唤醒任务；使能确认后更新电流目标，
 *          CAN 报文由 Can_Tx_Task 统一发送。
 *          本任务同时负责把蓝牙（UART7）下行指令分发给 Balance。
 */
#include "QD4310.h"
#include "cmsis_os2.h"
#include "fdcan.h"

#include "Balance.h"
#include "Init.h"
#include "bsp_uart.h"
#include "dvc_erictool.h"
// 力矩（电流）模式：下发每个电机的 Q 轴电流指令，单位 A，范围 [-10, 10]。
// 用电流模式而非速度模式：LQR 需要执行器是力矩源；速度模式下内置速度环会与
// 平衡环争夺控制权并引入延迟。0 表示零电流，不代表失能。
//
// 控制量按【共模 / 差模】组织：
//   i_common 正 = 车前进（LQR 输出，两轮同向）
//   i_diff   正 = 车右转（偏航控制器输出，两轮反向）

namespace
{
constexpr uint8_t DRIVE_MOTOR_CAN1_ID = 1U;
constexpr uint8_t DRIVE_MOTOR_CAN2_ID = 0U;
constexpr uint32_t DRIVE_ENABLE_RETRY_PERIOD_MS = 20U;
// 与 callback.cpp 中 Task1ms_Callback() 设置的线程标志保持一致。
constexpr uint32_t DRIVE_CONTROL_FLAG = 0x0001U;

// 安装符号 BALANCE_MOTOR_x_SIGN 定义在 Balance.h，命令与反馈共用同一份。
// 两台电机镜像安装，符号必须相反，否则转向会变成两轮同向（差模失效）。
static_assert(BALANCE_MOTOR_LEFT_SIGN * BALANCE_MOTOR_RIGHT_SIGN < 0.0f,
              "left and right motors are mirror-mounted: signs must be opposite");

struct DriveMotorState
{
    QD4310_t motor;               // CAN 接收回调更新的电机反馈。
    uint32_t last_enable_ms;      // 最近一次提交使能请求的系统毫秒时间。
};

// 回调长期持有 motor 的地址，因此实例必须覆盖整个任务生命周期。
DriveMotorState Drive_Motor_CAN1{};
DriveMotorState Drive_Motor_CAN2{};

/** 注册反馈回调，并立即提交首个使能请求。调用前 CAN BSP 应已初始化。 */
void DriveMotor_Init(DriveMotorState &drive, uint8_t id, FDCAN_HandleTypeDef *hfdcan)
{
    QD4310_Init(&drive.motor, id, hfdcan);
    QD4310_Enable(&drive.motor);
    drive.last_enable_ms = HAL_GetTick();
}

/**
 * @brief 更新单台电机：未使能时定时重试，已使能时发布本台电机的电流目标。
 * @note 使能请求入队不等于电机已经使能，必须等待反馈中的 enabled 标志。
 *       当前驱动没有反馈超时字段，本函数不能据此判断电机是否掉线。
 */
void DriveMotor_Update(DriveMotorState &drive, float target_current_a, uint32_t now_ms)
{
    if (!drive.motor.enabled)
    {
        // 按实际经过的毫秒重试；无符号减法允许系统计时正常回绕。
        if (static_cast<uint32_t>(now_ms - drive.last_enable_ms) >=
            DRIVE_ENABLE_RETRY_PERIOD_MS)
        {
            QD4310_Enable(&drive.motor);
            drive.last_enable_ms = now_ms;
        }
        return;
    }

    // 已使能期间刷新计时基准；反馈变为失能后，最多约 20 ms 再次尝试使能。
    drive.last_enable_ms = now_ms;
    QD4310_SetCurrent(&drive.motor, target_current_a);
}
} // namespace

extern "C" void Control_Task(void *argument)
{
    (void)argument;

    // 高于 CAN 发送任务，唤醒后优先生成本周期的电流目标。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    // FDCAN1 电机 ID 1：控制帧为 0x401，反馈帧为 0x501。
    DriveMotor_Init(Drive_Motor_CAN1, DRIVE_MOTOR_CAN1_ID, &hfdcan1);

    // FDCAN2 电机 ID 0：控制帧为 0x400，反馈帧为 0x500。
    DriveMotor_Init(Drive_Motor_CAN2, DRIVE_MOTOR_CAN2_ID, &hfdcan2);

    // 复位状态解算：累计器基准等首次有效反馈时再建立
    Balance.Init();

    for (;;)
    {
        // 标志在成功等待后自动清除；调度延迟时多次通知可能合并为一次。
        const uint32_t flags =
            osThreadFlagsWait(DRIVE_CONTROL_FLAG, osFlagsWaitAny, osWaitForever);
        if ((flags & osFlagsError) != 0U)
        {
            Error_Handler();
            return;
        }

        const uint32_t now_ms = HAL_GetTick();

        // 把电机原始反馈喂给 Balance，由它解算 4 个状态量。
        // enabled 作为"反馈有效"判据：首次有效反馈只记累计基准。
        Balance.Set_Wheel_Left(Drive_Motor_CAN1.motor.angle, Drive_Motor_CAN1.motor.speed,
                               Drive_Motor_CAN1.motor.enabled);
        Balance.Set_Wheel_Right(Drive_Motor_CAN2.motor.angle, Drive_Motor_CAN2.motor.speed,
                                Drive_Motor_CAN2.motor.enabled);
        Balance.TIM_1ms_Calculate_PeriodElapsedCallback();

        // ---- 串口（蓝牙）下行指令分发 ----
        // EricTool 的 Variable_Index 是粘滞的（匹配到后不会自动清零），所以每毫秒
        // 都会重复"应用同一条指令"：对幂等的调参指令无害；转向指令靠传入收帧时刻
        // 让 Balance 判定超时（停发 1 s 自动归零，见 Set_Yaw_Input）。
        const float command_value = EricTool_UART.Get_Variable_Value();
        switch (EricTool_UART.Get_Variable_Index())
        {
        case ERICTOOL_UART_VARIABLE_YAW:
            Balance.Set_Yaw_Input(command_value, UART7_Manage_Object.Rx_Timestamp);
            break;

        case ERICTOOL_UART_VARIABLE_MAX_TURN:
            Balance.Set_Max_Yaw_Speed(command_value);
            break;

        case ERICTOOL_UART_VARIABLE_YAW_KP:
            Balance.Set_Yaw_K_P(command_value);
            break;

        case ERICTOOL_UART_VARIABLE_YAW_KI:
            Balance.Set_Yaw_K_I(command_value);
            break;

        case ERICTOOL_UART_VARIABLE_MOVE:
            Balance.Set_Move_Input(command_value, UART7_Manage_Object.Rx_Timestamp);
            break;

        default:
            break;
        }
        // 共模（平衡+前后，来自 LQR）与差模（转向）合成每轮的"前进方向"电流
        const float i_common = Balance.Get_Current();        // LQR 输出的单侧共模电流 [A]
        const float i_diff   = Balance.Get_Current_Diff();    // 偏航环输出的差模电流
        const float i_left_fwd  = i_common + i_diff;          // 左轮
        const float i_right_fwd = i_common - i_diff;          // 右轮

        // 乘各电机的镜像安装符号，得到实际下发的 CAN 电流指令
        DriveMotor_Update(Drive_Motor_CAN1, BALANCE_MOTOR_LEFT_SIGN * i_left_fwd,  now_ms);
        DriveMotor_Update(Drive_Motor_CAN2, BALANCE_MOTOR_RIGHT_SIGN * i_right_fwd, now_ms);

    }
}

