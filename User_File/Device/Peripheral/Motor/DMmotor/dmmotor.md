# DM 电机驱动

本驱动通过 `Class_DMMotor` 适配当前工程的 `bsp_can`，提供 DM-J4310-2EC V1.2 手册列出的四种控制接口：MIT、位置-速度、速度和力位混控。实际可用模式取决于电机型号与固件版本，不能仅凭接口存在判断电机支持该模式。

## 使用前配置

达妙调试助手中的模式必须与 `Init()` 的模式及代码调用的控制接口一致。`Init()` 只设置本地实例并注册反馈回调，不会修改电机内部模式或映射参数。`can_id` 对应 CAN ID，`master_id` 对应 Master ID（反馈帧 ID）。以下是驱动默认编码范围，使用前必须与调试助手实际读出的数值核对：

- 位置：`-12.5 ~ 12.5 rad`
- 速度：`-30 ~ 30 rad/s`
- 力矩：`-10 ~ 10 N·m`
- MIT Kp：`0 ~ 500`
- MIT Kd：`0 ~ 5`

PMAX/VMAX/TMAX 是协议映射范围，不代表电机的额定或峰值能力；输出力矩、速度和控制增益应由应用层按电机和负载单独限制。所有模式的反馈解码均使用这些范围。

三个映射范围参数必须是有限正数。当前驱动接受 `can_id` 为 `0x00~0x0F`、`master_id` 为 `0x000~0x7FF`；同一 CAN 总线上的各电机应使用不同的发送 ID 和反馈 ID，并检查 `Init()` 返回值。

## 初始化和通用命令

```c
Class_DMMotor motor;

motor.Init(&hfdcan1,
           0x01,
           0x00,
           DM_MODE_SPEED); // FDCAN、CAN ID、Master ID、模式
motor.Enable();
motor.Disable();
motor.ClearError();
motor.SetZeroPosition();
```

`Enable()`、`Disable()`、`ClearError()` 和 `SetZeroPosition()` 返回命令入队结果：`true` 表示已入队，`false` 表示提交失败，可由调用方重试。入队成功不代表电机已经执行。`SetZeroPosition()` 仅在入队成功后重置本地位置展开状态，失败时保持原状态。

后续可选参数依次为反转、PMAX、VMAX 和 TMAX：

```c
motor.Init(&hfdcan1, 0x01, 0x00,
           DM_MODE_MIT, true, 12.5f, 30.0f, 10.0f);
```

## 同时使用 J4310 和 H6215

为每台电机创建独立实例，通过 `Init()` 的最后三个参数分别传入 PMAX、VMAX、TMAX，无需修改驱动默认值。下面的 `pmax4310` 等变量需由应用层按各自电机实际读数定义，CAN ID 和 Master ID 也需与电机配置一致：

```c
static Class_DMMotor motor4310;
static Class_DMMotor motor6215;

bool ok4310 = motor4310.Init(
    &hfdcan1, 0x01, 0x11, DM_MODE_MIT, false,
    pmax4310, vmax4310, tmax4310);

bool ok6215 = motor6215.Init(
    &hfdcan1, 0x02, 0x12, DM_MODE_MIT, false,
    pmax6215, vmax6215, tmax6215);
```

应用层应检查每个初始化结果，仅对成功初始化的实例发送控制命令。当前没有单独修改映射范围的 setter，也没有写入电机内部 PMAX/VMAX/TMAX 的公开接口；在调试助手中配置电机参数后，初始化一次本地实例即可。

[H6215 V1.0 手册](https://wiki.aifitlab.com/damiao-docs/dm-h6215-v10-motor-instruction-manual) 列出 MIT、位置-速度和速度三种模式，尚不能据此确认力位混控支持。该手册写参数回包表使用 `0x33`，与文字描述存在矛盾；当前驱动仅识别 `0x55 / 0x0A` 模式回包，使用 H6215 动态切换模式前需核对实际固件回包。如果返回 `0x33`，需要先适配参数回包识别，避免落入普通反馈解码。

## 控制模式

### MIT 模式

发送 ID 为 `can_id`，位置、速度、Kp、Kd 和力矩被压缩到 8 字节控制帧：

```c
motor.SetMIT(position_rad, velocity_rad_s, kp, kd, torque_nm);
```

只使用力矩前馈时可以调用：

```c
motor.SetTorque(1.0f);
```

该接口等价于 Kp、Kd、位置和速度均为零的 MIT 控制帧，因此电机必须配置为 MIT 模式。

`DM_KP_MIN/MAX` 和 `DM_KD_MIN/MAX` 定义协议中 Kp、Kd 的编码范围，分别为 `0~500` 和 `0~5`，两者均编码为 12 位整数。实际增益由 `SetMIT()` 的 `kp`、`kd` 参数传入，每个实例可以不同；调参时不应修改协议范围常量。

```text
输出力矩 = Kp × (目标位置 − 实际位置)
         + Kd × (目标速度 − 实际速度)
         + 前馈力矩
```

### 位置-速度模式

发送 ID 为 `0x100 + can_id`，数据包含 4 字节目标位置和 4 字节最大速度限幅。速度限幅应为非负值：

```c
motor.SetPositionSpeed(1.0f, 2.0f); // 1 rad，2 rad/s
```

已知限制：当前实现仍会在 `reverse=true` 时反转速度限幅的符号，该问题尚未修复，反向安装时不能直接套用这一接口。

### 速度模式

发送 ID 为 `0x200 + can_id`，数据为 4 字节速度：

```c
motor.SetSpeed(1.0f); // 1 rad/s
```

### 力位混控模式

发送 ID 为 `0x300 + can_id`。速度限幅范围为 `0~100 rad/s`，电流限幅使用最大相电流的标幺比例 `0~1`：

```c
motor.SetForcePosition(1.0f, 5.0f, 0.2f);
```

### CAN 切换模式

通过异步队列向 `0x7FF` 发送控制模式寄存器 `0x0A` 的写请求，电机参数修改不会自动保存到 Flash：

```c
motor.SetMode(DM_MODE_POSITION_SPEED);
```

- `SetMode()` 非阻塞，返回命令入队结果：`true` 表示已入队；参数非法、其他模式仍待应答或入队失败时返回 `false`。返回值不代表电机已经切换成功。
- 请求成功入队后记录待确认模式，只有收到等待期限内、目标匹配的 `0x55 / 0x0A` 回包才更新软件模式。入队失败时保留原模式。
- 已识别的模式参数回包不参与位置、速度、力矩等普通反馈解码，重复或过期回包也不会覆盖已确认模式。
- 等待期间允许重发相同目标，但不会延长原来的超时；不同目标需等待当前请求确认或超时。
- 超时基于 `SYS_Timestamp.Get_Now_Microsecond()` 的整数微秒计时，阈值为 `250000 us`。清除动作在下一次 `SetMode()` 或通过基础校验的接收回调中执行，没有独立定时器；没有后续调用或反馈时，不会在第 250 ms 自动执行代码。
- 超时只清除等待标志，不推断电机端是否已切换成功，也不发送回退命令。通信恢复后需重新确认两端模式。

使用前应确保工程的 `SYS_Timestamp` 和 CAN 发送任务已经初始化。首次接入可先通过调试助手设置并保存控制模式，再使用一致的 `Init()` 参数。

浮点数据使用 STM32 的 IEEE 754 单精度小端格式。四个控制接口均更新 BSP 周期发送槽，实际发送由 `Can_Tx_Task` 完成，因此控制任务应周期调用当前模式对应的接口。

## 反馈

收到 Master ID 对应的反馈后，统一从 `Struct_DMMotor_Feedback feedback` 读取，
调试器展开 `motor.feedback` 即可集中查看：

```c
motor.feedback.state;
motor.feedback.position;
motor.feedback.total_position;
motor.feedback.velocity;
motor.feedback.torque;
motor.feedback.mos_temperature;
motor.feedback.rotor_temperature;
```

位置单位为 rad，速度为 rad/s，转矩为 N*m，温度为摄氏度；`state` 保留协议状态码。
反馈包含电机 ID 校验，并支持多圈位置累计和方向反转。

MIT 的 `kp`、`kd` 是发给电机内部控制器的控制参数，不属于反馈；当前驱动没有本地 PID 对象。
结构体仅用于数据组织，不提供跨中断的一致快照保证。

## 接入示例

当前 [Control_Task.cpp](../../../../Task/Control_Task.cpp) 尚未创建或控制达妙电机。以下是待集成的速度控制示例，不代表工程上电后会自动执行：

- FDCAN1
- CAN ID：`0x01`
- Master ID：`0x00`
- 原生速度模式，发送 ID `0x201`
- 启动后等待 `2000 ms` 再使能
- 目标速度 `1.0 rad/s`
- 每 `1 ms` 刷新速度指令

```c
static Class_DMMotor dm_motor;

bool motor_ready = dm_motor.Init(
    &hfdcan1, 0x01, 0x00, DM_MODE_SPEED);
if (motor_ready)
{
    osDelay(2000);
    dm_motor.Enable();
    dm_motor.SetSpeed(1.0f);
}

for (;;)
{
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    if (motor_ready)
    {
        dm_motor.SetSpeed(1.0f);
    }
}
```

应用请求停止时，应先停止正常目标更新，再将当前模式的目标输出置零并发送失能命令。例如速度模式：

```c
motor.SetSpeed(0.0f);
motor.Disable();
```

以上调用通过 CAN 发送，不构成独立的硬件急停机制。

## 验证记录

当前修改已通过 Debug 构建及主机协议回归：模式确认前后发送 ID、参数回包与重复回包隔离、入队失败、`249999/250000 us` 超时边界、同目标重试不延期、迟到回包以及跨 32 位微秒边界。尚未完成电机实机验证。
