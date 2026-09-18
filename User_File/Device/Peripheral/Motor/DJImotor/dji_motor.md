# DJI 电机驱动

`Class_DJIMotor` 支持 M2006/C610、M3508/C620 和 GM6020，使用项目当前的
FDCAN 回调注册和周期发送接口，不使用动态内存。

原始驱动由 [Kylin-6](https://github.com/Kylin-6) 在
[PR #5](https://github.com/MermaidFAR/H7_BSP/pull/5) 贡献，后续适配由 zzm 维护。

## 函数命名与文件组织

保留 `Class_DJIMotor`、`Class_DJIMotor_Group` 和配置结构体；内部实现使用 C 风格的
指针、显式类型转换和文件级静态函数。参考
[麻神 MC02 BSP](https://github.com/yssickjgd/damiao_mc02_bsp) 的类型前缀、下划线命名、
文件分区及协议与调度分层，继续使用本工程的 CAN 注册器、发送槽、PID 与系统时间服务。

| 职责 | 函数名 |
|---|---|
| PID 参数初始化 | `PID_Init` |
| 目标设置 | `SetRef` |
| 外环选择 | `Set_Outer_Loop` |
| 反馈源选择 | `Set_Feedback_Source` |
| CAN 接收入口 | `CAN_RxCpltCallback` |
| 反馈超时检查 | `Check_Feedback_Timeout` |
| 清除本电机命令 | `Clear_Command` |

`Init()` 接收配置结构体的只读引用，例如 `motor.Init(config)`；接口声明采用
`Init(const Struct_DJIMotor_Init_Config &config)` 和 `SetRef(float ref)`，参数名简洁，
普通参数名不增加双下划线前缀，普通成员函数在实现文件中定义，声明不加 `inline`。反馈指针按用途
保留 `const`；普通标量直接传值。`constexpr` 用于有类型且受作用域约束的常量，
`nullptr` 用于空指针，整数常量不加 `U` 后缀。

头文件按 Exported 分区放公开类型和类声明，成员函数实现放在 `.cpp`；实现文件按 Includes、Private macros、
Private types、Private variables、Private function declarations、Function prototypes 分区。
文件头记录文件职责、原贡献者和本次维护信息，不复制参考库的作者或虚构历史。

`Control()`、`Update()` 与 `Send()` 仍沿用下文约定：单电机及 Group 的无参数 `Control()`
只计算；Group 的 `Control(ref...)` 计算后发布。本轮没有改变这些重载的行为。

## 协议配置

| 设备 | ID | 反馈 ID | 控制 ID | 指令范围 |
|---|---:|---:|---:|---:|
| M2006/C610 | 1~4 | `0x200 + ID` | `0x200` | ±10000 |
| M2006/C610 | 5~8 | `0x200 + ID` | `0x1FF` | ±10000 |
| M3508/C620 | 1~4 | `0x200 + ID` | `0x200` | ±16384 |
| M3508/C620 | 5~8 | `0x200 + ID` | `0x1FF` | ±16384 |
| GM6020 电压 | 1~4 | `0x204 + ID` | `0x1FF` | ±25000 |
| GM6020 电压 | 5~7 | `0x204 + ID` | `0x2FF` | ±25000 |
| GM6020 电流 | 1~4 | `0x204 + ID` | `0x1FE` | ±16384 |
| GM6020 电流 | 5~7 | `0x204 + ID` | `0x2FE` | ±16384 |

GM6020 电流模式要求固件版本不低于 1.0.11.2，并通过 RoboMaster Assistant
2.7 或更高版本开启电流环。

## 初始化

```c
Class_DJIMotor motor;

Struct_DJIMotor_Init_Config config = {};
config.hfdcan = &hfdcan1;
config.can_id = 1;
config.motor_type = Enum_DJIMotor_Type::GM6020;
config.close_loop = DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP;
config.outer_loop = DJI_MOTOR_SPEED_LOOP;
config.current_pid.K_P = 0.5f;
config.current_pid.Out_Max = 16000.0f;
config.current_pid.D_T = 0.001f;
config.speed_pid.K_P = 10.0f;
config.speed_pid.Out_Max = 16000.0f;
config.speed_pid.D_T = 0.001f;
config.control_mode = Enum_DJIMotor_Control_Mode::CURRENT;
config.feedback_timeout_ms = 20;

if (motor.Init(config))
{
    motor.SetRef(1.5707963f); // 位置环目标 90° = pi/2 rad
}
```

非 GM6020 电机只接受 `CURRENT`。`gear_ratio <= 0` 时使用型号默认值：M2006 为
36、M3508 为官方标称约 19、GM6020 为 1。若实际机构或精度要求不同，应显式填写
实测或设计减速比。

## 控制与发送

单个电机的 `Control()` 只计算 PID 并更新共享帧槽。普通业务应把同一物理控制帧中的
电机组成 `Class_DJIMotor_Group`，由 Group 完成一次计算和一次非阻塞发布。

`Disable()` 清除本电机所占槽并立即发布整帧，保留其他电机槽的命令。Group 的
`Disable()` 一次清除全部成员并只发布一次。返回值表示 BSP 周期槽是否接受本次零指令；
失败时应重试失能或继续调用 Group 的 `Send()`，不能将调用返回等同于电机已经停转。

对象首次收到合法反馈前，以及超过 `feedback_timeout_ms` 没有反馈后，`Control()`
都会保持该槽为零并清除 PID 积分。超时判断复用
`SYS_Timestamp.Get_Now_Microsecond()`，以 64 位整数微秒计算；使用前应初始化系统时间服务。

## 反馈量和单位

反馈集中在 `Struct_DJIMotor_Feedback feedback` 中，例如 `motor.feedback.output_speed`。
原有 `motor.output_speed` 等访问需要增加 `.feedback`。

驱动默认采用弧度制：位置环参考值和外部角度反馈为 `rad`，速度环参考值和外部速度
反馈为 `rad/s`。电流环和开环参考值仍是对应协议控制量，不进行角度单位换算。
从旧角度制配置迁移时，若要保持近似相同的控制输出，角度环和速度环中作用于误差的
PID 增益通常需要乘以 `180/pi`，之后仍应结合实机重新整定。

需要继续使用角度制业务代码时，可调用显式兼容接口：

```c
motor.SetRef_Degree(90.0f);                 // 位置环：90 deg
gimbal.Control_Degree(yaw_deg, pitch_deg); // 位置环：deg；速度环：deg/s
```

这些接口只负责乘以 `pi/180`，随后复用默认弧度制控制链。开环和电流环不应使用
`Degree` 接口，因为它们的参考值不是角度或角速度。

- `feedback.encoder`：协议原始 13 位转子编码器值，范围 0~8191。
- `feedback.rotor_angle`、`feedback.rotor_total_angle`：转子侧角度，单位 rad。
- `feedback.rotor_speed`：转子侧滤波速度，单位 rad/s。
- `feedback.output_angle`、`feedback.output_total_angle`、`feedback.output_speed`：上述转子量除以减速比。
- `feedback.rotor_angle_degree`、`feedback.rotor_total_angle_degree`：转子侧角度，单位 °。
- `feedback.rotor_speed_degree_per_second`：转子侧速度，单位 °/s。
- `feedback.output_angle_degree`、`feedback.output_total_angle_degree`：输出侧角度，单位 °。
- `feedback.output_speed_degree_per_second`：输出侧速度，单位 °/s。
- `feedback.current_raw`：协议返回的原始实际转矩电流值，不声明为安培。
- `feedback.temperature`：M3508 和 GM6020 的电机温度；C610 对应字节为空，因此保持 0。
- `online`：最近一次接收或超时检查得到的在线状态。
- `last_feedback_timestamp_us`：最近反馈的 64 位系统微秒时间戳；任务中读取时使用
  `Get_Last_Feedback_Timestamp_Us()`，由接口保护 32 位 MCU 上的完整快照。

反向配置作用于角度、速度以及控制输出的逻辑方向；`feedback.encoder` 和 `feedback.current_raw` 始终保留
协议原始值。内部电流环会根据反向配置转换 `feedback.current_raw` 的符号。

速度低通采用 `DJI_MOTOR_ROTOR_SPEED_LPF_ALPHA * old + (1-alpha) * measured`，当前 alpha
为 0.85，明确表示保留 85% 旧值。

## 冲突规则

`Init()` 同时检查同一 FDCAN 总线上的反馈 ID 和控制帧 slot。比如 M2006/M3508 ID 5
和 GM6020 ID 1 的反馈 ID 都是 `0x205`，在同一总线上注册时后者会返回 `false`。

## 多电机 Group

`Class_DJIMotor_Group` 只保存 1~4 个已经初始化的电机指针，不复制对象、不分配动态
内存，也不参与 PID。一个 Group 必须包含同一 `(FDCAN, TX ID)` 物理帧内的全部已注册
电机，并独占该物理帧的常规控制发送权；单电机 `Disable()` 可主动发布清零后的整帧。
跨物理帧、遗漏已有 slot、重复指针、空洞参数、未初始化
电机，或第二个 Group 争用相同物理帧时，`Init()` 返回 `false`。Group 建立后也不允许再向
该物理帧注册新电机。

### 四个 M3508 底盘

```c
Class_DJIMotor motor1;
Class_DJIMotor motor2;
Class_DJIMotor motor3;
Class_DJIMotor motor4;
Class_DJIMotor_Group chassis;

Struct_DJIMotor_Init_Config config = {};
config.hfdcan = &hfdcan1;
config.can_id = 1;
config.motor_type = Enum_DJIMotor_Type::M3508;
config.close_loop = DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP;
config.outer_loop = DJI_MOTOR_SPEED_LOOP;
config.current_pid.K_P = 0.5f;
config.current_pid.Out_Max = 16384.0f;
config.current_pid.D_T = 0.001f;
config.speed_pid.K_P = 10.0f;
config.speed_pid.Out_Max = 16000.0f;
config.speed_pid.D_T = 0.001f;
config.control_mode = Enum_DJIMotor_Control_Mode::CURRENT;

bool ok = motor1.Init(config);
config.can_id = 2;
ok = motor2.Init(config) && ok;
config.can_id = 3;
ok = motor3.Init(config) && ok;
config.can_id = 4;
ok = motor4.Init(config) && ok;
ok = ok && chassis.Init(&motor1, &motor2, &motor3, &motor4);
```

控制周期：

```c
bool submitted = chassis.Control(v1, v2, v3, v4);
```

`Control(ref...)` 依次设置目标、计算四台电机并只发布一次 CAN1/0x200 帧。
返回 `false` 表示 Group 未初始化、至少一台电机未使能或掉线，或者发布到 BSP 周期槽失败；
即使个别电机掉线，其 slot 仍会清零，其他在线电机的帧仍会发布。

高级用法仍可分开调用：

```c
chassis.Update(v1, v2, v3, v4); // SetRef + PID 计算，不发送
bool submitted = chassis.Send();
```

也可以使用 `SetRef()`、无参数 `Control()`、`Send()` 分三步执行。

### 两个 GM6020 云台电机

两个电机应配置为同一 FDCAN、同一控制模式，且 ID 均位于 1~4 或均位于 5~7：

```c
Class_DJIMotor yaw_motor;
Class_DJIMotor pitch_motor;
Class_DJIMotor_Group gimbal;

Struct_DJIMotor_Init_Config gm_config = {};
gm_config.hfdcan = &hfdcan2;
gm_config.can_id = 1;
gm_config.motor_type = Enum_DJIMotor_Type::GM6020;
gm_config.close_loop = DJI_MOTOR_SPEED_LOOP;
gm_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
gm_config.speed_pid.K_P = 25.0f;
gm_config.speed_pid.K_I = 2.0f;
gm_config.speed_pid.I_Out_Max = 10000.0f;
gm_config.speed_pid.Out_Max = 24000.0f;
gm_config.speed_pid.D_T = 0.001f;
gm_config.control_mode = Enum_DJIMotor_Control_Mode::VOLTAGE;

bool ok = yaw_motor.Init(gm_config);
gm_config.can_id = 2;
ok = pitch_motor.Init(gm_config) && ok;
ok = ok && gimbal.Init(&yaw_motor, &pitch_motor);

bool submitted = gimbal.Control(yaw_target, pitch_target);
```

其中 `yaw_target`、`pitch_target` 在位置环时使用 rad，在速度环时使用 rad/s。

### 500Hz 底盘与 1kHz 云台

```c
// 1kHz Gimbal Task
gimbal.Control(yaw_ref, pitch_ref);       // 只发布云台的物理帧
```

```c
// 500Hz Chassis Task
chassis.Control(v1, v2, v3, v4);         // 只发布底盘的物理帧
```

两个 Group 的 `Init()` 已保证它们不可能拥有同一 `(FDCAN, TX ID)`，因此不同任务不会
重复发布同一物理帧，也不会夹带另一个模块的旧 slot。

`Send()` 使用非阻塞的 `CAN_Tx_Perform()` 更新 BSP 周期槽并立即返回。实际 HAL FDCAN
提交由 `Can_Tx_Task` 执行；硬件 Tx FIFO 满时 BSP 不等待、不 busy-wait，并保留未确认版本，
在后续发送任务周期重试。因此 Group 的返回值能反映周期槽提交结果，不能同步反映稍后发生
的硬件 FIFO 状态。

Group 的 `Enable()` 依次使能所有成员，`Disable()` 批量清零后一次发布。Group 不拥有
电机，因此成员电机对象的生命周期必须长于 Group；推荐都使用静态或全局对象。

## PID 调试

在调试器中展开 `motor.feedback.pid.current`、`speed`、`angle`，分别查看电流、速度和角度环。
原有 `motor.current_pid`、`motor.speed_pid`、`motor.angle_pid` 仍负责计算。

```c
motor.feedback.pid.speed.kp = 2.0f;
motor.feedback.pid.speed.ki = 0.1f;
motor.feedback.pid.speed.kd = 0.0f;
// 下一次 motor.Control() 或电机组 Control()/Update() 时应用。
```

- `kp/ki/kd`：可调入口，初始化时从配置载入，下一次 `Control()` 开始时写入实际 PID。
  没有改调试入口时，原 PID setter 的修改会同步回来；两处同时修改同一增益时，调试入口优先。
  NaN/Inf 不写入 PID，调试字段恢复为实际值；修改增益不重置积分和历史状态。
- `kf/integral_out_max/out_max`：实际 PID 的前馈增益和限幅观察值。
- `target/now/error`：实际 PID 的目标、反馈和最近一次计算经过死区处理后的误差。
- `integral_error/out`：积分状态和 PID 自身输出；`out` 尚未包含后级环和最终电机指令限幅。
- `active`：最近一次控制是否执行该环；关闭、掉线或本轮跳过时为 false。
  未执行的环保留最近一次计算值，因此不能把此时的 `out` 当作当前发给电机的指令。

除 `kp/ki/kd` 外均为观察字段，修改不会用于控制，并在下次刷新覆盖。停止时调用 `Control()`
也能应用增益，但不会因此使能电机；只调用 `Send()` 不应用新增益。调试快照不保证跨中断或
跨控制周期读取的一致性，需要一次改齐多个增益时应暂停目标后修改，再恢复运行。
