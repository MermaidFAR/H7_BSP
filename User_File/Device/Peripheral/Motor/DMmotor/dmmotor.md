# DM 电机驱动

本驱动使用 C++ 封装并适配当前工程的 `bsp_can`，支持 DM-J4310-2EC V1.2 手册列出的四种控制模式：MIT、位置-速度、速度和力位混控。

## 使用前配置

达妙调试助手中的模式必须与 `Init()` 的模式及代码调用的控制接口一致。`can_id` 对应 CAN ID，`master_id` 对应 Master ID（反馈帧 ID）。本驱动按照 J4310 手册截图使用以下默认映射范围，必须与调试助手实际读出的数值一致：

- 位置：`-12.5 ~ 12.5 rad`
- 速度：`-30 ~ 30 rad/s`
- 力矩：`-10 ~ 10 N·m`
- MIT Kp：`0 ~ 500`
- MIT Kd：`0 ~ 5`

## 初始化和通用命令

```cpp
Class_DMMotor motor;

motor.Init(&hfdcan1,
           0x01U,
           0x00U,
           Enum_DMMotor_Mode::SPEED); // FDCAN、CAN ID、Master ID、模式
motor.Enable();
motor.Disable();
motor.ClearError();
motor.SetZeroPosition();
```

后续可选参数依次为反转、PMAX、VMAX 和 TMAX：

```cpp
motor.Init(&hfdcan1, 0x01U, 0x00U,
           Enum_DMMotor_Mode::MIT, true, 12.5f, 30.0f, 10.0f);
```

## 控制模式

### MIT 模式

发送 ID 为 `can_id`，位置、速度、Kp、Kd 和力矩被压缩到 8 字节控制帧：

```cpp
motor.SetMIT(position_rad, velocity_rad_s, kp, kd, torque_nm);
```

只使用力矩前馈时可以调用：

```cpp
motor.SetTorque(1.0f);
```

该接口等价于 Kp、Kd、位置和速度均为零的 MIT 控制帧，因此电机必须配置为 MIT 模式。

### 位置-速度模式

发送 ID 为 `0x100 + can_id`，数据包含 4 字节位置和4 字节速度：

```cpp
motor.SetPositionSpeed(1.0f, 2.0f); // 1 rad，2 rad/s
```

### 速度模式

发送 ID 为 `0x200 + can_id`，数据为 4 字节速度：

```cpp
motor.SetSpeed(1.0f); // 1 rad/s
```

### 力位混控模式

发送 ID 为 `0x300 + can_id`。速度限幅范围为 `0~100 rad/s`，电流限幅使用最大相电流的标幺比例 `0~1`：

```cpp
motor.SetForcePosition(1.0f, 5.0f, 0.2f);
```

### CAN 切换模式

写入控制模式寄存器 `0x0A`，修改立即生效但不会自动保存到 Flash：

```cpp
motor.SetMode(Enum_DMMotor_Mode::POSITION_SPEED);
```

浮点数据使用 STM32 的 IEEE 754 单精度小端格式。四个控制接口均更新 BSP 周期发送槽，实际发送由 `Can_Tx_Task` 完成，因此控制任务应周期调用当前模式对应的接口。

## 反馈

收到 Master ID 对应的反馈后，可从实例直接读取：

```cpp
motor.state;
motor.position;
motor.total_position;
motor.velocity;
motor.torque;
motor.mos_temperature;
motor.rotor_temperature;
```

反馈包含电机 ID 校验，并支持多圈位置累计和方向反转。

## 当前工程示例

当前 [Control_Task.cpp](../../../Task/Control_Task.cpp) 使用：

- FDCAN1
- CAN ID：`0x01`
- Master ID：`0x00`
- 原生速度模式，发送 ID `0x201`
- 启动后等待 `2000 ms` 再使能
- 目标速度 `1.0 rad/s`
- 每 `1 ms` 刷新速度指令

```cpp
static Class_DMMotor dm_motor;

if (dm_motor.Init(&hfdcan1, 0x01U, 0x00U, Enum_DMMotor_Mode::SPEED))
{
    osDelay(2000U);
    dm_motor.Enable();
    dm_motor.SetSpeed(1.0f);
}

for (;;)
{
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    dm_motor.SetSpeed(1.0f);
}
```

急停时应先把当前模式的目标输出置零，再发送失能命令。例如速度模式：

```cpp
motor.SetSpeed(0.0f);
motor.Disable();
```
