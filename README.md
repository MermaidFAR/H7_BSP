# H7_BSP

STM32H723ZG 板级支持包工程，面向 RoboMaster/机器人控制场景。项目使用 STM32CubeMX 生成的 HAL 外设初始化作为底层基础，在 `User_File/` 中维护用户态 C++ BSP、设备组件、中间件算法与 FreeRTOS 任务。

当前工程已经不是纯 CubeMX 框架：系统初始化、回调分发、BMI088 姿态解算链路、DMA 可访问内存段、SystemView 集成、FreeRTOS 任务入口、CAN/UART BSP 均已接通。仍未完成的重点集中在通信传输任务、电源/ADC 的系统级接入、以及部分并发标志和边角问题收口。

## 当前状态

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| 构建 | 已通过 | `cmake --build build/Debug` 可生成 `build/Debug/H7_BSP.elf` |
| MCU | 已配置 | STM32H723ZG，Flash 1024K，DTCMRAM/RAM_D1/RAM_D2/RAM_D3 分区已在链接脚本中定义 |
| RTOS | 已接入 | FreeRTOS + CMSIS-RTOS V2，创建 `TransportTask` 与 `InsTask` |
| SystemView | 已接入 | 使用 `User_Config/FreeRTOS_Patch/port_patched.c` 替换原始 FreeRTOS `port.c` |
| BMI088 | 已形成主链路 | EXTI 数据就绪触发 SPI DMA 读取，陀螺仪完成后通知 `InsTask` 执行 EKF |
| DMA 缓冲区 | 已修复关键布局 | SPI/ADC 管理对象放入 `.dma_buffer`，链接到 RAM_D1，MPU 配置为 non-cacheable |
| TransportTask | 骨架完成 | 当前只初始化 USB Device 并周期让出 CPU，尚无协议和数据收发 |
| CAN/FDCAN BSP | 已实现 | `bsp_can` v2 双通道发送架构：周期通道（`CAN_Tx_Perform` + `BSP_CAN_SendPer`）+ 异步队列（`CAN_Tx_Submit` + `BSP_CAN_SendAsync`），`CanTxTask` 1ms 周期执行。已修复 FDCAN2 Message RAM 重叠（`MessageRAMOffset` 0→853，三路均匀三等分 0/853/1706）、`BSP_CAN_SendMsg` 缺 `len==0` 保护、`CanTxTask` 的 `vTaskDelayUntil` 周期写法（`xLastWakeTime` 移出循环）。待确认：FDCAN2 `AutoRetransmission=DISABLE` 与 FDCAN1/3 不一致 |
| UART BSP | 已实现（未接入设备） | `bsp_uart` 双缓冲 DMA 接收：`HAL_UARTEx_ReceiveToIdle_DMA` + IDLE 中断 + `Rx_Buffer_0/1` 交替收不定长帧。接管 7 路有 RX DMA 的口（USART1/2/3、UART5、USART6、UART7、USART10）；UART4/8/9 因 DMA stream 占满不接管；UART5 无 TX DMA 时发送自动回退阻塞。管理对象入 `.dma_buffer`。回调用 `UART_Init(huart, cb)` 绑定，可传 `nullptr` 退化轮询。尚未在 `Init.cpp` 接入具体设备 |

最近一次本地构建结果：

| 内存区域 | 使用量 | 总量 | 使用率 |
| --- | ---: | ---: | ---: |
| DTCMRAM | 103968 B | 128 KB | 79.32% |
| RAM_D1 | 6336 B | 320 KB | 1.93% |
| RAM_D2 | 0 B | 32 KB | 0.00% |
| RAM_D3 | 0 B | 16 KB | 0.00% |
| ITCMRAM | 0 B | 64 KB | 0.00% |
| FLASH | 150608 B | 1024 KB | 14.36% |

## 目录结构

```text
Core/                         STM32CubeMX 生成代码，除 USER CODE 区外谨慎修改
Drivers/                      STM32 HAL/CMSIS 驱动库
Middlewares/                  ST/FreeRTOS/USB 等第三方中间件
SystemView/                   SEGGER SystemView 与 RTT 支持
USB_DEVICE/                   STM32 USB Device CDC 相关代码
User_Config/                  工具与补丁配置，包含 FreeRTOS patched port
 User_File/Device/Onboard/      板载器件封装（BMI088、WS2812、Buzzer 等）
 User_File/Device/Peripheral/  外接器件（QD4310 电机、EricTool 等）
 User_File/Middleware/BSP/     外设 BSP 抽象层
 User_File/System/             系统服务，如初始化、回调、时间戳
 User_File/Middleware/Algorithm/算法组件，如矩阵、PID、EKF、四元数等
 User_File/Task/               FreeRTOS 用户任务
```

## 启动与运行链路

系统启动大致按下面的顺序运行：

- `main()` 先执行 `MPU_Config()`，将 RAM_D1 所在区域配置为 non-cacheable，以便 DMA1/DMA2 访问 `.dma_buffer` 中的缓冲区时不受 D-Cache 一致性问题影响。
- `HAL_Init()`、系统时钟、公共外设时钟和 CubeMX 外设初始化依次完成，包括 GPIO、DMA、MDMA、FDCAN、SPI、UART、TIM、ADC 等。
- `System_Init()` 在 RTOS 内核启动前执行，完成 SystemView、时间戳、EXTI 优先级、SPI2/SPI6 BSP 绑定、TIM5 启动和 BMI088 初始化。
- `osKernelInitialize()` 后创建 `TransportTask` 与 `InsTask`。
- `osKernelStart()` 启动调度器后，姿态解算不再依赖高频定时器轮询，而由 BMI088 数据就绪中断和 SPI DMA 完成回调驱动。

当前的核心数据流：

```text
BMI088 数据就绪 EXTI
    -> HAL_GPIO_EXTI_Callback
    -> BSP_BMI088.EXTI_Flag_Callback
    -> 发起 SPI2 DMA 读取
    -> HAL_SPI_TxRxCpltCallback
    -> SPI2_Callback
    -> BSP_BMI088.SPI_RxCpltCallback
    -> 陀螺仪数据完成时 vTaskNotifyGiveFromISR(InsTask)
    -> Ins_Task 被唤醒
    -> BSP_BMI088.EKF_Calculate()
```

## 已实现功能详解

### 构建系统

- CMake 工程已经接入用户源码、SystemView 源码和 CMSIS-DSP 库。
- `CMakePresets.json` 提供 `Debug` 与 `Release` preset，生成器为 Ninja。
- 根 `CMakeLists.txt` 从 STM32CubeMX 子工程中读取 FreeRTOS 源码列表，并过滤原始 `port.c`，改用 `User_Config/FreeRTOS_Patch/port_patched.c`。
- 链接 `arm_cortexM7lfsp_math`，算法层可以使用 CMSIS-DSP 的 Cortex-M7 FPU 优化库。

常用构建命令：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出文件：

```text
build/Debug/H7_BSP.elf
```

### 烧录与调试入口

VS Code 任务中已经配置了多种烧录方式：

- `Flash H7_BSP (ST-Link/OpenOCD)`：使用 OpenOCD 直接烧录并校验 ELF。
- `Flash H7_BSP (J-Link/Ozone)`：调用 Ozone 工程下载。
- `Generate J-Link Flash Script`：生成 J-Link CLI 所需脚本。
- `Flash H7_BSP (J-Link/CLI)`：通过 J-Link 命令行烧录。

调试配置中也保留了 J-Link GDB Server 与 OpenOCD 调试入口。

### 内存与 DMA 布局

STM32H7 的 DTCMRAM 无法被 DMA1/DMA2 直接访问。项目已经为 SPI/ADC DMA 缓冲区建立专用段：

```ld
.dma_buffer (NOLOAD) :
{
  . = ALIGN(32);
  _sdma_buffer = .;
  *(.dma_buffer)
  *(.dma_buffer*)
  . = ALIGN(32);
  _edma_buffer = .;
} >RAM_D1
```

已放入 `.dma_buffer` 的对象：

- `SPI1_Manage_Object` 到 `SPI6_Manage_Object`
- `ADC1_Manage_Object` 到 `ADC3_Manage_Object`

同时，`main.c` 的 MPU 配置将 `0x24000000` 起始的 RAM_D1 区域设置为 non-cacheable。这解决了 H7 上常见的“全局 DMA 缓冲区落在 DTCMRAM 导致 DMA 读写失败”问题。

### SystemView 集成

- `SystemView/SEGGER`、`SystemView/FreeRTOS`、`SystemView/Config` 已加入 include 路径。
- SystemView 源文件已加入构建。
- `System_Init()` 中调用 `SEGGER_SYSVIEW_Conf()`。
- FreeRTOS port 使用 patched 版本，便于接入运行时追踪探针。
- `configGENERATE_RUN_TIME_STATS` 与 `configUSE_TRACE_FACILITY` 已启用。

### FreeRTOS 配置与任务

当前创建五个 CMSIS-RTOS V2 任务：

| 任务 | 优先级 | 栈大小 | 当前职责 |
| --- | --- | ---: | --- |
| `InsTask` | `osPriorityHigh1` | `2048 * 4` 字节 | 等待陀螺仪 SPI 完成通知，执行 BMI088 EKF 姿态解算 |
| `CanTxTask` | `osPriorityHigh` | `1024 * 4` 字节 | 1ms 周期发送：先排空异步队列（`BSP_CAN_SendAsync`），再发送三路周期帧（`BSP_CAN_SendPer`） |
| `TIM_1ms_Task` | `osPriorityLow` | — | 1ms 模式分频器：pulse() 调度 1ms/10ms/50ms/128ms 周期回调 |
| `TransportTask` | `osPriorityNormal` | `2048 * 4` 字节 | 初始化 USB Device，后续预留通信传输逻辑 |
| `StatusTask` | `osPriorityLow` | `1024 * 4` 字节 | 系统状态监控骨架，当前 100 ms 周期让出 CPU |

栈溢出保护已经开启：

- `configCHECK_FOR_STACK_OVERFLOW` 设置为 `2`。
- `vApplicationStackOverflowHook()` 会记录溢出任务名并触发断点。

### 后续任务拆分原则

STM32H723ZG 可以承载多个 FreeRTOS 应用任务，实际限制主要来自任务栈、FreeRTOS heap、CPU 时间预算和优先级设计，而不是任务数量本身。不要为了减少任务数，把云台、底盘、发射、通信和调试逻辑都塞进 `InsTask`。

`InsTask` 应保持为姿态数据生产者：等待 BMI088 数据完成通知，执行 `BSP_BMI088.EKF_Calculate()`，发布最新姿态快照，必要时低频更新调试镜像或通知其他任务。云台、底盘、发射等解算应作为姿态数据消费者独立运行。

推荐后续任务职责：

| 任务 | 建议职责 | 备注 |
| --- | --- | --- |
| `InsTask` | BMI088/EKF 姿态解算，发布 `INS_State` 快照 | 高优先级、短路径、避免阻塞 |
| `GimbalTask` | 读取姿态快照和控制输入，执行云台 yaw/pitch 解算 | 可由 INS 更新通知或固定周期触发 |
| `ChassisTask` | 读取电机反馈、遥控输入和姿态快照，执行底盘解算 | 按底盘控制周期运行 |
| `ShootTask` | 发射机构状态机和控制 | 可后续按复杂度拆出 |
| `TransportTask` | USB/串口/遥测/上位机协议 | 低于控制链路优先级，避免阻塞高优先级任务 |

任务之间推荐传递“事件 + 最新状态快照”，而不是让多个任务直接访问 BMI088 内部对象。`InsTask` 可维护一个扁平的 `INS_State`：

```cpp
typedef struct
{
  uint32_t sequence;
  uint64_t timestamp_us;
  float yaw_rad;
  float pitch_rad;
  float roll_rad;
  float gyro_x_rad_s;
  float gyro_y_rad_s;
  float gyro_z_rad_s;
  float accel_x_m_s2;
  float accel_y_m_s2;
  float accel_z_m_s2;
} Struct_INS_State;
```

事件通知建议优先使用 FreeRTOS direct task notification bits，而不是“二值信号量 + 全局任务标记”。普通信号量只能表达“有事发生”，不适合同时表达云台、底盘、发射等多个事件；全局标记在多个事件连续到来时容易被覆盖。事件 bit 可合并多个待处理事件：

```cpp
enum Enum_Solve_Event
{
  SOLVE_EVENT_INS_UPDATE = 1U << 0,
  SOLVE_EVENT_GIMBAL = 1U << 1,
  SOLVE_EVENT_CHASSIS = 1U << 2,
  SOLVE_EVENT_SHOOT = 1U << 3,
};
```

短期可用单个 `SolveTask` 读取 notification bits 后按事件分发，便于快速验证；长期建议将 `GimbalTask`、`ChassisTask`、`ShootTask` 按实时性和职责拆开，分别设置优先级和周期。

新增任务时不要无脑给 8 KB 栈。`InsTask` 因 EKF 矩阵临时对象较多可先保持 8 KB，云台/底盘/发射任务应从 2 KB 到 4 KB 起步，并用 `uxTaskGetStackHighWaterMark()`、`xPortGetFreeHeapSize()` 和 SystemView 观察实际余量与执行时间。

通信也应按实时性分层，而不是全部塞进 `TransportTask`。控制链路通信要短、确定、尽量非阻塞；调试和上位机通信可以放低优先级任务。

推荐通信职责划分：

| 通信类型 | 建议位置 | 设计原则 |
| --- | --- | --- |
| CAN 电机反馈 | CAN RX 回调或轻量反馈分发层 | 回调内只取报文、更新反馈或投递事件，不做复杂控制 |
| CAN 电机发送 | 控制任务直接非阻塞发送，复杂后拆 `CanTxTask` | 云台/底盘只生成目标值，复杂发送仲裁由发送任务统一处理 |
| USB/串口遥测 | `TransportTask` | 低优先级，适合上位机、调试数据、命令解析 |
| 遥控/视觉/裁判系统 | 独立输入任务或通信解析任务 | 根据是否参与控制链路设置优先级，解析结果发布为状态快照 |

`TransportTask` 更适合承担 USB CDC、串口遥测、上位机协议和低频调试输出，不应承担高实时电机控制、姿态解算、云台核心控制或底盘核心控制。若后续 `TransportTask` 主要用于 USB/串口遥测，优先级应低于 `InsTask`、`GimbalTask` 和 `ChassisTask`。

当前 CAN BSP 已在 `User_File/Middleware/BSP/CAN/` 完成实现（`bsp_can` v2），采用**双通道发送架构**：

- **周期通道**：`CAN_Tx_Perform()` 写入三路 FDCAN 缓冲区，`BSP_CAN_SendPer()` 在 `CanTxTask` 每个 1ms 周期内批量发送，适用于控制帧（电机指令）。
- **异步通道**：`CAN_Tx_Submit()` 将消息压入深度为 16 的 RTOS 消息队列（按值拷贝，调用方可用栈变量），`BSP_CAN_SendAsync()` 在 `CanTxTask` 每个周期内排空队列，适用于非实时指令（传感器配置、模式切换）。
- **底层发送函数** `BSP_CAN_SendMsg()` 已改为 `static`，不再对外暴露；外部统一通过两条通道接口使用。
- `BSP_CAN_ConfigInit()` 在 RTOS 内核启动后、`MX_FREERTOS_Init()` 任务创建前调用，内部同时创建三路发送互斥锁和异步消息队列。CAN 发送互斥锁使用 CMSIS-RTOS V2 的 `osMutexId_t`，最多等待 **2ms** 后放弃，避免阻塞控制任务。`BSP_CAN_SendMsg()` 是任务上下文接口，不应在中断回调中直接调用。

### 系统初始化

`System_Init()` 当前负责：

- 配置 SystemView。
- 绑定 `SYS_Timestamp` 到 `htim5`。
- 将 `EXTI15_10_IRQn` 优先级设为 `5`，与 FreeRTOS 可调用 ISR API 的临界优先级保持一致。
- 将 SPI2 绑定到 `SPI2_Callback()`，用于 BMI088。
- 将 SPI6 注册到 SPI BSP，当前未绑定回调，预留给 WS2812。
- 启动 TIM5 中断。
- 初始化 `BSP_BMI088`。
- 设置全局 `init_finished` 标志，避免初始化未完成时处理中断回调。

### 回调分发中心

`User_File/System/callback/callback.cpp` 当前提供统一 HAL 回调分发：

- `HAL_GPIO_EXTI_Callback()`：过滤 BMI088 加速度计和陀螺仪数据就绪引脚，转交给 `BSP_BMI088.EXTI_Flag_Callback()`。
- `HAL_TIM_PeriodElapsedCallback()`：保留 TIM2 HAL Tick 递增，TIM5 用于时间戳小时级溢出计数，TIM6/TIM7 分支仍为空或注释。
- `SPI2_Callback()`：根据 SPI2 当前片选目标判断本次传输属于 BMI088 加速度计还是陀螺仪，然后调用 `BSP_BMI088.SPI_RxCpltCallback()`。

高频 10 us / 125 us 定时器回调已经被注释废弃，原因是高频中断会干扰 RTOS 调度。当前设计转向 EXTI + DMA + 任务通知。

### 时间戳系统

`SYS_Timestamp` 是全局时间戳对象，绑定 TIM5：

- TIM5 预期以 1 MHz 计数，即 1 tick = 1 us。
- 定时器溢出按 3600 秒累计一次。
- `Get_Current_Timestamp()` / `Get_Now_Microsecond()` 提供微秒级时间。
- `Delay_Second()`、`Delay_Millisecond()`、`Delay_Microsecond()` 使用忙等实现，主要用于 RTOS 启动前的设备寄存器初始化阶段。

### SPI BSP

SPI BSP 当前采用 C 风格管理对象 + 自由函数：

- 每个 SPI 外设有一个 `Struct_SPI_Manage_Object`。
- 管理对象包含 HAL handle、片选 GPIO、Tx/Rx 缓冲区、长度、时间戳和完成回调。
- SPI1 到 SPI5 使用 DMA 发送或 DMA 全双工收发。
- SPI6 因 BDMA 可访问区域限制，当前走阻塞式 `HAL_SPI_Transmit()` / `HAL_SPI_TransmitReceive()`。
- HAL 完成回调中会拉回片选电平、记录接收时间戳，并调用注册的业务回调。

当前 SPI2 已服务 BMI088，SPI6 预留给 WS2812。

### UART BSP

UART BSP 采用双缓冲 DMA 接收范式（仿 SCUT-Robotlab / 达妙 `drv_uart` 改写）：

- 每个被接管的 UART 有一个 `Struct_UART_Manage_Object`，含 HAL handle、双缓冲 `Rx_Buffer_0/1`、Active/Ready 指针、`Rx_Timestamp` 和业务回调。
- 接收用 `HAL_UARTEx_ReceiveToIdle_DMA`，IDLE 中断触发 `HAL_UARTEx_RxEventCallback`：先切换双缓冲、重启 DMA，再调用业务回调，实现收不定长帧且不丢数据。
- 仅接管具备 RX DMA 的 7 路：USART1/2/3、UART5、USART6、UART7、USART10。**UART4/8/9 因 STM32H7 DMA1+DMA2 共 16 条 stream 已被占满（7 路 UART 收发 + SPI + ADC），分不到 DMA，未接管**；如需使用应改中断/阻塞方式单独实现。
- UART5 仅有 RX DMA、无 TX DMA，`UART_Transmit_Data()` 检测 `huart->hdmatx == nullptr` 时自动回退 `HAL_UART_Transmit()` 阻塞发送。
- 管理对象（含双 512B 缓冲）放入 `.dma_buffer`，落入 RAM_D1。
- HAL 回调（`HAL_UARTEx_RxEventCallback` / `HAL_UART_ErrorCallback`）显式用 `extern "C"` 以正确覆写 HAL 弱符号。

回调注册模型与 SPI BSP 同构（「实例分支 + 单回调」），区别于 CAN BSP 的「CAN ID 查表」注册表模型：

- `UART_Init(huart, callback)` 把 `void(uint8_t*, uint16_t)` 回调绑定到对应管理对象，同时启动 DMA 接收。
- 回调可传 `nullptr` 退化为轮询模式：DMA 照常接收、双缓冲照常切换、`Rx_Timestamp` 照常更新，但不触发回调，由任务自行轮询 `Rx_Buffer_Ready`。
- 错误中断 `HAL_UART_ErrorCallback` 统一调用 `UART_Reinit()` 重启接收。

当前 UART BSP 已在 `System_Init()` 中以轮询模式（`nullptr` 回调）初始化全部 10 路 UART。具体设备的业务回调绑定（如 DBUS 遥控器 / 裁判系统）待后续按实际外接配置接入。

### ADC BSP

ADC BSP 已提供基础 DMA 初始化：

- `ADC1_Manage_Object` 到 `ADC3_Manage_Object` 已放入 `.dma_buffer`。
- `ADC_Init()` 会执行 ADC 校准，并根据实例启动对应 ADC DMA。

需要注意：当前 `System_Init()` 没有调用 `ADC_Init()`，因此 ADC BSP 是“可用模块”，但还没有纳入系统启动链路。

### Power 组件

`BSP_Power` 已提供板载电源控制抽象：

- 控制两个 24V 输出与一个 5V 输出。
- 支持从 ADC 缓冲区读取电源电压估算值。
- 可通过 `Set_DC24_0()`、`Set_DC24_1()`、`Set_DC5()` 控制 GPIO 输出。

当前限制：

- `System_Init()` 尚未调用 `BSP_Power.Init()`。
- ADC 采样链路未在系统初始化中启动。
- BMI088 加热器默认关闭，因此暂未依赖电源电压闭环计算。

### BMI088 设备层

BMI088 是当前最完整的设备链路，由 `Class_BMI088` 管理加速度计、陀螺仪和 EKF。

加速度计已实现：

- SPI2 绑定与片选绑定。
- 芯片 ID 检测、软重启、寄存器配置确认。
- 量程配置为 ±24g。
- 输出数据率配置为 1600 Hz。
- 数据就绪中断映射到 INT1。
- 加速度原始值转换为 m/s²。
- 温度读取与温控 PID 代码。
- 加热 PWM 逻辑，包含预热、PID 控制和 NaN 防护。

陀螺仪已实现：

- SPI2 绑定与片选绑定。
- 芯片 ID 检测、软重启、寄存器配置确认。
- 量程配置为 2000 dps。
- 反馈频率配置为 2000 Hz，带宽 532 Hz。
- 数据就绪中断映射。
- 角速度原始值转换为 rad/s。
- 数据合法性检查。

BMI088 总控已实现：

- 根据 EXTI 标记加速度计/陀螺仪数据就绪。
- 在没有 SPI 传输进行时按优先级发起读取请求。
- SPI 完成回调中更新数据标志和时间戳。
- 陀螺仪数据完成后从 ISR 通知 `InsTask`。
- 移除了“在 SPI 回调中再发起新 DMA 传输”的旧模式，避免 DMA-in-DMA 竞态。

### 姿态 EKF 链路

`BSP_BMI088.EKF_Calculate()` 当前由 `InsTask` 被通知后执行。核心逻辑：

- 获取最新加速度计与陀螺仪原始数据。
- 陀螺仪数据非法时复用上一次合法值。
- 加热器启用时应用加速度仿射校正和陀螺仪零偏修正。
- 使用加速度方向初始化四元数，Yaw 初始为 0。
- 配置 EKF 非线性状态模型、测量模型及其雅可比函数。
- 每次计算根据微秒时间戳更新 `D_T`。
- 使用陀螺仪作为输入进行预测。
- 加速度数据有效时执行卡方检验，通过后进行观测更新。
- 对四元数归一化。
- 输出四元数、欧拉角、旋转矩阵、轴角、机体系/大地系加速度和角速度。
- 更新 Ozone Timeline Data Plot 使用的全局浮点数组。

### 算法库

算法目录已经纳入构建，主要包含：

- `Basic`：基础数学工具。
- `Complex`：复数运算。
- `Matrix`：定长矩阵模板与常用矩阵构造。
- `Quaternion`：四元数、欧拉角、旋转矩阵、轴角转换。
- `PID`：PID 控制器，已被 BMI088 加热器逻辑使用。
- `Filter/EKF`：扩展卡尔曼滤波模板，当前姿态解算正在使用。
- `Filter/Kalman`：线性 Kalman 滤波模板。
- `Filter/Frequency`：频域/频率滤波组件。
- `FSM`：有限状态机模板。
- `Queue`：固定容量队列模板。
- `Slope`：斜坡规划器。

这些模块大多是可复用中间件，当前直接参与主链路的是矩阵、四元数、PID 和 EKF。

## 未实现与待完成内容

### TransportTask 仍是骨架

当前 `TransportTask` 已经从空文件补齐为可构建任务：

- 调用 `MX_USB_DEVICE_Init()`。
- 在循环中 `osDelay(1)` 让出 CPU。

尚未实现：

- USB CDC 数据收发协议。
- 与上位机/裁判系统/调试工具的数据格式约定。
- 发送 BMI088 姿态、IMU 原始数据、系统状态等 telemetry。
- 接收控制命令或参数调试命令。
- 与 UART/CAN/FDCAN 的桥接或路由。
- 队列、互斥、事件标志等 RTOS 通信机制。

### CAN 发送通道选择

| 场景 | 推荐接口 | 原因 |
| --- | --- | --- |
| 电机控制帧（高频、实时） | `CAN_Tx_Perform()` + `BSP_CAN_SendPer()` | 按节拍精确发出，不受队列深度限制 |
| 传感器配置/使能指令（低频、非实时） | `CAN_Tx_Submit()` | 入队后在下一个 1ms 周期发出，无需关心对象生命周期 |

`CAN_Tx_Submit()` 队列满（16 帧）时返回 `false`，调用方应检查返回值。若频繁满队，应改用周期通道或降低提交频率。

### Power/ADC 已接入系统初始化

Power 和 ADC 模块已在 `System_Init()` 中调用初始化：

- `ADC_Init(&hadc1, 1)`：启动 ADC1 DMA 采样。
- `BSP_Power.Init(true, true, false)`：使能两个 24V 输出，5V 暂不使能。

电源电压检测依赖 ADC 缓冲区数据，当前链路已接通。

### BMI088 温控已形成基本闭环

加速度计温度读取和加热 PID 代码已经实现。`BMI088_TIM_128ms_Calculate_PeriodElapsedCallback()` 已接入 `TIM_1ms_Task` 的 `pulse(128, ...)` 调度。

注意：

- 加热器默认关闭（`Init(false)`），若启用需确保 `BSP_Power` 和 ADC 电压采样有效（当前链路已接通）。
- 温度定期读取和加热 PID 已接入 128 ms 调度。

### 定时器回调仍有空分支

当前回调中心中存在预留但未实现的分支：

- `Task1s_Callback()` 为空。
- TIM7 的 1 ms 回调逻辑被注释。
- 10 us / 125 us 高频回调被废弃并保留注释。
- TIM6 分支存在，但当前 `main.c` 中没有看到 `MX_TIM6_Init()` 调用。

这说明定时器框架已经搭好，但周期任务调度策略还未最终收口。

### 并发标志需要更严格语义

中断和任务之间共享了一些状态标志。全局 `init_finished` 已改为 `volatile bool`，但 BMI088 内部仍有若干普通 `bool` 标志，例如：

- BMI088 内部的 `Init_Finished_Flag`。
- Accel/Gyro/Temperature ready、transfering、update 标志。

当前 Debug / `-O0` 下问题不明显，但这些变量跨 ISR 与任务上下文使用，后续建议根据访问路径改成 `volatile`、临界区保护、原子语义或 RTOS 同步对象。

### SPI 边角问题

SPI5 全双工完成回调的 Tx 长度参数已修正为 `Tx_Buffer_Length`。当前仍建议后续继续检查 SPI BSP 的边界条件：

- SPI1 到 SPI5 当前代码重复度较高，后续可在 BSP 重构时收敛。
- SPI6 仍因 BDMA 内存可访问限制使用阻塞传输。
- SPI 回调中不要恢复“完成回调内继续发起新 DMA”的旧模式，以免 DMA-in-DMA 竞态复发。

### EKF 栈压力仍需观察

仓库备忘中记录过：EKF 矩阵运算在 Debug / `-O0` 下可能产生较多临时矩阵对象，增加任务栈压力。当前 `InsTask` 已给到 8KB 栈，并启用了栈溢出 hook，但后续仍建议：

- 观察 `uxTaskGetStackHighWaterMark()`。
- 拆分过长矩阵表达式。
- 必要时提升优化等级或改写关键矩阵运算。

### 版本状态待整理

当前工作区存在大量未提交修改和未跟踪文件。建议在下一阶段按主题拆分提交：

- CubeMX 外设配置变更。
- FreeRTOS/SystemView 集成。
- DMA 内存布局修复。
- BMI088 姿态链路。
- Task 模块新增。
- README 与更新日志文档。

## C/C++ 协作设计

本工程底层由 CubeMX、HAL、FreeRTOS、USB Device 等 C 代码组成，用户层主要使用 C++ 类和全局对象。推荐采用“双层结构”：C 侧只调用稳定的 C ABI 门面函数，C++ 侧负责对象、设备、算法和任务逻辑。

```text
CubeMX C / HAL C / FreeRTOS C
  |
  | 只调用 extern "C" 函数
  v
C ABI 边界层
  |
  | 转交初始化、回调、任务入口
  v
C++ BSP / Device / Algorithm / Task
```

### 边界层职责

不要建立一个巨大的 `c_cpp_bridge.cpp`。更合适的方式是按职责拆成很薄的边界文件：

| 文件 | 角色 | 说明 |
| --- | --- | --- |
| `System/Init/Init.h/.cpp` | 启动边界 | `main.c` 只调用 `System_Init()`，具体初始化在 C++ 中完成 |
| `System/callback/callback.h/.cpp` | HAL 回调边界 | HAL C 回调进入后转交给 C++ 设备对象 |
| `User_File/Task/user_task.h` | 任务入口边界 | `freertos.c` 只看到 `Ins_Task(void *)` / `Transport_Task(void *)` |
| `Task/*.cpp` | 任务实现 | 任务入口是 C ABI，任务内部调用 C++ 对象 |
| `System/Debug/*` | 调试边界 | 后续用于 Ozone/SystemView/RTT 调试探针 |

### 头文件规则

- C 可见区域只放 C 兼容函数声明。
- C 可见头文件需要自己包含 `<stdint.h>`、`<stdbool.h>` 等基础类型来源。
- C 可见函数不要使用 C++ 引用、类、模板、默认参数、重载函数或 `std::` 类型。
- C++ 类头尽量放在 `.cpp`，必要时放进 `#ifdef __cplusplus`。
- C 文件不要 include `bsp_bmi088.h`、`alg_matrix.h`、`alg_filter_ekf.h` 这类 C++ 头。

### 当前已收口的边界

- `System_Init()` 定义已显式使用 `extern "C"`。
- `HAL_GPIO_EXTI_Callback()`、`HAL_TIM_PeriodElapsedCallback()` 已显式使用 `extern "C"`。
- `HAL_SPI_TxCpltCallback()`、`HAL_SPI_TxRxCpltCallback()` 已显式使用 `extern "C"`。
- `SPI2_Callback()` 已显式使用 `extern "C"`，头文件只暴露 C 风格原型。
- `Ins_Task()`、`Transport_Task()` 已显式使用 `extern "C"`。
- `Init.h`、`callback.h`、`user_task.h` 的 C 可见区域已净化，C++ 依赖移到实现文件。

### 全局对象规则

C++ 全局对象可以存在，但构造函数不要进行硬件操作。硬件绑定、HAL 调用、DMA 启动统一放进显式 `Init()`：

- `BSP_BMI088` 可以是全局对象，但 SPI 绑定和寄存器初始化放在 `BSP_BMI088.Init()`。
- `SYS_Timestamp` 可以是全局对象，但 TIM 绑定放在 `SYS_Timestamp.Init(&htim5)`。
- C 文件不要直接访问 C++ 全局对象，必须通过 C ABI 函数或任务入口间接触发。

### 符号检查

构建后可使用 `nm` 检查关键 C ABI 符号是否保持原名：

```powershell
arm-none-eabi-nm build/Debug/H7_BSP.elf | findstr "System_Init Ins_Task Transport_Task HAL_GPIO_EXTI_Callback HAL_TIM_PeriodElapsedCallback HAL_SPI_TxRxCpltCallback"
```

如果看到 `_Z...` 形式的符号名，说明仍然存在 C++ name mangling，需要检查对应函数是否缺少 `extern "C"`。

## Ozone 调试设计

Ozone 直接观察 C++ class、模板矩阵和嵌套对象时会很冗长。推荐单独建立 Ozone 调试镜像层，把复杂 C++ 状态转换成扁平、带单位、便于 Watch 和 Timeline Data Plot 使用的 C 风格全局变量。

建议后续新增：

```text
User_File/System/Debug/sys_debug_ozone.h
User_File/System/Debug/sys_debug_ozone.cpp
```

推荐数据形式：

```cpp
typedef struct
{
    uint32_t sequence;

    float timestamp_us;
    float calculate_time_us;

    float accel_raw_x_m_s2;
    float accel_raw_y_m_s2;
    float accel_raw_z_m_s2;

    float gyro_raw_x_rad_s;
    float gyro_raw_y_rad_s;
    float gyro_raw_z_rad_s;

    float euler_yaw_deg;
    float euler_pitch_deg;
    float euler_roll_deg;

    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;

    float accel_chi_square;
} Struct_Debug_Ozone_IMU;

extern volatile Struct_Debug_Ozone_IMU DBG_Ozone_IMU;
```

实现中建议使用：

```cpp
__attribute__((used))
volatile Struct_Debug_Ozone_IMU DBG_Ozone_IMU = {0};
```

设计要点：

- Ozone 看 `DBG_Ozone_IMU.euler_yaw_deg` 比展开 `Class_Matrix_f32<3, 1>` 更清晰。
- 字段名直接带单位，例如 `_deg`、`_rad_s`、`_m_s2`、`_us`。
- `sequence` 可用于判断结构体是否正在更新。
- 调试变量普通放在 `.bss` 即可，现阶段不需要单独 section。
- 姿态角、原始 IMU、计算耗时适合 Ozone；任务切换和中断频率适合 SystemView；文本日志适合 RTT。
- 当前 `ozone_accel`、`ozone_gyro`、`ozone_euler` 已经是调试镜像雏形，后续可迁移到 `sys_debug_ozone.*`。

## CMake Tools 解析提示

如果 VS Code 提示类似下面的信息：

```text
查找要添加的命令调用时发生分析错误
cmake/stm32cubemx/CMakeLists.txt:143:1: unexpected #
```

优先区分这是“CMake 本身错误”还是“VS Code/CMake Tools 的命令分析器错误”。当前 `cmake --build build/Debug` 可以成功，说明该文件对 CMake 来说是可解析的。第 143 行附近是生成文件里的注释/空行，更像扩展在尝试自动寻找 `target_sources()` 插入点时解析失败。

建议处理顺序：

- 先运行 `cmake --preset Debug` 或 VS Code 的 CMake reconfigure，让 CMake File API 刷新。
- 用户源码继续注册在根 `CMakeLists.txt`，尽量不要手动改 `cmake/stm32cubemx/CMakeLists.txt`。
- 如果该提示只在“自动添加源文件到 CMake”时出现，可以忽略自动添加，手动维护根 `CMakeLists.txt`。
- 新增用户源文件时，优先添加到根 `CMakeLists.txt` 的 `target_sources(${CMAKE_PROJECT_NAME} PRIVATE ...)`。
- 如果提示持续影响扩展功能，再考虑规范化生成文件的换行/格式，或清理 build 缓存后重新 configure。

当前仓库中已将 `cmake/stm32cubemx/CMakeLists.txt` 从混合换行规范化为 CRLF，并清理了 `MX_LINK_LIBS` 段尾随空白。`cmake --preset Debug` 与 `cmake --build --preset Debug` 均可正常执行。

VS Code 工作区设置保留 `cmake.cmakePath = cube-cmake`，并显式绑定 `cmake.configurePreset = Debug`、`cmake.buildPreset = Debug`。此前的 `-DCMAKE_COMMAND=cube-cmake` 会产生未使用变量警告，已移除。

## 开发约定

- 用户代码主要放在 `User_File/` 下。
- CubeMX 生成文件只在 `USER CODE BEGIN/END` 区域内修改。
- 新增用户模块时，需要同时注册源码与 include 路径到根 `CMakeLists.txt`。
- 使用 DMA 的全局缓冲区必须放在 DMA 可访问内存区域，当前推荐使用 `.dma_buffer`。
- FreeRTOS ISR API 必须在优先级不高于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 的中断中调用，当前关键外设优先级统一为 `5`。
- C++ 代码禁用异常和 RTTI，避免引入依赖异常机制的库或写法。

## 建议下一步

- 完成 `TransportTask` 的 USB CDC 或串口通信协议。
- 接入 `ADC_Init()` 与 `BSP_Power.Init()`，让电源电压检测和 BMI088 温控有真实数据来源。
- 决定 BMI088 加热器是否启用，并把 128 ms 温控周期接入回调或 RTOS timer。
- 把 ISR/任务共享标志做并发语义收口。
- 对 SPI BSP 做一次系统性重构，降低重复分支和边界错误概率。
- 对 `InsTask` 做栈水位观测，确认 EKF 在 Debug 和 Release 下的运行余量。

## 待迁移模块（对比 dm02_test 模板）

与达妙 MC-02 模板工程（`damiao_mc02_bsp/dm02_test`）对比，以下模块尚未迁移，按优先级排列：

### 🔴 高优先级

| 模块 | 模板路径 | 说明 |
| --- | --- | --- |
| DJI 电机 | `2_Device/Motor/Motor_DJI/dvc_motor_dji` | C610/C620/M3508/M2006，CAN 总线控制，机器人核心执行器 |
| 达妙电机 | `2_Device/Motor/Motor_DM/dvc_motor_dm` | DM 系列电机，CAN 总线控制 |

### 🟡 中优先级

| 模块 | 模板路径 | 说明 |
| --- | --- | --- |
| USB CDC 封装 | `1_Middleware/Driver/USB/drv_usb` | 底层协议栈（`USB_DEVICE/`）已由 CubeMX 生成，缺统一封装层 |
| Vofa+ 上位机 | `2_Device/Plotter/Vofa/dvc_vofa` | 调试期数据可视化工具，通过 USB/UART 发送帧格式数据 |
| 功率计 | `2_Device/Powermeter/dvc_powermeter` | 裁判系统功率监控 |

### 🟢 低优先级

| 模块 | 模板路径 | 说明 |
| --- | --- | --- |
| 看门狗 WDG | `1_Middleware/Driver/WDG/drv_wdg` | 独立看门狗，防止程序死锁 |
| Serialplot | `2_Device/Plotter/Serialplot/dvc_serialplot` | 另一种上位机调试工具 |
