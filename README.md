# H7_BSP

面向达妙 MC-02 开发板的 STM32H7 板级支持与机器人控制框架，基于 **STM32H723VGT6 / Cortex-M7 / 480 MHz**。工程围绕外设通信、设备驱动、控制与估计算法、系统服务组织代码，供机器人项目组合和复用。

底层使用 STM32CubeMX、HAL 与 FreeRTOS，任务接口采用 CMSIS-RTOS V2，构建使用 CMake + Ninja。用户层保持 C 风格运算、结构体与自由函数，设备和算法保留简洁的 `Class_` 封装。

[整体架构](#整体架构) · [通信与外设](#通信与外设-bsp) · [设备层](#设备层) · [算法层](#算法层) · [接入方式](#接入方式) · [构建与调试](#构建与调试)

## 整体架构

![H7_BSP 框架模块与主要使用关系](Assets/Architecture/H7_BSP.svg)

框架以模块职责划分边界：BSP 处理外设收发，Device 处理设备协议与状态，Algorithm 提供计算组件，System 提供共享服务；Task 和 Application 负责调度与业务组合。图中展示模块组织和主要使用关系。[交互图](Assets/Architecture/H7_BSP.html) 可下载后在本地浏览器打开。

| 层次 | 职责 | 入口 |
| --- | --- | --- |
| Application / Task | 组织控制逻辑、任务周期与模块协作 | [Application](User_File/Application)、[Task](User_File/Task) |
| Device | 封装电机、板载器件与外接工具 | [Device](User_File/Device) |
| Algorithm | 提供控制、观测、滤波、数学与调度辅助组件 | [Algorithm](User_File/Middleware/Algorithm) |
| System | 统一初始化、回调、时间戳与调试服务 | [System](User_File/System) |
| BSP | 管理外设实例、缓冲区、收发与回调注册 | [BSP](User_File/Middleware/BSP) |
| HAL / RTOS / 工程配置 | 外设初始化、任务调度、内存布局与构建 | [Core](Core)、[User_Config](User_Config)、[CMakeLists.txt](CMakeLists.txt) |

### 工程目录

```text
User_File/
├── Application/            应用控制与通信接入
├── Task/                   CMSIS-RTOS V2 任务入口
├── Device/Onboard/         板载设备
├── Device/Peripheral/      外接电机与调试工具
├── Middleware/Algorithm/   数学、控制、观测与滤波组件
├── Middleware/BSP/         外设管理与收发接口
└── System/                 初始化、回调、时间戳、参数与调试
Core/                       CubeMX 生成的启动、外设与 RTOS 配置
Drivers/                    HAL / CMSIS 驱动
Middlewares/                FreeRTOS、USB Device、CMSIS-DSP 等依赖
USB_DEVICE/                 USB CDC 设备配置
User_Config/                链接脚本、FreeRTOS 补丁与烧录配置
SystemView/                 SEGGER SystemView 与 RTT
sysid/                      系统辨识数据、脚本与报告
```

## 通信与外设 BSP

BSP 以外设管理对象和接口函数承接 HAL，设备层通过注册回调与收发接口使用总线资源。

| 模块 | 提供的能力 | 使用入口 |
| --- | --- | --- |
| CAN / FDCAN | 按总线与 ID 分发接收；命令队列与最新周期帧两条发送通道 | [bsp_can.h](User_File/Middleware/BSP/CAN/bsp_can.h) |
| UART | DMA + IDLE 不定长接收、双缓冲、接收回调与错误恢复 | [bsp_uart.h](User_File/Middleware/BSP/UART/bsp_uart.h) |
| SPI | 外设管理、片选、收发缓冲与完成回调 | [SPI](User_File/Middleware/BSP/SPI) |
| USB | CDC 收发封装，供调试通信组件使用 | [USB](User_File/Middleware/BSP/USB) |
| OSPI | 外部存储器收发与自动轮询接口 | [OSPI](User_File/Middleware/BSP/OSPI) |
| ADC | 校准、DMA 采样与采样缓冲管理 | [ADC](User_File/Middleware/BSP/ADC) |

CAN 的两条发送通道适用于不同数据语义：

- `CAN_Tx_Perform()` 更新 `(FDCAN, ID)` 对应的周期槽，同一键保留最新数据，适合连续控制目标。
- `CAN_Tx_Submit()` 将命令复制到 FIFO 队列，适合使能、复位与模式设置等需要按顺序处理的操作。
- `CanTxTask` 调用 `BSP_CAN_SendAsync()` / `BSP_CAN_SendPer()` 处理发送。调用方需要检查提交结果；进入软件缓冲与总线发送完成是不同阶段。

CAN 接收回调在中断上下文执行。UART 的 DMA 接收须同时具备 CubeMX 的 RX DMA 配置和 BSP 管理入口，接入新端口时需同步核对。

## 设备层

设备层维护协议编解码、状态、反馈与设备操作，复用 BSP 通信接口和系统时间服务。

### 电机组件

| 驱动 | 支持范围 | 接入说明 |
| --- | --- | --- |
| DJI | M2006/C610、M3508/C620、GM6020；反馈、控制环与分组发送 | [DJI 电机驱动](User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md) |
| 达妙 | MIT、位置-速度、速度、力位混控接口；实际模式取决于型号与固件 | [达妙电机驱动](User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md) |
| QDrive | QD4310 协议与控制接口 | [QDrive](User_File/Device/Peripheral/Motor/QDrive) |

电机型号、CAN ID、反馈源、方向、映射范围和控制参数由使用方配置；应用层负责控制周期、目标生成与输出边界。

### 板载设备与外接工具

| 组件 | 功能 |
| --- | --- |
| BMI088 | 加速度计、陀螺仪、温控与姿态组件；当前采集采用 FIFO / SPI DMA，姿态使用 VQF |
| W25Q64JV | 基于 OSPI 的外部 Flash 驱动 |
| Power | 板载电源输出控制与 ADC 电压采样 |
| WS2812 / Buzzer / Key | 灯效、蜂鸣器与按键处理 |
| EricTool | USB / UART 调试通信与数据输出 |

板载组件位于 [Onboard](User_File/Device/Onboard)，外接组件位于 [Peripheral](User_File/Device/Peripheral)。硬件资源绑定和设备初始化集中在 [Init.cpp](User_File/System/Init/Init.cpp)。

## 算法层

算法按用途独立组织，设备和应用通过输入、参数与计算接口组合使用。现有源码在根 CMake 中显式注册；组件进入构建不代表已经接入具体控制闭环。

| 分类 | 组件 | 内容 |
| --- | --- | --- |
| 控制 | [PID](User_File/Middleware/Algorithm/PID) | 通用 PID 控制器 |
| 控制 | [SMC](User_File/Middleware/Algorithm/SMC) | 单轴二阶对象滑模控制，线性滑模面与饱和边界层 |
| 观测 | [DOB](User_File/Middleware/Algorithm/DOB) | 一阶名义模型、零阶保持离散与 Q 滤波扰动估计 |
| 状态估计 | [Kalman](User_File/Middleware/Algorithm/Filter/Kalman)、[EKF](User_File/Middleware/Algorithm/Filter/EKF) | 线性与扩展卡尔曼滤波组件 |
| 姿态估计 | [VQF](User_File/Middleware/Algorithm/Filter/VQF) | 姿态与陀螺仪零偏估计 |
| 信号滤波 | [Frequency](User_File/Middleware/Algorithm/Filter/Frequency)、[IIR](User_File/Middleware/Algorithm/Filter/IIR) | FIR 频率滤波与 IIR 低通、陷波等组件 |
| 数学 | [Basic](User_File/Middleware/Algorithm/Basic)、[Complex](User_File/Middleware/Algorithm/Complex)、[Matrix](User_File/Middleware/Algorithm/Matrix)、[Quaternion](User_File/Middleware/Algorithm/Quaternion) | 基础运算、复数、定长矩阵与姿态表示转换 |
| 辅助 | [Slope](User_File/Middleware/Algorithm/Slope)、[FSM](User_File/Middleware/Algorithm/FSM)、[Queue](User_File/Middleware/Algorithm/Queue)、[Pulse](User_File/Middleware/Algorithm/Pulse) | 斜坡、状态机、队列与周期分频 |

使用算法时需要明确量纲、采样周期、状态初始化和输出限幅。模型相关约定以模块头文件为准，例如 DOB 使用 `y[k]` 与上一周期实际输入 `u[k-1]`，SMC 由调用方提供同一时刻的状态及其导数。

## 系统服务

| 服务 | 作用 | 入口 |
| --- | --- | --- |
| 初始化 | 绑定 BSP、配置设备与建立系统服务 | [System/Init](User_File/System/Init) |
| 回调分发 | 将 HAL 回调转交外设与设备处理逻辑 | [System/callback](User_File/System/callback) |
| 时间戳 | 提供统一微秒时间，供周期测量与超时判断使用 | [System/Timestamp](User_File/System/Timestamp) |
| 参数配置 | 集中维护当前 IMU 采样、姿态与零偏估计参数 | [System/IMU](User_File/System/IMU) |
| 调试数据 | 导出便于 Watch、绘图与遥测读取的状态 | [System/debug](User_File/System/debug) |
| 周期与任务 | CMSIS-RTOS V2 任务入口、线程标志和周期回调 | [Task](User_File/Task) |

[main.c](Core/Src/main.c) 完成 MPU、HAL 和外设初始化后调用 `System_Init()`，随后初始化 RTOS、创建任务并启动调度。CAN 发送资源在内核初始化后建立，周期服务与设备计算按职责由任务调度。

当前应用接入仍需按项目配置：[Control_Task.cpp](User_File/Task/Control_Task.cpp) 中的 Gimbal / Balance 初始化与循环调用尚未启用。驱动和算法可作为框架组件使用，具体应用需要补齐资源绑定与调度。

## 接入方式

### 添加设备或通信协议

- 在 `Device/Onboard` 或 `Device/Peripheral` 中放置设备实现，参照同层组件组织配置、状态和接口。
- 复用现有 BSP 管理对象与回调注册；设备层负责协议解析，任务或应用层负责业务处理和控制目标。
- 在显式 `Init()` 中绑定资源，按依赖顺序接入系统初始化；需要 RTOS 对象的部分放在内核初始化之后。
- 按需求接入周期回调或任务，明确 ISR 与任务边界、缓冲区所有权和数据有效期。
- 在根 `CMakeLists.txt` 注册源码与 include 路径，并检查收发结果、超时和实际硬件行为。

### 组合控制与算法

- 在应用层组织目标、反馈和状态，调用设备接口及算法组件；已有数学与时间服务优先复用。
- 按模块约定初始化模型与参数，在确定的采样周期内更新输入、计算输出并提交给设备。
- 控制计算、通信发送和调试输出按实时性分配，接入后观察任务栈水位、周期与执行时间。

### 代码风格

沿用同层模块的 `Struct_`、`Enum_`、`Class_` 命名和文件组织，内部优先使用直接的 C 风格运算。保留已有必要模板与类接口，新模块避免无必要的抽象层、动态分配和重复实现。

硬件操作放在 `Init()` 中，构造函数不访问硬件。C 调用入口通过 `extern "C"` 暴露，C 可见头文件保持兼容。任务与同步接口使用 CMSIS-RTOS V2，中断内调用 RTOS API 前核对优先级和接口限制。

## 工程基础

### DMA 与内存布局

项目使用自有 [h7_memory.ld](User_Config/Linker/h7_memory.ld)，由 [h7_linker.cmake](User_Config/Linker/h7_linker.cmake) 选择：

| 区域 | 地址 | 大小 | 用途 |
| --- | --- | --- | --- |
| `DTCMRAM` | `0x20000000` | 128 KiB | 普通数据、栈、部分 FreeRTOS heap；DMA1/DMA2 不可直接访问 |
| `RAM_DMA` | `0x24000000` | 64 KiB | `.dma_buffer`；MPU 配置为共享、不可缓存 |
| `RAM_D1` | `0x24010000` | 256 KiB | `.ram_d1_data` 与部分 FreeRTOS heap |
| `RAM_D2` / `RAM_D3` | `0x30000000` / `0x38000000` | 32 / 16 KiB | 其余 SRAM，使用时核对对应 DMA 的可达性 |

DMA1/DMA2 缓冲区应放入 `.dma_buffer`，并核对对齐、生命周期和传输长度；BDMA 等控制器需要单独确认内存可达性。`RAM_DMA` 的地址与大小必须与 `main.c` / `.ioc` 中的 MPU 配置一致，链接脚本包含一致性断言。

FreeRTOS 使用 `heap_5`，默认总量 64 KiB，分为 **48 KiB DTCMRAM + 16 KiB RAM_D1**。DTCM 分区由 `H7_FREERTOS_DTCM_HEAP_SIZE` 配置，heap 与 port 补丁位于 [FreeRTOS_Patch](User_Config/FreeRTOS_Patch)。

### CubeMX 与构建边界

- 外设配置入口为 [H7_BSP.ioc](H7_BSP.ioc)，生成代码的用户修改放在 `USER CODE BEGIN/END` 区域。
- 用户源码与 include 路径由根 [CMakeLists.txt](CMakeLists.txt) 显式维护；链接布局、RTOS 补丁维护在 `User_Config/`。
- CubeMX 重生成后重新 configure/build，检查用户集成、链接脚本选择和补丁接入。
- 工程链接 CMSIS-DSP Cortex-M7 浮点库，C++ 工具链禁用异常和 RTTI。

## 构建与调试

### 环境与构建

准备 CMake 3.22 或更高版本、Ninja 和 GNU Arm 工具链，确保 `arm-none-eabi-gcc` / `arm-none-eabi-g++` 等命令可用。在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物为 `build/Debug/H7_BSP.elf`，链接映射为同目录下的 `H7_BSP.map`。Release 使用对应 preset：

```powershell
cmake --preset Release
cmake --build --preset Release
```

[CMakePresets.json](CMakePresets.json) 管理构建配置。Debug 使用 `-Og -g3`，Release 使用 `-Os -g0`。

### 烧录与观察

[VS Code 任务](.vscode/tasks.json) 提供 DAPLink、ST-Link、J-Link 选择与烧录入口；[调试配置](.vscode/launch.json) 和 [Ozone 工程](H7_BSP.jdebug) 提供源码调试入口。使用前按本机安装位置检查工具路径、探针和目标芯片配置。

- Ozone / GDB：观察设备反馈、算法状态、系统调试数据与任务栈水位。
- SystemView / RTT：观察任务调度、中断和运行时信息。
- EricTool：通过 USB / UART 输出数据；`TransportTask` 中保留了 USB 周期输出的使用示例。
- [sysid](sysid/README.md)：系统辨识数据、采集分析脚本与实验报告。

## 文档与参考

- [DJI 电机驱动](User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md) · [达妙电机驱动](User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md) · [更新记录](CHANGELOG.md)。
- [FreeRTOS heap memory management](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/01-Memory-management)。
- [ST AN4891：STM32H7 系统架构与性能](https://www.st.com/resource/en/application_note/an4891-stm32h72x-stm32h73x-and-singlecore-stm32h74x75x-system-architecture-and-performance-stmicroelectronics.pdf)。
- [ST AN4839：STM32F7/H7 一级缓存](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)。

## 致谢

感谢 [Kylin-6](https://github.com/Kylin-6) 在 [PR #4](https://github.com/MermaidFAR/H7_BSP/pull/4) 中贡献达妙电机驱动及初版使用说明，并在 [PR #5](https://github.com/MermaidFAR/H7_BSP/pull/5) 中贡献 DJI 电机原始驱动。

部分驱动参考 [达妙 MC02 BSP](https://github.com/yssickjgd/damiao_mc02_bsp)；UART 实现参考 SCUT-Robotlab / 达妙 `drv_uart` 的组织方式。感谢相关开源项目与原作者。

<details>
<summary>维护架构图</summary>

架构图由 [Archify](https://github.com/tt-a1i/archify) 生成。编辑 [H7_BSP.architecture.json](Tools/Architecture/H7_BSP.architecture.json)，使用 [版本记录](Tools/Architecture/Archify.lock.json) 对应的 Archify 技能目录执行：

```powershell
.\Tools\Architecture\Build.ps1 -ArchifyRoot "<Archify 技能目录>"
```

[Build.ps1](Tools/Architecture/Build.ps1) 更新 `Assets/Architecture/` 下的 SVG 与交互 HTML。

</details>
