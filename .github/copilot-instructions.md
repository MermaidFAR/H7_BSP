
# H7_BSP Copilot Instructions

## 项目概述

STM32H723ZG 嵌入式 BSP（板级支持包）项目，面向机器人应用（RoboMaster）。使用 FreeRTOS + CMSIS-RTOS V2，C/C++ 混合编程（C11/C++，用户代码为 `.cpp`），`arm-none-eabi-gcc` 工具链，CMake + Ninja 构建。

## 架构分层

```
User_File/Device/       → 具体硬件器件（BMI088, Power），Specialized 全局单例
User_File/Middleware/   → 可复用中间件
  ├── BSP/              → 外设驱动抽象（SPI, ADC, CAN[待迁移]），C 风格 struct + 自由函数
  ├── Algorithm/        → 算法库（PID, FSM, Matrix, EKF），纯 C++ Reusable 类
  └── System/           → 系统服务（Timestamp, callback 分发中心）
User_File/Task/         → FreeRTOS 任务实现（规划中）
Core/                   → STM32CubeMX 生成代码（勿手动修改 USER CODE 区域外的内容）
Drivers/                → HAL 驱动库（只读，CubeMX 管理）
```

**关键数据流**: 硬件中断 → `HAL_xxx_Callback()` → BSP 层分发（`tim_callback.cpp`）→ Device 层处理 → Algorithm 计算

## 命名规范（严格遵循）

| 元素 | 规则 | 示例 |
|------|------|------|
| 文件名 | `前缀_功能名.cpp/.h`，BSP 用 `bsp_`，算法用 `alg_`，系统用 `sys_` | `bsp_spi.cpp`, `alg_pid.cpp` |
| 类名 | `Class_` + PascalCase | `Class_BMI088`, `Class_PID` |
| 结构体 | `Struct_` + PascalCase | `Struct_SPI_Manage_Object` |
| 枚举类型 | `Enum_` + PascalCase，值全大写 | `Enum_SPI_Status`, `SPI_STATUS_FREE` |
| 函数参数 | 双下划线前缀 | `__SPI_Manage_Object`, `__Kp` |
| 全局实例 | `BSP_` / `SYS_` + 器件名 | `BSP_BMI088`, `SYS_Timestamp` |
| include guard | `#ifndef __FILE_NAME_H` | `#ifndef __BSP_SPI_H` |

> **注**: BSP 层驱动命名规范正在迁移中，将重新实现。具体命名规则待后续确定。

## 代码模式

### 类设计
- 用 `public` / `protected` 分区（不使用 `private`）
- 构造函数**不做硬件操作**，通过显式 `Init()` 方法绑定外设并初始化
- Getter/Setter 声明为 `inline`，定义在头文件类外
- `Reusable` 类可多次实例化；`Specialized` 类绑定具体硬件，全局单例

### C/C++ 互操作
- 通过 `extern "C"` 函数提供 C 接口（如 `SYS_Timestamp_Init`），供 `main.c` 调用
- 头文件中 C 语言可见部分包裹在 `extern "C" {}` 中

### 回调函数命名
- 定时器: `TIM_Xxxus/ms/s_Calculate_PeriodElapsedCallback()`
- SPI 完成: `SPI_RxCpltCallback()`
- 外部中断: `EXTI_Flag_Callback()`

### 文件分区注释（每个文件必须包含）
```cpp
/* Includes ------------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
```

## 构建与烧录

- **构建**: CMake preset `Debug`/`Release`，生成器为 Ninja，工具链 `cmake/gcc-arm-none-eabi.cmake`
- **烧录方式**:
  - ST-Link/OpenOCD: 任务 `Flash H7_BSP (ST-Link/OpenOCD)`
  - J-Link/Ozone: 任务 `Flash H7_BSP (J-Link/Ozone)`
  - J-Link/CLI: 先运行 `Generate J-Link Flash Script`，再运行 `Flash H7_BSP (J-Link/CLI)`
- **输出**: `build/Debug/H7_BSP.elf`

## 添加新模块的步骤

1. 在 `User_File/` 对应层级创建目录，放置 `.cpp` + `.h` 文件对
2. 在根 `CMakeLists.txt` 的 `target_sources` 和 `target_include_directories` 中注册
3. 若需定时回调，在 `User_File/Middleware/System/callback/tim_callback.cpp` 的对应 TIM 分支中添加调用
4. 若需 SPI/ADC 外设，在 BSP 管理对象数组中注册并绑定回调指针
5. **DMA 缓冲区必须放在 DMA 可访问的内存区域**（见下方内存布局章节）

## STM32H7 内存布局（重要）

```
DTCMRAM  0x20000000  128K  ← 当前 .data/.bss/stack，仅 CPU+MDMA 可访问，DMA1/DMA2 不可达
RAM_D1   0x24000000  320K  ← MPU 已配置为 Non-cacheable，DMA1/DMA2 可访问
RAM_D2   0x30000000   32K  ← DMA1/DMA2 可访问
RAM_D3   0x38000000   16K  ← 仅 BDMA 可访问（SPI6 等 D3 域外设）
```

**⚠ 当前已知问题**: 链接脚本将 `.data`/`.bss` 放在 DTCMRAM，但 SPI/ADC 等使用 DMA1/DMA2 的外设缓冲区（`Struct_SPI_Manage_Object` 的 `Tx_Buffer`/`Rx_Buffer` 等）作为全局变量也位于 DTCMRAM，DMA 无法访问这些地址。需要通过链接脚本增加 DMA 专用段或调整内存分配策略来修复。

## 关键注意事项

- **FreeRTOS port 已打补丁**: `User_Config/FreeRTOS_Patch/port_patched.c` 注入了 SystemView 探针，根 CMakeLists 中用它替换了原始 `port.c`，禁止由 CubeMX 覆写
- **CubeMX 生成文件**（`Core/`, `Drivers/`）只在 `USER CODE BEGIN/END` 块内修改
- **DSP 库**: 链接了 `Middlewares/ST/ARM/DSP/`，可直接使用 CMSIS-DSP 函数
- **SystemView**: 集成 SEGGER SystemView 用于实时任务分析，配置在 `SystemView/` 目录
- **C++ 限制**: 链接器使用 `--specs=nano.specs`，禁用 RTTI（`-fno-rtti`）和异常（`-fno-exceptions`）
- **中断优先级**: 所有外设中断均配置为优先级 5，等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，处于 FreeRTOS 可管理的临界值边界
- **BSP 驱动重构中**: CAN(FDCAN) 驱动尚未迁移，BSP 层（`User_File/Middleware/BSP/`）将重新实现
