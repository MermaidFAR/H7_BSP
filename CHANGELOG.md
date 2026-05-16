# 更新日志

本文件记录 H7_BSP 当前阶段的工程进展、已验证结果和仍待完成事项。

格式遵循“日期 + 分类”的方式维护。当前项目尚未形成正式版本号，因此先使用日期条目。

## 2026-05-16

### 新增

- 新增项目 README，集中说明工程定位、目录结构、启动链路、已实现功能、未实现功能、构建方式和后续建议。
- 新增本更新日志，用于持续记录 BSP 开发进度。
- 新增 `User_File/Task/TransportTask.cpp` 的实际任务入口，原空文件已补齐为可构建骨架。
- 在 `TransportTask` 中接入 `MX_USB_DEVICE_Init()`，USB Device 初始化从 CubeMX 弱任务实现迁移到用户任务实现中。
- 在 `User_File/Task/user_task.h` 中声明 `Ins_Task()` 与 `Transport_Task()`，使用户任务入口与 FreeRTOS 创建逻辑保持一致。
- 在根 `CMakeLists.txt` 中注册 `TransportTask.cpp`，任务文件已纳入构建。

### 已完成

- 完成一轮最小 C/C++ 边界收口：`System_Init()`、HAL GPIO/TIM/SPI 回调、`SPI2_Callback()`、任务入口均显式使用 C ABI。
- 净化 `Init.h`、`callback.h`、`user_task.h` 的 C 可见区域，C++ 依赖移入实现文件。
- README 增补 C/C++ 协作设计、Ozone 调试镜像设计、CMake Tools 解析提示说明。
- README 增补 FreeRTOS 后续任务拆分原则，明确 `InsTask` 只作为姿态数据生产者，云台/底盘/发射通过 `INS_State` 快照和事件通知解耦。
- README 增补通信任务分层原则，区分 CAN 控制链路、USB/串口遥测、遥控/视觉/裁判系统解析与 `TransportTask` 职责边界。
- 将 `cmake/stm32cubemx/CMakeLists.txt` 从混合换行规范化为 CRLF，并清理 `MX_LINK_LIBS` 段尾随空白，以规避 VS Code/CMake Tools 自动分析器误报。
- 验证新增用户源文件应手动注册到根 `CMakeLists.txt` 的 `target_sources`；临时测试源文件已从构建列表清理。
- 调整 `.vscode/settings.json`：保留 `cube-cmake`，显式绑定 Debug configure/build preset，移除会触发 unused warning 的 `-DCMAKE_COMMAND=cube-cmake`。
- 修复 SPI5 全双工完成回调的 Tx 长度参数，避免把 `Rx_Buffer_Length` 同时传给 Tx/Rx 长度。
- 清理 `bsp_spi.cpp` 中的尾随空白，使相关文件不再触发 `git diff --check` 的该项报告。
- 将全局 `init_finished` 改为 `volatile bool`，降低初始化完成标志在中断上下文读取时的优化风险。
- 确认 `CMakePresets.json` 提供 `Debug` 与 `Release` preset，使用 Ninja 和 `cmake/gcc-arm-none-eabi.cmake` 工具链。
- 确认根 `CMakeLists.txt` 已接入用户算法、BSP、System、Device、Task 和 SystemView 源码。
- 确认 FreeRTOS 原始 `port.c` 已被过滤，改用 `User_Config/FreeRTOS_Patch/port_patched.c`。
- 确认 `System_Init()` 已接入 SystemView、时间戳、EXTI 优先级、SPI2、SPI6、TIM5 和 BMI088 初始化。
- 确认 `callback.cpp` 已接管 HAL EXTI、TIM、SPI2 回调分发。
- 确认 `Core/Src/main.c` 中的 `HAL_TIM_PeriodElapsedCallback()` 是弱定义，用户层 `callback.cpp` 可以覆盖。
- 确认 `InsTask` 由任务通知驱动，收到 BMI088 陀螺仪数据完成通知后执行 `BSP_BMI088.EKF_Calculate()`。
- 确认 BMI088 已建立 EXTI 数据就绪、SPI DMA 读取、SPI 完成回调、任务通知、EKF 解算的主链路。
- 确认 SPI/ADC 管理对象已放入 `.dma_buffer`，链接脚本将该段放入 RAM_D1。
- 确认 RAM_D1 MPU 配置为 non-cacheable，降低 H7 D-Cache 与 DMA 一致性风险。
- 确认 `configCHECK_FOR_STACK_OVERFLOW` 已启用，`vApplicationStackOverflowHook()` 已实现。
- 确认主要源码目录中已无空文件。

### 构建验证

- 构建命令：`cmake --build build/Debug`
- 构建结果：成功
- 输出文件：`build/Debug/H7_BSP.elf`
- ELF 大小：3254876 B
- 最近构建时间：2026-05-16 00:52:35

内存占用摘要：

| 区域 | 使用量 | 总量 | 使用率 |
| --- | ---: | ---: | ---: |
| DTCMRAM | 103968 B | 128 KB | 79.32% |
| RAM_D1 | 6336 B | 320 KB | 1.93% |
| RAM_D2 | 0 B | 32 KB | 0.00% |
| RAM_D3 | 0 B | 16 KB | 0.00% |
| ITCMRAM | 0 B | 64 KB | 0.00% |
| FLASH | 150608 B | 1024 KB | 14.36% |

### 仍未完成

- `TransportTask` 仍只有 USB Device 初始化和 `osDelay(1)` 循环，尚无实际通信协议、收发队列、遥测输出或命令解析。
- CAN/FDCAN 用户层 BSP 尚未迁移，当前只有 CubeMX 外设初始化。
- Power/ADC 模块已有代码，但 `System_Init()` 尚未调用 `ADC_Init()` 与 `BSP_Power.Init()`。
- BMI088 加热器默认关闭，温度读取与 128 ms PID 温控周期尚未接入当前调度链路。
- `Task1s_Callback()`、TIM7 1 ms 分支等周期任务仍为空或注释状态。
- ISR 与任务之间共享的 BMI088 内部 `Init_Finished_Flag` 及 ready/update/transfering 标志仍是普通 `bool`，后续需明确并发语义。
- EKF 矩阵表达式在 Debug / `-O0` 下可能有较大栈压力，需要继续观察 `InsTask` 栈水位。
- 工作区存在大量未提交/未跟踪改动，需要后续按主题整理提交。

### 风险与注意

- DTCMRAM 当前使用率约 79.32%，FreeRTOS heap、`.data`、`.bss`、栈都在该区域，需要关注后续任务和全局对象增长。
- SPI6 当前走阻塞传输，这是因为 BDMA 可访问内存区域限制尚未单独处理。
- 当前 BMI088 读取策略已经避免在 SPI 完成回调中继续发起新的 DMA 传输，后续修改时应保持这一原则，避免 DMA-in-DMA 竞态复发。
- 若后续启用 BMI088 加热器，需要先确保 ADC 电压采样和电源组件初始化有效，否则加热功率计算没有可靠输入。

## 2026-04-10 之前

### 已有基础

- CubeMX 工程已生成 STM32H723ZG 外设初始化代码。
- 工程已具备 CMake + Ninja 构建结构。
- `User_File/Middleware/Algorithm/` 已包含基础数学、矩阵、四元数、PID、FSM、队列、斜坡、Kalman、EKF、频率滤波等算法组件。
- `User_File/Middleware/BSP/` 已有 SPI 与 ADC 抽象雏形。
- `User_File/Device/Components/` 已有 BMI088 与 Power 组件。
- SystemView、SEGGER RTT、USB Device、FreeRTOS、CMSIS-DSP 等依赖已放入工程。
