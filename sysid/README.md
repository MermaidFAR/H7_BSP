# H7_BSP 云台系统辨识数据索引

本目录集中保存 yaw 云台双环系统辨识数据、脚本和结果。原则是保留原始数据、不覆盖旧实验、按控制环路归档。

## 目录结构

- `speed_loop/`
  - `data/`：速度环激励、RTT 解析数据、Ozone 采样数据
  - `scripts/`：速度环采集与辨识脚本
  - `results/`：速度环辨识和 PID 对比结果图
  - `logs/`：后续当前参数速度环 RTT 原始日志输出目录
- `position_loop/`
  - `logs/`：位置环 RTT 原始日志，含多组参数实测
  - `scripts/`：位置环 Python/MATLAB 辨识与参数搜索脚本
  - `results/`：位置环模型、参数对比图、JSON/MAT 结果
- `reports/`
  - 系统辨识经验、实验结论和后续计划文档

## 关键数据

- 速度环旧数据：`speed_loop/data/speed_loop_legacy_sysid_data.csv`
- 速度环旧 MATLAB 数据：`speed_loop/data/speed_loop_legacy_sysid_data.mat`
- 速度环旧激励：`speed_loop/data/speed_loop_legacy_excitation.csv`
- Ozone 速度目标/IMU 数据：`speed_loop/data/ozone_yaw_speed_sampling_260701.csv`
- 位置环旧双算日志：`position_loop/logs/rtt_angle_sysid.log`
- 位置环修复双算后日志：`position_loop/logs/rtt_angle_sysid_after_single_calc.log`
- 位置环 `Kp=8.5, Ki=0` 日志：`position_loop/logs/rtt_angle_sysid_kp8p5_ki0.log`
- 位置环 `Kp=10, Ki=0` 日志：`position_loop/logs/rtt_angle_sysid_kp10_ki0.log`

## 当前可信结论

- 速度环旧模型：`G_spd(s) = 1.051 * exp(-30ms*s) / (0.108s + 1)`，闭环带宽约 `1.47Hz`。
- 速度环旧数据不代表当前固件参数。旧脚本/数据记录的参数语境与当前 `Kp=0.15, Ki=0.63` 不完全一致，只能说明速度环仍可能有提速空间。
- 位置环曾存在外环重复计算 bug：`Yaw_Angle_PID` 在一个 1kHz 周期内被计算两次。已修复。
- 当前生产参数：速度环 `Kp=0.15, Ki=0.63`；位置环 `Kp=10.00, Ki=0.00`。
- 位置环 `Kp=10, Ki=0` 三组实测中最好：RMSE 约 `0.374rad`、MAE 约 `0.149rad`、90% 到达中位约 `0.2s`、P90 超调约 `2.35%`。

## 使用建议

- 复跑旧速度环辨识：运行 `python sysid/speed_loop/scripts/identify_speed.py`。
- 后续继续优化速度环时，不要用 legacy 数据直接改 PID。应重新采当前固件参数下的 `±5 / ±10 / ±20rad/s` 速度环数据。
- 复跑位置环 MATLAB 参数分析：运行 `h7_yaw_matlab_rate_model_opt.m` 或 `h7_yaw_matlab_sysid_opt.m`。
- 所有新采集数据请使用 `current` 或日期后缀命名，避免覆盖 `legacy` 文件。
