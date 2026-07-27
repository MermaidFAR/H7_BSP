# 系统辨识数据清单

## 速度环

| 文件 | 类型 | 用途 | 状态 |
|------|------|------|------|
| `speed_loop/data/speed_loop_legacy_excitation.csv` | CSV | 旧速度环激励，PRBS + chirp + step | legacy |
| `speed_loop/data/speed_loop_legacy_excitation.mat` | MAT | 旧速度环激励 MATLAB 数据 | legacy |
| `speed_loop/data/speed_loop_legacy_sysid_data.csv` | CSV | 旧速度环 RTT 解析数据 | legacy |
| `speed_loop/data/speed_loop_legacy_sysid_data.mat` | MAT | 旧速度环 MATLAB 数据 | legacy |
| `speed_loop/data/ozone_yaw_speed_sampling_260701.csv` | CSV | Ozone 采样，含 yaw angle、target speed、gyro Z | legacy/reference |
| `speed_loop/scripts/sysid_capture.py` | Python | 下一轮当前参数速度环采集脚本 | active template |
| `speed_loop/scripts/identify_speed.py` | Python | 旧速度环一阶加延迟辨识脚本 | active for legacy replay |
| `speed_loop/results/speed_loop_legacy_sysid_results.png` | PNG | 旧速度环模型拟合图 | result |
| `speed_loop/results/speed_loop_legacy_pid_comparison.png` | PNG | 旧速度环 PID 对比 | result |
| `speed_loop/results/speed_loop_legacy_pid_validation.png` | PNG | 旧速度环 PID 验证 | result |

## 位置环

| 文件 | 类型 | 用途 | 状态 |
|------|------|------|------|
| `position_loop/logs/rtt_angle_sysid.log` | RTT log | 外环重复计算 bug 存在时的旧位置环日志 | legacy |
| `position_loop/logs/rtt_angle_sysid_after_single_calc.log` | RTT log | 修复重复计算后，`Kp=7, Ki=1` | baseline |
| `position_loop/logs/rtt_angle_sysid_kp8p5_ki0.log` | RTT log | 修复后 `Kp=8.5, Ki=0` | candidate |
| `position_loop/logs/rtt_angle_sysid_kp10_ki0.log` | RTT log | 修复后 `Kp=10, Ki=0` | final candidate |
| `position_loop/scripts/identify_angle_legacy.py` | Python | 旧位置环一阶模型分析脚本 | legacy replay |
| `position_loop/scripts/h7_yaw_matlab_sysid_opt.m` | MATLAB | 直接 `SpdOut -> yaw angle` 模型和参数搜索 | analysis |
| `position_loop/scripts/h7_yaw_matlab_rate_model_opt.m` | MATLAB | `SpdOut -> yaw rate -> integral` 结构化模型 | preferred analysis |
| `position_loop/results/h7_yaw_pid_param_three_way_compare.json` | JSON | `7/1`、`8.5/0`、`10/0` 三组指标 | final result |
| `position_loop/results/h7_yaw_pid_param_three_way_compare.png` | PNG | 三组位置环参数对比图 | final result |
| `position_loop/results/h7_yaw_position_loop_before_after.png` | PNG | 修复外环重复计算前后对比图 | result |

## 当前生产参数

- 速度环：`Kp=0.15, Ki=0.63, Kd=0, OutMax=1.65A, IOutMax=1.2`
- 位置环：`Kp=10.00, Ki=0.00, Kd=0, OutMax=50rad/s, IOutMax=15`

## 后续数据缺口

- 当前速度环参数下的大幅值速度环数据仍缺失。
- 需要覆盖 `±5 / ±10 / ±20rad/s`。
- 需要更高 RTT 采样率或二进制遥测，以提升速度环辨识可信度。
