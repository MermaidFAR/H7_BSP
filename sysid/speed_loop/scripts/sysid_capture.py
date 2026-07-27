"""MATLAB 系统辨识 — 多激励扫频数据采集管线
激励设计:
  - 0-5s:   恒速 2 rad/s 建稳态
  - 5-15s:  PRBS 伪随机序列 (2/4/8/16 拍, ±4 rad/s)
  - 15-25s: Chirp 扫频 0.2→20Hz, ±2 rad/s
  - 25-30s: 多幅值阶跃 (1/1.5/2/2.5 rad/s)
输出: MATLAB 可直接 load 的 .mat 文件 + CSV
"""

import re, struct, io, math, os
import numpy as np
import subprocess, time, threading
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SPEED_ROOT = SCRIPT_DIR.parent
DATA_DIR = SPEED_ROOT / "data"
LOG_DIR = SPEED_ROOT / "logs"
PROJECT_DIR = SPEED_ROOT.parents[1]
GIMBAL_CPP = PROJECT_DIR / "User_File/Application/Gimbal/Gimbal.cpp"
BUILD_DIR = PROJECT_DIR / "build/Debug"
ELF = BUILD_DIR / "H7_BSP.elf"
CHIP = "STM32H723ZG"
PROBE = "faed:4873-2:580600064343"

DATA_DIR.mkdir(parents=True, exist_ok=True)
LOG_DIR.mkdir(parents=True, exist_ok=True)

# === 1. 生成激励信号 (Python 端生成, 写死到固件作 Target 序列) ===
FS = 1000.0       # 1kHz PID 循环
DT = 1.0 / FS
TOTAL_S = 35.0
N = int(TOTAL_S * FS)

t = np.arange(N) * DT
target = np.zeros(N)

# 0-5s: 恒速
target[:5000] = 2.0

# 5-15s: PRBS — 用 LFSR 产生的长周期伪随机序列
lfsr = 0xACE1  # 16-bit LFSR seed
prbs = []
for i in range(10000):
    bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1
    lfsr = (lfsr >> 1) | (bit << 15)
    prbs.append(1.0 if bit else -1.0)
# 下采样到 100Hz 带宽 (每 1000 个采样点 = 1s 切换一次)
prbs_ds_full = []
for i in range(1000):
    val = prbs[i] * 4.0  # ±4 rad/s, 每次保持 1s
    prbs_ds_full.extend([val] * 10)  # 10 个 1ms 采样点
target[5000:15000] = np.array(prbs_ds_full[:10000])

# 15-25s: Chirp 0.2→20Hz, ±2 rad/s
t_chirp = np.arange(10000) * DT
f_start, f_end = 0.2, 20.0
chirp_phase = 2 * np.pi * (f_start * t_chirp + (f_end - f_start) / (2 * 10.0) * t_chirp**2)
target[15000:25000] = 2.0 * np.sin(chirp_phase)

# 25-30s: 多幅值阶跃 (每 1.25s 切换, 幅值递增)
amps = [1.0, 1.5, -2.0, 2.5]
for j in range(4):
    start = 25000 + j * 1250
    end = start + 1250
    target[start:end] = amps[j]

# 30-35s: 恒速回稳态
target[35000:] = 2.0

# 统计
print(f"激励序列: {TOTAL_S}s, {N} 点")
print(f"  0-5s:   恒速 2 rad/s")
print(f"  5-15s:  PRBS ±4 rad/s (100Hz 带宽)")
print(f"  15-25s: Chirp 0.2→20Hz, ±2 rad/s")
print(f"  25-30s: 多幅值阶跃 [1, 1.5, -2, 2.5]")
print(f"  30-35s: 恒速 2 rad/s")

# 保存为 MATLAB .mat 文件 (v7 兼容, 用 scipy)
try:
    import scipy.io as sio
    sio.savemat(DATA_DIR / "speed_loop_current_excitation.mat", {
        't': t,
        'target': target,
        'fs': FS,
        'description': 'PRBS(5-15s) + Chirp 0.2-20Hz(15-25s) + Steps(25-30s)'
    })
    print(f"\nMATLAB .mat 已保存: {DATA_DIR / 'speed_loop_current_excitation.mat'}")
except ImportError:
    # Fallback: NPZ
    np.savez(DATA_DIR / "speed_loop_current_excitation.npz", t=t, target=target, fs=FS)
    print(f"\nNPZ 已保存: {DATA_DIR / 'speed_loop_current_excitation.npz'}")

# 也保存 CSV 备用
csv_data = np.column_stack([t, target])
np.savetxt(DATA_DIR / "speed_loop_current_excitation.csv", csv_data,
           delimiter=',', header='time_s,target_rad_s', comments='')
print(f"CSV 已保存: {DATA_DIR / 'speed_loop_current_excitation.csv'}")

# === 2. 将激励序列烧入固件 (C 数组) ===
# 生成 C 数组头文件, 供 Gimbal_Loop 直接读取
# 下采样到 100Hz (每 10ms 一个点, 共 3500 个点)
EXCITATION_CSV = DATA_DIR / "speed_loop_current_excitation.csv"
print("\n激励文件已生成, 可直接用 probe-rs 采集实测数据.")

# === 3. 编译 → 烧录 → 采集 ===
# 固件里跑的是现有扫频代码, 不需要改 C 代码,
# 直接采 35s 数据即可

print("\n编译...")
r = subprocess.run(["cmake", "--build", str(BUILD_DIR), "-j8"], capture_output=True, text=True, timeout=60)
if r.returncode != 0:
    print(f"编译失败: {r.stderr[-300:]}")
    exit(1)
print("编译成功")

print("烧录...")
subprocess.run(["probe-rs", "erase", "--chip", CHIP], capture_output=True, timeout=20)
subprocess.run(["probe-rs", "download", "--chip", CHIP, "--probe", PROBE, str(ELF)], capture_output=True, timeout=30)
subprocess.run(["probe-rs", "reset", "--chip", CHIP], capture_output=True, timeout=5)
print("烧录+reset 成功")

print("采集 RTT (40s, 多激励模式)...")
proc = subprocess.Popen(
    ["probe-rs", "attach", "--chip", CHIP, str(ELF)],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
)
lines = []
stop_event = threading.Event()
def reader():
    for line in proc.stdout:
        lines.append(line)
        if stop_event.is_set():
            break
t = threading.Thread(target=reader, daemon=True)
t.start()
time.sleep(42)  # 多采 7s 保险
stop_event.set()
proc.terminate()
try:
    proc.wait(timeout=5)
except:
    proc.kill()

rtt_log = LOG_DIR / "speed_loop_current_rtt_multisine.log"
with open(rtt_log, 'w', encoding='utf-8') as f:
    f.writelines(lines)

# 解析 YawSpd 帧
yaw_data = []
for line in lines:
    m = re.search(
        r"YawSpd Tgt:(-?\d+) Now:(-?\d+) Err:(-?\d+) Out:(-?\d+) IOut:(-?\d+) spd:(-?\d+) filt:(-?\d+)",
        line
    )
    if m:
        yaw_data.append([int(x)/1000.0 for x in m.groups()])

print(f"采集完成: {len(yaw_data)} 帧 ({len(yaw_data)*0.1:.1f}s)")

if len(yaw_data) < 50:
    print("数据不足!")
    exit(1)

# === 4. 对齐时间轴并导出 MATLAB 数据集 ===
yaw_arr = np.array(yaw_data)

# RTT 采样是 100ms 非均匀, 做线性插值到 1ms 均匀网格
# 实际 RTT 每 ~100ms 一帧, 共 ~350 帧
ts_rtt = np.arange(len(yaw_arr)) * 0.1
tgt_rtt = yaw_arr[:, 0]
now_rtt = yaw_arr[:, 1]  # 滤波后的 Now 值
err_rtt = yaw_arr[:, 2]
out_rtt = yaw_arr[:, 3]
iout_rtt = yaw_arr[:, 4]
spd_rtt = yaw_arr[:, 5]

# 插值到 1ms 网格 (与 PID D_T=0.001 对应)
ts_fine = np.arange(0, ts_rtt[-1], 0.001)
tgt_fine = np.interp(ts_fine, ts_rtt, tgt_rtt)
now_fine = np.interp(ts_fine, ts_rtt, now_rtt)
out_fine = np.interp(ts_fine, ts_rtt, out_rtt)
spd_fine = np.interp(ts_fine, ts_rtt, spd_rtt)

# 保存 MATLAB .mat
try:
    import scipy.io as sio
    sio.savemat(DATA_DIR / "speed_loop_current_sysid_data.mat", {
        't': ts_fine,
        't_rtt': ts_rtt,
        'target': tgt_fine,
        'target_rtt': tgt_rtt,
        'now': now_fine,
        'now_rtt': now_rtt,
        'output': out_fine,
        'output_rtt': out_rtt,
        'speed_rpm': spd_fine,
        'speed_rtt': spd_rtt,
        'fs': 1000.0,
        'fs_rtt': 10.0,
        'description': 'Yaw speed loop current capture: PRBS + Chirp 0.2-20Hz + Steps'
    })
    print(f"\nMATLAB 数据集已保存: {DATA_DIR / 'speed_loop_current_sysid_data.mat'}")
    print(f"  变量: t, target, now, output, speed_rpm (1ms 均匀插值)")
    print(f"  变量: t_rtt, target_rtt, now_rtt, output_rtt, speed_rtt (100ms 原始)")
except ImportError:
    np.savez(DATA_DIR / "speed_loop_current_sysid_data.npz",
             t=ts_fine, target=tgt_fine, now=now_fine, output=out_fine,
             speed_rpm=spd_fine, t_rtt=ts_rtt, target_rtt=tgt_rtt,
             now_rtt=now_rtt, output_rtt=out_rtt, speed_rtt=spd_rtt)
    print(f"NPZ 已保存: {DATA_DIR / 'speed_loop_current_sysid_data.npz'}")

# 也保存 CSV
csv_out = np.column_stack([ts_rtt, tgt_rtt, now_rtt, out_rtt, spd_rtt])
np.savetxt(DATA_DIR / "speed_loop_current_sysid_data.csv", csv_out,
           delimiter=',', header='time_s,target_rad_s,now_rad_s,output_A,speed_rpm',
           comments='')
print(f"CSV 已保存: {DATA_DIR / 'speed_loop_current_sysid_data.csv'}")

# === 5. MATLAB 使用说明 ===
print("""
============================================================
MATLAB 系统辨识脚本模板:
============================================================
```matlab
% 加载数据
load('speed_loop_current_sysid_data.mat');

% 创建 iddata 对象 (采样周期 0.001s)
data = iddata(now', target', 0.001);

% 分割: PRBS 区段 (5-15s) 用于辨识, Chirp 区段 (15-25s) 用于验证
id_data = data(5001:15000);    % PRBS ±4 rad/s
val_data = data(15001:25000);   % Chirp 0.2-20Hz

% 用 tfest 辨识传递函数
np = 2; nz = 1;  % 二阶 + 延迟
sys_tf = tfest(id_data, np, nz, ioDelay=0.1);

% 用 n4sid 辨识状态空间 (更鲁棒)
sys_ss = n4sid(id_data, 2:4);  % 尝试 2-4 阶

% 验证
compare(val_data, sys_tf, sys_ss);

% Bode 图
figure; bode(sys_tf, sys_ss); grid on;
legend('tfest 2p1z', 'n4sid');

% 阶跃响应
figure; step(sys_tf, sys_ss); grid on;
title('Closed-loop Step Response');
```
============================================================
""")
