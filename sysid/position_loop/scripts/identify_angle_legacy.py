"""Yaw 位置环系统辨识 — 时域拟合 + 频响估计

数据来源: probe-rs RTT Channel 0, 每 100ms 一帧
激励: PRBS ±1 rad (5-15s) + Chirp 0.1→5Hz (15-25s) + 阶跃 (25-30s)
"""

import numpy as np
import re, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.signal import cont2discrete, lsim, lti
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
POSITION_ROOT = SCRIPT_DIR.parent
LOG = POSITION_ROOT / "logs" / "rtt_angle_sysid.log"
RESULTS_DIR = POSITION_ROOT / "results"

# === 解析 ===
data = []
with open(LOG) as f:
    for line in f:
        m = re.search(r"Angle Tgt:(-?\d+) Now:(-?\d+) Err:(-?\d+) SpdOut:(-?\d+) SpdNow:(-?\d+) YawOut:(-?\d+)", line)
        if m:
            data.append([int(x)/1000.0 for x in m.groups()])

N = len(data)
FS = 10.0; dt = 1/FS
t_arr = np.arange(N) * dt
tgt  = np.array([d[0] for d in data])  # Target angle (rad)
now  = np.array([d[1] for d in data])  # Actual angle (rad)
spd_out = np.array([d[3] for d in data])  # Speed loop target (rad/s)
spd_now = np.array([d[4] for d in data])  # Speed now (rad/s)

print(f"Data: {N} frames @ {FS}Hz, {t_arr[-1]:.1f}s")
print(f"Angle range: {np.min(tgt):.2f} ~ {np.max(tgt):.2f} rad")
print(f"Now range:   {np.min(now):.2f} ~ {np.max(now):.2f} rad")

# === 1. 阶跃响应指标 ===
print("\n=== 阶跃响应时域指标 ===")
steps = [i for i in range(1, N) if abs(tgt[i] - tgt[i-1]) > 0.2]

# 取最后一个大幅值阶跃
if steps:
    s = steps[-2] if len(steps) > 1 else steps[0]
    step_mag = abs(tgt[s] - tgt[s-1])
    print(f"  参考阶跃 (帧{s}): {tgt[s-1]:.2f} → {tgt[s]:.2f} (mag={step_mag:.2f})")

    # 上升时间
    thr10, thr90 = None, None
    for offset in range(1, 30):
        if s+offset >= N: break
        resp = abs(now[s+offset] - tgt[s-1]) / step_mag
        if not thr10 and resp > 0.1: thr10 = offset
        if not thr90 and resp > 0.9: thr90 = offset
    if thr10 and thr90:
        print(f"  10%→90% 上升时间: {(thr90-thr10)*100}ms")

    # 超调
    if tgt[s] > tgt[s-1]:
        peak = np.max(now[s:s+20]) if s+20 < N else now[s]
    else:
        peak = np.min(now[s:s+20]) if s+20 < N else now[s]
    overshoot = (abs(peak - now[s+20]) / step_mag * 100) if s+20 < N else 0
    print(f"  超调: ~{abs(peak - tgt[s])/step_mag*100:.0f}%")

    # 5% 误差带稳态时间
    for offset in range(1, min(30, N-s)):
        if s+offset < N and abs(now[s+offset] - tgt[s]) / max(abs(tgt[s]), 0.01) < 0.05:
            print(f"  5% 稳态时间: {offset*100}ms")
            break

# === 2. 频响估计 (PRBS 区) ===
print("\n=== 频响估计 ===")
id_start, id_end = 50, 150  # 5-15s PRBS
u_id, y_id = tgt[id_start:id_end] - np.mean(tgt[id_start:id_end]), now[id_start:id_end] - np.mean(now[id_start:id_end])

from scipy.signal import csd, coherence
nperseg = min(50, len(u_id)//2)
f_c, Txy = csd(y_id, u_id, fs=FS, nperseg=nperseg, noverlap=nperseg//2)
f_p, Pxx = csd(u_id, u_id, fs=FS, nperseg=nperseg, noverlap=nperseg//2)
H = Txy / (Pxx + 1e-10)
mag_db = 20 * np.log10(np.abs(H))
phase_deg = np.angle(H, deg=True)
f_c, coh = coherence(y_id, u_id, fs=FS, nperseg=nperseg, noverlap=nperseg//2)

valid = (coh > 0.6) & (f_c > 0.02)
fc_v, mv_v, pv_v = f_c[valid], mag_db[valid], phase_deg[valid]
print(f"  Valid freq points (coh>0.6): {len(fc_v)}, range {fc_v[0]:.3f}-{fc_v[-1]:.3f}Hz")
dc_gain = np.mean(np.abs(H[valid][:3]))
dc_db = 20 * math.log10(dc_gain)
print(f"  DC gain: {dc_gain:.3f} ({dc_db:.1f}dB)")

# -3dB 带宽
idx_3db = np.argmin(np.abs(mag_db[valid] - (mag_db[valid][0] - 3)))
bw_3db = f_c[valid][idx_3db] if idx_3db < len(fc_v) else 0.5
print(f"  -3dB bandwidth: {bw_3db:.2f}Hz")

# === 3. 传递函数拟合 ===
print("\n=== 传递函数拟合 ===")

def sim_tf(K, tau, L, u_in, dt):
    """仿真 K*exp(-Ls)/(tau*s+1) via bilinear discretization"""
    a0 = dt / (2*tau)
    a1 = (1 - a0) / (1 + a0)
    b1 = a0 / (1 + a0)
    y = np.zeros(len(u_in))
    x = 0.0
    for i in range(len(u_in)):
        x = a1*x + b1*(u_in[i] + (u_in[i-1] if i>0 else u_in[i]))
        y[i] = K * x
    L_samps = int(round(L / dt))
    if L_samps > 0:
        y = np.roll(y, L_samps)
        y[:L_samps] = y[L_samps]
    return y

def loss(params, u, y, dt):
    K, tau, L = params
    y_pred = sim_tf(K, tau, L, u, dt)
    return np.sum((y - y_pred)**2) / (np.var(y) + 1e-6)

# 初始猜测
K0 = dc_gain
tau0 = 1/(2*np.pi*max(bw_3db, 0.1))
L0 = 0.05  # 位置环延迟通常比速度环大
print(f"  Initial: K={K0:.3f}, tau={tau0:.3f}s, L={L0*1000:.0f}ms")

bounds = [(0.5, 2.0), (0.02, 0.5), (0.02, 0.3)]
res = minimize(loss, [K0, tau0, L0], args=(u_id, y_id, dt),
               method='L-BFGS-B', bounds=bounds, options={'maxiter': 5000})
K, tau, L = res.x
fc = 1/(2*np.pi*tau)
print(f"  Fitted: K={K:.3f}, tau={tau:.3f}s (fc={fc:.2f}Hz), L={L*1000:.0f}ms")

# 验证
y_pred_val = sim_tf(K, tau, L, tgt[id_end:id_end+100] - np.mean(tgt[id_end:id_end+100]), dt)
y_pred_val += np.mean(tgt[id_end:id_end+100])
y_val = now[id_end:id_end+100]
nrmse_val = np.sqrt(np.mean((y_val - y_pred_val)**2)) / max(np.max(y_val)-np.min(y_val), 0.01) * 100
print(f"  Chirp validation NRMSE: {nrmse_val:.1f}%")

# 阶跃验证
step_range = now[250:300]
step_tgt = tgt[250:300]
y_pred_step = sim_tf(K, tau, L, step_tgt - np.mean(step_tgt), dt) + np.mean(step_tgt)
nrmse_step = np.sqrt(np.mean((step_range - y_pred_step)**2)) / max(np.max(step_range)-np.min(step_range), 0.01) * 100
print(f"  Step validation NRMSE: {nrmse_step:.1f}%")

tf_str = f"G_pos(s) = {K:.3f} * exp(-{L*1000:.0f}ms·s) / ({tau:.3f}s + 1)"
print(f"\n传递函数: {tf_str}")

# === 4. PID 调参建议 ===
print("\n=== PID 建议 ===")
print(f"  位置环内环是速度环 (K_P=0.15, K_I=0.63)")
print(f"  当前角度 PID: K_P=0.0448, K_I=1.011")
print(f"  截止频率 fc={fc:.2f}Hz, 延迟 L={L*1000:.0f}ms")

# IMC 调参法则参考
lambda_g = 3 * L  # 期望闭环时间常数
K_p_imc = tau / (K * (lambda_g + L))
K_i_imc = K_p_imc / (tau + L)
print(f"  IMC 建议: K_P≈{K_p_imc:.3f}, K_I≈{K_i_imc:.3f}")

# === 5. Plot ===
fig, axes = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle(f'Yaw Position Loop ID — {tf_str}', fontsize=13)

# Full overview
ax = axes[0, 0]
ax.plot(t_arr, tgt, 'k--', lw=0.6, alpha=0.4, label='Target Angle')
ax.plot(t_arr, now, '#2196F3', lw=1.0, label='Actual Angle')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (rad)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
ax.set_title('Full Excitation Response')

# PRBS fit
ax = axes[0, 1]
t_p = np.arange(len(u_id)) * dt
y_pred_id = sim_tf(K, tau, L, u_id, dt)
ax.plot(t_p, y_id + np.mean(tgt[id_start:id_end]), '#2196F3', lw=1.0, label='Measured')
ax.plot(t_p, y_pred_id + np.mean(tgt[id_start:id_end]), '#FF5722', lw=1.2, label=f'Model (NRMSE={nrmse_val:.1f}%)')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (rad)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
ax.set_title('PRBS Fit (5-15s)')

# Bode
ax = axes[1, 0]
freqs = np.logspace(-2, 1, 200)
s = 1j*2*np.pi*freqs
Hb = K * np.exp(-L*s) / (tau*s + 1)
ax.semilogx(f_c, mag_db, '.', color='#BBDEFB', ms=2, alpha=0.5)
ax.semilogx(fc_v, mv_v, '.', color='#2196F3', ms=4, label=f'Valid (n={len(fc_v)})')
ax.semilogx(freqs, 20*np.log10(np.abs(Hb)), '#FF5722', lw=2, label='Model')
ax.axhline(y=-3, color='red', ls='--', lw=0.8, alpha=0.4)
ax.set_xlabel('Freq (Hz)'); ax.set_ylabel('Mag (dB)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2, which='both')
ax2 = ax.twinx()
ax2.semilogx(f_c, phase_deg, '.', color='#FFCCBC', ms=2, alpha=0.5)
ax2.semilogx(freqs, np.angle(Hb, deg=True), '#FF9800', lw=1.5, alpha=0.7, label='Phase')
ax2.set_ylabel('Phase (deg)')
ax2.legend(fontsize=7, loc='lower left')
ax.set_title(f'Bode — fc={fc:.2f}Hz, L={L*1000:.0f}ms')

# Step validation
ax = axes[1, 1]
t_s = np.arange(len(step_range)) * dt
ax.plot(t_s, step_tgt, 'k--', lw=0.7, alpha=0.4, label='Target')
ax.plot(t_s, step_range, '#2196F3', lw=1.0, label='Measured')
ax.plot(t_s, y_pred_step, '#FF5722', lw=1.2, label=f'Model (NRMSE={nrmse_step:.1f}%)')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (rad)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
ax.set_title('Step Validation (25-30s)')

plt.tight_layout()
figpath = RESULTS_DIR / "angle_loop_legacy_sysid_results.png"
plt.savefig(figpath, dpi=150, bbox_inches='tight')
print(f"\nSaved: {figpath}")
plt.close()
