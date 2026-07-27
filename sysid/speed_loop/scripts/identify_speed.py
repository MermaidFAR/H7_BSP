"""Yaw 速度环传递函数辨识 — 时域参数化拟合 (最鲁棒的方式)

方法: 直接用 Nelder-Mead 在时域拟合阶跃/PRBS 响应
  1. 从阶跃响应提取一阶+延迟模型
  2. 从 PRBS 响应交叉验证
  3. 从 Chirp 响应最终验证
"""

import numpy as np
import scipy.io as sio
import scipy.signal as signal
import scipy.optimize as opt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SPEED_ROOT = SCRIPT_DIR.parent
DATA_DIR = SPEED_ROOT / "data"
RESULTS_DIR = SPEED_ROOT / "results"

data = sio.loadmat(DATA_DIR / "speed_loop_legacy_sysid_data.mat")

t_rtt = data['t_rtt'][0]
u_rtt = data['target_rtt'][0]
y_rtt = data['now_rtt'][0]
FS, N = 10.0, len(u_rtt)

print(f"Data: {N} frames @ {FS}Hz, {t_rtt[-1]:.1f}s")
print(f"Target range: {np.min(u_rtt):.1f} ~ {np.max(u_rtt):.1f} rad/s")
print(f"Now range:    {np.min(y_rtt):.1f} ~ {np.max(y_rtt):.1f} rad/s")

# === Partition (same as before) ===
id_range = (int(5*FS), int(15*FS))      # PRBS
val_range = (int(15*FS), int(25*FS))     # Chirp
step_range = (int(25*FS), int(30*FS))    # Steps
dc_range = (10, 40)  # 恒速 1-4s, clean DC

u_id, y_id = u_rtt[id_range[0]:id_range[1]], y_rtt[id_range[0]:id_range[1]]
u_val, y_val = u_rtt[val_range[0]:val_range[1]], y_rtt[val_range[0]:val_range[1]]
u_step, y_step = u_rtt[step_range[0]:step_range[1]], y_rtt[step_range[0]:step_range[1]]

# === Step response feature extraction ===
# 找 25-30s 区的每次阶跃
switches = []
for i in range(1, len(u_step)):
    if abs(u_step[i] - u_step[i-1]) > 0.5:
        switches.append(i)

print(f"\nStep region switches: {len(switches)}")
for s in switches[:3]:
    step_size = u_step[s] - u_step[s-1]
    print(f"  frame{s}: {u_step[s-1]:.1f} → {u_step[s]:.1f} (step={step_size:.1f})")

# === Model: continuous-time 1st order + delay via bilinear discretization ===
def sim_tf_1st(K, tau, L, u_in, dt):
    """Simulate K*e^(-Ls)/(tau*s+1) via bilinear discretization"""
    # Discretize 1/(tau*s+1) with bilinear (Tustin)
    a0 = dt / (2*tau)
    a1 = (1 - a0) / (1 + a0)
    b1 = a0 / (1 + a0)

    y = np.zeros(len(u_in))
    x = 0.0
    for i in range(len(u_in)):
        # Bilinear 1st-order state update
        x = a1 * x + b1 * (u_in[i] + u_in[max(0,i-1)])
        y[i] = K * x

    # Apply delay
    L_samps = int(round(L / dt))
    if L_samps > 0:
        y = np.roll(y, L_samps)
        y[:L_samps] = y[L_samps]
    return y

def fit_loss_td(params, u, y_meas, dt):
    """Time-domain NRMSE loss for 1st order + delay"""
    K, tau, L = params
    y_pred = sim_tf_1st(K, tau, L, u, dt)
    err = y_meas - y_pred
    return np.sum(err**2) / (np.var(y_meas) + 1e-6)

# === Fit on PRBS data ===
dt = 1.0 / FS

# Initial guess from step response
# 取最后一个大幅值阶跃 (25s 附近)
last_step = switches[-1] if switches else 20
step_mag = abs(u_step[last_step] - u_step[last_step-1])
# Find 63.2% point
t63_idx = None
t10_idx = None
t90_idx = None
for i in range(last_step, min(last_step+30, len(y_step))):
    resp = abs(y_step[i] - y_step[last_step-1]) / step_mag
    if not t10_idx and resp > 0.1:
        t10_idx = i
    if not t63_idx and resp > 0.632:
        t63_idx = i
    if not t90_idx and resp > 0.9:
        t90_idx = i
        break

tau_est = (t63_idx - last_step) * dt if t63_idx else 0.08
K_est = np.mean(y_id[:20]) / np.mean(u_id[:20]) if abs(np.mean(u_id[:20])) > 0.1 else 0.95
L_est = (t10_idx - last_step) * dt if t10_idx else 0.05

print(f"\nStep-based estimates:")
print(f"  K  = {K_est:.3f}")
print(f"  τ  = {tau_est:.3f}s")
print(f"  L  = {L_est*1000:.0f}ms (10% response)")
if t90_idx:
    print(f"  Tr = {(t90_idx - t10_idx) * dt * 1000:.0f}ms (10-90% rise)")

# Optimize on PRBS data
print(f"\nFitting on PRBS (5-15s, {len(u_id)} points)...")
p0 = [K_est, max(tau_est, 0.02), min(max(L_est, 0.03), 0.12)]
print(f"  Initial: K={p0[0]:.3f}, τ={p0[1]:.3f}s, L={p0[2]*1000:.0f}ms")

# Bounded optimization
bounds = [(0.5, 1.5), (0.02, 0.30), (0.03, 0.15)]
res = opt.minimize(fit_loss_td, p0, args=(u_id, y_id, dt),
                    method='L-BFGS-B', bounds=bounds,
                    options={'maxiter': 5000})
K_fit, tau_fit, L_fit = res.x
fc_fit = 1/(2*np.pi*tau_fit)
print(f"  Fitted: K={K_fit:.3f}, τ={tau_fit:.3f}s (fc={fc_fit:.2f}Hz), L={L_fit*1000:.0f}ms")
print(f"  PRBS loss: {res.fun:.3f}")

# Validate on Chirp
y_pred_val = sim_tf_1st(K_fit, tau_fit, L_fit, u_val, dt)
rmse_val = np.sqrt(np.mean((y_val - y_pred_val)**2))
nrmse_val = rmse_val / (np.max(y_val) - np.min(y_val)) * 100
print(f"  Chirp NRMSE: {nrmse_val:.1f}%")

# Validate on Steps
y_pred_step = sim_tf_1st(K_fit, tau_fit, L_fit, u_step, dt)
rmse_step = np.sqrt(np.mean((y_step - y_pred_step)**2))
nrmse_step = rmse_step / (np.max(y_step) - np.min(y_step)) * 100
print(f"  Steps NRMSE: {nrmse_step:.1f}%")

# === Continuous transfer function ===
tf_str = f"G(s) = {K_fit:.3f} * exp(-{L_fit*1000:.0f}ms·s) / ({tau_fit:.3f}s + 1)"
print(f"\n传递函数 (闭环速度环):")
print(f"  {tf_str}")
print(f"  截止频率 fc = 1/(2πτ) = {fc_fit:.2f} Hz")
print(f"  延迟 L = {L_fit*1000:.0f} ms")

# === PID tuning suggestion ===
print(f"\nPID 优化建议:")
print(f"  注意: 这份 legacy 数据不代表当前固件速度环参数")
print(f"  采集时脚本记录: K_P=0.070, K_I=0.348, K_D=0")
print(f"  当前固件参数: K_P=0.150, K_I=0.630, K_D=0")
print(f"  闭环带宽 ~{fc_fit:.1f}Hz, 延迟 ~{L_fit*1000:.0f}ms")
if L_fit * fc_fit < 0.15:
    print(f"  带宽×延迟 = {L_fit*fc_fit:.3f} < 0.15 → 系统可进一步提速")
    print(f"  但旧数据量纲/工况不足, 不直接给当前固件 PID 建议")
    print(f"  下一步应重新采当前 0.15/0.63 参数下 ±5/±10/±20rad/s 数据")
else:
    print(f"  带宽×延迟 = {L_fit*fc_fit:.3f} → 已接近延迟限制")

# === Plot ===
fig, axes = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle(f'Yaw Speed Loop ID — {tf_str}', fontsize=13, fontweight='bold')

# PRBS fit
ax = axes[0, 0]
t_p = np.arange(len(u_id)) * dt
y_p = sim_tf_1st(K_fit, tau_fit, L_fit, u_id, dt)
ax.plot(t_p, u_id, 'k--', lw=0.6, alpha=0.4, label='Target')
ax.plot(t_p, y_id, '#2196F3', lw=1.0, label='Measured')
ax.plot(t_p, y_p, '#FF5722', lw=1.5, alpha=0.7, label='Model (1st+delay)')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed (rad/s)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
ax.set_title(f'PRBS Fit (5-15s) — K={K_fit:.2f}, τ={tau_fit:.3f}s, L={L_fit*1000:.0f}ms')

# Chirp validation
ax = axes[0, 1]
t_c = np.arange(len(u_val)) * dt
ax.plot(t_c, u_val, 'k--', lw=0.6, alpha=0.4, label='Target')
ax.plot(t_c, y_val, '#2196F3', lw=1.0, label='Measured')
ax.plot(t_c, y_pred_val, '#FF5722', lw=1.5, alpha=0.7, label=f'Model (NRMSE={nrmse_val:.1f}%)')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed (rad/s)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
ax.set_title('Chirp Validation (0.2→20Hz)')

# Step validation
ax = axes[1, 0]
t_s = np.arange(len(u_step)) * dt
ax.plot(t_s, u_step, 'k--', lw=0.6, alpha=0.4, label='Target')
ax.plot(t_s, y_step, '#2196F3', lw=1.0, label='Measured')
ax.plot(t_s, y_pred_step, '#FF5722', lw=1.5, alpha=0.7, label=f'Model (NRMSE={nrmse_step:.1f}%)')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed (rad/s)')
ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
ax.set_title('Step Validation (Multi-amplitude)')

# Frequency response of identified model
ax = axes[1, 1]
freqs = np.logspace(-1.5, 1, 200)
s = 1j * 2 * np.pi * freqs
H = K_fit * np.exp(-L_fit * s) / (tau_fit * s + 1)
ax.semilogx(freqs, 20*np.log10(np.abs(H)), '#2196F3', lw=2, label='Model mag')
ax.axhline(y=-3, color='red', ls='--', lw=0.8, alpha=0.5, label='-3dB')
ax.set_xlabel('Frequency (Hz)'); ax.set_ylabel('Magnitude (dB)')
ax.legend(fontsize=7, loc='lower left'); ax.grid(True, alpha=0.2, which='both')
ax2 = ax.twinx()
ax2.semilogx(freqs, np.angle(H, deg=True), '#FF5722', lw=1.5, alpha=0.8, label='Phase')
ax2.set_ylabel('Phase (deg)')
ax2.legend(fontsize=7, loc='upper right')
ax2.axhline(y=-90, color='gray', ls=':', lw=0.5)
ax.set_title(f'Bode — fc={fc_fit:.2f}Hz, delay={L_fit*1000:.0f}ms')

plt.tight_layout()
figpath = RESULTS_DIR / "speed_loop_legacy_sysid_results.png"
plt.savefig(figpath, dpi=150, bbox_inches='tight')
print(f"\nSaved: {figpath}")
plt.close()
