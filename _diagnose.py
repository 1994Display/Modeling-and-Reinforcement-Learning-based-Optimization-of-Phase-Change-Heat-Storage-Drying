
"""[9;1m v9 diagnosis: random PCM init + stronger temp guidance"""

import numpy as np
import sys
sys.path.insert(0, ".")

from my_test import DryingSystemEnv


def run_fixed_T(env, T_pcm_start, action_fn, n_steps=200):
    """Run with fixed initial T_pcm (override reset)"""
    env.reset(seed=42)
    # Manually set initial PCM temp
    T_amb = env.state[1]
    RH_amb = env.state[2]
    T_water = T_pcm_start * 0.5 + T_amb * 0.5
    T_in = T_pcm_start * 0.6 + T_amb * 0.4
    T_out = T_in - 2.0
    init = np.array(
        [T_pcm_start, T_amb, RH_amb, T_in, T_out, RH_amb, T_water],
        dtype=np.float32,
    )
    env.state = init
    env.steps = 0
    total = 0.0
    for _ in range(n_steps):
        a = np.array(action_fn(env.state[0]), dtype=np.float32)
        _, r, d, tr, _ = env.step(a)
        total += r
        if d or tr:
            break
    return total


env = DryingSystemEnv()

print("=" * 60)
print("v9 Strategy Comparison (fixed cold start, 200 steps)")
print("=" * 60)

strategies = {
    "always-fan [1,0,0,0]  ": lambda T: [1.0, 0.0, 0.0, 0.0],
    "smart>35              ": lambda T: [1.0 if T > 35 else 0.0, 0.0, 0.0, 0.0],
    "smart>40              ": lambda T: [1.0 if T > 40 else 0.0, 0.0, 0.0, 0.0],
    "idle   [0,0,0,0]      ": lambda T: [0.0, 0.0, 0.0, 0.0],
    "damper [0,1,0,0]      ": lambda T: [0.0, 1.0, 0.0, 0.0],
    "all-on [1,1,1,0]      ": lambda T: [1.0, 1.0, 1.0, 0.0],
}

for name, fn in strategies.items():
    s = run_fixed_T(env, 25.0, fn, 200)
    print(f"  {name}: {s:+.2f}")

# Verify: smart>35 must beat fan-always
a_fan = run_fixed_T(env, 25.0, strategies["always-fan [1,0,0,0]  "], 200)
a_smart = run_fixed_T(env, 25.0, strategies["smart>35              "], 200)
a_idle = run_fixed_T(env, 25.0, strategies["idle   [0,0,0,0]      "], 200)
a_damper = run_fixed_T(env, 25.0, strategies["damper [0,1,0,0]      "], 200)

print()
for txt, ok in [
    ("smart35 > idle       ", a_smart > a_idle),
    ("smart35 > fan-always ", a_smart > a_fan),
    ("smart35 > damper     ", a_smart > a_damper),
]:
    print(f"  {txt}: {'PASS' if ok else 'FAIL'}")

# Also test from hot start: fan-always should be good
print()
print("=" * 60)
print("Hot start comparison (T=70, 200 steps)")
print("=" * 60)
a_fan_hot = run_fixed_T(env, 70.0, strategies["always-fan [1,0,0,0]  "], 200)
a_smart_hot = run_fixed_T(env, 70.0, strategies["smart>35              "], 200)
a_idle_hot = run_fixed_T(env, 70.0, strategies["idle   [0,0,0,0]      "], 200)
print(f"  fan-always (hot): {a_fan_hot:+.2f}")
print(f"  smart>35   (hot): {a_smart_hot:+.2f}")
print(f"  idle       (hot): {a_idle_hot:+.2f}")
print()
for txt, ok in [
    ("fan > idle (hot)    ", a_fan_hot > a_idle_hot),
]:
    print(f"  {txt}: {'PASS' if ok else 'FAIL'}")

env.close()

# ==== Model temperature sensitivity ====
print()
print("=" * 60)
print("Model Temperature Sensitivity (7 -> 128 -> 128 -> 4)")
print("=" * 60)

import torch
from train_ppo import PolicyNet

p = PolicyNet(5, 5, 128)
try:
    sd = torch.load("policy_best.pt", map_location="cpu", weights_only=True)
    p.load_state_dict(sd)
    p.eval()
except Exception as ex:
    print(f"No model available: {ex}")
    sys.exit(0)

# v34.9: 5维观测 [PCM, 干燥仓温度, 干燥仓湿度, 水箱温度, 环境温度]
sc = np.array([100, 100, 80, 100, 40], dtype=np.float32)
prev = None
has_var = False
temps = [20, 25, 30, 35, 40, 50, 60, 70, 80]

for T in temps:
    T_dryer = T * 0.9  # 干燥仓温度≈进口风温
    raw = np.array([T, T_dryer, 50, 20, 19], dtype=np.float32)
    norm = np.clip(raw / sc, 0, 1)
    with torch.no_grad():
        m, _ = p(torch.FloatTensor(norm.reshape(1, -1)))
        a = m.numpy()[0]
    tag = "  <--" if prev is not None and np.max(np.abs(a - prev)) > 0.02 else ""
    if prev is not None and np.max(np.abs(a - prev)) > 0.02:
        has_var = True
    print(f"  T={T:3d}: fan={a[0]:.3f} damper={a[1]:.3f} pump={a[2]:.3f} disch={a[3]:.3f} heater={'ON' if a[4]>=0.5 else 'OFF'}{tag}")
    prev = a.copy()

print(f"\n  Variation: {'PASS' if has_var else 'FAIL'}")
