"""
生成奖励曲线图：逐步奖励累积、各子项分解、不同策略对比
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import rcParams
import sys
sys.path.insert(0, ".")
from my_test import DryingSystemEnv

rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
rcParams["axes.unicode_minus"] = False

env = DryingSystemEnv()

# ==================== 1. 好策略：高温开风机，低温等待 ====================
def good_policy(state):
    T_pcm = state[0]
    if T_pcm > 40:
        return np.array([0.6, 0.0, 0.0, 0.0], dtype=np.float32)  # 高温全速干燥
    elif T_pcm > 30:
        return np.array([0.3, 0.0, 0.0, 0.0], dtype=np.float32)  # 中温半速
    else:
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)  # 低温等待

# ==================== 2. 差策略：疯狂全开 ====================
def bad_policy(state):
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

# ==================== 3. 中等策略：一直半开 ====================
def mid_policy(state):
    T_pcm = state[0]
    if T_pcm > 35:
        return np.array([0.6, 0.0, 0.0, 0.0], dtype=np.float32)
    else:
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# ==================== 运行单集 ====================
def run_episode(env, policy_fn, seed=42, T_init=55.0):
    env.reset(seed=seed)
    T_amb = env.state[1]
    RH_amb = env.state[2]
    T_water = T_init * 0.5 + T_amb * 0.5
    T_in = T_init * 0.6 + T_amb * 0.4
    T_out = T_in - 2.0
    init = np.array([T_init, T_amb, RH_amb, T_in, T_out, RH_amb, T_water], dtype=np.float32)
    env.state = init
    env.steps = 0

    history = {"reward": [], "cum_reward": [], "humidity": [], "energy": [],
               "safety": [], "discharge": [], "smoothness": [],
               "T_pcm": [], "RH_dryer": [], "fan": []}
    cum = 0
    for step in range(600):
        action = policy_fn(env.state)
        next_obs, reward, terminated, truncated, info = env.step(action)
        history["reward"].append(reward)
        cum += reward
        history["cum_reward"].append(cum)
        history["humidity"].append(info["humidity"])
        history["energy"].append(info["energy"])
        history["safety"].append(info["safety"])
        history["discharge"].append(info["discharge"])
        history["smoothness"].append(info["smoothness"])
        history["T_pcm"].append(info["T_pcm"])
        history["RH_dryer"].append(info["RH_dryer"])
        history["fan"].append(action[0])
        if terminated:
            break
    return history, step + 1

# ==================== 绘图 ====================
fig, axes = plt.subplots(2, 3, figsize=(18, 12))
fig.suptitle("粮食干燥系统 —— 奖励曲线分析 (v33)", fontsize=16, fontweight="bold", y=0.98)

# ---- (a) 累积奖励：三种策略对比 ----
ax = axes[0, 0]
colors = {"good": "#07A0C3", "mid": "#F0A202", "bad": "#E8313F"}
labels = {"good": "好策略(>40°C开风机, <30°C等待)", "mid": "中策略(>35°C开风机)", "bad": "差策略(全程满风机)"}

for name, fn in [("good", good_policy), ("mid", mid_policy), ("bad", bad_policy)]:
    hist, n = run_episode(env, fn, seed=42, T_init=55.0)
    ax.plot(range(n), hist["cum_reward"], color=colors[name], linewidth=2, label=labels[name])
    ax.scatter(n - 1, hist["cum_reward"][-1], color=colors[name], s=60, zorder=5)
ax.set_xlabel("步数"); ax.set_ylabel("累积奖励")
ax.set_title("策略对比 (PCM初始=55°C)")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# ---- (b) 奖励分解：好策略 ----
ax = axes[0, 1]
hist, n = run_episode(env, good_policy, seed=42, T_init=55.0)
ax.fill_between(range(n), np.cumsum(hist["humidity"]), alpha=0.7, color="#07A0C3", label="干燥进度(RH)")
ax.fill_between(range(n), np.cumsum(hist["energy"]), alpha=0.7, color="#E8313F", label="能耗")
ax.fill_between(range(n), np.cumsum(hist["smoothness"]), alpha=0.4, color="#888888", label="平滑")
x_safety = np.where(np.array(hist["safety"]) != 0)[0]
if len(x_safety) > 0:
    ax.scatter(x_safety, [hist["cum_reward"][i] for i in x_safety], color="#F0A202", s=30, label="安全惩罚", zorder=5)
ax.set_xlabel("步数"); ax.set_ylabel("累积贡献")
ax.set_title("好策略奖励分解 (累积)")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# ---- (c) 每步奖励 vs PCM温度关系 ----
ax = axes[0, 2]
hist, n = run_episode(env, good_policy, seed=42, T_init=55.0)
sc = ax.scatter(hist["T_pcm"], hist["reward"], c=hist["RH_dryer"], cmap="RdYlGn_r", s=50, edgecolors="white", linewidth=0.5)
cbar = plt.colorbar(sc, ax=ax, shrink=0.8)
cbar.set_label("干燥室湿度(%)", fontsize=9)
ax.set_xlabel("PCM 温度 (°C)"); ax.set_ylabel("每步奖励")
ax.set_title("每步奖励 vs PCM温度")
ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
ax.axvline(x=40, color="#07A0C3", linestyle="--", alpha=0.5, label="风机开启阈值")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# ---- (d) 训练奖励曲线模拟 ----
ax = axes[1, 0]
np.random.seed(1234)
episodes = 300
avg_rewards = []
running_best = []
best = -999
# 模拟训练曲线: 前期噪音大, 后期收敛
base = np.linspace(0, 1, episodes)
signal = base * 800 + 200  # 从 200 → 1000 的趋势
noise_scale = np.exp(-base * 3) * 600 + 50  # 噪声从 650 → 50
noise = np.random.randn(episodes) * noise_scale
raw = signal + noise
for r in raw:
    avg_rewards.append(r)
    if r > best:
        best = r
    running_best.append(best)

ax.plot(range(episodes), avg_rewards, color="#07A0C3", alpha=0.4, linewidth=0.8, label="每集总奖励")
# 滑动平均
window = 20
smooth = np.convolve(avg_rewards, np.ones(window)/window, mode="valid")
ax.plot(range(window-1, episodes), smooth, color="#07A0C3", linewidth=2.5, label=f"滑动平均(window={window})")
ax.plot(range(episodes), running_best, color="#E8313F", linewidth=1.5, linestyle="--", label="历史最优")
ax.set_xlabel("Episode"); ax.set_ylabel("Episode 总奖励")
ax.set_title("模拟训练过程 (噪声从大到小)")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# ---- (e) 干燥效率 vs PCM温度 ----
ax = axes[1, 1]
T_range = np.linspace(15, 75, 200)
dry_eff = np.clip((T_range - 20) / 40, 0, 1)
# 不同风机下的除湿量
for fan_val, color, label in [(1.0, "#E8313F", "fan=100%"), (0.6, "#F0A202", "fan=60%"), (0.3, "#07A0C3", "fan=30%")]:
    dehum = fan_val * 0.8 * dry_eff * 1.0
    rh_reward = dehum * 50
    ax.plot(T_range, rh_reward, color=color, linewidth=2, label=label)
ax.axvline(x=40, color="gray", linestyle="--", alpha=0.5)
ax.axhline(y=0, color="gray", alpha=0.3)
ax.text(41, 2, "40°C 阈值", fontsize=8, color="gray")
ax.set_xlabel("PCM 温度 (°C)"); ax.set_ylabel("每步 RH 奖励")
ax.set_title("RH 奖励与 PCM 温度的关系 (damper=0)")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

# ---- (f) 不同初始温度对总奖励的影响 ----
ax = axes[1, 2]
T_inits = [30, 40, 50, 60, 70, 80]
names = ["good", "mid", "bad"]
colors2 = {"good": "#07A0C3", "mid": "#F0A202", "bad": "#E8313F"}
fns2 = {"good": good_policy, "mid": mid_policy, "bad": bad_policy}
x = np.arange(len(T_inits))
width = 0.25
for j, name in enumerate(names):
    vals = []
    for T in T_inits:
        h, _ = run_episode(env, fns2[name], seed=42, T_init=T)
        vals.append(h["cum_reward"][-1])
    ax.bar(x + j * width, vals, width, color=colors2[name], alpha=0.85, label=labels[name])

ax.set_xticks(x + width)
ax.set_xticklabels([f"{t}°C" for t in T_inits])
ax.set_xlabel("PCM 初始温度"); ax.set_ylabel("Episode 总奖励")
ax.set_title("不同初始温度下的策略表现")
ax.legend(fontsize=9); ax.grid(True, alpha=0.3, axis="y")

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig("reward_curves.png", dpi=200, bbox_inches="tight")
print("奖励曲线图已保存: reward_curves.png")
env.close()
