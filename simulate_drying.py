"""
干燥系统批次效果仿真器
用法:
  python simulate_drying.py                  # 使用已训练的 AI 策略
  python simulate_drying.py --manual         # 使用手动阈值策略
  python simulate_drying.py --compare        # AI vs 手动策略对比
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse
from my_test import DryingSystemEnv

# ============ 控制策略定义 ============

def ai_policy(state, policy_net, scale):
    """加载训练好的 AI 模型做推理（需要 PyTorch）
    v34.9: 输入为完整 7 维 state，转换为 5 维观测再喂网络
      obs = [PCM温度, 干燥仓温度(T_in), 干燥仓湿度, 水箱温度, 环境温度常量25]
    """
    import torch
    T_pcm, T_amb, RH_amb, T_in, T_out, RH_dryer, T_water = state
    obs = np.array([T_pcm, T_in, RH_dryer, T_water, 25.0], dtype=np.float32)
    norm = np.clip(obs / scale, 0, 1)
    with torch.no_grad():
        m, _ = policy_net(torch.FloatTensor(norm.reshape(1, -1)))
    return m.numpy()[0]

def manual_smart(state):
    """智能手动策略：高温开风机干燥，低温用加热片补热"""
    T_pcm = state[0]   # PCM温度
    RH = state[5]       # 干燥室湿度
    T_water = state[6]  # 水温

    if T_pcm > 45:
        fan = 0.7       # 高温全速干燥
        damper = 0.1    # 90%送干燥仓
        heater = 0.0    # 高温不需要加热
    elif T_pcm > 35:
        fan = 0.4
        damper = 0.2
        heater = 0.0
    elif T_pcm > 25:
        fan = 0.2       # 中低温：开加热片辅助升温
        damper = 0.0
        heater = 1.0
    else:
        fan = 0.0       # 极低温：仅靠加热片补热
        damper = 0.0
        heater = 1.0

    # 干燥完成时开排粮
    discharge = 1.0 if RH < 15 else 0.0

    # 水箱满了就多分热去干燥
    if T_water > 80:
        damper = 0.0

    return np.array([fan, damper, 0.0, discharge, heater], dtype=np.float32)

def manual_always_fan(state):
    """全开风机策略（低效对照）"""
    return np.array([0.8, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

def idle_policy(state):
    """什么也不做（基线对照）"""
    return np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)


# ============ 仿真运行 ============

def run_batch(env, policy_fn, T_init=60.0, seed=42, max_steps=600):
    """运行一个完整的干燥批次，返回详细日志"""
    env.reset(seed=seed)

    # 手动设置初始状态
    s = env.state
    T_amb, RH_amb = s[1], s[2]
    init = np.array([
        T_init, T_amb, RH_amb,
        T_init * 0.6 + T_amb * 0.4,  # T_in
        T_init * 0.6 + T_amb * 0.4 - 2,  # T_out
        RH_amb,
        T_init * 0.5 + T_amb * 0.5,  # T_water
    ], dtype=np.float32)
    env.state = init
    env.steps = 0

    log = {
        "step": [], "T_pcm": [], "T_amb": [], "RH_dryer": [],
        "T_water": [], "T_in": [], "T_out": [],
        "fan": [], "damper": [], "pump": [], "discharge": [], "heater": [],
        "reward": [], "cum_reward": [],
    }
    cum = 0.0
    dry_time = None
    discharge_count = 0

    for i in range(max_steps):
        action = policy_fn(env.state)
        fan = float(np.clip(action[0], 0, 1))
        damper = float(np.clip(action[1], 0, 1))
        pump = float(np.clip(action[2], 0, 1))
        discharge = 1 if float(np.clip(action[3], 0, 1)) >= 0.5 else 0
        heater = 1 if float(np.clip(action[4], 0, 1)) >= 0.5 else 0

        obs, reward, terminated, truncated, info = env.step(action)
        cum += reward

        log["step"].append(i)
        log["T_pcm"].append(info["T_pcm"])
        log["RH_dryer"].append(info["RH_dryer"])
        log["T_water"].append(info["T_water"])
        log["T_in"].append(info["T_in"])
        log["T_out"].append(info["T_out"])
        log["fan"].append(fan)
        log["damper"].append(damper)
        log["pump"].append(pump)
        log["discharge"].append(discharge)
        log["heater"].append(heater)
        log["reward"].append(reward)
        log["cum_reward"].append(cum)

        if discharge:
            discharge_count += 1

        if terminated and dry_time is None:
            dry_time = i + 1

        if truncated or terminated:
            break

    steps = i + 1

    # 计算干燥指标
    rh_initial = log["RH_dryer"][0]
    rh_final = log["RH_dryer"][-1]
    total_energy = sum(log["fan"])  # 风机总使用量（代理能耗）
    drying_rate = (rh_initial - rh_final) / max(steps, 1) * 100  # 平均每百步降湿

    return {
        "steps": steps,
        "dry_time": dry_time,
        "rh_initial": rh_initial,
        "rh_final": rh_final,
        "discharges": discharge_count,
        "total_reward": cum,
        "total_energy": total_energy,
        "drying_rate": drying_rate,
        "T_pcm_final": log["T_pcm"][-1],
        "T_water_final": log["T_water"][-1],
        "log": log,
    }


# ============ 可视化 ============

def plot_batch(results, title, filename="drying_simulation.png"):
    """绘制干燥过程曲线"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(title, fontsize=14, fontweight="bold")

    colors = {"AI控制": "#07A0C3", "手动智能": "#07A0C3",
              "全开风机": "#E8313F", "空闲基线": "#888888"}
    colors_sim = {"AI": "#07A0C3", "手动": "#2ECC40",
                  "全开": "#E8313F", "空闲": "#888888"}

    for name, res in results.items():
        log = res["log"]
        steps = res["steps"]
        c = colors.get(name, colors_sim.get(name, "#333333"))
        x = log["step"]

        # (a) 干燥室湿度
        axes[0, 0].plot(x, log["RH_dryer"], color=c, linewidth=2, label=name)
        axes[0, 0].axhline(y=15, color="gray", linestyle="--", alpha=0.5)
        axes[0, 0].text(5, 17, "排粮阈值 15%", fontsize=8, color="gray")

        # (b) PCM 温度
        axes[0, 1].plot(x, log["T_pcm"], color=c, linewidth=2)
        axes[0, 1].axhspan(37, 44, color="orange", alpha=0.15)
        axes[0, 1].text(5, 41, "相变区间", fontsize=8, color="orange")

        # (c) 水箱温度
        axes[0, 2].plot(x, log["T_water"], color=c, linewidth=2)

        # (d) 风机转速 + 加热片开关
        axes[1, 0].step(x, log["fan"], color=c, linewidth=1.5, where="post")
        axes[1, 0].fill_between(x, 1.0, 0.98,
                                where=[h == 1 for h in log["heater"]],
                                color="#E8313F", alpha=0.6, step="post",
                                label=name + " [加热开]")

        # (e) 累积奖励
        axes[1, 1].plot(x, log["cum_reward"], color=c, linewidth=2)

        # (f) 进出口风温
        axes[1, 2].plot(x, log["T_in"], color=c, linewidth=1.5, linestyle="-")
        axes[1, 2].plot(x, log["T_out"], color=c, linewidth=1, linestyle="--", alpha=0.6)

    # 标签
    axes[0, 0].set_title("干燥室湿度"); axes[0, 0].set_xlabel("步数"); axes[0, 0].set_ylabel("RH (%)")
    axes[0, 1].set_title("PCM 储热温度"); axes[0, 1].set_xlabel("步数"); axes[0, 1].set_ylabel("°C")
    axes[0, 2].set_title("水箱温度"); axes[0, 2].set_xlabel("步数"); axes[0, 2].set_ylabel("°C")
    axes[1, 0].set_title("风机转速 / 加热片"); axes[1, 0].set_xlabel("步数"); axes[1, 0].set_ylabel("fan (红色条=加热开)")
    axes[1, 1].set_title("累积奖励"); axes[1, 1].set_xlabel("步数"); axes[1, 1].set_ylabel("奖励")
    axes[1, 2].set_title("进出口风温"); axes[1, 2].set_xlabel("步数"); axes[1, 2].set_ylabel("°C")

    for ax in axes.flat:
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=200, bbox_inches="tight")
    print(f"\n图表已保存: {filename}")


# ============ 主程序 ============

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="干燥系统批次仿真")
    parser.add_argument("--manual", action="store_true", help="使用手动阈值策略（默认尝试加载AI模型）")
    parser.add_argument("--compare", action="store_true", help="同时运行多种策略并对比")
    parser.add_argument("--T-init", type=float, default=60.0, help="PCM初始温度 (℃)")
    parser.add_argument("--steps", type=int, default=600, help="最大仿真步数")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    args = parser.parse_args()

    env = DryingSystemEnv()

    if args.compare:
        # ====== 多策略对比模式 ======
        print("=" * 60)
        print("        干燥系统 — 多策略批次仿真对比")
        print(f"        PCM初始温度: {args.T_init}°C")
        print("=" * 60)

        results = {}

        # 手动智能策略
        print("\n[1/3] 手动智能策略...")
        results["手动智能"] = run_batch(env, manual_smart, args.T_init, args.seed, args.steps)

        # 全开策略
        print("[2/3] 全开风机策略...")
        results["全开风机"] = run_batch(env, manual_always_fan, args.T_init, args.seed, args.steps)

        # 空闲策略
        print("[3/3] 空闲基线...")
        results["空闲基线"] = run_batch(env, idle_policy, args.T_init, args.seed, args.steps)

        # 尝试加载 AI 模型
        try:
            import torch
            from train_ppo import PolicyNet
            p = PolicyNet(5, 5, 128)
            sd = torch.load("policy_best.pt", map_location="cpu", weights_only=True)
            p.load_state_dict(sd)
            p.eval()
            scale = np.array([100, 100, 80, 100, 40], dtype=np.float32)
            print("\n[AI] AI 策略...")
            results["AI控制"] = run_batch(
                env, lambda s: ai_policy(s, p, scale), args.T_init, args.seed, args.steps)
        except Exception as e:
            print(f"\n[AI] 模型加载失败（跳过）: {e}")

        # 打印结果表
        print("\n" + "=" * 80)
        print(f"{'策略':<10} {'步数':>6} {'干燥时间':>8} {'RH初始':>8} {'RH最终':>8} "
              f"{'排粮次数':>8} {'总奖励':>10} {'终温PCM':>8} {'终温水箱':>8}")
        print("-" * 80)
        for name, res in results.items():
            dt = f"{res['dry_time']}步" if res['dry_time'] else "未完成"
            print(f"{name:<10} {res['steps']:>6} {dt:>8} {res['rh_initial']:>7.1f}% "
                  f"{res['rh_final']:>7.1f}% {res['discharges']:>8} "
                  f"{res['total_reward']:>10.1f} {res['T_pcm_final']:>7.1f}°C "
                  f"{res['T_water_final']:>7.1f}°C")

        # 绘制对比图
        plot_batch(results, f"干燥系统仿真对比 (PCM初始={args.T_init}°C)", "drying_compare.png")

    else:
        # ====== 单策略模式 ======
        if args.manual:
            policy = manual_smart
            policy_name = "手动智能策略"
        else:
            try:
                import torch
                from train_ppo import PolicyNet
                p = PolicyNet(5, 5, 128)
                sd = torch.load("policy_best.pt", map_location="cpu", weights_only=True)
                p.load_state_dict(sd)
                p.eval()
                scale = np.array([100, 100, 80, 100, 40], dtype=np.float32)
                policy = lambda s: ai_policy(s, p, scale)
                policy_name = "AI 策略 (policy_best.pt)"
            except Exception as e:
                print(f"AI模型加载失败: {e}，改用默认手动策略")
                policy = manual_smart
                policy_name = "手动智能策略（备用）"

        print(f"\n控制策略: {policy_name}")
        print(f"PCM 初始: {args.T_init}°C\n")

        result = run_batch(env, policy, args.T_init, args.seed, args.steps)

        # 打印步骤详情（每20步）
        log = result["log"]
        print(f"{'步':>4} {'PCM°C':>7} {'RH%':>7} {'水箱°C':>7} {'风机':>6} {'风门':>6} {'加热':>5} {'排粮':>5} {'奖励':>8}")
        print("-" * 64)
        for i in range(0, result["steps"], max(1, result["steps"] // 30)):
            print(f"{i:>4} {log['T_pcm'][i]:>7.1f} {log['RH_dryer'][i]:>7.1f} "
                  f"{log['T_water'][i]:>7.1f} {log['fan'][i]:>5.2f} "
                  f"{log['damper'][i]:>5.2f} {'开' if log['heater'][i] else '关':>5} "
                  f"{'是' if log['discharge'][i] else '否':>5} "
                  f"{log['cum_reward'][i]:>8.1f}")

        print("-" * 64)
        print(f"\n干燥{'完成' if result['dry_time'] else '未完成'} | 用时{result['steps']}步")
        if result['dry_time']:
            print(f"干燥至<15%RH 耗时: {result['dry_time']} 步 = {result['dry_time']} 分钟(仿真)")
        print(f"湿度: {result['rh_initial']:.1f}% → {result['rh_final']:.1f}%")
        print(f"PCM:  → {result['T_pcm_final']:.1f}°C")
        print(f"水箱: → {result['T_water_final']:.1f}°C")
        print(f"排粮次数: {result['discharges']}")
        print(f"总奖励: {result['total_reward']:.1f}")

        # 单策略图
        plot_batch({policy_name: result}, f"{policy_name} - 干燥过程", "drying_single.png")

    env.close()
    print("\n仿真完成！")
