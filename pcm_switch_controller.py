"""
PCM 集热器切换控制器（规则版）

目标：判断何时切换集热器（取代"光照强度 ≥400 lux"的简单阈值）
核心："吸热完成的同时放热完成，然后切换，达到持续供热"

输入（4 特征）:
  [PCM温度, 光照强度, PCM焓值, PCM温度趋势]

输出:
  切换指令: 0 = 保持当前状态, 1 = 切换（集热<->放热）

决策规则（基于焓值边界 + 温度趋势）:
  - 集热状态(吸热): 焓值 >= H_high 且 温度趋势 <= 0（吸满不再升）→ 切换去放热
  - 放热状态(放热): 焓值 <= H_low 且 温度趋势 >= 0（放空不再降）→ 切换去集热

用法:
  python pcm_switch_controller.py                    # 运行吸放衔接仿真
  python pcm_switch_controller.py --plot              # 绘制曲线图
  python pcm_switch_controller.py --steps 1200        # 指定仿真步数
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse
from pcm_balance_model import PCMBalanceModel


class SwitchController:
    """
    切换决策器：基于焓值 + 温度趋势 + 光照的规则判断

    ┌──────────────────────┐
    │  输入: [T_pcm, 光照, H_pcm, dT] │
    └──────────┬───────────┘
               ▼
    ┌──────────────────────┐
    │  规则决策 (吸放衔接)   │
    └──────────┬───────────┘
               ▼
    ┌──────────────────────┐
    │  输出: 切换(0/1)      │
    └──────────────────────┘
    """

    def __init__(self, H_high_frac=0.85, H_low_frac=0.25,
                 trend_threshold=0.05, min_hold_steps=10):
        """
        H_high_frac: 吸热上限（焓值达到最大储能量的比例）
        H_low_frac:  放热下限（焓值降到最大储能量的比例）
        trend_threshold: 温度趋势阈值 ℃/步（|dT| 小于此值认为"完成"）
        min_hold_steps: 最小保持步数（防频繁切换）
        """
        self.H_high_frac = H_high_frac
        self.H_low_frac = H_low_frac
        self.trend_threshold = trend_threshold
        self.min_hold_steps = min_hold_steps
        self.hold_steps = 0

        # 用于趋势计算的温度历史
        self.temp_history = []

    def reset(self):
        """重置决策器状态"""
        self.hold_steps = 0
        self.temp_history = []

    def _compute_trend(self, T_now):
        """计算温度趋势（最近几步的温升速率 ℃/步）"""
        self.temp_history.append(T_now)
        if len(self.temp_history) > 5:
            self.temp_history.pop(0)
        if len(self.temp_history) >= 3:
            # 简单线性趋势: (最新-最早)/(间隔步数)
            return (self.temp_history[-1] - self.temp_history[0]) / (len(self.temp_history) - 1)
        return 0.0

    def decide(self, T_pcm, light_intensity, H_pcm, current_state, dt,
               T_high=40.0, T_low=31.0):
        """
        决策是否切换集热器

        主判据：PCM 温度 + 温度趋势（dT≈0 表示吸/放热接近完成）
        辅助：焓值（作为输入特征保留，未来 ML 扩展用）

        参数:
          T_pcm:          PCM 温度 ℃
          light_intensity: 光照强度 lux（0 = 遮蔽, >0 = 有光）
          H_pcm:          PCM 当前焓值 kJ（保留作为特征，供后续 ML 使用）
          current_state:  当前状态（0=集热吸热, 1=放热）
          dt:             仿真步长 s
          T_high:         吸热完成参考温度 ℃（实际集热可达高温区）
          T_low:          放热完成参考温度 ℃（实际放热可达低温区）

        返回:
          (switch, info)  switch: 0/1 是否切换
        """
        dT = self._compute_trend(T_pcm)
        self.hold_steps += 1

        switch = 0
        reason = ""

        if current_state == 0:  # 集热吸热状态
            # 吸热完成：温度达到参考高温区 且 温升趋缓（接近集热平衡）
            if T_pcm >= T_high and dT <= self.trend_threshold:
                if self.hold_steps >= self.min_hold_steps:
                    switch = 1
                    reason = f"吸热完成(T={T_pcm:.1f}>={T_high:.0f}+趋缓)"
            # 附加：集热时有光照才吸热（无光集热无意义，提前切换）
            elif light_intensity < 100 and T_pcm > T_low + 2.0:
                if self.hold_steps >= self.min_hold_steps:
                    switch = 1
                    reason = "光照不足，提前停止集热"
        else:  # 放热状态
            # 放热完成：温度降到参考低温区 且 温降趋缓（接近放热平衡）
            if T_pcm <= T_low and dT >= -self.trend_threshold:
                if self.hold_steps >= self.min_hold_steps:
                    switch = 1
                    reason = f"放热完成(T={T_pcm:.1f}<={T_low:.0f}+趋缓)"

        if switch:
            self.hold_steps = 0

        info = {
            "dT": dT,
            "T_high": T_high,
            "T_low": T_low,
            "reason": reason,
        }
        return switch, info


class PCMSwitchSimulation:
    """
    带切换控制器的 PCM 仿真
    切换不再依赖固定时长，由 SwitchController 动态决策
    """

    def __init__(self, H_high_frac=0.85, H_low_frac=0.25, min_hold_steps=10):
        self.phys = PCMBalanceModel()   # 复用物理模型
        self.controller = SwitchController(
            H_high_frac=H_high_frac, H_low_frac=H_low_frac,
            min_hold_steps=min_hold_steps)

        # 光照配置（都是白天，但直射/遮蔽不同）
        self.light_exposed = 800.0    # 阳光直射照度 lux
        self.light_shaded = 50.0      # 遮蔽处照度 lux

        # 风机功率：集热时小（少放热多储热），放热时大（加速放热供热）
        self.fan_collect = 0.15
        self.fan_release = 0.85

    def _get_light(self, state):
        return self.light_exposed if state == self.phys.STATE_EXPOSED else self.light_shaded

    def _get_fan(self, state):
        return self.fan_collect if state == self.phys.STATE_EXPOSED else self.fan_release

    def _get_H_bounds(self):
        """计算 PCM 实际工作范围的焓值边界 (H_min, H_max) kJ
        实际工作范围: 从环境温度(25°C) 到 相变完成(44°C液态)
        因为太阳能 70W 只能把 PCM 加热到相变区附近，吸不满"理论最大储热"
        H_min = 放空时的焓值（环境温度对应）
        H_max = 吸满时的焓值（相变完成，液态 49°C）
        """
        H_min = self.phys._temp_to_enthalpy(self.phys.T_amb)      # 25°C 环境
        H_max = self.phys._temp_to_enthalpy(self.phys.T_liquidus + 5.0)  # 49°C 液态
        return H_min, H_max

    def run(self, T_init=25.0, max_steps=1200, T_amb_profile=None):
        """
        运行仿真，由切换控制器动态决策集热/放热切换
        T_amb_profile: 可选，环境温度变化序列（默认常量）
        """
        self.phys.reset(T_init)
        self.controller.reset()

        log = {
            "step": [], "T_pcm": [], "state": [], "light": [],
            "H_pcm": [], "dT": [], "fan": [], "switch": [], "reason": [],
        }

        for step in range(max_steps):
            # 当前状态
            cur_state = self.phys.state
            light = self._get_light(cur_state)
            fan = self._get_fan(cur_state)

            # 可选环境温度变化
            if T_amb_profile is not None:
                self.phys.T_amb = T_amb_profile[step]

            # 决策是否切换（主判据：温度；焓值作为特征传入）
            switch, cinfo = self.controller.decide(
                T_pcm=self.phys.T_pcm,
                light_intensity=light,
                H_pcm=self.phys.H_pcm,
                current_state=cur_state,
                dt=self.phys.dt,
            )

            # 执行物理步进（auto_switch=False：状态切换完全由控制器决定）
            T, info = self.phys.step(fan, auto_switch=False)

            # 如果决策要切换，立即切换状态
            if switch:
                self.phys.state = 1 - cur_state
                self.phys.step_in_phase = 0

            # 记录
            log["step"].append(step)
            log["T_pcm"].append(info["T_pcm"])
            log["state"].append(1 if info["state"] == "SHADED" else 0)
            log["light"].append(light)
            log["H_pcm"].append(info["H_pcm"])
            log["dT"].append(cinfo["dT"])
            log["fan"].append(fan)
            log["switch"].append(switch)
            log["reason"].append(cinfo["reason"])

        return log

    def analyze(self, log):
        """分析仿真结果：切换次数、吸放衔接质量"""
        switches = [i for i, s in enumerate(log["switch"]) if s]
        state_arr = np.array(log["state"])
        T_arr = np.array(log["T_pcm"])
        H_arr = np.array(log["H_pcm"])

        # 切换次数
        n_switches = len(switches)

        # 统计各状态占比
        collect_pct = 100 * (state_arr == 0).mean()
        release_pct = 100 * (state_arr == 1).mean()

        # 吸放衔接质量：切换时刻的焓值
        switch_H = [H_arr[i] for i in switches]
        H_min, H_max = self._get_H_bounds()

        # 持续供热评估：放热状态下温度是否维持在相变区间附近
        release_temps = T_arr[state_arr == 1]
        in_phaseband = 100 * ((release_temps >= 30) & (release_temps <= 44)).mean() if len(release_temps) else 0

        return {
            "n_switches": n_switches,
            "collect_pct": collect_pct,
            "release_pct": release_pct,
            "switch_H": switch_H,
            "H_max": H_max,
            "release_in_band_pct": in_phaseband,
        }

    def plot(self, log, filename="pcm_switch_control.png"):
        """绘制仿真曲线图"""
        fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
        fig.suptitle("PCM 集热器切换控制（吸放衔接持续供热）", fontsize=14, fontweight="bold")

        x = log["step"]
        states = log["state"]

        # 背景色区分集热/放热
        for i, s in enumerate(states):
            color = "#FFF3CD" if s == 0 else "#D1ECF1"
            if i == 0 or states[i-1] != s:
                start = x[i]
            if i == len(states)-1 or states[i+1] != s:
                for ax in axes:
                    ax.axvspan(start, x[i], alpha=0.3, color=color)

        # (1) PCM 温度
        axes[0].plot(x, log["T_pcm"], color="#07A0C3", linewidth=2)
        axes[0].axhspan(self.phys.T_solidus, self.phys.T_liquidus, color="orange", alpha=0.15)
        axes[0].set_ylabel("PCM 温度 (°C)")
        axes[0].set_title("PCM 温度 (黄色=集热, 蓝=放热)")
        axes[0].grid(True, alpha=0.3)

        # (2) 焓值 + 切换标记
        axes[1].plot(x, log["H_pcm"], color="#2ECC40", linewidth=2, label="焓值")
        H_min, H_max = self._get_H_bounds()
        H_range = H_max - H_min
        axes[1].axhline(H_min + H_range*0.85, color="red", linestyle="--", alpha=0.5, label="吸热上限(85%)")
        axes[1].axhline(H_min + H_range*0.25, color="blue", linestyle="--", alpha=0.5, label="放热下限(25%)")
        for i, sw in enumerate(log["switch"]):
            if sw:
                axes[1].scatter(x[i], log["H_pcm"][i], color="red", s=30, zorder=5)
        axes[1].set_ylabel("焓值 (kJ)")
        axes[1].set_title("PCM 焓值 (红点=切换时刻)")
        axes[1].legend(fontsize=8)
        axes[1].grid(True, alpha=0.3)

        # (3) 风机功率 + 状态
        axes[2].plot(x, log["fan"], color="#E8313F", linewidth=1.5, label="风机功率")
        axes[2].plot(x, log["light"], color="#FFD700", linewidth=1.0, alpha=0.7, label="光照(lux)")
        axes[2].set_ylabel("风机/光照")
        axes[2].set_xlabel("步数")
        axes[2].legend(fontsize=8)
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(filename, dpi=200, bbox_inches="tight")
        print(f"\n图表已保存: {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PCM 集热器切换控制器")
    parser.add_argument("--steps", type=int, default=1200, help="仿真步数")
    parser.add_argument("--plot", action="store_true", help="绘制曲线图")
    parser.add_argument("--T-init", type=float, default=25.0, help="PCM 初始温度")
    parser.add_argument("--high-frac", type=float, default=0.85, help="吸热上限比例")
    parser.add_argument("--low-frac", type=float, default=0.25, help="放热下限比例")
    parser.add_argument("--min-hold", type=int, default=10, help="最小保持步数")
    args = parser.parse_args()

    sim = PCMSwitchSimulation(
        H_high_frac=args.high_frac,
        H_low_frac=args.low_frac,
        min_hold_steps=args.min_hold,
    )

    print("=" * 60)
    print("PCM 集热器切换控制器（规则版）")
    print("=" * 60)
    print(f"输入特征: [PCM温度, 光照, 焓值, 温度趋势]")
    print(f"切换规则: 吸热完成(T>=40C且温度趋缓) -> 去放热")
    print(f"          放热完成(T<=31C且温度趋缓) -> 去集热")
    print(f"仿真步数: {args.steps} 步")
    print("=" * 60)

    log = sim.run(T_init=args.T_init, max_steps=args.steps)
    result = sim.analyze(log)

    print(f"\n=== 仿真结果 ===")
    print(f"切换次数: {result['n_switches']} 次")
    print(f"集热占比: {result['collect_pct']:.1f}%")
    print(f"放热占比: {result['release_pct']:.1f}%")
    print(f"放热期温度维持相变区: {result['release_in_band_pct']:.1f}%")
    print(f"切换时刻焓值: {[f'{h:.0f}' for h in result['switch_H']]} kJ")
    print(f"最大储热量: {result['H_max']:.0f} kJ")

    # 打印切换日志
    print(f"\n=== 切换时刻日志 ===")
    for i, sw in enumerate(log["switch"]):
        if sw:
            state_name = "放热->集热" if log["state"][i] == 1 else "集热->放热"
            print(f"  步{i:4d}: {state_name}  "
                  f"T={log['T_pcm'][i]:.1f}°C H={log['H_pcm'][i]:.0f}kJ "
                  f"dT={log['dT'][i]:+.3f} 原因: {log['reason'][i]}")

    if args.plot:
        sim.plot(log)
