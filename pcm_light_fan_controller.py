"""
光驱集热统一周期控制器（吸热/放热同时进行，不分先后）

物理结构（真实系统）:
  4 个集热器，3 个在阳光下吸热 + 1 个在遮蔽处放热，同时进行、同时结束。
  切换 = 转盘换位（把正在吸热和正在放热的集热器互换）。
  统一换位周期 = 10~20s（光照决定）；周期内吸放同时，周期结束换位。

目标（按用户需求）:
  1. 根据实时光照估算换位周期 T ∈ [t_min=10, t_max=20]s
     - t_min 可人为设定；t_max = t_min + 5
  2. 换位周期 = 干燥剩余时间（倒计时显示，实时倒数到 0）
  3. 风机 = 放热匹配功率（单倍），随光照实时调整
  4. 周期结束（吸热与放热同时完成）→ 转盘换位

模型要点:
  - 换位周期: T = ΔH_target / 净吸热功率(光照驱动)，钳位 [t_min, t_max]
  - 剩余时间: T − 已进行时间（倒计时）
  - 风机: P_solar(光照) / 满功率对流能力（放热匹配，单倍）
  - 切换: 倒计时到 0 或超时 t_max

用法:
  python pcm_light_fan_controller.py                     # 恒定光照仿真
  python pcm_light_fan_controller.py --vary              # 光照随时间变化(模拟云)
  python pcm_light_fan_controller.py --plot              # 绘制曲线
  python pcm_light_fan_controller.py --t-min 10 --t-max 20
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse


class LightFanController:
    """光驱集热统一周期控制器（吸热/放热同时，不分阶段）"""

    def __init__(self, t_min=10.0, t_max=20.0, dt=0.1,
                 dh_target_kj=0.60, fan_max_remove_w=90.0,
                 solar_w_per_lux=0.09, solar_max_w=80.0,
                 loss_coef_w_per_c=0.5, absorb_fan=0.15,
                 pcm_eff_mass_cp_j=100.0, t_amb=25.0, t_in=30.0):
        # ---- 时间参数 ----
        self.t_min = t_min              # 最短换位周期 s（可人为设定）
        self.t_max = t_max              # 最长换位周期 s
        self.dt = dt                    # 仿真步长 s

        # ---- 能量参数 ----
        self.dh_target_kj = dh_target_kj            # 每周期目标储热量 kJ
        self.fan_max_remove_w = fan_max_remove_w    # 风机满功率对流散热 W
        self.solar_w_per_lux = solar_w_per_lux      # 太阳能 W/lux
        self.solar_max_w = solar_max_w              # 太阳能最大 W
        self.loss_coef_w_per_c = loss_coef_w_per_c  # 自然散热 W/°C
        self.absorb_fan = absorb_fan                # 吸热对流损耗基准 0~1
        self.pcm_eff_mass_cp_j = pcm_eff_mass_cp_j  # 有效热容 J/°C（温度估算）

        # ---- 环境 ----
        self.t_amb = t_amb
        self.t_in = t_in

        # ---- 状态 ----
        self.phase_elapsed = 0.0    # 本周期已进行时间 s
        self.t_pcm = t_amb          # PCM 温度估算 ℃
        self.absorb_remain = 0.0    # 换位剩余时间 s（=干燥剩余时间显示）
        self.collector_fan = 0.0    # 集热循环控制风机 0~1
        self.absorb_duration = t_min  # 换位周期 s（光照驱动）
        self.switch_count = 0

    def reset(self):
        self.__init__(self.t_min, self.t_max, self.dt,
                      self.dh_target_kj, self.fan_max_remove_w,
                      self.solar_w_per_lux, self.solar_max_w,
                      self.loss_coef_w_per_c, self.absorb_fan,
                      self.pcm_eff_mass_cp_j, self.t_amb, self.t_in)

    # ==================== 功率计算 ====================

    def solar_power_w(self, light):
        """太阳能功率 W：随实时光照线性变化并封顶"""
        return float(np.clip(light * self.solar_w_per_lux, 0.0, self.solar_max_w))

    def loss_power_w(self):
        """自然散热功率 W"""
        return self.loss_coef_w_per_c * max(self.t_pcm - self.t_amb, 0.0)

    def absorb_net_w(self, light):
        """净吸热功率 W（吸热对流损耗 + 自然散热）"""
        p_conv = self.absorb_fan * self.fan_max_remove_w
        return self.solar_power_w(light) - p_conv - self.loss_power_w()

    # ==================== 单步 ====================

    def step(self, light):
        """
        单步推进（dt 秒）
        light: 当前光照 lux
        返回: (switch, info)  switch=1 表示本步结束触发换位
        """
        dt = self.dt
        self.phase_elapsed += dt
        switch = 0

        # 换位周期估算（光照驱动）：净吸热功率吸满目标储热的时间，钳位 [t_min, t_max]
        p_net_now = max(self.absorb_net_w(light), 2.0)
        self.absorb_duration = float(np.clip(
            self.dh_target_kj * 1000.0 / p_net_now, self.t_min, self.t_max))

        # 干燥剩余时间（倒计时）：换位周期 − 已进行时间，实时倒数到 0
        self.absorb_remain = float(np.clip(
            self.absorb_duration - self.phase_elapsed, 0.0, self.absorb_duration))

        # 风机 = 放热匹配功率（单倍）：随光照实时调整，强光放热快、弱光放热慢
        self.collector_fan = float(np.clip(
            self.solar_power_w(light) / self.fan_max_remove_w, 0.0, 1.0))

        # 温度估算（动态积分）：吸热与放热同时作用，取净热流
        p_conv = self.collector_fan * self.fan_max_remove_w
        p_loss = self.loss_power_w()
        dT = (self.solar_power_w(light) - p_conv - p_loss) * dt / self.pcm_eff_mass_cp_j
        self.t_pcm += dT

        # 切换判定：倒计时到 0（已持续 ≥ 换位周期）；或超时 t_max
        if (self.absorb_remain <= 0.0) or (self.phase_elapsed >= self.t_max):
            self.phase_elapsed = 0.0
            self.switch_count += 1
            switch = 1

        info = {
            "absorb_remain": self.absorb_remain,
            "collector_fan": self.collector_fan,
            "t_pcm": self.t_pcm,
            "absorb_duration": self.absorb_duration,
        }
        return switch, info


def run_sim(ctrl, light_profile, max_steps=4000):
    """运行仿真，返回日志"""
    log = {
        "step": [], "light": [], "fan": [],
        "absorb_remain": [], "t_pcm": [], "switch": [],
    }
    for i in range(max_steps):
        light = light_profile(i * ctrl.dt) if callable(light_profile) else light_profile
        switch, info = ctrl.step(light)
        log["step"].append(i * ctrl.dt)
        log["light"].append(light)
        log["fan"].append(info["collector_fan"])
        log["absorb_remain"].append(info["absorb_remain"])
        log["t_pcm"].append(info["t_pcm"])
        log["switch"].append(switch)
        # 提前结束：已跑多个完整周期
        if ctrl.switch_count >= 20 and i > 100:
            break
    return log


def analyze(log, t_min, t_max):
    """验证约束: 换位周期∈[t_min,t_max]、波动≤允许全范围、风机随光照自适应"""
    st = np.array(log["step"])
    sw = np.array(log["switch"])
    fan = np.array(log["fan"])

    switch_idx = np.where(sw == 1)[0]
    switch_times = [st[i] for i in switch_idx]
    bounds = [0.0] + switch_times
    intervals = [b - a for a, b in zip(bounds[:-1], bounds[1:])]
    # 末段可能不完整，丢弃
    if len(intervals) > 1 and intervals[-1] < 1.0:
        intervals = intervals[:-1]

    print("\n=== 约束验证 ===")
    if intervals:
        print(f"换位周期: min={min(intervals):.1f}s max={max(intervals):.1f}s "
              f"波动={max(intervals)-min(intervals):.1f}s (要求: [{t_min},{t_max}])")
        ok1 = all(t_min - 0.5 <= d <= t_max + 0.5 for d in intervals)
        ok2 = (max(intervals) - min(intervals)) <= (t_max - t_min)
        print(f"  周期∈[{t_min},{t_max}]: {'OK' if ok1 else 'FAIL'}  "
              f"波动≤全范围({t_max-t_min:.0f}s): {'OK' if ok2 else 'FAIL'}")
    else:
        ok1 = ok2 = False

    # 吸热与放热同时进行、同时结束 → 放热时间恒 = 换位周期（无需独立校验）
    print(f"\n放热阶段风机功率: min={fan.min():.2f} max={fan.max():.2f} "
          f"(随光照自适应，放热时间=换位周期)")

    all_ok = ok1 and ok2
    print(f"\n=== 总评: {'全部满足要求' if all_ok else '存在未满足项'} ===")
    return all_ok


def plot(log, t_min, t_max, filename="pcm_light_fan_control.png"):
    """绘制曲线"""
    x = log["step"]
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f"光驱集热统一周期控制 (换位周期∈[{t_min:.0f},{t_max:.0f}]s)", fontsize=14, fontweight="bold")

    # (1) 剩余时间（倒计时）+ 切换
    axes[0].plot(x, log["absorb_remain"], color="#07A0C3", linewidth=1.5, label="干燥剩余(s)")
    axes[0].axhline(t_min, color="green", linestyle="--", alpha=0.6, label=f"最短{t_min:.0f}s")
    axes[0].axhline(t_max, color="red", linestyle="--", alpha=0.6, label=f"最长{t_max:.0f}s")
    for i, s in enumerate(log["switch"]):
        if s:
            axes[0].scatter(x[i], 0, color="black", s=20, zorder=5)
    axes[0].set_ylabel("剩余时间 (s)")
    axes[0].set_title("干燥剩余时间（倒计时到0换位），黑点=换位")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # (2) 光照 + 风机功率
    axes[1].plot(x, log["light"], color="#FFD700", linewidth=1.2, label="光照(lux)")
    axes[1].plot(x, [f*100 for f in log["fan"]], color="#E8313F", linewidth=1.5, label="风机功率(%)")
    axes[1].set_ylabel("光照 / 风机")
    axes[1].set_title("实时光照 → 换位周期；风机=放热匹配功率(单倍)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # (3) PCM 温度估算
    axes[2].plot(x, log["t_pcm"], color="#2ECC40", linewidth=1.5)
    axes[2].set_ylabel("PCM 温度估算 (°C)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].set_title("PCM 动态温度估算")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=200, bbox_inches="tight")
    print(f"\n图表已保存: {filename}")


def light_profile_varying(t, base=800.0, period=40.0, amp=300.0):
    """光照随时间变化（模拟云层），周期 40s"""
    return max(150.0, base - amp * (0.5 + 0.5 * np.sin(2 * np.pi * t / period)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="光驱集热统一周期控制器仿真")
    parser.add_argument("--t-min", type=float, default=10.0, help="最短换位周期 s（可人为设定）")
    parser.add_argument("--t-max", type=float, default=20.0, help="最长换位周期 s")
    parser.add_argument("--vary", action="store_true", help="光照随时间变化（模拟云）")
    parser.add_argument("--light", type=float, default=800.0, help="恒定光照 lux")
    parser.add_argument("--plot", action="store_true", help="绘制曲线图")
    args = parser.parse_args()

    ctrl = LightFanController(t_min=args.t_min, t_max=args.t_max)
    print("=" * 62)
    print("光驱集热统一周期控制器（吸热/放热同时进行）")
    print("=" * 62)
    print(f"最短换位周期: {args.t_min}s  最长换位周期: {args.t_max}s")
    print(f"目标储热/周期: {ctrl.dh_target_kj} kJ  风机满功率对流: {ctrl.fan_max_remove_w} W")
    print(f"太阳能: {ctrl.solar_w_per_lux}W/lux 封顶{ctrl.solar_max_w}W")
    if args.vary:
        print(f"光照: 动态变化（模拟云，基准{args.light}lux ±300）")
    else:
        print(f"光照: 恒定 {args.light} lux")
    print("=" * 62)

    if args.vary:
        profile = lambda t: light_profile_varying(t, base=args.light)
        log = run_sim(ctrl, profile)
    else:
        log = run_sim(ctrl, args.light)

    all_ok = analyze(log, args.t_min, args.t_max)
    print(f"\n总换位次数: {ctrl.switch_count}")

    if args.plot:
        plot(log, args.t_min, args.t_max)

    import sys
    sys.exit(0 if all_ok else 1)
