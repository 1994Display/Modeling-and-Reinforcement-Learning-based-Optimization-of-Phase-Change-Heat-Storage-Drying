"""
相变材料储热放热平衡模型（独立于 AI 干燥模型）

场景：PCM 在"阳光直射"和"遮蔽处"之间循环
  - 直射状态：PCM 在集热器中，接收太阳辐射，吸热储热
  - 遮蔽状态：PCM 移出集热器，无太阳辐射，通过自然散热+风机对流放热
  - 平衡目标：直射阶段吸热量 = 遮蔽阶段放热量（相同 PCM 温度下）
  - 调节手段：风机功率（影响对流放热速率）

用法:
  python pcm_balance_model.py                    # 默认运行平衡扫描
  python pcm_balance_model.py --fan 0.5          # 指定风机功率运行
  python pcm_balance_model.py --scan              # 扫描风机功率找平衡点
  python pcm_balance_model.py --plot              # 绘制温度曲线图
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse


class PCMBalanceModel:
    """
    相变材料储热放热平衡模型

    两种状态循环：
      STATE_EXPOSED  = 阳光直射（集热）：Q_solar 注入，PCM 吸热
      STATE_SHADED   = 遮蔽处（散热）：无太阳辐射，风机+自然散热放热

    平衡条件：直射阶段吸热量 = 遮蔽阶段放热量
    """

    # 运行状态
    STATE_EXPOSED = 0   # 阳光直射（集热吸热）
    STATE_SHADED = 1   # 遮蔽处（散热放热）

    def __init__(self):
        # ========== PCM 物理参数 ==========
        self.m_pcm = 5.0              # kg
        self.cp_solid = 2.0           # kJ/(kg·K) 固态比热
        self.cp_liquid = 2.5          # kJ/(kg·K) 液态比热
        self.latent_heat = 180.0      # kJ/kg 相变潜热
        self.T_solidus = 37.0         # 固相点 ℃
        self.T_liquidus = 44.0        # 液相点 ℃
        self.T_ref = 20.0             # 焓值参考温度 ℃

        # ========== 环境参数 ==========
        self.T_amb = 25.0             # 环境温度 ℃（常量）
        self.T_in = 25.0              # 进口风温 ℃（风机进风温度）

        # ========== 能量参数 ==========
        self.P_solar = 70.0           # W 太阳能辐射功率（直射时）
        self.loss_coef = 0.5          # W/℃ 自然散热系数
        self.fan_coef = 6.0           # W/℃ 风机对流换热系数

        # ========== 仿真参数 ==========
        self.dt = 60.0                # 秒/步
        self.exposed_steps = 120      # 直射阶段步数（2小时）
        self.shaded_steps = 120       # 遮蔽阶段步数（2小时）

        # ========== 状态变量 ==========
        self.T_pcm = 25.0             # PCM 温度 ℃
        self.H_pcm = 0.0              # PCM 焓值 kJ（用焓值法保证相变对称）
        self.state = self.STATE_EXPOSED
        self.step_in_phase = 0        # 当前阶段内的步数
        self.total_steps = 0

        # 初始化焓值
        self.H_pcm = self._temp_to_enthalpy(self.T_pcm)

    # ==================== 焓值法（保证相变吸热放热对称） ====================

    def _temp_to_enthalpy(self, T):
        """根据温度计算 PCM 焓值 kJ（升温路径）"""
        H_solid = self.m_pcm * self.cp_solid * (self.T_solidus - self.T_ref)
        H_liquid = H_solid + self.m_pcm * self.latent_heat
        if T <= self.T_solidus:
            return self.m_pcm * self.cp_solid * (T - self.T_ref)
        elif T >= self.T_liquidus:
            return H_liquid + self.m_pcm * self.cp_liquid * (T - self.T_liquidus)
        else:
            f = (T - self.T_solidus) / (self.T_liquidus - self.T_solidus)
            return H_solid + self.m_pcm * self.latent_heat * f + \
                   self.m_pcm * self.cp_solid * (T - self.T_solidus) * (1 - f) * 0.3  # 过渡项

    def _enthalpy_to_temp(self, H):
        """根据焓值反推 PCM 温度 ℃（降温路径，与升温对称）"""
        H_solid = self.m_pcm * self.cp_solid * (self.T_solidus - self.T_ref)
        H_liquid = H_solid + self.m_pcm * self.latent_heat
        if H <= H_solid:
            return self.T_ref + H / (self.m_pcm * self.cp_solid)
        elif H >= H_liquid:
            return self.T_liquidus + (H - H_liquid) / (self.m_pcm * self.cp_liquid)
        else:
            f = (H - H_solid) / (self.m_pcm * self.latent_heat)
            return self.T_solidus + f * (self.T_liquidus - self.T_solidus)

    def _get_cp_eff(self, T):
        """表观比热 kJ/(kg·K)（用于能量计算）"""
        if T < self.T_solidus:
            return self.cp_solid
        elif T > self.T_liquidus:
            return self.cp_liquid
        else:
            apparent = self.cp_solid + (self.latent_heat / (self.T_liquidus - self.T_solidus))
            return min(apparent, 30.0)

    # ==================== 能量计算 ====================

    def _calc_heat_flows(self, T_pcm, fan_power, is_exposed):
        """
        计算各热流分量 kJ（单步 dt 内）
        返回: Q_solar, Q_loss, Q_conv, Q_net
        """
        # 太阳能（仅直射时）
        if is_exposed:
            Q_solar = (self.P_solar * self.dt) / 1000.0   # kJ
        else:
            Q_solar = 0.0

        # 自然散热（PCM 高于环境时）
        if T_pcm > self.T_amb:
            Q_loss = self.loss_coef * (T_pcm - self.T_amb) * self.dt / 1000.0
        else:
            Q_loss = 0.0

        # 风机对流换热
        if T_pcm > self.T_in:
            Q_conv = fan_power * self.fan_coef * (T_pcm - self.T_in) * self.dt / 1000.0
        else:
            Q_conv = -fan_power * self.fan_coef * (self.T_in - T_pcm) * self.dt / 1000.0

        # 净热量
        Q_net = Q_solar - Q_loss - Q_conv

        return Q_solar, Q_loss, Q_conv, Q_net

    # ==================== 仿真步进 ====================

    def reset(self, T_init=25.0):
        """重置仿真"""
        self.T_pcm = T_init
        self.H_pcm = self._temp_to_enthalpy(self.T_pcm)
        self.state = self.STATE_EXPOSED
        self.step_in_phase = 0
        self.total_steps = 0
        return self.T_pcm

    def step(self, fan_power=0.5, auto_switch=True):
        """
        单步仿真
        fan_power: 风机功率 0~1
        auto_switch: True=按固定时长自动切换状态（平衡模型用）
                     False=不自动切换，由外部控制器决定切换（切换控制器用）
        返回: T_pcm, info(含各热流分量、当前状态)
        """
        fan_power = np.clip(fan_power, 0.0, 1.0)
        is_exposed = (self.state == self.STATE_EXPOSED)

        # 计算热流
        Q_solar, Q_loss, Q_conv, Q_net = self._calc_heat_flows(
            self.T_pcm, fan_power, is_exposed)

        # 用焓值法更新 PCM 状态（保证相变吸热放热对称）
        self.H_pcm += Q_net
        self.T_pcm = self._enthalpy_to_temp(self.H_pcm)
        self.T_pcm = np.clip(self.T_pcm, 0.0, 110.0)

        # 阶段切换（auto_switch=False 时由外部控制器接管）
        self.step_in_phase += 1
        if auto_switch:
            if is_exposed and self.step_in_phase >= self.exposed_steps:
                self.state = self.STATE_SHADED
                self.step_in_phase = 0
            elif not is_exposed and self.step_in_phase >= self.shaded_steps:
                self.state = self.STATE_EXPOSED
                self.step_in_phase = 0

        self.total_steps += 1

        info = {
            "T_pcm": self.T_pcm,
            "H_pcm": self.H_pcm,
            "state": "EXPOSED" if is_exposed else "SHADED",
            "Q_solar": Q_solar,
            "Q_loss": Q_loss,
            "Q_conv": Q_conv,
            "Q_net": Q_net,
            "fan_power": fan_power,
            "step": self.total_steps,
        }
        return self.T_pcm, info

    # ==================== 平衡验证 ====================

    def run_cycles(self, fan_power=0.5, cycles=3, T_init=25.0):
        """
        运行多个直射-遮蔽周期，返回温度历史和平衡误差
        平衡判据：最后一个周期起始温度 ≈ 结束温度
        """
        self.reset(T_init)
        steps_per_cycle = self.exposed_steps + self.shaded_steps
        total_steps = steps_per_cycle * cycles

        log = {
            "step": [], "T_pcm": [], "state": [],
            "Q_solar": [], "Q_loss": [], "Q_conv": [], "Q_net": [],
            "H_pcm": [],
        }

        for _ in range(total_steps):
            T, info = self.step(fan_power)
            log["step"].append(info["step"])
            log["T_pcm"].append(info["T_pcm"])
            log["state"].append(info["state"])
            log["Q_solar"].append(info["Q_solar"])
            log["Q_loss"].append(info["Q_loss"])
            log["Q_conv"].append(info["Q_conv"])
            log["Q_net"].append(info["Q_net"])
            log["H_pcm"].append(info["H_pcm"])

        # 平衡误差：最后一个周期的起止温度差
        T_start_last_cycle = log["T_pcm"][-steps_per_cycle]
        T_end_last_cycle = log["T_pcm"][-1]
        balance_error = abs(T_end_last_cycle - T_start_last_cycle)

        # 分别统计最后一个周期的吸热/放热
        last_cycle = log["T_pcm"][-steps_per_cycle:]
        exposed_temps = last_cycle[:self.exposed_steps]
        shaded_temps = last_cycle[self.exposed_steps:]

        return {
            "balance_error": balance_error,
            "T_start": T_start_last_cycle,
            "T_end": T_end_last_cycle,
            "T_exposed_mean": np.mean(exposed_temps),
            "T_shaded_mean": np.mean(shaded_temps),
            "log": log,
        }

    def find_balance_fan(self, fan_range=None, cycles=3, tolerance=1.0):
        """
        扫描风机功率，找到使吸热放热平衡的值
        返回: 平衡风机功率，或 None
        """
        if fan_range is None:
            fan_range = np.arange(0.0, 1.01, 0.05)

        results = []
        for fan_test in fan_range:
            res = self.run_cycles(fan_power=fan_test, cycles=cycles)
            results.append({
                "fan": fan_test,
                "error": res["balance_error"],
                "T_start": res["T_start"],
                "T_end": res["T_end"],
            })
            print(f"  fan={fan_test:.2f}: error={res['balance_error']:.2f}°C "
                  f"(T: {res['T_start']:.1f}→{res['T_end']:.1f})")

        # 找最小误差
        best = min(results, key=lambda r: r["error"])
        print(f"\n  最佳风机功率: {best['fan']:.2f} (误差 {best['error']:.2f}°C)")
        if best["error"] < tolerance:
            print(f"  [OK] 达到平衡（误差 < {tolerance}°C）")
        else:
            print(f"  [FAIL] 未达到平衡（误差 >= {tolerance}°C），需调整参数")
        return best["fan"], results

    # ==================== 可视化 ====================

    def plot_cycles(self, fan_power=0.5, cycles=3, filename="pcm_balance.png"):
        """绘制温度和热流曲线"""
        res = self.run_cycles(fan_power=fan_power, cycles=cycles)
        log = res["log"]

        fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
        fig.suptitle(f"PCM 储热放热平衡 (fan={fan_power:.2f}, error={res['balance_error']:.2f}°C)",
                     fontsize=14, fontweight="bold")

        x = log["step"]
        states = log["state"]

        # 用背景色区分直射/遮蔽
        for i, s in enumerate(states):
            color = "#FFF3CD" if s == "EXPOSED" else "#D1ECF1"
            if i == 0 or states[i-1] != s:
                start = x[i]
            if i == len(states)-1 or states[i+1] != s:
                for ax in axes:
                    ax.axvspan(start, x[i], alpha=0.3, color=color)

        # (1) PCM 温度
        axes[0].plot(x, log["T_pcm"], color="#07A0C3", linewidth=2)
        axes[0].axhspan(self.T_solidus, self.T_liquidus, color="orange", alpha=0.15)
        axes[0].text(x[5], self.T_liquidus+0.5, "相变区间", fontsize=8, color="orange")
        axes[0].set_ylabel("PCM 温度 (°C)")
        axes[0].set_title("PCM 温度变化")
        axes[0].grid(True, alpha=0.3)

        # (2) 热流分量
        axes[1].plot(x, [q*1000/self.dt for q in log["Q_solar"]], label="太阳能", color="#FFD700", linewidth=1.5)
        axes[1].plot(x, [q*1000/self.dt for q in log["Q_loss"]], label="自然散热", color="#888888", linewidth=1.5)
        axes[1].plot(x, [q*1000/self.dt for q in log["Q_conv"]], label="风机对流", color="#07A0C3", linewidth=1.5)
        axes[1].plot(x, [q*1000/self.dt for q in log["Q_net"]], label="净热流", color="#E8313F", linewidth=2, linestyle="--")
        axes[1].set_ylabel("热流功率 (W)")
        axes[1].set_title("热流分量")
        axes[1].legend(fontsize=8)
        axes[1].grid(True, alpha=0.3)

        # (3) 焓值
        axes[2].plot(x, log["H_pcm"], color="#2ECC40", linewidth=2)
        axes[2].set_ylabel("焓值 (kJ)")
        axes[2].set_title("PCM 焓值（储能量）")
        axes[2].set_xlabel("步数")
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(filename, dpi=200, bbox_inches="tight")
        print(f"\n图表已保存: {filename}")


# ==================== 主程序 ====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PCM 储热放热平衡模型")
    parser.add_argument("--fan", type=float, default=0.5, help="风机功率 0~1")
    parser.add_argument("--scan", action="store_true", help="扫描风机功率找平衡点")
    parser.add_argument("--plot", action="store_true", help="绘制温度曲线图")
    parser.add_argument("--cycles", type=int, default=3, help="仿真周期数")
    args = parser.parse_args()

    model = PCMBalanceModel()

    print("=" * 60)
    print("相变材料储热放热平衡模型")
    print("=" * 60)
    print(f"PCM: {model.m_pcm}kg, 相变 {model.T_solidus}~{model.T_liquidus}°C, 潜热 {model.latent_heat} kJ/kg")
    print(f"太阳能: {model.P_solar}W, 自然散热: {model.loss_coef}W/°C, 风机系数: {model.fan_coef}W/°C")
    print(f"直射阶段: {model.exposed_steps}步, 遮蔽阶段: {model.shaded_steps}步")
    print(f"环境温度: {model.T_amb}°C, 进口风温: {model.T_in}°C")
    print("=" * 60)

    if args.scan:
        print("\n=== 扫描风机功率找平衡点 ===")
        balance_fan, results = model.find_balance_fan(cycles=args.cycles)

        if args.plot:
            print(f"\n绘制平衡风机曲线 (fan={balance_fan:.2f})...")
            model.plot_cycles(fan_power=balance_fan, cycles=args.cycles,
                              filename="pcm_balance_best.png")

    else:
        print(f"\n=== 运行仿真 (fan={args.fan:.2f}, {args.cycles} 周期) ===")
        res = model.run_cycles(fan_power=args.fan, cycles=args.cycles)
        print(f"平衡误差: {res['balance_error']:.2f}°C")
        print(f"  周期起始温度: {res['T_start']:.2f}°C")
        print(f"  周期结束温度: {res['T_end']:.2f}°C")
        print(f"  直射段均温: {res['T_exposed_mean']:.2f}°C")
        print(f"  遮蔽段均温: {res['T_shaded_mean']:.2f}°C")

        if res["balance_error"] < 1.0:
            print("  [OK] 达到平衡")
        else:
            print("  [FAIL] 未平衡，建议调整风机功率（用 --scan 扫描）")

    if args.plot and not args.scan:
        model.plot_cycles(fan_power=args.fan, cycles=args.cycles)
