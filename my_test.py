import gymnasium as gym
from gymnasium import spaces
import numpy as np


class DryingSystemEnv(gym.Env):
    """
    5kg复合相变材料粮食干燥系统仿真环境
    相变温度: 37-44℃ | 辐照度: 800W/m2 | 热效率: 12%
    环境温度固定为18.6℃
    舵机控制热风分配：干燥仓 vs 水箱
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode=None):
        super(DryingSystemEnv, self).__init__()
        
        # ---------- 1. 动作空间（统一为 Box，兼容 Stable-Baselines3 等 RL 库） ----------
        # 动作维度: [fan, damper, pump, discharge, heater]
        #   fan:       0~1  风机转速
        #   damper:    0~1  舵机角度 (0→全干燥仓, 1→全水箱)
        #   pump:      0~1  水泵功率
        #   discharge: 0~1  排粮控制 (连续值，>=0.5 触发排粮)
        #   heater:    0~1  加热片开关 (>=0.5 开启加热, <0.5 关闭加热)
        self.action_space = spaces.Box(
            low=np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        # ---------- 2. 观测空间（归一化到 [0,1]） ----------
        # v34.9: 5维观测 [PCM温度, 干燥仓温度(DHT11), 干燥仓湿度(DHT11), 水箱温度, 环境温度常量25]
        # 归一化后全部缩放到 [0, 1]
        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(5,), dtype=np.float32
        )

        # ---------- 3. 物理参数 ----------
        self.m_pcm = 5.0              # kg
        self.cp_solid = 2.0           # kJ/(kg·K)
        self.cp_liquid = 2.5
        self.latent_heat = 180.0      # kJ/kg
        self.T_solidus = 37.0
        self.T_liquidus = 44.0
        
        self.ambient_temp = 18.6
        self.ambient_hum = 60.0
        
        # 环境扰动参数（高斯噪声模拟天气波动，不包含昼夜变化）
        self.temp_noise_std = 2.0     # 温度扰动标准差 (℃)
        self.hum_noise_std = 5.0      # 湿度扰动标准差 (%)
        
        self.dt = 60.0                # 秒
        self.max_steps = 600         # 短 episode 让排粮终端奖励比例更大
        
        self.P_solar = 70.0          # W (v30: 提高至可持续干燥水平，fan=0.6@60°C可维持)
        
        # 水箱参数 (v34.6: 水量 20kg → 5kg，蓄热更快、更容易达到水泵工作温度)
        self.m_water = 5.0            # kg (水量减少，温升更快)
        self.cp_water = 4.18          # kJ/(kg·K)
        self.T_water_init = 18.6      # 水箱初始温度
        self.T_water_threshold_pump = 45.0  # 水泵补热水温门槛（℃）

        # 状态变量
        self.state = None
        self.steps = 0
        self.prev_action = np.zeros(5, dtype=np.float32)  # 上一步动作，用于平滑惩罚

        # 加热片参数（v34: 新增加热片执行器）
        # 实测：原参数(30W+coef4)会把PCM推至90℃过热，导致模型弃用加热片
        # v34.1: 降为温和辅助热源，功率与太阳能(70W)同量级，避免一开就过热
        self.P_heater = 40.0        # W (加热片功率，略低于太阳能)
        self.heater_coef = 1.5      # 换热系数 W/℃（减弱对流传热，防止过热）
        self.T_heater_cutoff = 70.0 # 加热片关闭温度（PCM 超过此值强制关断，防过热）

    def _get_pcm_cp(self, T):
        if T < self.T_solidus:
            return self.cp_solid
        elif T > self.T_liquidus:
            return self.cp_liquid
        else:
            apparent = self.cp_solid + (self.latent_heat / (self.T_liquidus - self.T_solidus))
            return min(apparent, 30.0)

    def _physical_model(self, state, action):
        T_pcm, T_amb, RH_amb, T_in, T_out, RH_dryer, T_water = state
        # 统一从 Box 动作空间解析（兼容 Stable-Baselines3）
        # action 形状为 (5,)，顺序: [fan, damper, pump, discharge, heater]
        fan = float(np.clip(action[0], 0.0, 1.0))
        damper = float(np.clip(action[1], 0.0, 1.0))   # 0→干燥仓, 1→水箱
        pump = float(np.clip(action[2], 0.0, 1.0))
        discharge = 1 if float(action[3]) >= 0.5 else 0  # 连续值阈值判断排粮
        # 加热片：连续值，>=0.5 开启加热，<0.5 关闭（与 ESP32 硬件映射一致）
        heater = 1 if float(action[4]) >= 0.5 else 0

        # ===== 环境温湿度扰动（高斯噪声模拟天气波动） =====
        if self.np_random is None:
            self.np_random = np.random.default_rng()
        temp_noise = self.np_random.normal(0, self.temp_noise_std)
        hum_noise = self.np_random.normal(0, self.hum_noise_std)

        T_amb = np.clip(self.ambient_temp + temp_noise, 10.0, 40.0)
        RH_amb = np.clip(self.ambient_hum + hum_noise, 20.0, 95.0)

        # ----- A. PCM温度变化 -----
        Q_solar = (self.P_solar * self.dt) / 1000.0   # kJ
        
        # 热损失 (仅当T_pcm > T_amb)
        if T_pcm > T_amb:
            loss_coef = 0.5   # W/℃
            Q_loss = loss_coef * (T_pcm - T_amb) * self.dt / 1000.0
        else:
            Q_loss = 0.0

        # 风机换热（回退到已验证的 v24 参数）
        fan_coef = 6.0   # v30: 降低换热系数，配合 P_solar=70W 使干燥可持续
        if T_pcm > T_in:
            Q_conv_total = fan * fan_coef * (T_pcm - T_in) * self.dt / 1000.0
        else:
            Q_conv_total = -fan * fan_coef * (T_in - T_pcm) * self.dt / 1000.0

        # 加热片贡献热量（v34.1）：温和辅助热源，随温度升高而减弱，防止过热
        Q_heater = 0.0
        if heater == 1:
            # 线性衰减：低温时补热最多，接近截止温度时趋于 0
            # 温度越低，加热片功率越大；越接近 cutoff，功率越小 → 自然防过热
            if T_pcm < self.T_heater_cutoff:
                eff = (self.T_heater_cutoff - T_pcm) / self.T_heater_cutoff  # 0~1 衰减系数
                Q_heater = (self.P_heater * eff) * self.dt / 1000.0 + \
                           self.heater_coef * (self.T_heater_cutoff - T_pcm) * self.dt / 1000.0
            # T_pcm >= cutoff 时加热片无效（安全保护）

        # 净热量
        Q_net = Q_solar - Q_loss - Q_conv_total + Q_heater
        cp_eff = self._get_pcm_cp(T_pcm)
        if cp_eff < 0.1:
            cp_eff = 2.0
        dT_pcm = Q_net / (self.m_pcm * cp_eff)
        T_pcm_new = T_pcm + dT_pcm
        T_pcm_new = np.clip(T_pcm_new, 0.0, 110.0)

        # ----- B. 热风温度（进口风温）取决于PCM温度及舵机分配 -----
        # 热风温度 = 环境 + 从PCM获得的热量 / (风量*比热) 简化：热风温度近似为PCM温度*0.6 + 环境*0.4
        T_hot = T_amb + 0.6 * (T_pcm_new - T_amb)   # 最高可达约 T_amb + 0.6*(T_pcm-T_amb)

        # 舵机分配：进入干燥仓的比例 = 1 - damper，进入水箱的比例 = damper
        frac_dryer = 1.0 - damper
        frac_tank = damper

        # 进口风温 (进入干燥仓的风温) 与 进入水箱的风温 相同，但流量分配不同
        T_in_new = T_hot   # 实际进口温度（未混合前）

        # ----- C. 干燥仓湿度变化（干燥效率随 PCM 温度增长） =====
        # 低温时开风机 = 白白耗能，干燥效率几乎为零
        dry_efficiency = np.clip((T_pcm_new - 20.0) / 40.0, 0.0, 1.0)  # 20°C→0%, 60°C→100%
        dehumidify = fan * frac_dryer * 0.8 * dry_efficiency * (self.dt / 60.0)
        RH_dryer_new = RH_dryer - dehumidify
        RH_dryer_new = np.clip(RH_dryer_new, 5.0, 95.0)

        # 出口风温 = 进口风温 - 干燥室蒸发冷却 (与除湿量相关)
        cooling = dehumidify * 0.5   # 每除湿1%降温约0.5℃（简化）
        T_out_new = T_in_new - cooling
        T_out_new = np.clip(T_out_new, T_amb, 110.0)

        # ----- D. 水箱温度变化 (吸收进入水箱的热量) -----
        # 进入水箱的热功率 = 风机带出的热量 * frac_tank
        Q_tank_in = Q_conv_total * frac_tank   # 注意 Q_conv_total 是从PCM带走的热量（可正可负）
        # 但如果Q_conv_total为负（环境加热PCM），水箱不应被冷却，所以仅当Q_conv_total>0时水箱吸热
        if Q_conv_total > 0:
            Q_tank_absorb = Q_conv_total * frac_tank
        else:
            Q_tank_absorb = 0.0

        # 水箱热损失 (与环境的温差)
        if T_water > T_amb:
            tank_loss_coef = 0.2   # W/℃
            Q_tank_loss = tank_loss_coef * (T_water - T_amb) * self.dt / 1000.0
        else:
            Q_tank_loss = 0.0

        # 水箱净热量
        Q_tank_net = Q_tank_absorb - Q_tank_loss
        dT_water = Q_tank_net / (self.m_water * self.cp_water)
        T_water_new = T_water + dT_water
        T_water_new = np.clip(T_water_new, T_amb, 90.0)   # 上限90℃

        # ----- E. 水泵直接干燥（v34.5: 水箱热水→直接对干燥仓粮食干燥）-----
        # 语义：水泵把水箱里的热水泵送到干燥仓换热器，直接加热粮食加速干燥
        # 特点：只做热量交换、无湿度交互（水密封不喷湿粮食），热量不进 PCM
        # 水箱需先蓄热（damper 分热）达到门槛后，泵才有热可送
        pump_boost = 0.0   # 水泵额外增加的除湿量
        pump_effective = 0.0
        if pump > 0 and T_water >= self.T_water_threshold_pump:
            water_temp_eff = T_water
            if water_temp_eff > T_amb:   # 水箱有热可送
                pump_effective = pump
                # 直接干燥效率：水温越高、泵流量越大，除湿越强
                # 45℃→0.25, 60℃→0.55, 80℃→0.85（与风机除湿可比，蓄热后用泵才有意义）
                pump_dry_eff = np.clip((T_water - 40.0) / 45.0, 0.0, 1.0)
                pump_boost = pump * pump_dry_eff * 0.8 * (self.dt / 60.0)
                # 水箱消耗热量（送热水给干燥仓，水箱降温）
                Q_pump = pump * 3.0 * (T_water - T_amb) * self.dt / 1000.0
                T_water_new -= Q_pump / (self.m_water * self.cp_water)
        # 水温不达标 → pump_effective=0，水泵空转无效
        T_pcm_new = np.clip(T_pcm_new, 0.0, 110.0)
        T_water_new = np.clip(T_water_new, T_amb, 90.0)

        # 水泵的除湿叠加到干燥室（与风机除湿相加，无湿度交互）
        if pump_boost > 0:
            RH_dryer_new = RH_dryer_new - pump_boost
            RH_dryer_new = np.clip(RH_dryer_new, 5.0, 95.0)

        # 排粮舵机打开时重置干燥室湿度，并造成少量温度损失
        if discharge == 1:
            RH_dryer_new = self.ambient_hum
            T_pcm_new = T_pcm_new - 0.5

        new_state = np.array([T_pcm_new, T_amb, RH_amb, T_in_new, T_out_new, RH_dryer_new, T_water_new],
                             dtype=np.float32)
        return new_state, dehumidify

    def _normalize_obs(self, state):
        """归一化到 ~[0, 1] 区间，提升训练稳定性
        v34.9: 观测重构为 5 维（传感器在干燥仓内部，删掉进出口风温）
          [T_pcm, T_dryer(=T_in), RH_dryer, T_water, T_amb_const]
        """
        T_pcm, T_amb, RH_amb, T_in, T_out, RH_dryer, T_water = state
        # 观测向量: [PCM温度, 干燥仓温度, 干燥仓湿度, 水箱温度, 环境温度常量]
        obs = np.array([T_pcm, T_in, RH_dryer, T_water, 25.0], dtype=np.float32)
        # 各维度缩放因子: [T_pcm, T_dryer, RH_dryer, T_water, T_amb]
        scale = np.array([100.0, 100.0, 80.0, 100.0, 40.0], dtype=np.float32)
        return (np.clip(obs, 0, scale) / scale).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 环境温湿度带扰动
        temp_noise = self.np_random.normal(0, self.temp_noise_std)
        hum_noise = self.np_random.normal(0, self.hum_noise_std)
        T_amb_init = np.clip(self.ambient_temp + temp_noise, 10.0, 40.0)
        RH_amb_init = np.clip(self.ambient_hum + hum_noise, 20.0, 95.0)

        # v34.12: 初始温度扩展到 [25, 85] — 覆盖冷启动低温场景，让模型学会低温开加热片
        # 此前 v22 用 [45,85]，模型从没见过 30℃ 低温，导致冷启动时加热片不启动
        T_pcm_init = float(self.np_random.uniform(25.0, 85.0))
        T_water_init = T_pcm_init * 0.5 + T_amb_init * 0.5  # 初始水箱温度
        T_in_init = T_pcm_init * 0.6 + T_amb_init * 0.4     # 初始热风温度
        T_out_init = T_in_init - 2.0                         # 出口风温略低

        initial_state = np.array(
            [T_pcm_init, T_amb_init, RH_amb_init, T_in_init, T_out_init, RH_amb_init, T_water_init],
            dtype=np.float32
        )
        self.state = initial_state
        self.steps = 0
        self.prev_action = np.zeros(5, dtype=np.float32)  # 重置动作历史
        return self._normalize_obs(self.state), {}

    def step(self, action):
        next_state, dehumidify = self._physical_model(self.state, action)
        T_pcm, T_amb, RH_amb, T_in, T_out, RH_dryer, T_water = next_state
        # 从统一 Box 动作中解析
        fan = float(np.clip(action[0], 0.0, 1.0))
        damper = float(np.clip(action[1], 0.0, 1.0))
        pump = float(np.clip(action[2], 0.0, 1.0))
        discharge = 1 if float(action[3]) >= 0.5 else 0
        discharge_val = float(np.clip(action[3], 0.0, 1.0))  # 连续值，用于平滑奖励
        heater = 1 if float(action[4]) >= 0.5 else 0  # 加热片开关

        # ============ 奖励函数 v33：极简设计，只有干燥+能耗+安全 ============
        # 核心哲学：奖励条款越少，模型越难"作弊"，必须真正学会干燥

        old_RH = self.state[5]
        delta_RH = old_RH - RH_dryer                     # 正=变干了

        grain_ready = (RH_dryer < 15.0)

        # ====== 1. 干燥进度（绝对主导） ======
        humidity_reward = delta_RH * 50.0                 # v33: 最强信号

        # ====== 2. 成功排粮 ======
        discharge_reward = 0.0
        if grain_ready:
            discharge_reward = discharge_val * 250.0 + 100.0  # 最大 350
        elif discharge_val > 0.3:
            discharge_reward = -(discharge_val - 0.3) * 3.0   # 非干燥时禁止排粮

        # ====== 3. 能耗（轻度惩罚，避免浪费） ======
        # v34.3: 提高能耗惩罚权重，驱动模型学会"够用就减"，避免恒定满速输出
        # 风机耗电最高，惩罚最重；加热片/水泵次之
        energy_cost = -(fan * 0.25 + pump * 0.15 + heater * 0.15)

        # ====== 3a. 风机引导奖励（v34.11 重写：极陡梯度 + 强惩罚）======
        # 目标：让风机真正"动起来"——湿度高强加风、低湿减风省能
        # 风机是干燥主驱动，必须随干燥室湿度明显变化：
        #  期望风量 = 基础(湿度) × 温度增强 + 底部偏置
        #  湿度主导更陡峭：RH 10%→0.05, 30%→0.30, 50%→0.65, 70%→0.85, 90%→0.95
        #  温度修正：T 40℃→0.4, 50℃→0.6, 60℃→1.0
        # v34.11: 偏差惩罚系数从 2.5 提升到 4.0，让模型无法用"恒定中速"逃避
        base_fan = np.clip((RH_dryer - 5.0) / 50.0, 0.05, 0.95)  # 陡峭湿度主导
        temp_factor = np.clip((T_pcm - 36.0) / 24.0, 0.4, 1.0)   # 更陡温度系数
        expected_fan = np.clip(base_fan * temp_factor + 0.10, 0.0, 1.0)
        fan_dev = fan - expected_fan
        fan_guidance = -4.0 * abs(fan_dev)  # 极强惩罚，逼模型匹配期望

        # ====== 3aa. 风门引导奖励（v34.10 重写：陡峭温度梯度）======
        # 目标：让风门随温度明显变化——温度高多分热给水箱蓄热、温度低全送干燥仓
        # 物理：damper=0 全送干燥仓，damper=1 全送水箱蓄热
        #  温度低(<45)：热量不足 → 全送干燥仓（期望 damper≈0.05）
        #  温度中(45~60)：干燥够用 → 多余热分给水箱（期望 damper 快速上升）
        #  温度高(>60)：热量过剩 → 大幅蓄热（期望 damper≈0.7）
        #  关键：45~60℃ 内期望 damper 从 0.05 陡升到 0.6，产生强梯度
        if T_pcm < 45.0:
            expected_damper = 0.05   # 低温全送干燥仓
        elif T_pcm < 60.0:
            expected_damper = 0.05 + (T_pcm - 45.0) / 15.0 * 0.60   # 45→60℃: 0.05→0.65
        else:
            expected_damper = 0.65 + (T_pcm - 60.0) / 20.0 * 0.10   # >60℃: 0.65→0.75
        damper_dev = damper - expected_damper
        # v34.11: 系数从 2.0 提高到 4.0
        damper_guidance = -4.0 * abs(damper_dev)

        # ====== 3b. 加热片引导奖励（v34.15：低温加热优先）======
        # 目标：让模型学会"低温时开加热、高温时关加热"，低温加热是核心功能
        # 冷启动/太阳能不足时（PCM 低温），加热片必须启动才能正常干燥。
        # v34.15 关键：低温开加热的激励大幅增强（+3.0），低温关加热重罚（-2.0），
        #   使"低温开加热"的净收益（+3.0 vs -2.0 = 5 差距）远超模型"全关省事"的倾向。
        #  - 温度 < 50 开加热 → +3.0（低温补热：强鼓励，优先级最高）
        #  - 50~53 开加热 → 0（过渡带：中性）
        #  - 温度 >= 53 开加热 → -4.0（高温还加热：重罚，防过热）
        #  - 温度 < 50 却关加热 → -2.0（低温漏开：重罚）
        heater_guidance = 0.0
        if heater == 1:
            if T_pcm < 50.0:
                heater_guidance = 3.0   # 低温开加热：强鼓励（v34.15: 1.5→3.0）
            elif T_pcm < 53.0:
                heater_guidance = 0.0   # 50~53℃ 过渡带：中性
            else:
                heater_guidance = -4.0  # >=53℃ 高温还加热：重罚（防过热）
        elif T_pcm < 50.0:
            heater_guidance = -2.0      # 低温却关加热：重罚（v34.15: 0.5→2.0，防漏开）

        # ====== 3c. 水泵直接干燥引导奖励（v34.4 重设计）======
        # 水泵作用：水箱热水→直接对干燥仓粮食干燥（无湿度交互，水密封）
        # 目标：让模型学会"水箱有热水 + 干燥室还湿 时开泵直接干燥"
        #  - 水箱有热(T_water>45) 且干燥室湿度高(RH_dryer>25) 开泵 → 强鼓励
        #  - 干燥室已达标(RH_dryer<15) 还开泵 → 强惩罚（粮已干，泵多余）
        #  - 水箱没热(<45) 开泵 → 空转（下方 pump_waste 惩罚）
        pump_guidance = 0.0
        if pump > 0:
            if T_water > 45.0 and RH_dryer > 25.0:
                pump_guidance = 1.5   # 水箱有热且干燥室湿：开泵直接干燥，鼓励（v34.10: 1.0→1.5）
            elif RH_dryer < 15.0:
                pump_guidance = -1.5  # 粮已干还开泵：惩罚（多余）（v34.10: 1.0→1.5）
            else:
                pump_guidance = -0.5  # 其他开泵场景：轻微下拉，防贴 0.5 游走
        elif T_water > 45.0 and RH_dryer > 30.0:
            pump_guidance = -1.0      # 水箱有热且干燥室湿却关泵：惩罚（v34.10: 0.5→1.0）

        # ====== 水泵空转惩罚（水温不达标时开水泵完全无效） ======
        pump_waste_penalty = 0.0
        if pump > 0 and T_water < self.T_water_threshold_pump:
            pump_waste_penalty = -pump * 0.3

        # ====== 4. 高温安全 ======
        safety_penalty = 0.0
        if T_pcm > 90.0:
            safety_penalty -= (T_pcm - 90.0) * 0.5
        if T_out > 60.0:
            safety_penalty -= (T_out - 60.0) * 0.3          # 防烤粮

        # ====== 5. 动作平滑（轻度，防抖动） ======
        delta_fan = abs(fan - self.prev_action[0])
        delta_pump = abs(pump - self.prev_action[2])
        delta_heater = abs(heater - self.prev_action[4])  # 加热片频繁启停会伤继电器/MOS
        smoothness_penalty = -(delta_fan + delta_pump + delta_heater * 0.05) * 0.05

        # ====== 6. 时间惩罚 ======
        time_penalty = -0.01                               # v33: 加大时间压力

        # ====== 总奖励 ======
        reward = (humidity_reward +
                  energy_cost +
                  fan_guidance +
                  damper_guidance +
                  heater_guidance +
                  pump_guidance +
                  pump_waste_penalty +
                  discharge_reward +
                  safety_penalty +
                  smoothness_penalty +
                  time_penalty)

        reward = np.clip(reward, -10, 500)  # 提高上限，排粮+350 不被截断

        self.state = next_state
        self.steps += 1
        self.prev_action = np.array([fan, damper, pump, discharge_val, heater], dtype=np.float32)

        terminated = grain_ready
        truncated = self.steps >= self.max_steps

        info = {
            "T_pcm": T_pcm,
            "RH_dryer": RH_dryer,
            "T_in": T_in,
            "T_out": T_out,
            "T_water": T_water,
            "grain_ready": grain_ready,
            "reward": reward,
            "humidity": humidity_reward,
            "energy": energy_cost,
            "heater_guidance": heater_guidance,
            "pump_guidance": pump_guidance,
            "fan_guidance": fan_guidance,
            "damper_guidance": damper_guidance,
            "pump_waste": pump_waste_penalty,
            "safety": safety_penalty,
            "discharge": discharge_reward,
            "smoothness": smoothness_penalty,
        }

        return self._normalize_obs(next_state), reward, terminated, truncated, info

    def render(self):
        if self.state is not None:
            print(f"Step {self.steps}: PCM={self.state[0]:.2f}℃, RH={self.state[5]:.2f}%, Water={self.state[6]:.2f}℃")


# 测试代码
if __name__ == "__main__":
    env = DryingSystemEnv()
    obs, info = env.reset()
    print("动作空间:", env.action_space)
    print("观测空间:", env.observation_space)
    print("\n随机策略测试 (100步)...")
    total_reward = 0
    for i in range(100):
        action = env.action_space.sample()  # 返回 shape=(5,) 的 numpy 数组
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if i % 10 == 0:
            print(f"Step {i}: PCM={obs[0]:.2f}, RH={obs[2]:.2f}, "
                  f"Water={obs[3]:.2f}, action={action.round(2)}, Reward={reward:.2f}")
        if terminated or truncated:
            break
    print(f"总奖励: {total_reward:.2f}")
    env.close()