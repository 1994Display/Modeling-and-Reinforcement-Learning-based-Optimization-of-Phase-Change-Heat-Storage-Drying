# Modeling-and-Reinforcement-Learning-based-Optimization-of-Phase-Change-Heat-Storage-Drying
开源相变材料（PCM）干燥系统的光照-送风热平衡仿真模型与相变控温干燥 PPO 强化学习控制模型（Python / PyTorch / Gym）
1. 项目背景
太阳能干燥普遍存在两个核心矛盾：
1. 光照波动：太阳辐照度随昼夜与天气剧烈变化，集热/储热单元的热输出不稳定；
2. 热量分配：富余热量若不引导储存会导致「烤粮」，若不利用则白白散失。
本仓库将干燥舱、相变储热体与储热水箱抽象为一个集中参数热平衡系统，用「光照输入 → PCM 储放热 → 送风分配 → 仓内除湿」的因果链建模，并训练 PPO（Proximal Policy Optimization）智能体，学会在光照与热量约束下自主决策：
- 风机转速：调节 PCM 与气流的换热强度；
- 风门开度：调节热风在「干燥仓 ↔ 储热水箱」之间的分配比例（防烤粮 + 储热）；
- 水泵流量：调节辅助换热；
- 排粮时机：干燥室湿度达标后自动排粮。
这套模型同时是「中国大学生智能装备创新设计大赛 · 国家一等奖」作品《光热相变储能 AI 智能粮食干燥设备》的 AI 控制核心，已通过仿真与整机实测双重验证。
2. 系统与物理模型
                 太阳辐照 P_solar
                       │
                       ▼
              ┌─────────────────┐   热损失 Q_loss
              │  PCM 储热体      │ ──────────────► 环境
              │  5kg，37~44℃相变 │
              └───┬─────────┬───┘
          风机对流 │         │ 风机对流（可逆）
                  ▼         ▼
        ┌────────────┐   ┌────────────┐
        │  干燥仓     │   │  储热水箱   │
        │  除湿/排粮  │   │  20kg 水    │
        └────────────┘   └────────────┘
             ▲ damper 分配热风比例（0→全干燥仓，1→全水箱）
2.1 关键物理参数（drying_env.py）
      参数
      值
      说明
      PCM 质量
      5.0 kg
      石蜡基复合相变材料
      相变温度区间
      37 ~ 44 ℃
      固液两相有效工作窗口
      相变潜热
      180 kJ/kg
      显热比热 2.0（固）/ 2.5（液）kJ/(kg·K)
      相变区等效比热
      cp + L/(T_liq−T_sol)，上限 30
      表观比热法处理潜热
      太阳输入功率
      P_solar = 100 W（可修改）
      实测部署时由光照传感器动态给定
      环境条件
      18.6 ℃ / 60% RH
      初始与边界条件
      水箱
      20 kg 水，比热 4.18 kJ/(kg·K)，上限 90 ℃
      阶段储热
      时间步长
      dt = 60 s，单回合最多 2000 步
      一步 = 一分钟
2.2 热平衡核心方程
- PCM 能量守恒：Q_net = Q_solar − Q_loss − Q_conv，其中对流换热 Q_conv = k·fan·(T_pcm − T_in)·dt（可正可负，双向换热）；相变区内用表观比热吸收潜热，实现温度「平台期」。
- 热风温度：T_hot = T_amb + 0.6·(T_pcm − T_amb)，再按风门比例分配至干燥仓与水箱。
- 仓内除湿：ΔRH = −k·fan·(1−damper) + k₂·pump（除湿正比于送到干燥仓的风量，水泵加湿）。
- 水箱储热：仅当换热为正时吸热 Q_tank = Q_conv·damper，伴随对环境热损失。
- 排粮事件：discharge=1 时仓内湿度复位至环境水平，PCM 与水箱产生少量扰动——真实设备中该动作由 ESP32 状态机在湿度稳定达标后触发。
3. 强化学习建模
3.1 观测空间（7 维，Box）
      序号
      状态量
      范围
      0
      PCM 温度 T_pcm
      0 ~ 120 ℃
      1
      环境温度 T_amb
      −10 ~ 50 ℃
      2
      环境湿度 RH_amb
      0 ~ 100 %
      3
      进口风温 T_in
      0 ~ 100 ℃
      4
      出口风温 T_out
      0 ~ 120 ℃
      5
      干燥室湿度 RH_dryer
      0 ~ 100 %
      6
      水箱温度 T_water
      0 ~ 120 ℃
3.2 动作空间（Dict）
      键
      类型
      范围
      含义
      fan
      Box
      0.0 ~ 1.0
      风机转速
      damper
      Box
      0.0 ~ 1.0
      风门分配（0→全干燥仓，1→全水箱）
      pump
      Box
      0.0 ~ 1.0
      水泵流量
      discharge
      Discrete(2)
      0 / 1
      排粮执行（部署端由独立状态机接管）
3.3 奖励函数（7 项，clip 至 [−100, 150]）
      分项
      表达式（摘要）
      设计意图
      ① 相变区间奖励
      区间内 +5.0；区间外 −0.5×距最近边界
      让 PCM 稳定停在 37~44 ℃ 有效储热窗口
      ② 有效除湿奖励
      dehumidify × 2.0
      直接奖励物理目标，鼓励把热风导向干燥仓
      ③ 能量成本惩罚
      `−(0.3·fan + 0.2·pump + 0.1·
      damper−0.5
      )`
      抑制风机水泵空转与风门频繁大幅切换
      ④ 水箱储热奖励
      ≥50 ℃ 给 +10；升温过程 (T−T_amb)×0.05
      阶段性鼓励储热
      ⑤ 完成奖励
      RH_dryer < 15% 给 +100
      干燥达标事件奖励
      ⑥ 排粮惩罚
      排粮时 −3.0
      防止频繁排粮中断干燥
      ⑦ 湿度塑形
      15 / (RH_dryer + 1)
      低湿度区梯度更陡，加速收敛
终局条件：terminated = RH_dryer < 15%（干燥完成）；truncated = 2000 步超时。
4. 快速开始
4.1 环境
pip install gymnasium numpy
# 训练需要
pip install stable-baselines3 torch
4.2 运行仿真（随机策略冒烟测试）
python drying_env.py
4.3 训练 PPO
from stable_baselines3 import PPO
from drying_env import DryingSystemEnv

env = DryingSystemEnv()
model = PPO("MultiInputPolicy", env, verbose=1, seed=42)
model.learn(total_timesteps=500_000)
model.save("ppo_drying")
4.4 评估
import numpy as np
from stable_baselines3 import PPO
from drying_env import DryingSystemEnv

env = DryingSystemEnv()
model = PPO.load("ppo_drying")
obs, _ = env.reset()
done = False
while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, r, term, trunc, info = env.step(action)
    env.render()
done = term or trunc
print("完成步数:", env.steps, "| 末状态:", obs)
5. 边缘部署（ESP32-S3）
训练完成的策略模型按以下流程部署到 ESP32-S3（N16R8），作为干燥设备的本地控制器：
1. 导出：PyTorch → ONNX → TensorFlow Lite；
2. 量化：INT8 全整数量化，量化前后动作输出做三端（PyTorch / ONNX / TFLite）一致性校验；
3. 推理：ESP32-S3 上以固定周期（实测 5 s）执行前向推理，输入来自 DHT11（环境温湿度）、DS18B20（水温）等真实传感器；
4. 执行：fan / damper / pump 映射为 PWM 输出；discharge 不直接驱动硬件，由 ESP32 独立状态机在湿度稳定达标后触发排粮舵机；
5. 安全：支持 AUTO（AI）/ 手动双模式切换，切换时执行器安全归零。
6. 项目结构
.
├── drying_env.py        # Gymnasium 仿真环境（热平衡模型 + 奖励函数）
├── train_ppo.py         # PPO 训练脚本
├── convert_model.py     # ONNX / TFLite 导出与 INT8 量化
├── esp32_controller/    # ESP32-S3 边缘部署固件（Arduino 框架）
└── README.md
  部分脚本仍在整理中，欢迎 Issue 催更 / PR。
7. 适用场景与扩展方向
- 光热储热系统的控制算法研究：替换 P_solar 为光照传感器实测序列即可模拟真实天气；
- 相变材料储放热策略仿真：调整 T_solidus / T_liquidus / latent_heat 对比不同 PCM 配方；
- 多能互补干燥装备的 AI 控制器原型验证；
- 嵌入式 RL（TinyML）教学案例：完整覆盖「仿真 → 训练 → 量化 → 边缘部署」链路。
8. 许可证
MIT License。引用或二次开发请附本仓库链接。
