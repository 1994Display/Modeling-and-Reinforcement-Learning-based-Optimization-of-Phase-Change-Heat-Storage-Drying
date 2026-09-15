"""
极端工况测试：加载 v15 最优模型，测试各极端条件下的执行器响应
用法: python test_extreme.py
"""
import numpy as np
import torch
from train_ppo import PolicyNet

# 加载模型
print("=" * 70)
print("                 v22 模型 — 极端工况执行器响应测试")
print("=" * 70)

p = PolicyNet(5, 5, 128)
try:
    sd = torch.load("policy_best.pt", map_location="cpu", weights_only=True)
    p.load_state_dict(sd)
    p.eval()
    print("\n[OK] 模型已加载: policy_best.pt\n")
except Exception as ex:
    print(f"[FAIL] 加载模型失败: {ex}")
    exit(1)

# 归一化缩放因子 (5维观测): [PCM温度, 干燥仓温度, 干燥仓湿度, 水箱温度, 环境温度]
sc = np.array([100, 100, 80, 100, 40], dtype=np.float32)

def predict(raw_obs):
    """输入原始值(5维观测)，返回 [fan, damper, pump, discharge, heater]"""
    norm = np.clip(np.array(raw_obs, dtype=np.float32) / sc, 0, 1)
    with torch.no_grad():
        m, _ = p(torch.FloatTensor(norm.reshape(1, -1)))
    return m.numpy()[0]

def make_obs(T_pcm, T_amb, RH_amb, RH_dryer, T_water):
    """构造 5 维原始观测: [PCM温度, 干燥仓温度(T_in), 干燥仓湿度, 水箱温度, 环境温度常量25]"""
    T_in = T_pcm * 0.6 + T_amb * 0.4  # 干燥仓温度≈进口风温
    return [T_pcm, T_in, RH_dryer, T_water, 25.0]

def label(a):
    """为动作值添加趋势标记"""
    labels = []
    for val, name in zip(a, ["fan", "damper", "pump", "discharge", "heater"]):
        if name == "heater":
            labels.append(f"heater={'ON' if val >= 0.5 else 'OFF'}")
        elif val < 0.1:   labels.append(f"{name}=Low")
        elif val < 0.4:  labels.append(f"{name}=Mid")
        elif val < 0.7:  labels.append(f"{name}=High")
        else:            labels.append(f"{name}=MAX")
    return " | ".join(labels)


# ================================================================
# 1. 温度梯度扫描（固定其他变量为中等值）
# ================================================================
print("-" * 70)
print("【1】温度梯度扫描 (T_amb=19℃, RH_amb=60%, RH_dryer=50%, T_water=25℃)")
print("-" * 70)
print(f"{'T_pcm':>6}  {'fan':>8}  {'damper':>8}  {'pump':>8}  {'discharge':>8}  {'heater':>8}  | 解读")
print("-" * 80)
for T in [10, 20, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100]:
    obs = make_obs(T, 19, 60, 50, 25)
    a = predict(obs)
    print(f"{T:>5.0f}℃ {a[0]:>8.3f} {a[1]:>8.3f} {a[2]:>8.3f} {a[3]:>8.3f} {a[4]:>8.3f}  | {label(a)}")

# ================================================================
# 2. 环境湿度梯度扫描
# ================================================================
print()
print("-" * 70)
print("【2】环境湿度梯度扫描 (T_pcm=45℃, T_amb=19℃, RH_dryer=50%, T_water=25℃)")
print("-" * 70)
print(f"{'RH_amb':>6}  {'fan':>8}  {'damper':>8}  {'pump':>8}  {'discharge':>8}  {'heater':>8}  | 解读")
print("-" * 80)
for RH in [10, 20, 30, 40, 50, 60, 70, 80, 90, 95]:
    obs = make_obs(45, 19, RH, 50, 25)
    a = predict(obs)
    print(f"{RH:>5.0f}%  {a[0]:>8.3f} {a[1]:>8.3f} {a[2]:>8.3f} {a[3]:>8.3f} {a[4]:>8.3f}  | {label(a)}")

# ================================================================
# 3. 干燥室湿度梯度扫描
# ================================================================
print()
print("-" * 70)
print("【3】干燥室湿度梯度扫描 (T_pcm=45℃, T_amb=19℃, RH_amb=60%, T_water=25℃)")
print("-" * 70)
print(f"{'RH_dry':>6}  {'fan':>8}  {'damper':>8}  {'pump':>8}  {'discharge':>8}  {'heater':>8}  | 解读")
print("-" * 80)
for RH in [5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90]:
    obs = make_obs(45, 19, 60, RH, 25)
    a = predict(obs)
    grain = "->排粮" if RH < 15 else ""
    print(f"{RH:>5.0f}%  {a[0]:>8.3f} {a[1]:>8.3f} {a[2]:>8.3f} {a[3]:>8.3f} {a[4]:>8.3f}  | {label(a)} {grain}")

# ================================================================
# 4. 组合极端场景
# ================================================================
print()
print("-" * 70)
print("【4】组合极端场景")
print("-" * 70)

scenarios = [
    # (描述, T_pcm, T_amb, RH_amb, RH_dryer, T_water)
    ("[冷] 极冷干燥  (冬天早晨)   ", 10, 5, 30, 20, 5),
    ("[热] 极热高湿  (夏天暴雨后) ", 85, 38, 90, 80, 35),
    ("[危] 过热预警  (PCM逼近上限)", 100, 35, 60, 15, 30),
    ("[湿] 高湿低温  (阴冷雨天)   ", 25, 12, 85, 75, 15),
    ("[标] 标准工况  (春季晴天)   ", 50, 22, 55, 40, 28),
    ("[水] 水箱滚烫  (长期蓄热)   ", 60, 20, 60, 45, 85),
    ("[冻] 全冻状态  (系统刚启动) ", 2, 0, 40, 60, 0),
    ("[排] 粮食已干  (准备排粮)   ", 35, 18, 40, 10, 20),
    ("[干] 高温干燥  (沙漠气候)   ", 70, 40, 15, 25, 25),
    ("[潮] 湿热共存  (热带雨季)  ", 55, 32, 88, 70, 30),
]

print(f"{'场景':<28} {'fan':>8} {'damper':>8} {'pump':>8} {'discharge':>8} {'heater':>8} | 解读")
print("-" * 80)
for desc, Tp, Ta, RHa, RHd, Tw in scenarios:
    obs = make_obs(Tp, Ta, RHa, RHd, Tw)
    a = predict(obs)
    print(f"{desc:<28} {a[0]:>8.3f} {a[1]:>8.3f} {a[2]:>8.3f} {a[3]:>8.3f} {a[4]:>8.3f} | {label(a)}")

# ================================================================
# 5. 水箱温度对 damper 策略的影响
# ================================================================
print()
print("-" * 70)
print("【5】水箱温度对 damper 分配策略的影响 (T_pcm=50℃, 固定其他)")
print("-" * 70)
print(f"{'T_water':>7}  {'fan':>8}  {'damper':>8}  {'pump':>8}  {'discharge':>8}  {'heater':>8}  | 热风分配")
print("-" * 80)
for Tw in [15, 25, 35, 45, 55, 65, 75, 85]:
    obs = make_obs(50, 19, 60, 45, Tw)
    a = predict(obs)
    dryer_pct = (1 - a[1]) * 100
    tank_pct = a[1] * 100
    print(f"{Tw:>6.0f}℃ {a[0]:>8.3f} {a[1]:>8.3f} {a[2]:>8.3f} {a[3]:>8.3f} {a[4]:>8.3f}  | 干燥仓{dryer_pct:.0f}% / 水箱{tank_pct:.0f}%")

# ================================================================
# 总结
# ================================================================
print()
print("=" * 70)
print("                        测试完成")
print("=" * 70)
print("""
执行器含义:
  fan       [0-1]  风机转速        高=强力通风干燥/冷却
  damper    [0-1]  舵机角度        0=全送干燥仓, 1=全送水箱
  pump      [0-1]  水泵功率        加湿干燥室（通常应低）
  discharge [0-1]  排粮信号        ≥0.5 触发排粮 (湿度<15%时合理)
  heater    [0-1]  加热片开关      ≥0.5 开启加热, <0.5 关闭

预期合理行为:
  - PCM高温 → fan ↑  (散热+干燥)
  - PCM低温 → fan ↓  (避免无效冷却)
  - 干燥室高湿 → fan ↑  (加快除湿)
  - 干燥室低湿 → discharge ↑  (粮食已干, 排粮)
  - 水箱高温 → damper ↓  (减少向水箱供热)
  - 环境高湿 → 策略保守  (外部湿气进入)
  - PCM低温/高湿 → heater ON  (加热片补热加速干燥)
  - PCM过热 → heater OFF  (安全保护)
""")
