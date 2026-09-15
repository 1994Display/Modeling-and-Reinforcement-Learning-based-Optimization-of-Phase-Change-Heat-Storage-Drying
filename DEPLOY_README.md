# 粮食干燥系统 AI 控制 —— 从训练到 ESP32-S3 部署

> **版本：v34.6**
> - 动作空间扩展为 5 维（新增加热片 `heater`）
> - 水泵语义修正：水箱热水 → 直接对干燥仓粮食干燥（仅热量交换、无湿度交互）
> - 水箱水量 20kg → 5kg（蓄热更快）
> - 为各执行器（风机/风门/水泵/加热片）新增引导奖励，提升控制灵敏度

## 整体流程

```
PC (训练)                              ESP32-S3 (推理)
┌──────────────────────┐              ┌──────────────────────────┐
│  my_test.py           │              │  esp32_controller.ino     │
│  (Gym 仿真环境)        │              │  (基于 zhinengzhizao.ino) │
│       ↓               │              │         ↑                │
│  train_ppo.py         │              │  policy_model.h          │
│  (PPO 训练)            │              │  (TFLite float32 模型)       │
│       ↓               │              │         ↑                │
│  policy.onnx          │              │  convert_model.py        │
│       ↓               │              │  (ONNX → TFLite → .h)   │
│  policy.tflite ───────┼──────────────┤                          │
└──────────────────────┘              └──────────────────────────┘
```

> **推荐：一条命令完成训练+转换**
> 使用 `train_v34.py` 整合脚本：训练 PPO → 导出 ONNX → 转换 TFLite → 生成 C 头文件。

## 动作 → 硬件映射

| 动作维度 | 范围 | 含义 | ESP32 引脚 | 执行器 | 映射 |
|---------|------|------|-----------|--------|------|
| `action[0]` fan | 0~1 | 风机转速 | GPIO8 | 维可思MOSFET | 0~1 → 0~100% PWM |
| `action[1]` damper | 0~1 | 风门角度 | GPIO7 | MG90S 舵机 | 0~1 → 0~90° (0°=全干燥, 45°=半储热, 90°=全储热) |
| `action[2]` pump | 0~1 | 水泵功率 | GPIO6 | 水泵 | 0~1 → 0~100% PWM |
| `action[3]` discharge | 0~1 | 排粮控制 | PCA9685 CH5 | MG90S 排粮舵机 | ≥0.5 触发排粮（由状态机独立控制） |
| `action[4]` heater | 0~1 | 加热片开关 | GPIO9 | 加热片 MOS驱动 | ≥0.5 → HIGH(加热), <0.5 → LOW(关闭) |

## 第一步：PC 端训练

```powershell
cd c:/Users/HTY/Desktop/gym
# 推荐：整合脚本（训练 + ONNX + TFLite + C头文件 一条命令完成）
& E:\anaconda\python.exe train_v34.py

# 或分步执行：
# & E:\anaconda\python.exe train_ppo.py       # 训练 PPO
# & E:\anaconda\python.exe convert_model.py   # 转换模型
```

- 训练 500,000 步（约 10-30 分钟）
- 输出文件：
  - `policy_best.pt`  → PyTorch 最优模型
  - `policy_final.pt` → PyTorch 最终模型
  - `policy.onnx`     → ONNX 中间格式

## 第二步：模型转换

```powershell
cd c:/Users/HTY/Desktop/gym
& E:\anaconda\python.exe convert_model.py
```

- 输出文件：
  - `policy.tflite`    → TFLite float32 模型
  - `policy_model.h`   → C 头文件

## 第三步：ESP32-S3 部署

1. 将 `policy_model.h` 复制到 `esp32_controller/` 目录：
   ```powershell
   Copy-Item c:\Users\HTY\Desktop\gym\policy_model.h c:\Users\HTY\Desktop\gym\esp32_controller\policy_model.h -Force
   ```
2. 打开 `esp32_controller.ino`，取消第 38 行注释：
   ```cpp
   #include "policy_model.h"   // 取消此行注释
   ```
3. Arduino IDE 选择 `ESP32S3 Dev Module`
4. 安装库：`TFT_eSPI`, `DHT sensor library`, `ESP32Servo`, `TensorFlowLite_ESP32`
5. 编译上传

> **注：** `.ino` 中的 `#include "policy_model.h"` 必须在编译前与训练生成的 `policy_model.h` 匹配（5 维输出）。重新训练后必须重新生成并复制该头文件。

## 运行模式

| 屏幕显示 | 模式 | 说明 |
|---------|------|------|
| **AI** | RL 模型推理 | 模型加载成功后自动使用，每 60 秒推理一次 |
| **AUTO** | AI 模型推理 | TFLite 策略网络，每 60 秒推理一次控制风机/风门/水泵/排粮/加热片 |
| **MANUAL** | 手动控制 | 触摸屏按钮手动操作各元件 |

## 模型参数

| 项目 | 值 |
|------|-----|
| 网络结构 | 5(输入)→7(增强)→128→128→128→LayerNorm→5 (ReLU) |
| 参数量 | ~35,000 |
| 模型大小 | ~140 KB (float32) |
| ESP32 推理时间 | ~5-12 ms |
| 推理间隔 | 60 秒 |
| 张量内存 | 128 KB |
| 动作维度 | 5 (fan, damper, pump, discharge, heater) |

## 加热片控制（v34 新增）

- **AI 决策**：AUTO 模式下加热片由模型第 5 维输出控制，不再强制常开
- **滞回防抖**：ESP32 端使用滞回机制防止加热片频繁启停（避免损伤 MOS/继电器）：
  ```
  当前关闭 → 需 heaterAct ≥ 0.55 才开启
  当前开启 → 需 heaterAct <  0.45 才关闭
  0.45~0.55 为死区，保持上次状态
  ```
- **安全保护**：PCM 温度 ≥ 90℃ 时强制关闭加热片（`HEATER_TEMP_CUTOFF_C`）

## 水泵控制（v34.6 语义修正）

- **物理语义**：水泵把水箱热水泵送到干燥仓，**直接对粮食干燥**（仅热量交换、无湿度交互，水密封）
- **蓄热-释热循环**：风门 damper 分热 → 水箱蓄热（水量 5kg，温升快）→ 达到 45℃ 阈值 → 开泵用热水直接干燥粮食 → 水箱降温需重新蓄热
- **水温门槛**：`WATER_TEMP_PUMP_THRESHOLD = 45℃`，低于此值开泵无效（空转）
- 正常干燥时 PCM 稳定在 54~60℃，加热片/水泵作为低温辅助热源

## 训练细节（v34.6）

奖励函数包含引导奖励，驱动模型学会精细控制各执行器：

| 引导项 | 逻辑 | 幅度 |
|--------|------|------|
| **风机引导** | 干燥室高湿(RH>35)却小风 → 惩罚；快干(RH<20)却大风 → 惩罚；PCM>65 小风 → 惩罚 | ±0.4 |
| **加热片引导** | PCM<52 开加热 → 强鼓励；PCM≥55 开加热 → 强惩罚；52~55 中性带下拉 | ±1.5 |
| **水泵引导** | 水箱有热(>45)+干燥室湿(>25)开泵 → 鼓励；粮已干(RH<15)开泵 → 惩罚 | ±1.0 |
| **能耗惩罚** | 风机 0.25 / 水泵 0.15 / 加热片 0.15 | — |
| **早停** | 80,000 步无改善即停止 | — |
| **噪声退火** | σ 从 0.22 线性退火到 0.08（保留更多探索） | — |

## 模型验证

训练/转换后应验证：

1. **维度**：模型输入 `[1,7]`，输出 `[1,5]`（fan, damper, pump, discharge, heater）
2. **一致性**：PyTorch ↔ TFLite 输出 Max diff < 1e-4
3. **灵敏度**：`test_extreme.py` 可查看各元件对温度/湿度的响应

```powershell
cd c:/Users/HTY/Desktop/gym
& E:\anaconda\python.exe test_extreme.py
```

## 传感器说明

当前使用 DHT11 测量环境温湿度。其他 5 个状态量（PCM温度、进/出口风温、干燥室湿度、水箱温度）通过 `updateStateEstimate()` 基于上一次动作和物理规律估算。

如需接入更多真实传感器，修改 `readSensors()` 和 `updateStateEstimate()` 函数即可。

## 调优建议

1. **训练效果不好** → 增大 `total_steps` 到 1,000,000，或调整 `lr`/`gamma`
2. **推理太慢** → 减小 `hidden` 到 32（需重新训练）
3. **模型太大** → 减小 `hidden` 到 16（需重新训练）
4. **状态估计不准** → 接入真实温度传感器，替换 `updateStateEstimate()` 中的估计逻辑
5. **加热片抖动** → 已内置滞回（0.45~0.55 死区），如需更宽可调整 `esp32_controller.ino` 中的滞回阈值
6. **水泵用不起来** → 水箱温度难达 45℃ 阈值时，可降低 `WATER_TEMP_PUMP_THRESHOLD`（`my_test.py` 和 `.ino` 同步修改）
7. **想重新训练** → 修改 `my_test.py` 奖励函数后运行 `train_v34.py`（自动重新生成全部 5 个文件）

## 观测归一化（v4 新增）

模型训练时使用归一化到 [0, 1] 的观测值。ESP32 端已自动处理：

```
obs_norm[i] = clamp(raw_obs[i] / scale[i], 0, 1)

scale = [100, 40, 80, 80, 100, 80, 80]
```

| 维度 | 含义 | 缩放因子 |
|------|------|---------|
| obs[0] | PCM 温度（估算） | /100 |
| obs[1] | 干燥仓温度（DHT11 实测） | /100 |
| obs[2] | 干燥仓湿度（DHT11 实测） | /80 |
| obs[3] | 水箱温度（DS18B20） | /100 |
| obs[4] | 环境温度（常量 25℃） | /40 |

> **注（v34.9）**：温湿度传感器（DHT11）安装在**干燥仓内部**，故删除了进口/出口风温，用干燥仓实测温湿度作为观测。观测空间由 7 维改为 **5 维**。

## 版本历史

| 版本 | 变更 |
|------|------|
| **v34.9**（当前） | 观测重构为 5 维：DHT11 在干燥仓内部，删进出口风温，新增干燥仓温湿度；风机/风门引导加强随温度响应 |
| **v34.8** | 风机/风门随温度灵敏；新增风门引导奖励 |
| **v34.6** | 水箱水量 20kg→5kg（蓄热更快，水泵可用）；完善引导奖励 |
| **v34.5** | 修正水泵物理语义（热水→直接干燥粮食）；新增风机/水泵引导奖励；能耗惩罚加强；噪声退火放宽 |
| **v34.2** | 加热片引导奖励加强（±1.5）；ESP32 加热片滞回防抖 |
| **v34** | 动作空间 4→5 维（新增加热片 heater）；加热片物理模型（温和辅助热源） |
| **v33** | 3 层 MLP + 噪声退火 + 极简奖励 |
