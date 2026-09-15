"""
PC 端 PPO 训练脚本 —— 干燥系统 RL 策略
输出: 训练好的 PyTorch 模型 + ONNX 模型
"""
import os
import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
from my_test import DryingSystemEnv

# ===================== 1. 策略网络（轻量级，适合 ESP32-S3 部署） =====================
class PolicyNet(nn.Module):
    """v33: 3 隐层 MLP + 噪声退火 — 更深网络自学温度依赖，噪声向零退火防"掷骰子"作弊"""
    def __init__(self, obs_dim=5, act_dim=5, hidden=128):
        super().__init__()
        self.aug_dim = obs_dim + 2
        self.net = nn.Sequential(
            nn.Linear(self.aug_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),            # v33: 第三层，更多容量学温度→fan 映射
            nn.ReLU(),
        )
        self.mean_ln = nn.LayerNorm(hidden)
        self.mean = nn.Linear(hidden, act_dim)
        self.log_std = nn.Parameter(torch.ones(act_dim) * -1.5)  # v33: std≈0.22，起步更紧

    def get_current_std(self):
        """返回当前标准差（用于噪声退火监控）"""
        return self.log_std.exp().clamp(0.01, 0.50)

    def forward(self, x):
        t = x[:, 0:1]
        x_aug = torch.cat([x, t * t, torch.sigmoid((t - 0.35) * 200.0)], dim=-1)
        h = self.net(x_aug)
        h_norm = self.mean_ln(h)
        mean = 0.05 + 0.90 * torch.sigmoid(self.mean(h_norm))
        std = self.get_current_std()                       # v33: 统一调用
        return mean, std

    def get_action(self, obs, deterministic=False):
        mean, std = self.forward(obs)
        if deterministic:
            return mean
        dist = Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        return action.clamp(0, 1), log_prob

    def evaluate(self, obs, action):
        mean, std = self.forward(obs)
        dist = Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob, entropy


class ValueNet(nn.Module):
    """v33: 3 隐层价值网络 — 匹配策略网络容量"""
    def __init__(self, obs_dim=5, hidden=128):
        super().__init__()
        self.aug_dim = obs_dim + 2
        self.net = nn.Sequential(
            nn.Linear(self.aug_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),            # v33: 第三层
            nn.ReLU(),
            nn.Linear(hidden, 1),
        )
    def forward(self, x):
        t = x[:, 0:1]
        x_aug = torch.cat([x, t * t, torch.sigmoid((t - 0.35) * 200.0)], dim=-1)
        return self.net(x_aug)




# ===================== 2. PPO 算法 =====================
class PPO:
    def __init__(self, obs_dim=5, act_dim=5, hidden=128,
                 lr=5e-5, gamma=0.99, lam=0.95, clip_eps=0.1,
                 epochs=2, batch_size=128, device="cpu"):
        self.device = device
        self.policy = PolicyNet(obs_dim, act_dim, hidden).to(device)
        self.value = ValueNet(obs_dim, hidden).to(device)
        self.optimizer_p = torch.optim.Adam(self.policy.parameters(), lr=lr)
        self.optimizer_v = torch.optim.Adam(self.value.parameters(), lr=lr)

        self.gamma = gamma
        self.lam = lam
        self.clip_eps = clip_eps
        self.epochs = epochs
        self.batch_size = batch_size

    def compute_gae(self, rewards, values, dones, next_value):
        """GAE 优势估计"""
        advantages = []
        gae = 0
        values = values + [next_value]
        for t in reversed(range(len(rewards))):
            delta = rewards[t] + self.gamma * values[t + 1] * (1 - dones[t]) - values[t]
            gae = delta + self.gamma * self.lam * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        return advantages

    def update(self, trajectories):
        obs = torch.FloatTensor(np.array(trajectories["obs"])).to(self.device)
        acts = torch.FloatTensor(np.array(trajectories["acts"])).to(self.device)
        rets = torch.FloatTensor(np.array(trajectories["rets"])).to(self.device)
        advs = torch.FloatTensor(np.array(trajectories["advs"])).to(self.device)

        # 标准化优势（先 clip 极端值防梯度爆炸，匹配奖励上限 500）
        advs = torch.clamp(advs, -20.0, 20.0)
        advs = (advs - advs.mean()) / (advs.std() + 1e-8)
        old_log_probs = trajectories["log_probs"]

        dataset_size = len(obs)
        for _ in range(self.epochs):
            indices = torch.randperm(dataset_size)
            for start in range(0, dataset_size, self.batch_size):
                idx = indices[start:start + self.batch_size]
                batch_obs = obs[idx]
                batch_acts = acts[idx]
                batch_rets = rets[idx]
                batch_advs = advs[idx]
                batch_old_logp = old_log_probs[idx]

                # Policy loss
                log_prob, entropy = self.policy.evaluate(batch_obs, batch_acts)
                ratio = (log_prob - batch_old_logp).exp()
                surr1 = ratio * batch_advs
                surr2 = torch.clamp(ratio, 1 - self.clip_eps, 1 + self.clip_eps) * batch_advs
                policy_loss = -torch.min(surr1, surr2).mean() - 0.5 * entropy.mean()  # v33: 降熵系数，鼓励收敛

                # Value loss
                value_pred = self.value(batch_obs).squeeze(-1)
                value_loss = nn.MSELoss()(value_pred, batch_rets)

                self.optimizer_p.zero_grad()
                self.optimizer_v.zero_grad()
                (policy_loss + value_loss).backward()
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 2.0)
                torch.nn.utils.clip_grad_norm_(self.value.parameters(), 2.0)
                # NaN 守卫：跳过含 NaN 的更新
                all_finite = True
                for p in self.policy.parameters():
                    if p.grad is not None and not torch.isfinite(p.grad).all():
                        all_finite = False
                        break
                if not all_finite:
                    continue
                self.optimizer_p.step()
                self.optimizer_v.step()


# ===================== 3. 训练循环 =====================
def train(total_steps=500_000, save_interval=50_000, log_interval=10_000):
    env = DryingSystemEnv()
    ppo = PPO(obs_dim=5, act_dim=5, hidden=128, lr=3e-5, device="cpu")  # v34.9: 观测5维
    device = ppo.device

    # LR 调度：3e-5 → 1e-5，更温和防止策略坍缩
    scheduler_p = torch.optim.lr_scheduler.CosineAnnealingLR(
        ppo.optimizer_p, T_max=total_steps // 2048, eta_min=1e-5)
    scheduler_v = torch.optim.lr_scheduler.CosineAnnealingLR(
        ppo.optimizer_v, T_max=total_steps // 2048, eta_min=1e-5)

    obs_dim = 5
    act_dim = 5
    trajectories = {"obs": [], "acts": [], "rews": [], "dones": [],
                    "values": [], "log_probs": []}

    obs, _ = env.reset()
    episode_reward = 0
    episode_steps = 0
    best_reward = -float("inf")
    steps_no_improve = 0
    reward_history = []

    # v33: 噪声退火参数 — log_std 从 -1.5(σ≈0.22) 线性退火到 -3.0(σ≈0.05)
    # v34.3: 放宽终点退火到 -2.5(σ≈0.08)，保留更多探索，防止策略坍缩成恒定输出
    log_std_start = -1.5
    log_std_end = -2.5

    for step in range(1, total_steps + 1):
        # v34.3: 每步退火 log_std（终点放宽）
        progress = step / total_steps
        log_std_target = log_std_start + (log_std_end - log_std_start) * progress
        with torch.no_grad():
            ppo.policy.log_std.copy_(torch.ones(act_dim) * log_std_target)
        obs_tensor = torch.FloatTensor(obs).unsqueeze(0).to(device)
        with torch.no_grad():
            action, log_prob = ppo.policy.get_action(obs_tensor)
            value = ppo.value(obs_tensor).squeeze()

        action_np = action.squeeze(0).cpu().numpy()
        next_obs, reward, terminated, truncated, _ = env.step(action_np)
        done = terminated or truncated

        trajectories["obs"].append(obs)
        trajectories["acts"].append(action_np)
        trajectories["rews"].append(reward)
        trajectories["dones"].append(1.0 if done else 0.0)
        trajectories["values"].append(value.item())
        trajectories["log_probs"].append(log_prob.item())

        episode_reward += reward
        episode_steps += 1

        # 每收集 2048 步更新一次
        if step % 2048 == 0 or done:
            with torch.no_grad():
                if done:
                    next_value = 0.0
                else:
                    next_value = ppo.value(
                        torch.FloatTensor(next_obs).unsqueeze(0).to(device)
                    ).squeeze().item()

            advs = ppo.compute_gae(trajectories["rews"], trajectories["values"],
                                    trajectories["dones"], next_value)
            trajectories["advs"] = advs
            trajectories["rets"] = [a + v for a, v in zip(advs, trajectories["values"])]
            trajectories["log_probs"] = torch.FloatTensor(trajectories["log_probs"])

            ppo.update(trajectories)
            scheduler_p.step()
            scheduler_v.step()

            # 清空缓存
            for k in trajectories:
                trajectories[k] = []

        if done:
            reward_history.append(episode_reward)
            obs, _ = env.reset()
            episode_reward = 0
            episode_steps = 0
        else:
            obs = next_obs

        # 日志 & 保存
        if step % log_interval == 0:
            avg_reward = np.mean(reward_history[-10:]) if reward_history else 0
            std = ppo.policy.get_current_std()[0].item()
            print(f"Step {step:7d} | Avg Reward (last 10): {avg_reward:8.2f} | σ={std:.4f} | Episodes: {len(reward_history)}")

        if step % save_interval == 0 and reward_history:
            avg = np.mean(reward_history[-10:])
            if avg > best_reward:
                best_reward = avg
                steps_no_improve = 0
                torch.save(ppo.policy.state_dict(), "policy_best.pt")
                print(f"  → 保存最优模型 (avg reward: {avg:.2f})")
            else:
                steps_no_improve += save_interval
                if steps_no_improve >= 80_000:  # v34: 早停阈值收紧，防策略坍缩后浪费算力
                    print(f"\n  [Early Stop] {steps_no_improve} steps no improve, peak {best_reward:.2f}")
                    break

    # 最终保存
    torch.save(ppo.policy.state_dict(), "policy_final.pt")
    print(f"\n训练完成！总步数: {total_steps}, 总 episode: {len(reward_history)}")
    return ppo, reward_history


# ===================== 4. 导出 ONNX（供 ESP32 部署） =====================
def export_onnx(policy, path="policy.onnx", obs_dim=5):
    """将策略网络导出为 ONNX 格式（只保留确定性推理部分）"""
    policy.eval()
    dummy_input = torch.randn(1, obs_dim)

    # 包装为纯推理 forward（含特征增强，只输出 mean）
    class InferenceNet(nn.Module):
        def __init__(self, policy_net):
            super().__init__()
            self.net = policy_net.net
            self.mean_ln = policy_net.mean_ln
            self.mean = policy_net.mean

        def forward(self, x):
            t = x[:, 0:1]
            x_aug = torch.cat([x, t * t, torch.sigmoid((t - 0.35) * 200.0)], dim=-1)
            h = self.net(x_aug)
            h_norm = self.mean_ln(h)
            return 0.05 + 0.90 * torch.sigmoid(self.mean(h_norm))   # 与训练一致

    inference_model = InferenceNet(policy)
    inference_model.eval()

    torch.onnx.export(
        inference_model,
        dummy_input,
        path,
        input_names=["obs"],
        output_names=["action"],
        opset_version=11,
    )
    print(f"ONNX 模型已导出至: {path}")
    return path


# ===================== 5. 主函数 =====================
if __name__ == "__main__":
    print("=" * 60)
    print("干燥系统 PPO 训练")
    print("v33: 3层MLP(128->128->128) + 噪声退火(sigma 0.22->0.05) + 极简奖励(仅干燥/能耗/安全), ~27K参数")
    print("=" * 60)

    # 先快速测试环境
    print("\n[1/3] 环境自检...")
    env = DryingSystemEnv()
    obs, _ = env.reset()
    for _ in range(5):
        action = env.action_space.sample()
        obs, rew, term, trunc, _ = env.step(action)
    print(f"  环境 OK, 动作空间: {env.action_space}, 观测空间: {env.observation_space}")
    env.close()

    # 训练
    print("\n[2/3] 开始训练...")
    ppo, history = train(total_steps=500_000, save_interval=10_000, log_interval=10_000)

    # 导出 ONNX
    print("\n[3/3] 导出 ONNX 模型...")
    export_onnx(ppo.policy, "policy.onnx")

    print("\n完成！输出文件:")
    print("  - policy_best.pt   : PyTorch 最优模型")
    print("  - policy_final.pt  : PyTorch 最终模型")
    print("  - policy.onnx      : ONNX 模型（供 ESP32 部署用）")
