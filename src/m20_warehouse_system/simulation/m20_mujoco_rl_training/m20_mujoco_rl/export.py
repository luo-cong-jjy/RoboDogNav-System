# ======================================================================
# export.py —— 检查点加载与 ONNX 导出工具（中文注释版）
# 作用：从训练生成的 .pt 检查点恢复 Actor-Critic 模型，
#       并导出"确定性策略"（仅 actor，不含随机噪声）为 ONNX 文件，
#       供 SDK 部署端推理使用
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Checkpoint and ONNX helpers."""

from __future__ import annotations

# 路径库：检查点/输出路径处理
from pathlib import Path

# PyTorch 基础
import torch
import torch.nn as nn

# 维度常量（观测/动作）
from .constants import ACTION_DIM, OBS_DIM
# 训练时的 Actor-Critic 模型定义
from .ppo import ActorCritic


# 确定性策略包装器：只保留 actor 网络，输出 tanh 限幅后的均值动作
class DeterministicPolicy(nn.Module):
    # 构造函数：从 ActorCritic 中提取 actor 与动作限幅
    def __init__(self, model: ActorCritic):
        super().__init__()
        # 复用 actor 网络（共享权重）
        self.actor = model.actor
        # 记录动作限幅
        self.action_limit = float(model.action_limit)

    # 前向：观测 -> 确定性动作（tanh 压缩后乘限幅）
    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return torch.tanh(self.actor(obs)) * self.action_limit


# 从 .pt 检查点加载并恢复 ActorCritic 模型（返回评估模式）
def load_actor_critic(checkpoint_path: str | Path, device: str = "cpu") -> ActorCritic:
    checkpoint_path = Path(checkpoint_path)
    # 加载检查点字典（映射到指定设备）
    ckpt = torch.load(checkpoint_path, map_location=device)
    # 读取隐藏层维度（默认 (512, 256, 128)）
    hidden_dims = tuple(ckpt.get("hidden_dims", (512, 256, 128)))
    # 读取 actor 观测维度（兼容旧字段 obs_dim）
    actor_obs_dim = int(ckpt.get("actor_obs_dim", ckpt.get("obs_dim", OBS_DIM)))
    # 读取 critic 观测维度
    critic_obs_dim = int(ckpt.get("critic_obs_dim", actor_obs_dim))
    # 用检查点中的超参数重建模型
    model = ActorCritic(
        actor_obs_dim=actor_obs_dim,
        critic_obs_dim=critic_obs_dim,
        action_dim=int(ckpt.get("action_dim", ACTION_DIM)),
        hidden_dims=hidden_dims,
        action_limit=float(ckpt.get("action_limit", 1.0)),
        log_std_min=float(ckpt.get("log_std_min", -4.0)),
        log_std_max=float(ckpt.get("log_std_max", 0.5)),
    ).to(device)
    # 载入模型权重
    model.load_state_dict(ckpt["model_state_dict"])
    # 把 log_std 裁剪回允许范围
    model.clamp_log_std_()
    # 切换到评估模式
    model.eval()
    return model


# 把检查点导出为 ONNX 文件（确定性策略，输入 obs -> 输出 actions）
def export_onnx(checkpoint_path: str | Path, output_path: str | Path, device: str = "cpu") -> Path:
    output_path = Path(output_path)
    # 确保输出目录存在
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # 加载 Actor-Critic 模型
    model = load_actor_critic(checkpoint_path, device=device)
    # 包装为确定性策略并切到评估模式
    policy = DeterministicPolicy(model).to(device).eval()
    # 构造全零哑输入（形状 1 x actor_obs_dim）
    dummy_obs = torch.zeros(1, model.actor_obs_dim, dtype=torch.float32, device=device)
    # 导出 ONNX：
    torch.onnx.export(
        policy,
        dummy_obs,
        output_path,
        export_params=True,        # 导出参数（权重固化）
        opset_version=11,          # ONNX opset 版本
        input_names=["obs"],       # 输入名
        output_names=["actions"],  # 输出名
        dynamic_axes={},           # 不使用动态维度（固定 batch=1）
        dynamo=False,              # 使用传统导出路径
    )
    return output_path
