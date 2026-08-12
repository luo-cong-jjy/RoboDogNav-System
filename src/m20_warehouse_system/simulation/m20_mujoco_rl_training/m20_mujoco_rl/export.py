"""Checkpoint and ONNX helpers."""

from __future__ import annotations

from pathlib import Path

import torch
import torch.nn as nn

from .constants import ACTION_DIM, OBS_DIM
from .ppo import ActorCritic


class DeterministicPolicy(nn.Module):
    def __init__(self, model: ActorCritic):
        super().__init__()
        self.actor = model.actor
        self.action_limit = float(model.action_limit)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return torch.tanh(self.actor(obs)) * self.action_limit


def load_actor_critic(checkpoint_path: str | Path, device: str = "cpu") -> ActorCritic:
    checkpoint_path = Path(checkpoint_path)
    ckpt = torch.load(checkpoint_path, map_location=device)
    hidden_dims = tuple(ckpt.get("hidden_dims", (512, 256, 128)))
    actor_obs_dim = int(ckpt.get("actor_obs_dim", ckpt.get("obs_dim", OBS_DIM)))
    critic_obs_dim = int(ckpt.get("critic_obs_dim", actor_obs_dim))
    model = ActorCritic(
        actor_obs_dim=actor_obs_dim,
        critic_obs_dim=critic_obs_dim,
        action_dim=int(ckpt.get("action_dim", ACTION_DIM)),
        hidden_dims=hidden_dims,
        action_limit=float(ckpt.get("action_limit", 1.0)),
        log_std_min=float(ckpt.get("log_std_min", -4.0)),
        log_std_max=float(ckpt.get("log_std_max", 0.5)),
    ).to(device)
    model.load_state_dict(ckpt["model_state_dict"])
    model.clamp_log_std_()
    model.eval()
    return model


def export_onnx(checkpoint_path: str | Path, output_path: str | Path, device: str = "cpu") -> Path:
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    model = load_actor_critic(checkpoint_path, device=device)
    policy = DeterministicPolicy(model).to(device).eval()
    dummy_obs = torch.zeros(1, model.actor_obs_dim, dtype=torch.float32, device=device)
    torch.onnx.export(
        policy,
        dummy_obs,
        output_path,
        export_params=True,
        opset_version=11,
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={},
        dynamo=False,
    )
    return output_path
