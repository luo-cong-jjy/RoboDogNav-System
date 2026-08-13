# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


def joint_pos_rel_without_wheel(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    wheel_asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """返回相对默认姿态的关节位置，但把轮子关节位置清零。

    M20 是轮足机器人，总共有 16 个关节：
    12 个腿部姿态关节 + 4 个轮子连续旋转关节。腿部关节角能表示腿的姿态，
    但轮子角度会一直转，绝对角度本身没有稳定物理含义。

    因此策略仍然可以看到完整 joint_pos 向量的形状，但 wheel joint 的“位置”
    被置 0，避免网络学习到无意义的轮子累计角度。轮子的速度仍然会通过 joint_vel 提供。
    """
    # 从场景里取出机器人 Articulation，里面保存关节位置、默认位置、速度等张量。
    asset: Articulation = env.scene[asset_cfg.name]
    # Isaac Lab 的 joint_pos 是绝对关节角；减去 default_joint_pos 后得到“相对默认站姿”的偏移。
    joint_pos_rel = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    # 把 wheel joint 对应列清零。注意这里不是删除轮子维度，而是保留维度、清掉位置值。
    joint_pos_rel[:, wheel_asset_cfg.joint_ids] = 0
    return joint_pos_rel


def phase(env: ManagerBasedRLEnv, cycle_time: float) -> torch.Tensor:
    """生成周期相位观测 ``[sin(phase), cos(phase)]``。

    有些步态奖励或策略会希望知道当前处于一个周期的哪个位置。
    直接给 phase 角度会有 0 和 2π 的跳变，所以这里用 sin/cos 表示，更连续。
    """
    if not hasattr(env, "episode_length_buf") or env.episode_length_buf is None:
        env.episode_length_buf = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)
    phase = env.episode_length_buf[:, None] * env.step_dt / cycle_time
    phase_tensor = torch.cat([torch.sin(2 * torch.pi * phase), torch.cos(2 * torch.pi * phase)], dim=-1)
    return phase_tensor
