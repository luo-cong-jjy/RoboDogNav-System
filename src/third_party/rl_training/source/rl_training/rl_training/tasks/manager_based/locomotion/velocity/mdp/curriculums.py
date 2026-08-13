# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Common functions that can be used to create curriculum for the learning environment.

The functions can be passed to the :class:`isaaclab.managers.CurriculumTermCfg` object to enable
the curriculum introduced by the function.
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.terrains import TerrainImporter

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def terrain_levels_vel(
    env: ManagerBasedRLEnv, env_ids: Sequence[int], asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """地形课程学习：根据机器人表现调整地形难度。

    直觉理解：
        如果机器人能从当前地形块走得足够远，就把它移动到更难的地形；
        如果走得太差，就降低难度。这样训练不会一开始就把机器人扔到最难地形。

    本函数还会把平均地形等级同步到 rewards.py 中的全局 ``gait_level``，
    让部分奖励项可以随训练难度逐步启用。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    terrain: TerrainImporter = env.scene.terrain
    command = env.command_manager.get_command("base_velocity")

    # 机器人从当前地形原点走出的平面距离。
    distance = torch.norm(asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1)
    # 走过半个地形块，认为当前难度可以升级。
    move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
    # 如果连“指令速度 × episode 时间”的一半都没走到，认为当前难度可能太高。
    move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
    move_down *= ~move_up

    terrain.update_env_origins(env_ids, move_up, move_down)

    mean_level = torch.mean(terrain.terrain_levels.float())
    # 局部导入可以避免 curriculums.py 和 rewards.py 在模块加载阶段互相导入导致循环依赖。
    from .rewards import update_gait_level_from_terrain_mean

    update_gait_level_from_terrain_mean(mean_level)
    return mean_level


def gait_level_curve(env: ManagerBasedRLEnv, env_ids: Sequence[int]) -> torch.Tensor:
    """把当前 gait_level 作为 curriculum 曲线记录到日志里。"""
    from .rewards import gait_level

    return torch.tensor(gait_level, device=env.device)


def command_levels_vel(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str,
    range_multiplier: Sequence[float] = (0.1, 1.0),
) -> None:
    """速度指令课程学习：训练越好，允许的速度指令范围越大。

    例如训练初期只给机器人很小的速度范围，让它先学会站稳和慢速移动；
    当速度跟踪奖励达到阈值后，再逐步扩大 ``lin_vel_x`` 和 ``lin_vel_y`` 范围。
    """
    base_velocity_ranges = env.command_manager.get_term("base_velocity").cfg.ranges
    # 第一次进入时记录原始速度范围，并设置初始/最终范围。
    if env.common_step_counter == 0:
        env._original_vel_x = torch.tensor(base_velocity_ranges.lin_vel_x, device=env.device)
        env._original_vel_y = torch.tensor(base_velocity_ranges.lin_vel_y, device=env.device)
        env._initial_vel_x = env._original_vel_x * range_multiplier[0]
        env._final_vel_x = env._original_vel_x * range_multiplier[1]
        env._initial_vel_y = env._original_vel_y * range_multiplier[0]
        env._final_vel_y = env._original_vel_y * range_multiplier[1]

        # 训练一开始先用较小指令范围。
        base_velocity_ranges.lin_vel_x = env._initial_vel_x.tolist()
        base_velocity_ranges.lin_vel_y = env._initial_vel_y.tolist()

    # 不需要每个 step 都更新课程。这里按 episode 节奏检查一次。
    if env.common_step_counter % env.max_episode_length == 0:
        episode_sums = env.reward_manager._episode_sums[reward_term_name]
        reward_term_cfg = env.reward_manager.get_term_cfg(reward_term_name)
        delta_command = torch.tensor([-0.1, 0.1], device=env.device)

        # 如果速度跟踪奖励超过最大可能奖励的 80%，说明当前速度范围已经学得不错，可以加难。
        if torch.mean(episode_sums[env_ids]) / env.max_episode_length_s > 0.8 * reward_term_cfg.weight:
            new_vel_x = torch.tensor(base_velocity_ranges.lin_vel_x, device=env.device) + delta_command
            new_vel_y = torch.tensor(base_velocity_ranges.lin_vel_y, device=env.device) + delta_command

            # 限制在最终范围内，避免越调越大。
            new_vel_x = torch.clamp(new_vel_x, min=env._final_vel_x[0], max=env._final_vel_x[1])
            new_vel_y = torch.clamp(new_vel_y, min=env._final_vel_y[0], max=env._final_vel_y[1])

            # 写回 command 配置，后续重新采样速度指令时就会使用新范围。
            base_velocity_ranges.lin_vel_x = new_vel_x.tolist()
            base_velocity_ranges.lin_vel_y = new_vel_y.tolist()

    return torch.tensor(base_velocity_ranges.lin_vel_x[1], device=env.device)
