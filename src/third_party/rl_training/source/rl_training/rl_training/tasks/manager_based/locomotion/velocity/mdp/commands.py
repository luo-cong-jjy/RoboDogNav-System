# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Sequence

from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.utils import configclass

import rl_training.tasks.manager_based.locomotion.velocity.mdp as mdp

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class UniformThresholdVelocityCommand(mdp.UniformVelocityCommand):
    """带阈值的速度指令生成器。

    强化学习中的 command 可以理解成“这一个 episode/一段时间里希望机器人做什么”。
    对速度跟踪任务来说，command 通常是三维：

    * ``v_x``：机体系前后速度；
    * ``v_y``：机体系左右速度；
    * ``omega_z``：绕竖直方向的偏航角速度。

    这个类继承 Isaac Lab 的 ``UniformVelocityCommand``，会从给定范围内随机采样速度。
    额外增加的逻辑是：如果采样到的平面速度太小，就直接置 0，避免机器人学习到很多
    “似动非动”的模糊指令。
    """

    cfg: mdp.UniformThresholdVelocityCommandCfg
    """The configuration of the command generator."""

    def __init__(self, cfg: mdp.UniformThresholdVelocityCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        # 额外记录到 TensorBoard 的指标。
        # base_z：机体高度，方便观察机器人是否趴下或跳得过高。
        # knee_pos：膝关节相对默认姿态的偏移，方便观察腿部动作幅度。
        self.metrics["base_z"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["knee_pos"] = torch.zeros(self.num_envs, device=self.device)
        self._metric_step_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # 找到所有名字里包含 knee/Knee 的关节。M20 中对应四个膝关节。
        knee_joint_ids = self.robot.find_joints(".*[Kk]nee.*")[0]
        self._knee_joint_ids = torch.tensor(knee_joint_ids, dtype=torch.long, device=self.device)

    def reset(self, env_ids: Sequence[int] | None = None) -> dict[str, float]:
        """在指定环境 reset 时，清空统计量并重新采样速度指令。"""
        if env_ids is None:
            env_ids = slice(None)

        extras = {}
        for metric_name, metric_value in self.metrics.items():
            if metric_name in {"base_z", "knee_pos"}:
                # base_z/knee_pos 是逐步累加的，这里除以步数得到 episode 平均值。
                step_count = torch.clamp(self._metric_step_counter[env_ids].float(), min=1.0)
                extras[metric_name] = torch.mean(metric_value[env_ids] / step_count).item()
            else:
                extras[metric_name] = torch.mean(metric_value[env_ids]).item()
            metric_value[env_ids] = 0.0

        self._metric_step_counter[env_ids] = 0
        self.command_counter[env_ids] = 0
        self._resample(env_ids)
        return extras

    def _update_metrics(self):
        """每个仿真步更新 command 相关统计量。"""
        super()._update_metrics()

        # 1) base_z：根节点世界坐标 z 值，也就是机体高度。
        base_z = self.robot.data.root_pos_w[:, 2]

        # 2) knee_pos：膝关节偏离默认站立姿态的幅度。
        # 如果当前没有明显运动指令，并且机体速度也很小，就把膝关节偏移放大惩罚/统计，
        # 因为静止时更希望机器人保持默认站姿。
        cmd = torch.linalg.norm(self.vel_command_b, dim=1)
        body_vel = torch.linalg.norm(self.robot.data.root_lin_vel_b[:, :2], dim=1)

        if self._knee_joint_ids.numel() > 0:
            running_reward = torch.linalg.norm(
                self.robot.data.joint_pos[:, self._knee_joint_ids]
                - self.robot.data.default_joint_pos[:, self._knee_joint_ids],
                dim=1,
            )
        else:
            running_reward = torch.zeros(self.num_envs, device=self.device)

        knee_pos = torch.where(
            torch.logical_or(cmd > 0.1, body_vel > 0.5),
            running_reward,
            5.0 * running_reward,
        )

        self.metrics["base_z"] += base_z
        self.metrics["knee_pos"] += knee_pos
        self._metric_step_counter += 1

    def _resample_command(self, env_ids: Sequence[int]):
        super()._resample_command(env_ids)
        # 将很小的 xy 速度指令直接归零。
        # 例如采样到 0.03 m/s 这种速度，机器人几乎是在原地，不如明确告诉它“站住”。
        self.vel_command_b[env_ids, :2] *= (torch.norm(self.vel_command_b[env_ids, :2], dim=1) > 0.2).unsqueeze(1)


@configclass
class UniformThresholdVelocityCommandCfg(mdp.UniformVelocityCommandCfg):
    """UniformThresholdVelocityCommand 的配置类。

    Isaac Lab 的配置对象会通过 ``class_type`` 知道真正要实例化哪个 command 类。
    """

    class_type: type = UniformThresholdVelocityCommand


class DiscreteCommandController(CommandTerm):
    """
    Command generator that assigns discrete commands to environments.

    Commands are stored as a list of predefined integers.
    The controller maps these commands by their indices (e.g., index 0 -> 10, index 1 -> 20).

    初学者提示：
        这个类不是 M20 速度跟踪主线最重要的部分。它演示的是“离散指令”：
        指令不是连续速度，而是从一个整数列表里随机取值。当前 M20 rough 配置主要用的是
        ``UniformThresholdVelocityCommand``。
    """

    cfg: DiscreteCommandControllerCfg
    """Configuration for the command controller."""

    def __init__(self, cfg: DiscreteCommandControllerCfg, env: ManagerBasedEnv):
        """
        Initialize the command controller.

        Args:
            cfg: The configuration of the command controller.
            env: The environment object.
        """
        # Initialize the base class
        super().__init__(cfg, env)

        # Validate that available_commands is non-empty
        if not self.cfg.available_commands:
            raise ValueError("The available_commands list cannot be empty.")

        # Ensure all elements are integers
        if not all(isinstance(cmd, int) for cmd in self.cfg.available_commands):
            raise ValueError("All elements in available_commands must be integers.")

        # Store the available commands
        self.available_commands = self.cfg.available_commands

        # Create buffers to store the command
        # -- command buffer: stores discrete action indices for each environment
        self.command_buffer = torch.zeros(self.num_envs, dtype=torch.int32, device=self.device)

        # -- current_commands: stores a snapshot of the current commands (as integers)
        self.current_commands = [self.available_commands[0]] * self.num_envs  # Default to the first command

    def __str__(self) -> str:
        """返回命令生成器的简要字符串。

        Isaac Lab/RSL-RL 打印环境信息时可能会调用它，方便在日志中查看当前 command term。
        """
        return (
            "DiscreteCommandController:\n"
            f"\tNumber of environments: {self.num_envs}\n"
            f"\tAvailable commands: {self.available_commands}\n"
        )

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """返回当前离散命令张量。

        Shape 通常是 ``(num_envs,)`` 或逻辑上的 ``(num_envs, 1)``：
        每个并行环境对应一个离散整数命令。
        """
        return self.command_buffer

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        """更新日志指标。

        当前离散命令控制器没有额外指标要统计，所以留空。
        如果后续想看每个命令被采样的频率，可以在这里累计直方图。
        """
        pass

    def _resample_command(self, env_ids: Sequence[int]):
        """给指定 env 重新采样离散命令。"""
        # 先在 [0, len(available_commands)) 中采样索引。
        sampled_indices = torch.randint(
            len(self.available_commands), (len(env_ids),), dtype=torch.int32, device=self.device
        )
        # 再把索引映射到真正的整数命令值。
        sampled_commands = torch.tensor(
            [self.available_commands[idx.item()] for idx in sampled_indices], dtype=torch.int32, device=self.device
        )
        # 写入这些 env 的 command buffer。
        self.command_buffer[env_ids] = sampled_commands

    def _update_command(self):
        """把当前 command buffer 同步成 Python list 快照。"""
        self.current_commands = self.command_buffer.tolist()


@configclass
class DiscreteCommandControllerCfg(CommandTermCfg):
    """离散命令控制器配置。

    配置类只保存参数，真正的运行逻辑在 ``DiscreteCommandController`` 里。
    ``class_type`` 告诉 Isaac Lab：看到这个 cfg 时应该实例化哪个 CommandTerm。
    """

    class_type: type = DiscreteCommandController

    available_commands: list[int] = []
    """可采样的离散命令列表。

    例如 ``[10, 20, 30, 40, 50]`` 表示每次重采样时从这些整数里选一个。
    """
