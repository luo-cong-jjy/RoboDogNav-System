# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils #引入了 Isaac Lab 封装的底层数学工具库（基于 PyTorch）。
from isaaclab.assets import Articulation, RigidObject#Articulation（关节体/多体系统），RigidObject（刚体）
from isaaclab.managers import ManagerTermBase #这是所有自定义 MDP 组件的基类。
from isaaclab.managers import RewardTermCfg as RewTerm #奖励项的配置模板。
from isaaclab.managers import SceneEntityCfg #用于在配置中动态指定场景中的对象。
from isaaclab.sensors import ContactSensor, RayCaster #ContactSensor（接触传感器），RayCaster（射线投射器）
from isaaclab.utils.math import quat_apply_inverse, yaw_quat#从 math_utils 中单独拎出来的高频函数：quat_apply_inverse（四元数逆向旋转），yaw_quat（偏航角四元数

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
# 这两行代码的作用是“仅供IDE代码提示使用，运行时不导入”：TYPE_CHECKING 在真正运行代码时永远是 False，
# 因此把导入语句放在 if TYPE_CHECKING: 下面，意味着程序在真正启动时会直接跳过这行导入。这种写法主要是为了
# 防止循环引用导致报错并加快程序启动速度，它告诉 Python：“这个类我只用来做类型提示（Type Hint），方便我写
# 代码时有提示，但运行时千万别去加载它！”


# reward 函数都是“逐 env 批量计算”的：输入 env，返回形状为 (num_envs,) 的 torch.Tensor。
# Isaac Lab 的 RewardManager 会再乘以配置里的 weight，并把所有 reward term 相加。
#
# 记住这个符号习惯：
#   正权重 + 正返回值  -> 鼓励该行为；
#   负权重 + 正返回值  -> 惩罚该行为。

# 全局课程标量，范围约为 [0, 1]，由 curriculums.py 根据平均地形等级更新。
# 部分 reward 会乘上 gait_level，表示训练前期先弱化复杂步态约束，后期再逐渐启用。
gait_level: float = 0.0

def update_gait_level_from_terrain_mean(terrain_level_mean: float | torch.Tensor) -> float:
    """根据平均地形等级更新全局 gait_level。

    Mapping rule:
    - mean <= 0.0 -> 0.0
    - 0.0 < mean < 3.0 -> 使用 exp 函数映射
    - mean == 3.0 -> 1.0
    - mean >= 3.0 -> 1.0

    这段代码就是一个“智能难度调节器”。
    它看着机器人走过的平均地形等级，如果等级太低，就不给高难度任务（返回 0）；如果等级极高，就拉满难度（返回 1）；
    如果在中间，就用一条先平缓、后陡峭的指数曲线，让难度慢慢升上去，防止机器人被突然增加的难度逼疯。
    """
    global gait_level

    mean_tensor = torch.as_tensor(terrain_level_mean, dtype=torch.float32)
    if mean_tensor.numel() == 0:
        mean_val = 0.0
    else:
        mean_val = float(torch.mean(mean_tensor).item())

    if math.isnan(mean_val) or math.isinf(mean_val):
        mean_val = 0.0

    if mean_val <= 0.0:
        gait_level = 0.0
    elif mean_val < 3.0:
        # exp 映射：mean=0 时接近 0，mean=3 时恰好为 1
        gait_level = math.exp(mean_val - 3.0)
    else:  # mean_val >= 3.0
        gait_level = 1.0

    return gait_level

def get_gait_level_tensor(env: ManagerBasedRLEnv) -> torch.Tensor:
    """把 Python 标量 gait_level 扩展成每个并行 env 一份的 tensor。"""
    return torch.full((env.num_envs,), gait_level, device=env.device)


def track_lin_vel_xy_exp(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """平面线速度跟踪奖励。

    奖励形式约为 ``exp(-error / std^2)``：
        * 速度误差越小，奖励越接近 1；
        * 速度误差越大，奖励越接近 0。

    这是 M20 速度跟踪任务最核心的正奖励之一。

    env: ManagerBasedRLEnv：传入整个环境对象，里面包含了所有机器人的状态和指令。
    std: float：标准差（Standard Deviation）。它是指数函数里的一个超参数，
    用来控制奖励衰减的“宽容度”。std 越大，对误差的容忍度越高。
    """
    # 取出机器人根状态。
    asset: RigidObject = env.scene[asset_cfg.name]
    # command 的前两维是期望 vx/vy；root_lin_vel_b 前两维是机体系实际 vx/vy。
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - asset.data.root_lin_vel_b[:, :2]),
        dim=1,
    )
    reward = torch.exp(-lin_vel_error / std**2)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def joint_torques_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """关节力矩平方惩罚。

    返回值越大表示用力越猛。配置里通常给负权重，用来鼓励省力、减少电机负担。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    reward = torch.sum(torch.square(asset.data.applied_torque[:, asset_cfg.joint_ids]), dim=1)
    return reward * get_gait_level_tensor(env)


def action_rate_l2(env: ManagerBasedRLEnv) -> torch.Tensor:
    """动作变化率惩罚。

    当前动作和上一帧动作差得越大，惩罚越大。这个项能抑制策略高频抖动。
    """
    reward = torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)
    return reward * get_gait_level_tensor(env)


def contact_forces(env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """接触力超阈值惩罚。

    只惩罚超过 threshold 的部分。用于避免机器人用过大的轮地/足地冲击力“硬砸”地面。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    violation = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] - threshold
    reward = torch.sum(violation.clip(min=0.0), dim=1)
    return reward * get_gait_level_tensor(env)


def track_ang_vel_z_exp(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """偏航角速度跟踪奖励。

    command 第 3 维是期望 yaw rate，root_ang_vel_b[:, 2] 是机体系实际 yaw rate。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_b[:, 2])
    reward = torch.exp(-ang_vel_error / std**2)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def track_lin_vel_xy_yaw_frame_exp(
    env, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) in the gravity aligned robot frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_apply_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
    )
    reward = torch.exp(-lin_vel_error / std**2)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def track_ang_vel_z_world_exp(
    env, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) in world frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_w[:, 2])
    reward = torch.exp(-ang_vel_error / std**2)
    reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def joint_power(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """关节功率惩罚项。

    功率近似为 ``|joint_vel * torque|``。配置中给负权重时，会鼓励低能耗动作。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    reward = torch.sum(
        torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids] * asset.data.applied_torque[:, asset_cfg.joint_ids]),
        dim=1,
    )
    return reward * get_gait_level_tensor(env)


def stand_still_without_cmd(
    env: ManagerBasedRLEnv,
    command_name: str,
    command_threshold: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """无速度指令时，惩罚关节偏离默认站姿。

    直觉：如果命令是站住，腿就不要乱动；否则策略可能学会原地抖动来骗部分奖励。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    diff_angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    reward = torch.sum(torch.abs(diff_angle), dim=1)
    reward *= torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) < command_threshold
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward

def joint_pos_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    stand_still_scale: float,
    velocity_threshold: float,
    command_threshold: float,
) -> torch.Tensor:
    """关节姿态偏离默认值的惩罚。

    如果机器人正在运动，按正常尺度惩罚；如果命令很小且机体几乎不动，
    用 ``stand_still_scale`` 放大惩罚，让站立姿态更稳。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = torch.linalg.norm(env.command_manager.get_command(command_name), dim=1)
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    running_reward = torch.linalg.norm(
        (asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]), dim=1
    )
    reward = torch.where(
        torch.logical_or(cmd > command_threshold, body_vel > velocity_threshold),
        running_reward,
        stand_still_scale * running_reward,
    )
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def wheel_vel_penalty(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    command_name: str,
    velocity_threshold: float,
    command_threshold: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """轮子空转/静止转动惩罚。

    * 机器人运动时：只惩罚离地轮子的转速，避免轮子悬空狂转；
    * 机器人应站住时：惩罚所有轮子转速，避免原地乱转。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = torch.linalg.norm(env.command_manager.get_command(command_name), dim=1)
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    joint_vel = torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids])
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_air = contact_sensor.compute_first_air(env.step_dt)[:, sensor_cfg.body_ids]
    running_reward = torch.sum(in_air * joint_vel, dim=1)
    standing_reward = torch.sum(joint_vel, dim=1)
    reward = torch.where(
        torch.logical_or(cmd > command_threshold, body_vel > velocity_threshold),
        running_reward,
        standing_reward,
    )
    return reward


class GaitReward(ManagerTermBase):
    """步态接触时序奖励。

    它会比较指定足端对的接触/离地时间，让策略更倾向某种步态，例如 trot。
    对 M20 当前 rough 配置来说，这类步态奖励大多权重为 0 或弱化；因为 M20 是轮足机器人，
    主要速度来源常常是轮子，腿更多负责支撑和姿态。
    """

    def __init__(self, cfg: RewTerm, env: ManagerBasedRLEnv):
        """初始化步态奖励项。

        Isaac Lab 中有两类 reward 写法：
        1. 普通函数：每次调用时直接从 env 里取数据计算；
        2. ``ManagerTermBase`` 子类：初始化时缓存一些不变的信息，后续调用更快。

        这里使用第 2 种，因为足端 body id、同步足对等信息不需要每个 step 都重新查。
        """
        super().__init__(cfg, env)
        # std 控制指数奖励的“宽容度”：std 越大，时序误差稍大也不会掉分太快。
        self.std: float = cfg.params["std"]
        # command_name 通常是 "base_velocity"，用于读取当前速度指令。
        self.command_name: str = cfg.params["command_name"]
        # max_err 用来裁剪接触/离地时间误差，避免异常大误差支配奖励。
        self.max_err: float = cfg.params["max_err"]
        # velocity_threshold：机器人实际速度超过这个值时，认为它处于运动状态。
        self.velocity_threshold: float = cfg.params["velocity_threshold"]
        # command_threshold：速度指令超过这个值时，才强制步态约束。
        self.command_threshold: float = cfg.params["command_threshold"]
        # ContactSensor 保存每个足端/轮端的接触时间、离地时间和接触力。
        self.contact_sensor: ContactSensor = env.scene.sensors[cfg.params["sensor_cfg"].name]
        # asset 是机器人本体，后续用它读取机身速度等状态。
        self.asset: Articulation = env.scene[cfg.params["asset_cfg"].name]
        # synced_feet_pair_names 形如 (("fl_wheel", "hr_wheel"), ("fr_wheel", "hl_wheel"))。
        # 对四足 trot 来说，左前+右后同步，右前+左后同步。
        synced_feet_pair_names = cfg.params["synced_feet_pair_names"]
        # 当前实现只支持“两组同步足对”的步态奖励；更多足端组合需要另写逻辑。
        if (
            len(synced_feet_pair_names) != 2
            or len(synced_feet_pair_names[0]) != 2
            or len(synced_feet_pair_names[1]) != 2
        ):
            raise ValueError("This reward only supports gaits with two pairs of synchronized feet, like trotting.")
        # find_bodies 会把正则/名字转成 body id；reward 运行时用 id 索引张量更快。
        synced_feet_pair_0 = self.contact_sensor.find_bodies(synced_feet_pair_names[0])[0]
        synced_feet_pair_1 = self.contact_sensor.find_bodies(synced_feet_pair_names[1])[0]
        # 缓存两组同步足对，后面 __call__ 每个 step 直接使用。
        self.synced_feet_pairs = [synced_feet_pair_0, synced_feet_pair_1]

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        std: float,
        command_name: str,
        max_err: float,
        velocity_threshold: float,
        command_threshold: float,
        synced_feet_pair_names,
        asset_cfg: SceneEntityCfg,
        sensor_cfg: SceneEntityCfg,
    ) -> torch.Tensor:
        """计算步态接触时序奖励。

        奖励由 6 个子项相乘：
        * 2 个同步项：同一同步足对的接触/离地时间应接近；
        * 4 个异步项：不同同步足对之间应一只接触、一只离地，形成交替支撑。

        相乘的好处是：只要某个关键关系很差，总奖励就会明显下降。
        """
        # 同步足对 0：例如 fl 与 hr，希望两者 air/contact 时间接近。
        sync_reward_0 = self._sync_reward_func(self.synced_feet_pairs[0][0], self.synced_feet_pairs[0][1])
        # 同步足对 1：例如 fr 与 hl，希望两者 air/contact 时间接近。
        sync_reward_1 = self._sync_reward_func(self.synced_feet_pairs[1][0], self.synced_feet_pairs[1][1])
        # 两组同步关系都好，sync_reward 才高。
        sync_reward = sync_reward_0 * sync_reward_1
        # 异步关系：一组足端处于接触时，另一组足端应更像处于离地，形成交替。
        async_reward_0 = self._async_reward_func(self.synced_feet_pairs[0][0], self.synced_feet_pairs[1][0])
        async_reward_1 = self._async_reward_func(self.synced_feet_pairs[0][1], self.synced_feet_pairs[1][1])
        async_reward_2 = self._async_reward_func(self.synced_feet_pairs[0][0], self.synced_feet_pairs[1][1])
        async_reward_3 = self._async_reward_func(self.synced_feet_pairs[1][0], self.synced_feet_pairs[0][1])
        # 四个异步关系同时满足时，async_reward 才高。
        async_reward = async_reward_0 * async_reward_1 * async_reward_2 * async_reward_3
        # cmd 是当前速度指令的模长；指令很小时不强迫机器人走步态。
        cmd = torch.linalg.norm(env.command_manager.get_command(self.command_name), dim=1)
        # body_vel 是机器人实际平面速度；即使指令小，如果身体还在动，也可以继续约束步态。
        body_vel = torch.linalg.norm(self.asset.data.root_com_lin_vel_b[:, :2], dim=1)
        # 只有“应该动”或“已经在动”时启用步态奖励，否则返回 0。
        reward = torch.where(
            torch.logical_or(cmd > self.command_threshold, body_vel > self.velocity_threshold),
            sync_reward * async_reward,
            0.0,
        )
        # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
        return reward

    """
    Helper functions.
    """

    def _sync_reward_func(self, foot_0: int, foot_1: int) -> torch.Tensor:
        """奖励两只足/轮同步。

        同步的含义不是“都接触地面”，而是二者的当前离地时间、当前接触时间相近。
        """
        # current_air_time：当前连续离地持续了多久。
        air_time = self.contact_sensor.data.current_air_time
        # current_contact_time：当前连续接触持续了多久。
        contact_time = self.contact_sensor.data.current_contact_time
        # 两只足的离地时间差，平方后裁剪，避免异常值过大。
        se_air = torch.clip(torch.square(air_time[:, foot_0] - air_time[:, foot_1]), max=self.max_err**2)
        # 两只足的接触时间差，平方后裁剪。
        se_contact = torch.clip(torch.square(contact_time[:, foot_0] - contact_time[:, foot_1]), max=self.max_err**2)
        # 指数形式把误差映射到 (0, 1]，误差越小越接近 1。
        return torch.exp(-(se_air + se_contact) / self.std)

    def _async_reward_func(self, foot_0: int, foot_1: int) -> torch.Tensor:
        """奖励两只足/轮反同步。

        反同步的直觉：foot_0 离地时，foot_1 更应接触；foot_0 接触时，foot_1 更应离地。
        """
        air_time = self.contact_sensor.data.current_air_time
        contact_time = self.contact_sensor.data.current_contact_time
        # foot_0 的离地时间应接近 foot_1 的接触时间。
        se_act_0 = torch.clip(torch.square(air_time[:, foot_0] - contact_time[:, foot_1]), max=self.max_err**2)
        # foot_0 的接触时间应接近 foot_1 的离地时间。
        se_act_1 = torch.clip(torch.square(contact_time[:, foot_0] - air_time[:, foot_1]), max=self.max_err**2)
        return torch.exp(-(se_act_0 + se_act_1) / self.std)


def joint_mirror(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, mirror_joints: list[list[str]]) -> torch.Tensor:
    """镜像关节位置惩罚。

    用于鼓励对角腿/左右腿形成对称姿态，避免策略学出很别扭的“偏腿”运动。
    返回值越大表示越不对称；配置中通常给负权重。
    """
    # 取出机器人资产；asset_cfg 里记录了要检查哪些关节。
    asset: Articulation = env.scene[asset_cfg.name]
    # 第一次调用时把正则表达式形式的 joint name 转成 joint id，并缓存到 env。
    if not hasattr(env, "joint_mirror_joints_cache") or env.joint_mirror_joints_cache is None:
        env.joint_mirror_joints_cache = [
            [asset.find_joints(joint_name) for joint_name in joint_pair] for joint_pair in mirror_joints
        ]
    # 每个并行环境一个 reward 值。
    reward = torch.zeros(env.num_envs, device=env.device)
    # 遍历每一组镜像关节，例如 fl_hipy 和 hr_hipy。
    for joint_pair in env.joint_mirror_joints_cache:
        # 两组关节位置差的平方和，越大表示越不对称。
        diff = torch.sum(
            torch.square(asset.data.joint_pos[:, joint_pair[0][0]] - asset.data.joint_pos[:, joint_pair[1][0]]),
            dim=-1,
        )
        reward += diff
    # 对镜像组数量取平均，避免组越多 reward 数值越大。
    reward *= 1 / len(mirror_joints) if len(mirror_joints) > 0 else 0
    # 姿态门控：机器人翻倒时减少该项影响，避免倒地状态下的异常姿态主导训练。
    reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward * get_gait_level_tensor(env)


def action_mirror(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, mirror_joints: list[list[str]]) -> torch.Tensor:
    """镜像动作惩罚。

    与 ``joint_mirror`` 类似，但比较的是策略输出动作，而不是实际关节位置。
    这能直接约束 actor 输出更对称。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    # 缓存 joint id，避免每个 step 都按名字查找。
    if not hasattr(env, "action_mirror_joints_cache") or env.action_mirror_joints_cache is None:
        env.action_mirror_joints_cache = [
            [asset.find_joints(joint_name) for joint_name in joint_pair] for joint_pair in mirror_joints
        ]
    reward = torch.zeros(env.num_envs, device=env.device)
    # 遍历每组镜像关节，比较两侧动作幅值。
    for joint_pair in env.action_mirror_joints_cache:
        # 使用 abs 是因为左右/对角关节可能符号方向相反，但幅值应接近。
        diff = torch.sum(
            torch.square(
                torch.abs(env.action_manager.action[:, joint_pair[0][0]])
                - torch.abs(env.action_manager.action[:, joint_pair[1][0]])
            ),
            dim=-1,
        )
        reward += diff
    # 取平均，保持奖励尺度稳定。
    reward *= 1 / len(mirror_joints) if len(mirror_joints) > 0 else 0
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def action_sync(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, joint_groups: list[list[str]]) -> torch.Tensor:
    """同组动作同步惩罚。

    joint_groups 可以把四条腿的同类关节放到一组，例如四个 hipx。
    该函数惩罚同组动作幅值的方差，鼓励它们变化趋势一致。
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # 第一次调用时缓存每组 joint id。
    if not hasattr(env, "action_sync_joint_cache") or env.action_sync_joint_cache is None:
        env.action_sync_joint_cache = [
            [asset.find_joints(joint_name) for joint_name in joint_group] for joint_group in joint_groups
        ]

    reward = torch.zeros(env.num_envs, device=env.device)
    # 遍历每个“应该同步”的关节组。
    for joint_group in env.action_sync_joint_cache:
        if len(joint_group) < 2:
            # 少于两个关节没有比较意义，直接跳过。
            continue

        # 取同组所有关节的动作幅值，形状约为 (num_envs, num_joints_in_group)。
        actions = torch.stack(
            [torch.abs(env.action_manager.action[:, joint[0]]) for joint in joint_group], dim=1
        )

        # 每个环境内，该关节组动作幅值的平均值。
        mean_actions = torch.mean(actions, dim=1, keepdim=True)

        # 方差越大表示同组关节动作越不同步。
        variance = torch.mean(torch.square(actions - mean_actions), dim=1)

        # reward 作为惩罚项累加，配置中给负权重。
        reward += variance.squeeze()
    # 对组数取平均，避免组数改变后尺度变化。
    reward *= 1 / len(joint_groups) if len(joint_groups) > 0 else 0
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


# def feet_air_time(
#     env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
# ) -> torch.Tensor:
#     """Reward long steps taken by the feet using L2-kernel.

#     This function rewards the agent for taking steps that are longer than a threshold. This helps ensure
#     that the robot lifts its feet off the ground and takes steps. The reward is computed as the sum of
#     the time for which the feet are in the air.

#     If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
#     """
#     # extract the used quantities (to enable type-hinting)
#     contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
#     # compute the reward
#     first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
#     last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
#     reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
#     # no reward for zero command
#     reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
#     # print(torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1), "command norm")
#     reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    # return reward

# def feet_air_time(
#     env: ManagerBasedRLEnv,
#     asset_cfg: SceneEntityCfg,
#     sensor_cfg: SceneEntityCfg,
#     mode_time: float,
#     velocity_threshold: float,
# ) -> torch.Tensor:
#     """Reward longer feet air and contact time."""
#     # extract the used quantities (to enable type-hinting)
#     contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
#     asset: Articulation = env.scene[asset_cfg.name]
#     if contact_sensor.cfg.track_air_time is False:
#         raise RuntimeError("Activate ContactSensor's track_air_time!")
#     # compute the reward
#     current_air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
#     current_contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]

#     t_max = torch.max(current_air_time, current_contact_time)
#     t_min = torch.clip(t_max, max=mode_time)
#     stance_cmd_reward = torch.clip(current_contact_time - current_air_time, -mode_time, mode_time)
#     cmd = torch.norm(env.command_manager.get_command("base_velocity"), dim=1).unsqueeze(dim=1).expand(-1, 4)
#     body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1).unsqueeze(dim=1).expand(-1, 4)
#     reward = torch.where(
#         torch.logical_or(cmd > 0.0, body_vel > velocity_threshold),
#         torch.where(t_max < mode_time, t_min, 0),
#         stance_cmd_reward,
#     )
#     return torch.sum(reward, dim=1)


def feet_air_time_positive_biped(env, command_name: str, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """双足机器人单脚支撑奖励。

    这个函数更偏双足机器人：希望“同一时刻只有一只脚接触地面”，并奖励合理的离地/接触持续时间。
    M20 当前配置一般不依赖它，但保留在通用 locomotion reward 库里。

    如果速度指令很小，说明机器人应站立，奖励置 0。
    """
    # 从场景里取接触传感器。
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # 当前离地持续时间，按指定 body_ids 取足端/轮端。
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    # 当前接触持续时间。
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    # contact_time > 0 表示当前处于接触状态。
    in_contact = contact_time > 0.0
    # 如果在接触，用 contact_time；如果在空中，用 air_time。
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    # 双足任务中，希望恰好一只脚接触地面。
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    # 如果不是单脚支撑，该 env 奖励为 0；如果是，取两脚中较小的模式持续时间。
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    # 持续时间奖励最多给到 threshold，避免无限鼓励“拖着不换脚”。
    reward = torch.clamp(reward, max=threshold)
    # 无平面移动指令时不奖励迈步。
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_air_time_variance_penalty(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """足端离地/接触时间方差惩罚。

    四条腿/四个轮端的节奏差异太大，说明步态不均衡。这个函数惩罚各足端 air/contact
    时间的方差，让运动节奏更一致。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # 上一次离地持续时间，裁剪到 0.5s 以内，防止极端值影响方差。
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    # 上一次接触持续时间，同样裁剪。
    last_contact_time = contact_sensor.data.last_contact_time[:, sensor_cfg.body_ids]
    # 每个 env 内，对足端维度求方差；方差越大，惩罚越大。
    reward = torch.var(torch.clip(last_air_time, max=0.5), dim=1) + torch.var(
        torch.clip(last_contact_time, max=0.5), dim=1)
    # print(last_air_time, "last air time")
    # print(last_contact_time, "last contact time")
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward




def feet_contact(
    env: ManagerBasedRLEnv, command_name: str, expect_contact_num: int, sensor_cfg: SceneEntityCfg
) -> torch.Tensor:
    """接触数量惩罚。

    名字里叫 reward，但实际返回 ``contact_num != expect_contact_num``，通常配负权重使用。
    它表达的是：运动时希望接触足/轮数量接近期望值。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute_first_contact 只在“刚接触”的那个控制步为 True。
    contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    # 当前 step 中首次接触的足端数量。
    contact_num = torch.sum(contact, dim=1)
    # 数量不等于期望值则记 1，否则记 0。
    reward = (contact_num != expect_contact_num).float()
    # 只有命令较大时才检查接触数量；站立时不强制这种步态模式。
    reward *= torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) > 0.5
    # 姿态门控：翻倒时弱化该项。
    reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_contact_without_cmd(env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """无移动指令时的足端接触奖励。

    当 command 很小，机器人应稳定站住。此时足端/轮端发生接触是合理的，因此给正奖励。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # 首次接触矩阵，形状约为 (num_envs, num_feet)。
    contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    # print(contact, "contact")
    # 每个环境把足端接触数相加。
    reward = torch.sum(contact, dim=-1).float()
    # print(reward, "reward after sum")
    # 只有命令小于 0.5 时生效，也就是“应该站住”的场景。
    reward *= torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) < 0.5
    # print(env.command_manager.get_command(command_name), "env.command_manager.get_command(command_name)")
    # print(reward, "reward after multiply")
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_stumble(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """足端绊碰/撞竖直面惩罚。

    如果水平接触力远大于竖直接触力，通常表示足端撞到了墙、台阶侧面等竖直障碍。
    对爬台阶来说，这个项可以帮助减少“硬怼台阶边缘”的策略。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # 竖直方向接触力大小。
    forces_z = torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2])
    # 水平方向接触力大小。
    forces_xy = torch.linalg.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :2], dim=2)
    # 如果水平力超过竖直力 4 倍，认为存在绊碰；任意一个足端触发则该 env 记 1。
    reward = torch.any(forces_xy > 4 * forces_z, dim=1).float()
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_distance_y_exp(
    env: ManagerBasedRLEnv, stance_width: float, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """足端横向间距奖励。

    将足端位置转换到机体系后，希望左右足端 y 坐标接近期望站宽 ``stance_width``。
    指数形式返回值越接近 1，表示步宽越接近期望。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # 世界系足端位置减去机身位置，得到相对机身的世界系向量。
    cur_footsteps_translated = asset.data.body_link_pos_w[:, asset_cfg.body_ids, :] - asset.data.root_link_pos_w[
        :, :
    ].unsqueeze(1)
    # 需要检查的足端数量。
    n_feet = len(asset_cfg.body_ids)
    # 准备保存机体系下足端坐标。
    footsteps_in_body_frame = torch.zeros(env.num_envs, n_feet, 3, device=env.device)
    for i in range(n_feet):
        # 用根链接四元数的逆旋转，把世界系相对向量转到机体系。
        footsteps_in_body_frame[:, i, :] = math_utils.quat_apply(
            math_utils.quat_conjugate(asset.data.root_link_quat_w), cur_footsteps_translated[:, i, :]
        )
    # 偶数/奇数足端约定为左右两侧，分别给 +1/-1 的横向符号。
    side_sign = torch.tensor(
        [1.0 if i % 2 == 0 else -1.0 for i in range(n_feet)],
        device=env.device,
    )
    # 每个 env 一份期望站宽。
    stance_width_tensor = stance_width * torch.ones([env.num_envs, 1], device=env.device)
    # 期望 y 坐标：左侧 +stance_width/2，右侧 -stance_width/2。
    desired_ys = stance_width_tensor / 2 * side_sign.unsqueeze(0)
    # 计算当前 y 坐标和期望 y 坐标的平方误差。
    stance_diff = torch.square(desired_ys - footsteps_in_body_frame[:, :, 1])
    # 误差越小，奖励越接近 1。
    reward = torch.exp(-torch.sum(stance_diff, dim=1) / (std**2))
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_distance_xy_exp(
    env: ManagerBasedRLEnv,
    stance_width: float,
    stance_length: float,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """足端前后/左右站位奖励。

    与 ``feet_distance_y_exp`` 相比，这里同时约束 x 和 y：
    * 前腿 x 约为 +stance_length/2；
    * 后腿 x 约为 -stance_length/2；
    * 左右腿 y 约为 +/- stance_width/2。
    """
    asset: RigidObject = env.scene[asset_cfg.name]

    # 世界系下足端位置相对机身位置。
    cur_footsteps_translated = asset.data.body_link_pos_w[:, asset_cfg.body_ids, :] - asset.data.root_link_pos_w[
        :, :
    ].unsqueeze(1)

    # 转到机体系，方便用固定的前后/左右坐标检查站位。
    footsteps_in_body_frame = torch.zeros(env.num_envs, 4, 3, device=env.device)
    for i in range(4):
        footsteps_in_body_frame[:, i, :] = math_utils.quat_apply(
            math_utils.quat_conjugate(asset.data.root_link_quat_w), cur_footsteps_translated[:, i, :]
        )

    # 每个 env 一份期望站长/站宽。
    stance_width_tensor = stance_width * torch.ones([env.num_envs, 1], device=env.device)
    stance_length_tensor = stance_length * torch.ones([env.num_envs, 1], device=env.device)

    # 期望 x：前两条腿在前，后两条腿在后。
    desired_xs = torch.cat(
        [stance_length_tensor / 2, stance_length_tensor / 2, -stance_length_tensor / 2, -stance_length_tensor / 2],
        dim=1,
    )
    # 期望 y：左右交替。
    desired_ys = torch.cat(
        [stance_width_tensor / 2, -stance_width_tensor / 2, stance_width_tensor / 2, -stance_width_tensor / 2], dim=1
    )

    # 计算前后和左右方向误差。
    stance_diff_x = torch.square(desired_xs - footsteps_in_body_frame[:, :, 0])
    stance_diff_y = torch.square(desired_ys - footsteps_in_body_frame[:, :, 1])

    # 合并误差，并用指数函数转换为奖励。
    stance_diff = stance_diff_x + stance_diff_y
    reward = torch.exp(-torch.sum(stance_diff, dim=1) / std**2)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_height(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    target_height: float,
    tanh_mult: float,
) -> torch.Tensor:
    """足端高度误差惩罚/奖励项。

    注意：这里返回的是 ``(当前高度 - 目标高度)^2`` 的和，所以如果配置为负权重，
    它就是“希望足端高度接近 target_height”的惩罚项。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # 直接使用世界系 z 坐标计算足端高度误差。
    foot_z_target_error = torch.square(asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - target_height)
    # foot_velocity_tanh = torch.tanh(
    #     tanh_mult * torch.linalg.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2)
    # )
    # reward = torch.sum(foot_z_target_error * foot_velocity_tanh, dim=1)
    reward = torch.sum(foot_z_target_error, dim=1)
    # print(foot_z_target_error, "foot_z_target_error")
    # 只有命令足够大时才启用，否则站立时不要求足端按摆动高度运动。
    reward *= torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) > 0.2
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_height_body(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    target_height: float,
    tanh_mult: float,
) -> torch.Tensor:
    """机体系足端高度误差项。

    与 ``feet_height`` 不同，这里先把足端位置和速度转到机体系，再检查 z 方向高度。
    机体系高度更适合在坡地/机身有姿态变化时表达“相对身体抬脚多少”。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # 世界系足端位置减去机身位置，得到足端相对机身的位置向量。
    cur_footpos_translated = asset.data.body_pos_w[:, asset_cfg.body_ids, :] - asset.data.root_pos_w[:, :].unsqueeze(1)
    # 准备保存机体系足端位置。
    footpos_in_body_frame = torch.zeros(env.num_envs, len(asset_cfg.body_ids), 3, device=env.device)
    # 世界系足端速度减去机身速度，得到足端相对机身速度。
    cur_footvel_translated = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :] - asset.data.root_lin_vel_w[
        :, :
    ].unsqueeze(1)
    # 准备保存机体系足端速度。
    footvel_in_body_frame = torch.zeros(env.num_envs, len(asset_cfg.body_ids), 3, device=env.device)
    for i in range(len(asset_cfg.body_ids)):
        # 位置向量转到机体系。
        footpos_in_body_frame[:, i, :] = math_utils.quat_apply_inverse(
            asset.data.root_quat_w, cur_footpos_translated[:, i, :]
        )
        # 速度向量转到机体系。
        footvel_in_body_frame[:, i, :] = math_utils.quat_apply_inverse(
            asset.data.root_quat_w, cur_footvel_translated[:, i, :]
        )
    # 机体系 z 坐标与目标高度的平方误差。
    foot_z_target_error = torch.square(footpos_in_body_frame[:, :, 2] - target_height).view(env.num_envs, -1)
    # 横向速度越大，tanh 权重越接近 1；站着不动的脚不太参与高度项。
    foot_velocity_tanh = torch.tanh(tanh_mult * torch.norm(footvel_in_body_frame[:, :, :2], dim=2))
    # 将高度误差按足端运动速度加权后求和。
    reward = torch.sum(foot_z_target_error * foot_velocity_tanh, dim=1)
    # 只有存在移动/转向指令时启用。
    reward *= torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) > 0.1
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def feet_slide(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """足端/轮端接触时横向滑动惩罚。

    只有接触地面时才惩罚足端横向速度。这样可以鼓励“支撑相不打滑”，但不会惩罚摆动相。
    对轮足 M20 来说，如果作用在 wheel body 上，它表达的是轮端接触时不要发生不合理侧滑。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # contacts 表示当前/最近历史中该 body 是否有明显接触力。
    contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
    asset: RigidObject = env.scene[asset_cfg.name]

    # feet_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    # reward = torch.sum(feet_vel.norm(dim=-1) * contacts, dim=1)

    # 足端速度减去机身速度，得到相对机身的世界系速度。
    cur_footvel_translated = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :] - asset.data.root_lin_vel_w[
        :, :
    ].unsqueeze(1)
    # 准备保存机体系速度。
    footvel_in_body_frame = torch.zeros(env.num_envs, len(asset_cfg.body_ids), 3, device=env.device)
    for i in range(len(asset_cfg.body_ids)):
        # 转到机体系后，xy 分量就是相对身体的横向/前后滑动速度。
        footvel_in_body_frame[:, i, :] = math_utils.quat_apply_inverse(
            asset.data.root_quat_w, cur_footvel_translated[:, i, :]
        )
    # 计算每个足端 xy 平面速度大小。
    foot_leteral_vel = torch.sqrt(torch.sum(torch.square(footvel_in_body_frame[:, :, :2]), dim=2)).view(
        env.num_envs, -1
    )
    # 只在接触时惩罚滑动速度。
    reward = torch.sum(foot_leteral_vel * contacts, dim=1)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def _bernstein_torch(n: int, k: int, t: torch.Tensor) -> torch.Tensor:
    """计算 Bernstein 基函数 ``B_k^n(t)``。

    Bezier 曲线可以看作多个 Bernstein 基函数对控制点的加权和。
    """
    # 组合数 C(n, k)。
    coeff = float(math.comb(n, k))
    # B_k^n(t) = C(n,k) * (1-t)^(n-k) * t^k。
    return coeff * (1.0 - t) ** (n - k) * t**k


def _bezier_curve_torch(control_points: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
    """批量计算 Bezier 曲线点。

    ``control_points`` 是二维控制点序列，``t`` 是 [0, 1] 的曲线参数。
    返回值最后一维是曲线上的二维坐标，例如 ``[q, z]``。
    """
    # n 是曲线阶数，控制点数量 = n + 1。
    n = control_points.shape[0] - 1
    # 输出形状在 t 的基础上多一个坐标维度 2。
    out = torch.zeros(*t.shape, 2, device=t.device, dtype=t.dtype)
    # Bezier(t) = sum_k B_k^n(t) * P_k。
    for k in range(n + 1):
        out = out + _bernstein_torch(n, k, t).unsqueeze(-1) * control_points[k]
    return out


def _bezier_curve_derivative_torch(control_points: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
    """批量计算 Bezier 曲线对 t 的导数。"""
    n = control_points.shape[0] - 1
    # Bezier 导数仍是 Bezier 曲线，控制点变成 n*(P_{k+1}-P_k)。
    delta_ctrl = n * (control_points[1:] - control_points[:-1])
    out = torch.zeros(*t.shape, 2, device=t.device, dtype=t.dtype)
    for k in range(n):
        out = out + _bernstein_torch(n - 1, k, t).unsqueeze(-1) * delta_ctrl[k]
    return out


def phase_foot_trajectory_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    asset_cfg: SceneEntityCfg,
    std: float = 0.1,
    command_threshold: float = 0.1,
    cycle_time: float = 0.4,
    phase_offsets: tuple[float, ...] = (0.0, 1.0, 1.0, 0.0),
    gait_span: float = -0.008,
    gait_psi: float = 0.15,
    gait_delta: float = 0.03,
    x_offset: float = 0.0,
    stance_span: float = 0.20,
    stand_ref_z_offset: float = -0.2,
    velocity_weight: float = 0.5,
) -> torch.Tensor:
    """相位足端轨迹跟踪奖励。

    这是一个比较高级的步态项：先根据 episode 时间生成周期相位，再用分段轨迹
    生成每个足端/轮端在机体系下的参考位置和速度，最后奖励实际足端轨迹接近参考轨迹。

    当前 M20 rough 配置中该项通常为 0；它更像是给需要显式步态轨迹的机器人预留的工具。
    """
    asset: Articulation = env.scene[asset_cfg.name]
    # body_ids 是要跟踪的足端/轮端 body id。
    body_ids = asset_cfg.body_ids
    # 跟踪的足端数量。
    num_feet = len(body_ids)

    # 没有足端 body 时直接返回全 0，避免后续空张量计算。
    if num_feet == 0:
        return torch.zeros(env.num_envs, device=env.device)
    # phase_offsets 必须和足端数量一致，每个足端一个相位偏置。
    if len(phase_offsets) != num_feet:
        raise ValueError(f"phase_offsets length ({len(phase_offsets)}) must match tracked feet ({num_feet}).")

    # 第一次调用时，把当前足端相对机身的位置作为站立参考点缓存下来。
    if (not hasattr(env, "phase_foot_ref_body")) or (env.phase_foot_ref_body.shape[1] != num_feet):
        # 世界系足端位置相对机身位置。
        rel_foot_pos_w = asset.data.body_pos_w[:, body_ids, :] - asset.data.root_pos_w[:, :].unsqueeze(1)
        foot_pos_b = torch.zeros(env.num_envs, num_feet, 3, device=env.device)
        for i in range(num_feet):
            # 转为机体系足端位置。
            foot_pos_b[:, i, :] = math_utils.quat_apply_inverse(asset.data.root_quat_w, rel_foot_pos_w[:, i, :])
        # 用第 0 个 env 的初始足端位置作为公共参考。
        ref = foot_pos_b[0].detach().clone()
        # z 方向可以额外偏移，给轨迹留出抬脚/压低空间。
        ref[:, 2] += stand_ref_z_offset
        # 缓存在 env 上，后续 step 不再重复构造。
        env.phase_foot_ref_body = ref.unsqueeze(0)

    # 扩展成每个 env 一份参考站立点。
    stand_ref_body = env.phase_foot_ref_body.to(env.device).expand(env.num_envs, -1, -1)

    # 根据 episode 运行时间生成相位 S，范围 [0, 2)。
    phase_time = env.episode_length_buf.float() * env.step_dt
    # 每个足端一个相位偏置，用来形成交替步态。
    phase_offsets_t = torch.tensor(phase_offsets, device=env.device, dtype=phase_time.dtype).unsqueeze(0)
    S = torch.remainder((2.0 * phase_time / max(cycle_time, 1e-6)).unsqueeze(1) + phase_offsets_t, 2.0)

    # 以下构造局部二维轨迹 (q, z)：
    # q 表示前后摆动位移，z 表示抬脚高度。
    tau = float(gait_span)
    psi = float(gait_psi)
    delta = float(gait_delta)
    # stance_span 是支撑相长度，限制在 (0, 2) 内。
    stance_span = float(stance_span)
    stance_span = min(max(stance_span, 1e-6), 2.0 - 1e-6)

    # 初始化轨迹位置和对相位 S 的导数。
    q = torch.zeros_like(S)
    z = torch.zeros_like(S)
    dq_dS = torch.zeros_like(S)
    dz_dS = torch.zeros_like(S)

    # 支撑相：S < stance_span，足端接触/贴近地面。
    stance_mask = S < stance_span
    if stance_mask.any():
        # 把支撑相归一化到 [0,1]。
        s_stance = S / stance_span
        # 支撑相 q 从 +tau 线性走到 -tau。
        q_stance = tau * (1.0 - 2.0 * s_stance)
        # 支撑相 z 基本保持 delta。
        z_stance = torch.full_like(S, delta)
        # q 对 S 的导数。
        dq_dS_stance = torch.full_like(S, -2.0 * tau / stance_span)
        # z 对 S 的导数为 0。
        dz_dS_stance = torch.zeros_like(S)

        # 只把支撑相位置写入 q/z。
        q = torch.where(stance_mask, q_stance, q)
        z = torch.where(stance_mask, z_stance, z)
        dq_dS = torch.where(stance_mask, dq_dS_stance, dq_dS)
        dz_dS = torch.where(stance_mask, dz_dS_stance, dz_dS)

    # 摆动相：足端离地，用 Bezier 曲线生成平滑抬脚轨迹。
    swing_mask = ~stance_mask
    if swing_mask.any():
        # 把摆动相归一化到 [0,1]，作为 Bezier 参数。
        t_bezier = torch.clamp((S - stance_span) / (2.0 - stance_span), 0.0, 1.0)
        # Bezier 控制点，二维坐标含义是 [q, z]。
        ctrl = torch.tensor(
            [
                [-tau, 0.0],
                [-0.95 * tau, 0.80 * psi],
                [-0.55 * tau, 1.00 * psi],
                [0.55 * tau, 1.00 * psi],
                [0.95 * tau, 0.80 * psi],
                [tau, 0.0],
            ],
            device=env.device,
            dtype=S.dtype,
        )
        # 计算摆动相轨迹点。
        qz_swing = _bezier_curve_torch(ctrl, t_bezier)
        # 计算轨迹对 t 的导数。
        dqz_dt = _bezier_curve_derivative_torch(ctrl, t_bezier)
        # 链式法则：t 对 S 的导数。
        dt_dS = 1.0 / (2.0 - stance_span)

        # 写入摆动相 q/z 和导数。
        q = torch.where(swing_mask, qz_swing[..., 0], q)
        z = torch.where(swing_mask, qz_swing[..., 1] + delta, z)
        dq_dS = torch.where(swing_mask, dqz_dt[..., 0] * dt_dS, dq_dS)
        dz_dS = torch.where(swing_mask, dqz_dt[..., 1] * dt_dS, dz_dS)

    # S = 2 * t / cycle_time，所以 dS/dt = 2/cycle_time。
    dS_dt = 2.0 / max(cycle_time, 1e-6)
    # 链式法则得到 q/z 对真实时间的速度。
    dq_dt = dq_dS * dS_dt
    dz_dt = dz_dS * dS_dt

    # 参考足端位置：站立参考点 + 周期轨迹偏移。
    ref_pos_b = stand_ref_body + torch.stack(
        [q + float(x_offset), torch.zeros_like(q), z],
        dim=-1,
    )
    # 参考足端速度。
    ref_vel_b = torch.stack(
        [dq_dt, torch.zeros_like(dq_dt), dz_dt],
        dim=-1,
    )

    # 实际足端位置/速度也转换到机体系，用来和参考轨迹比较。
    rel_foot_pos_w = asset.data.body_pos_w[:, body_ids, :] - asset.data.root_pos_w[:, :].unsqueeze(1)
    rel_foot_vel_w = asset.data.body_lin_vel_w[:, body_ids, :] - asset.data.root_lin_vel_w[:, :].unsqueeze(1)
    foot_pos_b = torch.zeros(env.num_envs, num_feet, 3, device=env.device)
    foot_vel_b = torch.zeros(env.num_envs, num_feet, 3, device=env.device)
    for i in range(num_feet):
        foot_pos_b[:, i, :] = math_utils.quat_apply_inverse(asset.data.root_quat_w, rel_foot_pos_w[:, i, :])
        foot_vel_b[:, i, :] = math_utils.quat_apply_inverse(asset.data.root_quat_w, rel_foot_vel_w[:, i, :])

    # 实际轨迹和参考轨迹的偏差。
    pos_offset = foot_pos_b - ref_pos_b
    vel_offset = foot_vel_b - ref_vel_b

    # 对所有足端求和，得到每个 env 在 x/y/z 三个方向上的位置/速度误差。
    pos_err = torch.sum(torch.square(pos_offset), dim=1)
    vel_err = torch.sum(torch.square(vel_offset), dim=1)

    # 总误差 = 位置误差 + velocity_weight * 速度误差。
    total_err = torch.sum(pos_err, dim=1) + float(velocity_weight) * torch.sum(vel_err, dim=1)
    # 指数核：误差越小，奖励越接近 1。
    reward = torch.exp(-total_err / max(std, 1e-6) ** 2)

    # 只有存在 x/y/yaw 运动指令时才启用该轨迹奖励。
    command = env.command_manager.get_command(command_name)
    gate = torch.linalg.norm(command[:, :3], dim=1) > command_threshold

    # 以下均值主要用于临时调试，可打印查看轨迹误差偏向哪个方向。
    pos_offset_xyz_mean = torch.mean(pos_offset, dim=(0, 1))
    vel_offset_xyz_mean = torch.mean(vel_offset, dim=(0, 1))
    # print(
    #     "Offset xyz mean | "
    #     f"pos(x,y,z)=({pos_offset_xyz_mean[0].item():.4f}, {pos_offset_xyz_mean[1].item():.4f}, {pos_offset_xyz_mean[2].item():.4f}) | "
    #     f"vel(x,y,z)=({vel_offset_xyz_mean[0].item():.4f}, {vel_offset_xyz_mean[1].item():.4f}, {vel_offset_xyz_mean[2].item():.4f})"
    # )
    # print("Reward:", reward * gate.float())
    return reward * gate.float() * get_gait_level_tensor(env)

def foot_impact_velocity(
    env,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    speed_threshold: float = 0.10,
) -> torch.Tensor:
    """足端落地冲击速度惩罚。

    只在足端“首次接触地面”的控制步生效，并惩罚超过阈值的向下速度。
    这样可以鼓励落地更柔和，减少爬台阶/越障时的冲击。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    asset: RigidObject = env.scene[asset_cfg.name]

    # 首次接触事件，形状约为 (num_envs, num_feet)。
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids].float()
    # 足端世界系线速度。
    foot_lin_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]

    # z 速度为负表示向下运动；只取向下速度。
    downward_speed = torch.clamp(-foot_lin_vel[:, :, 2], min=0.0)
    # 小于阈值的落地速度不惩罚，超过部分才惩罚。
    downward_speed = torch.clamp(downward_speed - speed_threshold, min=0.0)

    # 只在首次接触时计算向下速度平方惩罚。
    penalty = torch.sum(first_contact * torch.square(downward_speed), dim=1)
    return penalty * get_gait_level_tensor(env)

# def stand_still_joint_deviation_l1(
#     env, command_name: str, command_threshold: float = 0.06, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
# ) -> torch.Tensor:
#     """Penalize offsets from the default joint positions when the command is very small."""
    # command = env.command_manager.get_command(command_name)
#     # Penalize motion when command is nearly zero.
#     return joint_deviation_l1(env, asset_cfg) * (torch.norm(command[:, :], dim=1) < command_threshold)

# def joint_deviation_l1(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
#     """Penalize joint positions that deviate from the default one."""
#     # extract the used quantities (to enable type-hinting)
#     asset: Articulation = env.scene[asset_cfg.name]
#     # compute out of limits constraints
#     angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
#     return torch.sum(torch.abs(angle), dim=1)


# def smoothness_1(env: ManagerBasedRLEnv) -> torch.Tensor:
#     # Penalize changes in actions
#     diff = torch.square(env.action_manager.action - env.action_manager.prev_action)
#     diff = diff * (env.action_manager.prev_action[:, :] != 0)  # ignore first step
#     return torch.sum(diff, dim=1)


# def joint_acc_l2_new(env: ManagerBasedRLEnv) -> torch.Tensor:

# def smoothness_2(env: ManagerBasedRLEnv) -> torch.Tensor:
#     # Penalize changes in actions
#     diff = torch.square(env.action_manager.action - 2 * env.action_manager.prev_action + env.action_manager.prev_prev_action)
#     diff = diff * (env.action_manager.prev_action[:, :] != 0)  # ignore first step
#     diff = diff * (env.action_manager.prev_prev_action[:, :] != 0)  # ignore second step
#     # print(torch.sum(diff, dim=1), "smoothness l2")
#     return torch.sum(diff, dim=1)


def upward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """机身朝上方向奖励/惩罚项。

    ``projected_gravity_b[:, 2]`` 表示重力方向在机体系 z 轴上的投影。
    正常站立时它通常接近 -1；如果机身翻倒，该值会接近 0 或变正。
    这里返回 ``(1 - projected_gravity_z)^2``，具体鼓励/惩罚取决于配置权重。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # 注意这个函数名叫 upward，但返回值本身是误差形式；读配置权重时要结合判断。
    reward = torch.square(1 - asset.data.projected_gravity_b[:, 2])
    return reward





def base_height_l2(
    env: ManagerBasedRLEnv,
    target_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sensor_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """机身高度误差惩罚。

    平地时 target_height 是世界系绝对高度；
    粗糙地形时可用 height_scanner_base 扫描局部地面高度，把目标高度修正为
    ``局部地面高度 + target_height``。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    if sensor_cfg is not None:
        # RayCaster 会向地面打射线，ray_hits_w[..., 2] 是命中点世界系 z 高度。
        sensor: RayCaster = env.scene[sensor_cfg.name]
        ray_hits = sensor.data.ray_hits_w[..., 2]
        # 如果射线数据异常，退回当前高度，避免 NaN/Inf 把训练损坏。
        if torch.isnan(ray_hits).any() or torch.isinf(ray_hits).any() or torch.max(torch.abs(ray_hits)) > 1e6:
            adjusted_target_height = asset.data.root_link_pos_w[:, 2]
        else:
            # 用射线命中点平均高度估计机器人脚下局部地面高度。
            adjusted_target_height = target_height + torch.mean(ray_hits, dim=1)
    else:
        # 平地或未配置传感器时，直接使用固定目标高度。
        adjusted_target_height = target_height
    # 机身根节点高度与目标高度的平方误差。
    reward = torch.square(asset.data.root_pos_w[:, 2] - adjusted_target_height)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward * get_gait_level_tensor(env)


def lin_vel_z_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """机身 z 方向速度惩罚。

    惩罚上下跳动，鼓励机器人在移动/爬台阶时不要出现过大的垂直弹跳。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # root_lin_vel_b 是机体系线速度，索引 2 是 z 方向。
    reward = torch.square(asset.data.root_lin_vel_b[:, 2])
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def ang_vel_xy_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """机身 roll/pitch 角速度惩罚。

    x/y 轴角速度大，说明机身在快速前后翻滚或左右侧滚。该项能抑制粗糙地形上的剧烈晃动。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # root_ang_vel_b[:, :2] 对应机体系 roll/pitch 角速度。
    reward = torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def undesired_contacts(env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """非期望部位接触惩罚。

    例如只希望轮端/足端接触地面，那么机身、连杆等 body 的接触力超过阈值就记为违规。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # 历史接触力张量，包含最近若干物理步。
    net_contact_forces = contact_sensor.data.net_forces_w_history
    # 对力向量求范数，再取历史维度最大值；超过阈值视为接触违规。
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    # 每个 env 统计违规 body 数量。
    reward = torch.sum(is_contact, dim=1).float()
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def flat_orientation_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """机身非水平姿态惩罚。

    使用重力在机体系 x/y 方向的投影衡量倾斜程度：
    机器人越水平，重力越接近机体系 -z，x/y 分量越小。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # x/y 分量平方和越大，说明 roll/pitch 倾斜越严重。
    reward = torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward

def feet_air_time_lin_xy_cmd(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
    cmd_threshold: float = 0.1,
) -> torch.Tensor:
    """只在平面线速度命令存在时启用的足端腾空时间奖励。

    适用于“机器人在向前/侧向移动时，希望足端有合理腾空”的任务。
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # 首次接触事件：足端刚落地时为 True。
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    # 上一次离地持续时间，常用于衡量步长/摆动时间。
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    # 落地瞬间，如果此前离地时间超过 threshold，则得到正值；否则为负/小值。
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)

    # 只看 x/y 平动命令，不看 yaw 命令。
    cmd_lin_xy = torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1)
    # 平动命令足够大时才启用。
    reward *= cmd_lin_xy > cmd_threshold
    return reward * get_gait_level_tensor(env)

def feet_air_time_x_neg_cmd(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
    cmd_threshold: float = 0.1,
) -> torch.Tensor:
    """只在后退命令下启用的足端腾空时间奖励。"""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # 足端刚接触地面的事件。
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    # 上一次离地时间。
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    # 离地时间超过 threshold 时，在落地瞬间给奖励。
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)

    # 这里只要求 x 命令为负，即后退；cmd_threshold 当前没有参与计算。
    cmd_x = env.command_manager.get_command(command_name)[:, 0]
    reward *= cmd_x < 0.0

    return reward

def feet_air_time_ang_z_cmd(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
    cmd_threshold: float = 0.1,
) -> torch.Tensor:
    """只在偏航角速度命令存在时启用的足端腾空时间奖励。"""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # 足端刚落地事件。
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    # 记录上一段离地持续时间。
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    # 根据离地时间计算奖励。
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)

    # 只看 yaw 角速度命令。
    cmd_ang_z = torch.abs(env.command_manager.get_command(command_name)[:, 2])
    # 转向命令足够大时启用。
    reward *= cmd_ang_z > cmd_threshold
    return reward * get_gait_level_tensor(env)

def feet_air_time_including_ang_z(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """同时考虑平动和转向命令的足端腾空时间奖励。

    与 ``feet_air_time_lin_xy_cmd`` 的区别是：这里 command 的 x/y/yaw 任意方向有明显命令，
    都会启用奖励。对原地转向也需要迈步的四足机器人更合适。
    """
    # 接触传感器提供首次接触和上次离地时间。
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # 当前 step 哪些足端刚从空中落地。
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    # 对应足端上一次离地持续了多久。
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    # 离地时间超过 threshold 的部分，在落地时转成奖励。
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    # command 三维模长较小时，认为机器人应站住，不奖励迈步。
    reward *= torch.norm(env.command_manager.get_command(command_name), dim=1) > 0.1
    # reward *= torch.norm(env.command_manager.get_command(command_name)[:, :3], dim=1) > 0.1
    return reward

def lin_vel_xy_l2_with_ang_z_command(
    env: ManagerBasedRLEnv,
    command_name: str,
    command_threshold: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ) -> torch.Tensor:
    """纯转向命令下的平面漂移惩罚。

    如果 command 主要是 yaw 角速度、几乎没有 x/y 平移命令，那么机器人应原地转向。
    此时实际 x/y 平移速度越大，惩罚越大。
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # 实际机体系 x/y 平移速度平方和。
    reward = torch.sum(torch.square(asset.data.root_lin_vel_b[:, :2]), dim=1)
    # 当前速度命令。
    command = env.command_manager.get_command(command_name)
    # 条件 1：yaw 命令足够大；条件 2：x/y 平移命令足够小。
    reward *= (torch.sum(torch.square(command[:, 2:]), dim=1) > command_threshold) & \
            (torch.sum(torch.square(command[:, :2]), dim=1) < command_threshold)
    # reward *= torch.sum(torch.square(env.command_manager.get_command(command_name)[:, 2:]), dim=1) > command_threshold
    # reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward
