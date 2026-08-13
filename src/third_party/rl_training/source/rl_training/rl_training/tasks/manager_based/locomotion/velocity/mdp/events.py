# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Literal

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


# event 函数用于“训练时发生的事件”，例如：
# 1. startup：环境刚创建时随机化质量、摩擦、惯量等物理参数；
# 2. reset：某个 env 回合结束并重置时，随机化初始姿态、速度、外力等；
# 3. interval：训练过程中隔一段时间触发，例如随机推一下机器人。
# 这些事件不会直接给策略网络输入，也不直接给奖励，但会改变仿真世界，
# 从而提升策略对模型误差、地形变化和外界扰动的鲁棒性。


def randomize_rigid_body_inertia(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    inertia_distribution_params: tuple[float, float],
    operation: Literal["add", "scale", "abs"],
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """随机化刚体惯量张量。

    初学者理解：
        惯量描述“物体转起来有多难”。真实机器人 CAD/URDF/USD 中的惯量不可能完全准确，
        电池、线束、负载也会让惯量变化。训练时随机化惯量，可以避免策略只适应一个
        过于理想的仿真模型。

    这个函数只改惯量矩阵的对角线项 xx、yy、zz，并支持三种操作：
        * add：在默认值基础上加随机量；
        * scale：按比例缩放默认值；
        * abs：直接设置成随机绝对值。

    .. tip::
        This function uses CPU tensors to assign the body inertias. It is recommended to use this function
        only during the initialization of the environment.
    """
    # 取出目标资产，可能是机器人 Articulation，也可能是普通 RigidObject。
    asset: RigidObject | Articulation = env.scene[asset_cfg.name]

    # 解析需要随机化的并行环境 id；None 表示所有 env。
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device="cpu")
    else:
        env_ids = env_ids.cpu()

    # 解析需要随机化的刚体 id；slice(None) 表示这个资产下所有 body。
    if asset_cfg.body_ids == slice(None):
        body_ids = torch.arange(asset.num_bodies, dtype=torch.int, device="cpu")
    else:
        body_ids = torch.tensor(asset_cfg.body_ids, dtype=torch.int, device="cpu")

    # 获取 PhysX 当前惯量。最后一维长度为 9，对应 3x3 惯量矩阵展开。
    inertias = asset.root_physx_view.get_inertias()

    # 每次都从默认惯量开始随机化，避免多次 reset 后随机误差不断累积。
    inertias[env_ids[:, None], body_ids, :] = asset.data.default_inertia[env_ids[:, None], body_ids, :].clone()

    # 只随机化对角线元素：xx, yy, zz 在展开矩阵中的索引是 0, 4, 8。
    for idx in [0, 4, 8]:
        # 对某一个对角线分量应用随机化。
        randomized_inertias = _randomize_prop_by_op(
            inertias[:, :, idx],
            inertia_distribution_params,
            env_ids,
            body_ids,
            operation,
            distribution,
        )
        # 写回该对角线分量。
        inertias[env_ids[:, None], body_ids, idx] = randomized_inertias

    # 将随机化后的惯量写入 PhysX 仿真。
    asset.root_physx_view.set_inertias(inertias, env_ids)


def randomize_com_positions(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    com_distribution_params: tuple[float, float],
    operation: Literal["add", "scale", "abs"],
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """随机化刚体质心位置。

    初学者理解：
        质心 COM 决定机器人受力后的姿态变化。真实机器人如果背了负载、线束位置不同、
        电池安装有偏差，质心都会和仿真模型不同。训练时对 COM 做小范围随机化，
        可以让策略更不依赖“完美质心”。

    .. tip::
        This function is intended for initialization or offline adjustments, as it modifies physics properties directly.

    Args:
        env (ManagerBasedEnv): The simulation environment.
        env_ids (torch.Tensor | None): Specific environment indices to apply randomization, or None for all environments.
        asset_cfg (SceneEntityCfg): The configuration for the target asset whose COM will be randomized.
        com_distribution_params (tuple[float, float]): Parameters of the distribution (e.g., min and max for uniform).
        operation (Literal["add", "scale", "abs"]): The operation to apply for randomization.
        distribution (Literal["uniform", "log_uniform", "gaussian"]): The distribution to sample random values from.
    """
    # 取出需要修改 COM 的资产。
    asset: RigidObject | Articulation = env.scene[asset_cfg.name]

    # 解析需要应用随机化的 env id。
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device="cpu")
    else:
        env_ids = env_ids.cpu()

    # 解析需要应用随机化的 body id。
    if asset_cfg.body_ids == slice(None):
        body_ids = torch.arange(asset.num_bodies, dtype=torch.int, device="cpu")
    else:
        body_ids = torch.tensor(asset_cfg.body_ids, dtype=torch.int, device="cpu")

    # COM 偏移形状是 (num_envs, num_bodies, 3)，最后一维是 x/y/z。
    com_offsets = asset.root_physx_view.get_coms()

    # x/y/z 三个方向分别随机化。
    # 分开采样的好处是：可以让质心在三维空间里自由偏移，而不是所有方向共用同一个随机数。
    for dim_idx in range(3):
        # 对当前维度应用 add/scale/abs 随机化。
        randomized_offset = _randomize_prop_by_op(
            com_offsets[:, :, dim_idx],
            com_distribution_params,
            env_ids,
            body_ids,
            operation,
            distribution,
        )
        com_offsets[env_ids[:, None], body_ids, dim_idx] = randomized_offset[env_ids[:, None], body_ids]

    # 写回仿真。
    asset.root_physx_view.set_coms(com_offsets, env_ids)


"""
Internal helper functions.
"""


def _randomize_prop_by_op(
    data: torch.Tensor,
    distribution_parameters: tuple[float | torch.Tensor, float | torch.Tensor],
    dim_0_ids: torch.Tensor | None,
    dim_1_ids: torch.Tensor | slice,
    operation: Literal["add", "scale", "abs"],
    distribution: Literal["uniform", "log_uniform", "gaussian"],
) -> torch.Tensor:
    """按指定分布和操作随机化张量中的一部分数据。

    Args:
        data: The data tensor to be randomized. Shape is (dim_0, dim_1).
        distribution_parameters: The parameters for the distribution to sample values from.
        dim_0_ids: The indices of the first dimension to randomize.
        dim_1_ids: The indices of the second dimension to randomize.
        operation: The operation to perform on the data. Options: 'add', 'scale', 'abs'.
        distribution: The distribution to sample the random values from. Options: 'uniform', 'log_uniform'.

    Returns:
        The data tensor after randomization. Shape is (dim_0, dim_1).

    Raises:
        NotImplementedError: If the operation or distribution is not supported.
    """
    # 解析第 0 维索引数量，通常对应 env 维度。
    if dim_0_ids is None:
        n_dim_0 = data.shape[0]
        dim_0_ids = slice(None) # type: ignore
    else:
        n_dim_0 = len(dim_0_ids)
        if not isinstance(dim_1_ids, slice):
            dim_0_ids = dim_0_ids[:, None]
    # 解析第 1 维索引数量，通常对应 body 或 joint 维度。
    if isinstance(dim_1_ids, slice):
        n_dim_1 = data.shape[1]
    else:
        n_dim_1 = len(dim_1_ids)

    # 根据字符串选择采样函数。
    if distribution == "uniform":
        dist_fn = math_utils.sample_uniform
    elif distribution == "log_uniform":
        dist_fn = math_utils.sample_log_uniform
    elif distribution == "gaussian":
        dist_fn = math_utils.sample_gaussian
    else:
        raise NotImplementedError(
            f"Unknown distribution: '{distribution}' for joint properties randomization."
            " Please use 'uniform', 'log_uniform', 'gaussian'."
        )
    # 执行随机化操作。
    # add：data = data + random，适合“加几公斤/偏几厘米”；
    # scale：data = data * random，适合“质量/惯量按比例变化”；
    # abs：data = random，适合直接覆盖成某个绝对物理值。
    if operation == "add":
        data[dim_0_ids, dim_1_ids] += dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    elif operation == "scale":
        data[dim_0_ids, dim_1_ids] *= dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    elif operation == "abs":
        data[dim_0_ids, dim_1_ids] = dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    else:
        raise NotImplementedError(
            f"Unknown operation: '{operation}' for property randomization. Please use 'add', 'scale', or 'abs'."
        )
    return data


def bad_orientation_2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot") # type: ignore
) -> torch.Tensor:
    """当机器人姿态过差时终止 episode。

    ``projected_gravity_b`` 是重力方向在机体系下的投影。机器人正常站立时，
    这个向量大致沿机体 -z 方向。如果 z 分量变正，或者 x/y 分量太大，
    说明机器人可能已经翻倒或侧倾严重。
    """
    # 取出机器人资产并根据重力投影判断姿态是否越界。
    asset: RigidObject = env.scene[asset_cfg.name]
    # 条件 1：projected_gravity_b[:, 2] > 0，说明机体 z 轴方向和重力关系已经很异常。
    # 条件 2：x/y 分量绝对值大于 0.7，说明 roll 或 pitch 过大。
    # 任一条件满足，就认为该 env 姿态失败，需要终止。
    return (asset.data.projected_gravity_b[:, 2] > 0) | (asset.data.projected_gravity_b[:, :2].abs() > 0.7).any(-1)
