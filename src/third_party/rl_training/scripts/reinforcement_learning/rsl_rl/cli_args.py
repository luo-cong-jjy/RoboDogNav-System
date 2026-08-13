# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import random
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg


# 向命令行解析器“注册”参数
def add_rsl_rl_args(parser: argparse.ArgumentParser):
    """Add RSL-RL arguments to the parser.

    Args:
        parser: The parser to add the arguments to.
    """
    # create a new argument group
    arg_group = parser.add_argument_group("rsl_rl", description="Arguments for RSL-RL agent.")
    # -- experiment arguments
    arg_group.add_argument(
        "--experiment_name", type=str, default=None, help="Name of the experiment folder where logs will be stored."
    )#--experiment_name：指定实验的根文件夹名称，所有的日志都会保存在这个目录下。
    arg_group.add_argument("--run_name", type=str, default=None, help="Run name suffix to the log directory.")
    # -- load arguments#--run_name：作为日志目录的后缀，方便在同一次实验中区分不同的运行记录。
    arg_group.add_argument("--resume", action="store_true", default=False, help="Whether to resume from a checkpoint.")
    # 布尔开关，告诉程序是否需要从之前保存的断点（checkpoint）继续训练。
    arg_group.add_argument("--load_run", type=str, default=None, help="Name of the run folder to resume from.")
    # 指定要从哪个历史运行文件夹中加载模型。
    arg_group.add_argument("--checkpoint", type=str, default=None, help="Checkpoint file to resume from.")
    # 指定具体要加载哪一个模型权重文件。
    # -- logger arguments
    arg_group.add_argument(#选择使用哪种可视化和日志追踪平台（如 wandb、tensorboard 或 neptune）
        "--logger", type=str, default=None, choices={"wandb", "tensorboard", "neptune"}, help="Logger module to use."
    )
    arg_group.add_argument(#当使用 wandb 或 neptune 时，指定云端项目组的名称。
        "--log_project_name", type=str, default=None, help="Name of the logging project when using wandb or neptune."
    )

# 配置的“组装与解析”负责生成最终的算法配置对象（RslRlOnPolicyRunnerCfg）
def parse_rsl_rl_cfg(task_name: str, args_cli: argparse.Namespace) -> RslRlOnPolicyRunnerCfg:
    """Parse configuration for RSL-RL agent based on inputs.

    Args:
        task_name: The name of the environment.
        args_cli: The command line arguments.

    Returns:
        The parsed configuration for RSL-RL agent based on inputs.
    """
    from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

    # load the default configuration
    rslrl_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(task_name, "rsl_rl_cfg_entry_point")
    rslrl_cfg = update_rsl_rl_cfg(rslrl_cfg, args_cli)
    return rslrl_cfg

# 功能：命令行参数对配置的“热更新”，这个函数是参数生效的核心逻辑。它遍历命令行传入的参数，如果用户显式传入了某个值，就将其写入到 agent_cfg 对象中。
def update_rsl_rl_cfg(agent_cfg: RslRlOnPolicyRunnerCfg, args_cli: argparse.Namespace):
    """Update configuration for RSL-RL agent based on inputs.

    Args:
        agent_cfg: The configuration for RSL-RL agent.
        args_cli: The command line arguments.

    Returns:
        The updated configuration for RSL-RL agent based on inputs.
    """
    # override the default configuration with CLI arguments
    if hasattr(args_cli, "seed") and args_cli.seed is not None:
        # randomly sample a seed if seed = -1
        if args_cli.seed == -1:
            args_cli.seed = random.randint(0, 10000)
        agent_cfg.seed = args_cli.seed
    if args_cli.resume is not None:
        agent_cfg.resume = args_cli.resume#resume:恢复开关
    if args_cli.load_run is not None:
        agent_cfg.load_run = args_cli.load_run#load_run恢复路径
    if args_cli.checkpoint is not None:
        agent_cfg.load_checkpoint = args_cli.checkpoint #恢复到训练的哪一步
    if args_cli.run_name is not None:
        agent_cfg.run_name = args_cli.run_name#新日志命名
    if args_cli.logger is not None:#Logger（记录器），选“用哪个平台记录实验”
        agent_cfg.logger = args_cli.logger
    # set the project name for wandb and neptune
    if agent_cfg.logger in {"wandb", "neptune"} and args_cli.log_project_name:#wandb_project= WandB 里的项目名，不是日志等级
        agent_cfg.wandb_project = args_cli.log_project_name
        agent_cfg.neptune_project = args_cli.log_project_name

    return agent_cfg

# 新旧版本 RSL-RL 的“格式翻译器”这是整个脚本中最具技术含量的一个函数。它解决的是版本兼容性问题。
def convert_rsl_rl_cfg_dict(cfg_dict: dict) -> dict:
    """Convert old-style rsl-rl config dict (with 'policy' key) to new-style (with 'actor'/'critic' keys).

    rsl-rl v5+ expects separate 'actor' and 'critic' config dicts instead of a single 'policy' dict.
    This function performs the conversion so that IsaacLab's config format works with rsl-rl v5+.

    Args:
        cfg_dict: The config dict from agent_cfg.to_dict().

    Returns:
        The converted config dict compatible with rsl-rl v5+.
    """
    if "actor" in cfg_dict and "critic" in cfg_dict:
        # Already in new format
        return cfg_dict

    policy = cfg_dict.pop("policy", {})

    # Build distribution config for actor from noise std settings
    init_noise_std = policy.pop("init_noise_std", 1.0)
    noise_std_type = policy.pop("noise_std_type", "scalar")
    distribution_cfg = {
        "class_name": "GaussianDistribution",
        "init_std": init_noise_std,
        "std_type": noise_std_type,
    }

    # Handle observation normalization
    actor_obs_norm = policy.pop("actor_obs_normalization", False)
    critic_obs_norm = policy.pop("critic_obs_normalization", False)
    # Support legacy empirical_normalization flag
    empirical_norm = cfg_dict.pop("empirical_normalization", None)
    if empirical_norm is not None:
        actor_obs_norm = empirical_norm
        critic_obs_norm = empirical_norm

    actor_hidden_dims = policy.pop("actor_hidden_dims", [256, 256, 256])
    critic_hidden_dims = policy.pop("critic_hidden_dims", [256, 256, 256])
    activation = policy.pop("activation", "elu")

    cfg_dict["actor"] = {
        "class_name": "MLPModel",
        "hidden_dims": actor_hidden_dims,
        "activation": activation,
        "obs_normalization": actor_obs_norm,
        "distribution_cfg": distribution_cfg,
    }
    cfg_dict["critic"] = {
        "class_name": "MLPModel",
        "hidden_dims": critic_hidden_dims,
        "activation": activation,
        "obs_normalization": critic_obs_norm,
    }

    # Ensure obs_groups is a proper dict (handle MISSING sentinel)
    obs_groups = cfg_dict.get("obs_groups")
    if not isinstance(obs_groups, dict):
        cfg_dict["obs_groups"] = {}

    return cfg_dict
