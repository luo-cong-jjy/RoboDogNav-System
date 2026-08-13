# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

# 这个文件负责把 M20 任务注册到 Gym/Isaac Lab。
# 训练脚本里写的 --task=Rough-Deeprobotics-M20-v0，就是下面的 id。
# 当执行 import rl_training.tasks 时，tasks/__init__.py 会自动导入这个包，
# 于是 gym.register(...) 被执行，任务名才真正可用。

gym.register(
    id="Flat-Deeprobotics-M20-v0",
    # ManagerBasedRLEnv 是 Isaac Lab 的“管理器式强化学习环境”。
    # 它会根据 env_cfg_entry_point 加载 scene/action/observation/reward 等配置。
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        # 平地环境配置：地形是 plane，适合先跑通训练流程。
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:DeeproboticsM20FlatEnvCfg",
        # RSL-RL PPO 配置：网络结构、学习率、迭代次数等。
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DeeproboticsM20FlatPPORunnerCfg",
        # 这里保留了 cusrl 入口名，但当前仓库主线使用的是 rsl_rl 配置。
        "cusrl_cfg_entry_point": f"{agents.__name__}.cusrl_ppo_cfg:DeeproboticsM20FlatTrainerCfg",
    },
)

gym.register(
    id="Rough-Deeprobotics-M20-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        # 粗糙地形环境配置：训练鲁棒运动策略的主入口。
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:DeeproboticsM20RoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DeeproboticsM20RoughPPORunnerCfg",
        "cusrl_cfg_entry_point": f"{agents.__name__}.cusrl_ppo_cfg:DeeproboticsM20RoughTrainerCfg",
    },
)
