# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

# 这个文件负责把 Lite3 的训练任务注册到 Gym/Isaac Lab。
# 只要上层执行 ``import rl_training.tasks``，任务自动发现机制就会导入本包，
# 下面两个 ``gym.register`` 会把任务名挂到 Gym registry 里。

gym.register(
    id="Flat-Deeprobotics-Lite3-v0",
    # ManagerBasedRLEnv 会根据 kwargs 中的 env_cfg_entry_point 加载场景、观测、动作、奖励。
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    # Isaac Lab 自己负责检查配置，这里关闭 Gymnasium 默认 env checker。
    disable_env_checker=True,
    kwargs={
        # 平地环境：适合快速验证策略、动作维度、观测维度是否跑通。
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:DeeproboticsLite3FlatEnvCfg",
        # RSL-RL PPO 超参数配置。
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DeeproboticsLite3FlatPPORunnerCfg",
        # 预留 cusrl 入口；当前学习主线通常看 rsl_rl。
        "cusrl_cfg_entry_point": f"{agents.__name__}.cusrl_ppo_cfg:DeeproboticsLite3FlatTrainerCfg",
    },
)

gym.register(
    id="Rough-Deeprobotics-Lite3-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        # 粗糙地形环境：用于训练更鲁棒的足式运动策略。
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:DeeproboticsLite3RoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DeeproboticsLite3RoughPPORunnerCfg",
        "cusrl_cfg_entry_point": f"{agents.__name__}.cusrl_ppo_cfg:DeeproboticsLite3RoughTrainerCfg",
    },
)
