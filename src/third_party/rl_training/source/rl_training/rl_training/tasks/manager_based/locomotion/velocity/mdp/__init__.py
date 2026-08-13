# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This sub-module contains the functions that are specific to the locomotion environments.

初学者提示：
    在 Isaac Lab 的 ManagerBasedRLEnv 中，MDP 会被拆成很多小函数：

    * command：生成速度指令，比如希望机器人前进 1 m/s。
    * observation：生成策略网络输入，比如关节角、角速度、重力方向。
    * reward：计算奖励或惩罚，比如速度跟踪好不好、动作是否平滑。
    * event：reset、随机化、外力扰动等训练事件。
    * curriculum：课程学习，训练前期简单，后期逐步加难。

    这个 ``__init__.py`` 把 Isaac Lab 自带的 MDP 函数和本项目自定义函数统一导出。
    所以其他配置文件可以直接写 ``import ...mdp as mdp``，然后使用
    ``mdp.track_lin_vel_xy_exp``、``mdp.joint_pos_rel_without_wheel`` 等函数。
"""

# 先导入 Isaac Lab/Isaac Lab Tasks 已经实现好的通用 MDP 函数。
from isaaclab.envs.mdp import *  # noqa: F401, F403
from isaaclab_tasks.manager_based.locomotion.velocity.mdp import *  # noqa: F401, F403

# 再导入本仓库针对 DeepRobotics 任务额外写的函数。
from .commands import *  # noqa: F401, F403
from .curriculums import *  # noqa: F401, F403
from .events import *  # noqa: F401, F403
from .observations import *  # noqa: F401, F403
from .rewards import *  # noqa: F401, F403
