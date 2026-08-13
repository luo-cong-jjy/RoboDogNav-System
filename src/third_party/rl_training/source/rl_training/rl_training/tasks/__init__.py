# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Package containing task implementations for various robotic environments.

初学者提示：
    这个文件本身不定义机器人环境，但它非常关键。训练脚本执行
    ``import rl_training.tasks`` 时，会走到这里，然后通过 ``import_packages``
    自动导入子目录里的任务配置。子目录被导入后，里面的 ``gym.register(...)``
    才会执行，Isaac Lab/Gym 才知道有哪些任务名可以用。

    也就是说，README 里的任务名，例如 ``Rough-Deeprobotics-M20-v0``，
    并不是训练脚本硬编码出来的，而是在子包导入时注册进 Gym 的。
"""

import os
import toml

from isaaclab_tasks.utils import import_packages

##
# Register Gym environments.
##


# 黑名单用于跳过不希望自动导入的子包。这里跳过 utils，避免把工具模块当任务配置加载。
_BLACKLIST_PKGS = ["utils"]
# 自动导入当前 tasks 包下面的所有配置子包，从而触发各机器人任务的 gym.register。
import_packages(__name__, _BLACKLIST_PKGS)
