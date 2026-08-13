# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""轮足机器人速度跟踪任务配置包。

M20 属于 wheeled legged robot：每条腿既有关节姿态控制，又有轮子速度控制。
因此它放在 ``wheeled/deeprobotics_m20`` 下，而不是纯四足的 ``quadruped`` 下。
"""

# 这个文件主要承担“包标记”的作用。
# 具体 Gym task 的注册在 ``deeprobotics_m20/__init__.py`` 中完成。
