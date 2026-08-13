# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""速度跟踪型运动任务包。

这个包下面的环境都属于同一类任务：
    给机器人一个速度指令 ``[vx, vy, yaw_rate]``，
    让策略学习如何输出关节动作，使机器人尽量跟踪这个速度。

对 M20 来说，真正重要的子路径是：
    ``config/wheeled/deeprobotics_m20``：M20 平地/粗糙地形任务注册和配置；
    ``mdp``：观测、奖励、事件、课程学习等函数；
    ``velocity_env_cfg.py``：通用速度跟踪环境骨架。

这些环境继承了 legged_gym/Isaac Lab 系列四足运动任务的设计思想：
    并行大量机器人环境、随机命令、随机地形、PPO 训练速度跟踪策略。

Reference:
    https://github.com/leggedrobotics/legged_gym
"""
