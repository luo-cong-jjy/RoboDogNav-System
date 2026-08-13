# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Lite3 训练算法配置包。

这里本身不定义算法参数，具体 PPO 配置在 ``rsl_rl_ppo_cfg.py``。
保留这个包入口是为了让 Gym 注册文件能统一引用 ``agents.__name__``。
"""
