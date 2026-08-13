# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""纯四足机器人速度跟踪任务配置包。

Lite3 这类机器人没有轮子，动作通常全部是腿部关节位置目标。
它和 M20 的 wheeled 配置形成对照：
    * quadruped：足式步态、足端离地时间、落脚节奏更重要；
    * wheeled：腿部姿态 + 轮子速度共同决定运动。
"""

# 具体任务注册在更深一级的 ``deeprobotics_lite3/__init__.py`` 中完成。
