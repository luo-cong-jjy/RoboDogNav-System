# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause
# 
# # Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""速度跟踪任务的机器人配置包。

这个目录本身不直接注册 Gym 任务，真正的任务注册在更深的机器人目录里，例如：
    * ``quadruped/deeprobotics_lite3``；
    * ``wheeled/deeprobotics_m20``。

保留这个 ``__init__.py`` 的原因是：Python 需要它把 ``config`` 识别成包，
上层自动导入任务配置时才能继续递归进入具体机器人目录。
"""

# 这里不主动 import 具体机器人配置，是为了避免用户只想导入 config 包时，
# 立刻触发所有 Gym 任务注册和较重的依赖加载。
