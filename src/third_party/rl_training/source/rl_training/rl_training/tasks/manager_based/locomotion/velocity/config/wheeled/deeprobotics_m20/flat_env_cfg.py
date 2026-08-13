# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from .rough_env_cfg import DeeproboticsM20RoughEnvCfg


@configclass
class DeeproboticsM20FlatEnvCfg(DeeproboticsM20RoughEnvCfg):
    """M20 平地速度跟踪任务。

    Flat 环境继承 Rough 环境，然后把地形相关内容关掉。
    对初学者来说，建议先用 Flat 跑通训练、play、日志和 checkpoint，
    再切到 Rough 研究粗糙地形和随机化。
    """

    def __post_init__(self):
        # 先继承 M20 rough 的机器人、动作、观测、奖励等配置。
        super().__post_init__()

        # 平地没有高度扫描传感器参与 base height 计算，直接使用世界系高度目标。
        self.rewards.base_height_l2.params["sensor_cfg"] = None
        # 把粗糙地形 generator 换成无限平面。
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # 平地环境不需要高度扫描。
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None
        # 平地没有地形等级，自然也不需要地形课程学习。
        self.curriculum.terrain_levels = None

        # 清理 weight=0 的奖励项。
        if self.__class__.__name__ == "DeeproboticsM20FlatEnvCfg":
            self.disable_zero_weight_rewards()
