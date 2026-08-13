# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from .rough_env_cfg import DeeproboticsLite3RoughEnvCfg


@configclass
class DeeproboticsLite3FlatEnvCfg(DeeproboticsLite3RoughEnvCfg):
    """Lite3 平地速度跟踪任务。

    Flat 任务继承 Rough 任务，然后把地形生成器和地形课程学习关掉。
    初学者调试时通常先跑 Flat：如果平地都跑不起来，就不用急着看 Rough。
    """

    def __post_init__(self):
        # 先执行 RoughEnvCfg 的完整初始化，得到机器人、动作、观测、奖励等默认配置。
        super().__post_init__()

        # 下面开始把 rough 专属内容改成 flat。
        # self.rewards.base_height_l2.params["sensor_cfg"] = None
        # terrain_type="plane" 表示使用平面，而不是程序化粗糙地形。
        self.scene.terrain.terrain_type = "plane"
        # 平地不需要 terrain_generator。
        self.scene.terrain.terrain_generator = None
        # 平地通常不需要给 actor 高度扫描观测。
        # self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # self.observations.critic.height_scan = None
        # 没有地形等级，自然也不需要 terrain curriculum。
        self.curriculum.terrain_levels = None

        # 清理 weight=0 的 reward，减少无效计算。
        if self.__class__.__name__ == "DeeproboticsLite3FlatEnvCfg":
            self.disable_zero_weight_rewards()
