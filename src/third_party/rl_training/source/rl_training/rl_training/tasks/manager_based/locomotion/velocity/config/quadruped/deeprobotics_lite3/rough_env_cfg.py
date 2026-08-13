# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass

from rl_training.tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg
# from isaaclab.sensors.ray_caster import GridPatternCfg
##
# Pre-defined configs
##
from rl_training.assets.deeprobotics import DEEPROBOTICS_LITE3_CFG  # isort: skip


@configclass
class DeeproboticsLite3RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Lite3 粗糙地形速度跟踪任务。

    这个配置继承通用 ``LocomotionVelocityRoughEnvCfg``，再把通用占位机器人替换成 Lite3。
    它和 M20 的最大区别是：Lite3 是纯四足机器人，没有轮子动作，策略只输出 12 个腿关节位置目标。
    """

    # Lite3 USD 中机身主链接名称。高度扫描、质量随机化、奖励等会用它定位 base。
    base_link_name = "TORSO"
    # 足端链接名称正则。接触传感器和足端奖励会用它筛选 FOOT body。
    foot_link_name = ".*_FOOT"
    # fmt: off
    # 12 个腿部关节，顺序必须和 USD/部署导出时的关节顺序保持一致。
    joint_names = [
        "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
        "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
        "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
        "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
    ]

    # 全部关键链接名称，主要用于质量随机化等按 body 名筛选的事件。
    link_names = [
       'TORSO', 
       'FL_HIP', 'FR_HIP', 'HL_HIP', 'HR_HIP', 
       'FL_THIGH', 'FR_THIGH', 'HL_THIGH', 'HR_THIGH', 
       'FL_SHANK', 'FR_SHANK', 'HL_SHANK', 'HR_SHANK', 
       'FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT',
    ]

    # 按关节类型分组，方便给不同关节组设置奖励权重。
    hipx_joint_names = [
        "FL_HipX_joint", "FR_HipX_joint", "HL_HipX_joint", "HR_HipX_joint",
    ]

    hipy_joint_names = [
        "FL_HipY_joint", "FR_HipY_joint", "HL_HipY_joint", "HR_HipY_joint",
    ]

    knee_joint_names = [
        "FL_Knee_joint", "FR_Knee_joint", "HL_Knee_joint", "HR_Knee_joint",
    ]
    # fmt: on

    def __post_init__(self):
        # 先创建通用 rough 速度跟踪环境：地形、传感器、默认 reward/event/curriculum 都在父类里建立。
        super().__post_init__()

        # ------------------------------Scene------------------------------
        # 把通用 robot 占位替换为 Lite3 USD 资产。
        self.scene.robot = DEEPROBOTICS_LITE3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # 高度扫描传感器挂到 TORSO 上，用于地形高度估计或 critic 辅助。
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        self.scene.height_scanner_base.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        # Lite3 尺寸比大型机器人小，扫描分辨率调细一些。
        self.scene.height_scanner.pattern_cfg.resolution = 0.07 #  = GridPatternCfg(resolution=0.07, size=[1.6, 1.0]),

        # ------------------------------Observations------------------------------
        # policy 不直接使用 base_lin_vel，减少对仿真真值速度的依赖，更接近真机部署。
        self.observations.policy.base_lin_vel = None # type: ignore
        # actor 不使用 height_scan，采用本体感知策略。
        self.observations.policy.height_scan = None # type: ignore
        # 角速度/关节速度缩放，控制网络输入量级。
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05
        # 明确观测使用 Lite3 的 12 个关节，且保持指定顺序。
        self.observations.policy.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.policy.joint_vel.params["asset_cfg"].joint_names = self.joint_names

        # ------------------------------Actions------------------------------
        # action 是 12 个关节位置目标。HipX 横摆自由度更容易让腿外撇，所以缩放更小。
        self.actions.joint_pos.scale = {".*_HipX_joint": 0.125, "^(?!.*_HipX_joint).*": 0.25}
        # clip 很宽，实际主要由动作 scale、关节限位和执行器限制约束。
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}
        # 动作只作用于 Lite3 的 12 个腿关节。
        self.actions.joint_pos.joint_names = self.joint_names

        # ------------------------------Events------------------------------
        # reset 时随机化初始位姿/速度，让策略学会从不同姿态恢复。
        self.events.randomize_reset_base.params = {
            "pose_range": {
                "x": (-1.0, 1.0),
                "y": (-1.0, 1.0),
                "z": (0.0, 0.0),
                "roll": (-0.3, 0.3),
                "pitch": (-0.3, 0.3),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.2, 0.2),
                "roll": (-0.05, 0.05),
                "pitch": (-0.05, 0.05),
                "yaw": (-0.0, 0.0),
            },
        }


        # 对所有列出的链接做质量随机化，提高对真实质量误差的鲁棒性。
        self.events.randomize_rigid_body_mass.params["asset_cfg"].body_names = self.link_names # [self.base_link_name]
        # self.events.randomize_rigid_body_mass_base.params["asset_cfg"].body_names = [self.base_link_name]
        # Lite3 这里不单独随机 base 质量，统一交给上面的整体 body 质量随机化。
        self.events.randomize_rigid_body_mass_base = None
        # 对 TORSO 质心做随机化，模拟电池/线束/负载导致的 COM 偏移。
        self.events.randomize_com_positions.params["asset_cfg"].body_names = self.base_link_name # [self.base_link_name]
        # self.events.randomize_com_positions = None
        # 关闭 reset 外力扰动和间歇推搡，避免训练初期过难。
        self.events.randomize_apply_external_force_torque = None
        self.events.randomize_push_robot = None
        # 执行器增益随机化作用于全部 12 个关节。
        self.events.randomize_actuator_gains.params["asset_cfg"].joint_names = self.joint_names

        # 地形比例：Lite3 当前更多训练随机粗糙和平滑坡，不训练 boxes/stairs。
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].proportion = 0.4
        self.scene.terrain.terrain_generator.sub_terrains["hf_pyramid_slope"].proportion = 0.3
        self.scene.terrain.terrain_generator.sub_terrains["hf_pyramid_slope_inv"].proportion = 0.3
        self.scene.terrain.terrain_generator.sub_terrains["boxes"].proportion = 0.0
        self.scene.terrain.terrain_generator.sub_terrains["pyramid_stairs"].proportion = 0.0
        self.scene.terrain.terrain_generator.sub_terrains["pyramid_stairs_inv"].proportion = 0.0
        # Lite3 机体小，因此 random_rough 高度范围也设置得比 M20 小。
        # self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        # self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_width = 0.8
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # ------------------------------Rewards------------------------------
        # 动作变化率惩罚：抑制策略输出抖动。
        self.rewards.action_rate_l2.weight = -0.1 #-0.02
        # self.rewards.smoothness_2.weight = -0.0075

        # 机身高度保持在约 0.55m，避免趴下或跳起。
        self.rewards.base_height_l2.weight = -50.0
        self.rewards.base_height_l2.params["target_height"] = 0.55
        self.rewards.base_height_l2.params["asset_cfg"].body_names = [self.base_link_name]

        # 足端腾空相关奖励：鼓励移动/转向时形成合理步态。
        self.rewards.feet_air_time_lin_xy.weight = 5.0 # 5.0
        self.rewards.feet_air_time_lin_xy.params["threshold"] = 0.5
        self.rewards.feet_air_time_lin_xy.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_air_time_x_neg.weight = 0.0 # 5.0
        self.rewards.feet_air_time_x_neg.params["threshold"] = 0.5
        self.rewards.feet_air_time_x_neg.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_air_time_ang_z.weight = 5.0 # 5.0
        self.rewards.feet_air_time_ang_z.params["threshold"] = 0.5
        self.rewards.feet_air_time_ang_z.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_air_time_variance.weight = -0.0 # -8.0
        self.rewards.feet_air_time_variance.params["sensor_cfg"].body_names = [self.foot_link_name]
        # 足端接触时滑动惩罚，减少支撑脚打滑。
        self.rewards.feet_slide.weight = -0.05
        self.rewards.feet_slide.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.params["asset_cfg"].body_names = [self.foot_link_name]
        # 落地向下速度惩罚，减少冲击。
        self.rewards.foot_impact_velocity.weight = -2.0 # -10.0
        self.rewards.foot_impact_velocity.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.foot_impact_velocity.params["asset_cfg"].body_names = [self.foot_link_name]
        # 站立时约束关节不要偏离默认姿态。
        self.rewards.stand_still.weight = -0.5 # -1.0
        self.rewards.stand_still.params["asset_cfg"].joint_names = self.joint_names
        self.rewards.stand_still.params["command_threshold"] = 0.1
        self.rewards.feet_height_body.weight = -0.0 # -2.5
        self.rewards.feet_height_body.params["target_height"] = -0.35
        self.rewards.feet_height_body.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height.weight = -0.0 # -0.2
        self.rewards.feet_height.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height.params["target_height"] = 0.05
        # 接触力惩罚，鼓励落脚更轻。
        self.rewards.contact_forces.weight = -1e-1 # -2e-2
        self.rewards.contact_forces.params["sensor_cfg"].body_names = [self.foot_link_name]

        # 机身上下速度和 roll/pitch 角速度惩罚，用来压住颠簸。
        self.rewards.lin_vel_z_l2.weight = -20.0 #-2.0
        self.rewards.ang_vel_xy_l2.weight = -0.25 # -0.05

        # 速度跟踪是任务核心正奖励。
        self.rewards.track_lin_vel_xy_exp.weight = 4.0
        self.rewards.track_ang_vel_z_exp.weight = 1.5

        # 非足端 body 接触地面会被惩罚。
        self.rewards.undesired_contacts.weight = -0.5
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]

        # 能耗/关节平滑/姿态约束。
        self.rewards.joint_torques_l2.weight = -2.5e-4
        self.rewards.joint_acc_l2.weight = -1e-8
        self.rewards.joint_deviation_l1.weight = -0.0
        self.rewards.joint_deviation_l1.params["asset_cfg"].joint_names = [".*HipX.*"]
        self.rewards.joint_power.weight = -8e-4
        self.rewards.flat_orientation_l2.weight = -20.0

        # 以下奖励用于显式改善步态时序和足端轨迹。
        self.rewards.feet_gait.weight = 0.5
        self.rewards.feet_gait.params["synced_feet_pair_names"] = [
            ["FL_FOOT", "HR_FOOT"],
            ["FR_FOOT", "HL_FOOT"]
        ]

        self.rewards.phase_foot_trajectory_exp.weight = 2.0
        self.rewards.phase_foot_trajectory_exp.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.joint_mirror.weight = -0.05
        self.rewards.joint_mirror.params["mirror_joints"] = [
            ["FL_(HipX|HipY|Knee).*", "HR_(HipX|HipY|Knee).*"],
            ["FR_(HipX|HipY|Knee).*", "HL_(HipX|HipY|Knee).*"],
        ]

        self.rewards.joint_pos_limits.weight = -5.0
        # self.rewards.joint_pos_penalty.weight = -1.0
        # 无命令站立时鼓励足端保持接触。
        self.rewards.feet_contact_without_cmd.weight = 0.1
        self.rewards.feet_contact_without_cmd.params["sensor_cfg"].body_names = [self.foot_link_name]

        # 针对不同关节组的默认姿态偏离惩罚。
        self.rewards.hipx_joint_pos_penalty.weight = -0.4
        self.rewards.hipx_joint_pos_penalty.params["asset_cfg"].joint_names = self.hipx_joint_names
        self.rewards.hipy_joint_pos_penalty.weight = -0.0
        self.rewards.hipy_joint_pos_penalty.params["asset_cfg"].joint_names = self.hipy_joint_names
        self.rewards.knee_joint_pos_penalty.weight = -2
        self.rewards.knee_joint_pos_penalty.params["asset_cfg"].joint_names = self.knee_joint_names


        # 清理 weight=0 的奖励项，减少 RewardManager 计算开销。
        if self.__class__.__name__ == "DeeproboticsLite3RoughEnvCfg":
            self.disable_zero_weight_rewards()

        # ------------------------------Terminations------------------------------
        # 关闭非法接触终止，避免 rough 初期因为偶发接触过早结束。
        self.terminations.illegal_contact = None
        # self.terminations.bad_orientation_2 = None

        # ------------------------------Curriculums------------------------------
        # self.curriculum.command_levels.params["range_multiplier"] = (0.2, 1.0)
        # 关闭速度命令课程，直接使用下面的完整命令范围。
        self.curriculum.command_levels = None

        # ------------------------------Commands------------------------------
        # Lite3 速度命令范围，比 M20 略保守。
        self.commands.base_velocity.ranges.lin_vel_x = (-1.5, 1.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.8, 0.8)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.8, 0.8)
