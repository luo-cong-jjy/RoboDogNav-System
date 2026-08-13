# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import rl_training.tasks.manager_based.locomotion.velocity.mdp as mdp
from rl_training.tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    ActionsCfg,
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

##
# Pre-defined configs
##
from rl_training.assets.deeprobotics import DEEPROBOTICS_M20_CFG  # isort: skip


@configclass
class DeeproboticsM20ActionsCfg(ActionsCfg):
    """M20 动作空间配置。

    M20 是轮足机器人：每条腿有 3 个腿部姿态关节 + 1 个轮子关节。
    因此控制上拆成两类动作：
        * joint_pos：12 个腿部关节目标位置；
        * joint_vel：4 个轮子关节目标速度。
    """

    # 腿部位置动作。具体 joint_names 会在 DeeproboticsM20RoughEnvCfg.__post_init__ 里改成 leg_joint_names。
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[""], scale=0.25, use_default_offset=True, clip=None, preserve_order=True
    )

    # 轮子速度动作。具体 joint_names 会在 __post_init__ 里改成 wheel_joint_names。
    joint_vel = mdp.JointVelocityActionCfg(
        asset_name="robot", joint_names=[""], scale=20.0, use_default_offset=True, clip=None, preserve_order=True
    )


@configclass
class DeeproboticsM20RewardsCfg(RewardsCfg):
    """M20 额外奖励项。

    通用 RewardsCfg 里已经有腿式机器人常见奖励。M20 还需要单独管理轮子相关项，
    因为轮子和腿部关节的物理意义不同：腿部更像姿态/支撑，轮子更像速度输出。
    """

    joint_vel_wheel_l2 = RewTerm(
        func=mdp.joint_vel_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names="")}
    )

    joint_acc_wheel_l2 = RewTerm(
        func=mdp.joint_acc_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names="")}
    )

    joint_torques_wheel_l2 = RewTerm(
        func=mdp.joint_torques_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names="")}
    )


@configclass
class DeeproboticsM20RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """M20 粗糙地形速度跟踪任务。

    这是学习 M20 强化学习配置最重要的文件。它继承通用 rough 速度跟踪环境，
    然后把“通用四足任务”改成“轮足 M20 任务”：
        * 指定 M20 USD 资产；
        * 明确 12 个腿部关节和 4 个轮子关节；
        * 设置观测、动作、随机化、奖励权重和速度命令范围。
    """

    actions: DeeproboticsM20ActionsCfg = DeeproboticsM20ActionsCfg()
    rewards: DeeproboticsM20RewardsCfg = DeeproboticsM20RewardsCfg()

    # base_link_name 用于定位机身，height scanner、质量随机化、COM 随机化等都会用到它。
    base_link_name = "base_link"
    # M20 的接触/足端名称使用 wheel，因为实际落地部件是轮子。
    foot_link_name = ".*_wheel"

    # fmt: off
    # 12 个腿部姿态关节：每条腿 hipx、hipy、knee。
    # 注意：这里不包含 wheel_joint，所以它不是 M20 的全部关节。
    leg_joint_names = [
        "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
        "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
        "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
        "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
    ]
    # 4 个轮子连续旋转关节。M20 全部可控关节 = 12 个腿部关节 + 4 个轮子关节 = 16。
    wheel_joint_names = [
        "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
    ]

    # 按关节类型分组，方便给不同关节设置不同奖励权重。
    hipx_joint_names = [
        "fl_hipx_joint", "fr_hipx_joint", "hl_hipx_joint", "hr_hipx_joint",
    ]

    hipy_joint_names = [
        "fl_hipy_joint", "fr_hipy_joint", "hl_hipy_joint", "hr_hipy_joint",
    ]

    knee_joint_names = [
        "fl_knee_joint", "fr_knee_joint", "hl_knee_joint", "hr_knee_joint",
    ]
    # 观测里会使用完整 16 个关节；动作里则拆成 leg_joint_names 和 wheel_joint_names 两组。
    joint_names = leg_joint_names + wheel_joint_names
    # fmt: on

    def __post_init__(self):
        """在通用 rough 环境基础上覆盖 M20 专属配置。"""
        # 先调用父类，建立通用 scene、observation、reward、event、curriculum 配置。
        super().__post_init__()

        # ------------------------------Scene------------------------------
        # 把通用环境里的 robot 占位替换成 M20 资产。DEEPROBOTICS_M20_CFG 内部指向 M20.usd。
        self.scene.robot = DEEPROBOTICS_M20_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # 高度扫描传感器挂在 base_link 上，随机身 yaw 方向旋转。
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name
        self.scene.height_scanner_base.prim_path = "{ENV_REGEX_NS}/Robot/" + self.base_link_name

        # ------------------------------Observations------------------------------
        # M20 的部署策略最终会导出到 sdk_deploy/policy.onnx。
        # 因此 actor/policy 观测最好只使用真机容易获得的量：IMU、关节位置/速度、速度命令、上一帧动作。
        # M20 轮子会连续转动，wheel_joint 的绝对角度没有稳定意义。
        # 所以 joint_pos 使用自定义函数：保留 16 维形状，但把 wheel joint 位置置零。
        self.observations.policy.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.policy.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )
        # critic 同样使用这个处理方式，保证轮子位置不把无意义的累计角度带进 value 估计。
        self.observations.critic.joint_pos.func = mdp.joint_pos_rel_without_wheel
        self.observations.critic.joint_pos.params["wheel_asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=self.wheel_joint_names
        )
        # 观测缩放用于把不同量纲的数据压到神经网络更容易学习的数值范围。
        self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05
        # policy 不直接使用 base_lin_vel。仿真里这个量很容易拿到，
        # 但真实机器人上不一定能准确测量，去掉它更贴近部署。
        self.observations.policy.base_lin_vel = None
        # M20 当前策略也不使用 height_scan，把地形感知压力更多交给随机化和本体反馈。
        self.observations.policy.height_scan = None
        # policy 能看到完整 16 个关节的相对位置/速度；其中 wheel 位置会在函数里置零。
        self.observations.policy.joint_pos.params["asset_cfg"].joint_names = self.joint_names
        self.observations.policy.joint_vel.params["asset_cfg"].joint_names = self.joint_names

        # ------------------------------Actions------------------------------
        # 这是 M20 和 Lite3 最大的区别：
        #   Lite3：12 维动作全部是腿关节位置目标；
        #   M20：16 维动作 = 12 个腿关节位置目标 + 4 个轮子速度目标。
        # 爬台阶时，腿关节负责调整机身高度/姿态/轮子接触位置，轮子负责持续提供滚动速度。
        # 动作缩放：
        # hipx 控制外展/内收，过大容易让腿外摆，所以 scale 小一些；
        # hipy/knee 控制前后摆和伸缩，scale 稍大。
        self.actions.joint_pos.scale = {".*_hipx_joint": 0.125, "^(?!.*_hipx_joint).*": 0.25}
        # 轮子动作是目标速度，策略输出会乘以 5.0 后成为 wheel_joint 目标速度。
        self.actions.joint_vel.scale = 5.0
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}
        self.actions.joint_vel.clip = {".*": (-100.0, 100.0)}
        # 关键点：腿部 12 维走 joint_pos，轮子 4 维走 joint_vel。
        self.actions.joint_pos.joint_names = self.leg_joint_names
        self.actions.joint_vel.joint_names = self.wheel_joint_names

        # ------------------------------Events------------------------------
        # 这些随机化是 sim-to-real 的关键：训练时故意让机器人面对不同初始姿态、
        # 不同摩擦、不同质心和不同质量，让策略不要只适应一个“完美仿真世界”。
        # reset 时随机化 base 初始姿态和速度，让策略学会从不同起始状态恢复。
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
        # base_link 单独做质量随机化；其他 body 用正则表达式排除 base_link。
        self.events.randomize_rigid_body_mass_base.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_rigid_body_mass.params["asset_cfg"].body_names = [
            f"^(?!.*{self.base_link_name}).*"
        ]
        # 对 base 质心和外力扰动做随机化，增强抗负载变化和外界扰动的能力。
        self.events.randomize_com_positions.params["asset_cfg"].body_names = [self.base_link_name]
        self.events.randomize_apply_external_force_torque.params["asset_cfg"].body_names = [self.base_link_name]

        # 调整粗糙地形难度范围。boxes/random_rough 是 Isaac Lab rough terrain 中的子地形。
        # boxes 可以理解成许多小台阶/小方块，random_rough 是连续起伏地面。
        # M20 的“能越台阶/爬楼梯”能力，主要就是在这类不平地形速度跟踪任务里学出来的。
        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.2)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.16)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # 摩擦和弹性随机化，用于提高 sim-to-real 鲁棒性。
        self.events.randomize_rigid_body_material.params["static_friction_range"] = [0.35, 1.5]
        self.events.randomize_rigid_body_material.params["dynamic_friction_range"] = [0.35, 1.5]
        self.events.randomize_rigid_body_material.params["restitution_range"] = [0.0, 0.7]

        # ------------------------------Rewards------------------------------
        # General
        self.rewards.is_terminated.weight = 0

        # Root penalties：约束机身不要乱跳、乱滚、离目标高度太远。
        self.rewards.lin_vel_z_l2.weight = -2.0
        self.rewards.ang_vel_xy_l2.weight = -0.02
        self.rewards.flat_orientation_l2.weight = 0
        self.rewards.base_height_l2.weight = -0.5
        self.rewards.base_height_l2.params["target_height"] = 0.40
        self.rewards.base_height_l2.params["asset_cfg"].body_names = [self.base_link_name]
        self.rewards.body_lin_acc_l2.weight = 0
        self.rewards.body_lin_acc_l2.params["asset_cfg"].body_names = [self.base_link_name]

        # Joint penalties：腿部关节和轮子关节分开处理。
        # 腿部更关注力矩、加速度、限位、功率、站立偏移；
        # 轮子更关注速度/加速度等旋转行为。
        self.rewards.joint_torques_l2.weight = -2.5e-5
        self.rewards.joint_torques_l2.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.joint_torques_wheel_l2.weight = 0
        self.rewards.joint_torques_wheel_l2.params["asset_cfg"].joint_names = self.wheel_joint_names
        self.rewards.joint_vel_l2.weight = 0
        self.rewards.joint_vel_l2.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.joint_vel_wheel_l2.weight = 0
        self.rewards.joint_vel_wheel_l2.params["asset_cfg"].joint_names = self.wheel_joint_names
        self.rewards.joint_acc_l2.weight = -2e-7
        self.rewards.joint_acc_l2.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.joint_acc_wheel_l2.weight = -1e-7
        self.rewards.joint_acc_wheel_l2.params["asset_cfg"].joint_names = self.wheel_joint_names
        # self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_hip_l1", -0.2, [".*_hip_joint"])
        self.rewards.joint_pos_limits.weight = -5.0
        self.rewards.joint_pos_limits.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.joint_vel_limits.weight = 0
        self.rewards.joint_vel_limits.params["asset_cfg"].joint_names = self.wheel_joint_names
        self.rewards.joint_power.weight = -2e-5
        self.rewards.joint_power.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.stand_still.weight = -2.0
        self.rewards.stand_still.params["asset_cfg"].joint_names = self.leg_joint_names
        self.rewards.hipx_joint_pos_penalty.weight = -0.4
        self.rewards.hipx_joint_pos_penalty.params["asset_cfg"].joint_names = self.hipx_joint_names
        self.rewards.hipy_joint_pos_penalty.weight = -0.1
        self.rewards.hipy_joint_pos_penalty.params["asset_cfg"].joint_names = self.hipy_joint_names
        self.rewards.knee_joint_pos_penalty.weight = -0.1
        self.rewards.knee_joint_pos_penalty.params["asset_cfg"].joint_names = self.knee_joint_names
        self.rewards.wheel_vel_penalty.weight = 0
        self.rewards.wheel_vel_penalty.params["sensor_cfg"].body_names = self.foot_link_name
        self.rewards.wheel_vel_penalty.params["asset_cfg"].joint_names = self.wheel_joint_names
        self.rewards.joint_mirror.weight = -0.03
        # 对角腿镜像约束：fl 对 hr，fr 对 hl，鼓励较自然的对称姿态。
        self.rewards.joint_mirror.params["mirror_joints"] = [
            ["fl_(hipx|hipy|knee).*", "hr_(hipx|hipy|knee).*"],
            ["fr_(hipx|hipy|knee).*", "hl_(hipx|hipy|knee).*"],
        ]

        # Action penalties：惩罚动作变化过快，减少抖动。
        self.rewards.action_rate_l2.weight = -0.01

        # Contact sensor：非轮端 body 接触地面会被视为不希望接触；轮端接触力过大也会惩罚。
        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]
        self.rewards.contact_forces.weight = -1.5e-4
        self.rewards.contact_forces.params["sensor_cfg"].body_names = [self.foot_link_name]

        # Velocity-tracking rewards：任务核心正奖励，鼓励跟踪 vx/vy 和 yaw_rate 指令。
        # 注意这里没有“爬楼梯专用奖励”。策略能上台阶，是因为在不平地形上保持速度跟踪、
        # 姿态稳定、接触不过猛这些目标组合起来，自然筛选出了能越障的动作模式。
        self.rewards.track_lin_vel_xy_exp.weight = 2.0 # 1.8
        self.rewards.track_ang_vel_z_exp.weight = 1.0 # 1.2

        # Others：M20 这里关闭了多数传统足式步态奖励，因为它是轮足机器人；
        # 当前主要依赖轮子速度跟踪、机身稳定和腿部姿态约束。
        self.rewards.feet_air_time.weight = 0
        self.rewards.feet_air_time.params["threshold"] = 0.5
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact.weight = 0
        self.rewards.feet_contact.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_contact_without_cmd.weight = 0.1
        self.rewards.feet_contact_without_cmd.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_stumble.weight = 0
        self.rewards.feet_stumble.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.weight = 0
        self.rewards.feet_slide.params["sensor_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_slide.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height.weight = 0
        self.rewards.feet_height.params["target_height"] = 0.1
        self.rewards.feet_height.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_height_body.weight = 0
        self.rewards.feet_height_body.params["target_height"] = -0.4
        self.rewards.feet_height_body.params["asset_cfg"].body_names = [self.foot_link_name]
        self.rewards.feet_gait.weight = 0
        self.rewards.feet_gait.params["synced_feet_pair_names"] = (("fl_wheel", "hr_wheel"), ("fr_wheel", "hl_wheel"))
        self.rewards.upward.weight = 0.08

        # 清理所有 weight=0 的奖励项，减少训练时计算开销。
        if self.__class__.__name__ == "DeeproboticsM20RoughEnvCfg":
            self.disable_zero_weight_rewards()

        # ------------------------------Terminations------------------------------
        # 当前 M20 配置关闭非法接触和姿态终止，避免训练过早结束；
        # 真实部署或更严格训练时可以重新打开。
        # self.terminations.illegal_contact.params["sensor_cfg"].body_names = [self.base_link_name]
        self.terminations.illegal_contact = None
        self.terminations.bad_orientation_2 = None

        # ------------------------------Curriculums------------------------------
        # 关闭速度指令课程，直接使用下面设置的完整命令范围。
        # self.curriculum.command_levels.params["range_multiplier"] = (0.2, 1.0)
        self.curriculum.command_levels = None

        # ------------------------------Commands------------------------------
        # M20 速度跟踪命令范围：前后速度、横向速度、偏航角速度。
        self.commands.base_velocity.ranges.lin_vel_x = (-2.0, 2.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 1.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
