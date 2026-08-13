# Copyright (c) 2025 Deep Robotics
# SPDX-License-Identifier: BSD 3-Clause

# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import inspect #导入 Python 的内置检查模块。它提供了获取对象（如模块、类、函数）信息的工具，常用于查看源码、获取参数签名或动态分析模块结构。
import math
import sys
from dataclasses import MISSING  #来自 dataclasses，用于标记某些配置项是“必填的”，如果用户忘了配置就会报错。
from isaaclab.sim import PhysxCfg, SimulationCfg #配置底层物理引擎（如重力、时间步长、物理求解器参数等）
import isaaclab.sim as sim_utils  #导入 Isaac Lab 的仿真工具模块。这个模块包含了在 Omniverse 中生成对象、修改物理属性、控制仿真器（如 SimulationContext）等底层功能。
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  #导入资产配置类。ArticulationCfg 用于定义机器人本体（如关节连接的刚体、执行器模型等），而 AssetBaseCfg 是资产的基础配置类。
from isaaclab.envs import ManagerBasedRLEnvCfg #这是最核心的基类。你后续定义的环境配置类必须继承它。它规定了强化学习环境的标准结构。
from isaaclab.managers import CurriculumTermCfg as CurrTerm #定义课程学习（比如随着训练进度，逐渐增加指令的速度或地形的难度）。
from isaaclab.managers import EventTermCfg as EventTerm #定义随机化事件（比如训练时给机器人施加随机推力、改变摩擦力，用于 Sim-to-Real 迁移）。
from isaaclab.managers import ObservationGroupCfg as ObsGroup #定义给神经网络的观测值（比如机器人的关节角度、速度等）。
from isaaclab.managers import ObservationTermCfg as ObsTerm #定义给神经网络的观测值（比如机器人的关节角度、速度等）。
from isaaclab.managers import RewardTermCfg as RewTerm  #定义奖励函数（比如鼓励向前跑、惩罚摔倒等）。
from isaaclab.managers import SceneEntityCfg #用于在配置中引用场景里的具体实体（比如在奖励函数里指定“只惩罚左前腿的碰撞”）。
from isaaclab.managers import TerminationTermCfg as DoneTerm #(别名 DoneTerm): 定义终止条件（比如机器人倒地、超时，此时回合结束）。
from isaaclab.scene import InteractiveSceneCfg  #用于配置仿真场景中的实体（如机器人、地形、光源等）。
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns # ContactSensorCfg & RayCasterCfg：配置足端接触传感器和射线扫描器（用于感知地形高度）。patterns: 传感器的一些预设采样模式。
from isaaclab.terrains import TerrainImporterCfg  #配置地形生成器（如平地、粗糙地形、楼梯等）。
from isaaclab.utils import configclass #Isaac Lab 提供的一个装饰器，用来把普通的 Python dataclass 变成支持合并、覆盖的配置类。
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR  #导入官方资产的云端路径常量。这些常量指向托管在 Nucleus 服务器上的机器人模型、地形等 USD 文件，方便在配置中直接引用。
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise  #导入均匀噪声配置类。在强化学习中，通常用于给观测值（Observations）添加噪声，以提高模型在真实环境中的鲁棒性。

import rl_training.tasks.manager_based.locomotion.velocity.mdp as mdp #非常重要！ 这是你当前任务目录下的自定义模块（import rl_training.tasks...mdp as mdp）。里面通常包含了你专门为这个机器人写的奖励函数、观测函数和事件函数。

##
# Pre-defined configs
##
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # isort: skip，导入 Isaac Lab 官方预设的粗糙地形配置，方便你直接拿来用。


##
# Scene definition
##


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """训练场景配置。

    初学者可以把 Scene 理解成“仿真世界里有什么东西”：
        * terrain：地面或粗糙地形；
        * robot：机器人本体，具体机器人模型由子类配置填入；
        * sensors：高度扫描、接触力等传感器；
        * lights：灯光，只影响可视化。
    """

    # 地形。默认使用 Isaac Lab 的 rough terrain generator，M20 的 flat 配置会把它改成 plane。
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",# 4. 指定地形对象在 USD 场景图（Stage）中的唯一路径
        terrain_type="generator",# 5. 地形类型设为 "generator"，表示使用程序化生成器来动态创建地形
        terrain_generator=ROUGH_TERRAINS_CFG, #指定具体的地形生成器配置，这里使用了 Isaac Lab 预设的粗糙地形配置
        max_init_terrain_level=5, #限制训练初期的地形最大复杂度等级，配合课程学习（Curriculum）使用
        collision_group=-1, # 8. 碰撞组设置，-1 表示该地形会与场景中所有其他碰撞组发生碰撞
        physics_material=sim_utils.RigidBodyMaterialCfg(  
            friction_combine_mode="multiply",# 摩擦力合并模式：当两个物体接触时，摩擦系数采用相乘的方式结合
            restitution_combine_mode="multiply",#  弹性系数（恢复系数）合并模式：同样采用相乘的方式结合
            static_friction=1.0,#  静摩擦系数：防止机器人静止时发生滑动的阻力
            dynamic_friction=1.0,#动摩擦系数：机器人在地形上运动时受到的摩擦阻力
            restitution=1.0,# 弹性系数：控制碰撞后的反弹程度，1.0 表示完全弹性碰撞
        ),
        visual_material=sim_utils.MdlFileCfg(#视觉材质配置：定义地形的外观渲染效果
            #材质文件路径：使用 NVIDIA Nucleus 提供的大理石砖纹理材质
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,#启用 UV 纹理投影：让纹理自动贴合地形表面的几何形状
            texture_scale=(0.25, 0.25),#纹理缩放：控制纹理在表面重复的密度，(0.25, 0.25) 表示纹理缩小并密集排列
        ),
        debug_vis=False,#调试可视化：设为 False 表示关闭地形原点等调试标记的显示
    )
    # 机器人资产占位。这里先写 MISSING，具体用 M20 还是 Lite3，由各机器人 rough_env_cfg.py 设置。
    robot: ArticulationCfg = MISSING
    # 高度扫描传感器：向地面打射线，用于感知前方/周围地形高度。
    # M20 当前 policy 观测中关闭了 height_scan，但 critic 或其他机器人任务仍可能使用。
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base", # 传感器挂载的路径。{ENV_REGEX_NS} 是 Isaac Lab 的并行环境命名空间占位符，运行时会自动替换为 /World/envs/env_0/Robot/base 等，实现多环境并行克隆。
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),## 4. 传感器相对于机器人基座（base）的偏移量。这里设置在机器人正上方 20 米处向下发射射线，确保能覆盖机器人的整个活动范围。
        ray_alignment="yaw",#射线对齐方式设为 "yaw"（偏航角）。表示射线网格会跟随机器人的朝向旋转，但忽略机器人的俯仰（pitch）和横滚（roll），这样无论机器人怎么倾斜，感知网格始终平行于地面。
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),#射线网格的采样模式配置。分辨率为 0.1 米（每 10 厘米打一根射线），网格大小为 1.6m x 1.0m（长 x 宽）。
        debug_vis=False,#关闭调试可视化（设为 True 会在仿真界面中画出射线的落点）。
        mesh_prim_paths=["/World/ground"],#指定射线需要检测碰撞的网格对象。这里指向了前面定义的地形 "/World/ground"。
    )
    height_scanner_base = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=(0.1, 0.1)),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
    # 接触传感器：记录机器人各 body 与环境的接触力、离地时间、接触时间等。
    # 许多足端/轮端奖励和非法接触终止条件都依赖它。
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    # 场景灯光。训练物理本身不依赖它，主要为了 play 时可视化舒服。
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """速度指令配置。

    强化学习策略不是“自由乱动”，而是要跟踪 command。
    这里的 base_velocity 会周期性随机采样 ``vx, vy, yaw_rate``，
    然后这些指令会作为 observation 的一部分输入给策略网络。
    """

    #定义基座速度指令生成器
    base_velocity = mdp.UniformThresholdVelocityCommandCfg(
        asset_name="robot",#指定该指令作用于哪个资产，这里对应场景中名为 "robot" 的机器人
        resampling_time_range=(10.0, 10.0),#指令重采样的时间范围（秒）。这里设为 (10.0, 10.0)，表示每隔固定的 10 秒，系统会重新随机生成一个新的目标速度指令。
        rel_standing_envs=0.02,#【核心训练技巧】零速度（站立）指令的概率。设为 0.02 表示有 2% 的环境会收到速度为 0 的指令。# 这能强制策略学会“稳定站立不动”的能力，防止它只会跑不会停
        rel_heading_envs=1.0,#使用朝向（Heading）指令模式的比例。设为 1.0 表示 100% 的环境都采用“目标朝向”模式，而不是直接控制角速度。
        heading_command=True,#启用朝向命令模式。机器人会根据当前朝向与目标朝向的误差，自动计算出一个角速度来对齐目标方向。
        heading_control_stiffness=0.5, #朝向控制的刚度参数。决定了机器人纠正朝向误差时的响应速度/力度。
        debug_vis=True,  #开启调试可视化。设为 True 时，仿真界面会画出目标速度的箭头，方便观察。
        ranges=mdp.UniformThresholdVelocityCommandCfg.Ranges(#定义指令的采样范围（Ranges），前后线速度范围：在 -1.0 到 1.0 m/s 之间随机采样，左右线速度范围：在 -1.0 到 1.0 m/s 之间随机采样，偏航角速度范围：在 -1.0 到 1.0 rad/s 之间随机采样，目标朝向范围：在 -π 到 π 弧度（即 360 度全向）之间随机采样
            lin_vel_x=(-1.0, 1.0), lin_vel_y=(-1.0, 1.0), ang_vel_z=(-1.0, 1.0), heading=(-math.pi, math.pi)
        ),
    )


@configclass
class ActionsCfg:
    """动作空间配置。

    通用版本默认只配置关节位置动作。具体机器人可以继承后改写：
    M20 会把动作拆成腿部 ``JointPositionAction`` 和轮子 ``JointVelocityAction``。
    """

    # JointPositionActionCfg 的含义：
    # 策略输出 action，环境把它按 scale 缩放，并在 use_default_offset=True 时叠加到默认关节角上，
    # 最终形成目标关节位置，再交给执行器/PD 控制。
    # 指定该动作作用于哪个资产，这里对应场景中名为 "robot" 的机器人
    # 指定要控制的关节。[".*"] 是正则表达式，表示匹配机器人上的所有关节
    # . 【核心缩放参数】动作缩放因子。可以和汽车方向盘的转向角度类似理解
    # 神经网络输出的值通常在 [-1, 1] 之间，乘以 0.5 后，实际的目标关节角度
    # 偏移量会被限制在 [-0.5, 0.5] 弧度内。这能防止网络输出过大导致机器人动作剧烈而摔倒。

    # 【核心偏移参数】是否使用默认关节角作为基准。
    # 设为 True 时，最终的目标关节位置 = 机器人的默认初始关节角 + (网络输出 × scale)。
    # 这能让机器人围绕一个自然的站立姿态进行微调，而不是从零度开始扭曲。
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True, clip=None, preserve_order=True
    )


@configclass
class ObservationsCfg:
    """观测配置。

    Observation 是策略网络真正能看到的输入。这里分成 policy 和 critic 两组：
        * policy：actor 使用，部署时也需要尽量能在真机上获得；
        * critic：训练时辅助估值，可以包含更完整/更干净的信息。

    每个 ObsTerm 都包含：
        * func：如何从 env 中取数据；
        * noise：训练时加的观测噪声；
        * clip：裁剪范围；
        * scale：数值缩放。
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """actor/policy 的观测组。"""

        # 注意：observation terms 的顺序会影响最终拼接后的网络输入顺序。
        # 如果导出策略或部署，必须保持同样的观测顺序和缩放。
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,   # 调用内置函数，获取机器人基座在世界坐标系下的线速度 (vx, vy, vz)
            noise=Unoise(n_min=-0.1, n_max=0.1),   # 添加均匀噪声：在 [-0.1, 0.1] 之间随机加干扰。目的：模拟真实传感器的误差，防止网络过度依赖精确速度（增强鲁棒性）
            clip=(-100.0, 100.0),# 裁剪范围：如果速度超过这个值，强制截断。防止极端异常值破坏网络，就像是一个极其宽泛的安全网。它告诉神经网络：“只要你输出的不是 NaN 或者几百万这种离谱的数字，我都照单全收。” 真正限制机器人速度的，是物理引擎里的电机扭矩限制和摩擦力，而不是这个 clip。
            scale=1.0, # 缩放因子：将原始速度乘以 1.0。如果真实速度范围是 0~2，可以设为 0.5 映射到 0~1
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, #角速度
            noise=Unoise(n_min=-0.2, n_max=0.2),
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        projected_gravity = ObsTerm( 
            func=mdp.projected_gravity,  #投影重力（Projected Gravity）,【核心作用】：它相当于机器人的“内耳前庭”，让机器人知道哪边是“下”，自己有没有倾斜
            noise=Unoise(n_min=-0.05, n_max=0.05), # 模拟真实 IMU 的静态漂移噪声
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,  # 获取由前面 `base_velocity` 指令生成器生成的目标速度 (vx_cmd, vy_cmd, yaw_cmd),【核心作用】：这是机器人的“任务目标”，告诉它现在该跑多快、往哪拐
            # velocity_commands 里的 vx, vy 和 base_lin_vel 里的 vx, vy 在物理上指的是同一个东西（本体坐标系下的前后、左右平移速度），但在强化学习中，一个是“理想目标”，一个是“骨感现实”。神经网络正是通过对比这两者，才学会了如何精准地控制机器人！
            params={"command_name": "base_velocity"},  # 指定要获取哪个指令生成器的数据
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,#关节相对位置（Relative Joint Positions），# 获取所有关节的当前角度，减去“默认站立角度（Default Joint Pos）”。【核心作用】：用相对值而不是绝对值，让网络更容易理解“腿是伸直还是弯曲”
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            # asset_cfg: 指定从名为 "robot" 的资产中获取数据
            # joint_names=".*": 正则表达式，匹配所有关节
            # preserve_order=True: 【极其重要】强制保持关节在 URDF/USD 中的原始物理顺序。
            #                      如果不加这个，Python 可能会按字母表排序，导致真机和仿真关节顺序错乱！
            noise=Unoise(n_min=-0.01, n_max=0.01),
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,  # 关节相对速度（Relative Joint Velocities），获取所有关节的角速度（同样通常是相对默认状态的，或绝对值）【核心作用】：让网络知道腿是在“快速伸出”还是“缓慢收回”，用于防抖和缓冲
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            noise=Unoise(n_min=-1.5, n_max=1.5),# 速度噪声给得比较大（±1.5），因为真实电机测速时高频噪声很明显
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        actions = ObsTerm(
            func=mdp.last_action,  # 获取神经网络在上一帧输出的动作。# 【核心作用】：让网络有“记忆”。知道上一秒自己做了什么，这一秒才能做出平滑的过渡，防止动作突变
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        height_scan = ObsTerm(
            func=mdp.height_scan,  # 获取地形高度扫描仪的数据（通常是基座前方/周围的一排点云高度）【核心作用】：机器人的“眼睛”，让它提前看到前方的台阶、坑洼，从而提前调整步态
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},  # 指定使用场景中的高度扫描仪传感器
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
            scale=1.0,
        )

        def __post_init__(self):
            # policy 训练时开启 corruption/noise，有助于增强鲁棒性。
            # 【核心训练技巧】policy 训练时开启 corruption/noise，有助于增强鲁棒性。
            # 设为 True 时，上面配置的所有 `noise=Unoise(...)` 才会真正生效。
            # 设为 False 时，所有噪声被屏蔽，通常用于评估（Evaluation）或真机部署前的测试。
            self.enable_corruption = True
            # concatenate_terms=True 表示把上面的各项按顺序拼成一个长向量，作为神经网络输入。
            # 【核心拼接机制】concatenate_terms=True 表示把上面的 8 个观测项，
            # 按照代码中定义的上下顺序，首尾相连拼成一个长长的一维向量（1D Tensor）。
            # 例如：[vx, vy, vz, wx, wy, wz, gx, gy, gz, cmd_x, ..., joint_1, ..., height_1, ...]
            # 这个长向量就是神经网络真正的输入（Input）。
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """critic/value function 的观测组。

        Critic 只在训练时使用，用来估计价值函数。它可以不加噪声，
        甚至在某些任务中可以看到比 policy 更多的信息。
        """

        # 和 policy 类似，但默认没有 observation noise。
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        actions = ObsTerm(
            func=mdp.last_action,
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
            scale=1.0,
        )
        # joint_effort = ObsTerm(
        #     func=mdp.joint_effort,
        #     clip=(-100, 100),
        #     scale=0.01,
        # )

        def __post_init__(self):
            # critic 训练时通常不加噪声，让价值估计更稳定。
            self.enable_corruption = False
            self.concatenate_terms = True

    # 注册两个观测组。RSL-RL 会用 policy 组给 actor，critic 组给 critic。
    # Critic 需要这些输入，是因为它要当“上帝视角的裁判”，必须看懂局势才能打分；它不加噪声，是为了保证打分的绝对客观。
    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    """事件配置。

    Event 是 Isaac Lab 中“在特定时机自动执行的函数”：
        * startup：环境创建时执行一次，例如随机化摩擦/质量；
        * reset：某些 env reset 时执行，例如重置关节、随机初始姿态；
        * interval：训练过程中按时间间隔执行，例如随机推机器人。

    这些事件常用于 domain randomization，让策略不要只适应一个固定仿真世界。
    """

    # startup：仿真启动时随机化地面/机器人材质摩擦。
    # 【作用】真实世界的地面可能是大理石、草地、泥地或瓷砖。如果仿真里只训练了完美的橡胶地面，真机一上冰面就会劈叉。
    randomize_rigid_body_material = EventTerm(
        func=mdp.randomize_rigid_body_material, # 调用内置的随机化材质函数
        mode="startup",  # 触发时机：仿真启动时 表示在整个仿真会话开始时只随机化一次，之后保持不变
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),  # 指定对名为 "robot" 的资产中，所有匹配 ".*" (即全部) 的刚体生效
            "static_friction_range": (0.3, 1.0),  # 静摩擦系数范围：0.3(较滑) 到 1.0(很涩)。逼迫网络学会在打滑边缘找回抓地力
            "dynamic_friction_range": (0.3, 0.8), # 动摩擦系数范围：滑动时的摩擦力，通常小于静摩擦
            "restitution_range": (0.0, 0.4),  # 恢复系数（弹性）范围：0.0 表示完全非弹性碰撞（落地不弹），0.4 表示有一定弹性
            "num_buckets": 1024,  # 物理引擎内部用于处理摩擦的哈希桶数量，1024 是默认的高效配置
        },
    )

    # 随机化非 base 刚体质量。M20 子类会设置 body_names，决定作用在哪些 body 上。
    # 【作用】真实机器人可能会加装雷达、相机等设备，导致四肢质量发生变化。
    randomize_rigid_body_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,  # 调用随机化质量函数
        mode="startup",                     # 触发时机：仿真启动时
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=""), # 注意这里 body_names 为空字符串，具体作用在哪些部件上，通常由 M20 的子类配置去指定（比如只随机化腿部）
            "mass_distribution_params": (0.85, 1.15),  # 质量分布参数：在原始质量的 85% 到 115% 之间随机
            "operation": "scale",  # 操作方式：scale（乘法缩放）。即 新质量 = 原质量 × 随机系数
            "recompute_inertia": True,  # 【极其重要】质量改变后，必须重新计算转动惯量矩阵，否则物理引擎的受力计算会出错
        }, 
    )

    # 随机化 base 质量。base 质量对整机质心和姿态稳定影响很大。
    # 【作用】专门针对机器人的躯干（Base）进行质量随机化，模拟电池消耗、背负重物等情况。
    randomize_rigid_body_mass_base = EventTerm(
        func=mdp.randomize_rigid_body_mass,  # 同样调用随机化质量函数
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=""), # 同样由子类指定具体作用在 Base 上
            "mass_distribution_params": (-1.0, 3.0),  # 质量分布参数：这里跨度非常大！允许减少 1kg 或增加 3kg
            "operation": "add", # 【注意】操作方式变成了 add（加法）。即 新质量 = 原质量 + 随机值。因为 Base 质量变化通常是绝对值变化
            "recompute_inertia": True,   # 重新计算转动惯量
        },
    )

    # 随机化惯量，提升对建模误差的鲁棒性。
    # 【作用】CAD 模型算出来的惯量往往不准，随机化惯量能防止网络对完美的转动惯量产生过拟合。
    randomize_rigid_body_inertia = EventTerm(
        func=mdp.randomize_rigid_body_inertia,  # 调用随机化惯量函数
        mode="startup",                         # 触发时机：仿真启动时
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),  # 对所有刚体生效
            "inertia_distribution_params": (0.85, 1.15),   # 惯量分布参数：在原始惯量的 85% 到 115% 之间随机
            "operation": "scale",    # 操作方式：乘法缩放
        }, 
    )

    # 随机化质心位置，模拟负载、电池位置、建模误差等。
    # 【作用】真实机器人不是完美的对称体。电池没电了、线缆的走向都会导致质心偏移，几厘米的偏移对平衡影响极大。
    randomize_com_positions = EventTerm(
        func=mdp.randomize_rigid_body_com,   # 调用随机化质心函数
        mode="startup",                      # 触发时机：仿真启动时
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),  # 对所有刚体生效
            "com_range": {"x": (-0.03, 0.03), "y": (-0.03, 0.03), "z": (-0.02, 0.02)},  # 质心偏移范围（单位：米）# X轴（前后）偏移 ±3 厘米等等
        },
    )

    # reset：回合重置时施加随机外力/力矩。训练 play 时通常会关闭。
    # 【作用】在每次“死亡”重新开始（Reset）时，给机器人一记“闷棍”，测试它的起步抗冲击能力。
    randomize_apply_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,   # 调用施加外力/力矩函数
        mode="reset",                           # 触发时机：环境重置时
        params={ 
            "asset_cfg": SceneEntityCfg("robot", body_names=""),    # 由子类指定作用在哪些部件上（通常是 Base）
            "force_range": (-10.0, 10.0),      # 随机外力范围：X/Y/Z 轴各施加 -10N 到 10N 的力
            "torque_range": (-10.0, 10.0),     # 随机力矩范围：X/Y/Z 轴各施加 -10Nm 到 10Nm 的扭矩
        },
    )

    # reset 时重置关节状态。这里 position_range=(1,1) 基本表示使用默认位置。
    # 【作用】控制机器人每次重新开始时，关节的初始姿态。
    randomize_reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,  # 调用按缩放重置关节的函数（也可以用
        # func=mdp.reset_joints_by_offset,  # 被注释掉的备选方案
        mode="reset",                       # 触发时机：环境重置时
        params={
            "position_range": (1.0, 1.0),  # 【注意】缩放范围是 (1.0, 1.0)，意味着 新角度 = 默认角度 × 1.0。即完全没有随机化，每次都在标准站立姿势开始
            "velocity_range": (0.0, 0.0),  # 初始关节速度范围：(0.0, 0.0) 表示每次起步时关节都是绝对静止的
        },
    )

    # reset 时随机化执行器 PD 增益，避免策略过度依赖某个精确刚度/阻尼。
    # 【作用】真机电机的 Kp/Kd 会随温度、老化变化。这是解决 Sim-to-Real 鸿沟最关键的一环！
    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,  # 调用随机化 PD 增益函数
        mode="reset",                       # 触发时机：环境重置时（每个回合的电机特性都不一样）
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),   # 对所有关节生效
            "stiffness_distribution_params": (0.85, 1.15),   # 刚度(Kp)分布参数：在原始 Kp 的 85% 到 115% 之间随机
            "damping_distribution_params": (0.85, 1.15),     # 阻尼(Kd)分布参数：在原始 Kd 的 85% 到 115% 之间随机
            "operation": "scale",     # 操作方式：乘法缩放
            "distribution": "uniform",   # 分布类型：均匀分布（uniform），即范围内的每个值被抽到的概率相等
        },
    )

    # reset 机器人根位姿和速度，例如随机 yaw、roll/pitch、小速度扰动。
    # 【作用】训练抗干扰能力的基石！强迫机器人学会从各种倾斜、各种初速度下瞬间找回平衡。
    randomize_reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,  # 调用均匀分布重置根状态的函数
        mode="reset",                       # 触发时机：环境重置时
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},    # 初始位姿随机范围，# X轴位置偏移 ±0.5 米
            "velocity_range": {  # 初始速度随机范围，X轴初速度 ±0.5 m/s之间
                "x": (-0.5, 0.5),           
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )

    # interval：每隔一段时间随机“推”机器人，训练抗扰动能力。
    # 【作用】模拟真实世界中被人踢了一脚、被风吹了一下等动态干扰。
    randomize_push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,   # 调用通过设置速度来“推”机器人的函数
        mode="interval",                        # 触发时机：按固定时间间隔触发
        interval_range_s=(10.0, 15.0),      # 触发间隔：每隔 10 秒到 15 秒之间随机触发一次
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},  # 推动的速度范围（直接给基座一个瞬时速度），# X轴推力 ±0.5 m/s，# Y轴推力 ±0.5 m/s
    )


@configclass
class RewardsCfg:
    """奖励项配置。

    这里只定义“有哪些奖励项”和它们默认参数。默认 weight 大多是 0，
    具体机器人配置会在自己的 rough_env_cfg.py 中把需要的项打开并赋权重。

    读 reward 时要记住：
        ``实际贡献 = weight * reward_function(env)``

    因此 weight 为负时，该项就是惩罚。
    """

    # 1. 通用/终止状态奖励 (General / Termination)
    # General：是否因终止而给惩罚/奖励。
    is_terminated = RewTerm(func=mdp.is_terminated, weight=0.0)
    # 当环境被终止（比如机器人摔倒、超时）时给予的奖励/惩罚。weight=0.0：默认关闭。通常我们会给一个负权重（如 -2.0），表示“如果你摔倒了，就扣分”。

    # 2. 机体（Root/Base）惩罚项
    #    针对机器人躯干（身体）的运动和姿态进行约束，防止它乱扭或乱跳。
    # Root penalties：和机体根节点运动/姿态相关的惩罚。

    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=0.0)
    # lin_vel_z_l2：惩罚 Z 轴（垂直方向）的速度。
    # 目的：防止机器人跳来跳去，鼓励平稳落地。L2 范数意味着偏差越大，惩罚越重（平方级）。

    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=0.0)
    # ang_vel_xy_l2：惩罚 X/Y 轴的角速度（即左右倾斜和前后翻滚的速度）。
    # 目的：鼓励机器人保持姿态稳定，不要像喝醉了一样乱晃。

    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=0.0)
    # flat_orientation_l2：惩罚机体姿态偏离“水平”。
    # 目的：强制机器人学会保持身体平直，不要翻车。


    base_height_l2 = RewTerm(
        func=mdp.base_height_l2,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=""),
            "sensor_cfg": SceneEntityCfg("height_scanner_base"),
            "target_height": 0.0,
        },
    )
    # base_height_l2：惩罚机体高度偏离目标值。
    # params.target_height=0.0：目标高度是 0（通常需要根据具体机器人调整，比如 0.3 米）。
    # 目的：防止机器人把腿缩起来导致身体过低，或者站得太高导致不稳定。


    body_lin_acc_l2 = RewTerm(
        func=mdp.body_lin_acc_l2,
        weight=0.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="")},
    )
    # body_lin_acc_l2：惩罚机体的线加速度。
    # 目的：让运动更平滑，减少急加速急减速带来的机械冲击。

    # 3. 关节（Joint）惩罚项
    #    针对电机的输出进行约束，保护硬件并鼓励高效运动。
    # Joint penalties：和关节力矩、速度、加速度、位置限制相关的惩罚。
    joint_torques_l2 = RewTerm(
        func=mdp.joint_torques_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")}
    )
    # joint_torques_l2：惩罚关节力矩（电流）过大。
    # 目的：省电、保护电机不过载。通常会给一个较小的负权重。

    joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")}
    )
    # joint_vel_l2：惩罚关节速度过大。
    # 目的：防止电机转得太快，超出物理极限。

    joint_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")}
    )
    # joint_acc_l2：惩罚关节加速度（ jerk ）。
    # 目的：让动作更丝滑，减少机械磨损。

    joint_deviation_l1 = RewTerm(
            func=mdp.joint_deviation_l1,
            weight=0.0,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
        )
    # joint_deviation_l1：惩罚关节偏离默认（零点）位置。
    # 目的：鼓励机器人在站立或行走时，尽量保持在机械中立位，避免关节过度弯曲导致奇异位形。
    
    def create_joint_deviation_l1_rewterm(self, attr_name, weight, joint_names_pattern):
        """动态创建一个“关节偏离默认姿态”的奖励项。

        有些机器人希望给 hip、knee 等不同关节组不同权重。
        这个小工具可以在子类里按关节正则表达式快速添加奖励项。
        """
        rew_term = RewTerm(
            func=mdp.joint_deviation_l1,
            weight=weight,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names_pattern)},
        )
        setattr(self, attr_name, rew_term)# setattr：利用反射机制，动态地给这个配置类添加一个新的属性

    joint_pos_limits = RewTerm(
        func=mdp.joint_pos_limits, weight=0.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")}
    )
    # joint_pos_limits：惩罚关节接近物理极限位置。
    # soft_ratio=1.0：在达到极限的 100% 时开始生效（即硬边界）。

    joint_vel_limits = RewTerm(
        func=mdp.joint_vel_limits,
        weight=0.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*"), "soft_ratio": 1.0},
    )
    # joint_vel_limits：惩罚关节速度接近极限。
    # soft_ratio=1.0：同样是在达到极限时才生效。


    joint_power = RewTerm(
        func=mdp.joint_power,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
        },
    )
    # joint_power：惩罚关节功率（力矩 x 速度）过大。
    # 目的：直接优化能耗效率。

    stand_still_without_cmd = RewTerm(
        func=mdp.stand_still_without_cmd,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "command_threshold": 0.1,
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
        },
    )
    # stand_still_without_cmd：当没有速度指令时，奖励静止不动。
    # 目的：当指令速度为 0 时，鼓励机器人立刻停下来，不要因为惯性滑行。
    

    joint_pos_penalty = RewTerm(
        func=mdp.joint_pos_penalty,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
        },
    )
    # joint_pos_penalty：这是一个更复杂的关节位置惩罚。
    # 包含了站立不动时的放大系数 (stand_still_scale=5.0)，意味着静止时对姿态要求更严。


    # 下面三个是针对特定关节组（Hip X/Y, Knee）的惩罚，逻辑同上，但可以单独调参。
    hipx_joint_pos_penalty = RewTerm(
        func=mdp.joint_pos_penalty,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
        },
    )

    hipy_joint_pos_penalty = RewTerm(
        func=mdp.joint_pos_penalty,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
        },
    )

    knee_joint_pos_penalty = RewTerm(
        func=mdp.joint_pos_penalty,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
        },
    )



    # wheel_vel_penalty：针对轮式腿（如有）的速度惩罚。
    wheel_vel_penalty = RewTerm(
        func=mdp.wheel_vel_penalty,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=""),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
            "command_name": "base_velocity",
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
        },
    )

    # joint_mirror：惩罚对侧关节运动不对称。
    # mirror_joints: [["FR.*", "RL.*"], ...] 定义了哪些关节应该对称运动。
    # 目的：防止机器人走“顺拐”。
    joint_mirror = RewTerm(
        func=mdp.joint_mirror,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mirror_joints": [["FR.*", "RL.*"], ["FL.*", "RR.*"]],
        },
    )

    # action_mirror：惩罚动作输出不对称。
    # 目的：同上，鼓励对称步态。
    action_mirror = RewTerm(
        func=mdp.action_mirror,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mirror_joints": [["FR.*", "RL.*"], ["FL.*", "RR.*"]],
        },
    )

    # action_sync：惩罚关节组内运动不同步。
    # 例如：四条腿的髋关节应该在同一时间有相似的运动趋势。
    action_sync = RewTerm(
        func=mdp.action_sync,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "joint_groups": [
                ["FR_hip_joint", "FL_hip_joint", "RL_hip_joint", "RR_hip_joint"],
                ["FR_thigh_joint", "FL_thigh_joint", "RL_thigh_joint", "RR_thigh_joint"],
                ["FR_calf_joint", "FL_calf_joint", "RL_calf_joint", "RR_calf_joint"],
            ],
        },
    )

    # Action penalties：和策略输出动作相关的惩罚。
    # 4. 动作（Action）与平滑性惩罚

    # applied_torque_limits：惩罚输出的力矩超过软限制。
    applied_torque_limits = RewTerm(
        func=mdp.applied_torque_limits,
        weight=0.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    )

    # action_rate_l2：惩罚动作的变化率（即加速度）。
    # 目的：让控制信号更平滑，减少对电机的冲击。这是非常重要的正则化项。
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=0.0)
    # smoothness_1 = RewTerm(func=mdp.smoothness_1, weight=0.0)  # Same as action_rate_l2
    # smoothness_2 = RewTerm(func=mdp.smoothness_2, weight=0.0)  # Unvaliable now

    # Contact sensor：和接触力、非法接触、足端落地冲击相关的奖励/惩罚。
    # 5. 接触传感器 (Contact Sensors)
    #    利用脚部或身体的接触力进行奖励或惩罚。

    # undesired_contacts：惩罚身体（非脚部）接触地面。
    # 目的：如果机器人用手臂撑地或者侧翻，扣大分。
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
            "threshold": 1.0,
        },
    )

    # contact_forces：惩罚脚部接触力过大。
    # 目的：鼓励“猫步”，轻盈落地，减少对关节的冲击。
    contact_forces = RewTerm(
        func=mdp.contact_forces,
        weight=0.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=""), "threshold": 100.0},
    )

    # foot_impact_velocity：惩罚脚落地时的速度过大。
    # 目的：防止“跺脚”，鼓励缓冲。
    foot_impact_velocity = RewTerm(
        func=mdp.foot_impact_velocity,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "speed_threshold": 0.10,
        }
    )

    # 6. 速度跟踪奖励 (核心正向奖励)
    #    这是机器人唯一“得分”的地方，也是训练的核心驱动力。
    # Velocity-tracking rewards：速度跟踪任务最核心的正奖励。

    # track_lin_vel_xy_exp：奖励机器人在 X/Y 方向的速度跟踪误差小。
    # std=math.sqrt(0.5)：控制高斯函数的宽度，决定给分的“宽容度”。
    # 公式通常是 exp(-error^2 / (2*std^2))。越接近目标速度，得分越高（最大为 1）。
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=0.0, params={"command_name": "base_velocity", "std": math.sqrt(0.5)}
    )

    # track_ang_vel_z_exp：奖励机器人绕 Z 轴旋转（转身）的速度跟踪。
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.0, params={"command_name": "base_velocity", "std": math.sqrt(0.5)}
    )

    # Others：步态、足端高度、足端滑动、站立等辅助项。不同机器人会选择性开启。
    # feet_air_time = RewTerm(
    #     func=mdp.feet_air_time,
    #     weight=0.0,
    #     params={
    #         "command_name": "base_velocity",
    #         "threshold": 0.5,
    #         "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
    #     },
    # )


    # 7. 步态与足端轨迹 (Gait & Feet Air Time)
    #    控制走路的节奏和美感。

    # feet_air_time：奖励足端在空中停留的时间符合预期。
    # 这是实现“奔跑”而不是“滑行”的关键。如果权重为负，则是惩罚（不希望出现）。
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_including_ang_z,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "threshold": 0.5,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
        },
    )

    # feet_air_time_lin_xy：针对 X/Y 方向移动时的空中时间奖励。
    feet_air_time_lin_xy = RewTerm(
        func=mdp.feet_air_time_lin_xy_cmd,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "threshold": 0.5,
            "cmd_threshold": 0.1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
        },
    )

    # feet_air_time_x_neg：针对后退行走的特定奖励。
    feet_air_time_x_neg = RewTerm(
        func=mdp.feet_air_time_x_neg_cmd,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "threshold": 0.5,
            "cmd_threshold": 0.1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
        },
    )

    # feet_air_time_ang_z：针对转身动作的足端腾空奖励。
    feet_air_time_ang_z = RewTerm(
        func=mdp.feet_air_time_ang_z_cmd,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "threshold": 0.5,
            "cmd_threshold": 0.1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
        },
    )

    # feet_air_time_variance：惩罚足端腾空时间的方差过大。
    feet_air_time_variance = RewTerm(
        func=mdp.feet_air_time_variance_penalty,
        weight=0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="")},
    )

    # feet_gait：更高级的步态奖励，可能涉及相位控制。
    feet_gait = RewTerm(
        func=mdp.GaitReward,
        weight=0.0,
        params={
            "std": math.sqrt(0.5),
            "command_name": "base_velocity",
            "max_err": 0.2,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
            "synced_feet_pair_names": (("", ""), ("", "")),
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("contact_forces"),
        },
    )

    # phase_foot_trajectory_exp：奖励足端轨迹符合预设的曲线（如椭圆或倒摆模型）。
    # 这是实现“定点踩坑”或“精确落脚”的高级奖励。
    phase_foot_trajectory_exp = RewTerm(
        func=mdp.phase_foot_trajectory_exp,
        weight=0.0,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "std": 0.12,
            "command_threshold": 0.1,
            "cycle_time": 0.425,  # 一个完整步态周期时长（秒）
            "phase_offsets": (0.0, 1.0, 1.0, 0.0),
            "gait_span": -0.00,    # x 方向摆动半跨度（米），符号决定前后扫动方向
            "gait_psi": 0.05,   # 摆动抬脚高度尺度（米）
            "gait_delta": 0.02, # z 方向轨迹偏置（米）
            "x_offset": 0.0,    # 轨迹整体 x 偏移（米）
            "stance_span": 0.0, # 支撑相在 S∈[0,2) 中占据的长度
            "stand_ref_z_offset": -0.0, # 基准站立足端参考在 body-z 的偏置（米）
            "velocity_weight": 0.0, # 速度误差在总误差中的权重
        },
    )
    
    # feet_contact：奖励特定数量的脚接触地面（如双足支撑）。
    feet_contact = RewTerm(
        func=mdp.feet_contact,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
            "command_name": "base_velocity",
            "expect_contact_num": 2,
        },
    )

    # feet_contact_without_cmd：当没有移动指令时，奖励脚保持接触（不要乱动）。
    feet_contact_without_cmd = RewTerm(
        func=mdp.feet_contact_without_cmd,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
            "command_name": "base_velocity",
        },
    )

    # feet_stumble：惩罚脚在接触地面时发生剧烈碰撞（绊倒）。
    feet_stumble = RewTerm(
        func=mdp.feet_stumble,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
        },
    )

    # feet_slide：惩罚脚在地面滑动。
    # 目的：鼓励抓地力，防止在冰面上“溜冰”。
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=0.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=""),
            "asset_cfg": SceneEntityCfg("robot", body_names=""),
        },
    )


    # 8. 其他辅助项

    # stand_still：静止时的综合奖励/惩罚。
    stand_still = RewTerm(
        func=mdp.stand_still_joint_deviation_l1,
        weight=0,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", joint_names=""),
        },
    ) # negetive

    # feet_height：奖励脚达到目标高度（腾空时）。
    # tanh_mult：控制 S 形曲线的陡峭度。
    feet_height = RewTerm(
        func=mdp.feet_height,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=""),
            "tanh_mult": 2.0,
            "target_height": 0.05,
            "command_name": "base_velocity",
        },
    )

    # feet_height_body：相对于身体的脚高度奖励。
    feet_height_body = RewTerm(
        func=mdp.feet_height_body,
        weight=0.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=""),
            "tanh_mult": 2.0,
            "target_height": -0.3,
            "command_name": "base_velocity",
        },
    )

    # feet_distance_y_exp：奖励左右脚的横向距离符合预期（保持步宽）。
    feet_distance_y_exp = RewTerm(
        func=mdp.feet_distance_y_exp,
        weight=0.0,
        params={
            "std": math.sqrt(0.25),
            "asset_cfg": SceneEntityCfg("robot", body_names=""),
            "stance_width": float,
        },
    )

    # feet_distance_xy_exp = RewTerm(
    #     func=mdp.feet_distance_xy_exp,
    #     weight=0.0,
    #     params={
    #         "std": math.sqrt(0.25),
    #         "asset_cfg": SceneEntityCfg("robot", body_names=""),
    #         "stance_length": float,
    #         "stance_width": float,
    #     },
    # )

    # upward：奖励机体朝上向量（Z轴）保持竖直。
    upward = RewTerm(func=mdp.upward, weight=0.0)

    # lin_vel_xy_l2_with_ang_z_command = RewTerm(
    #     func=mdp.lin_vel_xy_l2_with_ang_z_command,
    #     weight=0,
    #     params={
    #         "command_name": "base_velocity",
    #         "command_threshold": 0.1,
    #     },
    # ) # negetive


@configclass
class TerminationsCfg:
    """episode 终止条件配置。

    Termination 决定“什么时候这一回合算结束”。例如超时、走出地形、
    碰撞到不该碰的 body、姿态翻倒等。
    """

    # episode 达到最大时长时终止，这是正常 timeout。
    # time_out: 当 episode 达到最大时长（由 env_cfg.episode_length_s 定义）时终止。
    # time_out=True: 标记此终止为“超时”，而非“失败”。
    # 作用：防止机器人卡住不动，或者作为一个正常的回合结束标志。
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # 机器人走出地形范围时终止。time_out=True 表示它更像边界超时，不一定算失败。
    # terrain_out_of_bounds: 当机器人走出预设的地形范围时终止。
    # params.distance_buffer=3.0: 机器人中心距离地形边缘 3 米时触发。
    # time_out=True: 同样视为一种“边界超时”，不一定会被计为一次严重的训练失败。
    terrain_out_of_bounds = DoneTerm(
        func=mdp.terrain_out_of_bounds,
        params={"asset_cfg": SceneEntityCfg("robot"), "distance_buffer": 3.0},
        time_out=True,
    )

    # 不希望接触地面的 body 发生接触时终止，例如机身撞地。
    # illegal_contact: 检测非法接触。
    # 例如：机器人的机身（非脚部）触碰到地面，或者手臂撑地。
    # params.threshold=1.0: 当接触力超过 1.0 N 时判定为接触。
    # 注意：这里 body_names="" 是空的，通常子类会填入具体的机身部件名（如 "base", "torso"）。
    illegal_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=""), "threshold": 1.0},
    )

    # bad_orientation_2: 检测姿态是否过差（例如翻倒）。
    # 这是一个硬编码的函数，通常检查机器人的朝上向量是否偏离了世界坐标系的 Z 轴太多。
    # 如果翻倒，这一回合直接结束，通常意味着一次严重的失败。
    bad_orientation_2 = DoneTerm(func=mdp.bad_orientation_2)
    




@configclass
class CurriculumCfg:
    """课程学习配置。

    Curriculum 的目标是“先易后难”。比如先在简单地形/小速度指令下训练，
    当策略表现变好后，再逐步增加地形和命令难度。
    """

    # 地形难度课程：机器人走得好就升地形等级，走得差就降。
    # terrain_levels: 地形难度分级。
    # func=mdp.terrain_levels_vel: 根据机器人当前的速度跟踪表现，动态调整地形等级。
    # 逻辑：如果机器人走得稳，就让它去更崎岖的地形；如果走得差，就降回简单地形。
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)


    # 记录当前 gait_level，供 TensorBoard 观察，也被部分 reward 用来调节强度。
    # gait_level: 记录当前的步态难度等级。
    # 这通常是一个供 TensorBoard 可视化的指标，用于监控训练进度。
    # 也可能被某些奖励函数用来调节奖励的强度。
    gait_level = CurrTerm(func=mdp.gait_level_curve)

    # 速度指令范围课程：速度跟踪奖励好时，逐渐扩大命令速度范围。
    # command_levels: 速度指令范围的课程学习。
    # func=mdp.command_levels_vel: 根据指定的奖励项（track_lin_vel_xy_exp）的表现，
    # 动态扩大或缩小下发给机器人的速度指令范围。
    # params.range_multiplier: 范围扩增的倍率，从 0.1 倍逐渐增加到 1.0 倍。
    # 效果：一开始只让机器人学走慢步，学好了再逐渐让它学跑快步。
    command_levels = CurrTerm(
        func=mdp.command_levels_vel,
        params={
            "reward_term_name": "track_lin_vel_xy_exp",
            "range_multiplier": (0.1, 1.0),
        },
    )


##
# Environment configuration
##


@configclass
class LocomotionVelocityRoughEnvCfg(ManagerBasedRLEnvCfg):
    """通用粗糙地形速度跟踪环境配置。

    这是整个 RL 任务的骨架。它不绑定具体机器人，而是定义：
        * 仿真步长和 episode 时长；
        * 默认并行环境数量；
        * scene、observations、actions、commands；
        * rewards、terminations、events、curriculum。

    M20/Lite3 等具体机器人会继承这个类，再替换 robot 资产、关节名字、
    动作空间、奖励权重和命令范围。
    """

    # decimation=4 表示策略每 4 个物理步输出一次动作。
    # 物理 dt=0.005s 时，策略控制周期为 0.005 * 4 = 0.02s，也就是 50 Hz。
    decimation = 4 #动作重采样率。
    # 每个 episode 最长 20 秒。如果 time_out=True，20 秒后环境会自动重置。
    episode_length_s = 20.0
    # 仿真配置：物理步长、渲染间隔、PhysX 参数等。

    # sim: 仿真器底层配置。
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 200,  # 物理仿真步长，约 0.005 秒 (200 Hz)
        render_interval=decimation,  # 渲染频率与策略步长对齐
        physx=PhysxCfg(
            
             gpu_collision_stack_size=2**27,           # 134,217,728  # GPU 碰撞检测的内存栈大小，非常大以支持复杂场景
        ),
    )
    # 场景配置。num_envs=4096 表示默认并行训练 4096 个机器人环境。
    # scene: 场景配置，定义了并行环境的数量和间距。
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    # 基础 MDP 组件：观测、动作、命令。
    # 定义了观测空间、动作空间和命令空间的结构。
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()

    # 引入之前定义的奖励、终止、事件和课程学习配置。
    # 训练相关 MDP 组件：奖励、终止、事件、课程学习。
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """配置对象创建后做二次整理。

        Isaac Lab 的 configclass 支持继承。很多子类会先调用 ``super().__post_init__()``，
        再覆盖自己的机器人专属参数。
        """

        # 重新确认基础控制参数。
        # 通用设置。
        self.decimation = 4
        self.episode_length_s = 20.0
        
        # 仿真设置。
        # 仿真参数微调。
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation

        # 将场景中的物理材质（如摩擦力）同步给仿真器。
        self.sim.physics_material = self.scene.terrain.physics_material

        # PhysX 求解器参数优化，针对 GPU 仿真进行设置。
        self.sim.physx.gpu_max_rigid_patch_count = 2**19  # 增加接触 patch 数量以提高稳定性
        self.sim.physx.max_position_iteration_count = 4  # 位置求解迭代次数
        self.sim.physx.max_velocity_iteration_count = 1  # 速度求解迭代次数

        # 更新传感器刷新周期。高度扫描通常跟控制周期走，接触力传感器跟物理步走。
        # 高度扫描仪 (Height Scanner): 通常不需要每一步都更新，跟随策略步长 (decimation) 即可。
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        # 接触力传感器 (Contact Forces): 需要高频检测（如脚踢到石头），跟随物理步长。
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

        # 如果启用了地形课程学习，就让 terrain_generator 也进入 curriculum 模式。
        # 这样 terrain_generator 可以根据 CurriculumCfg 的指令动态生成不同难度的地形。
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False

    def disable_zero_weight_rewards(self):
        """把 weight 为 0 的 reward 项设为 None。

        这样 RewardManager 不会再创建和计算这些无效项，可以减少训练开销。
        子类通常先设置自己需要的 reward 权重，最后调用这个函数清理掉 0 权重项。
        """
        # 遍历 rewards 类的所有属性
        for attr in dir(self.rewards):
            # 跳过私有属性和方法
            if not attr.startswith("__"):
                reward_attr = getattr(self.rewards, attr)
                # 如果该属性不是函数（即是一个奖励项配置）且权重为 0
                if not callable(reward_attr) and reward_attr.weight == 0:
                    # 将其设置为 None，从而在后续被 Manager 忽略
                    setattr(self.rewards, attr, None)

# 这是一个元编程工具函数，用于在运行时动态创建观测组（Observation Group）类。
def create_obsgroup_class(class_name, terms, enable_corruption=False, concatenate_terms=True):
    """
    Dynamically create and register a ObsGroup class based on the given configuration terms.

    :param class_name: Name of the configuration class.
    :param terms: Configuration terms, a dictionary where keys are term names and values are term content.
    :param enable_corruption: Whether to enable corruption for the observation group. Defaults to False.
    :param concatenate_terms: Whether to concatenate the observation terms in the group. Defaults to True.
    :return: The dynamically created class.
    翻译：
    动态创建一个 ObsGroup 配置类。
    
    在 Isaac Lab 中，观测值（Observations）通常分组管理（如 Policy 组、Critc 组）。
    这个函数允许通过传入不同的 terms（观测项字典）来生成不同的配置类。

    :param class_name: 生成的类名。
    :param terms: 观测项字典，键为名称，值为配置内容。
    :param enable_corruption: 是否开启观测噪声/损坏（用于域随机化）。
    :param concatenate_terms: 是否将该组内的所有观测值拼接成一个大向量。
    :return: 生成的类对象。
    """
    # Dynamically determine the module name
    module_name = inspect.getmodule(inspect.currentframe()).__name__

    # Define the post-init function
    # 定义一个 __post_init__ 函数，用于在类实例化后设置特定参数。
    def post_init_wrapper(self):
        setattr(self, "enable_corruption", enable_corruption)
        setattr(self, "concatenate_terms", concatenate_terms)

    # Dynamically create the class using ObsGroup as the base class
    # 使用 type() 动态创建类。
    # 继承自 ObsGroup，并注入传入的 terms 和上面定义的 post_init_wrapper。
    terms["__post_init__"] = post_init_wrapper
    dynamic_class = configclass(type(class_name, (ObsGroup,), terms))

    # Custom serialization and deserialization
    # 为动态类添加序列化方法，以便能被 pickle 序列化（在多进程训练时必需）。
    def __getstate__(self):
        state = self.__dict__.copy()
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)

    # Add custom serialization methods to the class
    dynamic_class.__getstate__ = __getstate__
    dynamic_class.__setstate__ = __setstate__

    # Place the class in the global namespace for accessibility
    # 将生成的类放入全局命名空间和模块字典中，使其在其他地方可导入。
    globals()[class_name] = dynamic_class

    # Register the dynamic class in the module's dictionary
    if module_name in sys.modules:
        sys.modules[module_name].__dict__[class_name] = dynamic_class
    else:
        raise ImportError(f"Module {module_name} not found.")

    # Return the class for external instantiation
    return dynamic_class
