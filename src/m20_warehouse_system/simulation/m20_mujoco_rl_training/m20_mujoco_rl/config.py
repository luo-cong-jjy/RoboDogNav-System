# ======================================================================
# config.py —— M20 MuJoCo 训练配置数据类（中文注释版）
# 作用：用 dataclass 定义训练所需的全部配置：
#   HeightScanConfig：特权 critic 的高度扫描网格
#   TerrainConfig：地形生成参数（平地/简易台阶/官方台阶/随机箱体）
#   RandomizationConfig：域随机化与重置随机化参数
#   RewardConfig：奖励函数权重与阈值
#   M20EnvConfig：环境配置（仿真步长、指令、终止条件等）
#   PPOConfig：PPO 训练超参数与评估指标权重
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Configuration dataclasses for M20 MuJoCo training."""

from __future__ import annotations

# 操作系统接口：读取环境变量
import os
# 数据类：自动生成构造器；field 用于默认工厂
from dataclasses import dataclass, field
# 路径库
from pathlib import Path
# 类型标注：Optional
from typing import Optional


# 包根目录（config.py 的上级目录）
def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


# 工作空间根目录（包含 src/third_party 的上级目录；找不到时回退到上 5 级）
def workspace_root() -> Path:
    # 从当前文件向上查找包含 src/third_party 的目录
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "third_party").is_dir():
            return parent
    # 兜底：上 5 级
    return Path(__file__).resolve().parents[5]


# 返回 M20 MJCF 模型目录（优先环境变量 M20_MJCF_DIR，其次包内资源，最后 SDK 部署目录）
def default_m20_mjcf_dir() -> Path:
    # 若设置了环境变量则优先使用
    env_dir = os.environ.get("M20_MJCF_DIR")
    if env_dir:
        return Path(env_dir).expanduser().resolve()

    # 包内自带资源目录（assets/m20_mjcf/mjcf）
    packaged_dir = package_root() / "assets" / "m20_mjcf" / "mjcf"
    if (packaged_dir / "M20.xml").exists():
        return packaged_dir

    # 回退到 SDK 部署目录中的 M20_description
    return (
        workspace_root()
        / "src"
        / "third_party"
        / "sdk_deploy"
        / "src"
        / "M20_sdk_deploy"
        / "M20_description"
        / "m20_mjcf"
        / "mjcf"
    )


# 高度扫描配置：训练时作为特权 critic 输入的"航向对齐地形高度扫描"
@dataclass
class HeightScanConfig:
    """Yaw-aligned terrain scan used as privileged critic input during training."""

    # 扫描网格分辨率（米）
    resolution: float = 0.10
    # 扫描区域 x 方向长度（米）
    size_x: float = 1.60
    # 扫描区域 y 方向宽度（米）
    size_y: float = 1.00
    # 射线起点相对机身的 z 偏移（米）
    z_offset: float = 2.0
    # 高度值裁剪范围（米，超出则截断）
    clip: float = 1.0


# 地形生成配置
@dataclass
class TerrainConfig:
    """Terrain generation options.

    name:
        flat            only the floor from M20.xml
        stair_easy      low progressive stairs for curriculum start
        stair_official  close to sdk_deploy stair.xml
        random_boxes    randomly placed box obstacles
    """

    # 地形类型（见上方 docstring 说明）
    name: str = "stair_official"
    # 每级台阶高度（米）
    stair_height: float = 0.05
    # 台阶高度随机范围（可选，用于训练环境间差异化）
    stair_height_range: Optional[tuple[float, float]] = None
    # 台阶深度（米）
    stair_depth: float = 0.35
    # 台阶级数
    stair_count: int = 5
    # 第一级台阶起始 x 位置（米）
    stair_start_x: float = 1.0
    # 台阶后平台长度（米）
    plateau_length: float = 2.0
    # 随机箱体数量
    random_box_count: int = 40
    # 随机箱体高度范围（米）
    random_box_height: tuple[float, float] = (0.02, 0.12)
    # 随机箱体 x 分布范围（米）
    random_box_area_x: tuple[float, float] = (0.8, 5.0)
    # 随机箱体 y 分布范围（米）
    random_box_area_y: tuple[float, float] = (-1.0, 1.0)
    # 地形随机种子
    seed: int = 1


# 域随机化配置：重置随机化、物理随机化、扰动/推力
@dataclass
class RandomizationConfig:
    # 是否启用随机化（总开关）
    enabled: bool = True
    # 每次 reset 时是否重新随机化物理属性
    randomize_physics_on_reset: bool = True
    # 重置位置 x 噪声范围（米）
    base_x_range: tuple[float, float] = (-0.1, 0.1)
    # 重置位置 y 噪声范围（米）
    base_y_range: tuple[float, float] = (-0.08, 0.08)
    # 重置位置 z 噪声范围（米）
    base_z_range: tuple[float, float] = (0.0, 0.0)
    # 重置横滚角噪声范围（弧度）
    base_roll_range: tuple[float, float] = (-0.08, 0.08)
    # 重置俯仰角噪声范围（弧度）
    base_pitch_range: tuple[float, float] = (-0.08, 0.08)
    # 重置航向角噪声范围（弧度）
    base_yaw_range: tuple[float, float] = (-3.14, 3.14)
    # 重置线速度噪声范围（米/秒，x/y）
    base_lin_vel_range: tuple[float, float] = (-0.1, 0.1)
    # 重置 z 向速度噪声范围（米/秒）
    base_z_vel_range: tuple[float, float] = (-0.05, 0.05)
    # 重置角速度噪声范围（弧度/秒）
    base_ang_vel_range: tuple[float, float] = (-0.03, 0.03)
    # 关节位置噪声范围（弧度）
    joint_pos_noise_range: tuple[float, float] = (-0.01, 0.01)
    # 关节速度噪声范围（弧度/秒）
    joint_vel_range: tuple[float, float] = (-0.01, 0.01)
    # 地面摩擦系数随机范围（乘性）
    friction_range: tuple[float, float] = (0.5, 1.2)
    # 机身质量附加量范围（千克）
    base_mass_add_range: tuple[float, float] = (-0.5, 1.5)
    # 连杆质量缩放范围（乘性）
    link_mass_scale_range: tuple[float, float] = (0.92, 1.08)
    # 机身质心 x 偏移范围（米）
    base_com_x_range: tuple[float, float] = (-0.015, 0.015)
    # 机身质心 y 偏移范围（米）
    base_com_y_range: tuple[float, float] = (-0.015, 0.015)
    # 机身质心 z 偏移范围（米）
    base_com_z_range: tuple[float, float] = (-0.01, 0.01)
    # PD 位置增益缩放范围
    kp_scale_range: tuple[float, float] = (0.92, 1.08)
    # PD 速度增益缩放范围
    kd_scale_range: tuple[float, float] = (0.92, 1.08)
    # 是否在重置时施加外部力/力矩（wrench）
    reset_wrench_enabled: bool = True
    # 重置力范围（牛顿，x/y/z）
    reset_force_range: tuple[float, float] = (-5.0, 5.0)
    # 重置力矩范围（牛·米）
    reset_torque_range: tuple[float, float] = (-5.0, 5.0)
    # 重置 wrench 持续时间范围（秒）
    reset_wrench_duration_range: tuple[float, float] = (0.01, 0.04)
    # 训练中是否施加推力扰动
    push_enabled: bool = True
    # 两次推力间隔范围（秒）
    push_interval_range_s: tuple[float, float] = (5.0, 8.0)
    # 推力速度增量范围（米/秒，x/y）
    push_lin_vel_range: tuple[float, float] = (-0.15, 0.15)


# 奖励函数配置：各奖励项权重与阈值（权重为负表示惩罚）
@dataclass
class RewardConfig:
    # 线速度跟踪奖励权重（跟踪前向/横向指令）
    track_lin_weight: float = 2.0
    # 航向角速度跟踪奖励权重
    track_yaw_weight: float = 1.0
    # 前向位移进步奖励权重
    progress_weight: float = 0.4
    # 机身下地形高度上升奖励权重（爬坡/上台阶）
    terrain_height_progress_weight: float = 2.0
    # 处于高处地形（台阶上）的奖励权重
    stair_height_weight: float = 0.6
    # z 向速度惩罚权重
    z_vel_weight: float = -2.0
    # 机身倾斜（姿态）惩罚权重
    orientation_weight: float = -0.6
    # 机身高度偏离目标高度惩罚权重
    base_height_weight: float = -0.5
    # 关节力矩惩罚权重
    torque_weight: float = -2.5e-5
    # 动作变化率（加速度）惩罚权重
    action_rate_weight: float = -0.03
    # 腿部动作幅度 L2 惩罚权重
    leg_action_l2_weight: float = -0.30
    # 轮子动作幅度 L2 惩罚权重
    wheel_action_l2_weight: float = -0.04
    # 动作饱和（超出阈值）惩罚权重
    action_saturation_weight: float = -8.0
    # 动作饱和判定阈值（归一化动作）
    action_saturation_threshold: float = 0.48
    # 存活奖励（每步固定正奖励）
    alive_weight: float = 0.05
    # 腿部关节加速度惩罚权重
    joint_acc_weight: float = -2.0e-7
    # 轮子关节加速度惩罚权重
    wheel_acc_weight: float = -1.0e-7
    # 关节位置越限惩罚权重
    joint_pos_limits_weight: float = -5.0
    # 关节限位软边界余量比例（相对关节行程）
    joint_limit_margin_ratio: float = 0.05
    # 关节功率惩罚权重
    joint_power_weight: float = -2.0e-5
    # 静止指令时的站立惩罚权重
    stand_still_weight: float = -2.0
    # 判定"静止指令"的指令幅值阈值
    stand_still_command_threshold: float = 0.1
    # 静止时允许的最大速度阈值
    stand_still_velocity_threshold: float = 0.1
    # hipx 关节偏离默认位置惩罚权重
    hipx_pos_weight: float = -0.6
    # hipy 关节偏离默认位置惩罚权重
    hipy_pos_weight: float = -0.2
    # knee 关节偏离默认位置惩罚权重
    knee_pos_weight: float = -0.2
    # 左右腿关节镜像对称惩罚权重
    joint_mirror_weight: float = -0.06
    # 非轮子（不应接触）接触惩罚权重
    undesired_contact_weight: float = -2.0
    # 轮子接触力过大惩罚权重
    contact_force_weight: float = -1.5e-4
    # 轮子接触力阈值（牛顿）
    contact_force_threshold: float = 120.0
    # 静止指令下"轮子不该着地"奖励权重（负为惩罚）
    feet_contact_without_cmd_weight: float = 0.1
    # 机身朝上奖励权重
    upward_weight: float = 0.08
    # 台阶上前进奖励权重
    stair_forward_progress_weight: float = 0.0
    # 触发台阶前进奖励所需的最小高度比例（相对单级台阶高）
    stair_forward_progress_height_fraction: float = 0.5
    # 轮子腾空时间奖励权重（鼓励跳跃越障）
    wheel_air_time_weight: float = 0.0
    # 轮子腾空时间阈值（秒）
    wheel_air_time_threshold: float = 0.08
    # 轮子抬升高度奖励权重（上台阶时抬轮）
    wheel_clearance_weight: float = 0.0
    # 轮子抬升目标（相对标称离地高度，米）
    wheel_clearance_lift_target: float = 0.035
    # 触发抬轮奖励的前方地形高度差阈值（米）
    wheel_clearance_terrain_threshold: float = 0.008
    # 轮子绊到垂直面惩罚权重
    wheel_stumble_weight: float = 0.0
    # 判定"绊倒"的接触法线 z 分量阈值
    wheel_stumble_normal_z_threshold: float = 0.35
    # 判定"绊倒"的接触力阈值（牛顿）
    wheel_stumble_force_threshold: float = 20.0


# 环境配置：仿真步长、控制步长、指令生成、终止条件、观测开关
@dataclass
class M20EnvConfig:
    # 自定义模型 XML 路径（默认 None，由 terrain 决定）
    model_xml: Optional[str] = None
    # 地形配置
    terrain: TerrainConfig = field(default_factory=TerrainConfig)
    # 随机化配置
    randomization: RandomizationConfig = field(default_factory=RandomizationConfig)
    # 奖励配置
    reward: RewardConfig = field(default_factory=RewardConfig)
    # 高度扫描配置
    height_scan: HeightScanConfig = field(default_factory=HeightScanConfig)
    # 随机种子
    seed: int = 1
    # 仿真步长（秒）
    sim_dt: float = 0.001
    # 控制步长（秒，策略动作间隔）
    control_dt: float = 0.02
    # 单回合时长（秒）
    episode_seconds: float = 8.0
    # The policy action is a normalized command.  Keep it conservative here:
    # leg targets are later scaled by 0.125/0.25 rad and wheel targets by 5 rad/s.
    # 注释（原文）：策略动作是归一化指令，这里保持保守：腿部目标之后按
    # 0.125/0.25 弧度缩放，轮子目标按 5 rad/s 缩放
    # 动作裁剪范围（归一化动作的限幅）
    action_clip: float = 1.0
    # 初始机身 x 位置（米）
    base_init_x: float = 0.0
    # 初始机身 y 位置（米）
    base_init_y: float = 0.0
    # 初始机身 z 高度（米）
    base_init_height: float = 0.50
    # 重置后物理沉降时间（秒）
    reset_settle_seconds: float = 0.0
    # 指令重新采样间隔（秒）
    command_resample_seconds: float = 4.0
    # 前向速度指令范围（米/秒）
    forward_command_range: tuple[float, float] = (0.35, 0.9)
    # 横向速度指令范围（米/秒）
    lateral_command_range: tuple[float, float] = (-0.15, 0.15)
    # 航向角速度指令范围（弧度/秒）
    yaw_command_range: tuple[float, float] = (-0.3, 0.3)
    # 指令模式：forward（只前进）/ random（随机横向+转向）
    command_mode: str = "forward"
    # 目标机身高度（米，奖励基准）
    target_base_height: float = 0.40
    # 终止判断：机身低于该高度视为摔倒（米）
    terminate_base_height: float = 0.18
    # 终止判断：投影重力 z 分量（朝下阈值）
    terminate_projected_gravity_z: float = -0.25
    # 是否在非轮子接触时终止回合
    terminate_on_undesired_contact: bool = False
    # 触发接触终止的非轮子接触数阈值
    undesired_contact_termination_threshold: float = 0.5
    # 需要连续多少步非轮子接触才终止
    undesired_contact_termination_steps: int = 1
    # 接触终止时的一次性惩罚奖励
    undesired_contact_terminal_penalty: float = -200.0
    # critic 是否额外获得机身线速度（特权信息）
    critic_base_lin_vel: bool = True
    # critic 是否额外获得高度扫描（特权信息）
    critic_height_scan: bool = True
    # actor 观测是否包含高度扫描（改变策略输入维度）
    include_height_scan: bool = False


# PPO 训练配置
@dataclass
class PPOConfig:
    # 随机种子
    seed: int = 1
    # 并行环境数量
    num_envs: int = 8
    # 训练迭代轮数
    iterations: int = 2000
    # 每个环境每轮的步数
    steps_per_env: int = 24
    # 学习率
    learning_rate: float = 3.0e-4
    # 折扣因子
    gamma: float = 0.99
    # GAE lambda 参数
    gae_lambda: float = 0.95
    # PPO 裁剪系数
    clip_param: float = 0.2
    # 价值损失系数
    value_loss_coef: float = 1.0
    # 熵正则系数
    entropy_coef: float = 0.0002
    # 梯度裁剪最大范数
    max_grad_norm: float = 1.0
    # 每轮数据的学习轮数
    num_learning_epochs: int = 5
    # mini-batch 数量
    num_mini_batches: int = 4
    # 策略初始噪声标准差
    init_noise_std: float = 0.15
    # 确定性动作限幅（tanh 输出乘此值）
    action_limit: float = 0.60
    # 策略 log_std 下限
    log_std_min: float = -4.0
    # 策略 log_std 上限
    log_std_max: float = -1.2
    # 网络隐藏层维度
    hidden_dims: tuple[int, ...] = (512, 256, 128)
    # 定期保存间隔（轮）
    save_interval: int = 50
    # 固定评估间隔（轮，0 表示禁用）
    eval_interval: int = 100
    # 每次评估的回合数
    eval_episodes: int = 3
    # actor 均值动作 L2 正则系数
    mean_action_l2_coef: float = 0.02
    # actor 均值动作饱和正则系数
    mean_action_saturation_coef: float = 4.0
    # actor 均值动作饱和阈值
    mean_action_saturation_threshold: float = 0.48
    # 评估分数：动作幅度惩罚权重
    eval_action_weight: float = 250.0
    # 评估分数：动作饱和惩罚权重
    eval_action_saturation_weight: float = 5000.0
    # 评估分数：动作饱和阈值
    eval_action_saturation_threshold: float = 0.48
    # 评估分数：非轮子接触惩罚权重
    eval_undesired_contact_weight: float = 100.0
    # 评估分数：接触力惩罚权重
    eval_contact_force_weight: float = 5.0
    # 评估分数：终止率惩罚权重
    eval_termination_weight: float = 1000.0
    # 评估分数：回合回报标准差惩罚权重
    eval_return_std_weight: float = 0.0
    # 评估分数：最差回合回报奖励权重
    eval_return_min_weight: float = 0.0
    # 评估候选有效所需的最小地形高度比例
    eval_min_terrain_height_fraction: float = 0.0
    # 评估候选有效所需的最小前向距离（米）
    eval_min_forward_distance: float = 0.0
