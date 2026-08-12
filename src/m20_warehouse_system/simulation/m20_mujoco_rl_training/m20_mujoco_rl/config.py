"""Configuration dataclasses for M20 MuJoCo training."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "third_party").is_dir():
            return parent
    return Path(__file__).resolve().parents[5]


def default_m20_mjcf_dir() -> Path:
    env_dir = os.environ.get("M20_MJCF_DIR")
    if env_dir:
        return Path(env_dir).expanduser().resolve()

    packaged_dir = package_root() / "assets" / "m20_mjcf" / "mjcf"
    if (packaged_dir / "M20.xml").exists():
        return packaged_dir

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


@dataclass
class HeightScanConfig:
    """Yaw-aligned terrain scan used as privileged critic input during training."""

    resolution: float = 0.10
    size_x: float = 1.60
    size_y: float = 1.00
    z_offset: float = 2.0
    clip: float = 1.0


@dataclass
class TerrainConfig:
    """Terrain generation options.

    name:
        flat            only the floor from M20.xml
        stair_easy      low progressive stairs for curriculum start
        stair_official  close to sdk_deploy stair.xml
        random_boxes    randomly placed box obstacles
    """

    name: str = "stair_official"
    stair_height: float = 0.05
    stair_height_range: Optional[tuple[float, float]] = None
    stair_depth: float = 0.35
    stair_count: int = 5
    stair_start_x: float = 1.0
    plateau_length: float = 2.0
    random_box_count: int = 40
    random_box_height: tuple[float, float] = (0.02, 0.12)
    random_box_area_x: tuple[float, float] = (0.8, 5.0)
    random_box_area_y: tuple[float, float] = (-1.0, 1.0)
    seed: int = 1


@dataclass
class RandomizationConfig:
    enabled: bool = True
    randomize_physics_on_reset: bool = True
    base_x_range: tuple[float, float] = (-0.1, 0.1)
    base_y_range: tuple[float, float] = (-0.08, 0.08)
    base_z_range: tuple[float, float] = (0.0, 0.0)
    base_roll_range: tuple[float, float] = (-0.08, 0.08)
    base_pitch_range: tuple[float, float] = (-0.08, 0.08)
    base_yaw_range: tuple[float, float] = (-3.14, 3.14)
    base_lin_vel_range: tuple[float, float] = (-0.1, 0.1)
    base_z_vel_range: tuple[float, float] = (-0.05, 0.05)
    base_ang_vel_range: tuple[float, float] = (-0.03, 0.03)
    joint_pos_noise_range: tuple[float, float] = (-0.01, 0.01)
    joint_vel_range: tuple[float, float] = (-0.01, 0.01)
    friction_range: tuple[float, float] = (0.5, 1.2)
    base_mass_add_range: tuple[float, float] = (-0.5, 1.5)
    link_mass_scale_range: tuple[float, float] = (0.92, 1.08)
    base_com_x_range: tuple[float, float] = (-0.015, 0.015)
    base_com_y_range: tuple[float, float] = (-0.015, 0.015)
    base_com_z_range: tuple[float, float] = (-0.01, 0.01)
    kp_scale_range: tuple[float, float] = (0.92, 1.08)
    kd_scale_range: tuple[float, float] = (0.92, 1.08)
    reset_wrench_enabled: bool = True
    reset_force_range: tuple[float, float] = (-5.0, 5.0)
    reset_torque_range: tuple[float, float] = (-5.0, 5.0)
    reset_wrench_duration_range: tuple[float, float] = (0.01, 0.04)
    push_enabled: bool = True
    push_interval_range_s: tuple[float, float] = (5.0, 8.0)
    push_lin_vel_range: tuple[float, float] = (-0.15, 0.15)


@dataclass
class RewardConfig:
    track_lin_weight: float = 2.0
    track_yaw_weight: float = 1.0
    progress_weight: float = 0.4
    terrain_height_progress_weight: float = 2.0
    stair_height_weight: float = 0.6
    z_vel_weight: float = -2.0
    orientation_weight: float = -0.6
    base_height_weight: float = -0.5
    torque_weight: float = -2.5e-5
    action_rate_weight: float = -0.03
    leg_action_l2_weight: float = -0.30
    wheel_action_l2_weight: float = -0.04
    action_saturation_weight: float = -8.0
    action_saturation_threshold: float = 0.48
    alive_weight: float = 0.05
    joint_acc_weight: float = -2.0e-7
    wheel_acc_weight: float = -1.0e-7
    joint_pos_limits_weight: float = -5.0
    joint_limit_margin_ratio: float = 0.05
    joint_power_weight: float = -2.0e-5
    stand_still_weight: float = -2.0
    stand_still_command_threshold: float = 0.1
    stand_still_velocity_threshold: float = 0.1
    hipx_pos_weight: float = -0.6
    hipy_pos_weight: float = -0.2
    knee_pos_weight: float = -0.2
    joint_mirror_weight: float = -0.06
    undesired_contact_weight: float = -2.0
    contact_force_weight: float = -1.5e-4
    contact_force_threshold: float = 120.0
    feet_contact_without_cmd_weight: float = 0.1
    upward_weight: float = 0.08
    stair_forward_progress_weight: float = 0.0
    stair_forward_progress_height_fraction: float = 0.5
    wheel_air_time_weight: float = 0.0
    wheel_air_time_threshold: float = 0.08
    wheel_clearance_weight: float = 0.0
    wheel_clearance_lift_target: float = 0.035
    wheel_clearance_terrain_threshold: float = 0.008
    wheel_stumble_weight: float = 0.0
    wheel_stumble_normal_z_threshold: float = 0.35
    wheel_stumble_force_threshold: float = 20.0


@dataclass
class M20EnvConfig:
    model_xml: Optional[str] = None
    terrain: TerrainConfig = field(default_factory=TerrainConfig)
    randomization: RandomizationConfig = field(default_factory=RandomizationConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    height_scan: HeightScanConfig = field(default_factory=HeightScanConfig)
    seed: int = 1
    sim_dt: float = 0.001
    control_dt: float = 0.02
    episode_seconds: float = 8.0
    # The policy action is a normalized command.  Keep it conservative here:
    # leg targets are later scaled by 0.125/0.25 rad and wheel targets by 5 rad/s.
    action_clip: float = 1.0
    base_init_x: float = 0.0
    base_init_y: float = 0.0
    base_init_height: float = 0.50
    reset_settle_seconds: float = 0.0
    command_resample_seconds: float = 4.0
    forward_command_range: tuple[float, float] = (0.35, 0.9)
    lateral_command_range: tuple[float, float] = (-0.15, 0.15)
    yaw_command_range: tuple[float, float] = (-0.3, 0.3)
    command_mode: str = "forward"
    target_base_height: float = 0.40
    terminate_base_height: float = 0.18
    terminate_projected_gravity_z: float = -0.25
    terminate_on_undesired_contact: bool = False
    undesired_contact_termination_threshold: float = 0.5
    undesired_contact_termination_steps: int = 1
    undesired_contact_terminal_penalty: float = -200.0
    critic_base_lin_vel: bool = True
    critic_height_scan: bool = True
    include_height_scan: bool = False


@dataclass
class PPOConfig:
    seed: int = 1
    num_envs: int = 8
    iterations: int = 2000
    steps_per_env: int = 24
    learning_rate: float = 3.0e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_param: float = 0.2
    value_loss_coef: float = 1.0
    entropy_coef: float = 0.0002
    max_grad_norm: float = 1.0
    num_learning_epochs: int = 5
    num_mini_batches: int = 4
    init_noise_std: float = 0.15
    action_limit: float = 0.60
    log_std_min: float = -4.0
    log_std_max: float = -1.2
    hidden_dims: tuple[int, ...] = (512, 256, 128)
    save_interval: int = 50
    eval_interval: int = 100
    eval_episodes: int = 3
    mean_action_l2_coef: float = 0.02
    mean_action_saturation_coef: float = 4.0
    mean_action_saturation_threshold: float = 0.48
    eval_action_weight: float = 250.0
    eval_action_saturation_weight: float = 5000.0
    eval_action_saturation_threshold: float = 0.48
    eval_undesired_contact_weight: float = 100.0
    eval_contact_force_weight: float = 5.0
    eval_termination_weight: float = 1000.0
    eval_return_std_weight: float = 0.0
    eval_return_min_weight: float = 0.0
    eval_min_terrain_height_fraction: float = 0.0
    eval_min_forward_distance: float = 0.0
