#!/usr/bin/env python3
"""Train an M20 policy in MuJoCo."""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from datetime import datetime
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m20_mujoco_rl.config import (  # noqa: E402
    HeightScanConfig,
    M20EnvConfig,
    PPOConfig,
    RandomizationConfig,
    RewardConfig,
    TerrainConfig,
)
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402


TRAINING_PRESETS = {
    "official_like_stair": {
        "description": "Low-stair curriculum step with privileged critic, close to the current M20 rough-policy recipe.",
        "args": {
            "terrain": "stair_easy",
            "stair_height_range": (0.020, 0.023),
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 3000,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.02, 0.02),
            "base_y_range": (-0.02, 0.02),
            "base_yaw_range": (-0.03, 0.03),
            "forward_command_range": (0.30, 0.55),
            "progress_weight": 0.8,
            "terrain_height_progress_weight": 8.0,
            "stair_height_weight": 2.0,
            "undesired_contact_weight": -20.0,
            "contact_force_weight": -0.0003,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.75,
            "mean_action_l2_coef": 0.005,
            "mean_action_saturation_coef": 2.0,
            "mean_action_saturation_threshold": 0.70,
            "learning_rate": 5.0e-4,
            "entropy_coef": 0.003,
            "init_noise_std": 0.45,
            "log_std_min": -3.0,
            "log_std_max": -0.4,
            "eval_interval": 50,
            "eval_episodes": 3,
        },
    },
    "official_like_stair_soft_refine": {
        "description": "Keep the 2.3cm stair skill while pulling deterministic actions away from the action limit.",
        "args": {
            "terrain": "stair_easy",
            "stair_height": 0.023,
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 2000,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.015, 0.015),
            "base_y_range": (-0.015, 0.015),
            "base_yaw_range": (-0.02, 0.02),
            "forward_command_range": (0.25, 0.45),
            "progress_weight": 0.6,
            "terrain_height_progress_weight": 6.0,
            "stair_height_weight": 1.5,
            "undesired_contact_weight": -20.0,
            "contact_force_weight": -0.0003,
            "action_saturation_weight": -16.0,
            "action_saturation_threshold": 0.55,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.68,
            "mean_action_l2_coef": 0.04,
            "mean_action_saturation_coef": 10.0,
            "mean_action_saturation_threshold": 0.55,
            "learning_rate": 2.0e-4,
            "entropy_coef": 0.0008,
            "init_noise_std": 0.20,
            "log_std_min": -4.0,
            "log_std_max": -1.0,
            "eval_action_weight": 500.0,
            "eval_action_saturation_weight": 12000.0,
            "eval_action_saturation_threshold": 0.55,
            "eval_return_std_weight": 0.25,
            "eval_return_min_weight": 0.10,
            "eval_min_terrain_height_fraction": 0.90,
            "eval_min_forward_distance": 0.25,
            "eval_interval": 50,
            "eval_episodes": 10,
        },
    },
    "official_like_stair_vision_lift": {
        "description": "Actor height-scan stair stage with wheel lift/stumble rewards to escape rolling into stair edges.",
        "args": {
            "terrain": "stair_easy",
            "stair_height_range": (0.023, 0.026),
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 2500,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.02, 0.02),
            "base_y_range": (-0.015, 0.015),
            "base_yaw_range": (-0.02, 0.02),
            "forward_command_range": (0.18, 0.38),
            "include_height_scan": True,
            "height_scan_resolution": 0.10,
            "height_scan_size_x": 1.60,
            "height_scan_size_y": 1.00,
            "progress_weight": 0.5,
            "terrain_height_progress_weight": 7.0,
            "stair_height_weight": 1.8,
            "undesired_contact_weight": -20.0,
            "contact_force_weight": -0.0005,
            "action_saturation_weight": -10.0,
            "action_saturation_threshold": 0.62,
            "wheel_air_time_weight": 1.0,
            "wheel_air_time_threshold": 0.08,
            "wheel_clearance_weight": 2.0,
            "wheel_clearance_lift_target": 0.035,
            "wheel_clearance_terrain_threshold": 0.008,
            "wheel_stumble_weight": -3.0,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.68,
            "mean_action_l2_coef": 0.01,
            "mean_action_saturation_coef": 6.0,
            "mean_action_saturation_threshold": 0.62,
            "learning_rate": 2.5e-4,
            "entropy_coef": 0.0015,
            "init_noise_std": 0.28,
            "log_std_min": -4.0,
            "log_std_max": -0.8,
            "eval_action_weight": 350.0,
            "eval_action_saturation_weight": 6000.0,
            "eval_action_saturation_threshold": 0.62,
            "eval_return_std_weight": 0.20,
            "eval_return_min_weight": 0.10,
            "eval_min_terrain_height_fraction": 0.90,
            "eval_min_forward_distance": 0.25,
            "eval_interval": 50,
            "eval_episodes": 10,
        },
    },
    "official_like_stair_vision_lift_0030_refine": {
        "description": "Refine the 2.7cm height-scan stair policy toward 3.0cm while limiting action saturation.",
        "args": {
            "terrain": "stair_easy",
            "stair_height_range": (0.027, 0.030),
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 1800,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.015, 0.015),
            "base_y_range": (-0.012, 0.012),
            "base_yaw_range": (-0.015, 0.015),
            "forward_command_range": (0.16, 0.34),
            "include_height_scan": True,
            "height_scan_resolution": 0.10,
            "height_scan_size_x": 1.60,
            "height_scan_size_y": 1.00,
            "progress_weight": 0.45,
            "terrain_height_progress_weight": 9.0,
            "stair_height_weight": 2.2,
            "undesired_contact_weight": -22.0,
            "contact_force_weight": -0.0006,
            "action_saturation_weight": -12.0,
            "action_saturation_threshold": 0.66,
            "wheel_air_time_weight": 1.2,
            "wheel_air_time_threshold": 0.08,
            "wheel_clearance_weight": 2.6,
            "wheel_clearance_lift_target": 0.045,
            "wheel_clearance_terrain_threshold": 0.008,
            "wheel_stumble_weight": -4.0,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.72,
            "mean_action_l2_coef": 0.015,
            "mean_action_saturation_coef": 8.0,
            "mean_action_saturation_threshold": 0.66,
            "learning_rate": 1.2e-4,
            "entropy_coef": 0.0008,
            "init_noise_std": 0.18,
            "log_std_min": -4.0,
            "log_std_max": -1.1,
            "eval_action_weight": 420.0,
            "eval_action_saturation_weight": 8000.0,
            "eval_action_saturation_threshold": 0.66,
            "eval_return_std_weight": 0.25,
            "eval_return_min_weight": 0.15,
            "eval_min_terrain_height_fraction": 0.90,
            "eval_min_forward_distance": 0.25,
            "eval_interval": 50,
            "eval_episodes": 10,
        },
    },
    "official_like_stair_vision_lift_0027_desat": {
        "description": "Keep the 2.7cm stair skill while pulling the height-scan policy away from saturated actions.",
        "args": {
            "terrain": "stair_easy",
            "stair_height_range": (0.026, 0.027),
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 1400,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.015, 0.015),
            "base_y_range": (-0.012, 0.012),
            "base_yaw_range": (-0.015, 0.015),
            "forward_command_range": (0.18, 0.36),
            "include_height_scan": True,
            "height_scan_resolution": 0.10,
            "height_scan_size_x": 1.60,
            "height_scan_size_y": 1.00,
            "progress_weight": 0.45,
            "terrain_height_progress_weight": 7.5,
            "stair_height_weight": 1.8,
            "undesired_contact_weight": -22.0,
            "contact_force_weight": -0.0006,
            "action_saturation_weight": -20.0,
            "action_saturation_threshold": 0.56,
            "wheel_air_time_weight": 1.0,
            "wheel_air_time_threshold": 0.08,
            "wheel_clearance_weight": 2.2,
            "wheel_clearance_lift_target": 0.040,
            "wheel_clearance_terrain_threshold": 0.008,
            "wheel_stumble_weight": -4.0,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.68,
            "mean_action_l2_coef": 0.04,
            "mean_action_saturation_coef": 18.0,
            "mean_action_saturation_threshold": 0.56,
            "learning_rate": 1.0e-4,
            "entropy_coef": 0.0005,
            "init_noise_std": 0.12,
            "log_std_min": -4.0,
            "log_std_max": -1.4,
            "eval_action_weight": 700.0,
            "eval_action_saturation_weight": 16000.0,
            "eval_action_saturation_threshold": 0.56,
            "eval_return_std_weight": 0.30,
            "eval_return_min_weight": 0.15,
            "eval_min_terrain_height_fraction": 0.90,
            "eval_min_forward_distance": 0.25,
            "eval_interval": 50,
            "eval_episodes": 10,
        },
    },
    "official_like_stair_vision_lift_0027_progress": {
        "description": "Resume the low-action 2.7cm candidate and push it across later stair steps instead of stopping on the first step.",
        "args": {
            "terrain": "stair_easy",
            "stair_height_range": (0.026, 0.027),
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 1200,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.015, 0.015),
            "base_y_range": (-0.012, 0.012),
            "base_yaw_range": (-0.015, 0.015),
            "forward_command_range": (0.24, 0.44),
            "include_height_scan": True,
            "height_scan_resolution": 0.10,
            "height_scan_size_x": 1.60,
            "height_scan_size_y": 1.00,
            "progress_weight": 0.95,
            "terrain_height_progress_weight": 6.5,
            "stair_height_weight": 0.9,
            "stair_forward_progress_weight": 1.4,
            "stair_forward_progress_height_fraction": 0.5,
            "undesired_contact_weight": -22.0,
            "contact_force_weight": -0.0006,
            "action_saturation_weight": -12.0,
            "action_saturation_threshold": 0.60,
            "wheel_air_time_weight": 0.8,
            "wheel_air_time_threshold": 0.08,
            "wheel_clearance_weight": 2.2,
            "wheel_clearance_lift_target": 0.040,
            "wheel_clearance_terrain_threshold": 0.008,
            "wheel_stumble_weight": -4.0,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.68,
            "mean_action_l2_coef": 0.025,
            "mean_action_saturation_coef": 10.0,
            "mean_action_saturation_threshold": 0.60,
            "learning_rate": 8.0e-5,
            "entropy_coef": 0.0007,
            "init_noise_std": 0.10,
            "log_std_min": -4.0,
            "log_std_max": -1.5,
            "eval_action_weight": 480.0,
            "eval_action_saturation_weight": 9000.0,
            "eval_action_saturation_threshold": 0.60,
            "eval_return_std_weight": 0.25,
            "eval_return_min_weight": 0.15,
            "eval_min_terrain_height_fraction": 1.80,
            "eval_min_forward_distance": 0.50,
            "eval_interval": 50,
            "eval_episodes": 10,
        },
    },
    "official_like_stair_vision_lift_0027_progress_v2": {
        "description": "Guard the 2.7cm first-step skill while nudging forward progress, with valid-only eval candidate saving.",
        "args": {
            "terrain": "stair_easy",
            "stair_height_range": (0.026, 0.027),
            "stair_depth": 0.35,
            "stair_start_x": 1.00,
            "stair_count": 3,
            "iterations": 900,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.85,
            "base_init_y": 0.0,
            "base_init_height": 0.62,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.015, 0.015),
            "base_y_range": (-0.012, 0.012),
            "base_yaw_range": (-0.015, 0.015),
            "forward_command_range": (0.20, 0.38),
            "include_height_scan": True,
            "height_scan_resolution": 0.10,
            "height_scan_size_x": 1.60,
            "height_scan_size_y": 1.00,
            "progress_weight": 0.75,
            "terrain_height_progress_weight": 7.0,
            "stair_height_weight": 1.4,
            "stair_forward_progress_weight": 0.8,
            "stair_forward_progress_height_fraction": 0.5,
            "undesired_contact_weight": -22.0,
            "contact_force_weight": -0.0006,
            "action_saturation_weight": -10.0,
            "action_saturation_threshold": 0.62,
            "wheel_air_time_weight": 0.8,
            "wheel_air_time_threshold": 0.08,
            "wheel_clearance_weight": 2.0,
            "wheel_clearance_lift_target": 0.040,
            "wheel_clearance_terrain_threshold": 0.008,
            "wheel_stumble_weight": -4.0,
            "terminate_on_undesired_contact": True,
            "undesired_contact_termination_steps": 2,
            "undesired_contact_terminal_penalty": -500.0,
            "action_limit": 0.68,
            "mean_action_l2_coef": 0.018,
            "mean_action_saturation_coef": 7.0,
            "mean_action_saturation_threshold": 0.62,
            "learning_rate": 6.0e-5,
            "entropy_coef": 0.0005,
            "init_noise_std": 0.08,
            "log_std_min": -4.0,
            "log_std_max": -1.6,
            "eval_action_weight": 420.0,
            "eval_action_saturation_weight": 6000.0,
            "eval_action_saturation_threshold": 0.62,
            "eval_return_std_weight": 0.25,
            "eval_return_min_weight": 0.15,
            "eval_min_terrain_height_fraction": 0.90,
            "eval_min_forward_distance": 0.25,
            "eval_interval": 50,
            "eval_episodes": 10,
        },
    },
    "official_like_rough": {
        "description": "Rough-terrain stage inspired by official boxes/random_rough training before fixed-stair evaluation.",
        "args": {
            "terrain": "random_boxes",
            "iterations": 6000,
            "num_envs": 8,
            "steps_per_env": 24,
            "episode_seconds": 8.0,
            "base_init_x": 0.0,
            "base_init_y": 0.0,
            "base_init_height": 0.58,
            "reset_settle_seconds": 0.5,
            "base_x_range": (-0.20, 0.20),
            "base_y_range": (-0.15, 0.15),
            "base_yaw_range": (-0.30, 0.30),
            "forward_command_range": (0.20, 0.55),
            "progress_weight": 0.6,
            "terrain_height_progress_weight": 2.0,
            "stair_height_weight": 0.6,
            "undesired_contact_weight": -8.0,
            "contact_force_weight": -0.0002,
            "action_limit": 0.75,
            "mean_action_l2_coef": 0.005,
            "mean_action_saturation_coef": 2.0,
            "mean_action_saturation_threshold": 0.70,
            "learning_rate": 5.0e-4,
            "entropy_coef": 0.005,
            "init_noise_std": 0.60,
            "log_std_min": -3.0,
            "log_std_max": -0.2,
            "eval_interval": 100,
            "eval_episodes": 3,
        },
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train M20 MuJoCo RL policy.")
    parser.add_argument(
        "--list-presets",
        action="store_true",
        help="Print available training presets and exit.",
    )
    parser.add_argument(
        "--preset",
        default=None,
        choices=sorted(TRAINING_PRESETS),
        help="Apply an official-style training preset; explicit CLI arguments still override it.",
    )
    parser.add_argument("--terrain", default="stair_official", choices=["flat", "stair_easy", "random_boxes", "stair_official"])
    parser.add_argument("--model-xml", default=None, help="Optional custom MuJoCo XML path.")
    parser.add_argument("--num-envs", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--steps-per-env", type=int, default=24)
    parser.add_argument("--eval-interval", type=int, default=100, help="Iterations between fixed deterministic evals; 0 disables.")
    parser.add_argument("--eval-episodes", type=int, default=3, help="Episodes per fixed deterministic eval.")
    parser.add_argument("--action-limit", type=float, default=None, help="Override normalized policy action limit.")
    parser.add_argument("--learning-rate", type=float, default=None, help="Override PPO learning rate.")
    parser.add_argument("--entropy-coef", type=float, default=None, help="Override PPO entropy coefficient.")
    parser.add_argument("--init-noise-std", type=float, default=None, help="Override initial policy action std.")
    parser.add_argument("--log-std-min", type=float, default=None, help="Override minimum policy log standard deviation.")
    parser.add_argument("--log-std-max", type=float, default=None, help="Override maximum policy log standard deviation.")
    parser.add_argument("--mean-action-l2-coef", type=float, default=None, help="Override actor mean action L2 regularization.")
    parser.add_argument(
        "--mean-action-saturation-coef",
        type=float,
        default=None,
        help="Override actor mean action saturation regularization.",
    )
    parser.add_argument(
        "--mean-action-saturation-threshold",
        type=float,
        default=None,
        help="Override actor mean action saturation threshold.",
    )
    parser.add_argument("--eval-action-weight", type=float, default=None, help="Override fixed eval action magnitude penalty.")
    parser.add_argument(
        "--eval-action-saturation-weight",
        type=float,
        default=None,
        help="Override fixed eval action saturation penalty.",
    )
    parser.add_argument(
        "--eval-action-saturation-threshold",
        type=float,
        default=None,
        help="Override fixed eval action saturation threshold.",
    )
    parser.add_argument(
        "--eval-undesired-contact-weight",
        type=float,
        default=None,
        help="Override fixed eval undesired contact penalty.",
    )
    parser.add_argument(
        "--eval-contact-force-weight",
        type=float,
        default=None,
        help="Override fixed eval contact force penalty.",
    )
    parser.add_argument("--eval-return-std-weight", type=float, default=None, help="Penalty weight for eval return std.")
    parser.add_argument("--eval-return-min-weight", type=float, default=None, help="Bonus weight for eval worst episode return.")
    parser.add_argument(
        "--eval-min-terrain-height-fraction",
        type=float,
        default=None,
        help="Minimum fraction of the target stair height required before an eval can become best.",
    )
    parser.add_argument(
        "--eval-min-forward-distance",
        type=float,
        default=None,
        help="Minimum fixed-eval forward distance required before an eval can become best.",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", default="cpu", help="cpu or cuda")
    parser.add_argument("--log-dir", default="logs/m20_mujoco")
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--resume-checkpoint", default=None, help="Warm-start policy weights from a checkpoint.")
    parser.add_argument("--command-mode", default="forward", choices=["forward", "random"])
    parser.add_argument("--episode-seconds", type=float, default=8.0)
    parser.add_argument("--base-init-x", type=float, default=0.0, help="Nominal initial base x position.")
    parser.add_argument("--base-init-y", type=float, default=0.0, help="Nominal initial base y position.")
    parser.add_argument("--base-init-height", type=float, default=None, help="Nominal initial base z position.")
    parser.add_argument(
        "--reset-settle-seconds",
        type=float,
        default=None,
        help="Physics settle time after reset before the episode starts.",
    )
    parser.add_argument("--base-x-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"), help="Reset x noise around base-init-x.")
    parser.add_argument("--base-y-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"), help="Reset y noise around base-init-y.")
    parser.add_argument("--base-yaw-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"), help="Reset yaw noise range.")
    parser.add_argument(
        "--forward-command-range",
        type=float,
        nargs=2,
        default=None,
        metavar=("MIN", "MAX"),
        help="Override sampled forward velocity command range.",
    )
    parser.add_argument("--progress-weight", type=float, default=None, help="Override forward progress reward weight.")
    parser.add_argument(
        "--terrain-height-progress-weight",
        type=float,
        default=None,
        help="Override reward for increasing terrain height under the base.",
    )
    parser.add_argument("--stair-height-weight", type=float, default=None, help="Override reward for being on higher terrain.")
    parser.add_argument(
        "--stair-forward-progress-weight",
        type=float,
        default=None,
        help="Override forward progress reward while the base is already on stair terrain.",
    )
    parser.add_argument(
        "--stair-forward-progress-height-fraction",
        type=float,
        default=None,
        help="Fraction of one stair height needed before stair-forward progress reward is active.",
    )
    parser.add_argument("--undesired-contact-weight", type=float, default=None, help="Override non-wheel contact reward weight.")
    parser.add_argument("--contact-force-weight", type=float, default=None, help="Override contact force reward weight.")
    parser.add_argument("--contact-force-threshold", type=float, default=None, help="Override contact force threshold.")
    parser.add_argument("--action-saturation-weight", type=float, default=None, help="Override per-step action saturation reward weight.")
    parser.add_argument("--action-saturation-threshold", type=float, default=None, help="Override per-step action saturation threshold.")
    parser.add_argument("--wheel-air-time-weight", type=float, default=None, help="Override wheel air-time reward weight.")
    parser.add_argument("--wheel-air-time-threshold", type=float, default=None, help="Minimum wheel air time before landing reward.")
    parser.add_argument("--wheel-clearance-weight", type=float, default=None, help="Override wheel clearance reward weight.")
    parser.add_argument("--wheel-clearance-lift-target", type=float, default=None, help="Wheel-center lift target above nominal clearance.")
    parser.add_argument(
        "--wheel-clearance-terrain-threshold",
        type=float,
        default=None,
        help="Minimum front terrain height delta before wheel clearance reward is active.",
    )
    parser.add_argument("--wheel-stumble-weight", type=float, default=None, help="Override vertical-face wheel contact penalty.")
    parser.add_argument(
        "--include-height-scan",
        action="store_true",
        help="Append yaw-aligned height scan to actor observations. Changes policy input dimension.",
    )
    parser.add_argument(
        "--no-critic-height-scan",
        action="store_true",
        help="Disable privileged MuJoCo height scan for the critic/value network.",
    )
    parser.add_argument(
        "--no-critic-base-lin-vel",
        action="store_true",
        help="Disable privileged base linear velocity for the critic/value network.",
    )
    parser.add_argument("--height-scan-resolution", type=float, default=None, help="Privileged critic scan grid resolution.")
    parser.add_argument("--height-scan-size-x", type=float, default=None, help="Privileged critic scan grid length.")
    parser.add_argument("--height-scan-size-y", type=float, default=None, help="Privileged critic scan grid width.")
    contact_termination_group = parser.add_mutually_exclusive_group()
    contact_termination_group.add_argument(
        "--terminate-on-undesired-contact",
        action="store_true",
        default=None,
        help="End episode when a non-wheel robot body contacts terrain/objects.",
    )
    contact_termination_group.add_argument(
        "--allow-undesired-contact",
        dest="terminate_on_undesired_contact",
        action="store_false",
        default=None,
        help="Keep episodes running after non-wheel contacts, overriding presets that terminate.",
    )
    parser.add_argument(
        "--undesired-contact-termination-threshold",
        type=float,
        default=None,
        help="Non-wheel contact count above which contact termination triggers.",
    )
    parser.add_argument(
        "--undesired-contact-terminal-penalty",
        type=float,
        default=None,
        help="One-time reward penalty when non-wheel contact termination triggers.",
    )
    parser.add_argument(
        "--undesired-contact-termination-steps",
        type=int,
        default=None,
        help="Require this many consecutive non-wheel contact steps before termination.",
    )
    parser.add_argument("--no-randomization", action="store_true", help="Disable domain/reset randomization.")
    parser.add_argument("--stair-height", type=float, default=None, help="Override stair height for stair_easy.")
    parser.add_argument(
        "--stair-height-range",
        type=float,
        nargs=2,
        default=None,
        metavar=("MIN", "MAX"),
        help="Distribute stair_easy heights across train envs; fixed eval uses MAX.",
    )
    parser.add_argument("--stair-depth", type=float, default=None, help="Override stair depth for stair_easy.")
    parser.add_argument("--stair-start-x", type=float, default=None, help="Override first stair start x for stair_easy.")
    parser.add_argument("--stair-count", type=int, default=None, help="Override stair count for stair_easy.")
    parser.add_argument("--render", action="store_true", help="Open a MuJoCo viewer for one training environment.")
    parser.add_argument("--render-env", type=int, default=0, help="Which environment index to render.")
    parser.add_argument(
        "--render-real-time",
        action="store_true",
        help="Sleep during rendering so the selected MuJoCo environment plays close to real time.",
    )
    args = parser.parse_args()
    if args.list_presets:
        _print_presets()
        raise SystemExit(0)
    _apply_preset(args, parser)
    return args


def _print_presets() -> None:
    for name, preset in TRAINING_PRESETS.items():
        print(f"{name}: {preset['description']}")
        for key, value in preset["args"].items():
            print(f"  {key}: {value}")


def _apply_preset(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.preset is None:
        return

    preset = TRAINING_PRESETS[args.preset]["args"]
    parser_defaults = {action.dest: action.default for action in parser._actions}
    for key, value in preset.items():
        current_value = getattr(args, key)
        if current_value == parser_defaults.get(key):
            setattr(args, key, value)


def main() -> None:
    args = parse_args()

    terrain_cfg = TerrainConfig(name=args.terrain, seed=args.seed)
    if args.terrain == "stair_easy":
        if args.stair_height is not None:
            terrain_cfg.stair_height = args.stair_height
        if args.stair_height_range is not None:
            terrain_cfg.stair_height_range = tuple(args.stair_height_range)
            terrain_cfg.stair_height = float(args.stair_height_range[1])
        if args.stair_depth is not None:
            terrain_cfg.stair_depth = args.stair_depth
        if args.stair_start_x is not None:
            terrain_cfg.stair_start_x = args.stair_start_x
        if args.stair_count is not None:
            terrain_cfg.stair_count = args.stair_count
    elif args.terrain == "flat":
        terrain_cfg.stair_height = 0.0

    randomization_cfg = RandomizationConfig(enabled=not args.no_randomization)
    if args.base_x_range is not None:
        randomization_cfg.base_x_range = tuple(args.base_x_range)
    if args.base_y_range is not None:
        randomization_cfg.base_y_range = tuple(args.base_y_range)
    if args.base_yaw_range is not None:
        randomization_cfg.base_yaw_range = tuple(args.base_yaw_range)

    reward_cfg = RewardConfig()
    if args.progress_weight is not None:
        reward_cfg.progress_weight = args.progress_weight
    if args.terrain_height_progress_weight is not None:
        reward_cfg.terrain_height_progress_weight = args.terrain_height_progress_weight
    if args.stair_height_weight is not None:
        reward_cfg.stair_height_weight = args.stair_height_weight
    if args.stair_forward_progress_weight is not None:
        reward_cfg.stair_forward_progress_weight = args.stair_forward_progress_weight
    if args.stair_forward_progress_height_fraction is not None:
        reward_cfg.stair_forward_progress_height_fraction = args.stair_forward_progress_height_fraction
    if args.undesired_contact_weight is not None:
        reward_cfg.undesired_contact_weight = args.undesired_contact_weight
    if args.contact_force_weight is not None:
        reward_cfg.contact_force_weight = args.contact_force_weight
    if args.contact_force_threshold is not None:
        reward_cfg.contact_force_threshold = args.contact_force_threshold
    if args.action_saturation_weight is not None:
        reward_cfg.action_saturation_weight = args.action_saturation_weight
    if args.action_saturation_threshold is not None:
        reward_cfg.action_saturation_threshold = args.action_saturation_threshold
    if args.wheel_air_time_weight is not None:
        reward_cfg.wheel_air_time_weight = args.wheel_air_time_weight
    if args.wheel_air_time_threshold is not None:
        reward_cfg.wheel_air_time_threshold = args.wheel_air_time_threshold
    if args.wheel_clearance_weight is not None:
        reward_cfg.wheel_clearance_weight = args.wheel_clearance_weight
    if args.wheel_clearance_lift_target is not None:
        reward_cfg.wheel_clearance_lift_target = args.wheel_clearance_lift_target
    if args.wheel_clearance_terrain_threshold is not None:
        reward_cfg.wheel_clearance_terrain_threshold = args.wheel_clearance_terrain_threshold
    if args.wheel_stumble_weight is not None:
        reward_cfg.wheel_stumble_weight = args.wheel_stumble_weight

    height_scan_cfg = HeightScanConfig()
    if args.height_scan_resolution is not None:
        height_scan_cfg.resolution = args.height_scan_resolution
    if args.height_scan_size_x is not None:
        height_scan_cfg.size_x = args.height_scan_size_x
    if args.height_scan_size_y is not None:
        height_scan_cfg.size_y = args.height_scan_size_y

    forward_command_range = (
        tuple(args.forward_command_range)
        if args.forward_command_range is not None
        else M20EnvConfig().forward_command_range
    )

    env_cfg = M20EnvConfig(
        model_xml=args.model_xml,
        terrain=terrain_cfg,
        randomization=randomization_cfg,
        reward=reward_cfg,
        height_scan=height_scan_cfg,
        seed=args.seed,
        command_mode=args.command_mode,
        episode_seconds=args.episode_seconds,
        base_init_x=args.base_init_x,
        base_init_y=args.base_init_y,
        forward_command_range=forward_command_range,
    )
    env_cfg.critic_base_lin_vel = not args.no_critic_base_lin_vel
    env_cfg.critic_height_scan = not args.no_critic_height_scan
    env_cfg.include_height_scan = bool(args.include_height_scan)
    if args.base_init_height is not None:
        env_cfg.base_init_height = args.base_init_height
    if args.reset_settle_seconds is not None:
        env_cfg.reset_settle_seconds = args.reset_settle_seconds
    if args.terminate_on_undesired_contact is not None:
        env_cfg.terminate_on_undesired_contact = args.terminate_on_undesired_contact
    if args.undesired_contact_termination_threshold is not None:
        env_cfg.undesired_contact_termination_threshold = args.undesired_contact_termination_threshold
    if args.undesired_contact_terminal_penalty is not None:
        env_cfg.undesired_contact_terminal_penalty = args.undesired_contact_terminal_penalty
    if args.undesired_contact_termination_steps is not None:
        env_cfg.undesired_contact_termination_steps = args.undesired_contact_termination_steps

    ppo_kwargs = {
        "seed": args.seed,
        "num_envs": args.num_envs,
        "iterations": args.iterations,
        "steps_per_env": args.steps_per_env,
        "eval_interval": args.eval_interval,
        "eval_episodes": args.eval_episodes,
    }
    optional_ppo_args = {
        "action_limit": args.action_limit,
        "learning_rate": args.learning_rate,
        "entropy_coef": args.entropy_coef,
        "init_noise_std": args.init_noise_std,
        "log_std_min": args.log_std_min,
        "log_std_max": args.log_std_max,
        "mean_action_l2_coef": args.mean_action_l2_coef,
        "mean_action_saturation_coef": args.mean_action_saturation_coef,
        "mean_action_saturation_threshold": args.mean_action_saturation_threshold,
        "eval_action_weight": args.eval_action_weight,
        "eval_action_saturation_weight": args.eval_action_saturation_weight,
        "eval_action_saturation_threshold": args.eval_action_saturation_threshold,
        "eval_undesired_contact_weight": args.eval_undesired_contact_weight,
        "eval_contact_force_weight": args.eval_contact_force_weight,
        "eval_return_std_weight": args.eval_return_std_weight,
        "eval_return_min_weight": args.eval_return_min_weight,
        "eval_min_terrain_height_fraction": args.eval_min_terrain_height_fraction,
        "eval_min_forward_distance": args.eval_min_forward_distance,
    }
    ppo_kwargs.update({key: value for key, value in optional_ppo_args.items() if value is not None})
    ppo_cfg = PPOConfig(**ppo_kwargs)

    run_name = args.run_name or f"{args.terrain}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    log_dir = Path(args.log_dir).expanduser().resolve() / run_name

    def make_env(env_id: int) -> M20MujocoEnv:
        terrain = _terrain_for_env(env_cfg.terrain, args.seed + env_id, env_id, args.num_envs, eval_mode=False)
        cfg = replace(env_cfg, seed=args.seed + env_id, terrain=terrain)
        return M20MujocoEnv(cfg)

    eval_env_cfg = replace(env_cfg, randomization=RandomizationConfig(enabled=False))

    def make_eval_env(eval_id: int) -> M20MujocoEnv:
        eval_seed = args.seed + 100_000 + eval_id
        terrain = _terrain_for_env(eval_env_cfg.terrain, eval_seed, eval_id, args.eval_episodes, eval_mode=True)
        cfg = replace(eval_env_cfg, seed=eval_seed, terrain=terrain)
        return M20MujocoEnv(cfg)

    print(f"[INFO] log_dir={log_dir}")
    if args.preset is not None:
        print(f"[INFO] preset={args.preset}: {TRAINING_PRESETS[args.preset]['description']}")
    print(f"[INFO] terrain={args.terrain}, num_envs={args.num_envs}, iterations={args.iterations}")
    print(f"[INFO] randomization={not args.no_randomization}")
    print(
        "[INFO] base_init="
        f"({env_cfg.base_init_x}, {env_cfg.base_init_y}, {env_cfg.base_init_height}), "
        f"reset_settle_seconds={env_cfg.reset_settle_seconds}, "
        f"base_x_range={env_cfg.randomization.base_x_range}, "
        f"forward_command_range={env_cfg.forward_command_range}"
    )
    print(
        "[INFO] reward "
        f"progress={env_cfg.reward.progress_weight}, "
        f"terrain_height_progress={env_cfg.reward.terrain_height_progress_weight}, "
        f"stair_height={env_cfg.reward.stair_height_weight}, "
        f"stair_forward_progress={env_cfg.reward.stair_forward_progress_weight}, "
        f"action_saturation_weight={env_cfg.reward.action_saturation_weight}, "
        f"action_saturation_threshold={env_cfg.reward.action_saturation_threshold}, "
        f"wheel_clearance={env_cfg.reward.wheel_clearance_weight}, "
        f"wheel_air_time={env_cfg.reward.wheel_air_time_weight}, "
        f"wheel_stumble={env_cfg.reward.wheel_stumble_weight}"
    )
    print(
        "[INFO] contact "
        f"undesired_weight={env_cfg.reward.undesired_contact_weight}, "
        f"force_weight={env_cfg.reward.contact_force_weight}, "
        f"terminate_on_undesired={env_cfg.terminate_on_undesired_contact}, "
        f"threshold={env_cfg.undesired_contact_termination_threshold}, "
        f"steps={env_cfg.undesired_contact_termination_steps}"
    )
    print(
        "[INFO] privileged_critic "
        f"base_lin_vel={env_cfg.critic_base_lin_vel}, "
        f"height_scan={env_cfg.critic_height_scan}, "
        f"actor_height_scan={env_cfg.include_height_scan}, "
        f"grid={env_cfg.height_scan.size_x}x{env_cfg.height_scan.size_y}@{env_cfg.height_scan.resolution}"
    )
    if args.eval_interval > 0:
        print(f"[INFO] fixed_eval_interval={args.eval_interval}, eval_episodes={args.eval_episodes}")
    print(
        "[INFO] action_limit="
        f"{ppo_cfg.action_limit}, mean_action_l2_coef={ppo_cfg.mean_action_l2_coef}, "
        f"mean_action_saturation_coef={ppo_cfg.mean_action_saturation_coef}, "
        f"mean_action_saturation_threshold={ppo_cfg.mean_action_saturation_threshold}"
    )
    print(
        "[INFO] ppo "
        f"lr={ppo_cfg.learning_rate}, entropy_coef={ppo_cfg.entropy_coef}, "
        f"init_noise_std={ppo_cfg.init_noise_std}, "
        f"log_std_range=({ppo_cfg.log_std_min}, {ppo_cfg.log_std_max})"
    )
    if args.resume_checkpoint is not None:
        print(f"[INFO] resume_checkpoint={args.resume_checkpoint}")
    try:
        from m20_mujoco_rl.ppo import train_ppo
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing training dependency. Run: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    checkpoint = train_ppo(
        make_env,
        env_cfg,
        ppo_cfg,
        log_dir,
        device=args.device,
        render_env_id=args.render_env if args.render else None,
        render_real_time=args.render_real_time,
        resume_checkpoint=args.resume_checkpoint,
        eval_make_env=make_eval_env if args.eval_interval > 0 else None,
    )
    print(f"[INFO] latest checkpoint: {checkpoint}")


def _terrain_for_env(
    terrain_cfg: TerrainConfig,
    seed: int,
    env_id: int,
    env_count: int,
    eval_mode: bool,
) -> TerrainConfig:
    terrain = replace(terrain_cfg, seed=seed)
    if terrain.stair_height_range is None:
        return terrain

    low, high = terrain.stair_height_range
    if high < low:
        low, high = high, low
    if eval_mode or env_count <= 1 or high <= low:
        height = high
    else:
        frac = (env_id % env_count) / max(1, env_count - 1)
        height = low + frac * (high - low)
    return replace(terrain, stair_height=float(height))


if __name__ == "__main__":
    main()
