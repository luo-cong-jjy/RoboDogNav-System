#!/usr/bin/env python3
"""Play a trained M20 MuJoCo policy."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m20_mujoco_rl.config import M20EnvConfig, RandomizationConfig, RewardConfig, TerrainConfig  # noqa: E402
from m20_mujoco_rl.config import HeightScanConfig  # noqa: E402
from m20_mujoco_rl.constants import OBS_DIM  # noqa: E402
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Play M20 MuJoCo checkpoint.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--terrain", default="stair_official", choices=["flat", "stair_easy", "random_boxes", "stair_official"])
    parser.add_argument("--model-xml", default=None)
    parser.add_argument("--stair-height", type=float, default=None)
    parser.add_argument("--stair-depth", type=float, default=None)
    parser.add_argument("--stair-start-x", type=float, default=None)
    parser.add_argument("--stair-count", type=int, default=None)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument(
        "--episode-seconds",
        type=float,
        default=None,
        help="Episode length before automatic reset. Defaults to --seconds for visual playback.",
    )
    parser.add_argument("--base-init-x", type=float, default=0.0, help="Initial base x position for playback.")
    parser.add_argument("--base-init-y", type=float, default=0.0, help="Initial base y position for playback.")
    parser.add_argument("--base-init-height", type=float, default=None, help="Initial base z position for playback.")
    parser.add_argument("--reset-settle-seconds", type=float, default=None, help="Settle time after reset before playback starts.")
    parser.add_argument("--undesired-contact-weight", type=float, default=None)
    parser.add_argument("--contact-force-weight", type=float, default=None)
    parser.add_argument("--terminate-on-undesired-contact", action="store_true")
    parser.add_argument("--undesired-contact-terminal-penalty", type=float, default=None)
    parser.add_argument("--undesired-contact-termination-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=1)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    try:
        import torch
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing playback dependency. Run: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    from m20_mujoco_rl.export import load_actor_critic

    ckpt = torch.load(args.checkpoint, map_location="cpu")
    checkpoint_env_cfg = ckpt.get("env_cfg", {}) if isinstance(ckpt, dict) else {}
    checkpoint_actor_obs_dim = int(ckpt.get("actor_obs_dim", ckpt.get("obs_dim", OBS_DIM))) if isinstance(ckpt, dict) else OBS_DIM
    checkpoint_include_height_scan = bool(checkpoint_env_cfg.get("include_height_scan", False)) or checkpoint_actor_obs_dim > OBS_DIM

    terrain_cfg = TerrainConfig(name=args.terrain, seed=args.seed)
    if args.stair_height is not None:
        terrain_cfg.stair_height = args.stair_height
    if args.stair_depth is not None:
        terrain_cfg.stair_depth = args.stair_depth
    if args.stair_start_x is not None:
        terrain_cfg.stair_start_x = args.stair_start_x
    if args.stair_count is not None:
        terrain_cfg.stair_count = args.stair_count

    reward_cfg = RewardConfig()
    checkpoint_reward_cfg = checkpoint_env_cfg.get("reward", {})
    if isinstance(checkpoint_reward_cfg, dict):
        for key, value in checkpoint_reward_cfg.items():
            if hasattr(reward_cfg, key):
                setattr(reward_cfg, key, value)
    if args.undesired_contact_weight is not None:
        reward_cfg.undesired_contact_weight = args.undesired_contact_weight
    if args.contact_force_weight is not None:
        reward_cfg.contact_force_weight = args.contact_force_weight

    height_scan_cfg = HeightScanConfig()
    checkpoint_height_scan = checkpoint_env_cfg.get("height_scan", {})
    if isinstance(checkpoint_height_scan, dict):
        for key in ("resolution", "size_x", "size_y", "z_offset", "clip"):
            if key in checkpoint_height_scan:
                setattr(height_scan_cfg, key, checkpoint_height_scan[key])

    env_cfg = M20EnvConfig(
        model_xml=args.model_xml,
        terrain=terrain_cfg,
        randomization=RandomizationConfig(enabled=False),
        reward=reward_cfg,
        height_scan=height_scan_cfg,
        seed=args.seed,
        episode_seconds=args.episode_seconds if args.episode_seconds is not None else args.seconds,
        base_init_x=args.base_init_x,
        base_init_y=args.base_init_y,
    )
    if args.base_init_height is not None:
        env_cfg.base_init_height = args.base_init_height
    if args.reset_settle_seconds is not None:
        env_cfg.reset_settle_seconds = args.reset_settle_seconds
    env_cfg.terminate_on_undesired_contact = args.terminate_on_undesired_contact
    if args.undesired_contact_terminal_penalty is not None:
        env_cfg.undesired_contact_terminal_penalty = args.undesired_contact_terminal_penalty
    if args.undesired_contact_termination_steps is not None:
        env_cfg.undesired_contact_termination_steps = args.undesired_contact_termination_steps
    env_cfg.include_height_scan = checkpoint_include_height_scan
    env = M20MujocoEnv(env_cfg)
    policy = load_actor_critic(args.checkpoint, device=args.device)
    if env.observation_dim != policy.actor_obs_dim:
        raise RuntimeError(
            f"Policy/env observation dim mismatch: policy={policy.actor_obs_dim}, env={env.observation_dim}"
        )

    obs = env.reset(seed=args.seed)
    end_time = time.time() + args.seconds
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        while viewer.is_running() and time.time() < end_time:
            obs_t = torch.as_tensor(obs[None, :], dtype=torch.float32, device=args.device)
            with torch.no_grad():
                action = policy.deterministic_action(obs_t).cpu().numpy()[0]
            obs, reward, done, info = env.step(action)
            if done:
                print(f"[INFO] episode_return={info['episode_return']:.3f}, reset")
                obs = env.reset()
            viewer.sync()
            time.sleep(env.cfg.control_dt)


if __name__ == "__main__":
    main()
