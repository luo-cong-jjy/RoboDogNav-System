#!/usr/bin/env python3
"""Headless evaluation for an M20 MuJoCo policy checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m20_mujoco_rl.config import M20EnvConfig, RandomizationConfig, RewardConfig, TerrainConfig  # noqa: E402
from m20_mujoco_rl.config import HeightScanConfig  # noqa: E402
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402
from m20_mujoco_rl.constants import OBS_DIM  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate M20 MuJoCo checkpoint without a viewer.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--terrain", default="stair_official", choices=["flat", "stair_easy", "random_boxes", "stair_official"])
    parser.add_argument("--model-xml", default=None)
    parser.add_argument("--stair-height", type=float, default=None)
    parser.add_argument("--stair-depth", type=float, default=None)
    parser.add_argument("--stair-start-x", type=float, default=None)
    parser.add_argument("--stair-count", type=int, default=None)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--episode-seconds", type=float, default=8.0)
    parser.add_argument("--base-init-x", type=float, default=0.0)
    parser.add_argument("--base-init-y", type=float, default=0.0)
    parser.add_argument("--base-init-height", type=float, default=None)
    parser.add_argument("--reset-settle-seconds", type=float, default=None)
    parser.add_argument("--undesired-contact-weight", type=float, default=None)
    parser.add_argument("--contact-force-weight", type=float, default=None)
    parser.add_argument("--terminate-on-undesired-contact", action="store_true")
    parser.add_argument("--undesired-contact-terminal-penalty", type=float, default=None)
    parser.add_argument("--undesired-contact-termination-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=100)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--command-mode", default="forward", choices=["forward", "random"])
    parser.add_argument("--forward-command-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
    parser.add_argument("--json-output", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        import torch
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing evaluation dependency. Run: "
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
        command_mode=args.command_mode,
        episode_seconds=args.episode_seconds,
        base_init_x=args.base_init_x,
        base_init_y=args.base_init_y,
    )
    if args.forward_command_range is not None:
        env_cfg.forward_command_range = tuple(args.forward_command_range)
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

    episodes = []
    for episode_idx in range(args.episodes):
        obs = env.reset(seed=args.seed + episode_idx)
        start_x = float(env.data.qpos[0])
        reward_sum = 0.0
        action_abs_sum = 0.0
        action_abs_count = 0
        action_abs_max = 0.0
        reward_term_sums: dict[str, float] = {}
        max_x = start_x
        max_terrain_height = 0.0
        steps = 0
        info = {}
        done = False

        while not done:
            obs_t = torch.as_tensor(obs[None, :], dtype=torch.float32, device=args.device)
            with torch.no_grad():
                action = policy.deterministic_action(obs_t).cpu().numpy()[0]
            action_abs = np.abs(action)
            action_abs_sum += float(np.sum(action_abs))
            action_abs_count += int(action_abs.size)
            action_abs_max = max(action_abs_max, float(np.max(action_abs)))

            obs, reward, done, info = env.step(action)
            reward_sum += float(reward)
            steps += 1
            max_x = max(max_x, float(env.data.qpos[0]))
            max_terrain_height = max(max_terrain_height, float(env._terrain_height_below_base()))
            for key, value in info.get("reward_terms", {}).items():
                value_f = float(value)
                if np.isfinite(value_f):
                    reward_term_sums[key] = reward_term_sums.get(key, 0.0) + value_f

        episodes.append(
            {
                "episode": episode_idx,
                "return": reward_sum,
                "steps": steps,
                "seconds": steps * env.cfg.control_dt,
                "terminated": bool(info.get("terminated", False)),
                "truncated": bool(info.get("truncated", False)),
                "final_x": float(env.data.qpos[0]),
                "max_x": max_x,
                "forward_distance": float(env.data.qpos[0] - start_x),
                "max_terrain_height": max_terrain_height,
                "action_abs_mean": action_abs_sum / max(1, action_abs_count),
                "action_abs_max": action_abs_max,
                "reward_terms": {
                    key: value / max(1, steps)
                    for key, value in sorted(reward_term_sums.items())
                },
            }
        )

    summary = _summarize(args, episodes)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.json_output is not None:
        output_path = Path(args.json_output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[INFO] wrote {output_path}")


def _summarize(args: argparse.Namespace, episodes: list[dict[str, object]]) -> dict[str, object]:
    returns = np.array([episode["return"] for episode in episodes], dtype=np.float64)
    steps = np.array([episode["steps"] for episode in episodes], dtype=np.float64)
    seconds = np.array([episode["seconds"] for episode in episodes], dtype=np.float64)
    distances = np.array([episode["forward_distance"] for episode in episodes], dtype=np.float64)
    final_x = np.array([episode["final_x"] for episode in episodes], dtype=np.float64)
    max_x = np.array([episode["max_x"] for episode in episodes], dtype=np.float64)
    max_terrain_height = np.array([episode["max_terrain_height"] for episode in episodes], dtype=np.float64)
    action_abs = np.array([episode["action_abs_mean"] for episode in episodes], dtype=np.float64)
    action_max = np.array([episode["action_abs_max"] for episode in episodes], dtype=np.float64)
    terminated = np.array([episode["terminated"] for episode in episodes], dtype=np.float64)

    reward_terms: dict[str, float] = {}
    term_keys = sorted({key for episode in episodes for key in episode["reward_terms"]})
    for key in term_keys:
        reward_terms[key] = float(
            np.mean([episode["reward_terms"].get(key, 0.0) for episode in episodes])
        )

    return {
        "checkpoint": str(Path(args.checkpoint).expanduser().resolve()),
        "terrain": args.terrain,
        "episodes": len(episodes),
        "episode_seconds": args.episode_seconds,
        "return_mean": float(np.mean(returns)),
        "return_std": float(np.std(returns)),
        "return_min": float(np.min(returns)),
        "return_max": float(np.max(returns)),
        "terminated_rate": float(np.mean(terminated)),
        "mean_steps": float(np.mean(steps)),
        "mean_seconds": float(np.mean(seconds)),
        "final_x_mean": float(np.mean(final_x)),
        "max_x_mean": float(np.mean(max_x)),
        "forward_distance_mean": float(np.mean(distances)),
        "max_terrain_height_mean": float(np.mean(max_terrain_height)),
        "action_abs_mean": float(np.mean(action_abs)),
        "action_abs_max": float(np.max(action_max)),
        "reward_terms": reward_terms,
        "episodes_detail": episodes,
    }


if __name__ == "__main__":
    main()
