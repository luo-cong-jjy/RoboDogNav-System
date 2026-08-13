#!/usr/bin/env python3
"""Run staged curriculum training for the M20 MuJoCo policy."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m20_mujoco_rl.config import M20EnvConfig, PPOConfig, RandomizationConfig, TerrainConfig  # noqa: E402
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train M20 with a staged MuJoCo curriculum.")
    parser.add_argument("--num-envs", type=int, default=8)
    parser.add_argument("--steps-per-env", type=int, default=24)
    parser.add_argument("--eval-interval", type=int, default=100, help="Iterations between fixed deterministic evals; 0 disables.")
    parser.add_argument("--eval-episodes", type=int, default=3, help="Episodes per fixed deterministic eval.")
    parser.add_argument("--action-limit", type=float, default=None, help="Override normalized policy action limit.")
    parser.add_argument("--entropy-coef", type=float, default=None, help="Override PPO entropy coefficient.")
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
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", default="cpu", help="cpu or cuda")
    parser.add_argument("--log-dir", default="logs/m20_mujoco_curriculum")
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--command-mode", default="forward", choices=["forward", "random"])
    parser.add_argument("--episode-seconds", type=float, default=8.0)
    parser.add_argument("--no-randomization", action="store_true", help="Disable domain/reset randomization.")
    parser.add_argument("--start-checkpoint", default=None, help="Optional policy checkpoint for the first stage.")
    parser.add_argument("--flat-iterations", type=int, default=1000)
    parser.add_argument("--stair-easy-iterations", type=int, default=3000)
    parser.add_argument("--random-boxes-iterations", type=int, default=5000)
    parser.add_argument("--stair-official-iterations", type=int, default=8000)
    parser.add_argument("--stair-easy-height", type=float, default=0.05)
    parser.add_argument("--stair-easy-count", type=int, default=5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        from m20_mujoco_rl.ppo import train_ppo
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing training dependency. Run: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    run_name = args.run_name or f"curriculum_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir = Path(args.log_dir).expanduser().resolve() / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    stages = [
        (
            "01_flat",
            TerrainConfig(name="flat", seed=args.seed, stair_height=0.0),
            args.flat_iterations,
            (0.10, 0.45),
        ),
        (
            "02_stair_easy",
            TerrainConfig(
                name="stair_easy",
                seed=args.seed + 1000,
                stair_height=args.stair_easy_height,
                stair_count=args.stair_easy_count,
            ),
            args.stair_easy_iterations,
            (0.08, 0.35),
        ),
        (
            "03_random_boxes",
            TerrainConfig(name="random_boxes", seed=args.seed + 2000),
            args.random_boxes_iterations,
            (0.08, 0.30),
        ),
        (
            "04_stair_official",
            TerrainConfig(name="stair_official", seed=args.seed + 3000),
            args.stair_official_iterations,
            (0.08, 0.25),
        ),
    ]
    stages = [
        (name, terrain, iterations, command_range)
        for name, terrain, iterations, command_range in stages
        if iterations > 0
    ]
    if not stages:
        raise SystemExit("At least one curriculum stage must have iterations > 0.")

    print(f"[INFO] curriculum_log_dir={run_dir}")
    print(f"[INFO] stages={[(name, iterations) for name, _, iterations, _ in stages]}")
    print(f"[INFO] randomization={not args.no_randomization}")
    if args.eval_interval > 0:
        print(f"[INFO] fixed_eval_interval={args.eval_interval}, eval_episodes={args.eval_episodes}")
    optional_ppo_args = {
        "action_limit": args.action_limit,
        "entropy_coef": args.entropy_coef,
        "log_std_max": args.log_std_max,
        "mean_action_l2_coef": args.mean_action_l2_coef,
        "mean_action_saturation_coef": args.mean_action_saturation_coef,
        "mean_action_saturation_threshold": args.mean_action_saturation_threshold,
        "eval_action_weight": args.eval_action_weight,
        "eval_action_saturation_weight": args.eval_action_saturation_weight,
        "eval_undesired_contact_weight": args.eval_undesired_contact_weight,
        "eval_contact_force_weight": args.eval_contact_force_weight,
    }
    ppo_overrides = {key: value for key, value in optional_ppo_args.items() if value is not None}
    preview_ppo_cfg = PPOConfig(**ppo_overrides)
    print(
        "[INFO] action_limit="
        f"{preview_ppo_cfg.action_limit}, mean_action_l2_coef={preview_ppo_cfg.mean_action_l2_coef}, "
        f"mean_action_saturation_coef={preview_ppo_cfg.mean_action_saturation_coef}, "
        f"mean_action_saturation_threshold={preview_ppo_cfg.mean_action_saturation_threshold}"
    )

    previous_checkpoint = Path(args.start_checkpoint).expanduser().resolve() if args.start_checkpoint else None
    summary = {
        "run_dir": str(run_dir),
        "randomization": not args.no_randomization,
        "start_checkpoint": str(previous_checkpoint) if previous_checkpoint is not None else None,
        "stages": [],
    }
    summary_path = run_dir / "curriculum.json"

    for stage_index, (stage_name, terrain_cfg, iterations, command_range) in enumerate(stages):
        stage_seed = args.seed + stage_index * 1000
        stage_dir = run_dir / stage_name
        env_cfg = M20EnvConfig(
            terrain=terrain_cfg,
            randomization=RandomizationConfig(enabled=not args.no_randomization),
            seed=stage_seed,
            command_mode=args.command_mode,
            forward_command_range=command_range,
            episode_seconds=args.episode_seconds,
        )
        ppo_kwargs = {
            "seed": stage_seed,
            "num_envs": args.num_envs,
            "iterations": iterations,
            "steps_per_env": args.steps_per_env,
            "eval_interval": args.eval_interval,
            "eval_episodes": args.eval_episodes,
        }
        ppo_kwargs.update(ppo_overrides)
        ppo_cfg = PPOConfig(**ppo_kwargs)

        def make_env(env_id: int) -> M20MujocoEnv:
            terrain = replace(env_cfg.terrain, seed=stage_seed + env_id)
            cfg = replace(env_cfg, seed=stage_seed + env_id, terrain=terrain)
            return M20MujocoEnv(cfg)

        eval_env_cfg = replace(env_cfg, randomization=RandomizationConfig(enabled=False))

        def make_eval_env(eval_id: int) -> M20MujocoEnv:
            eval_seed = stage_seed + 100_000 + eval_id
            terrain = replace(eval_env_cfg.terrain, seed=eval_seed)
            cfg = replace(eval_env_cfg, seed=eval_seed, terrain=terrain)
            return M20MujocoEnv(cfg)

        print(
            f"[INFO] stage={stage_name} terrain={terrain_cfg.name} iterations={iterations} "
            f"resume={previous_checkpoint}"
        )
        checkpoint = train_ppo(
            make_env,
            env_cfg,
            ppo_cfg,
            stage_dir,
            device=args.device,
            resume_checkpoint=previous_checkpoint,
            eval_make_env=make_eval_env if args.eval_interval > 0 else None,
        )
        eval_checkpoint = stage_dir / "policy_eval_best.pt"
        next_checkpoint = eval_checkpoint if eval_checkpoint.exists() else checkpoint
        stage_record = {
            "name": stage_name,
            "terrain": asdict(terrain_cfg),
            "iterations": iterations,
            "forward_command_range": command_range,
            "seed": stage_seed,
            "log_dir": str(stage_dir),
            "resume_checkpoint": str(previous_checkpoint) if previous_checkpoint is not None else None,
            "checkpoint": str(checkpoint),
            "eval_checkpoint": str(eval_checkpoint) if eval_checkpoint.exists() else None,
            "next_checkpoint": str(next_checkpoint),
        }
        summary["stages"].append(stage_record)
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        previous_checkpoint = next_checkpoint

    print(f"[INFO] curriculum summary: {summary_path}")
    print(f"[INFO] final checkpoint: {previous_checkpoint}")


if __name__ == "__main__":
    main()
