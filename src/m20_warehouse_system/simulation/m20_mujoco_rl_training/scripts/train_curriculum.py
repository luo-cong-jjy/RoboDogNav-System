#!/usr/bin/env python3
# ======================================================================
# train_curriculum.py —— M20 MuJoCo 分阶段课程训练脚本（中文注释版）
# 作用：按"平地 -> 简易台阶 -> 随机箱体 -> 官方台阶"四阶段课程依次训练，
#       每个阶段继承上一阶段的检查点继续训练（warm-start），
#       并在每阶段用固定确定性评估筛选出 eval_best 检查点传给下一阶段。
# 用法：python3 scripts/train_curriculum.py [--num-envs 8] [--seed 1] ...
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Run staged curriculum training for the M20 MuJoCo policy."""

from __future__ import annotations

# 命令行参数解析
import argparse
# JSON：课程摘要写入
import json
# 系统接口：把包根目录加入模块搜索路径
import sys
# 数据类：复制配置
from dataclasses import asdict, replace
# 时间：生成运行名
from datetime import datetime
# 路径库
from pathlib import Path


# 包根目录（scripts 的上级）
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# 把包根目录插入 sys.path，便于直接运行脚本
sys.path.insert(0, str(PACKAGE_ROOT))

# 配置类与环境类（noqa: E402 忽略导入顺序检查）
from m20_mujoco_rl.config import M20EnvConfig, PPOConfig, RandomizationConfig, TerrainConfig  # noqa: E402
from m20_mujoco_rl.env import M20MujocoEnv  # noqa: E402


# 解析命令行参数
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train M20 with a staged MuJoCo curriculum.")
    # 并行环境数
    parser.add_argument("--num-envs", type=int, default=8)
    # 每环境每轮步数
    parser.add_argument("--steps-per-env", type=int, default=24)
    # 固定评估间隔（0 禁用）
    parser.add_argument("--eval-interval", type=int, default=100, help="Iterations between fixed deterministic evals; 0 disables.")
    # 每次评估回合数
    parser.add_argument("--eval-episodes", type=int, default=3, help="Episodes per fixed deterministic eval.")
    # PPO 超参数覆盖
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
    # 评估分数权重覆盖
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
    # 随机种子与设备
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", default="cpu", help="cpu or cuda")
    # 日志目录与运行名
    parser.add_argument("--log-dir", default="logs/m20_mujoco_curriculum")
    parser.add_argument("--run-name", default=None)
    # 指令模式
    parser.add_argument("--command-mode", default="forward", choices=["forward", "random"])
    # 回合时长
    parser.add_argument("--episode-seconds", type=float, default=8.0)
    # 关闭域随机化
    parser.add_argument("--no-randomization", action="store_true", help="Disable domain/reset randomization.")
    # 第一阶段可选的起始检查点
    parser.add_argument("--start-checkpoint", default=None, help="Optional policy checkpoint for the first stage.")
    # 各阶段迭代数
    parser.add_argument("--flat-iterations", type=int, default=1000)
    parser.add_argument("--stair-easy-iterations", type=int, default=3000)
    parser.add_argument("--random-boxes-iterations", type=int, default=5000)
    parser.add_argument("--stair-official-iterations", type=int, default=8000)
    # 简易台阶参数
    parser.add_argument("--stair-easy-height", type=float, default=0.05)
    parser.add_argument("--stair-easy-count", type=int, default=5)
    return parser.parse_args()


# 主函数
def main() -> None:
    args = parse_args()
    try:
        # 延迟导入训练函数
        from m20_mujoco_rl.ppo import train_ppo
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing training dependency. Run: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    # 运行目录：logs/<log_dir>/curriculum_<时间戳>
    run_name = args.run_name or f"curriculum_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir = Path(args.log_dir).expanduser().resolve() / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    # 定义课程阶段：每阶段 (名称, 地形配置, 迭代数, 前向指令范围)
    stages = [
        (
            "01_flat",   # 阶段 1：平地
            TerrainConfig(name="flat", seed=args.seed, stair_height=0.0),
            args.flat_iterations,
            (0.10, 0.45),   # 指令范围较宽
        ),
        (
            "02_stair_easy",   # 阶段 2：简易台阶
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
            "03_random_boxes",   # 阶段 3：随机箱体
            TerrainConfig(name="random_boxes", seed=args.seed + 2000),
            args.random_boxes_iterations,
            (0.08, 0.30),
        ),
        (
            "04_stair_official",   # 阶段 4：官方台阶
            TerrainConfig(name="stair_official", seed=args.seed + 3000),
            args.stair_official_iterations,
            (0.08, 0.25),
        ),
    ]
    # 过滤掉迭代数为 0 的阶段
    stages = [
        (name, terrain, iterations, command_range)
        for name, terrain, iterations, command_range in stages
        if iterations > 0
    ]
    if not stages:
        raise SystemExit("At least one curriculum stage must have iterations > 0.")

    # 打印训练配置概要
    print(f"[INFO] curriculum_log_dir={run_dir}")
    print(f"[INFO] stages={[(name, iterations) for name, _, iterations, _ in stages]}")
    print(f"[INFO] randomization={not args.no_randomization}")
    if args.eval_interval > 0:
        print(f"[INFO] fixed_eval_interval={args.eval_interval}, eval_episodes={args.eval_episodes}")
    # 可选 PPO 参数覆盖（仅保留非 None 的）
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
    # 预览覆盖后的 PPO 配置（打印关键项）
    preview_ppo_cfg = PPOConfig(**ppo_overrides)
    print(
        "[INFO] action_limit="
        f"{preview_ppo_cfg.action_limit}, mean_action_l2_coef={preview_ppo_cfg.mean_action_l2_coef}, "
        f"mean_action_saturation_coef={preview_ppo_cfg.mean_action_saturation_coef}, "
        f"mean_action_saturation_threshold={preview_ppo_cfg.mean_action_saturation_threshold}"
    )

    # 上一阶段检查点（第一阶段可用 --start-checkpoint）
    previous_checkpoint = Path(args.start_checkpoint).expanduser().resolve() if args.start_checkpoint else None
    # 课程摘要
    summary = {
        "run_dir": str(run_dir),
        "randomization": not args.no_randomization,
        "start_checkpoint": str(previous_checkpoint) if previous_checkpoint is not None else None,
        "stages": [],
    }
    summary_path = run_dir / "curriculum.json"

    # 逐阶段训练
    for stage_index, (stage_name, terrain_cfg, iterations, command_range) in enumerate(stages):
        # 每阶段用不同种子
        stage_seed = args.seed + stage_index * 1000
        stage_dir = run_dir / stage_name
        # 本阶段环境配置
        env_cfg = M20EnvConfig(
            terrain=terrain_cfg,
            randomization=RandomizationConfig(enabled=not args.no_randomization),
            seed=stage_seed,
            command_mode=args.command_mode,
            forward_command_range=command_range,
            episode_seconds=args.episode_seconds,
        )
        # 本阶段 PPO 配置
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

        # 训练环境工厂（每个环境不同种子，地形种子也各不相同）
        def make_env(env_id: int) -> M20MujocoEnv:
            terrain = replace(env_cfg.terrain, seed=stage_seed + env_id)
            cfg = replace(env_cfg, seed=stage_seed + env_id, terrain=terrain)
            return M20MujocoEnv(cfg)

        # 评估环境配置（关闭随机化）
        eval_env_cfg = replace(env_cfg, randomization=RandomizationConfig(enabled=False))

        # 评估环境工厂（固定种子）
        def make_eval_env(eval_id: int) -> M20MujocoEnv:
            eval_seed = stage_seed + 100_000 + eval_id
            terrain = replace(eval_env_cfg.terrain, seed=eval_seed)
            cfg = replace(eval_env_cfg, seed=eval_seed, terrain=terrain)
            return M20MujocoEnv(cfg)

        # 打印本阶段信息并启动训练
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
            resume_checkpoint=previous_checkpoint,   # 继承上一阶段权重
            eval_make_env=make_eval_env if args.eval_interval > 0 else None,
        )
        # 下一阶段优先使用评估最佳检查点（若无则用最新）
        eval_checkpoint = stage_dir / "policy_eval_best.pt"
        next_checkpoint = eval_checkpoint if eval_checkpoint.exists() else checkpoint
        # 记录本阶段信息
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
        # 每阶段结束后更新摘要文件
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        # 传给下一阶段
        previous_checkpoint = next_checkpoint

    # 打印最终结果
    print(f"[INFO] curriculum summary: {summary_path}")
    print(f"[INFO] final checkpoint: {previous_checkpoint}")


# 脚本入口
if __name__ == "__main__":
    main()
