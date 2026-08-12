"""A compact PPO trainer for the M20 MuJoCo environment."""

from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path
from typing import Callable

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

from .config import M20EnvConfig, PPOConfig
from .constants import ACTION_DIM, OBS_DIM
from .viewer_process import ViewerProcess, start_viewer_process


CONSOLE_REWARD_TERMS = (
    "track_lin",
    "track_yaw",
    "progress",
    "terrain_height_progress",
    "stair_height",
    "stair_forward_progress",
    "base_height",
    "torque",
    "action_rate",
    "leg_action_l2",
    "wheel_action_l2",
    "action_saturation",
    "joint_acc",
    "wheel_acc",
    "joint_power",
    "contact_force",
    "undesired_contacts",
    "wheel_air_time",
    "wheel_clearance",
    "wheel_stumble",
    "upward",
)


class ActorCritic(nn.Module):
    def __init__(
        self,
        obs_dim: int | None = None,
        actor_obs_dim: int | None = None,
        critic_obs_dim: int | None = None,
        action_dim: int = ACTION_DIM,
        hidden_dims: tuple[int, ...] = (512, 256, 128),
        init_noise_std: float = 0.25,
        action_limit: float = 1.0,
        log_std_min: float = -4.0,
        log_std_max: float = 0.5,
    ):
        super().__init__()
        if actor_obs_dim is None:
            actor_obs_dim = obs_dim if obs_dim is not None else OBS_DIM
        if critic_obs_dim is None:
            critic_obs_dim = actor_obs_dim
        self.actor_obs_dim = int(actor_obs_dim)
        self.critic_obs_dim = int(critic_obs_dim)
        self.action_dim = int(action_dim)
        self.actor = self._make_mlp(self.actor_obs_dim, action_dim, hidden_dims)
        self.critic = self._make_mlp(self.critic_obs_dim, 1, hidden_dims)
        self.log_std = nn.Parameter(torch.ones(action_dim) * np.log(init_noise_std))
        self.action_limit = float(action_limit)
        self.log_std_min = float(log_std_min)
        self.log_std_max = float(log_std_max)

    @staticmethod
    def _make_mlp(input_dim: int, output_dim: int, hidden_dims: tuple[int, ...]) -> nn.Sequential:
        layers: list[nn.Module] = []
        last_dim = input_dim
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(last_dim, hidden_dim))
            layers.append(nn.ELU())
            last_dim = hidden_dim
        layers.append(nn.Linear(last_dim, output_dim))
        return nn.Sequential(*layers)

    def distribution(self, obs: torch.Tensor) -> Normal:
        mean = torch.tanh(self.actor(obs)) * self.action_limit
        log_std = torch.clamp(self.log_std, self.log_std_min, self.log_std_max)
        std = torch.exp(log_std).expand_as(mean)
        return Normal(mean, std)

    def value(self, critic_obs: torch.Tensor) -> torch.Tensor:
        return self.critic(critic_obs).squeeze(-1)

    def act(
        self,
        obs: torch.Tensor,
        critic_obs: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist = self.distribution(obs)
        action = torch.clamp(dist.sample(), -self.action_limit, self.action_limit)
        log_prob = dist.log_prob(action).sum(dim=-1)
        value = self.value(obs if critic_obs is None else critic_obs)
        return action, log_prob, value

    def evaluate(
        self,
        obs: torch.Tensor,
        action: torch.Tensor,
        critic_obs: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist = self.distribution(obs)
        action = torch.clamp(action, -self.action_limit, self.action_limit)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self.value(obs if critic_obs is None else critic_obs)
        return log_prob, entropy, value

    def deterministic_action(self, obs: torch.Tensor) -> torch.Tensor:
        return torch.tanh(self.actor(obs)) * self.action_limit

    def clamp_log_std_(self) -> None:
        with torch.no_grad():
            self.log_std.clamp_(self.log_std_min, self.log_std_max)


def train_ppo(
    make_env: Callable[[int], object],
    env_cfg: M20EnvConfig,
    ppo_cfg: PPOConfig,
    log_dir: Path,
    device: str = "cpu",
    render_env_id: int | None = None,
    render_real_time: bool = False,
    resume_checkpoint: str | Path | None = None,
    eval_make_env: Callable[[int], object] | None = None,
) -> Path:
    log_dir.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(ppo_cfg.seed)
    np.random.seed(ppo_cfg.seed)

    envs = [make_env(i) for i in range(ppo_cfg.num_envs)]
    obs_np = np.stack([env.reset(seed=ppo_cfg.seed + i) for i, env in enumerate(envs)])
    critic_obs_np = np.stack([_env_critic_obs(env) for env in envs])
    actor_obs_dim = int(obs_np.shape[1])
    critic_obs_dim = int(critic_obs_np.shape[1])
    if actor_obs_dim != OBS_DIM:
        print(f"[INFO] actor observation dim={actor_obs_dim}; deployment 57-dim OBS_DIM={OBS_DIM}")
    viewer: ViewerProcess | None = None
    if render_env_id is not None:
        if render_env_id < 0 or render_env_id >= len(envs):
            raise ValueError(f"render_env_id must be in [0, {len(envs) - 1}], got {render_env_id}")
        viewer = start_viewer_process(
            envs[render_env_id].xml_path,
            control_dt=envs[render_env_id].cfg.control_dt,
            real_time=render_real_time,
        )
        print(
            "[INFO] MuJoCo viewer started in a separate process. "
            "If the window closes, training will continue."
        )

    model = ActorCritic(
        actor_obs_dim=actor_obs_dim,
        critic_obs_dim=critic_obs_dim,
        action_dim=ACTION_DIM,
        hidden_dims=ppo_cfg.hidden_dims,
        init_noise_std=ppo_cfg.init_noise_std,
        action_limit=ppo_cfg.action_limit,
        log_std_min=ppo_cfg.log_std_min,
        log_std_max=ppo_cfg.log_std_max,
    ).to(device)
    optimizer = optim.Adam(model.parameters(), lr=ppo_cfg.learning_rate)
    resume_steps = 0
    if resume_checkpoint is not None:
        resume_steps = _load_checkpoint_into_model(Path(resume_checkpoint), model, device)
        print(f"[INFO] resumed policy from {resume_checkpoint} at steps={resume_steps}")

    with (log_dir / "config.json").open("w", encoding="utf-8") as f:
        json.dump(
            {
                "env": asdict(env_cfg),
                "ppo": asdict(ppo_cfg),
                "resume_checkpoint": str(Path(resume_checkpoint).expanduser().resolve())
                if resume_checkpoint is not None else None,
            },
            f,
            ensure_ascii=False,
            indent=2,
        )

    total_steps = resume_steps
    best_mean_return = -1.0e9
    best_eval_score = -1.0e9
    best_eval_candidate_score = -1.0e9
    latest_path = log_dir / "policy_latest.pt"
    eval_best_path = log_dir / "policy_eval_best.pt"
    eval_candidate_path = log_dir / "policy_eval_candidate.pt"
    metrics_path = log_dir / "metrics.jsonl"
    eval_metrics_path = log_dir / "eval_metrics.jsonl"

    try:
        for iteration in range(1, ppo_cfg.iterations + 1):
            rollout = _collect_rollout(
                model,
                envs,
                obs_np,
                critic_obs_np,
                ppo_cfg,
                device,
                viewer=viewer,
                render_env_id=render_env_id,
                render_real_time=render_real_time,
            )
            obs_np = rollout["last_obs"]
            critic_obs_np = rollout["last_critic_obs"]
            total_steps += ppo_cfg.num_envs * ppo_cfg.steps_per_env

            advantages, returns = _compute_gae(rollout, ppo_cfg, model, device)
            stats = _ppo_update(model, optimizer, rollout, advantages, returns, ppo_cfg, device)
            log_std_mean = float(model.log_std.detach().mean().cpu())

            done_returns = rollout["done_returns"]
            mean_return = float(np.mean(done_returns)) if done_returns else float("nan")
            if done_returns and mean_return > best_mean_return:
                best_mean_return = mean_return
                _save_checkpoint(log_dir / "policy_best.pt", model, env_cfg, ppo_cfg, total_steps)

            if iteration % ppo_cfg.save_interval == 0 or iteration == ppo_cfg.iterations:
                _save_checkpoint(latest_path, model, env_cfg, ppo_cfg, total_steps)

            eval_metrics = None
            should_eval = (
                eval_make_env is not None
                and ppo_cfg.eval_interval > 0
                and (iteration % ppo_cfg.eval_interval == 0 or iteration == ppo_cfg.iterations)
            )
            if should_eval:
                eval_metrics = _evaluate_policy(model, eval_make_env, ppo_cfg, device)
                eval_record = {
                    "iteration": iteration,
                    "total_steps": total_steps,
                    **eval_metrics,
                }
                score = float(eval_metrics["score"])
                best_candidate_valid = bool(eval_metrics.get("best_candidate_valid", True))
                if np.isfinite(score) and best_candidate_valid and score > best_eval_candidate_score:
                    best_eval_candidate_score = score
                    eval_record["is_candidate_best"] = True
                    _save_checkpoint(eval_candidate_path, model, env_cfg, ppo_cfg, total_steps)
                else:
                    eval_record["is_candidate_best"] = False
                if np.isfinite(score) and best_candidate_valid and score > best_eval_score:
                    best_eval_score = score
                    eval_record["is_best"] = True
                    _save_checkpoint(eval_best_path, model, env_cfg, ppo_cfg, total_steps)
                else:
                    eval_record["is_best"] = False
                with eval_metrics_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(eval_record, ensure_ascii=False) + "\n")

            metrics = {
                "iteration": iteration,
                "total_steps": total_steps,
                "mean_reward": float(rollout["mean_reward"]),
                "done_return": mean_return if np.isfinite(mean_return) else None,
                "policy_loss": stats["policy_loss"],
                "value_loss": stats["value_loss"],
                "entropy": stats["entropy"],
                "mean_action_l2_loss": stats["mean_action_l2_loss"],
                "mean_action_saturation_loss": stats["mean_action_saturation_loss"],
                "mean_action_abs": stats["mean_action_abs"],
                "mean_action_max": stats["mean_action_max"],
                "action_abs_mean": float(rollout["action_abs_mean"]),
                "action_abs_max": float(rollout["action_abs_max"]),
                "rollout_mean_action_abs": float(rollout["mean_action_abs_mean"]),
                "rollout_mean_action_max": float(rollout["mean_action_abs_max"]),
                "log_std_mean": log_std_mean,
                "reward_terms": rollout["reward_terms"],
            }
            if eval_metrics is not None:
                metrics["eval"] = eval_metrics
            with metrics_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(metrics, ensure_ascii=False) + "\n")

            elapsed = time.strftime("%H:%M:%S")
            print(
                f"[{elapsed}] iter={iteration:05d}/{ppo_cfg.iterations} "
                f"steps={total_steps:09d} "
                f"reward={rollout['mean_reward']:.3f} "
                f"done_return={mean_return:.3f} "
                f"policy_loss={stats['policy_loss']:.4f} "
                f"value_loss={stats['value_loss']:.4f} "
                f"entropy={stats['entropy']:.3f} "
                f"action_abs={rollout['action_abs_mean']:.3f} "
                f"action_max={rollout['action_abs_max']:.3f} "
                f"mean_action={rollout['mean_action_abs_mean']:.3f} "
                f"mean_max={rollout['mean_action_abs_max']:.3f} "
                f"log_std={log_std_mean:.3f}"
            )
            print(
                "    actor_reg "
                f"l2={stats['mean_action_l2_loss']:.4g} "
                f"sat={stats['mean_action_saturation_loss']:.4g} "
                f"mean_abs={stats['mean_action_abs']:.3f} "
                f"mean_max={stats['mean_action_max']:.3f}"
            )
            print(f"    terms {_format_reward_terms(rollout['reward_terms'])}")
            if eval_metrics is not None:
                print(
                    "    eval "
                    f"score={float(eval_metrics['score']):.3f} "
                    f"return={float(eval_metrics['return_mean']):.3f} "
                    f"term_rate={float(eval_metrics['terminated_rate']):.2f} "
                    f"seconds={float(eval_metrics['mean_seconds']):.2f} "
                    f"dist={float(eval_metrics['forward_distance_mean']):.3f} "
                    f"max_x={float(eval_metrics['max_x_mean']):.3f} "
                    f"max_h={float(eval_metrics['max_terrain_height_mean']):.3f} "
                    f"action_abs={float(eval_metrics['action_abs_mean']):.3f} "
                    f"action_max={float(eval_metrics['action_abs_max']):.3f} "
                    f"action_sat={float(eval_metrics['action_saturation_mean']):.4f} "
                    f"undesired={float(eval_metrics['undesired_contact_penalty']):.2f} "
                    f"contact={float(eval_metrics['contact_force_penalty']):.2f} "
                    f"valid={bool(eval_metrics.get('best_candidate_valid', True))} "
                    f"best={best_eval_score:.3f}"
                )
    finally:
        if viewer is not None:
            viewer.close()

    _save_checkpoint(latest_path, model, env_cfg, ppo_cfg, total_steps)
    return latest_path


def _evaluate_policy(
    model: ActorCritic,
    make_env: Callable[[int], object],
    cfg: PPOConfig,
    device: str,
) -> dict[str, object]:
    was_training = model.training
    model.eval()
    episodes: list[dict[str, object]] = []

    try:
        for episode_idx in range(max(1, int(cfg.eval_episodes))):
            env = make_env(episode_idx)
            obs = env.reset(seed=cfg.seed + 100_000 + episode_idx)
            start_x = float(env.data.qpos[0])
            reward_sum = 0.0
            action_abs_sum = 0.0
            action_abs_count = 0
            action_abs_max = 0.0
            action_saturation_sum = 0.0
            action_saturation_count = 0
            reward_term_sums: dict[str, float] = {}
            max_x = start_x
            max_terrain_height = 0.0
            target_terrain_height = float(getattr(env.cfg.terrain, "stair_height", 0.0))
            steps = 0
            info = {}
            done = False

            while not done:
                obs_tensor = torch.as_tensor(obs[None, :], dtype=torch.float32, device=device)
                with torch.no_grad():
                    action = model.deterministic_action(obs_tensor).cpu().numpy()[0]

                action_abs = np.abs(action)
                action_abs_sum += float(np.sum(action_abs))
                action_abs_count += int(action_abs.size)
                action_abs_max = max(action_abs_max, float(np.max(action_abs)))
                action_saturation = np.maximum(action_abs - cfg.eval_action_saturation_threshold, 0.0) ** 2
                action_saturation_sum += float(np.sum(action_saturation))
                action_saturation_count += int(action_saturation.size)

                obs, reward, done, info = env.step(action)
                reward_sum += float(reward)
                steps += 1
                max_x = max(max_x, float(env.data.qpos[0]))
                max_terrain_height = max(max_terrain_height, float(env._terrain_height_below_base()))
                reward_terms = info.get("reward_terms", {})
                if isinstance(reward_terms, dict):
                    for key, value in reward_terms.items():
                        value_f = float(value)
                        if np.isfinite(value_f):
                            reward_term_sums[key] = reward_term_sums.get(key, 0.0) + value_f

            episodes.append(
                {
                    "return": reward_sum,
                    "steps": steps,
                    "seconds": steps * env.cfg.control_dt,
                    "terminated": bool(info.get("terminated", False)),
                    "truncated": bool(info.get("truncated", False)),
                    "final_x": float(env.data.qpos[0]),
                    "max_x": max_x,
                    "forward_distance": float(env.data.qpos[0] - start_x),
                    "max_terrain_height": max_terrain_height,
                    "target_terrain_height": target_terrain_height,
                    "action_abs_mean": action_abs_sum / max(1, action_abs_count),
                    "action_abs_max": action_abs_max,
                    "action_saturation_mean": action_saturation_sum / max(1, action_saturation_count),
                    "reward_terms": {
                        key: value / max(1, steps)
                        for key, value in sorted(reward_term_sums.items())
                    },
                }
            )
    finally:
        if was_training:
            model.train()

    returns = np.array([episode["return"] for episode in episodes], dtype=np.float64)
    steps = np.array([episode["steps"] for episode in episodes], dtype=np.float64)
    seconds = np.array([episode["seconds"] for episode in episodes], dtype=np.float64)
    distances = np.array([episode["forward_distance"] for episode in episodes], dtype=np.float64)
    final_x = np.array([episode["final_x"] for episode in episodes], dtype=np.float64)
    max_x = np.array([episode["max_x"] for episode in episodes], dtype=np.float64)
    max_terrain_height = np.array([episode["max_terrain_height"] for episode in episodes], dtype=np.float64)
    target_terrain_height = np.array([episode["target_terrain_height"] for episode in episodes], dtype=np.float64)
    action_abs = np.array([episode["action_abs_mean"] for episode in episodes], dtype=np.float64)
    action_max = np.array([episode["action_abs_max"] for episode in episodes], dtype=np.float64)
    action_saturation = np.array([episode["action_saturation_mean"] for episode in episodes], dtype=np.float64)
    terminated = np.array([episode["terminated"] for episode in episodes], dtype=np.float64)

    reward_terms: dict[str, float] = {}
    term_keys = sorted({key for episode in episodes for key in episode["reward_terms"]})
    for key in term_keys:
        reward_terms[key] = float(
            np.mean([episode["reward_terms"].get(key, 0.0) for episode in episodes])
        )

    return_mean = float(np.mean(returns))
    return_std = float(np.std(returns))
    return_min = float(np.min(returns))
    forward_distance_min = float(np.min(distances))
    forward_distance_mean = float(np.mean(distances))
    max_terrain_height_min = float(np.min(max_terrain_height))
    target_terrain_height_max = float(np.max(target_terrain_height))
    action_abs_mean = float(np.mean(action_abs))
    action_saturation_mean = float(np.mean(action_saturation))
    terminated_rate = float(np.mean(terminated))
    undesired_contact_penalty = max(-float(reward_terms.get("undesired_contacts", 0.0)), 0.0)
    contact_force_penalty = max(-float(reward_terms.get("contact_force", 0.0)), 0.0)
    score = (
        return_mean
        + cfg.eval_return_min_weight * return_min
        - cfg.eval_return_std_weight * return_std
        + 100.0 * forward_distance_mean
        - cfg.eval_action_weight * action_abs_mean
        - cfg.eval_action_saturation_weight * action_saturation_mean
        - cfg.eval_undesired_contact_weight * undesired_contact_penalty
        - cfg.eval_contact_force_weight * contact_force_penalty
        - cfg.eval_termination_weight * terminated_rate
    )
    best_candidate_reasons: list[str] = []
    if cfg.eval_min_terrain_height_fraction > 0.0 and target_terrain_height_max > 0.0:
        min_height = target_terrain_height_max * cfg.eval_min_terrain_height_fraction
        if max_terrain_height_min + 1.0e-9 < min_height:
            best_candidate_reasons.append(
                f"max_terrain_height_min {max_terrain_height_min:.4f} < required {min_height:.4f}"
            )
    if cfg.eval_min_forward_distance > 0.0:
        if forward_distance_min + 1.0e-9 < cfg.eval_min_forward_distance:
            best_candidate_reasons.append(
                f"forward_distance_min {forward_distance_min:.4f} < required {cfg.eval_min_forward_distance:.4f}"
            )
    best_candidate_valid = len(best_candidate_reasons) == 0

    return {
        "score": float(score),
        "best_candidate_valid": best_candidate_valid,
        "best_candidate_reasons": best_candidate_reasons,
        "episodes": len(episodes),
        "return_mean": return_mean,
        "return_std": return_std,
        "return_min": return_min,
        "return_max": float(np.max(returns)),
        "terminated_rate": terminated_rate,
        "mean_steps": float(np.mean(steps)),
        "mean_seconds": float(np.mean(seconds)),
        "final_x_mean": float(np.mean(final_x)),
        "max_x_mean": float(np.mean(max_x)),
        "forward_distance_min": forward_distance_min,
        "forward_distance_mean": forward_distance_mean,
        "max_terrain_height_min": max_terrain_height_min,
        "max_terrain_height_mean": float(np.mean(max_terrain_height)),
        "target_terrain_height_max": target_terrain_height_max,
        "action_abs_mean": action_abs_mean,
        "action_abs_max": float(np.max(action_max)),
        "action_saturation_mean": action_saturation_mean,
        "undesired_contact_penalty": undesired_contact_penalty,
        "contact_force_penalty": contact_force_penalty,
        "reward_terms": reward_terms,
    }


def _env_critic_obs(env: object) -> np.ndarray:
    get_critic_obs = getattr(env, "get_critic_obs", None)
    if callable(get_critic_obs):
        return np.asarray(get_critic_obs(), dtype=np.float32)
    get_obs = getattr(env, "_get_obs", None)
    if callable(get_obs):
        return np.asarray(get_obs(), dtype=np.float32)
    raise AttributeError("Environment must provide get_critic_obs() or _get_obs().")


def _collect_rollout(
    model: ActorCritic,
    envs: list[object],
    obs_np: np.ndarray,
    critic_obs_np: np.ndarray,
    cfg: PPOConfig,
    device: str,
    viewer: ViewerProcess | None = None,
    render_env_id: int | None = None,
    render_real_time: bool = False,
) -> dict[str, object]:
    actor_obs_dim = int(obs_np.shape[1])
    critic_obs_dim = int(critic_obs_np.shape[1])
    obs_buf = np.zeros((cfg.steps_per_env, cfg.num_envs, actor_obs_dim), dtype=np.float32)
    critic_obs_buf = np.zeros((cfg.steps_per_env, cfg.num_envs, critic_obs_dim), dtype=np.float32)
    action_buf = np.zeros((cfg.steps_per_env, cfg.num_envs, ACTION_DIM), dtype=np.float32)
    log_prob_buf = np.zeros((cfg.steps_per_env, cfg.num_envs), dtype=np.float32)
    reward_buf = np.zeros((cfg.steps_per_env, cfg.num_envs), dtype=np.float32)
    done_buf = np.zeros((cfg.steps_per_env, cfg.num_envs), dtype=np.float32)
    value_buf = np.zeros((cfg.steps_per_env, cfg.num_envs), dtype=np.float32)
    done_returns: list[float] = []
    reward_term_sums: dict[str, float] = {}
    reward_term_count = 0
    action_abs_sum = 0.0
    action_abs_count = 0
    action_abs_max = 0.0
    mean_action_abs_sum = 0.0
    mean_action_abs_count = 0
    mean_action_abs_max = 0.0

    for step in range(cfg.steps_per_env):
        obs_tensor = torch.as_tensor(obs_np, dtype=torch.float32, device=device)
        critic_obs_tensor = torch.as_tensor(critic_obs_np, dtype=torch.float32, device=device)
        with torch.no_grad():
            action_tensor, log_prob_tensor, value_tensor = model.act(obs_tensor, critic_obs_tensor)
            mean_action_tensor = model.deterministic_action(obs_tensor)

        action_np = action_tensor.cpu().numpy()
        action_abs = np.abs(action_np)
        action_abs_sum += float(np.sum(action_abs))
        action_abs_count += int(action_abs.size)
        action_abs_max = max(action_abs_max, float(np.max(action_abs)))
        mean_action_np = mean_action_tensor.cpu().numpy()
        mean_action_abs = np.abs(mean_action_np)
        mean_action_abs_sum += float(np.sum(mean_action_abs))
        mean_action_abs_count += int(mean_action_abs.size)
        mean_action_abs_max = max(mean_action_abs_max, float(np.max(mean_action_abs)))
        next_obs = np.zeros_like(obs_np)
        next_critic_obs = np.zeros_like(critic_obs_np)
        rewards = np.zeros(cfg.num_envs, dtype=np.float32)
        dones = np.zeros(cfg.num_envs, dtype=np.float32)

        for env_id, env in enumerate(envs):
            next_obs_i, reward, done, info = env.step(action_np[env_id])
            rewards[env_id] = reward
            dones[env_id] = float(done)
            reward_terms = info.get("reward_terms", {})
            if isinstance(reward_terms, dict):
                reward_term_count += 1
                for key, value in reward_terms.items():
                    value_f = float(value)
                    if np.isfinite(value_f):
                        reward_term_sums[key] = reward_term_sums.get(key, 0.0) + value_f
            if done:
                done_returns.append(float(info["episode_return"]))
                next_obs_i = env.reset()
            next_obs[env_id] = next_obs_i
            next_critic_obs[env_id] = _env_critic_obs(env)

            if viewer is not None and render_env_id == env_id:
                viewer.sync(env.data.qpos.copy(), env.data.qvel.copy())

        obs_buf[step] = obs_np
        critic_obs_buf[step] = critic_obs_np
        action_buf[step] = action_np
        log_prob_buf[step] = log_prob_tensor.cpu().numpy()
        value_buf[step] = value_tensor.cpu().numpy()
        reward_buf[step] = rewards
        done_buf[step] = dones
        obs_np = next_obs
        critic_obs_np = next_critic_obs

    return {
        "obs": obs_buf,
        "critic_obs": critic_obs_buf,
        "actions": action_buf,
        "log_probs": log_prob_buf,
        "values": value_buf,
        "rewards": reward_buf,
        "dones": done_buf,
        "last_obs": obs_np,
        "last_critic_obs": critic_obs_np,
        "done_returns": done_returns,
        "mean_reward": float(np.mean(reward_buf)),
        "reward_terms": {
            key: value / max(1, reward_term_count)
            for key, value in sorted(reward_term_sums.items())
        },
        "action_abs_mean": action_abs_sum / max(1, action_abs_count),
        "action_abs_max": action_abs_max,
        "mean_action_abs_mean": mean_action_abs_sum / max(1, mean_action_abs_count),
        "mean_action_abs_max": mean_action_abs_max,
    }


def _format_reward_terms(reward_terms: object) -> str:
    if not isinstance(reward_terms, dict):
        return ""
    parts = []
    for key in CONSOLE_REWARD_TERMS:
        if key in reward_terms:
            parts.append(f"{key}={float(reward_terms[key]):.3g}")
    return " ".join(parts)


def _compute_gae(
    rollout: dict[str, object],
    cfg: PPOConfig,
    model: ActorCritic,
    device: str,
) -> tuple[np.ndarray, np.ndarray]:
    rewards = rollout["rewards"]
    dones = rollout["dones"]
    values = rollout["values"]
    last_critic_obs = rollout["last_critic_obs"]
    assert isinstance(rewards, np.ndarray)
    assert isinstance(dones, np.ndarray)
    assert isinstance(values, np.ndarray)
    assert isinstance(last_critic_obs, np.ndarray)

    with torch.no_grad():
        last_values = model.value(torch.as_tensor(last_critic_obs, dtype=torch.float32, device=device))
    next_values = last_values.cpu().numpy()

    advantages = np.zeros_like(rewards, dtype=np.float32)
    last_gae = np.zeros(cfg.num_envs, dtype=np.float32)
    for step in reversed(range(cfg.steps_per_env)):
        next_non_terminal = 1.0 - dones[step]
        delta = rewards[step] + cfg.gamma * next_values * next_non_terminal - values[step]
        last_gae = delta + cfg.gamma * cfg.gae_lambda * next_non_terminal * last_gae
        advantages[step] = last_gae
        next_values = values[step]
    returns = advantages + values
    return advantages, returns


def _ppo_update(
    model: ActorCritic,
    optimizer: optim.Optimizer,
    rollout: dict[str, object],
    advantages: np.ndarray,
    returns: np.ndarray,
    cfg: PPOConfig,
    device: str,
) -> dict[str, float]:
    obs_np = rollout["obs"]
    critic_obs_np = rollout["critic_obs"]
    assert isinstance(obs_np, np.ndarray)
    assert isinstance(critic_obs_np, np.ndarray)
    obs = torch.as_tensor(obs_np.reshape(-1, obs_np.shape[-1]), dtype=torch.float32, device=device)
    critic_obs = torch.as_tensor(
        critic_obs_np.reshape(-1, critic_obs_np.shape[-1]),
        dtype=torch.float32,
        device=device,
    )
    actions = torch.as_tensor(rollout["actions"].reshape(-1, ACTION_DIM), dtype=torch.float32, device=device)
    old_log_probs = torch.as_tensor(rollout["log_probs"].reshape(-1), dtype=torch.float32, device=device)
    returns_t = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=device)
    adv_t = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=device)
    adv_std = adv_t.std(unbiased=False)
    adv_t = (adv_t - adv_t.mean()) / torch.clamp(adv_std, min=1.0e-8)

    batch_size = obs.shape[0]
    mini_batch_size = max(1, batch_size // cfg.num_mini_batches)

    last_policy_loss = 0.0
    last_value_loss = 0.0
    last_entropy = 0.0
    last_mean_action_l2 = 0.0
    last_mean_action_saturation = 0.0
    last_mean_action_abs = 0.0
    last_mean_action_max = 0.0
    for _ in range(cfg.num_learning_epochs):
        indices = torch.randperm(batch_size, device=device)
        for start in range(0, batch_size, mini_batch_size):
            mb_idx = indices[start: start + mini_batch_size]
            new_log_probs, entropy, values = model.evaluate(obs[mb_idx], actions[mb_idx], critic_obs[mb_idx])
            ratio = torch.exp(new_log_probs - old_log_probs[mb_idx])

            unclipped = ratio * adv_t[mb_idx]
            clipped = torch.clamp(ratio, 1.0 - cfg.clip_param, 1.0 + cfg.clip_param) * adv_t[mb_idx]
            policy_loss = -torch.min(unclipped, clipped).mean()
            value_loss = (returns_t[mb_idx] - values).pow(2).mean()
            entropy_loss = entropy.mean()
            mean_action = model.deterministic_action(obs[mb_idx])
            mean_action_l2_loss = mean_action.pow(2).mean()
            mean_action_saturation_loss = torch.relu(
                mean_action.abs() - cfg.mean_action_saturation_threshold
            ).pow(2).mean()

            loss = (
                policy_loss
                + cfg.value_loss_coef * value_loss
                - cfg.entropy_coef * entropy_loss
                + cfg.mean_action_l2_coef * mean_action_l2_loss
                + cfg.mean_action_saturation_coef * mean_action_saturation_loss
            )
            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), cfg.max_grad_norm)
            optimizer.step()
            model.clamp_log_std_()

            last_policy_loss = float(policy_loss.detach().cpu())
            last_value_loss = float(value_loss.detach().cpu())
            last_entropy = float(entropy_loss.detach().cpu())
            last_mean_action_l2 = float(mean_action_l2_loss.detach().cpu())
            last_mean_action_saturation = float(mean_action_saturation_loss.detach().cpu())
            mean_action_abs = mean_action.detach().abs()
            last_mean_action_abs = float(mean_action_abs.mean().cpu())
            last_mean_action_max = float(mean_action_abs.max().cpu())

    return {
        "policy_loss": last_policy_loss,
        "value_loss": last_value_loss,
        "entropy": last_entropy,
        "mean_action_l2_loss": last_mean_action_l2,
        "mean_action_saturation_loss": last_mean_action_saturation,
        "mean_action_abs": last_mean_action_abs,
        "mean_action_max": last_mean_action_max,
    }


def _save_checkpoint(
    path: Path,
    model: ActorCritic,
    env_cfg: M20EnvConfig,
    ppo_cfg: PPOConfig,
    total_steps: int,
) -> None:
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "obs_dim": model.actor_obs_dim,
            "actor_obs_dim": model.actor_obs_dim,
            "critic_obs_dim": model.critic_obs_dim,
            "action_dim": ACTION_DIM,
            "hidden_dims": tuple(ppo_cfg.hidden_dims),
            "action_limit": ppo_cfg.action_limit,
            "log_std_min": ppo_cfg.log_std_min,
            "log_std_max": ppo_cfg.log_std_max,
            "env_cfg": asdict(env_cfg),
            "ppo_cfg": asdict(ppo_cfg),
            "total_steps": total_steps,
        },
        path,
    )


def _load_checkpoint_into_model(path: Path, model: ActorCritic, device: str) -> int:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Resume checkpoint not found: {path}")

    ckpt = torch.load(path, map_location=device)
    actor_obs_dim = int(ckpt.get("actor_obs_dim", ckpt.get("obs_dim", OBS_DIM)))
    critic_obs_dim = int(ckpt.get("critic_obs_dim", actor_obs_dim))
    action_dim = int(ckpt.get("action_dim", ACTION_DIM))
    if action_dim != ACTION_DIM:
        raise ValueError(
            f"Checkpoint interface mismatch: actor_obs/action {actor_obs_dim}/{action_dim}, "
            f"expected */{ACTION_DIM}"
        )

    loaded_state = ckpt["model_state_dict"]
    current_state = model.state_dict()
    skip_critic = critic_obs_dim != model.critic_obs_dim
    compatible_state = {}
    skipped_keys = []
    for key, value in loaded_state.items():
        if skip_critic and key.startswith("critic."):
            skipped_keys.append(key)
            continue
        if key in current_state and current_state[key].shape == value.shape:
            compatible_state[key] = value
        elif (
            key in current_state
            and key.endswith(".0.weight")
            and current_state[key].ndim == 2
            and value.ndim == 2
            and current_state[key].shape[0] == value.shape[0]
        ):
            expanded = torch.zeros_like(current_state[key])
            copy_cols = min(expanded.shape[1], value.shape[1])
            expanded[:, :copy_cols] = value[:, :copy_cols]
            compatible_state[key] = expanded
            skipped_keys.append(f"{key}[partial_input_copy]")
        else:
            skipped_keys.append(key)
    current_state.update(compatible_state)
    model.load_state_dict(current_state)
    if skipped_keys:
        print(
            "[INFO] checkpoint warm-start loaded "
            f"{len(compatible_state)}/{len(loaded_state)} tensors; "
            f"skipped {len(skipped_keys)} incompatible tensors"
        )
    if skip_critic:
        print(
            "[INFO] critic input dim changed "
            f"{critic_obs_dim} -> {model.critic_obs_dim}; critic reinitialized, actor warm-started"
        )
    checkpoint_action_limit = ckpt.get("action_limit")
    if checkpoint_action_limit is not None:
        checkpoint_action_limit = float(checkpoint_action_limit)
        if abs(checkpoint_action_limit - model.action_limit) > 1.0e-6:
            print(
                "[INFO] checkpoint action_limit="
                f"{checkpoint_action_limit:.3f}, current action_limit={model.action_limit:.3f}; "
                "using current limit for resumed training/exported checkpoints"
            )
    model.clamp_log_std_()
    return int(ckpt.get("total_steps", 0))
