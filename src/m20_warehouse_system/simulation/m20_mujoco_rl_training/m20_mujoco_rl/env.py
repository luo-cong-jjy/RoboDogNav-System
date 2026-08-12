"""MuJoCo M20 reinforcement learning environment.

This is intentionally small and explicit.  It mirrors the deployment policy
interface instead of depending on Isaac Lab's manager based environment.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import mujoco
import numpy as np

from .config import M20EnvConfig
from .constants import (
    ACTION_DIM,
    ACTION_SCALE_POLICY,
    DEFAULT_POLICY_POS,
    DEFAULT_ROBOT_POS,
    GRAVITY_VEC,
    KD_ROBOT,
    KP_ROBOT,
    LEG_POLICY_INDICES,
    OBS_DIM,
    POLICY_ORDER,
    POLICY_TO_ROBOT,
    ROBOT_ORDER,
    ROBOT_TO_POLICY,
    WHEEL_POLICY_INDICES,
    WHEEL_ROBOT_INDICES,
)
from .math_utils import quat_rotate_inverse
from .terrain import resolve_model_xml


HIPX_POLICY_INDICES = np.array([0, 3, 6, 9], dtype=np.int64)
HIPY_POLICY_INDICES = np.array([1, 4, 7, 10], dtype=np.int64)
KNEE_POLICY_INDICES = np.array([2, 5, 8, 11], dtype=np.int64)
ZERO_WRENCH = np.zeros(6, dtype=np.float64)


class M20MujocoEnv:
    """Single M20 MuJoCo environment with a deployment-compatible policy API."""

    observation_dim = OBS_DIM
    action_dim = ACTION_DIM

    def __init__(self, cfg: M20EnvConfig | None = None, seed: int | None = None):
        self.cfg = cfg if cfg is not None else M20EnvConfig()
        if seed is not None:
            self.cfg = replace(self.cfg, seed=seed)
        self.rng = np.random.default_rng(self.cfg.seed)

        xml_path = resolve_model_xml(self.cfg.model_xml, self.cfg.terrain)
        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.model.opt.timestep = self.cfg.sim_dt
        self.data = mujoco.MjData(self.model)

        self.control_decimation = max(1, int(round(self.cfg.control_dt / self.cfg.sim_dt)))
        self.max_episode_steps = max(1, int(round(self.cfg.episode_seconds / self.cfg.control_dt)))
        self.command_resample_steps = max(1, int(round(self.cfg.command_resample_seconds / self.cfg.control_dt)))

        self.joint_ids = np.array([self._joint_id(name) for name in ROBOT_ORDER], dtype=np.int32)
        self.qpos_adr = np.array([self.model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.qvel_adr = np.array([self.model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.actuator_ids = np.array([self._actuator_id(name) for name in ROBOT_ORDER], dtype=np.int32)

        ctrlrange = self.model.actuator_ctrlrange[self.actuator_ids].astype(np.float64)
        self.ctrl_low = ctrlrange[:, 0]
        self.ctrl_high = ctrlrange[:, 1]
        self.leg_robot_indices = np.array(
            [idx for idx in range(ACTION_DIM) if idx not in set(WHEEL_ROBOT_INDICES)],
            dtype=np.int64,
        )
        self.joint_limited = self.model.jnt_limited[self.joint_ids].astype(bool)
        joint_range = self.model.jnt_range[self.joint_ids].astype(np.float64)
        self.joint_range_low = joint_range[:, 0]
        self.joint_range_high = joint_range[:, 1]

        self.base_body_id = self._body_id("base_link")
        robot_body_names = ["base_link", *(name.removesuffix("_joint") for name in ROBOT_ORDER)]
        wheel_body_names = [name.removesuffix("_joint") for name in ROBOT_ORDER if "wheel" in name]
        self.robot_body_ids = np.array([self._body_id(name) for name in robot_body_names], dtype=np.int32)
        self.wheel_body_ids = np.array([self._body_id(name) for name in wheel_body_names], dtype=np.int32)
        self.robot_body_id_set = {int(body_id) for body_id in self.robot_body_ids}
        self.wheel_body_id_set = {int(body_id) for body_id in self.wheel_body_ids}
        self.wheel_body_to_index = {int(body_id): idx for idx, body_id in enumerate(self.wheel_body_ids)}
        self.link_body_ids = self.robot_body_ids[self.robot_body_ids != self.base_body_id]
        self.randomized_geom_ids = self._randomized_geom_ids()
        self.nominal_body_mass = self.model.body_mass.copy()
        self.nominal_body_inertia = self.model.body_inertia.copy()
        self.nominal_body_ipos = self.model.body_ipos.copy()
        self.nominal_geom_friction = self.model.geom_friction.copy()
        self.contact_force_buffer = np.zeros(6, dtype=np.float64)
        self.gyro_sensor = self._sensor_slice("gyro")
        self.terrain_geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.ray_down = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        self.height_scan_offsets = self._make_height_scan_offsets()
        self.height_scan_dim = int(self.height_scan_offsets.shape[0])
        self.front_height_scan_mask = self._make_front_height_scan_mask()
        self.base_observation_dim = OBS_DIM
        self.observation_dim = OBS_DIM + (self.height_scan_dim if self.cfg.include_height_scan else 0)
        self.critic_observation_dim = (
            OBS_DIM
            + (3 if self.cfg.critic_base_lin_vel else 0)
            + (self.height_scan_dim if self.cfg.critic_height_scan else 0)
        )

        self.kp_robot = KP_ROBOT.copy()
        self.kd_robot = KD_ROBOT.copy()
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float64)
        self.command = np.zeros(3, dtype=np.float64)
        self.input_torque = np.zeros(ACTION_DIM, dtype=np.float64)
        self.prev_joint_vel_robot = np.zeros(ACTION_DIM, dtype=np.float64)
        self.wheel_contact_mask = np.zeros(len(self.wheel_body_ids), dtype=bool)
        self.wheel_air_time = np.zeros(len(self.wheel_body_ids), dtype=np.float64)
        self.nominal_wheel_clearance = np.zeros(len(self.wheel_body_ids), dtype=np.float64)
        self.external_wrench = np.zeros(6, dtype=np.float64)
        self.reset_wrench_steps_remaining = 0
        self.next_push_step = 0
        self.randomization_info: dict[str, Any] = {}
        self.step_count = 0
        self.episode_return = 0.0
        self.prev_base_x = 0.0
        self.undesired_contact_violation_steps = 0
        self._randomize_physics()

    def reset(self, seed: int | None = None) -> np.ndarray:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if self.cfg.randomization.randomize_physics_on_reset:
            self._randomize_physics()
        mujoco.mj_resetData(self.model, self.data)

        base_pos, base_quat, base_vel = self._sample_initial_base_state()
        self.data.qpos[0:3] = base_pos
        self.data.qpos[3:7] = base_quat

        if self.cfg.randomization.enabled:
            joint_noise = self._uniform_array(self.cfg.randomization.joint_pos_noise_range, ACTION_DIM)
            joint_vel = self._uniform_array(self.cfg.randomization.joint_vel_range, ACTION_DIM)
        else:
            joint_noise = np.zeros(ACTION_DIM, dtype=np.float64)
            joint_vel = np.zeros(ACTION_DIM, dtype=np.float64)
        joint_noise[3::4] = 0.0
        self.data.qpos[self.qpos_adr] = DEFAULT_ROBOT_POS + joint_noise
        self.data.qvel[:] = 0.0
        self.data.qvel[0:6] = base_vel
        self.data.qvel[self.qvel_adr] = joint_vel

        self.last_action.fill(0.0)
        self.input_torque.fill(0.0)
        mujoco.mj_forward(self.model, self.data)
        self._settle_after_reset()
        self.wheel_contact_mask.fill(False)
        self.wheel_air_time.fill(0.0)
        self.nominal_wheel_clearance = self._wheel_clearance_values()
        self._sample_reset_wrench()
        self._schedule_next_push()
        self.prev_joint_vel_robot = self.data.qvel[self.qvel_adr].copy()
        self.step_count = 0
        self.episode_return = 0.0
        self.prev_base_x = float(self.data.qpos[0])
        self.undesired_contact_violation_steps = 0
        self._sample_command()
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs()

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float64).reshape(ACTION_DIM)
        action = np.clip(action, -self.cfg.action_clip, self.cfg.action_clip)

        prev_x = float(self.data.qpos[0])
        prev_action = self.last_action.copy()
        prev_joint_vel_robot = self.data.qvel[self.qvel_adr].copy()
        target_pos_robot, target_vel_robot = self._policy_action_to_robot_targets(action)

        for _ in range(self.control_decimation):
            self._apply_external_wrench()
            self._apply_pd_torque(target_pos_robot, target_vel_robot)
            mujoco.mj_step(self.model, self.data)
            self._advance_external_wrench()

        self.step_count += 1
        self._maybe_push_robot()
        self.last_action = action.copy()
        if self.step_count % self.command_resample_steps == 0:
            self._sample_command()

        obs = self._get_obs()
        reward, reward_terms = self._compute_reward(action, prev_action, prev_x, prev_joint_vel_robot)
        self.prev_joint_vel_robot = self.data.qvel[self.qvel_adr].copy()
        undesired_contact_count = self._undesired_contact_count_from_reward_terms(reward_terms)
        if self.cfg.terminate_on_undesired_contact and (
            undesired_contact_count > self.cfg.undesired_contact_termination_threshold
        ):
            self.undesired_contact_violation_steps += 1
        else:
            self.undesired_contact_violation_steps = 0

        undesired_contact_terminated = (
            self.cfg.terminate_on_undesired_contact
            and self.undesired_contact_violation_steps
            >= max(1, self.cfg.undesired_contact_termination_steps)
        )
        if undesired_contact_terminated:
            reward += self.cfg.undesired_contact_terminal_penalty
            reward_terms["undesired_contact_terminal"] = self.cfg.undesired_contact_terminal_penalty

        terminated = self._is_terminated(obs) or undesired_contact_terminated
        truncated = self.step_count >= self.max_episode_steps
        done = bool(terminated or truncated)
        self.episode_return += reward
        info = {
            "terminated": terminated,
            "truncated": truncated,
            "undesired_contact_terminated": undesired_contact_terminated,
            "undesired_contact_count": undesired_contact_count,
            "undesired_contact_violation_steps": self.undesired_contact_violation_steps,
            "episode_return": self.episode_return,
            "reward_terms": reward_terms,
            "terrain": self.cfg.terrain.name,
            "xml_path": str(self.xml_path),
            "randomization": self.randomization_info,
        }
        return obs, float(reward), done, info

    def _undesired_contact_count_from_reward_terms(self, reward_terms: dict[str, float]) -> float:
        weight = self.cfg.reward.undesired_contact_weight
        if abs(weight) < 1.0e-8:
            return 0.0
        return max(0.0, float(reward_terms.get("undesired_contacts", 0.0)) / weight)

    def _policy_action_to_robot_targets(self, action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pos_policy = DEFAULT_POLICY_POS.copy()
        vel_policy = np.zeros(ACTION_DIM, dtype=np.float64)

        pos_policy[:12] += action[:12] * ACTION_SCALE_POLICY[:12]
        vel_policy[WHEEL_POLICY_INDICES] = action[WHEEL_POLICY_INDICES] * ACTION_SCALE_POLICY[WHEEL_POLICY_INDICES]

        return pos_policy[POLICY_TO_ROBOT], vel_policy[POLICY_TO_ROBOT]

    def _apply_pd_torque(self, target_pos_robot: np.ndarray, target_vel_robot: np.ndarray) -> None:
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.qvel_adr]
        torque = self.kp_robot * (target_pos_robot - q) + self.kd_robot * (target_vel_robot - dq)
        torque = np.nan_to_num(torque, nan=0.0, posinf=0.0, neginf=0.0)
        torque = np.clip(torque, self.ctrl_low, self.ctrl_high)
        self.input_torque = torque
        self.data.ctrl[self.actuator_ids] = torque

    def _get_base_obs(self) -> np.ndarray:
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.qvel_adr]
        quat = self.data.qpos[3:7].copy()

        base_ang_vel = self._base_ang_vel() * 0.25
        projected_gravity = quat_rotate_inverse(quat, GRAVITY_VEC)

        joint_pos_policy = q[ROBOT_TO_POLICY]
        joint_pos_policy[WHEEL_POLICY_INDICES] = 0.0
        joint_pos_rel = joint_pos_policy - DEFAULT_POLICY_POS
        joint_vel_policy = dq[ROBOT_TO_POLICY] * 0.05

        obs = np.concatenate(
            [
                base_ang_vel,
                projected_gravity,
                self.command,
                joint_pos_rel,
                joint_vel_policy,
                self.last_action,
            ]
        ).astype(np.float32)
        if obs.shape[0] != OBS_DIM:
            raise RuntimeError(f"Observation dim mismatch: {obs.shape[0]} != {OBS_DIM}")
        return obs

    def _get_obs(self) -> np.ndarray:
        parts = [self._get_base_obs()]
        if self.cfg.include_height_scan:
            parts.append(self._height_scan())
        obs = np.concatenate(parts).astype(np.float32)
        if obs.shape[0] != self.observation_dim:
            raise RuntimeError(f"Observation dim mismatch: {obs.shape[0]} != {self.observation_dim}")
        return obs

    def get_critic_obs(self) -> np.ndarray:
        parts = [self._get_base_obs()]
        if self.cfg.critic_base_lin_vel:
            parts.append(self._base_lin_vel_body().astype(np.float32))
        if self.cfg.critic_height_scan:
            parts.append(self._height_scan())
        return np.concatenate(parts).astype(np.float32)

    def _make_height_scan_offsets(self) -> np.ndarray:
        scan_cfg = self.cfg.height_scan
        resolution = max(float(scan_cfg.resolution), 1.0e-6)
        half_x = max(float(scan_cfg.size_x), 0.0) * 0.5
        half_y = max(float(scan_cfg.size_y), 0.0) * 0.5
        xs = np.arange(-half_x, half_x + 0.5 * resolution, resolution, dtype=np.float64)
        ys = np.arange(-half_y, half_y + 0.5 * resolution, resolution, dtype=np.float64)
        if xs.size == 0:
            xs = np.array([0.0], dtype=np.float64)
        if ys.size == 0:
            ys = np.array([0.0], dtype=np.float64)
        grid_x, grid_y = np.meshgrid(xs, ys, indexing="xy")
        return np.stack([grid_x.reshape(-1), grid_y.reshape(-1)], axis=1)

    def _make_front_height_scan_mask(self) -> np.ndarray:
        if self.height_scan_dim == 0:
            return np.zeros(0, dtype=bool)
        offsets = self.height_scan_offsets
        return (
            (offsets[:, 0] >= 0.05)
            & (offsets[:, 0] <= 0.75)
            & (np.abs(offsets[:, 1]) <= 0.45)
        )

    def _height_scan(self) -> np.ndarray:
        """Yaw-aligned terrain heights around the base for the privileged critic."""

        if self.height_scan_dim == 0:
            return np.zeros(0, dtype=np.float32)

        base_x = float(self.data.qpos[0])
        base_y = float(self.data.qpos[1])
        base_z = float(self.data.qpos[2])
        yaw = self._base_yaw()
        c = float(np.cos(yaw))
        s = float(np.sin(yaw))
        local_ground = self._terrain_height_at(base_x, base_y, base_z)
        heights = np.empty(self.height_scan_dim, dtype=np.float32)
        z_offset = float(self.cfg.height_scan.z_offset)

        for idx, (offset_x, offset_y) in enumerate(self.height_scan_offsets):
            world_x = base_x + c * float(offset_x) - s * float(offset_y)
            world_y = base_y + s * float(offset_x) + c * float(offset_y)
            terrain_height = self._terrain_height_at(world_x, world_y, base_z, z_offset=z_offset)
            heights[idx] = float(terrain_height - local_ground)

        clip = float(self.cfg.height_scan.clip)
        if clip > 0.0:
            heights = np.clip(heights, -clip, clip)
        return heights.astype(np.float32, copy=False)

    def _compute_reward(
        self,
        action: np.ndarray,
        prev_action: np.ndarray,
        prev_x: float,
        prev_joint_vel_robot: np.ndarray,
    ) -> tuple[float, dict[str, float]]:
        reward_cfg = self.cfg.reward
        quat = self.data.qpos[3:7].copy()
        base_lin_vel_body = quat_rotate_inverse(quat, self.data.qvel[0:3])
        base_ang_vel = self._base_ang_vel()
        projected_gravity = quat_rotate_inverse(quat, GRAVITY_VEC)
        q_robot = self.data.qpos[self.qpos_adr]
        dq_robot = self.data.qvel[self.qvel_adr]
        q_policy = q_robot[ROBOT_TO_POLICY]
        joint_pos_rel_policy = q_policy - DEFAULT_POLICY_POS

        lin_err = np.sum((base_lin_vel_body[:2] - self.command[:2]) ** 2)
        yaw_err = (base_ang_vel[2] - self.command[2]) ** 2
        track_lin = float(np.exp(-lin_err / 0.25))
        track_yaw = float(np.exp(-yaw_err / 0.25))

        z_vel_penalty = float(self.data.qvel[2] ** 2)
        orientation_penalty = float(np.sum(projected_gravity[:2] ** 2))
        terrain_height = self._terrain_height_below_base()
        base_height = float(self.data.qpos[2] - terrain_height)
        height_penalty = float((base_height - self.cfg.target_base_height) ** 2)
        torque_penalty = float(np.mean(self.input_torque ** 2))
        action_rate_penalty = float(np.mean((action - prev_action) ** 2))
        leg_action_penalty = float(np.mean(action[LEG_POLICY_INDICES] ** 2))
        wheel_action_penalty = float(np.mean(action[WHEEL_POLICY_INDICES] ** 2))
        saturation_threshold = reward_cfg.action_saturation_threshold
        action_saturation_penalty = float(np.mean(np.maximum(np.abs(action) - saturation_threshold, 0.0) ** 2))
        progress = float((self.data.qpos[0] - prev_x) / self.cfg.control_dt)
        prev_terrain_height = self._terrain_height_at(prev_x, self.data.qpos[1], self.data.qpos[2])
        terrain_height_progress = float(max(terrain_height - prev_terrain_height, 0.0) / self.cfg.control_dt)
        stair_height = float(max(terrain_height, 0.0))
        stair_forward_progress = self._stair_forward_progress(progress, terrain_height)
        joint_acc = (dq_robot - prev_joint_vel_robot) / self.cfg.control_dt
        joint_acc_penalty = float(np.mean(joint_acc[self.leg_robot_indices] ** 2))
        wheel_acc_penalty = float(np.mean(joint_acc[WHEEL_ROBOT_INDICES] ** 2))
        joint_limit_penalty = self._joint_pos_limit_penalty(q_robot)
        leg_power = dq_robot[self.leg_robot_indices] * self.input_torque[self.leg_robot_indices]
        joint_power_penalty = float(np.mean(np.abs(leg_power)))

        command_norm = float(np.linalg.norm(self.command))
        base_xy_speed = float(np.linalg.norm(base_lin_vel_body[:2]))
        stand_still_penalty = 0.0
        feet_contact_without_cmd = 0.0
        if command_norm < reward_cfg.stand_still_command_threshold:
            stand_still_penalty = float(np.mean(joint_pos_rel_policy[LEG_POLICY_INDICES] ** 2))
            stand_still_penalty += float(
                max(base_xy_speed - reward_cfg.stand_still_velocity_threshold, 0.0) ** 2
            )

        hipx_penalty = float(np.mean(np.abs(joint_pos_rel_policy[HIPX_POLICY_INDICES])))
        hipy_penalty = float(np.mean(np.abs(joint_pos_rel_policy[HIPY_POLICY_INDICES])))
        knee_penalty = float(np.mean(np.abs(joint_pos_rel_policy[KNEE_POLICY_INDICES])))
        fl_rel = joint_pos_rel_policy[0:3]
        fr_rel = joint_pos_rel_policy[3:6]
        hl_rel = joint_pos_rel_policy[6:9]
        hr_rel = joint_pos_rel_policy[9:12]
        mirror_penalty = float(np.mean((fl_rel - hr_rel) ** 2) + np.mean((fr_rel - hl_rel) ** 2))
        (
            undesired_contacts,
            contact_force_excess,
            wheel_contact_fraction,
            wheel_contact_mask,
            wheel_stumble_count,
        ) = self._contact_reward_terms()
        wheel_air_time = self._wheel_air_time_reward(wheel_contact_mask, command_norm)
        wheel_clearance = self._wheel_clearance_reward(command_norm)
        if command_norm < reward_cfg.stand_still_command_threshold:
            feet_contact_without_cmd = wheel_contact_fraction
        upward = float(np.clip(-projected_gravity[2], 0.0, 1.0))

        reward_terms = {
            "track_lin": reward_cfg.track_lin_weight * track_lin,
            "track_yaw": reward_cfg.track_yaw_weight * track_yaw,
            "progress": reward_cfg.progress_weight * progress,
            "terrain_height_progress": reward_cfg.terrain_height_progress_weight * terrain_height_progress,
            "stair_height": reward_cfg.stair_height_weight * stair_height,
            "stair_forward_progress": reward_cfg.stair_forward_progress_weight * stair_forward_progress,
            "z_vel": reward_cfg.z_vel_weight * z_vel_penalty,
            "orientation": reward_cfg.orientation_weight * orientation_penalty,
            "base_height": reward_cfg.base_height_weight * height_penalty,
            "torque": reward_cfg.torque_weight * torque_penalty,
            "action_rate": reward_cfg.action_rate_weight * action_rate_penalty,
            "leg_action_l2": reward_cfg.leg_action_l2_weight * leg_action_penalty,
            "wheel_action_l2": reward_cfg.wheel_action_l2_weight * wheel_action_penalty,
            "action_saturation": reward_cfg.action_saturation_weight * action_saturation_penalty,
            "joint_acc": reward_cfg.joint_acc_weight * joint_acc_penalty,
            "wheel_acc": reward_cfg.wheel_acc_weight * wheel_acc_penalty,
            "joint_pos_limits": reward_cfg.joint_pos_limits_weight * joint_limit_penalty,
            "joint_power": reward_cfg.joint_power_weight * joint_power_penalty,
            "stand_still": reward_cfg.stand_still_weight * stand_still_penalty,
            "hipx_pos": reward_cfg.hipx_pos_weight * hipx_penalty,
            "hipy_pos": reward_cfg.hipy_pos_weight * hipy_penalty,
            "knee_pos": reward_cfg.knee_pos_weight * knee_penalty,
            "joint_mirror": reward_cfg.joint_mirror_weight * mirror_penalty,
            "undesired_contacts": reward_cfg.undesired_contact_weight * undesired_contacts,
            "contact_force": reward_cfg.contact_force_weight * contact_force_excess,
            "feet_contact_without_cmd": reward_cfg.feet_contact_without_cmd_weight * feet_contact_without_cmd,
            "upward": reward_cfg.upward_weight * upward,
            "wheel_air_time": reward_cfg.wheel_air_time_weight * wheel_air_time,
            "wheel_clearance": reward_cfg.wheel_clearance_weight * wheel_clearance,
            "wheel_stumble": reward_cfg.wheel_stumble_weight * wheel_stumble_count,
            "alive": reward_cfg.alive_weight,
        }
        return float(sum(reward_terms.values())), reward_terms

    def _stair_forward_progress(self, progress: float, terrain_height: float) -> float:
        target_height = float(getattr(self.cfg.terrain, "stair_height", 0.0))
        if target_height <= 0.0:
            return 0.0
        min_height = target_height * float(self.cfg.reward.stair_forward_progress_height_fraction)
        if terrain_height < min_height:
            return 0.0
        return float(max(progress, 0.0))

    def _joint_pos_limit_penalty(self, q_robot: np.ndarray) -> float:
        limited = self.joint_limited
        if not np.any(limited):
            return 0.0

        span = self.joint_range_high - self.joint_range_low
        margin = np.maximum(span * self.cfg.reward.joint_limit_margin_ratio, 0.0)
        soft_low = self.joint_range_low + margin
        soft_high = self.joint_range_high - margin
        lower_violation = np.maximum(soft_low - q_robot, 0.0)
        upper_violation = np.maximum(q_robot - soft_high, 0.0)
        violation = lower_violation + upper_violation
        return float(np.mean(violation[limited] ** 2))

    def _contact_reward_terms(self) -> tuple[float, float, float, np.ndarray, float]:
        undesired_contacts = 0.0
        contact_force_excess = 0.0
        wheel_bodies_in_contact: set[int] = set()
        wheel_contact_mask = np.zeros(len(self.wheel_body_ids), dtype=bool)
        wheel_stumble_count = 0.0

        for contact_idx in range(self.data.ncon):
            contact = self.data.contact[contact_idx]
            body1 = int(self.model.geom_bodyid[contact.geom1])
            body2 = int(self.model.geom_bodyid[contact.geom2])
            body1_robot = body1 in self.robot_body_id_set
            body2_robot = body2 in self.robot_body_id_set
            if not (body1_robot or body2_robot):
                continue

            body1_wheel = body1 in self.wheel_body_id_set
            body2_wheel = body2 in self.wheel_body_id_set
            if body1_wheel or body2_wheel:
                if body1_wheel:
                    wheel_bodies_in_contact.add(body1)
                    wheel_contact_mask[self.wheel_body_to_index[body1]] = True
                if body2_wheel:
                    wheel_bodies_in_contact.add(body2)
                    wheel_contact_mask[self.wheel_body_to_index[body2]] = True
                mujoco.mj_contactForce(self.model, self.data, contact_idx, self.contact_force_buffer)
                force_norm = float(np.linalg.norm(self.contact_force_buffer[:3]))
                if np.isfinite(force_norm):
                    contact_force_excess += max(force_norm - self.cfg.reward.contact_force_threshold, 0.0)
                    normal = np.asarray(contact.frame[:3], dtype=np.float64)
                    normal_z = abs(float(normal[2]))
                    if (
                        normal_z < self.cfg.reward.wheel_stumble_normal_z_threshold
                        and force_norm > self.cfg.reward.wheel_stumble_force_threshold
                    ):
                        wheel_stumble_count += 1.0
            else:
                undesired_contacts += 1.0

        wheel_contact_fraction = len(wheel_bodies_in_contact) / max(1, len(self.wheel_body_id_set))
        return (
            float(undesired_contacts),
            float(contact_force_excess),
            float(wheel_contact_fraction),
            wheel_contact_mask,
            float(wheel_stumble_count),
        )

    def _wheel_air_time_reward(self, wheel_contact_mask: np.ndarray, command_norm: float) -> float:
        first_contact = wheel_contact_mask & ~self.wheel_contact_mask
        if command_norm > self.cfg.reward.stand_still_command_threshold:
            excess_air_time = np.maximum(self.wheel_air_time - self.cfg.reward.wheel_air_time_threshold, 0.0)
            reward = float(np.sum(excess_air_time * first_contact.astype(np.float64)))
        else:
            reward = 0.0
        self.wheel_air_time = np.where(wheel_contact_mask, 0.0, self.wheel_air_time + self.cfg.control_dt)
        self.wheel_contact_mask = wheel_contact_mask.copy()
        return reward

    def _wheel_clearance_reward(self, command_norm: float) -> float:
        if command_norm <= self.cfg.reward.stand_still_command_threshold:
            return 0.0
        terrain_delta = self._front_terrain_delta()
        if terrain_delta < self.cfg.reward.wheel_clearance_terrain_threshold:
            return 0.0
        target = max(float(self.cfg.reward.wheel_clearance_lift_target), 1.0e-6)
        clearance_delta = self._wheel_clearance_values() - self.nominal_wheel_clearance
        return float(np.mean(np.clip(clearance_delta / target, 0.0, 1.0)))

    def _front_terrain_delta(self) -> float:
        if self.height_scan_dim == 0:
            return 0.0
        heights = self._height_scan()
        if self.front_height_scan_mask.size == heights.size and np.any(self.front_height_scan_mask):
            heights = heights[self.front_height_scan_mask]
        return float(max(np.max(heights), 0.0))

    def _wheel_clearance_values(self) -> np.ndarray:
        clearances = np.zeros(len(self.wheel_body_ids), dtype=np.float64)
        for idx, body_id in enumerate(self.wheel_body_ids):
            pos = self.data.xpos[int(body_id)]
            terrain_height = self._terrain_height_at(float(pos[0]), float(pos[1]), float(pos[2]))
            clearances[idx] = float(pos[2] - terrain_height)
        return clearances

    def _terrain_height_below_base(self) -> float:
        """Estimate local ground height under the base, like Isaac Lab's height_scanner_base."""

        return self._terrain_height_at(self.data.qpos[0], self.data.qpos[1], self.data.qpos[2])

    def _terrain_height_at(
        self,
        x: float,
        y: float,
        z_reference: float,
        z_offset: float = 2.0,
    ) -> float:
        origin = np.array(
            [x, y, z_reference + z_offset],
            dtype=np.float64,
        )
        geomid = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(
            self.model,
            self.data,
            origin,
            self.ray_down,
            self.terrain_geomgroup,
            True,
            self.base_body_id,
            geomid,
        )
        if distance < 0.0 or not np.isfinite(distance):
            return 0.0
        return float(origin[2] - distance)

    def _randomize_physics(self) -> None:
        self._restore_nominal_model()
        self.kp_robot = KP_ROBOT.copy()
        self.kd_robot = KD_ROBOT.copy()
        self.randomization_info = {"enabled": bool(self.cfg.randomization.enabled)}
        if not self.cfg.randomization.enabled:
            mujoco.mj_setConst(self.model, self.data)
            return

        rand_cfg = self.cfg.randomization
        link_scales = self._uniform_array(rand_cfg.link_mass_scale_range, len(self.link_body_ids))
        for body_id, scale in zip(self.link_body_ids, link_scales):
            body_idx = int(body_id)
            self.model.body_mass[body_idx] = self.nominal_body_mass[body_idx] * scale
            self.model.body_inertia[body_idx] = self.nominal_body_inertia[body_idx] * scale

        base_mass_add = self._uniform(rand_cfg.base_mass_add_range)
        nominal_base_mass = float(self.nominal_body_mass[self.base_body_id])
        base_mass = max(0.1, nominal_base_mass + base_mass_add)
        base_scale = base_mass / max(nominal_base_mass, 1.0e-8)
        self.model.body_mass[self.base_body_id] = base_mass
        self.model.body_inertia[self.base_body_id] = self.nominal_body_inertia[self.base_body_id] * base_scale

        com_offset = np.array(
            [
                self._uniform(rand_cfg.base_com_x_range),
                self._uniform(rand_cfg.base_com_y_range),
                self._uniform(rand_cfg.base_com_z_range),
            ],
            dtype=np.float64,
        )
        self.model.body_ipos[self.base_body_id] = self.nominal_body_ipos[self.base_body_id] + com_offset

        friction = self._uniform(rand_cfg.friction_range)
        if len(self.randomized_geom_ids) > 0:
            nominal_friction = self.nominal_geom_friction[self.randomized_geom_ids]
            scale = friction / np.maximum(nominal_friction[:, 0], 1.0e-6)
            self.model.geom_friction[self.randomized_geom_ids] = nominal_friction * scale[:, None]
            self.model.geom_friction[self.randomized_geom_ids, 0] = friction
            self.model.geom_friction[self.randomized_geom_ids, 1:] = np.maximum(
                self.model.geom_friction[self.randomized_geom_ids, 1:],
                1.0e-5,
            )

        kp_scale = self._uniform_array(rand_cfg.kp_scale_range, ACTION_DIM)
        kd_scale = self._uniform_array(rand_cfg.kd_scale_range, ACTION_DIM)
        self.kp_robot = KP_ROBOT * kp_scale
        self.kd_robot = KD_ROBOT * kd_scale
        mujoco.mj_setConst(self.model, self.data)

        self.randomization_info.update(
            {
                "base_mass": float(base_mass),
                "base_mass_add": float(base_mass_add),
                "link_mass_scale_mean": float(np.mean(link_scales)) if len(link_scales) else 1.0,
                "base_com_offset": com_offset.tolist(),
                "friction": float(friction),
                "kp_scale_mean": float(np.mean(kp_scale)),
                "kd_scale_mean": float(np.mean(kd_scale)),
            }
        )

    def _restore_nominal_model(self) -> None:
        self.model.body_mass[:] = self.nominal_body_mass
        self.model.body_inertia[:] = self.nominal_body_inertia
        self.model.body_ipos[:] = self.nominal_body_ipos
        self.model.geom_friction[:] = self.nominal_geom_friction

    def _sample_initial_base_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.cfg.randomization.enabled:
            return (
                np.array([self.cfg.base_init_x, self.cfg.base_init_y, self.cfg.base_init_height], dtype=np.float64),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                np.zeros(6, dtype=np.float64),
            )

        rand_cfg = self.cfg.randomization
        pos = np.array(
            [
                self.cfg.base_init_x + self._uniform(rand_cfg.base_x_range),
                self.cfg.base_init_y + self._uniform(rand_cfg.base_y_range),
                self.cfg.base_init_height + self._uniform(rand_cfg.base_z_range),
            ],
            dtype=np.float64,
        )
        quat = self._quat_from_euler(
            self._uniform(rand_cfg.base_roll_range),
            self._uniform(rand_cfg.base_pitch_range),
            self._uniform(rand_cfg.base_yaw_range),
        )
        vel = np.array(
            [
                self._uniform(rand_cfg.base_lin_vel_range),
                self._uniform(rand_cfg.base_lin_vel_range),
                self._uniform(rand_cfg.base_z_vel_range),
                self._uniform(rand_cfg.base_ang_vel_range),
                self._uniform(rand_cfg.base_ang_vel_range),
                self._uniform(rand_cfg.base_ang_vel_range),
            ],
            dtype=np.float64,
        )
        return pos, quat, vel

    def _sample_reset_wrench(self) -> None:
        rand_cfg = self.cfg.randomization
        self.external_wrench[:] = 0.0
        self.reset_wrench_steps_remaining = 0
        if not (rand_cfg.enabled and rand_cfg.reset_wrench_enabled):
            return

        self.external_wrench[:3] = self._uniform_array(rand_cfg.reset_force_range, 3)
        self.external_wrench[3:] = self._uniform_array(rand_cfg.reset_torque_range, 3)
        duration = self._uniform(rand_cfg.reset_wrench_duration_range)
        self.reset_wrench_steps_remaining = max(1, int(round(duration / self.cfg.sim_dt)))
        self.randomization_info["reset_wrench"] = self.external_wrench.tolist()

    def _settle_after_reset(self) -> None:
        settle_steps = max(0, int(round(self.cfg.reset_settle_seconds / self.cfg.sim_dt)))
        if settle_steps == 0:
            return

        target_pos_robot = DEFAULT_ROBOT_POS.copy()
        target_vel_robot = np.zeros(ACTION_DIM, dtype=np.float64)
        for _ in range(settle_steps):
            self._apply_pd_torque(target_pos_robot, target_vel_robot)
            mujoco.mj_step(self.model, self.data)

        self.data.qvel[:] = 0.0
        self.input_torque.fill(0.0)
        self.data.ctrl[self.actuator_ids] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _apply_external_wrench(self) -> None:
        if self.reset_wrench_steps_remaining > 0:
            self.data.xfrc_applied[self.base_body_id] = self.external_wrench
        else:
            self.data.xfrc_applied[self.base_body_id] = ZERO_WRENCH

    def _advance_external_wrench(self) -> None:
        if self.reset_wrench_steps_remaining > 0:
            self.reset_wrench_steps_remaining -= 1
            if self.reset_wrench_steps_remaining == 0:
                self.data.xfrc_applied[self.base_body_id] = ZERO_WRENCH

    def _schedule_next_push(self) -> None:
        rand_cfg = self.cfg.randomization
        if not (rand_cfg.enabled and rand_cfg.push_enabled):
            self.next_push_step = self.max_episode_steps + 1
            return
        interval_s = self._uniform(rand_cfg.push_interval_range_s)
        self.next_push_step = self.step_count + max(1, int(round(interval_s / self.cfg.control_dt)))

    def _maybe_push_robot(self) -> None:
        rand_cfg = self.cfg.randomization
        if not (rand_cfg.enabled and rand_cfg.push_enabled):
            return
        if self.step_count < self.next_push_step:
            return
        push_xy = self._uniform_array(rand_cfg.push_lin_vel_range, 2)
        self.data.qvel[0:2] += push_xy
        self.randomization_info["last_push_xy"] = push_xy.tolist()
        self._schedule_next_push()

    def _randomized_geom_ids(self) -> np.ndarray:
        geom_ids: list[int] = []
        for geom_id in range(self.model.ngeom):
            has_contact = bool(self.model.geom_contype[geom_id] or self.model.geom_conaffinity[geom_id])
            if has_contact:
                geom_ids.append(geom_id)
        return np.array(geom_ids, dtype=np.int32)

    def _uniform(self, value_range: tuple[float, float]) -> float:
        low, high = value_range
        if high <= low:
            return float(low)
        return float(self.rng.uniform(low, high))

    def _uniform_array(self, value_range: tuple[float, float], size: int) -> np.ndarray:
        low, high = value_range
        if high <= low:
            return np.full(size, low, dtype=np.float64)
        return self.rng.uniform(low, high, size=size).astype(np.float64)

    @staticmethod
    def _quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        quat = np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=np.float64,
        )
        return quat / max(np.linalg.norm(quat), 1.0e-8)

    def _base_yaw(self) -> float:
        quat = self.data.qpos[3:7]
        w, x, y, z = (float(v) for v in quat)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    def _is_terminated(self, obs: np.ndarray) -> bool:
        base_z = float(self.data.qpos[2])
        projected_gravity_z = float(obs[5])
        too_low = base_z < self.cfg.terminate_base_height
        bad_orientation = projected_gravity_z > self.cfg.terminate_projected_gravity_z
        out_of_bounds = abs(float(self.data.qpos[1])) > 3.0 or float(self.data.qpos[0]) < -1.0
        return bool(too_low or bad_orientation or out_of_bounds)

    def _sample_command(self) -> None:
        if self.cfg.command_mode == "forward":
            self.command[0] = self.rng.uniform(*self.cfg.forward_command_range)
            self.command[1] = 0.0
            self.command[2] = 0.0
        elif self.cfg.command_mode == "random":
            self.command[0] = self.rng.uniform(*self.cfg.forward_command_range)
            self.command[1] = self.rng.uniform(*self.cfg.lateral_command_range)
            self.command[2] = self.rng.uniform(*self.cfg.yaw_command_range)
        else:
            raise ValueError(f"Unsupported command_mode: {self.cfg.command_mode}")

    def _base_ang_vel(self) -> np.ndarray:
        if self.gyro_sensor is not None:
            adr, dim = self.gyro_sensor
            return self.data.sensordata[adr: adr + dim].copy()
        quat = self.data.qpos[3:7].copy()
        return quat_rotate_inverse(quat, self.data.qvel[3:6])

    def _base_lin_vel_body(self) -> np.ndarray:
        quat = self.data.qpos[3:7].copy()
        return quat_rotate_inverse(quat, self.data.qvel[0:3])

    def _sensor_slice(self, name: str) -> tuple[int, int] | None:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            return None
        return int(self.model.sensor_adr[sid]), int(self.model.sensor_dim[sid])

    def _joint_id(self, name: str) -> int:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise KeyError(f"Joint not found in MuJoCo model: {name}")
        return int(jid)

    def _actuator_id(self, name: str) -> int:
        aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise KeyError(f"Actuator not found in MuJoCo model: {name}")
        return int(aid)

    def _body_id(self, name: str) -> int:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"Body not found in MuJoCo model: {name}")
        return int(bid)
