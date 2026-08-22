# ======================================================================
# env.py —— M20 MuJoCo 强化学习环境（中文注释版）
# 作用：实现与部署策略接口一致的单机器人 MuJoCo 仿真环境：
#   - reset/step 标准 gym 接口，观测与部署 57 维接口对齐
#   - PD 关节控制（腿位置控制 + 轮速度控制）、动作限幅与缩放
#   - 奖励函数（速度跟踪/进步/姿态/接触/轮子相关等 30+ 项）
#   - 域随机化（质量/惯量/质心/摩擦/PD 增益/初始状态/推力/外力）
#   - 特权 critic 观测（机身线速度 + 高度扫描）
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""MuJoCo M20 reinforcement learning environment.

This is intentionally small and explicit.  It mirrors the deployment policy
interface instead of depending on Isaac Lab's manager based environment.
"""

from __future__ import annotations

# 数据类：替换配置副本
from dataclasses import replace
# 类型标注：Any
from typing import Any

# MuJoCo 物理引擎
import mujoco
# NumPy
import numpy as np

# 环境配置
from .config import M20EnvConfig
# 常量：维度/缩放/默认位姿/重力/PD 增益/关节索引/顺序映射
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
# 四元数旋转工具
from .math_utils import quat_rotate_inverse
# 地形 XML 解析
from .terrain import resolve_model_xml


# 策略序中的 hipx 关节索引（每条腿第 1 个关节：fl/fr/hl/hr）
HIPX_POLICY_INDICES = np.array([0, 3, 6, 9], dtype=np.int64)
# 策略序中的 hipy 关节索引（每条腿第 2 个关节）
HIPY_POLICY_INDICES = np.array([1, 4, 7, 10], dtype=np.int64)
# 策略序中的 knee 关节索引（每条腿第 3 个关节）
KNEE_POLICY_INDICES = np.array([2, 5, 8, 11], dtype=np.int64)
# 零外力/力矩（6 维：力 xyz + 力矩 xyz）
ZERO_WRENCH = np.zeros(6, dtype=np.float64)


# M20 单环境封装：提供与部署兼容的策略接口
class M20MujocoEnv:
    """Single M20 MuJoCo environment with a deployment-compatible policy API."""

    # 观测维度（不含高度扫描时的基础维度）
    observation_dim = OBS_DIM
    # 动作维度
    action_dim = ACTION_DIM

    def __init__(self, cfg: M20EnvConfig | None = None, seed: int | None = None):
        # 使用默认配置或传入配置
        self.cfg = cfg if cfg is not None else M20EnvConfig()
        if seed is not None:
            # 覆盖种子
            self.cfg = replace(self.cfg, seed=seed)
        # 随机数生成器（用于随机化）
        self.rng = np.random.default_rng(self.cfg.seed)

        # 解析模型 XML 路径（根据地形配置选择/生成）
        xml_path = resolve_model_xml(self.cfg.model_xml, self.cfg.terrain)
        self.xml_path = xml_path
        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        # 设置仿真步长
        self.model.opt.timestep = self.cfg.sim_dt
        # 创建仿真数据
        self.data = mujoco.MjData(self.model)

        # 控制解算次数（控制步长/仿真步长，至少 1）
        self.control_decimation = max(1, int(round(self.cfg.control_dt / self.cfg.sim_dt)))
        # 单回合最大控制步数
        self.max_episode_steps = max(1, int(round(self.cfg.episode_seconds / self.cfg.control_dt)))
        # 指令重采样间隔步数
        self.command_resample_steps = max(1, int(round(self.cfg.command_resample_seconds / self.cfg.control_dt)))

        # 机器人序关节 ID 数组
        self.joint_ids = np.array([self._joint_id(name) for name in ROBOT_ORDER], dtype=np.int32)
        # 各关节在 qpos 中的地址
        self.qpos_adr = np.array([self.model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        # 各关节在 qvel 中的地址
        self.qvel_adr = np.array([self.model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int32)
        # 各关节对应执行器 ID
        self.actuator_ids = np.array([self._actuator_id(name) for name in ROBOT_ORDER], dtype=np.int32)

        # 执行器控制量范围（用于力矩裁剪）
        ctrlrange = self.model.actuator_ctrlrange[self.actuator_ids].astype(np.float64)
        self.ctrl_low = ctrlrange[:, 0]
        self.ctrl_high = ctrlrange[:, 1]
        # 腿部（非轮子）关节的机器人序索引
        self.leg_robot_indices = np.array(
            [idx for idx in range(ACTION_DIM) if idx not in set(WHEEL_ROBOT_INDICES)],
            dtype=np.int64,
        )
        # 关节是否有限位
        self.joint_limited = self.model.jnt_limited[self.joint_ids].astype(bool)
        # 关节运动范围
        joint_range = self.model.jnt_range[self.joint_ids].astype(np.float64)
        self.joint_range_low = joint_range[:, 0]
        self.joint_range_high = joint_range[:, 1]

        # 机身 body ID
        self.base_body_id = self._body_id("base_link")
        # 机器人所有 body 名（机身 + 各关节对应的 body）
        robot_body_names = ["base_link", *(name.removesuffix("_joint") for name in ROBOT_ORDER)]
        # 轮子 body 名
        wheel_body_names = [name.removesuffix("_joint") for name in ROBOT_ORDER if "wheel" in name]
        self.robot_body_ids = np.array([self._body_id(name) for name in robot_body_names], dtype=np.int32)
        self.wheel_body_ids = np.array([self._body_id(name) for name in wheel_body_names], dtype=np.int32)
        # 集合便于快速判断
        self.robot_body_id_set = {int(body_id) for body_id in self.robot_body_ids}
        self.wheel_body_id_set = {int(body_id) for body_id in self.wheel_body_ids}
        # 轮子 body -> 索引映射
        self.wheel_body_to_index = {int(body_id): idx for idx, body_id in enumerate(self.wheel_body_ids)}
        # 连杆 body（除机身外）
        self.link_body_ids = self.robot_body_ids[self.robot_body_ids != self.base_body_id]
        # 参与接触的几何体 ID（用于摩擦随机化）
        self.randomized_geom_ids = self._randomized_geom_ids()
        # 保存标称模型参数（用于随机化后恢复）
        self.nominal_body_mass = self.model.body_mass.copy()
        self.nominal_body_inertia = self.model.body_inertia.copy()
        self.nominal_body_ipos = self.model.body_ipos.copy()
        self.nominal_geom_friction = self.model.geom_friction.copy()
        # 接触力计算缓冲区（6 维）
        self.contact_force_buffer = np.zeros(6, dtype=np.float64)
        # 陀螺仪传感器切片（存在则使用，否则用四元数推算）
        self.gyro_sensor = self._sensor_slice("gyro")
        # 地形射线组掩码（只射中 terrain 组）
        self.terrain_geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        # 向下射线方向
        self.ray_down = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        # 高度扫描网格偏移（机体坐标系）
        self.height_scan_offsets = self._make_height_scan_offsets()
        # 高度扫描维度
        self.height_scan_dim = int(self.height_scan_offsets.shape[0])
        # 前方高度扫描掩码（用于轮子抬升奖励）
        self.front_height_scan_mask = self._make_front_height_scan_mask()
        # 基础观测维度
        self.base_observation_dim = OBS_DIM
        # 实际观测维度（可选加高度扫描）
        self.observation_dim = OBS_DIM + (self.height_scan_dim if self.cfg.include_height_scan else 0)
        # critic 观测维度（可选加机身线速度/高度扫描）
        self.critic_observation_dim = (
            OBS_DIM
            + (3 if self.cfg.critic_base_lin_vel else 0)
            + (self.height_scan_dim if self.cfg.critic_height_scan else 0)
        )

        # 当前 PD 增益（机器人序，随机化时可能缩放）
        self.kp_robot = KP_ROBOT.copy()
        self.kd_robot = KD_ROBOT.copy()
        # 上一步动作
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float64)
        # 当前指令（vx, vy, wz）
        self.command = np.zeros(3, dtype=np.float64)
        # 输入力矩（机器人序）
        self.input_torque = np.zeros(ACTION_DIM, dtype=np.float64)
        # 上一步关节速度（用于加速度惩罚）
        self.prev_joint_vel_robot = np.zeros(ACTION_DIM, dtype=np.float64)
        # 轮子接触掩码
        self.wheel_contact_mask = np.zeros(len(self.wheel_body_ids), dtype=bool)
        # 轮子腾空时间
        self.wheel_air_time = np.zeros(len(self.wheel_body_ids), dtype=np.float64)
        # 轮子标称离地高度（重置时测量）
        self.nominal_wheel_clearance = np.zeros(len(self.wheel_body_ids), dtype=np.float64)
        # 外部力/力矩（重置 wrench）
        self.external_wrench = np.zeros(6, dtype=np.float64)
        # 重置 wrench 剩余步数
        self.reset_wrench_steps_remaining = 0
        # 下次推力步数
        self.next_push_step = 0
        # 随机化信息（记录到 info）
        self.randomization_info: dict[str, Any] = {}
        # 步数计数
        self.step_count = 0
        # 回合累计回报
        self.episode_return = 0.0
        # 上一步 x 位置（用于进步奖励）
        self.prev_base_x = 0.0
        # 连续非轮子接触违规步数
        self.undesired_contact_violation_steps = 0
        # 初始化时随机化物理属性
        self._randomize_physics()

    # 重置环境：随机化物理、重置状态、采样初始状态与指令，返回观测
    def reset(self, seed: int | None = None) -> np.ndarray:
        # 指定种子则重置 RNG
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        # 按配置决定是否重新随机化物理属性
        if self.cfg.randomization.randomize_physics_on_reset:
            self._randomize_physics()
        # 重置 MuJoCo 数据
        mujoco.mj_resetData(self.model, self.data)

        # 采样初始机身位姿与速度
        base_pos, base_quat, base_vel = self._sample_initial_base_state()
        # 写入机身位置与姿态（自由关节 qpos 前 7 维）
        self.data.qpos[0:3] = base_pos
        self.data.qpos[3:7] = base_quat

        # 关节初始状态：启用随机化则加噪声
        if self.cfg.randomization.enabled:
            joint_noise = self._uniform_array(self.cfg.randomization.joint_pos_noise_range, ACTION_DIM)
            joint_vel = self._uniform_array(self.cfg.randomization.joint_vel_range, ACTION_DIM)
        else:
            joint_noise = np.zeros(ACTION_DIM, dtype=np.float64)
            joint_vel = np.zeros(ACTION_DIM, dtype=np.float64)
        # 轮子关节位置不加噪声（轮子位置无意义）
        joint_noise[3::4] = 0.0
        # 写入关节位置（默认位姿 + 噪声）
        self.data.qpos[self.qpos_adr] = DEFAULT_ROBOT_POS + joint_noise
        # 清空全部速度
        self.data.qvel[:] = 0.0
        # 写入机身速度
        self.data.qvel[0:6] = base_vel
        # 写入关节速度
        self.data.qvel[self.qvel_adr] = joint_vel

        # 清空动作与力矩
        self.last_action.fill(0.0)
        self.input_torque.fill(0.0)
        # 前向动力学
        mujoco.mj_forward(self.model, self.data)
        # 物理沉降（让机器人稳定落地）
        self._settle_after_reset()
        # 重置接触/腾空状态
        self.wheel_contact_mask.fill(False)
        self.wheel_air_time.fill(0.0)
        # 测量轮子标称离地高度
        self.nominal_wheel_clearance = self._wheel_clearance_values()
        # 采样重置外力
        self._sample_reset_wrench()
        # 安排第一次推力
        self._schedule_next_push()
        # 记录上一步关节速度
        self.prev_joint_vel_robot = self.data.qvel[self.qvel_adr].copy()
        # 重置计数
        self.step_count = 0
        self.episode_return = 0.0
        self.prev_base_x = float(self.data.qpos[0])
        self.undesired_contact_violation_steps = 0
        # 采样速度指令
        self._sample_command()
        mujoco.mj_forward(self.model, self.data)
        # 返回初始观测
        return self._get_obs()

    # 环境步进：执行动作 -> 积分物理 -> 计算奖励/终止 -> 返回 (obs, reward, done, info)
    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict[str, Any]]:
        # 动作转 float64 并整形
        action = np.asarray(action, dtype=np.float64).reshape(ACTION_DIM)
        # 裁剪动作
        action = np.clip(action, -self.cfg.action_clip, self.cfg.action_clip)

        # 记录步进前的状态（用于奖励计算）
        prev_x = float(self.data.qpos[0])
        prev_action = self.last_action.copy()
        prev_joint_vel_robot = self.data.qvel[self.qvel_adr].copy()
        # 把策略动作转换为机器人关节目标（位置/速度）
        target_pos_robot, target_vel_robot = self._policy_action_to_robot_targets(action)

        # 在每个仿真子步内应用 PD 力矩并步进
        for _ in range(self.control_decimation):
            self._apply_external_wrench()
            self._apply_pd_torque(target_pos_robot, target_vel_robot)
            mujoco.mj_step(self.model, self.data)
            self._advance_external_wrench()

        # 步数 +1
        self.step_count += 1
        # 到达推步时机则施加推力
        self._maybe_push_robot()
        # 记录本次动作
        self.last_action = action.copy()
        # 定时重新采样指令
        if self.step_count % self.command_resample_steps == 0:
            self._sample_command()

        # 计算观测与奖励
        obs = self._get_obs()
        reward, reward_terms = self._compute_reward(action, prev_action, prev_x, prev_joint_vel_robot)
        self.prev_joint_vel_robot = self.data.qvel[self.qvel_adr].copy()
        # 统计非轮子接触数量
        undesired_contact_count = self._undesired_contact_count_from_reward_terms(reward_terms)
        if self.cfg.terminate_on_undesired_contact and (
            undesired_contact_count > self.cfg.undesired_contact_termination_threshold
        ):
            # 累计违规步数
            self.undesired_contact_violation_steps += 1
        else:
            # 清零违规计数
            self.undesired_contact_violation_steps = 0

        # 是否因持续非轮子接触而终止
        undesired_contact_terminated = (
            self.cfg.terminate_on_undesired_contact
            and self.undesired_contact_violation_steps
            >= max(1, self.cfg.undesired_contact_termination_steps)
        )
        if undesired_contact_terminated:
            # 施加终止惩罚
            reward += self.cfg.undesired_contact_terminal_penalty
            reward_terms["undesired_contact_terminal"] = self.cfg.undesired_contact_terminal_penalty

        # 终止（摔倒/越界/接触违规）或截断（回合时长到）
        terminated = self._is_terminated(obs) or undesired_contact_terminated
        truncated = self.step_count >= self.max_episode_steps
        done = bool(terminated or truncated)
        # 累计回报
        self.episode_return += reward
        # 组装 info 字典
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

    # 从奖励项反推非轮子接触数量（用权重除回）
    def _undesired_contact_count_from_reward_terms(self, reward_terms: dict[str, float]) -> float:
        weight = self.cfg.reward.undesired_contact_weight
        # 权重为 0 时无法反推，直接返回 0
        if abs(weight) < 1.0e-8:
            return 0.0
        return max(0.0, float(reward_terms.get("undesired_contacts", 0.0)) / weight)

    # 策略动作 -> 机器人关节目标（位置 + 速度）
    def _policy_action_to_robot_targets(self, action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # 从默认位姿出发（策略序）
        pos_policy = DEFAULT_POLICY_POS.copy()
        vel_policy = np.zeros(ACTION_DIM, dtype=np.float64)

        # 腿部动作叠加到默认位置（按缩放系数）
        pos_policy[:12] += action[:12] * ACTION_SCALE_POLICY[:12]
        # 轮子动作作为目标角速度（按缩放系数）
        vel_policy[WHEEL_POLICY_INDICES] = action[WHEEL_POLICY_INDICES] * ACTION_SCALE_POLICY[WHEEL_POLICY_INDICES]

        # 重排为机器人序后返回
        return pos_policy[POLICY_TO_ROBOT], vel_policy[POLICY_TO_ROBOT]

    # 应用 PD 力矩：力矩 = kp*(目标位置-位置) + kd*(目标速度-速度)
    def _apply_pd_torque(self, target_pos_robot: np.ndarray, target_vel_robot: np.ndarray) -> None:
        # 当前关节位置/速度（机器人序）
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.qvel_adr]
        # PD 控制律
        torque = self.kp_robot * (target_pos_robot - q) + self.kd_robot * (target_vel_robot - dq)
        # 清理非有限值
        torque = np.nan_to_num(torque, nan=0.0, posinf=0.0, neginf=0.0)
        # 裁剪到执行器范围
        torque = np.clip(torque, self.ctrl_low, self.ctrl_high)
        self.input_torque = torque
        # 写入执行器控制量
        self.data.ctrl[self.actuator_ids] = torque

    # 基础观测（57 维）：角速度/投影重力/指令/关节位置差/关节速度/上一步动作
    def _get_base_obs(self) -> np.ndarray:
        # 关节位置/速度
        q = self.data.qpos[self.qpos_adr]
        dq = self.data.qvel[self.qvel_adr]
        # 机身姿态四元数
        quat = self.data.qpos[3:7].copy()

        # 机身角速度（缩放 0.25）
        base_ang_vel = self._base_ang_vel() * 0.25
        # 机体系中投影重力（判断姿态）
        projected_gravity = quat_rotate_inverse(quat, GRAVITY_VEC)

        # 关节位置转为策略序，轮子位置置 0（无意义）
        joint_pos_policy = q[ROBOT_TO_POLICY]
        joint_pos_policy[WHEEL_POLICY_INDICES] = 0.0
        # 相对默认位姿的关节位置差
        joint_pos_rel = joint_pos_policy - DEFAULT_POLICY_POS
        # 关节速度转策略序并缩放（0.05）
        joint_vel_policy = dq[ROBOT_TO_POLICY] * 0.05

        # 拼接全部观测分量
        obs = np.concatenate(
            [
                base_ang_vel,          # 3 维：机身角速度
                projected_gravity,     # 3 维：投影重力
                self.command,          # 3 维：速度指令
                joint_pos_rel,         # 16 维：关节位置差
                joint_vel_policy,      # 16 维：关节速度
                self.last_action,      # 16 维：上一步动作
            ]
        ).astype(np.float32)
        # 维度校验
        if obs.shape[0] != OBS_DIM:
            raise RuntimeError(f"Observation dim mismatch: {obs.shape[0]} != {OBS_DIM}")
        return obs

    # 完整观测：基础观测 +（可选）高度扫描
    def _get_obs(self) -> np.ndarray:
        parts = [self._get_base_obs()]
        # 可选追加高度扫描
        if self.cfg.include_height_scan:
            parts.append(self._height_scan())
        obs = np.concatenate(parts).astype(np.float32)
        # 维度校验
        if obs.shape[0] != self.observation_dim:
            raise RuntimeError(f"Observation dim mismatch: {obs.shape[0]} != {self.observation_dim}")
        return obs

    # 特权 critic 观测：基础观测 +（可选）机身线速度 +（可选）高度扫描
    def get_critic_obs(self) -> np.ndarray:
        parts = [self._get_base_obs()]
        # 可选机身线速度（机体系）
        if self.cfg.critic_base_lin_vel:
            parts.append(self._base_lin_vel_body().astype(np.float32))
        # 可选高度扫描
        if self.cfg.critic_height_scan:
            parts.append(self._height_scan())
        return np.concatenate(parts).astype(np.float32)

    # 生成高度扫描网格偏移（机体系 xy 网格）
    def _make_height_scan_offsets(self) -> np.ndarray:
        scan_cfg = self.cfg.height_scan
        # 分辨率/半长/半宽
        resolution = max(float(scan_cfg.resolution), 1.0e-6)
        half_x = max(float(scan_cfg.size_x), 0.0) * 0.5
        half_y = max(float(scan_cfg.size_y), 0.0) * 0.5
        # 生成 x/y 坐标网格
        xs = np.arange(-half_x, half_x + 0.5 * resolution, resolution, dtype=np.float64)
        ys = np.arange(-half_y, half_y + 0.5 * resolution, resolution, dtype=np.float64)
        if xs.size == 0:
            xs = np.array([0.0], dtype=np.float64)
        if ys.size == 0:
            ys = np.array([0.0], dtype=np.float64)
        # 笛卡尔网格并展平为 (N, 2) 偏移
        grid_x, grid_y = np.meshgrid(xs, ys, indexing="xy")
        return np.stack([grid_x.reshape(-1), grid_y.reshape(-1)], axis=1)

    # 前方高度扫描掩码：只保留机身前方一定区域内的网格点（用于抬轮奖励）
    def _make_front_height_scan_mask(self) -> np.ndarray:
        # 无扫描网格则返回空
        if self.height_scan_dim == 0:
            return np.zeros(0, dtype=bool)
        offsets = self.height_scan_offsets
        # 前方区域：x 在 [0.05, 0.75]，|y| <= 0.45
        return (
            (offsets[:, 0] >= 0.05)
            & (offsets[:, 0] <= 0.75)
            & (np.abs(offsets[:, 1]) <= 0.45)
        )

    # 高度扫描：以机身为中心、按航向旋转网格，向下射线测地形相对高度
    def _height_scan(self) -> np.ndarray:
        """Yaw-aligned terrain heights around the base for the privileged critic."""

        # 无扫描网格则返回空
        if self.height_scan_dim == 0:
            return np.zeros(0, dtype=np.float32)

        # 机身位置与航向
        base_x = float(self.data.qpos[0])
        base_y = float(self.data.qpos[1])
        base_z = float(self.data.qpos[2])
        yaw = self._base_yaw()
        c = float(np.cos(yaw))
        s = float(np.sin(yaw))
        # 机身正下方地面高度（作为基准）
        local_ground = self._terrain_height_at(base_x, base_y, base_z)
        heights = np.empty(self.height_scan_dim, dtype=np.float32)
        # 射线起点 z 偏移
        z_offset = float(self.cfg.height_scan.z_offset)

        # 逐网格点：机体系偏移 -> 世界系坐标 -> 测地面高度差
        for idx, (offset_x, offset_y) in enumerate(self.height_scan_offsets):
            world_x = base_x + c * float(offset_x) - s * float(offset_y)
            world_y = base_y + s * float(offset_x) + c * float(offset_y)
            terrain_height = self._terrain_height_at(world_x, world_y, base_z, z_offset=z_offset)
            heights[idx] = float(terrain_height - local_ground)

        # 可选高度裁剪
        clip = float(self.cfg.height_scan.clip)
        if clip > 0.0:
            heights = np.clip(heights, -clip, clip)
        return heights.astype(np.float32, copy=False)

    # 计算单步奖励：返回 (总奖励, 各奖励项字典)
    def _compute_reward(
        self,
        action: np.ndarray,
        prev_action: np.ndarray,
        prev_x: float,
        prev_joint_vel_robot: np.ndarray,
    ) -> tuple[float, dict[str, float]]:
        reward_cfg = self.cfg.reward
        # 基础状态量：姿态、速度、投影重力、关节状态
        quat = self.data.qpos[3:7].copy()
        base_lin_vel_body = quat_rotate_inverse(quat, self.data.qvel[0:3])
        base_ang_vel = self._base_ang_vel()
        projected_gravity = quat_rotate_inverse(quat, GRAVITY_VEC)
        q_robot = self.data.qpos[self.qpos_adr]
        dq_robot = self.data.qvel[self.qvel_adr]
        q_policy = q_robot[ROBOT_TO_POLICY]
        joint_pos_rel_policy = q_policy - DEFAULT_POLICY_POS

        # 速度跟踪：线速度/角速度与指令的平方误差，指数化为 [0,1] 奖励
        lin_err = np.sum((base_lin_vel_body[:2] - self.command[:2]) ** 2)
        yaw_err = (base_ang_vel[2] - self.command[2]) ** 2
        track_lin = float(np.exp(-lin_err / 0.25))
        track_yaw = float(np.exp(-yaw_err / 0.25))

        # 各项惩罚量
        z_vel_penalty = float(self.data.qvel[2] ** 2)                      # z 向速度
        orientation_penalty = float(np.sum(projected_gravity[:2] ** 2))    # 机身倾斜
        terrain_height = self._terrain_height_below_base()                 # 机身下地面高度
        base_height = float(self.data.qpos[2] - terrain_height)            # 离地高度
        height_penalty = float((base_height - self.cfg.target_base_height) ** 2)  # 高度偏差
        torque_penalty = float(np.mean(self.input_torque ** 2))            # 力矩
        action_rate_penalty = float(np.mean((action - prev_action) ** 2))  # 动作变化率
        leg_action_penalty = float(np.mean(action[LEG_POLICY_INDICES] ** 2))    # 腿部动作幅度
        wheel_action_penalty = float(np.mean(action[WHEEL_POLICY_INDICES] ** 2)) # 轮子动作幅度
        saturation_threshold = reward_cfg.action_saturation_threshold
        # 动作饱和惩罚（超出阈值部分）
        action_saturation_penalty = float(np.mean(np.maximum(np.abs(action) - saturation_threshold, 0.0) ** 2))
        # 前向位移进步（单位时间 x 增量）
        progress = float((self.data.qpos[0] - prev_x) / self.cfg.control_dt)
        # 地形高度上升进步（爬坡/上台阶）
        prev_terrain_height = self._terrain_height_at(prev_x, self.data.qpos[1], self.data.qpos[2])
        terrain_height_progress = float(max(terrain_height - prev_terrain_height, 0.0) / self.cfg.control_dt)
        # 处于高处地形（台阶上）的奖励
        stair_height = float(max(terrain_height, 0.0))
        # 台阶上前进奖励
        stair_forward_progress = self._stair_forward_progress(progress, terrain_height)
        # 关节加速度惩罚（腿/轮分开）
        joint_acc = (dq_robot - prev_joint_vel_robot) / self.cfg.control_dt
        joint_acc_penalty = float(np.mean(joint_acc[self.leg_robot_indices] ** 2))
        wheel_acc_penalty = float(np.mean(joint_acc[WHEEL_ROBOT_INDICES] ** 2))
        # 关节位置越限惩罚
        joint_limit_penalty = self._joint_pos_limit_penalty(q_robot)
        # 关节功率惩罚（力矩 x 速度）
        leg_power = dq_robot[self.leg_robot_indices] * self.input_torque[self.leg_robot_indices]
        joint_power_penalty = float(np.mean(np.abs(leg_power)))

        # 静止指令下的站立惩罚（腿应收拢、速度应小）
        command_norm = float(np.linalg.norm(self.command))
        base_xy_speed = float(np.linalg.norm(base_lin_vel_body[:2]))
        stand_still_penalty = 0.0
        feet_contact_without_cmd = 0.0
        if command_norm < reward_cfg.stand_still_command_threshold:
            stand_still_penalty = float(np.mean(joint_pos_rel_policy[LEG_POLICY_INDICES] ** 2))
            stand_still_penalty += float(
                max(base_xy_speed - reward_cfg.stand_still_velocity_threshold, 0.0) ** 2
            )

        # 各关节偏离默认位姿惩罚（hipx/hipy/knee 分开）
        hipx_penalty = float(np.mean(np.abs(joint_pos_rel_policy[HIPX_POLICY_INDICES])))
        hipy_penalty = float(np.mean(np.abs(joint_pos_rel_policy[HIPY_POLICY_INDICES])))
        knee_penalty = float(np.mean(np.abs(joint_pos_rel_policy[KNEE_POLICY_INDICES])))
        # 左右腿镜像对称惩罚（fl<->hr、fr<->hl 位置差）
        fl_rel = joint_pos_rel_policy[0:3]
        fr_rel = joint_pos_rel_policy[3:6]
        hl_rel = joint_pos_rel_policy[6:9]
        hr_rel = joint_pos_rel_policy[9:12]
        mirror_penalty = float(np.mean((fl_rel - hr_rel) ** 2) + np.mean((fr_rel - hl_rel) ** 2))
        # 接触相关奖励项
        (
            undesired_contacts,
            contact_force_excess,
            wheel_contact_fraction,
            wheel_contact_mask,
            wheel_stumble_count,
        ) = self._contact_reward_terms()
        # 轮子腾空/抬升奖励
        wheel_air_time = self._wheel_air_time_reward(wheel_contact_mask, command_norm)
        wheel_clearance = self._wheel_clearance_reward(command_norm)
        # 静止指令下：轮子着地视为违规
        if command_norm < reward_cfg.stand_still_command_threshold:
            feet_contact_without_cmd = wheel_contact_fraction
        # 机身朝上程度
        upward = float(np.clip(-projected_gravity[2], 0.0, 1.0))

        # 汇总全部奖励项（权重 x 原始量）
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
        # 总奖励 = 各项之和
        return float(sum(reward_terms.values())), reward_terms

    # 台阶上前进奖励：仅当机身已在台阶地形且高度足够时给前进奖励
    def _stair_forward_progress(self, progress: float, terrain_height: float) -> float:
        # 目标台阶高度（无则无此项）
        target_height = float(getattr(self.cfg.terrain, "stair_height", 0.0))
        if target_height <= 0.0:
            return 0.0
        # 触发所需的最低高度
        min_height = target_height * float(self.cfg.reward.stair_forward_progress_height_fraction)
        if terrain_height < min_height:
            return 0.0
        # 在台阶上：正的前进速度作为奖励
        return float(max(progress, 0.0))

    # 关节位置越限惩罚：超出软边界（限位内侧留余量）的部分平方
    def _joint_pos_limit_penalty(self, q_robot: np.ndarray) -> float:
        limited = self.joint_limited
        # 没有限位关节则无惩罚
        if not np.any(limited):
            return 0.0

        # 软边界 = 限位内侧留 margin 比例
        span = self.joint_range_high - self.joint_range_low
        margin = np.maximum(span * self.cfg.reward.joint_limit_margin_ratio, 0.0)
        soft_low = self.joint_range_low + margin
        soft_high = self.joint_range_high - margin
        # 上下越限量
        lower_violation = np.maximum(soft_low - q_robot, 0.0)
        upper_violation = np.maximum(q_robot - soft_high, 0.0)
        violation = lower_violation + upper_violation
        # 只统计有限位关节
        return float(np.mean(violation[limited] ** 2))

    # 接触奖励项：非轮子接触数、轮子接触力超限、轮子接触比例、轮子绊倒计数
    def _contact_reward_terms(self) -> tuple[float, float, float, np.ndarray, float]:
        undesired_contacts = 0.0
        contact_force_excess = 0.0
        wheel_bodies_in_contact: set[int] = set()
        wheel_contact_mask = np.zeros(len(self.wheel_body_ids), dtype=bool)
        wheel_stumble_count = 0.0

        # 遍历所有活动接触
        for contact_idx in range(self.data.ncon):
            contact = self.data.contact[contact_idx]
            # 接触双方 body
            body1 = int(self.model.geom_bodyid[contact.geom1])
            body2 = int(self.model.geom_bodyid[contact.geom2])
            body1_robot = body1 in self.robot_body_id_set
            body2_robot = body2 in self.robot_body_id_set
            # 与机器人无关的接触跳过
            if not (body1_robot or body2_robot):
                continue

            # 是否涉及轮子
            body1_wheel = body1 in self.wheel_body_id_set
            body2_wheel = body2 in self.wheel_body_id_set
            if body1_wheel or body2_wheel:
                # 记录接触轮子
                if body1_wheel:
                    wheel_bodies_in_contact.add(body1)
                    wheel_contact_mask[self.wheel_body_to_index[body1]] = True
                if body2_wheel:
                    wheel_bodies_in_contact.add(body2)
                    wheel_contact_mask[self.wheel_body_to_index[body2]] = True
                # 计算接触力
                mujoco.mj_contactForce(self.model, self.data, contact_idx, self.contact_force_buffer)
                force_norm = float(np.linalg.norm(self.contact_force_buffer[:3]))
                if np.isfinite(force_norm):
                    # 超过阈值的力计入惩罚
                    contact_force_excess += max(force_norm - self.cfg.reward.contact_force_threshold, 0.0)
                    # 接触法线 z 分量小（垂直面碰撞）且力大 -> 轮子绊倒
                    normal = np.asarray(contact.frame[:3], dtype=np.float64)
                    normal_z = abs(float(normal[2]))
                    if (
                        normal_z < self.cfg.reward.wheel_stumble_normal_z_threshold
                        and force_norm > self.cfg.reward.wheel_stumble_force_threshold
                    ):
                        wheel_stumble_count += 1.0
            else:
                # 非轮子机器人部件接触（不应发生）
                undesired_contacts += 1.0

        # 接触轮子占全部轮子比例
        wheel_contact_fraction = len(wheel_bodies_in_contact) / max(1, len(self.wheel_body_id_set))
        return (
            float(undesired_contacts),
            float(contact_force_excess),
            float(wheel_contact_fraction),
            wheel_contact_mask,
            float(wheel_stumble_count),
        )

    # 轮子腾空时间奖励：落地瞬间按腾空超时量给奖励
    def _wheel_air_time_reward(self, wheel_contact_mask: np.ndarray, command_norm: float) -> float:
        # 本轮新接触的轮子（此前腾空、本轮落地）
        first_contact = wheel_contact_mask & ~self.wheel_contact_mask
        if command_norm > self.cfg.reward.stand_still_command_threshold:
            # 腾空时间超出阈值的部分作为奖励（鼓励跳跃越障）
            excess_air_time = np.maximum(self.wheel_air_time - self.cfg.reward.wheel_air_time_threshold, 0.0)
            reward = float(np.sum(excess_air_time * first_contact.astype(np.float64)))
        else:
            reward = 0.0
        # 更新腾空时间：着地清零，否则累加控制步长
        self.wheel_air_time = np.where(wheel_contact_mask, 0.0, self.wheel_air_time + self.cfg.control_dt)
        self.wheel_contact_mask = wheel_contact_mask.copy()
        return reward

    # 轮子抬升奖励：前方地形升高时，鼓励轮子相对标称离地高度抬升
    def _wheel_clearance_reward(self, command_norm: float) -> float:
        # 静止指令下不计算
        if command_norm <= self.cfg.reward.stand_still_command_threshold:
            return 0.0
        # 前方地形高度差不足时不计算
        terrain_delta = self._front_terrain_delta()
        if terrain_delta < self.cfg.reward.wheel_clearance_terrain_threshold:
            return 0.0
        # 抬升目标（防除零）
        target = max(float(self.cfg.reward.wheel_clearance_lift_target), 1.0e-6)
        # 当前离地高度相对标称值的抬升量
        clearance_delta = self._wheel_clearance_values() - self.nominal_wheel_clearance
        # 归一化到 [0,1] 的抬升奖励
        return float(np.mean(np.clip(clearance_delta / target, 0.0, 1.0)))

    # 前方地形最大高度差（用于轮子抬升奖励触发判断）
    def _front_terrain_delta(self) -> float:
        if self.height_scan_dim == 0:
            return 0.0
        heights = self._height_scan()
        # 只取前方区域的网格
        if self.front_height_scan_mask.size == heights.size and np.any(self.front_height_scan_mask):
            heights = heights[self.front_height_scan_mask]
        # 返回最大正值高度差
        return float(max(np.max(heights), 0.0))

    # 各轮子当前离地高度（米）
    def _wheel_clearance_values(self) -> np.ndarray:
        clearances = np.zeros(len(self.wheel_body_ids), dtype=np.float64)
        for idx, body_id in enumerate(self.wheel_body_ids):
            pos = self.data.xpos[int(body_id)]
            terrain_height = self._terrain_height_at(float(pos[0]), float(pos[1]), float(pos[2]))
            clearances[idx] = float(pos[2] - terrain_height)
        return clearances

    # 机身正下方地面高度（类似 Isaac Lab 的 height_scanner_base）
    def _terrain_height_below_base(self) -> float:
        """Estimate local ground height under the base, like Isaac Lab's height_scanner_base."""

        return self._terrain_height_at(self.data.qpos[0], self.data.qpos[1], self.data.qpos[2])

    # 用向下的射线测量 (x, y) 处地面高度（射线从 z_reference + z_offset 向下打）
    def _terrain_height_at(
        self,
        x: float,
        y: float,
        z_reference: float,
        z_offset: float = 2.0,
    ) -> float:
        # 射线起点
        origin = np.array(
            [x, y, z_reference + z_offset],
            dtype=np.float64,
        )
        # 几何体 ID 输出缓冲区（-1 占位）
        geomid = np.array([-1], dtype=np.int32)
        # 执行射线检测：只与 terrain 组几何体碰撞，忽略机身
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
        # 未命中或距离无效则返回 0
        if distance < 0.0 or not np.isfinite(distance):
            return 0.0
        # 地面高度 = 起点 z - 射线距离
        return float(origin[2] - distance)

    # 随机化物理属性：连杆质量/惯量、机身质量/质心、摩擦、PD 增益
    def _randomize_physics(self) -> None:
        # 先恢复标称值
        self._restore_nominal_model()
        self.kp_robot = KP_ROBOT.copy()
        self.kd_robot = KD_ROBOT.copy()
        self.randomization_info = {"enabled": bool(self.cfg.randomization.enabled)}
        # 未启用随机化：更新常量后直接返回
        if not self.cfg.randomization.enabled:
            mujoco.mj_setConst(self.model, self.data)
            return

        rand_cfg = self.cfg.randomization
        # 连杆质量缩放（乘性，作用于质量与惯量）
        link_scales = self._uniform_array(rand_cfg.link_mass_scale_range, len(self.link_body_ids))
        for body_id, scale in zip(self.link_body_ids, link_scales):
            body_idx = int(body_id)
            self.model.body_mass[body_idx] = self.nominal_body_mass[body_idx] * scale
            self.model.body_inertia[body_idx] = self.nominal_body_inertia[body_idx] * scale

        # 机身质量附加量（加性），惯量按质量比例缩放
        base_mass_add = self._uniform(rand_cfg.base_mass_add_range)
        nominal_base_mass = float(self.nominal_body_mass[self.base_body_id])
        base_mass = max(0.1, nominal_base_mass + base_mass_add)
        base_scale = base_mass / max(nominal_base_mass, 1.0e-8)
        self.model.body_mass[self.base_body_id] = base_mass
        self.model.body_inertia[self.base_body_id] = self.nominal_body_inertia[self.base_body_id] * base_scale

        # 机身质心偏移
        com_offset = np.array(
            [
                self._uniform(rand_cfg.base_com_x_range),
                self._uniform(rand_cfg.base_com_y_range),
                self._uniform(rand_cfg.base_com_z_range),
            ],
            dtype=np.float64,
        )
        self.model.body_ipos[self.base_body_id] = self.nominal_body_ipos[self.base_body_id] + com_offset

        # 摩擦系数随机化（按标称值等比例调整各向摩擦）
        friction = self._uniform(rand_cfg.friction_range)
        if len(self.randomized_geom_ids) > 0:
            nominal_friction = self.nominal_geom_friction[self.randomized_geom_ids]
            scale = friction / np.maximum(nominal_friction[:, 0], 1.0e-6)
            self.model.geom_friction[self.randomized_geom_ids] = nominal_friction * scale[:, None]
            self.model.geom_friction[self.randomized_geom_ids, 0] = friction
            # 切向摩擦设下限避免数值问题
            self.model.geom_friction[self.randomized_geom_ids, 1:] = np.maximum(
                self.model.geom_friction[self.randomized_geom_ids, 1:],
                1.0e-5,
            )

        # PD 增益随机化（乘性）
        kp_scale = self._uniform_array(rand_cfg.kp_scale_range, ACTION_DIM)
        kd_scale = self._uniform_array(rand_cfg.kd_scale_range, ACTION_DIM)
        self.kp_robot = KP_ROBOT * kp_scale
        self.kd_robot = KD_ROBOT * kd_scale
        mujoco.mj_setConst(self.model, self.data)

        # 记录随机化信息（供 info 输出）
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

    # 恢复模型标称物理参数
    def _restore_nominal_model(self) -> None:
        self.model.body_mass[:] = self.nominal_body_mass
        self.model.body_inertia[:] = self.nominal_body_inertia
        self.model.body_ipos[:] = self.nominal_body_ipos
        self.model.geom_friction[:] = self.nominal_geom_friction

    # 采样初始机身状态（位置/姿态/速度），未启用随机化时返回固定值
    def _sample_initial_base_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # 未启用随机化：固定初始状态
        if not self.cfg.randomization.enabled:
            return (
                np.array([self.cfg.base_init_x, self.cfg.base_init_y, self.cfg.base_init_height], dtype=np.float64),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                np.zeros(6, dtype=np.float64),
            )

        rand_cfg = self.cfg.randomization
        # 随机位置
        pos = np.array(
            [
                self.cfg.base_init_x + self._uniform(rand_cfg.base_x_range),
                self.cfg.base_init_y + self._uniform(rand_cfg.base_y_range),
                self.cfg.base_init_height + self._uniform(rand_cfg.base_z_range),
            ],
            dtype=np.float64,
        )
        # 随机姿态（欧拉角 -> 四元数）
        quat = self._quat_from_euler(
            self._uniform(rand_cfg.base_roll_range),
            self._uniform(rand_cfg.base_pitch_range),
            self._uniform(rand_cfg.base_yaw_range),
        )
        # 随机速度（线速度 xyz + 角速度 xyz）
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

    # 采样重置外力/力矩（wrench）及其持续时间
    def _sample_reset_wrench(self) -> None:
        rand_cfg = self.cfg.randomization
        self.external_wrench[:] = 0.0
        self.reset_wrench_steps_remaining = 0
        # 未启用或关闭 wrench 则跳过
        if not (rand_cfg.enabled and rand_cfg.reset_wrench_enabled):
            return

        # 随机力（前 3 维）与力矩（后 3 维）
        self.external_wrench[:3] = self._uniform_array(rand_cfg.reset_force_range, 3)
        self.external_wrench[3:] = self._uniform_array(rand_cfg.reset_torque_range, 3)
        # 随机持续时间（转成仿真步数）
        duration = self._uniform(rand_cfg.reset_wrench_duration_range)
        self.reset_wrench_steps_remaining = max(1, int(round(duration / self.cfg.sim_dt)))
        self.randomization_info["reset_wrench"] = self.external_wrench.tolist()

    # 重置后的物理沉降：用默认关节目标 PD 控制跑若干步，让机器人稳定落地
    def _settle_after_reset(self) -> None:
        # 沉降步数
        settle_steps = max(0, int(round(self.cfg.reset_settle_seconds / self.cfg.sim_dt)))
        if settle_steps == 0:
            return

        # 以默认位姿为目标进行 PD 控制沉降
        target_pos_robot = DEFAULT_ROBOT_POS.copy()
        target_vel_robot = np.zeros(ACTION_DIM, dtype=np.float64)
        for _ in range(settle_steps):
            self._apply_pd_torque(target_pos_robot, target_vel_robot)
            mujoco.mj_step(self.model, self.data)

        # 沉降后清零速度与力矩
        self.data.qvel[:] = 0.0
        self.input_torque.fill(0.0)
        self.data.ctrl[self.actuator_ids] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # 应用外部力/力矩（重置 wrench 未结束则施加）
    def _apply_external_wrench(self) -> None:
        if self.reset_wrench_steps_remaining > 0:
            self.data.xfrc_applied[self.base_body_id] = self.external_wrench
        else:
            self.data.xfrc_applied[self.base_body_id] = ZERO_WRENCH

    # 推进 wrench 计时：到期后清零施加量
    def _advance_external_wrench(self) -> None:
        if self.reset_wrench_steps_remaining > 0:
            self.reset_wrench_steps_remaining -= 1
            if self.reset_wrench_steps_remaining == 0:
                self.data.xfrc_applied[self.base_body_id] = ZERO_WRENCH

    # 安排下一次推力扰动的步数
    def _schedule_next_push(self) -> None:
        rand_cfg = self.cfg.randomization
        # 未启用或关闭推力：设为不可能触发
        if not (rand_cfg.enabled and rand_cfg.push_enabled):
            self.next_push_step = self.max_episode_steps + 1
            return
        # 随机间隔（秒转步数）
        interval_s = self._uniform(rand_cfg.push_interval_range_s)
        self.next_push_step = self.step_count + max(1, int(round(interval_s / self.cfg.control_dt)))

    # 到推步时机则给机身加随机水平速度
    def _maybe_push_robot(self) -> None:
        rand_cfg = self.cfg.randomization
        if not (rand_cfg.enabled and rand_cfg.push_enabled):
            return
        if self.step_count < self.next_push_step:
            return
        # 随机水平速度增量
        push_xy = self._uniform_array(rand_cfg.push_lin_vel_range, 2)
        self.data.qvel[0:2] += push_xy
        self.randomization_info["last_push_xy"] = push_xy.tolist()
        # 安排下一次
        self._schedule_next_push()

    # 获取参与接触的几何体 ID 列表（contype 或 conaffinity 非零）
    def _randomized_geom_ids(self) -> np.ndarray:
        geom_ids: list[int] = []
        for geom_id in range(self.model.ngeom):
            has_contact = bool(self.model.geom_contype[geom_id] or self.model.geom_conaffinity[geom_id])
            if has_contact:
                geom_ids.append(geom_id)
        return np.array(geom_ids, dtype=np.int32)

    # 在范围内取一个均匀随机数（范围退化时返回下限）
    def _uniform(self, value_range: tuple[float, float]) -> float:
        low, high = value_range
        if high <= low:
            return float(low)
        return float(self.rng.uniform(low, high))

    # 在范围内取一个均匀随机数组（范围退化时返回全下限）
    def _uniform_array(self, value_range: tuple[float, float], size: int) -> np.ndarray:
        low, high = value_range
        if high <= low:
            return np.full(size, low, dtype=np.float64)
        return self.rng.uniform(low, high, size=size).astype(np.float64)

    # 欧拉角 -> 四元数（wxyz，静态方法）
    @staticmethod
    def _quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
        # 各轴半角余弦/正弦
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        # 欧拉角转四元数（z-y-x 顺序）
        quat = np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=np.float64,
        )
        # 归一化
        return quat / max(np.linalg.norm(quat), 1.0e-8)

    # 提取机身航向角 yaw（四元数 -> 欧拉角 z 分量）
    def _base_yaw(self) -> float:
        quat = self.data.qpos[3:7]
        w, x, y, z = (float(v) for v in quat)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    # 终止判断：机身过低 / 姿态翻转 / 越界
    def _is_terminated(self, obs: np.ndarray) -> bool:
        base_z = float(self.data.qpos[2])
        projected_gravity_z = float(obs[5])
        # 机身过低（摔倒）
        too_low = base_z < self.cfg.terminate_base_height
        # 姿态翻转（投影重力 z 朝上）
        bad_orientation = projected_gravity_z > self.cfg.terminate_projected_gravity_z
        # 越出边界
        out_of_bounds = abs(float(self.data.qpos[1])) > 3.0 or float(self.data.qpos[0]) < -1.0
        return bool(too_low or bad_orientation or out_of_bounds)

    # 采样速度指令（forward 模式只给前向速度，random 模式给横向+转向）
    def _sample_command(self) -> None:
        if self.cfg.command_mode == "forward":
            # 仅前向
            self.command[0] = self.rng.uniform(*self.cfg.forward_command_range)
            self.command[1] = 0.0
            self.command[2] = 0.0
        elif self.cfg.command_mode == "random":
            # 前向 + 横向 + 转向
            self.command[0] = self.rng.uniform(*self.cfg.forward_command_range)
            self.command[1] = self.rng.uniform(*self.cfg.lateral_command_range)
            self.command[2] = self.rng.uniform(*self.cfg.yaw_command_range)
        else:
            raise ValueError(f"Unsupported command_mode: {self.cfg.command_mode}")

    # 机身角速度（优先使用陀螺仪传感器，否则由四元数反算）
    def _base_ang_vel(self) -> np.ndarray:
        if self.gyro_sensor is not None:
            adr, dim = self.gyro_sensor
            return self.data.sensordata[adr: adr + dim].copy()
        quat = self.data.qpos[3:7].copy()
        return quat_rotate_inverse(quat, self.data.qvel[3:6])

    # 机身线速度（机体系）
    def _base_lin_vel_body(self) -> np.ndarray:
        quat = self.data.qpos[3:7].copy()
        return quat_rotate_inverse(quat, self.data.qvel[0:3])

    # 传感器切片（返回 (地址, 维度)；传感器不存在返回 None）
    def _sensor_slice(self, name: str) -> tuple[int, int] | None:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            return None
        return int(self.model.sensor_adr[sid]), int(self.model.sensor_dim[sid])

    # 按名称查关节 ID（找不到抛异常）
    def _joint_id(self, name: str) -> int:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise KeyError(f"Joint not found in MuJoCo model: {name}")
        return int(jid)

    # 按名称查执行器 ID（找不到抛异常）
    def _actuator_id(self, name: str) -> int:
        aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise KeyError(f"Actuator not found in MuJoCo model: {name}")
        return int(aid)

    # 按名称查 body ID（找不到抛异常）
    def _body_id(self, name: str) -> int:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise KeyError(f"Body not found in MuJoCo model: {name}")
        return int(bid)
