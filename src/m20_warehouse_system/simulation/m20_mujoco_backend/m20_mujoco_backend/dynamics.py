# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ============================================================
# 文件：dynamics.py
# 用途：M20 MuJoCo 仿真后端与单元测试共用的「纯数值工具」模块。
#       - 定义官方 SDK 关节坐标系 ↔ MuJoCo 原始坐标系的映射常量
#         （方向符号、位置偏移、速度上限、轮/腿索引、初始姿态）；
#       - 提供四元数、RPY 欧拉角、世界系→机体系向量旋转等数学工具；
#       - 实现 SDK PD 力矩律（含非有限值保护与执行器限幅）与
#         驻车制动（仅对四个轮子的速度目标施加物理阻尼）。
#       本模块不依赖 ROS 与 MuJoCo 引擎，可独立进行单元测试。
# ============================================================

"""Pure numerical helpers shared by the MuJoCo backend and its tests."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np

# ==================== 关节常量定义 ====================
# JOINT_NAMES：16 个关节名，顺序为四腿（fl/fr/hl/hr）
#              × 四个关节（髋横滚 hipx / 髋俯仰 hipy / 膝 knee / 轮 wheel），
#              与 MJCF 模型 <actuator> 中的电机顺序一一对应。
JOINT_NAMES = (
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint',
)
# JOINT_DIR：关节方向符号（±1）。SDK 坐标乘以此符号后转为
#            MuJoCo 原始坐标（并乘回符号还原），用于校正关节正负方向。
JOINT_DIR = np.asarray(
    [1, 1, -1, 1, 1, -1, 1, -1,
     -1, 1, -1, 1, -1, -1, 1, -1],
    dtype=np.float64,
)
# These are the official offsets after M20Interface range normalization.
# POS_OFFSET_DEG：官方 SDK 量程归一化后的位置偏移（单位：度），
#                 sdk = (raw - offset) * dir 中的 offset 常数。
POS_OFFSET_DEG = np.asarray(
    [-25, 229, 160, 0, 25, -131, -200, 0,
     -25, -229, -160, 0, 25, 131, 200, 0],
    dtype=np.float64,
)
# 将角度制偏移转为弧度制，供 sdk_to_raw/raw_to_sdk 直接使用。
POS_OFFSET_RAD = np.deg2rad(POS_OFFSET_DEG)
# JOINT_VELOCITY_LIMIT：关节速度上限（rad/s）。
#                       每条腿顺序为 [髋横滚, 髋俯仰, 膝, 轮] × 4。
JOINT_VELOCITY_LIMIT = np.asarray(
    [45.0, 22.4, 22.4, 100.0] * 4,
    dtype=np.float64,
)
# WHEEL_INDICES：四个轮子关节在 16 维向量中的索引（第 3/7/11/15 个）。
WHEEL_INDICES = np.asarray([3, 7, 11, 15], dtype=np.int64)
# LEG_INDICES：12 个腿部关节索引（除轮子以外的其余关节）。
LEG_INDICES = np.asarray(
    [index for index in range(16) if index not in WHEEL_INDICES],
    dtype=np.int64,
)
# JOINT_INITIAL_POSITION：各关节的初始位置（rad），
#                         四条腿对称布置的站立姿态（轮子初始角 0）。
JOINT_INITIAL_POSITION = np.asarray(
    [
        -0.438, -1.16, 2.76, 0.0,
        0.438, -1.16, 2.76, 0.0,
        -0.438, 1.16, -2.76, 0.0,
        0.438, 1.16, -2.76, 0.0,
    ],
    dtype=np.float64,
)


def yaw_quaternion(yaw: float) -> np.ndarray:
    """Return a MuJoCo-order quaternion (w, x, y, z)."""
    # 绕 Z 轴旋转 yaw 角：仅 w 与 z 分量非零（MuJoCo 四元数顺序 w,x,y,z）。
    half = float(yaw) * 0.5
    return np.asarray([math.cos(half), 0.0, 0.0, math.sin(half)])


def normalized_quaternion(values: Iterable[float]) -> np.ndarray:
    """Return a finite unit quaternion, falling back to identity."""
    # 输入非法（形状不对/含 NaN/零范数）时回退为单位四元数 [1,0,0,0]，
    # 保证下游 RPY 与旋转矩阵计算永远拿到有限值。
    quaternion = np.asarray(values, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        return np.asarray([1.0, 0.0, 0.0, 0.0])
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        return np.asarray([1.0, 0.0, 0.0, 0.0])
    return quaternion / norm


def quaternion_to_rpy(values: Iterable[float]) -> Tuple[float, float, float]:
    """Convert a MuJoCo-order quaternion into roll, pitch and yaw."""
    # 标准四元数→欧拉角公式（ZYX 顺序）：
    #   roll  = atan2(2(wx+yz), 1-2(x²+y²))
    #   pitch = asin(2(wy-zx))，先做 ±1 限幅避免 asin 域外错误
    #   yaw   = atan2(2(wz+xy), 1-2(y²+z²))
    w, x, y, z = normalized_quaternion(values)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(
        max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    )
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def world_vector_to_body(
    values: Iterable[float],
    orientation: Iterable[float],
) -> np.ndarray:
    """Rotate a finite world-frame vector into the quaternion's body frame."""
    # 先用四元数构造 body→world 旋转矩阵（R），再转置为 world→body，
    # 最后左乘向量得到机体系坐标；输入非有限时返回零向量。
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return np.zeros(3, dtype=np.float64)
    w, x, y, z = normalized_quaternion(orientation)
    body_to_world = np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    return body_to_world.T @ vector


def sdk_to_raw(
    positions: np.ndarray,
    velocities: np.ndarray,
    torques: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert official SDK joint coordinates into raw MuJoCo coordinates."""
    # SDK → 原始坐标：
    #   raw_pos = sdk_pos * dir + offset（先乘方向符号再加偏移）
    #   raw_vel = sdk_vel * dir
    #   raw_torque = sdk_torque * dir
    return (
        positions * JOINT_DIR + POS_OFFSET_RAD,
        velocities * JOINT_DIR,
        torques * JOINT_DIR,
    )


def raw_to_sdk(
    positions: np.ndarray,
    velocities: np.ndarray,
    torques: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert raw MuJoCo state into official SDK joint coordinates."""
    # 原始坐标 → SDK：
    #   sdk_pos = (raw_pos - offset) * dir
    # 轮子关节额外把角度归一化到 (-π, π] 区间（加减 2π 取模），
    # 保证连续旋转轮的 SDK 角度读数不跳变。
    sdk_position = (positions - POS_OFFSET_RAD) * JOINT_DIR
    sdk_position = sdk_position.copy()
    sdk_position[WHEEL_INDICES] = (
        (sdk_position[WHEEL_INDICES] + math.pi) % (2.0 * math.pi)
    ) - math.pi
    return (
        sdk_position,
        velocities * JOINT_DIR,
        torques * JOINT_DIR,
    )


def pd_torque(
    *,
    position: np.ndarray,
    velocity: np.ndarray,
    desired_position: np.ndarray,
    desired_velocity: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    feedforward: np.ndarray,
    low: np.ndarray,
    high: np.ndarray,
) -> np.ndarray:
    """Evaluate the SDK PD law and clamp it to official actuator limits."""
    # 官方低层控制律：τ = kp*(q_d - q) + kd*(dq_d - dq) + 前馈力矩。
    # 非有限（NaN/Inf）的力矩置 0，最后按执行器 ctrlrange 上下限截断。
    torque = (
        kp * (desired_position - position)
        + kd * (desired_velocity - velocity)
        + feedforward
    )
    torque = np.where(np.isfinite(torque), torque, 0.0)
    return np.clip(torque, low, high)


def apply_wheel_brake(
    desired_velocity: np.ndarray,
    kd: np.ndarray,
    feedforward: np.ndarray,
    brake_kd: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return command copies with a physical zero-speed wheel brake."""
    # 驻车制动：复制一份命令，仅对 4 个轮子的目标速度清零、
    # 增大阻尼系数（取 max(kd, brake_kd)）并清零前馈力矩，
    # 使轮子趋向停转；腿部关节命令保持不变（不修改输入数组）。
    stopped_velocity = np.asarray(desired_velocity).copy()
    stopped_kd = np.asarray(kd).copy()
    stopped_feedforward = np.asarray(feedforward).copy()
    stopped_velocity[WHEEL_INDICES] = 0.0
    stopped_kd[WHEEL_INDICES] = np.maximum(
        stopped_kd[WHEEL_INDICES],
        max(0.0, float(brake_kd)),
    )
    stopped_feedforward[WHEEL_INDICES] = 0.0
    return stopped_velocity, stopped_kd, stopped_feedforward
