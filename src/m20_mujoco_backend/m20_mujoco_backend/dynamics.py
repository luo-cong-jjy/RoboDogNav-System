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

"""Pure numerical helpers shared by the MuJoCo backend and its tests."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np


JOINT_NAMES = (
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint',
)
JOINT_DIR = np.asarray(
    [1, 1, -1, 1, 1, -1, 1, -1,
     -1, 1, -1, 1, -1, -1, 1, -1],
    dtype=np.float64,
)
# These are the official offsets after M20Interface range normalization.
POS_OFFSET_DEG = np.asarray(
    [-25, 229, 160, 0, 25, -131, -200, 0,
     -25, -229, -160, 0, 25, 131, 200, 0],
    dtype=np.float64,
)
POS_OFFSET_RAD = np.deg2rad(POS_OFFSET_DEG)
JOINT_VELOCITY_LIMIT = np.asarray(
    [45.0, 22.4, 22.4, 100.0] * 4,
    dtype=np.float64,
)
WHEEL_INDICES = np.asarray([3, 7, 11, 15], dtype=np.int64)
LEG_INDICES = np.asarray(
    [index for index in range(16) if index not in WHEEL_INDICES],
    dtype=np.int64,
)
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
    half = float(yaw) * 0.5
    return np.asarray([math.cos(half), 0.0, 0.0, math.sin(half)])


def normalized_quaternion(values: Iterable[float]) -> np.ndarray:
    """Return a finite unit quaternion, falling back to identity."""
    quaternion = np.asarray(values, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        return np.asarray([1.0, 0.0, 0.0, 0.0])
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        return np.asarray([1.0, 0.0, 0.0, 0.0])
    return quaternion / norm


def quaternion_to_rpy(values: Iterable[float]) -> Tuple[float, float, float]:
    """Convert a MuJoCo-order quaternion into roll, pitch and yaw."""
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


def sdk_to_raw(
    positions: np.ndarray,
    velocities: np.ndarray,
    torques: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert official SDK joint coordinates into raw MuJoCo coordinates."""
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
