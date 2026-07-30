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

"""Pure planar kinematics shared by the RViz backend and tests."""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class PlanarState:
    """Pose and world-frame velocity of the simulated robot body."""

    x: float
    y: float
    z: float
    yaw: float
    vx_world: float = 0.0
    vy_world: float = 0.0
    wz: float = 0.0


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp_planar_command(
    vx: float,
    vy: float,
    wz: float,
    limits: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Clamp body-frame planar velocity to backend defense limits."""
    max_vx, max_vy, max_wz = (abs(value) for value in limits)
    return (
        max(-max_vx, min(max_vx, vx)),
        max(-max_vy, min(max_vy, vy)),
        max(-max_wz, min(max_wz, wz)),
    )


def integrate_planar(
    state: PlanarState,
    body_command: Tuple[float, float, float],
    dt: float,
) -> PlanarState:
    """Integrate a body-frame Twist into map-frame pose and velocity."""
    if dt <= 0.0:
        return state
    vx, vy, wz = body_command
    cosine = math.cos(state.yaw)
    sine = math.sin(state.yaw)
    vx_world = cosine * vx - sine * vy
    vy_world = sine * vx + cosine * vy
    return PlanarState(
        x=state.x + vx_world * dt,
        y=state.y + vy_world * dt,
        z=state.z,
        yaw=normalize_angle(state.yaw + wz * dt),
        vx_world=vx_world,
        vy_world=vy_world,
        wz=wz,
    )
