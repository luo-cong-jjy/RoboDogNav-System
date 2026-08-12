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

"""Tests for pure planar kinematics."""

import math
from pathlib import Path

import pytest

from m20_warehouse_sim.kinematics import (
    PlanarState,
    clamp_planar_command,
    integrate_planar,
    normalize_angle,
)


ROOT = Path(__file__).parents[1]


def test_forward_motion_follows_heading() -> None:
    state = PlanarState(1.0, 2.0, 0.59, math.pi / 2.0)
    result = integrate_planar(state, (0.4, 0.0, 0.0), 2.0)
    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(2.8)
    assert result.z == 0.59
    assert result.vx_body == pytest.approx(0.4)
    assert result.vy_body == pytest.approx(0.0)
    assert result.vx_world == pytest.approx(0.0, abs=1.0e-12)
    assert result.vy_world == pytest.approx(0.4)


def test_planar_rotation_is_normalized() -> None:
    state = PlanarState(0.0, 0.0, 0.59, math.pi - 0.1)
    result = integrate_planar(state, (0.0, 0.0, 0.5), 1.0)
    assert -math.pi <= result.yaw <= math.pi
    assert result.yaw == pytest.approx(normalize_angle(math.pi + 0.4))


def test_backend_limits_are_defensive() -> None:
    assert clamp_planar_command(2.0, -2.0, 3.0, (0.45, 0.2, 0.65)) == (
        0.45,
        -0.2,
        0.65,
    )


def test_odometry_publishes_body_velocity_in_child_frame() -> None:
    source = (
        ROOT
        / 'm20_warehouse_sim'
        / 'kinematic_backend_node.py'
    ).read_text(encoding='utf-8')

    assert (
        'odometry.twist.twist.linear.x = self._state.vx_body'
        in source
    )
    assert (
        'odometry.twist.twist.linear.y = self._state.vy_body'
        in source
    )
