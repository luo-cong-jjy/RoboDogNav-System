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

"""Tests for independent collision lookahead behavior."""

from pathlib import Path

import numpy as np

from m20_inspection_core.collision_policy import (
    GridGeometry,
    first_blocking_footprint,
    inflate_blocked_grid,
    predict_poses,
    predict_positions,
    safe_rotation_recovery,
    trajectory_is_blocked,
)


def test_guard_source_clamps_prediction_to_backend_limits() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "self.declare_parameter('max_linear_x', 0.45)" in source
    assert "self.declare_parameter('max_linear_y', 0.20)" in source
    assert "self.declare_parameter('max_angular_z', 0.65)" in source
    assert 'max(-max_x, min(max_x, float(message.linear.x)))' in source


def test_forward_lookahead_detects_wall_before_contact() -> None:
    geometry = GridGeometry(30, 20, 0.1, 0.0, 0.0)
    values = np.zeros((20, 30), dtype=np.int8)
    values[:, 15] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.2
    )
    positions = predict_positions(
        1.0, 1.0, 0.0, (0.5, 0.0, 0.0), 1.0, 0.05
    )
    assert trajectory_is_blocked(blocked, geometry, positions)


def test_clear_turning_trajectory_is_not_stopped() -> None:
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.2
    )
    positions = predict_positions(
        0.0, 0.0, 0.0, (0.2, 0.0, 0.5), 1.0, 0.05
    )
    assert not trajectory_is_blocked(blocked, geometry, positions)


def test_predicted_translation_can_recover_with_requested_rotation() -> None:
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    values[:, 25] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )
    pose = (0.0, 0.0, 0.0)
    command = (0.5, 0.0, 0.5)
    blocked_sample = first_blocking_footprint(
        blocked,
        geometry,
        predict_poses(*pose, command, 1.0, 0.05),
        0.18,
        0.05,
    )

    assert blocked_sample is not None
    assert blocked_sample.sample_index > 0
    assert safe_rotation_recovery(
        blocked,
        geometry,
        pose,
        command,
        1.0,
        0.05,
        0.18,
    ) == (0.0, 0.0, 0.5)


def test_recovery_uses_opposite_rotation_when_requested_sweep_is_blocked() -> None:
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    cell_x, cell_y = geometry.cell(0.16, 0.08)
    values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rotation_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
    ) == (0.0, 0.0, -0.5)


def test_current_footprint_never_authorizes_rotation_recovery() -> None:
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    cell_x, cell_y = geometry.cell(0.18, 0.0)
    values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rotation_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.5),
        1.0,
        0.05,
        0.18,
    ) is None


def test_oriented_double_circle_detects_front_before_body_centre() -> None:
    geometry = GridGeometry(40, 20, 0.1, 0.0, 0.0)
    values = np.zeros((20, 40), dtype=np.int8)
    values[:, 20] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.30
    )
    poses = predict_poses(
        1.55, 1.0, 0.0, (0.0, 0.0, 0.0), 0.1, 0.05
    )

    sample = first_blocking_footprint(
        blocked, geometry, poses, 0.18, 0.05
    )

    assert sample is not None
    assert sample.sample_index == 0
    assert sample.circle == 'front'
    assert sample.cause == 'OCCUPIED'


def test_shared_edge_tolerance_only_opens_occupied_grid_doorways() -> None:
    geometry = GridGeometry(
        10, 10, 1.0, 0.0, 0.0, edge_tolerance=0.35
    )
    values = np.zeros((10, 10), dtype=np.int8)
    values[0, :] = 100
    values[-1, :] = 100
    values[:, 0] = 100
    values[:, -1] = 100
    values[4:7, 0] = 0
    values[4:7, -1] = 0
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 1.0
    )

    assert geometry.cell(-0.20, 5.0) == (0, 5)
    assert geometry.cell(10.20, 5.0) == (9, 5)
    assert not trajectory_is_blocked(
        blocked, geometry, [(-0.20, 5.0)]
    )
    assert geometry.cell(10.0, 5.0) == (9, 5)
    assert not trajectory_is_blocked(
        blocked, geometry, [(10.20, 5.0)]
    )
    assert trajectory_is_blocked(
        blocked, geometry, [(-0.351, 5.0)]
    )
    assert trajectory_is_blocked(
        blocked, geometry, [(10.351, 5.0)]
    )
    assert trajectory_is_blocked(
        blocked, geometry, [(10.20, 2.0)]
    )
