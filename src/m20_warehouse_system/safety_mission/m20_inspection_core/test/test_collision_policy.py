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
    command_drift_variants,
    conservative_raster_radius,
    first_blocking_command_envelope,
    GridGeometry,
    first_blocking_footprint,
    hard_body_raster_radius,
    inflate_blocked_grid,
    predict_poses,
    predict_positions,
    rasterize_online_occupancy,
    recovery_sweep_is_clear,
    safe_raster_shell_escape,
    safe_rolling_recovery,
    trajectory_is_blocked,
    update_clear_confirmation,
    update_recovery_budget,
)


def test_online_occupancy_filters_height_and_uses_world_coordinates() -> None:
    occupancy, geometry, accepted = rasterize_online_occupancy(
        [
            (10.4, -3.0, 0.40),
            (10.6, -3.0, 1.10),
            (10.8, -3.0, 1.80),
            (float('nan'), -3.0, 0.60),
        ],
        (10.0, -3.0),
        0.10,
        4.0,
        0.20,
        1.20,
    )

    assert accepted == 2
    assert occupancy.shape == (geometry.width * geometry.height,)
    for x in (10.4, 10.6):
        cell_x, cell_y = geometry.cell(x, -3.0)
        assert occupancy[cell_y * geometry.width + cell_x] == 100


def test_online_occupancy_drops_points_outside_local_window() -> None:
    occupancy, _geometry, accepted = rasterize_online_occupancy(
        [(0.5, 0.0, 0.5), (20.0, 0.0, 0.5)],
        (0.0, 0.0),
        0.10,
        4.0,
        0.0,
        1.0,
    )

    assert accepted == 1
    assert int(np.count_nonzero(occupancy)) == 1


def test_guard_source_clamps_prediction_to_backend_limits() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "self.declare_parameter('max_linear_x', 0.45)" in source
    assert "self.declare_parameter('max_linear_y', 0.20)" in source
    assert "self.declare_parameter('max_angular_z', 0.65)" in source
    assert "self.declare_parameter('recovery_forward_speed', 0.35)" in source
    assert (
        "self.declare_parameter('recovery_clear_confirm_sec', 0.30)"
        in source
    )
    assert (
        "self.declare_parameter('recovery_rearm_clear_sec', 1.50)"
        in source
    )
    assert "self.declare_parameter('recovery_max_active_sec', 6.0)" in source
    assert (
        "self.declare_parameter('recovery_max_displacement_m', 0.75)"
        in source
    )
    assert (
        "'recovery_positive_yaw_lateral_drift', 0.15"
        in source
    )
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


def test_conservative_raster_radius_closes_half_cell_aliasing_gap() -> None:
    geometry = GridGeometry(40, 20, 0.05, 0.0, 0.0)
    values = np.zeros((20, 40), dtype=np.int8)
    values[10, 20] = 100
    nominal = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.30
    )
    conservative = inflate_blocked_grid(
        values.ravel(),
        geometry,
        50,
        conservative_raster_radius(0.30, geometry.resolution),
    )

    # Seven cells are 0.35 m apart. The old six-cell kernel misses this
    # lookup even though a continuous centre can be less than 0.30 m away.
    assert not nominal[10, 27]
    assert conservative[10, 27]


def test_hard_body_radius_excludes_only_half_cell_shell() -> None:
    assert abs(hard_body_raster_radius(0.25, 0.10) - 0.1792893) < 1e-6


def test_raster_shell_escape_requires_hard_clear_sweep_and_clear_endpoint():
    geometry = GridGeometry(80, 40, 0.1, -4.0, -2.0)
    conservative = np.zeros((40, 80), dtype=bool)
    hard = np.zeros_like(conservative)
    pose = (0.0, 0.0, 0.0)
    front_x, front_y = geometry.cell(0.18, 0.0)
    conservative[front_y, front_x] = True

    arguments = (
        conservative,
        hard,
        geometry,
        pose,
        0.90,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
    )
    assert safe_raster_shell_escape(*arguments) == (-0.35, 0.0, 0.0)

    hard[front_y, front_x] = True
    assert safe_raster_shell_escape(*arguments) is None


def test_measured_turn_drift_is_checked_beside_nominal_sweep() -> None:
    geometry = GridGeometry(100, 100, 0.05, -2.5, -2.5)
    values = np.zeros((100, 100), dtype=np.int8)
    obstacle_x, obstacle_y = geometry.cell(0.14, 0.08)
    values[obstacle_y, obstacle_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )
    pose = (0.0, 0.0, 0.0)
    command = (0.35, 0.0, 0.65)
    nominal = first_blocking_footprint(
        blocked,
        geometry,
        predict_poses(*pose, command, 0.5, 0.05),
        0.0,
        0.05,
    )
    envelope = first_blocking_command_envelope(
        blocked,
        geometry,
        pose,
        command,
        0.5,
        0.05,
        0.0,
        0.15,
        0.10,
        0.05,
        0.65,
    )

    assert nominal is None
    assert envelope is not None
    assert envelope.motion_model == 'measured_lateral_drift'


def test_turn_drift_scales_with_requested_yaw_rate() -> None:
    variants = dict(
        command_drift_variants(
            (0.35, 0.0, -0.325),
            0.15,
            0.10,
            0.05,
            0.65,
        )
    )

    assert variants['measured_lateral_drift'] == (
        0.35,
        0.05,
        -0.325,
    )
    assert variants['opposite_lateral_uncertainty'] == (
        0.35,
        -0.025,
        -0.325,
    )


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


def test_predicted_translation_uses_opposite_rolling_turn() -> None:
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
    assert safe_rolling_recovery(
        blocked,
        geometry,
        pose,
        command,
        1.0,
        0.05,
        0.18,
        0.35,
        0.0,
        0.0,
        0.0,
        0.20,
    ) == (0.35, 0.0, -0.5)


def test_recovery_uses_yaw_when_opposite_arc_is_blocked() -> None:
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    cell_x, cell_y = geometry.cell(0.49, -0.17)
    values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.0,
        0.0,
        0.0,
        0.20,
    ) == (0.35, 0.0, 0.5)


def test_current_footprint_never_authorizes_rolling_recovery() -> None:
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    cell_x, cell_y = geometry.cell(0.18, 0.0)
    values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.5),
        1.0,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    ) is None


def test_pure_yaw_request_never_authorizes_rolling_recovery() -> None:
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    blocked = np.zeros((40, 40), dtype=bool)

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.5),
        1.0,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    ) is None


def test_weak_yaw_forward_request_can_use_clear_reverse() -> None:
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    front_x, front_y = geometry.cell(0.55, 0.0)
    values[front_y, front_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.03),
        1.0,
        0.02,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    ) == (-0.35, 0.0, 0.0)


def test_shared_edge_prediction_can_recover_straight_inward() -> None:
    geometry = GridGeometry(
        400, 400, 0.1, -40.0, -20.0, edge_tolerance=0.35
    )
    blocked = np.zeros((400, 400), dtype=bool)
    pose = (0.10, 0.12, -2.63)
    command = (-0.16, 0.0, -0.18)

    blocked_sample = first_blocking_footprint(
        blocked,
        geometry,
        predict_poses(*pose, command, 0.90, 0.05),
        0.18,
        0.05,
    )
    recovery = safe_rolling_recovery(
        blocked,
        geometry,
        pose,
        command,
        0.90,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    )

    assert blocked_sample is not None
    assert blocked_sample.cause == 'OUT_OF_BOUNDS'
    assert recovery == (0.35, 0.0, 0.0)


def test_measured_lateral_drift_falls_back_to_clear_reverse() -> None:
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    for obstacle_x, obstacle_y in (
        (0.53, -0.026),
        (0.458, 0.314),
    ):
        cell_x, cell_y = geometry.cell(obstacle_x, obstacle_y)
        values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    nominal = safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.0,
        0.0,
        0.0,
        0.20,
    )
    conservative = safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.15,
        0.15,
        0.05,
        0.20,
    )

    assert nominal is not None
    assert conservative == (-0.35, 0.0, 0.0)


def test_reverse_recovery_is_rejected_when_rear_sweep_is_blocked() -> None:
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    # Two front-side obstacles reject both drift-aware rolling arcs.  The
    # rear obstacle also rejects the straight reverse fallback.
    for obstacle_x, obstacle_y in (
        (0.53, -0.026),
        (0.458, 0.314),
        (-0.50, 0.0),
    ):
        cell_x, cell_y = geometry.cell(obstacle_x, obstacle_y)
        values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.15,
        0.15,
        0.05,
        0.20,
    ) is None


def test_latched_recovery_is_rechecked_against_current_sweep() -> None:
    geometry = GridGeometry(100, 100, 0.05, -2.5, -2.5)
    values = np.zeros((100, 100), dtype=np.int8)
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )
    recovery = (-0.35, 0.0, 0.0)
    arguments = (
        geometry,
        (0.0, 0.0, 0.0),
        recovery,
        0.9,
        0.05,
        0.18,
        0.15,
        0.10,
        0.05,
    )
    assert recovery_sweep_is_clear(blocked, *arguments)

    rear_x, rear_y = geometry.cell(-0.45, 0.0)
    blocked[rear_y, rear_x] = True
    assert not recovery_sweep_is_clear(blocked, *arguments)


def test_recovery_release_requires_continuous_clear_time() -> None:
    first_clear, confirmed = update_clear_confirmation(None, 10.0, 0.30)
    assert first_clear == 10.0
    assert not confirmed

    first_clear, confirmed = update_clear_confirmation(
        first_clear, 10.29, 0.30
    )
    assert first_clear == 10.0
    assert not confirmed

    first_clear, confirmed = update_clear_confirmation(
        first_clear, 10.30, 0.30
    )
    assert first_clear == 10.0
    assert confirmed


def test_disabled_or_rewound_clear_confirmation_is_deterministic() -> None:
    assert update_clear_confirmation(8.0, 7.0, 0.30) == (7.0, False)
    assert update_clear_confirmation(8.0, 8.0, 0.0) == (None, True)


def test_recovery_budget_refreshes_only_after_minimum_progress() -> None:
    progress_time, progress_xy, reason = update_recovery_budget(
        10.0,
        (0.0, 0.0),
        10.0,
        (0.0, 0.0),
        11.0,
        (0.04, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )

    assert progress_time == 11.0
    assert progress_xy == (0.04, 0.0)
    assert reason is None


def test_recovery_budget_stops_a_motionless_command() -> None:
    _, _, reason = update_recovery_budget(
        10.0,
        (0.0, 0.0),
        10.0,
        (0.0, 0.0),
        11.5,
        (0.01, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )

    assert reason == 'NO_PROGRESS'


def test_recovery_budget_has_absolute_time_and_distance_limits() -> None:
    common = (10.0, (0.0, 0.0), 15.0, (0.50, 0.0))
    assert update_recovery_budget(
        *common,
        16.0,
        (0.60, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )[2] == 'TIME_LIMIT'
    assert update_recovery_budget(
        10.0,
        (0.0, 0.0),
        10.5,
        (0.40, 0.0),
        11.0,
        (0.75, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )[2] == 'DISTANCE_LIMIT'


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
