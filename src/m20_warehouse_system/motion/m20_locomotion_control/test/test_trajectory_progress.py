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

"""Unit tests for measured-progress SCAN trajectory execution."""

import math

import pytest

from m20_locomotion_control.trajectory_progress import (
    Bspline2D,
    PathPoint,
    ProgressFollowerParameters,
    SampledPath,
    SpatialProgressFollower,
)


def _line_spline() -> Bspline2D:
    return Bspline2D(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        1,
        [-1.0, 0.0, 1.0, 2.0, 3.0],
    )


def _line_path() -> SampledPath:
    return SampledPath(
        [
            PathPoint(0.0, 0.0, 0.0, 0.0),
            PathPoint(1.0, 0.0, 1.0, 1.0),
            PathPoint(2.0, 0.0, 2.0, 2.0),
        ]
    )


def test_de_boor_evaluator_matches_linear_endpoints_and_midpoint():
    spline = _line_spline()

    assert spline.duration == 2.0
    assert spline.evaluate(0.0) == (0.0, 0.0)
    assert spline.evaluate(1.0) == (1.0, 0.0)
    assert spline.evaluate(2.0) == (2.0, 0.0)


def test_invalid_knot_contract_is_rejected():
    with pytest.raises(ValueError, match='knot count'):
        Bspline2D([(0.0, 0.0), (1.0, 0.0)], 1, [0.0, 1.0])


def test_sampled_path_preserves_arc_length_and_terminal_speed():
    path = SampledPath.from_spline(_line_spline(), 0.05)

    assert math.isclose(path.length_m, 2.0, abs_tol=1.0e-9)
    assert math.isclose(path.end_speed_mps, 1.0, abs_tol=1.0e-9)


def test_projection_and_arc_length_interpolation_are_spatial():
    path = _line_path()
    projection = path.closest_projection(0.70, 0.20)

    assert math.isclose(projection.progress_m, 0.70)
    assert math.isclose(projection.distance_m, 0.20)
    assert path.point_at(1.25) == (1.25, 0.0)


def test_progress_never_regresses_when_odometry_moves_backwards():
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    path = _line_path()

    first = follower.update(path, 1.0, 0.0, 0.0)
    second = follower.update(path, 0.5, 0.0, 0.0)

    assert first.progress_m == 1.0
    assert second.progress_m == 1.0


def test_straight_path_requests_m20_speed_without_wall_clock_advance():
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    result = follower.update(_line_path(), 0.0, 0.0, 0.0)

    assert result.command == (0.45, 0.0, 0.0)
    assert result.progress_m == 0.0
    assert result.target == (0.60, 0.0)


def test_heading_error_is_bounded_by_m20_yaw_envelope():
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    result = follower.update(_line_path(), 0.0, 0.0, math.pi / 4.0)

    assert result.command == (0.45, 0.0, -0.65)
    assert math.isclose(result.heading_error_rad, -math.pi / 4.0)


def test_path_behind_uses_reverse_without_demanding_a_turnaround():
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    result = follower.update(_line_path(), 0.0, 0.0, math.pi)

    assert result.reverse_tracking
    assert result.command == (-0.45, 0.0, 0.0)


def test_terminal_path_stops_only_near_a_zero_speed_endpoint():
    terminal = SampledPath(
        [
            PathPoint(0.0, 0.0, 0.0, 0.0),
            PathPoint(1.0, 0.0, 1.0, 1.0),
            PathPoint(1.01, 0.0, 2.0, 1.01),
        ]
    )
    follower = SpatialProgressFollower(ProgressFollowerParameters())

    result = follower.update(terminal, 1.0, 0.0, 0.0)

    assert result.finished
    assert result.command == (0.0, 0.0, 0.0)
