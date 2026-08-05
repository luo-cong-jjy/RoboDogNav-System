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

"""Unit tests for the deterministic grid route planner."""

import numpy as np

from m20_scan_navigation.grid_route import (
    astar_path,
    classify_continuous_transitions,
    clear_boundary_gateway_inflation,
    clearance_from_blocked,
    compress_route_for_scan,
    GridGeometry,
    line_is_free,
    line_respects_soft_clearance,
    make_blocked_grid,
    plan_metric_route,
    prepend_reverse_escape_for_heading,
    simplify_path,
)
from m20_scan_navigation.goal_policy import (
    goal_bearing_error,
    rear_goal_requires_route,
)


def test_route_detours_around_blocking_wall() -> None:
    geometry = GridGeometry(30, 20, 0.1, 0.0, 0.0)
    values = np.zeros((20, 30), dtype=np.int8)
    values[4:16, 14] = 100
    blocked = make_blocked_grid(values.ravel(), geometry, 50, 0.1)
    route = plan_metric_route(
        blocked, geometry, (0.5, 1.0), (2.5, 1.0), 0.5, 0.2
    )
    assert len(route) > 2
    assert any(y < 0.7 or y > 1.3 for _, y in route)
    cells = [geometry.world_to_cell(point) for point in route]
    assert all(
        line_is_free(start, end, blocked)
        for start, end in zip(cells, cells[1:])
    )


def test_inflation_blocks_neighboring_cells() -> None:
    geometry = GridGeometry(9, 9, 0.1, 0.0, 0.0)
    values = np.zeros((9, 9), dtype=np.int8)
    values[4, 4] = 100
    blocked = make_blocked_grid(values.ravel(), geometry, 50, 0.2)
    assert blocked[4, 4]
    assert blocked[4, 5]
    assert blocked[4, 6]
    # The outer safety band is deliberately blocked; an interior cell away
    # from the obstacle remains available.
    assert not blocked[2, 2]


def test_soft_clearance_cost_prefers_open_detour_without_new_blocking() -> None:
    geometry = GridGeometry(25, 15, 0.1, 0.0, 0.0)
    values = np.zeros((15, 25), dtype=np.int8)
    values[5:10, 10:15] = 100
    blocked = make_blocked_grid(values.ravel(), geometry, 50, 0.0)
    clearance = clearance_from_blocked(blocked)
    start = (2, 7)
    goal = (22, 7)

    shortest = astar_path(blocked, geometry, start, goal)
    preferred = astar_path(
        blocked, geometry, start, goal, clearance, 0.30, 4.0
    )

    assert shortest and preferred
    shortest_minimum = min(clearance[y, x] for x, y in shortest)
    preferred_minimum = min(clearance[y, x] for x, y in preferred)
    assert shortest_minimum == 1.0
    assert preferred_minimum >= 3.0


def test_simplification_preserves_the_preferred_clearance_corridor() -> None:
    geometry = GridGeometry(25, 15, 0.1, 0.0, 0.0)
    values = np.zeros((15, 25), dtype=np.int8)
    values[5:10, 10:15] = 100
    blocked = make_blocked_grid(values.ravel(), geometry, 50, 0.0)
    clearance = clearance_from_blocked(blocked)
    path = astar_path(
        blocked, geometry, (2, 7), (22, 7), clearance, 0.30, 4.0
    )
    simplified = simplify_path(path, blocked, clearance, 3.0)

    assert len(simplified) > 2
    assert all(
        line_respects_soft_clearance(
            start, end, blocked, clearance, 3.0
        )
        for start, end in zip(simplified, simplified[1:])
    )


def test_soft_margin_does_not_close_the_only_narrow_corridor() -> None:
    geometry = GridGeometry(20, 9, 0.1, 0.0, 0.0)
    blocked = np.ones((9, 20), dtype=bool)
    blocked[4, 1:19] = False
    clearance = clearance_from_blocked(blocked)
    path = astar_path(
        blocked, geometry, (1, 4), (18, 4), clearance, 0.30, 4.0
    )
    assert path
    assert all(cell[1] == 4 for cell in path)


def test_shared_gateway_clears_only_artificial_edge_inflation() -> None:
    geometry = GridGeometry(20, 20, 0.1, 0.0, -1.0)
    values = np.zeros((20, 20), dtype=np.int8)
    # Raw wall on the min-x boundary, with a 0.8 m central doorway.
    values[:, 0] = 100
    values[6:14, 0] = 0
    blocked = make_blocked_grid(values.ravel(), geometry, 50, 0.2)
    assert blocked[10, 0]
    cleared = clear_boundary_gateway_inflation(
        blocked,
        values.ravel(),
        geometry,
        50,
        0.0,
        0.0,
        0.8,
        0.2,
    )
    assert not cleared[10, 0]
    assert not cleared[10, 1]
    # Wall bodies and the 0.2 m wall-end margin remain blocked.
    assert cleared[2, 0]
    assert cleared[6, 0]


def test_scan_subgoals_are_long_line_of_sight_segments() -> None:
    geometry = GridGeometry(30, 20, 0.1, 0.0, 0.0)
    values = np.zeros((20, 30), dtype=np.int8)
    values[4:16, 14] = 100
    blocked = make_blocked_grid(values.ravel(), geometry, 50, 0.1)
    route = plan_metric_route(
        blocked, geometry, (0.5, 1.0), (2.5, 1.0), 0.5, 0.2
    )
    subgoals = compress_route_for_scan(route, blocked, geometry)
    assert subgoals
    assert len(subgoals) < len(route) - 1
    execution = [route[0], *subgoals]
    cells = [geometry.world_to_cell(point) for point in execution]
    assert all(
        line_is_free(start, end, blocked)
        for start, end in zip(cells, cells[1:])
    )
    assert subgoals[-1] == route[-1]


def test_open_shallow_corner_uses_continuous_handoff() -> None:
    geometry = GridGeometry(140, 100, 0.1, -5.0, -5.0)
    blocked = np.zeros((100, 140), dtype=bool)
    # Treat the map edge as the nearest hard boundary so the open interior
    # has a finite, realistic clearance field.
    blocked[[0, -1], :] = True
    blocked[:, [0, -1]] = True
    clearance = clearance_from_blocked(blocked)
    transitions = classify_continuous_transitions(
        (0.0, 0.0),
        [(2.0, 0.0), (4.0, 0.8), (6.0, 0.8)],
        blocked,
        geometry,
        clearance,
        0.70,
        0.54,
        0.70,
        0.30,
    )
    assert transitions == [True, True, False]


def test_sharp_or_tight_corner_retains_stop_boundary() -> None:
    geometry = GridGeometry(100, 100, 0.1, -5.0, -5.0)
    blocked = np.zeros((100, 100), dtype=bool)
    blocked[[0, -1], :] = True
    blocked[:, [0, -1]] = True
    clearance = clearance_from_blocked(blocked)
    sharp = classify_continuous_transitions(
        (0.0, 0.0),
        [(2.0, 0.0), (2.0, 2.0)],
        blocked,
        geometry,
        clearance,
        0.70,
        0.54,
        0.70,
        0.30,
    )
    assert sharp == [False, False]

    # The heading change is shallow, but a hard wall 0.2 m from the proposed
    # connector consumes the extra rolling-sweep reserve.
    blocked[53, 45:85] = True
    clearance = clearance_from_blocked(blocked)
    tight = classify_continuous_transitions(
        (0.0, 0.0),
        [(2.0, 0.0), (4.0, 0.8)],
        blocked,
        geometry,
        clearance,
        0.70,
        0.54,
        0.70,
        0.30,
    )
    assert tight == [False, False]


def test_sharp_boundary_turn_gets_straight_reverse_escape() -> None:
    geometry = GridGeometry(80, 80, 0.1, -4.0, -4.0)
    blocked = np.zeros((80, 80), dtype=bool)
    start = (-2.0, -1.5)
    subgoals = [(2.0, -1.5)]
    result = prepend_reverse_escape_for_heading(
        start,
        -np.pi / 2.0,
        subgoals,
        blocked,
        geometry,
        1.0,
        0.35,
        0.9,
        1.2,
        0.1,
    )
    assert len(result) == 2
    assert np.isclose(result[0][0], start[0])
    assert result[0][1] > start[1] + 1.1
    assert result[1] == subgoals[0]


def test_aligned_forward_or_reverse_route_needs_no_escape() -> None:
    geometry = GridGeometry(80, 80, 0.1, -4.0, -4.0)
    blocked = np.zeros((80, 80), dtype=bool)
    start = (0.0, 0.0)
    for subgoals in ([(2.0, 0.0)], [(-2.0, 0.0)]):
        result = prepend_reverse_escape_for_heading(
            start,
            0.0,
            subgoals,
            blocked,
            geometry,
            1.0,
            0.35,
            0.9,
            1.2,
            0.1,
        )
        assert result == subgoals


def test_rear_goal_policy_routes_position_goal_without_forcing_yaw() -> None:
    assert np.isclose(
        goal_bearing_error((0.0, 0.0), 0.0, (-2.0, 0.0)), np.pi
    )
    assert rear_goal_requires_route(
        (0.0, 0.0), 0.0, (-2.0, 0.0), 2.10, 0.35
    )
    assert not rear_goal_requires_route(
        (0.0, 0.0), 0.0, (2.0, 0.0), 2.10, 0.35
    )
    assert not rear_goal_requires_route(
        (0.0, 0.0), 0.0, (-0.2, 0.0), 2.10, 0.35
    )


def test_short_reverse_before_sharp_turn_is_extended_into_open_space() -> None:
    geometry = GridGeometry(80, 80, 0.1, -4.0, -4.0)
    blocked = np.zeros((80, 80), dtype=bool)
    start = (1.0, 1.0)
    # Body faces north.  The first short target is straight behind it, but the
    # next westward target would otherwise start a rolling turn near the edge.
    subgoals = [(1.0, 0.5), (-2.0, 0.5)]
    result = prepend_reverse_escape_for_heading(
        start,
        np.pi / 2.0,
        subgoals,
        blocked,
        geometry,
        1.0,
        0.35,
        0.9,
        1.2,
        0.1,
    )
    assert len(result) == 2
    assert result[0][1] < -0.1
    assert result[1] == subgoals[1]


def test_reverse_escape_may_leave_only_a_blocked_inflation_prefix() -> None:
    geometry = GridGeometry(80, 80, 0.1, -4.0, -4.0)
    blocked = np.zeros((80, 80), dtype=bool)
    start = (0.0, 0.0)
    start_cell = geometry.world_to_cell(start)
    blocked[start_cell[1], start_cell[0]] = True
    result = prepend_reverse_escape_for_heading(
        start,
        -np.pi / 2.0,
        [(2.0, 0.0)],
        blocked,
        geometry,
        1.0,
        0.35,
        0.9,
        1.2,
        0.1,
    )
    assert len(result) == 2
    assert result[0][1] > 1.1
