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
    GridGeometry,
    line_is_free,
    make_blocked_grid,
    plan_metric_route,
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
