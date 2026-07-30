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

"""Deterministic two-dimensional occupancy-grid route planning utilities."""

from dataclasses import dataclass
import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


Cell = Tuple[int, int]
Point = Tuple[float, float]


@dataclass(frozen=True)
class GridGeometry:
    """Metric geometry of a row-major occupancy grid."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def world_to_cell(self, point: Point) -> Cell:
        """Convert a world position to an integer grid cell."""
        return (
            int(math.floor((point[0] - self.origin_x) / self.resolution)),
            int(math.floor((point[1] - self.origin_y) / self.resolution)),
        )

    def cell_to_world(self, cell: Cell) -> Point:
        """Return the world position at the center of a grid cell."""
        return (
            self.origin_x + (cell[0] + 0.5) * self.resolution,
            self.origin_y + (cell[1] + 0.5) * self.resolution,
        )

    def contains(self, cell: Cell) -> bool:
        """Return whether a cell lies inside the grid."""
        return (
            0 <= cell[0] < self.width
            and 0 <= cell[1] < self.height
        )


def make_blocked_grid(
    values: Sequence[int],
    geometry: GridGeometry,
    occupied_threshold: int,
    inflation_radius: float,
) -> np.ndarray:
    """Convert occupancy values to a conservatively inflated blocked grid."""
    raw = np.asarray(values, dtype=np.int16).reshape(
        geometry.height, geometry.width
    )
    blocked = np.logical_or(raw < 0, raw >= occupied_threshold)
    radius_cells = int(math.ceil(inflation_radius / geometry.resolution))
    if radius_cells <= 0:
        return blocked

    padded = np.pad(
        blocked,
        radius_cells,
        mode='constant',
        constant_values=True,
    )
    inflated = np.zeros_like(blocked)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            y0 = radius_cells + dy
            x0 = radius_cells + dx
            inflated |= padded[
                y0:y0 + geometry.height,
                x0:x0 + geometry.width,
            ]
    return inflated


def nearest_free_cell(
    requested: Cell,
    blocked: np.ndarray,
    geometry: GridGeometry,
    max_radius_cells: int,
) -> Optional[Cell]:
    """Find the nearest free cell to a start or goal request."""
    best: Optional[Cell] = None
    best_distance = math.inf
    for dy in range(-max_radius_cells, max_radius_cells + 1):
        for dx in range(-max_radius_cells, max_radius_cells + 1):
            distance = math.hypot(dx, dy)
            if distance > max_radius_cells or distance >= best_distance:
                continue
            candidate = requested[0] + dx, requested[1] + dy
            if (
                geometry.contains(candidate)
                and not blocked[candidate[1], candidate[0]]
            ):
                best = candidate
                best_distance = distance
    return best


def _neighbors(cell: Cell) -> Iterable[Tuple[Cell, float]]:
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield (
                (cell[0] + dx, cell[1] + dy),
                math.sqrt(2.0) if dx and dy else 1.0,
            )


def astar_path(
    blocked: np.ndarray,
    geometry: GridGeometry,
    start: Cell,
    goal: Cell,
) -> List[Cell]:
    """Compute an eight-connected A* path without diagonal corner cutting."""
    if (
        not geometry.contains(start)
        or not geometry.contains(goal)
        or blocked[start[1], start[0]]
        or blocked[goal[1], goal[0]]
    ):
        return []

    frontier: List[Tuple[float, float, Cell]] = []
    heapq.heappush(frontier, (math.dist(start, goal), 0.0, start))
    costs: Dict[Cell, float] = {start: 0.0}
    parents: Dict[Cell, Cell] = {}

    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if cost > costs.get(current, math.inf):
            continue
        if current == goal:
            path = [current]
            while current != start:
                current = parents[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor, step_cost in _neighbors(current):
            if (
                not geometry.contains(neighbor)
                or blocked[neighbor[1], neighbor[0]]
            ):
                continue
            dx = neighbor[0] - current[0]
            dy = neighbor[1] - current[1]
            if dx and dy:
                if (
                    blocked[current[1], current[0] + dx]
                    or blocked[current[1] + dy, current[0]]
                ):
                    continue
            next_cost = cost + step_cost
            if next_cost >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = next_cost
            parents[neighbor] = current
            estimate = next_cost + math.dist(neighbor, goal)
            heapq.heappush(frontier, (estimate, next_cost, neighbor))
    return []


def _line_cells(start: Cell, end: Cell) -> Iterable[Cell]:
    """Yield a conservative supercover of a grid line."""
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0))
    if steps == 0:
        yield start
        return
    for index in range(steps + 1):
        ratio = index / steps
        yield (
            int(round(x0 + ratio * (x1 - x0))),
            int(round(y0 + ratio * (y1 - y0))),
        )


def line_is_free(start: Cell, end: Cell, blocked: np.ndarray) -> bool:
    """Return whether every sampled line cell is free."""
    height, width = blocked.shape
    return all(
        0 <= x < width and 0 <= y < height and not blocked[y, x]
        for x, y in _line_cells(start, end)
    )


def simplify_path(path: Sequence[Cell], blocked: np.ndarray) -> List[Cell]:
    """Remove redundant A* cells while retaining collision-free segments."""
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while (
            candidate > anchor + 1
            and not line_is_free(path[anchor], path[candidate], blocked)
        ):
            candidate -= 1
        simplified.append(path[candidate])
        anchor = candidate
    return simplified


def densify_points(
    points: Sequence[Point],
    maximum_spacing: float,
) -> List[Point]:
    """Sample metric line segments at a bounded waypoint spacing."""
    if not points:
        return []
    result = [points[0]]
    spacing = max(0.05, maximum_spacing)
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        steps = max(1, int(math.ceil(distance / spacing)))
        for index in range(1, steps + 1):
            ratio = index / steps
            result.append(
                (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            )
    return result


def plan_metric_route(
    blocked: np.ndarray,
    geometry: GridGeometry,
    start: Point,
    goal: Point,
    nearest_free_radius: float,
    waypoint_spacing: float,
) -> List[Point]:
    """Plan, simplify, and densify a metric route."""
    radius = int(math.ceil(nearest_free_radius / geometry.resolution))
    start_cell = nearest_free_cell(
        geometry.world_to_cell(start), blocked, geometry, radius
    )
    goal_cell = nearest_free_cell(
        geometry.world_to_cell(goal), blocked, geometry, radius
    )
    if start_cell is None or goal_cell is None:
        return []
    cells = astar_path(blocked, geometry, start_cell, goal_cell)
    if not cells:
        return []
    simplified = simplify_path(cells, blocked)
    points = [geometry.cell_to_world(cell) for cell in simplified]
    points[0] = start
    if geometry.world_to_cell(goal) == goal_cell:
        points[-1] = goal
    return densify_points(points, waypoint_spacing)
