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


def clearance_from_blocked(blocked: np.ndarray) -> np.ndarray:
    """Return an eight-connected distance-to-blocked map in grid cells.

    The hard occupancy layer remains binary.  This map is used only as a
    positive route cost outside that layer, so a narrow but genuinely free
    passage is never changed into an occupied one.
    """
    height, width = blocked.shape
    clearance = np.where(blocked, 0.0, np.inf).astype(np.float64)
    diagonal = math.sqrt(2.0)
    for y in range(height):
        for x in range(width):
            if clearance[y, x] == 0.0:
                continue
            candidates = [clearance[y, x]]
            if x > 0:
                candidates.append(clearance[y, x - 1] + 1.0)
            if y > 0:
                candidates.append(clearance[y - 1, x] + 1.0)
                if x > 0:
                    candidates.append(
                        clearance[y - 1, x - 1] + diagonal
                    )
                if x + 1 < width:
                    candidates.append(
                        clearance[y - 1, x + 1] + diagonal
                    )
            clearance[y, x] = min(candidates)
    for y in range(height - 1, -1, -1):
        for x in range(width - 1, -1, -1):
            if clearance[y, x] == 0.0:
                continue
            candidates = [clearance[y, x]]
            if x + 1 < width:
                candidates.append(clearance[y, x + 1] + 1.0)
            if y + 1 < height:
                candidates.append(clearance[y + 1, x] + 1.0)
                if x > 0:
                    candidates.append(
                        clearance[y + 1, x - 1] + diagonal
                    )
                if x + 1 < width:
                    candidates.append(
                        clearance[y + 1, x + 1] + diagonal
                    )
            clearance[y, x] = min(candidates)
    return clearance


def clear_boundary_gateway_inflation(
    blocked: np.ndarray,
    values: Sequence[int],
    geometry: GridGeometry,
    occupied_threshold: int,
    gateway_x: float,
    gateway_center_y: float,
    gateway_width: float,
    inflation_radius: float,
) -> np.ndarray:
    """Remove only the artificial outside-map inflation at a shared doorway.

    Active floor occupancy ends at the shared x boundary.  Normal inflation
    correctly treats space outside that image as blocked, except at the
    configured gateway where the adjacent flat region continues.  Preserve
    raw occupied/unknown cells and the inflated wall-end margin, while making
    the reserved doorway centerline reachable from either active floor.
    """
    result = np.array(blocked, dtype=bool, copy=True)
    if gateway_width <= 0.0:
        return result
    raw = np.asarray(values, dtype=np.int16).reshape(
        geometry.height, geometry.width
    )
    usable_half_width = max(
        0.0, gateway_width / 2.0 - max(0.0, inflation_radius)
    )
    boundary_depth = max(0.0, inflation_radius) + geometry.resolution
    for row in range(geometry.height):
        for column in range(geometry.width):
            if raw[row, column] < 0 or raw[row, column] >= occupied_threshold:
                continue
            x, y = geometry.cell_to_world((column, row))
            if (
                abs(x - gateway_x) <= boundary_depth
                and abs(y - gateway_center_y) <= usable_half_width
            ):
                result[row, column] = False
    return result


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
    clearance_cells: Optional[np.ndarray] = None,
    soft_clearance_margin: float = 0.0,
    soft_clearance_weight: float = 0.0,
) -> List[Cell]:
    """Compute A* with a soft preference away from the hard obstacle layer."""
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
            multiplier = 1.0
            if (
                clearance_cells is not None
                and soft_clearance_margin > 0.0
                and soft_clearance_weight > 0.0
            ):
                clearance = (
                    float(clearance_cells[neighbor[1], neighbor[0]])
                    * geometry.resolution
                )
                deficit = max(
                    0.0, 1.0 - clearance / soft_clearance_margin
                )
                multiplier += soft_clearance_weight * deficit * deficit
            next_cost = cost + step_cost * multiplier
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


def line_respects_soft_clearance(
    start: Cell,
    end: Cell,
    blocked: np.ndarray,
    clearance_cells: Optional[np.ndarray],
    minimum_clearance_cells: float,
) -> bool:
    """Keep shortcuts out of a soft band except at unavoidable endpoints.

    A start or goal may legitimately be inside the preferred band.  Such a
    segment is accepted only while it moves monotonically out of that band or
    approaches the endpoint monotonically.  A new mid-segment clearance dip
    is rejected so path simplification cannot undo the A* clearance choice.
    """
    cells = list(_line_cells(start, end))
    height, width = blocked.shape
    if any(
        not (0 <= x < width and 0 <= y < height) or blocked[y, x]
        for x, y in cells
    ):
        return False
    if clearance_cells is None or minimum_clearance_cells <= 0.0:
        return True

    values = [float(clearance_cells[y, x]) for x, y in cells]
    safe = [
        index
        for index, value in enumerate(values)
        if value + 1e-9 >= minimum_clearance_cells
    ]
    if not safe:
        endpoint_floor = min(values[0], values[-1])
        return min(values) + 1e-9 >= endpoint_floor

    first_safe = safe[0]
    last_safe = safe[-1]
    if any(
        value + 1e-9 < minimum_clearance_cells
        for value in values[first_safe:last_safe + 1]
    ):
        return False
    if any(
        values[index + 1] + 1e-9 < values[index]
        for index in range(first_safe)
    ):
        return False
    if any(
        values[index + 1] > values[index] + 1e-9
        for index in range(last_safe, len(values) - 1)
    ):
        return False
    return True


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _line_exits_blocked_prefix(
    start: Cell,
    end: Cell,
    blocked: np.ndarray,
) -> bool:
    """Allow a conservative-inflation prefix, then require continuous free space."""
    height, width = blocked.shape
    entered_free_space = False
    for x, y in _line_cells(start, end):
        if not (0 <= x < width and 0 <= y < height):
            return False
        if blocked[y, x]:
            if entered_free_space:
                return False
            continue
        entered_free_space = True
    return entered_free_space


def prepend_reverse_escape_for_heading(
    start: Point,
    yaw: float,
    subgoals: Sequence[Point],
    blocked: np.ndarray,
    geometry: GridGeometry,
    trigger_angle: float,
    reverse_alignment: float,
    minimum_distance: float,
    maximum_distance: float,
    sample_step: float,
) -> List[Point]:
    """Prepend a straight reverse escape before an unsupported sharp turn.

    The validated M20 policy cannot reliably execute zero-radius yaw.  If the
    first route segment is neither a feasible forward heading nor an already
    aligned straight reverse segment, retreat along the current body axis into
    inflated free space.  The existing bidirectional controller then executes
    this first segment as straight reverse motion, leaving room for the next
    rolling turn without changing SCAN's planned geometry or limits.
    """
    result = list(subgoals)
    if not result:
        return result
    dx = result[0][0] - start[0]
    dy = result[0][1] - start[1]
    if math.hypot(dx, dy) < 1e-6:
        return result
    path_yaw = math.atan2(dy, dx)
    forward_error = abs(normalize_angle(path_yaw - yaw))
    reverse_error = abs(normalize_angle(path_yaw + math.pi - yaw))
    if forward_error <= max(0.0, trigger_angle):
        return result

    minimum = max(0.05, minimum_distance)
    maximum = max(minimum, maximum_distance)
    step = max(geometry.resolution, sample_step)
    start_cell = geometry.world_to_cell(start)
    candidate = None
    distance = maximum
    while distance + 1e-9 >= minimum and candidate is None:
        trial = (
            start[0] - math.cos(yaw) * distance,
            start[1] - math.sin(yaw) * distance,
        )
        trial_cell = geometry.world_to_cell(trial)
        if (
            geometry.contains(trial_cell)
            and not blocked[trial_cell[1], trial_cell[0]]
            and _line_exits_blocked_prefix(
                start_cell, trial_cell, blocked
            )
        ):
            candidate = trial
        distance -= step
    if candidate is None:
        return result

    if reverse_error <= max(0.0, reverse_alignment):
        # A short first segment may already be reverse-aligned, yet leave the
        # chassis too close to the boundary for the following rolling turn.
        # Replace it with the farther safe retreat when segment two still
        # requires a large, non-reverse-aligned heading change.
        if len(result) < 2:
            return result
        next_dx = result[1][0] - result[0][0]
        next_dy = result[1][1] - result[0][1]
        if math.hypot(next_dx, next_dy) < 1e-6:
            return result
        next_yaw = math.atan2(next_dy, next_dx)
        next_forward_error = abs(normalize_angle(next_yaw - yaw))
        next_reverse_error = abs(
            normalize_angle(next_yaw + math.pi - yaw)
        )
        first_distance = math.dist(start, result[0])
        candidate_distance = math.dist(start, candidate)
        candidate_cell = geometry.world_to_cell(candidate)
        next_cell = geometry.world_to_cell(result[1])
        if (
            next_forward_error > max(0.0, trigger_angle)
            and next_reverse_error > max(0.0, reverse_alignment)
            and candidate_distance > first_distance + geometry.resolution
            and geometry.contains(next_cell)
            and line_is_free(candidate_cell, next_cell, blocked)
        ):
            return [candidate, *result[1:]]
        return result

    return [candidate, *result]


def simplify_path(
    path: Sequence[Cell],
    blocked: np.ndarray,
    clearance_cells: Optional[np.ndarray] = None,
    minimum_clearance_cells: float = 0.0,
) -> List[Cell]:
    """Remove cells without reintroducing a mid-segment clearance dip."""
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while (
            candidate > anchor + 1
            and not line_respects_soft_clearance(
                path[anchor],
                path[candidate],
                blocked,
                clearance_cells,
                minimum_clearance_cells,
            )
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


def compress_route_for_scan(
    points: Sequence[Point],
    blocked: np.ndarray,
    geometry: GridGeometry,
    clearance_cells: Optional[np.ndarray] = None,
    minimum_clearance: float = 0.0,
) -> List[Point]:
    """Return long line-of-sight targets suitable for SCAN execution.

    The occupancy route is densely sampled for RViz, but feeding every sample
    to SCAN creates very short stop-and-replan trajectories.  Starting at the
    first free sample (the measured pose can legitimately lie inside the
    conservative inflation band), greedily retain only the farthest visible
    target.  Every retained segment therefore stays inside the same inflated
    free space while avoiding sub-metre artificial goals where possible.
    """
    if len(points) < 2:
        return []

    free_anchor = None
    for index, point in enumerate(points):
        cell = geometry.world_to_cell(point)
        if geometry.contains(cell) and not blocked[cell[1], cell[0]]:
            free_anchor = index
            break
    if free_anchor is None:
        return []
    if free_anchor == len(points) - 1:
        return [points[-1]]

    subgoals: List[Point] = []
    anchor = free_anchor
    final_index = len(points) - 1
    while anchor < final_index:
        anchor_cell = geometry.world_to_cell(points[anchor])
        farthest = anchor
        for candidate in range(anchor + 1, len(points)):
            candidate_cell = geometry.world_to_cell(points[candidate])
            if line_respects_soft_clearance(
                anchor_cell,
                candidate_cell,
                blocked,
                clearance_cells,
                minimum_clearance / geometry.resolution,
            ):
                farthest = candidate
        if farthest == anchor:
            # The route was produced from this same blocked grid, so this is
            # only a defensive fail-closed guard against rounding regressions.
            return []
        subgoals.append(points[farthest])
        anchor = farthest

    return subgoals


def continuous_transition_is_safe(
    previous: Point,
    corner: Point,
    following: Point,
    blocked: np.ndarray,
    geometry: GridGeometry,
    clearance_cells: Optional[np.ndarray],
    maximum_heading_change: float,
    minimum_centerline_turn_radius: float,
    handoff_distance: float,
    minimum_extra_clearance: float,
) -> bool:
    """Return whether SCAN may carry velocity through one route corner.

    Sequential goals normally end at zero velocity.  Republishing the next
    goal before that endpoint gives SCAN a smooth connector, but is safe for
    M20 only when the bend is shallow, a minimum-radius rolling turn fits on
    both legs, and the shortcut from the handoff point stays in open space.
    The clearance argument is measured outside the already inflated hard
    grid; it therefore reserves the extra outer-corner sweep of a rolling
    wheel-leg turn without changing SCAN's obstacle map.
    """
    incoming = (corner[0] - previous[0], corner[1] - previous[1])
    outgoing = (following[0] - corner[0], following[1] - corner[1])
    incoming_length = math.hypot(*incoming)
    outgoing_length = math.hypot(*outgoing)
    if incoming_length < 1e-6 or outgoing_length < 1e-6:
        return False

    incoming_yaw = math.atan2(incoming[1], incoming[0])
    outgoing_yaw = math.atan2(outgoing[1], outgoing[0])
    heading_change = abs(normalize_angle(outgoing_yaw - incoming_yaw))
    if heading_change > max(0.0, maximum_heading_change):
        return False

    # A circular fillet of radius R consumes R*tan(theta/2) on each leg.
    # This is a necessary geometry check; the downstream SCAN B-spline is
    # still responsible for generating the actual smooth local trajectory.
    tangent = max(0.0, minimum_centerline_turn_radius) * math.tan(
        heading_change / 2.0
    )
    if min(incoming_length, outgoing_length) + 1e-9 < tangent:
        return False

    lookback = min(
        max(geometry.resolution, handoff_distance),
        incoming_length,
    )
    unit_incoming = (
        incoming[0] / incoming_length,
        incoming[1] / incoming_length,
    )
    handoff = (
        corner[0] - unit_incoming[0] * lookback,
        corner[1] - unit_incoming[1] * lookback,
    )
    handoff_cell = geometry.world_to_cell(handoff)
    following_cell = geometry.world_to_cell(following)
    if not (
        geometry.contains(handoff_cell)
        and geometry.contains(following_cell)
    ):
        return False

    required_cells = max(
        0.0, minimum_extra_clearance / geometry.resolution
    )
    for x, y in _line_cells(handoff_cell, following_cell):
        if (
            not geometry.contains((x, y))
            or blocked[y, x]
            or (
                clearance_cells is not None
                and float(clearance_cells[y, x]) + 1e-9
                < required_cells
            )
        ):
            return False
    return True


def classify_continuous_transitions(
    start: Point,
    subgoals: Sequence[Point],
    blocked: np.ndarray,
    geometry: GridGeometry,
    clearance_cells: Optional[np.ndarray],
    maximum_heading_change: float,
    minimum_centerline_turn_radius: float,
    handoff_distance: float,
    minimum_extra_clearance: float,
) -> List[bool]:
    """Classify each subgoal as continuous or stop-before-next.

    The returned list has the same length as ``subgoals``.  Its final entry
    is always false because the terminal goal must retain SCAN's normal
    zero-velocity completion semantics.
    """
    if not subgoals:
        return []
    points = [start, *subgoals]
    result = [
        continuous_transition_is_safe(
            points[index - 1],
            points[index],
            points[index + 1],
            blocked,
            geometry,
            clearance_cells,
            maximum_heading_change,
            minimum_centerline_turn_radius,
            handoff_distance,
            minimum_extra_clearance,
        )
        for index in range(1, len(points) - 1)
    ]
    result.append(False)
    return result


def plan_metric_route(
    blocked: np.ndarray,
    geometry: GridGeometry,
    start: Point,
    goal: Point,
    nearest_free_radius: float,
    waypoint_spacing: float,
    soft_clearance_margin: float = 0.0,
    soft_clearance_weight: float = 0.0,
    clearance_cells: Optional[np.ndarray] = None,
) -> List[Point]:
    """Plan, clearance-preserving simplify, and densify a metric route."""
    if clearance_cells is None and (
        soft_clearance_margin > 0.0 or soft_clearance_weight > 0.0
    ):
        clearance_cells = clearance_from_blocked(blocked)
    radius = int(math.ceil(nearest_free_radius / geometry.resolution))
    start_cell = nearest_free_cell(
        geometry.world_to_cell(start), blocked, geometry, radius
    )
    goal_cell = nearest_free_cell(
        geometry.world_to_cell(goal), blocked, geometry, radius
    )
    if start_cell is None or goal_cell is None:
        return []
    cells = astar_path(
        blocked,
        geometry,
        start_cell,
        goal_cell,
        clearance_cells,
        soft_clearance_margin,
        soft_clearance_weight,
    )
    if not cells:
        return []
    simplified = simplify_path(
        cells,
        blocked,
        clearance_cells,
        soft_clearance_margin / geometry.resolution,
    )
    points = [geometry.cell_to_world(cell) for cell in simplified]
    points[0] = start
    if geometry.world_to_cell(goal) == goal_cell:
        points[-1] = goal
    return densify_points(points, waypoint_spacing)
