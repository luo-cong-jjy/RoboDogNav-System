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

"""Pure occupancy-grid collision lookahead policy."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

import numpy as np


PlanarCommand = Tuple[float, float, float]
PlanarPose = Tuple[float, float, float]


@dataclass(frozen=True)
class BlockingSample:
    """First predicted footprint sample that intersects the blocked grid."""

    sample_index: int
    time_sec: float
    circle: str
    x: float
    y: float
    yaw: float
    cell_x: int
    cell_y: int
    cause: str


@dataclass(frozen=True)
class GridGeometry:
    """Metric occupancy-grid geometry used by the safety layer."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    edge_tolerance: float = 0.35

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert a metric position to a grid cell.

        A connected-floor doorway is centred on a shared map edge.  Odometry
        can settle a few centimetres on either side while the active map is
        committed, so clamp a small configurable band to the edge cell.
        Closed edges remain blocked by their occupied boundary cells.  A pose
        beyond the band remains out of bounds and therefore fail-closed.
        """
        lower_x = self.origin_x
        lower_y = self.origin_y
        upper_x = self.origin_x + self.width * self.resolution
        upper_y = self.origin_y + self.height * self.resolution
        tolerance = max(0.0, self.edge_tolerance)

        def _axis_cell(
            value: float,
            lower: float,
            upper: float,
            size: int,
        ) -> int:
            if lower - tolerance <= value < lower:
                return 0
            if upper <= value <= upper + tolerance:
                return size - 1
            return int(math.floor((value - lower) / self.resolution))

        cell_x = _axis_cell(x, lower_x, upper_x, self.width)
        cell_y = _axis_cell(y, lower_y, upper_y, self.height)
        return (
            cell_x,
            cell_y,
        )


def inflate_blocked_grid(
    values: Sequence[int],
    geometry: GridGeometry,
    occupied_threshold: int,
    radius: float,
) -> np.ndarray:
    """Inflate occupied and unknown cells for the M20 safety footprint."""
    raw = np.asarray(values, dtype=np.int16).reshape(
        geometry.height, geometry.width
    )
    blocked = np.logical_or(raw < 0, raw >= occupied_threshold)
    cells = int(math.ceil(max(0.0, radius) / geometry.resolution))
    if cells == 0:
        return blocked
    # Map-edge collision is still enforced by trajectory_is_blocked.  Padding
    # with occupied cells here would incorrectly seal an intentional doorway
    # whose centre lies on the shared F1/F2 boundary.
    padded = np.pad(blocked, cells, mode='constant', constant_values=False)
    inflated = np.zeros_like(blocked)
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy > cells * cells:
                continue
            y0 = cells + dy
            x0 = cells + dx
            inflated |= padded[
                y0:y0 + geometry.height,
                x0:x0 + geometry.width,
            ]
    return inflated


def predict_poses(
    x: float,
    y: float,
    yaw: float,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
) -> Iterable[PlanarPose]:
    """Yield constant-command body poses across a short safety horizon."""
    vx, vy, wz = command
    period = max(0.01, sample_period)
    steps = max(1, int(math.ceil(max(0.0, horizon) / period)))
    yield x, y, yaw
    for _ in range(steps):
        world_vx = math.cos(yaw) * vx - math.sin(yaw) * vy
        world_vy = math.sin(yaw) * vx + math.cos(yaw) * vy
        x += world_vx * period
        y += world_vy * period
        yaw += wz * period
        yield x, y, yaw


def predict_positions(
    x: float,
    y: float,
    yaw: float,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
) -> Iterable[Tuple[float, float]]:
    """Yield body-centre positions for legacy single-circle callers."""
    for pose_x, pose_y, _ in predict_poses(
        x, y, yaw, command, horizon, sample_period
    ):
        yield pose_x, pose_y


def first_blocking_footprint(
    blocked: np.ndarray,
    geometry: GridGeometry,
    poses: Iterable[PlanarPose],
    footprint_offset: float,
    sample_period: float,
) -> BlockingSample | None:
    """Return the first blocked front/rear circle centre along a trajectory."""
    offset = max(0.0, float(footprint_offset))
    period = max(0.01, float(sample_period))
    for sample_index, (x, y, yaw) in enumerate(poses):
        heading_x = math.cos(yaw)
        heading_y = math.sin(yaw)
        circles = (
            ('front', x + offset * heading_x, y + offset * heading_y),
            ('rear', x - offset * heading_x, y - offset * heading_y),
        )
        for name, circle_x, circle_y in circles:
            cell_x, cell_y = geometry.cell(circle_x, circle_y)
            out_of_bounds = (
                cell_x < 0
                or cell_y < 0
                or cell_x >= geometry.width
                or cell_y >= geometry.height
            )
            occupied = (
                not out_of_bounds and blocked[cell_y, cell_x]
            )
            if out_of_bounds or occupied:
                return BlockingSample(
                    sample_index=sample_index,
                    time_sec=sample_index * period,
                    circle=name,
                    x=circle_x,
                    y=circle_y,
                    yaw=yaw,
                    cell_x=cell_x,
                    cell_y=cell_y,
                    cause=(
                        'OUT_OF_BOUNDS'
                        if out_of_bounds
                        else 'OCCUPIED'
                    ),
                )
    return None


def safe_rotation_recovery(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
) -> PlanarCommand | None:
    """
    Return a collision-checked rotation that can break a predicted-stop hold.

    Recovery never translates the body.  The requested yaw direction is
    preferred; the opposite direction is considered only when the requested
    sweep is blocked.  A currently blocked footprint is never recoverable.
    """
    x, y, yaw = pose
    current = first_blocking_footprint(
        blocked,
        geometry,
        [(x, y, yaw)],
        footprint_offset,
        sample_period,
    )
    if current is not None:
        return None
    requested_yaw = float(command[2])
    if abs(requested_yaw) < 1.0e-3:
        return None
    for yaw_rate in (requested_yaw, -requested_yaw):
        recovery = (0.0, 0.0, yaw_rate)
        recovery_block = first_blocking_footprint(
            blocked,
            geometry,
            predict_poses(
                x,
                y,
                yaw,
                recovery,
                horizon,
                sample_period,
            ),
            footprint_offset,
            sample_period,
        )
        if recovery_block is None:
            return recovery
    return None


def trajectory_is_blocked(
    blocked: np.ndarray,
    geometry: GridGeometry,
    positions: Iterable[Tuple[float, float]],
) -> bool:
    """Return true when any predicted position enters a blocked cell."""
    for x, y in positions:
        cell_x, cell_y = geometry.cell(x, y)
        if (
            cell_x < 0
            or cell_y < 0
            or cell_x >= geometry.width
            or cell_y >= geometry.height
            or blocked[cell_y, cell_x]
        ):
            return True
    return False
