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

"""
Pure online-occupancy collision lookahead policy.

The runtime guard consumes SCAN's live 3-D occupied-voxel cloud.  A small
robot-centred raster is built in memory only so the existing, well-tested M20
footprint and rolling-recovery checks remain deterministic.  No PGM or map
YAML is involved.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
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
    motion_model: str = 'nominal'


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

    def contains(self, cell_x: int, cell_y: int) -> bool:
        """Return whether a cell index is inside this raster."""
        return (
            0 <= int(cell_x) < self.width
            and 0 <= int(cell_y) < self.height
        )


def rasterize_online_occupancy(
    points: Iterable[Sequence[float]],
    center_xy: Tuple[float, float],
    resolution: float,
    local_size: float,
    minimum_z: float,
    maximum_z: float,
) -> Tuple[np.ndarray, GridGeometry, int]:
    """
    Project SCAN occupied voxels into a robot-centred in-memory raster.

    The input points and ``center_xy`` must use the same world frame.  Only
    voxels intersecting the configured M20 body-height band are projected;
    floor and ceiling returns therefore do not become planar obstacles.
    ``occupied_count`` reports how many finite input voxels were accepted.
    """
    resolution = float(resolution)
    local_size = float(local_size)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('resolution must be finite and positive')
    if not math.isfinite(local_size) or local_size < 2.0 * resolution:
        raise ValueError('local_size must cover at least two cells')
    if not all(math.isfinite(value) for value in center_xy):
        raise ValueError('center_xy must be finite')
    if not math.isfinite(minimum_z) or not math.isfinite(maximum_z):
        raise ValueError('height limits must be finite')
    if maximum_z < minimum_z:
        raise ValueError('maximum_z must not be below minimum_z')

    cell_count = max(2, int(math.ceil(local_size / resolution)))
    extent = cell_count * resolution
    geometry = GridGeometry(
        width=cell_count,
        height=cell_count,
        resolution=resolution,
        origin_x=float(center_xy[0]) - 0.5 * extent,
        origin_y=float(center_xy[1]) - 0.5 * extent,
        # This is a moving local safety window.  Reaching its edge means the
        # online SCAN map is not covering the predicted motion, so fail closed.
        edge_tolerance=0.0,
    )
    occupancy = np.zeros(cell_count * cell_count, dtype=np.int8)
    accepted = 0
    for point in points:
        if len(point) < 3:
            continue
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if z < minimum_z or z > maximum_z:
            continue
        cell_x, cell_y = geometry.cell(x, y)
        if not geometry.contains(cell_x, cell_y):
            continue
        occupancy[cell_y * geometry.width + cell_x] = 100
        accepted += 1
    return occupancy, geometry, accepted


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


def conservative_raster_radius(
    footprint_radius: float,
    resolution: float,
) -> float:
    """
    Add half a cell diagonal to a continuous footprint before rasterizing.

    A metric circle centre can lie at a cell corner while collision lookup
    uses only the cell index.  Without this allowance a continuous overlap of
    up to half the cell diagonal can still be reported clear.
    """
    return max(0.0, float(footprint_radius)) + (
        max(0.0, float(resolution)) / math.sqrt(2.0)
    )


def hard_body_raster_radius(
    footprint_radius: float,
    resolution: float,
) -> float:
    """Rasterize the physical body without the conservative half-cell shell."""
    return max(
        0.0,
        float(footprint_radius)
        - max(0.0, float(resolution)) / math.sqrt(2.0),
    )


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


def command_drift_variants(
    command: PlanarCommand,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
    reference_yaw_rate: float,
) -> Tuple[Tuple[str, PlanarCommand], ...]:
    """Return nominal and measured M20 lateral-drift command variants."""
    vx, vy, wz = command
    variants = [('nominal', (vx, vy, wz))]
    yaw_reference = max(1.0e-6, abs(float(reference_yaw_rate)))
    yaw_fraction = min(1.0, abs(float(wz)) / yaw_reference)
    if yaw_fraction <= 1.0e-6:
        return tuple(variants)

    measured_limit = (
        positive_yaw_lateral_drift
        if wz >= 0.0
        else negative_yaw_lateral_drift
    )
    measured = max(0.0, float(measured_limit)) * yaw_fraction
    opposite = (
        max(0.0, float(opposite_lateral_uncertainty)) * yaw_fraction
    )
    if measured > 1.0e-6:
        variants.append(
            ('measured_lateral_drift', (vx, vy + measured, wz))
        )
    if opposite > 1.0e-6:
        variants.append(
            ('opposite_lateral_uncertainty', (vx, vy - opposite, wz))
        )
    return tuple(variants)


def first_blocking_command_envelope(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
    reference_yaw_rate: float,
) -> BlockingSample | None:
    """Check nominal and measured-drift sweeps, returning the earliest hit."""
    earliest = None
    for model, variant in command_drift_variants(
        command,
        positive_yaw_lateral_drift,
        negative_yaw_lateral_drift,
        opposite_lateral_uncertainty,
        reference_yaw_rate,
    ):
        sample = first_blocking_footprint(
            blocked,
            geometry,
            predict_poses(*pose, variant, horizon, sample_period),
            footprint_offset,
            sample_period,
        )
        if sample is None:
            continue
        sample = replace(sample, motion_model=model)
        if earliest is None or sample.sample_index < earliest.sample_index:
            earliest = sample
    return earliest


def recovery_sweep_is_clear(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    recovery: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
) -> bool:
    """Check the complete conservative sweep of one recovery command."""
    forward, _, yaw_rate = recovery
    lateral_speeds = [0.0]
    if abs(yaw_rate) > 1.0e-6:
        measured_drift = (
            max(0.0, float(positive_yaw_lateral_drift))
            if yaw_rate > 0.0
            else max(0.0, float(negative_yaw_lateral_drift))
        )
        lateral_speeds.extend(
            (
                measured_drift,
                -max(0.0, float(opposite_lateral_uncertainty)),
            )
        )
    for lateral_speed in lateral_speeds:
        swept_command = (forward, lateral_speed, yaw_rate)
        recovery_block = first_blocking_footprint(
            blocked,
            geometry,
            predict_poses(
                *pose,
                swept_command,
                horizon,
                sample_period,
            ),
            footprint_offset,
            sample_period,
        )
        if recovery_block is not None:
            return False
    return True


def recovery_endpoint_is_clear(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    recovery: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
) -> bool:
    """Require every recovery motion variant to end in clear space."""
    forward, _, yaw_rate = recovery
    lateral_speeds = [0.0]
    if abs(yaw_rate) > 1.0e-6:
        measured_drift = (
            max(0.0, float(positive_yaw_lateral_drift))
            if yaw_rate > 0.0
            else max(0.0, float(negative_yaw_lateral_drift))
        )
        lateral_speeds.extend(
            (
                measured_drift,
                -max(0.0, float(opposite_lateral_uncertainty)),
            )
        )
    for lateral_speed in lateral_speeds:
        poses = list(
            predict_poses(
                *pose,
                (forward, lateral_speed, yaw_rate),
                horizon,
                sample_period,
            )
        )
        if first_blocking_footprint(
            blocked,
            geometry,
            [poses[-1]],
            footprint_offset,
            sample_period,
        ) is not None:
            return False
    return True


def safe_raster_shell_escape(
    conservative_blocked: np.ndarray,
    hard_body_blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    speed: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
) -> PlanarCommand | None:
    """
    Leave a raster-only safety shell without authorizing physical overlap.

    Normal commands continue to use the conservative grid. This bounded seam
    applies only when that grid marks the current double-circle footprint but
    the hard physical-body grid does not. A straight direction is accepted
    only when its complete hard-body sweep is clear and its endpoint has
    returned to conservative free space. Reverse is tried first because the
    validated M20 policy can leave a front-side shell without zero-radius yaw.
    """
    current_pose = [pose]
    if first_blocking_footprint(
        conservative_blocked,
        geometry,
        current_pose,
        footprint_offset,
        sample_period,
    ) is None:
        return None
    if first_blocking_footprint(
        hard_body_blocked,
        geometry,
        current_pose,
        footprint_offset,
        sample_period,
    ) is not None:
        return None
    escape_speed = abs(float(speed))
    if escape_speed <= 1.0e-3:
        return None
    for recovery in (
        (-escape_speed, 0.0, 0.0),
        (escape_speed, 0.0, 0.0),
    ):
        if not recovery_sweep_is_clear(
            hard_body_blocked,
            geometry,
            pose,
            recovery,
            horizon,
            sample_period,
            footprint_offset,
            positive_yaw_lateral_drift,
            negative_yaw_lateral_drift,
            opposite_lateral_uncertainty,
        ):
            continue
        if recovery_endpoint_is_clear(
            conservative_blocked,
            geometry,
            pose,
            recovery,
            horizon,
            sample_period,
            footprint_offset,
            positive_yaw_lateral_drift,
            negative_yaw_lateral_drift,
            opposite_lateral_uncertainty,
        ):
            return recovery
    return None


def update_clear_confirmation(
    first_clear_time: float | None,
    now: float,
    confirmation_sec: float,
) -> Tuple[float | None, bool]:
    """Track whether recovery has seen a continuous clear interval."""
    duration = max(0.0, float(confirmation_sec))
    current_time = float(now)
    if duration <= 0.0:
        return None, True
    if first_clear_time is None or current_time < first_clear_time:
        first_clear_time = current_time
    confirmed = current_time - first_clear_time >= duration
    return first_clear_time, confirmed


def update_recovery_budget(
    start_time: float,
    start_xy: Tuple[float, float],
    progress_time: float,
    progress_xy: Tuple[float, float],
    now: float,
    current_xy: Tuple[float, float],
    max_active_sec: float,
    max_displacement_m: float,
    progress_timeout_sec: float,
    minimum_progress_m: float,
) -> Tuple[float, Tuple[float, float], str | None]:
    """Update bounded-recovery progress and return an exhaustion reason."""
    scalars = (
        start_time,
        start_xy[0],
        start_xy[1],
        progress_time,
        progress_xy[0],
        progress_xy[1],
        now,
        current_xy[0],
        current_xy[1],
    )
    if not all(math.isfinite(float(value)) for value in scalars):
        return progress_time, progress_xy, 'INVALID_STATE'
    if now < start_time or now < progress_time:
        return progress_time, progress_xy, 'CLOCK_REWIND'

    total_displacement = math.hypot(
        current_xy[0] - start_xy[0],
        current_xy[1] - start_xy[1],
    )
    if total_displacement >= max(0.0, float(max_displacement_m)):
        return progress_time, progress_xy, 'DISTANCE_LIMIT'
    if now - start_time >= max(0.0, float(max_active_sec)):
        return progress_time, progress_xy, 'TIME_LIMIT'

    checkpoint_displacement = math.hypot(
        current_xy[0] - progress_xy[0],
        current_xy[1] - progress_xy[1],
    )
    if checkpoint_displacement >= max(0.0, float(minimum_progress_m)):
        progress_time = now
        progress_xy = current_xy
    elif now - progress_time >= max(
        0.0, float(progress_timeout_sec)
    ):
        return progress_time, progress_xy, 'NO_PROGRESS'
    return progress_time, progress_xy, None


def safe_rolling_recovery(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    forward_speed: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
    minimum_yaw_rate: float,
) -> PlanarCommand | None:
    """
    Return a collision-checked M20 command that can break a predicted hold.

    The official M20 policy used by this project is not reliable under a
    zero-forward pure-yaw command.  Recovery therefore uses the validated
    positive-forward rolling envelope.  The opposite yaw direction is tried
    first to turn away from the rejected sweep.  Nominal and measured
    direction-dependent lateral-drift trajectories must remain clear.  A
    small opposite drift also covers residual uncertainty.  A reverse
    tracking request first checks a zero-yaw forward command; this lets the
    robot move inward from a shared map edge without continuing the rejected
    reverse turn. Straight reverse remains the final checked escape.  Reverse
    tracking may use only these zero-yaw checked escapes because the M20
    reverse-yaw drift envelope is not validated.  A currently blocked
    footprint is never recoverable by this policy.
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
    recovery_forward = float(forward_speed)
    if (
        abs(float(command[0])) <= 1.0e-3
        or recovery_forward <= 1.0e-3
    ):
        return None

    if (
        float(command[0]) > 1.0e-3
        and abs(requested_yaw) >= max(1.0e-3, minimum_yaw_rate)
    ):
        for yaw_rate in (-requested_yaw, requested_yaw):
            recovery = (recovery_forward, 0.0, yaw_rate)
            if recovery_sweep_is_clear(
                blocked,
                geometry,
                pose,
                recovery,
                horizon,
                sample_period,
                footprint_offset,
                positive_yaw_lateral_drift,
                negative_yaw_lateral_drift,
                opposite_lateral_uncertainty,
            ):
                return recovery

    if float(command[0]) < -1.0e-3:
        straight_forward = (recovery_forward, 0.0, 0.0)
        if recovery_sweep_is_clear(
            blocked,
            geometry,
            pose,
            straight_forward,
            horizon,
            sample_period,
            footprint_offset,
            positive_yaw_lateral_drift,
            negative_yaw_lateral_drift,
            opposite_lateral_uncertainty,
        ):
            return straight_forward

    # The cold-start envelope test showed that -0.35 m/s remains posture
    # stable while producing enough measured reverse motion to leave a
    # front-blocked planning boundary.  Keep yaw zero here: reverse-yaw has a
    # larger, direction-dependent lateral drift and is unnecessary for this
    # escape.  As with rolling recovery, authorization is valid only for the
    # current pose and is recomputed by the guard on every update.
    reverse = (-recovery_forward, 0.0, 0.0)
    if recovery_sweep_is_clear(
        blocked,
        geometry,
        pose,
        reverse,
        horizon,
        sample_period,
        footprint_offset,
        positive_yaw_lateral_drift,
        negative_yaw_lateral_drift,
        opposite_lateral_uncertainty,
    ):
        return reverse
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
