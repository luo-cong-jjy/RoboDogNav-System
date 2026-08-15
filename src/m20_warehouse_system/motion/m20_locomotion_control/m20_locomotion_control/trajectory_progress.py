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

"""Pure geometry for tracking a SCAN B-spline by measured spatial progress."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


Point2D = Tuple[float, float]
PlanarCommand = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_point(point: Sequence[float]) -> Point2D:
    if len(point) < 2:
        raise ValueError('B-spline control points must have x and y')
    result = (float(point[0]), float(point[1]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError('B-spline control points must be finite')
    return result


class Bspline2D:
    """Small dependency-free mirror of SCAN's de Boor evaluator."""

    def __init__(
        self,
        control_points: Sequence[Sequence[float]],
        order: int,
        knots: Sequence[float],
    ) -> None:
        self.control_points = tuple(
            _finite_point(point) for point in control_points
        )
        self.order = int(order)
        self.knots = tuple(float(value) for value in knots)
        if self.order < 1:
            raise ValueError('B-spline order must be positive')
        if len(self.control_points) < self.order + 1:
            raise ValueError('B-spline has too few control points')
        expected_knots = len(self.control_points) + self.order + 1
        if len(self.knots) != expected_knots:
            raise ValueError(
                'B-spline knot count must equal point count + order + 1'
            )
        if not all(math.isfinite(value) for value in self.knots):
            raise ValueError('B-spline knots must be finite')
        if any(
            right < left
            for left, right in zip(self.knots, self.knots[1:])
        ):
            raise ValueError('B-spline knots must be nondecreasing')
        self._start_knot = self.knots[self.order]
        self._end_knot = self.knots[-self.order - 1]
        if self._end_knot <= self._start_knot:
            raise ValueError('B-spline duration must be positive')

    @property
    def duration(self) -> float:
        """Return the SCAN trajectory time span."""
        return self._end_knot - self._start_knot

    def evaluate(self, time_sec: float) -> Point2D:
        """Evaluate at SCAN trajectory time using its de Boor convention."""
        knot_value = min(
            self._end_knot,
            max(self._start_knot, self._start_knot + float(time_sec)),
        )
        span = self.order
        while self.knots[span + 1] < knot_value:
            span += 1
        working = [
            list(self.control_points[span - self.order + index])
            for index in range(self.order + 1)
        ]
        for level in range(1, self.order + 1):
            for index in range(self.order, level - 1, -1):
                left_index = index + span - self.order
                right_index = index + 1 + span - level
                denominator = (
                    self.knots[right_index] - self.knots[left_index]
                )
                if denominator <= 0.0:
                    alpha = 0.0
                else:
                    alpha = (
                        knot_value - self.knots[left_index]
                    ) / denominator
                working[index][0] = (
                    (1.0 - alpha) * working[index - 1][0]
                    + alpha * working[index][0]
                )
                working[index][1] = (
                    (1.0 - alpha) * working[index - 1][1]
                    + alpha * working[index][1]
                )
        return (working[self.order][0], working[self.order][1])


@dataclass(frozen=True)
class PathPoint:
    """One time and arc-length indexed trajectory sample."""

    x: float
    y: float
    time_sec: float
    progress_m: float


@dataclass(frozen=True)
class PathProjection:
    """Closest point on a sampled path."""

    progress_m: float
    distance_m: float
    x: float
    y: float


class SampledPath:
    """Arc-length view of one SCAN local B-spline."""

    def __init__(self, points: Sequence[PathPoint]) -> None:
        self.points = tuple(points)
        if len(self.points) < 2:
            raise ValueError('sampled path requires at least two points')
        self.length_m = self.points[-1].progress_m
        if self.length_m <= 0.0:
            raise ValueError('sampled path must have positive length')
        time_delta = max(
            1.0e-6,
            self.points[-1].time_sec - self.points[-2].time_sec,
        )
        self.end_speed_mps = math.hypot(
            self.points[-1].x - self.points[-2].x,
            self.points[-1].y - self.points[-2].y,
        ) / time_delta

    @classmethod
    def from_spline(
        cls,
        spline: Bspline2D,
        sample_period_sec: float = 0.02,
    ) -> 'SampledPath':
        """Sample the exact B-spline densely enough for spatial projection."""
        period = max(0.005, float(sample_period_sec))
        sample_count = max(2, math.ceil(spline.duration / period) + 1)
        points = []
        progress = 0.0
        previous = None
        for index in range(sample_count):
            time_sec = spline.duration * index / (sample_count - 1)
            current = spline.evaluate(time_sec)
            if previous is not None:
                progress += math.hypot(
                    current[0] - previous[0],
                    current[1] - previous[1],
                )
            points.append(
                PathPoint(current[0], current[1], time_sec, progress)
            )
            previous = current
        return cls(points)

    def closest_projection(
        self,
        x: float,
        y: float,
        minimum_progress_m: float = 0.0,
    ) -> PathProjection:
        """Project a measured position onto the non-regressing path suffix."""
        minimum = max(0.0, min(float(minimum_progress_m), self.length_m))
        best = None
        for left, right in zip(self.points, self.points[1:]):
            if right.progress_m < minimum:
                continue
            dx = right.x - left.x
            dy = right.y - left.y
            segment_sq = dx * dx + dy * dy
            if segment_sq <= 1.0e-12:
                continue
            ratio = ((x - left.x) * dx + (y - left.y) * dy) / segment_sq
            ratio = max(0.0, min(1.0, ratio))
            progress = left.progress_m + ratio * (
                right.progress_m - left.progress_m
            )
            if progress < minimum:
                ratio = (
                    (minimum - left.progress_m)
                    / max(1.0e-12, right.progress_m - left.progress_m)
                )
                ratio = max(0.0, min(1.0, ratio))
                progress = minimum
            projected_x = left.x + ratio * dx
            projected_y = left.y + ratio * dy
            distance = math.hypot(x - projected_x, y - projected_y)
            candidate = PathProjection(
                progress, distance, projected_x, projected_y
            )
            if best is None or candidate.distance_m < best.distance_m:
                best = candidate
        if best is not None:
            return best
        final = self.points[-1]
        return PathProjection(
            final.progress_m,
            math.hypot(x - final.x, y - final.y),
            final.x,
            final.y,
        )

    def point_at(self, progress_m: float) -> Point2D:
        """Interpolate a point at arc-length progress."""
        progress = max(0.0, min(float(progress_m), self.length_m))
        for left, right in zip(self.points, self.points[1:]):
            if right.progress_m < progress:
                continue
            span = right.progress_m - left.progress_m
            ratio = 0.0 if span <= 1.0e-12 else (
                progress - left.progress_m
            ) / span
            return (
                left.x + ratio * (right.x - left.x),
                left.y + ratio * (right.y - left.y),
            )
        final = self.points[-1]
        return (final.x, final.y)


@dataclass(frozen=True)
class ProgressFollowerParameters:
    """Spatial follower settings independent of the M20 output envelope."""

    lookahead_m: float = 0.60
    max_speed_mps: float = 0.45
    yaw_gain: float = 1.50
    max_yaw_radps: float = 0.65
    finish_distance_m: float = 0.15
    terminal_speed_threshold_mps: float = 0.10
    terminal_slowdown_distance_m: float = 0.80
    terminal_min_speed_mps: float = 0.10
    reverse_tracking_enabled: bool = True
    reverse_enter_angle_rad: float = 2.10
    reverse_exit_angle_rad: float = 1.75


@dataclass(frozen=True)
class ProgressCommand:
    """Follower output and diagnostic state for one measured pose."""

    command: PlanarCommand
    progress_m: float
    remaining_m: float
    cross_track_m: float
    target: Point2D
    heading_error_rad: float
    reverse_tracking: bool
    finished: bool


class SpatialProgressFollower:
    """Generate a rolling command from measured path progress."""

    def __init__(self, parameters: ProgressFollowerParameters) -> None:
        self.parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Clear per-trajectory progress and direction state."""
        self.progress_m = 0.0
        self.reverse_tracking = False

    def start_trajectory(self) -> None:
        """Reset only spatial progress when SCAN publishes a new local path."""
        self.progress_m = 0.0

    def update(
        self,
        path: SampledPath,
        x: float,
        y: float,
        yaw: float,
    ) -> ProgressCommand:
        """Track the nearest measured progress and a fixed spatial lookahead."""
        projection = path.closest_projection(
            x,
            y,
            max(0.0, self.progress_m - 0.05),
        )
        self.progress_m = max(self.progress_m, projection.progress_m)
        target_progress = min(
            path.length_m,
            self.progress_m + max(0.05, self.parameters.lookahead_m),
        )
        target = path.point_at(target_progress)
        end = path.point_at(path.length_m)
        distance_to_end = math.hypot(end[0] - x, end[1] - y)
        remaining = max(0.0, path.length_m - self.progress_m)
        terminal = (
            path.end_speed_mps
            <= self.parameters.terminal_speed_threshold_mps
        )
        finished = (
            terminal
            and distance_to_end <= self.parameters.finish_distance_m
        )
        if finished:
            return ProgressCommand(
                (0.0, 0.0, 0.0),
                self.progress_m,
                remaining,
                projection.distance_m,
                target,
                0.0,
                self.reverse_tracking,
                True,
            )

        target_dx = target[0] - x
        target_dy = target[1] - y
        if math.hypot(target_dx, target_dy) < 0.03:
            behind = path.point_at(max(0.0, target_progress - 0.10))
            target_dx = target[0] - behind[0]
            target_dy = target[1] - behind[1]
        course_yaw = math.atan2(target_dy, target_dx)
        forward_error = normalize_angle(course_yaw - yaw)
        if self.parameters.reverse_tracking_enabled:
            if (
                not self.reverse_tracking
                and abs(forward_error)
                >= self.parameters.reverse_enter_angle_rad
            ):
                self.reverse_tracking = True
            elif (
                self.reverse_tracking
                and abs(forward_error)
                <= self.parameters.reverse_exit_angle_rad
            ):
                self.reverse_tracking = False
        else:
            self.reverse_tracking = False

        tracking_yaw = normalize_angle(
            course_yaw + (math.pi if self.reverse_tracking else 0.0)
        )
        heading_error = normalize_angle(tracking_yaw - yaw)
        speed = max(0.0, self.parameters.max_speed_mps)
        if terminal:
            slowdown = max(
                self.parameters.finish_distance_m,
                self.parameters.terminal_slowdown_distance_m,
            )
            ratio = max(0.0, min(1.0, distance_to_end / slowdown))
            speed = min(
                speed,
                max(
                    self.parameters.terminal_min_speed_mps,
                    self.parameters.max_speed_mps * ratio,
                ),
            )
        if self.reverse_tracking:
            speed = -speed
        yaw_rate = max(
            -self.parameters.max_yaw_radps,
            min(
                self.parameters.max_yaw_radps,
                self.parameters.yaw_gain * heading_error,
            ),
        )
        return ProgressCommand(
            (speed, 0.0, yaw_rate),
            self.progress_m,
            remaining,
            projection.distance_m,
            target,
            heading_error,
            self.reverse_tracking,
            False,
        )
