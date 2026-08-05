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

"""Pure target-direction policy for the rolling M20 execution boundary."""

import math
from typing import Tuple


Point = Tuple[float, float]


def normalize_angle(angle: float) -> float:
    """Wrap one angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a normalized or near-normalized quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def goal_bearing_error(start: Point, yaw: float, goal: Point) -> float:
    """Return the signed bearing from body forward axis to a position goal."""
    bearing = math.atan2(goal[1] - start[1], goal[0] - start[0])
    return normalize_angle(bearing - yaw)


def rear_goal_requires_route(
    start: Point,
    yaw: float,
    goal: Point,
    trigger_angle: float,
    minimum_distance: float,
) -> bool:
    """Select bidirectional routing for a meaningful rear-sector target."""
    if math.dist(start, goal) < max(0.0, minimum_distance):
        return False
    return abs(goal_bearing_error(start, yaw, goal)) >= min(
        math.pi, max(0.0, trigger_angle)
    )
