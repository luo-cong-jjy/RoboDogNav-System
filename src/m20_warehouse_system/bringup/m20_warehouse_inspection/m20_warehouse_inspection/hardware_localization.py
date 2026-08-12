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

"""Pure helpers for the real-robot localization boundary."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


PlanarTwist = Tuple[float, float, float]


@dataclass(frozen=True)
class PoseSample:
    """Timestamped planar pose expressed in the navigation world frame."""

    stamp_sec: float
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    """Return an angle in ``[-pi, pi)``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a normalized or near-normalized quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def finite_planar_twist(values: PlanarTwist) -> bool:
    """Check that a planar velocity sample is safe to forward."""
    return all(math.isfinite(value) for value in values)


def derive_body_twist(
    previous: Optional[PoseSample],
    current: PoseSample,
    minimum_dt: float = 0.02,
    maximum_dt: float = 0.50,
) -> Optional[PlanarTwist]:
    """
    Estimate child/body-frame planar twist from two world poses.

    Elevator-LIO's body Odometry currently carries the body pose but leaves
    its Twist at zero.  The factory motion-status velocity is preferred at
    runtime; this finite difference is the bounded fallback used before that
    status stream is ready or if it briefly becomes stale.
    """
    if previous is None:
        return None
    dt = current.stamp_sec - previous.stamp_sec
    if not math.isfinite(dt) or dt < minimum_dt or dt > maximum_dt:
        return None
    dx_world = current.x - previous.x
    dy_world = current.y - previous.y
    # Use the midpoint attitude to reduce turn-induced projection error.
    yaw_mid = previous.yaw + 0.5 * normalize_angle(
        current.yaw - previous.yaw
    )
    cosine = math.cos(yaw_mid)
    sine = math.sin(yaw_mid)
    linear_x = (cosine * dx_world + sine * dy_world) / dt
    linear_y = (-sine * dx_world + cosine * dy_world) / dt
    angular_z = normalize_angle(current.yaw - previous.yaw) / dt
    result = (linear_x, linear_y, angular_z)
    return result if finite_planar_twist(result) else None
