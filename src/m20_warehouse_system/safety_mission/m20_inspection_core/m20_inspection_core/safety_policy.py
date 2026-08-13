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

"""Pure command limiting and source selection for the safety supervisor."""

from dataclasses import dataclass
from typing import Optional, Tuple


PlanarCommand = Tuple[float, float, float]


@dataclass(frozen=True)
class TimedCommand:
    """A planar command and its monotonic receipt time in seconds."""

    command: PlanarCommand
    stamp: float


def clamp_command(
    command: PlanarCommand,
    limits: PlanarCommand,
) -> PlanarCommand:
    """Clamp x/y/yaw command components symmetrically."""
    return tuple(
        max(-abs(limit), min(abs(limit), value))
        for value, limit in zip(command, limits)
    )


def slew_command(
    current: PlanarCommand,
    target: PlanarCommand,
    dt: float,
    linear_acceleration: float,
    angular_acceleration: float,
) -> PlanarCommand:
    """Apply ordinary linear and angular acceleration limits."""
    if dt <= 0.0:
        return target
    deltas = (
        max(0.0, linear_acceleration) * dt,
        max(0.0, linear_acceleration) * dt,
        max(0.0, angular_acceleration) * dt,
    )
    output = []
    for old, new, maximum_delta in zip(current, target, deltas):
        difference = max(-maximum_delta, min(maximum_delta, new - old))
        output.append(old + difference)
    return tuple(output)


def proportional_ramp_command(
    current: PlanarCommand,
    target: PlanarCommand,
    dt: float,
    ramp_duration: float,
) -> PlanarCommand:
    """
    Ramp all components by one scale so command curvature stays constant.

    Collision recovery starts from a hard stop.  If its direction changes,
    restart from zero rather than crossing through an unvalidated mixed
    command.
    """
    if ramp_duration <= 0.0:
        return target
    if dt <= 0.0:
        return current
    norm_squared = sum(value * value for value in target)
    if norm_squared <= 1.0e-12:
        return target
    alignment = sum(
        old * new for old, new in zip(current, target)
    )
    if alignment <= 0.0:
        current_scale = 0.0
    else:
        current_scale = max(
            0.0,
            min(
                1.0,
                alignment / norm_squared,
            ),
        )
    scale = min(1.0, current_scale + dt / ramp_duration)
    return tuple(scale * value for value in target)


def valid_collision_recovery_command(command: PlanarCommand) -> bool:
    """
    Accept only recovery shapes validated for the current M20 policy.

    Forward recovery may be straight or a rolling turn after the collision
    guard validates its complete sweep. Straight reverse is also allowed as a
    checked escape. Pure yaw, lateral motion, and reverse-yaw remain forbidden.
    """
    vx, vy, wz = command
    epsilon = 1.0e-3
    if abs(vy) > 1.0e-9 or abs(vx) <= epsilon:
        return False
    if vx < -epsilon:
        return abs(wz) <= epsilon
    return True


def select_fresh_command(
    now: float,
    timeout: float,
    navigation: Optional[TimedCommand],
    manual: Optional[TimedCommand],
    manual_priority: bool,
) -> Tuple[PlanarCommand, str]:
    """Select the highest-priority command that has not timed out."""

    def fresh(sample: Optional[TimedCommand]) -> bool:
        return sample is not None and 0.0 <= now - sample.stamp <= timeout

    ordered = (
        (('MANUAL', manual), ('NAVIGATION', navigation))
        if manual_priority
        else (('NAVIGATION', navigation), ('MANUAL', manual))
    )
    for source, sample in ordered:
        if fresh(sample):
            return sample.command, source
    return (0.0, 0.0, 0.0), 'COMMAND_TIMEOUT'
