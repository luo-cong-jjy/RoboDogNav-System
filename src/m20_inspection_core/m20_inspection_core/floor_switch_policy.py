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

"""Pure transition-policy helpers shared by the floor switch action node."""

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence, Tuple


Pose2D = Tuple[float, float, float]


@dataclass(frozen=True)
class TransitionPlan:
    """Resolved directional elevator transition."""

    elevator_id: str
    source_floor: str
    target_floor: str
    source_trigger: Pose2D
    target_release: Pose2D
    trigger_tolerance_xy: float
    trigger_tolerance_yaw: float


def _pose3(values: Sequence[float], field: str) -> Pose2D:
    if len(values) != 3:
        raise ValueError(f'{field} must contain x, y, yaw')
    pose = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in pose):
        raise ValueError(f'{field} contains a non-finite value')
    return pose  # type: ignore[return-value]


def resolve_transition(
    config: Mapping[str, Any],
    elevator_id: str,
    source_floor: str,
    target_floor: str,
) -> TransitionPlan:
    """Resolve forward or explicitly enabled reverse elevator travel."""
    elevators = config.get('elevators', {})
    floors = config.get('floors', {})
    if elevator_id not in elevators:
        raise ValueError(f'unknown elevator {elevator_id!r}')
    if source_floor not in floors or target_floor not in floors:
        raise ValueError('source_floor or target_floor is unknown')
    if source_floor == target_floor:
        raise ValueError('source_floor and target_floor must differ')

    elevator = elevators[elevator_id]
    forward = (
        source_floor == elevator['source_floor']
        and target_floor == elevator['target_floor']
    )
    reverse = (
        source_floor == elevator['target_floor']
        and target_floor == elevator['source_floor']
    )
    if forward:
        trigger = _pose3(
            elevator['source_trigger_pose'], 'source_trigger_pose'
        )
        release = _pose3(
            elevator['target_release_pose'], 'target_release_pose'
        )
    elif reverse and bool(elevator.get('reverse_transition_enabled', False)):
        trigger = _pose3(
            floors[source_floor]['elevator_cabin_pose'],
            'reverse source elevator_cabin_pose',
        )
        release = _pose3(
            floors[target_floor]['elevator_lobby_pose'],
            'reverse target elevator_lobby_pose',
        )
    else:
        raise ValueError(
            f'elevator {elevator_id!r} does not support '
            f'{source_floor}->{target_floor}'
        )

    return TransitionPlan(
        elevator_id=elevator_id,
        source_floor=source_floor,
        target_floor=target_floor,
        source_trigger=trigger,
        target_release=release,
        trigger_tolerance_xy=float(elevator['trigger_tolerance_xy']),
        trigger_tolerance_yaw=float(elevator['trigger_tolerance_yaw']),
    )


def pose_error(current: Pose2D, target: Pose2D) -> Tuple[float, float]:
    """Return planar distance and wrapped absolute yaw error."""
    distance = math.hypot(current[0] - target[0], current[1] - target[1])
    yaw_error = abs(
        math.atan2(
            math.sin(current[2] - target[2]),
            math.cos(current[2] - target[2]),
        )
    )
    return distance, yaw_error
