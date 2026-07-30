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

"""Pure configuration resolver for deterministic inspection mission steps."""

from dataclasses import dataclass
import math
from typing import Any, Mapping, Tuple


Pose2D = Tuple[float, float, float]


@dataclass(frozen=True)
class MissionStep:
    """Resolved mission step without ROS runtime objects."""

    step_type: str
    name: str
    floor_id: str
    pose: Pose2D
    dwell_sec: float
    elevator_id: str = ''
    target_floor: str = ''


def _pose(values, field: str) -> Pose2D:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f'{field} must contain x, y, yaw')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{field} contains a non-finite value')
    return result  # type: ignore[return-value]


def display_step_index(step_index: int, step_count: int) -> int:
    """Clamp an internal cursor to a valid zero-based display step."""
    if step_count <= 0:
        return 0
    return max(0, min(int(step_index), int(step_count) - 1))


def resolve_mission(
    config: Mapping[str, Any], mission_id: str
) -> Tuple[MissionStep, ...]:
    """Resolve named points and elevator references into executable steps."""
    mission = config.get('mission', {})
    if mission_id != str(mission.get('id', '')):
        raise ValueError(f'unknown mission {mission_id!r}')
    floors = config.get('floors', {})
    steps = []
    for index, raw in enumerate(mission.get('sequence', [])):
        step_type = raw.get('type')
        if step_type in {'inspection', 'transit'}:
            floor_id = str(raw['floor'])
            point_name = str(raw['point'])
            points = {
                str(point['name']): point
                for point in floors[floor_id]['inspection_points']
            }
            if point_name not in points:
                raise ValueError(
                    f'mission step {index} has unknown point {point_name!r}'
                )
            point = points[point_name]
            step_name = (
                point_name
                if step_type == 'inspection'
                else str(raw['name']).strip()
            )
            steps.append(
                MissionStep(
                    step_type=str(step_type),
                    name=step_name,
                    floor_id=floor_id,
                    pose=_pose(point['pose'], f'{point_name}.pose'),
                    dwell_sec=(
                        max(0.0, float(point.get('dwell_sec', 0.0)))
                        if step_type == 'inspection'
                        else 0.0
                    ),
                )
            )
        elif step_type == 'elevator_transfer':
            source = str(raw['from'])
            target = str(raw['to'])
            elevator = str(raw['elevator'])
            steps.append(
                MissionStep(
                    step_type='elevator_transfer',
                    name=f'{elevator}:{source}->{target}',
                    floor_id=source,
                    pose=_pose(
                        floors[source]['elevator_lobby_pose'],
                        f'{source}.elevator_lobby_pose',
                    ),
                    dwell_sec=0.0,
                    elevator_id=elevator,
                    target_floor=target,
                )
            )
        elif step_type == 'terminal':
            floor_id = str(raw['floor'])
            location = str(raw.get('location', ''))
            pose_key_by_location = {
                'elevator_lobby': 'elevator_lobby_pose',
                'initial_pose': 'initial_pose',
            }
            if location not in pose_key_by_location:
                raise ValueError(
                    f'mission step {index} has unsupported terminal '
                    f'location {location!r}'
                )
            pose_key = pose_key_by_location[location]
            name = str(raw.get('name', '')).strip()
            if not name:
                raise ValueError(
                    f'mission step {index} terminal name is empty'
                )
            steps.append(
                MissionStep(
                    step_type='terminal',
                    name=name,
                    floor_id=floor_id,
                    pose=_pose(
                        floors[floor_id][pose_key],
                        f'{floor_id}.{pose_key}',
                    ),
                    dwell_sec=0.0,
                )
            )
        else:
            raise ValueError(
                f'mission step {index} has unsupported type {step_type!r}'
            )
    if not steps:
        raise ValueError('mission has no executable steps')
    return tuple(steps)
