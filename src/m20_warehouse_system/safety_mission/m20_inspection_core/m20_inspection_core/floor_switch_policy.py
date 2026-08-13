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

"""Resolve floor routes and replaceable transport policies from YAML."""

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence, Tuple


Pose2D = Tuple[float, float, float]


@dataclass(frozen=True)
class TransitionPlan:
    """Resolved directional route plus its transport/handoff strategy."""

    connector_id: str
    source_floor: str
    target_floor: str
    source_trigger: Pose2D
    target_release: Pose2D
    trigger_tolerance_xy: float
    trigger_tolerance_yaw: float
    transfer_adapter: str
    pose_handoff: str
    transition_delay_sec: float
    transfer_timeout_sec: float
    external_action_name: str


TRANSFER_ADAPTERS = frozenset({'timed_hold', 'external_action'})
POSE_HANDOFFS = frozenset(
    {'preserve', 'set_simulation_pose', 'wait_for_target', 'none'}
)


def _pose3(values: Sequence[float], field: str) -> Pose2D:
    if len(values) != 3:
        raise ValueError(f'{field} must contain x, y, yaw')
    pose = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in pose):
        raise ValueError(f'{field} contains a non-finite value')
    return pose  # type: ignore[return-value]


def _finite_number(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field} must be a number') from error
    if not math.isfinite(result):
        raise ValueError(f'{field} contains a non-finite value')
    return result


def _transfer_policy(
    config: Mapping[str, Any],
    connector: Mapping[str, Any],
    route: Mapping[str, Any],
) -> Tuple[str, str, float, float, str]:
    """
    Merge system, connector, and route transport settings.

    The legacy simulation keys remain a compatibility fallback for old test
    profiles. New profiles should use ``floor_switch`` and optional
    ``transfer`` overrides on a connector or an individual route.
    """
    simulation = config.get('simulation', {})
    transaction = config.get('map_switch_transaction', {})
    settings = {
        'adapter': 'timed_hold',
        'pose_handoff': (
            'set_simulation_pose'
            if bool(simulation.get('teleport_on_floor_switch', False))
            else 'preserve'
        ),
        'transition_delay_sec': simulation.get(
            'elevator_transition_delay_sec', 0.0
        ),
        'timeout_sec': transaction.get('relocate_timeout_sec', 30.0),
        'external_action_name': '/m20/floor_transfer/execute',
    }
    system_policy = config.get('floor_switch', {})
    if isinstance(system_policy, Mapping):
        aliases = {
            'transfer_adapter': 'adapter',
            'pose_handoff': 'pose_handoff',
            'transition_delay_sec': 'transition_delay_sec',
            'transfer_timeout_sec': 'timeout_sec',
            'external_action_name': 'external_action_name',
        }
        for source_key, target_key in aliases.items():
            if source_key in system_policy:
                settings[target_key] = system_policy[source_key]
    for owner, path in (
        (connector, 'connector.transfer'),
        (route, 'route.transfer'),
    ):
        override = owner.get('transfer', {})
        if override is None:
            continue
        if not isinstance(override, Mapping):
            raise ValueError(f'{path} must be a mapping')
        for key in settings:
            if key in override:
                settings[key] = override[key]

    adapter = str(settings['adapter']).strip()
    handoff = str(settings['pose_handoff']).strip()
    action_name = str(settings['external_action_name']).strip()
    delay = _finite_number(
        settings['transition_delay_sec'], 'transition_delay_sec'
    )
    timeout = _finite_number(settings['timeout_sec'], 'transfer timeout_sec')
    if adapter not in TRANSFER_ADAPTERS:
        raise ValueError(
            f'unsupported transfer adapter {adapter!r}; '
            f'expected one of {sorted(TRANSFER_ADAPTERS)}'
        )
    if handoff not in POSE_HANDOFFS:
        raise ValueError(
            f'unsupported pose handoff {handoff!r}; '
            f'expected one of {sorted(POSE_HANDOFFS)}'
        )
    if delay < 0.0:
        raise ValueError('transition_delay_sec cannot be negative')
    if timeout <= 0.0:
        raise ValueError('transfer timeout_sec must be greater than zero')
    if adapter == 'external_action' and not action_name:
        raise ValueError('external_action_name cannot be empty')
    return adapter, handoff, delay, timeout, action_name


def resolve_transition(
    config: Mapping[str, Any],
    connector_id: str,
    source_floor: str,
    target_floor: str,
) -> TransitionPlan:
    """Resolve an explicit route or a legacy bidirectional connector."""
    connectors = config.get('elevators', {})
    floors = config.get('floors', {})
    if connector_id not in connectors:
        raise ValueError(f'unknown floor connector {connector_id!r}')
    if source_floor not in floors or target_floor not in floors:
        raise ValueError('source_floor or target_floor is unknown')
    if source_floor == target_floor:
        raise ValueError('source_floor and target_floor must differ')

    connector = connectors[connector_id]
    if not isinstance(connector, Mapping):
        raise ValueError(f'connector {connector_id!r} must be a mapping')
    route: Mapping[str, Any] = {}
    routes = connector.get('transitions')
    if routes is not None:
        if not isinstance(routes, list) or not routes:
            raise ValueError(
                f'{connector_id}.transitions must be a non-empty list'
            )
        matches = [
            candidate
            for candidate in routes
            if isinstance(candidate, Mapping)
            and str(candidate.get('from', '')) == source_floor
            and str(candidate.get('to', '')) == target_floor
        ]
        if len(matches) != 1:
            qualifier = 'duplicate' if len(matches) > 1 else 'no'
            raise ValueError(
                f'connector {connector_id!r} has {qualifier} route for '
                f'{source_floor}->{target_floor}'
            )
        route = matches[0]
        trigger = _pose3(
            route['source_trigger_pose'],
            f'{connector_id} {source_floor}->{target_floor} '
            'source_trigger_pose',
        )
        release = _pose3(
            route['target_release_pose'],
            f'{connector_id} {source_floor}->{target_floor} '
            'target_release_pose',
        )
    else:
        forward = (
            source_floor == connector['source_floor']
            and target_floor == connector['target_floor']
        )
        reverse = (
            source_floor == connector['target_floor']
            and target_floor == connector['source_floor']
        )
        if forward:
            route = connector
            trigger = _pose3(
                connector['source_trigger_pose'], 'source_trigger_pose'
            )
            release = _pose3(
                connector['target_release_pose'], 'target_release_pose'
            )
        elif reverse and bool(
            connector.get('reverse_transition_enabled', False)
        ):
            trigger = _pose3(
                connector.get(
                    'reverse_source_trigger_pose',
                    floors[source_floor]['elevator_cabin_pose'],
                ),
                'reverse source trigger pose',
            )
            release = _pose3(
                connector.get(
                    'reverse_target_release_pose',
                    floors[target_floor]['elevator_lobby_pose'],
                ),
                'reverse target release pose',
            )
        else:
            raise ValueError(
                f'connector {connector_id!r} does not support '
                f'{source_floor}->{target_floor}'
            )

    tolerance_xy = _finite_number(
        route.get(
            'trigger_tolerance_xy', connector.get('trigger_tolerance_xy')
        ),
        'trigger_tolerance_xy',
    )
    tolerance_yaw = _finite_number(
        route.get(
            'trigger_tolerance_yaw', connector.get('trigger_tolerance_yaw')
        ),
        'trigger_tolerance_yaw',
    )
    if tolerance_xy <= 0.0 or tolerance_yaw <= 0.0:
        raise ValueError('trigger tolerances must be greater than zero')
    adapter, handoff, delay, timeout, action_name = _transfer_policy(
        config, connector, route
    )

    return TransitionPlan(
        connector_id=connector_id,
        source_floor=source_floor,
        target_floor=target_floor,
        source_trigger=trigger,
        target_release=release,
        trigger_tolerance_xy=tolerance_xy,
        trigger_tolerance_yaw=tolerance_yaw,
        transfer_adapter=adapter,
        pose_handoff=handoff,
        transition_delay_sec=delay,
        transfer_timeout_sec=timeout,
        external_action_name=action_name,
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
