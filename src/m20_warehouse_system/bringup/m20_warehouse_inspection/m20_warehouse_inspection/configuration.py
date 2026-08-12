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

"""Load and cross-check the system-level YAML configuration."""

import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml


class ConfigurationError(ValueError):
    """Report one or more invalid system-configuration fields."""


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _number(
    mapping: Mapping[str, Any],
    key: str,
    path: str,
    errors: List[str],
    *,
    positive: bool = False,
) -> float:
    value = mapping.get(key)
    if not _is_number(value):
        errors.append(f'{path}.{key} must be a number')
        return 0.0
    result = float(value)
    if positive and result <= 0.0:
        errors.append(f'{path}.{key} must be greater than zero')
    return result


def _sequence(
    value: Any,
    length: int,
    path: str,
    errors: List[str],
) -> List[float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
        or not all(_is_number(item) for item in value)
    ):
        errors.append(f'{path} must contain exactly {length} numbers')
        return [0.0] * length
    return [float(item) for item in value]


def _mapping(
    mapping: Mapping[str, Any],
    key: str,
    path: str,
    errors: List[str],
) -> Mapping[str, Any]:
    value = mapping.get(key)
    if not isinstance(value, Mapping):
        errors.append(f'{path}.{key} must be a mapping')
        return {}
    return value


def _safe_relative_path(value: Any, path: str, errors: List[str]) -> None:
    if not isinstance(value, str) or not value:
        errors.append(f'{path} must be a non-empty relative path')
        return
    candidate = Path(value)
    if candidate.is_absolute() or '..' in candidate.parts:
        errors.append(f'{path} must stay inside the package directory')


def _validate_transfer_settings(
    value: Any,
    path: str,
    errors: List[str],
    *,
    system_scope: bool = False,
) -> None:
    """Validate a system default or connector/route transfer override."""
    if value is None:
        return
    if not isinstance(value, Mapping):
        errors.append(f'{path} must be a mapping')
        return
    adapter_key = 'transfer_adapter' if system_scope else 'adapter'
    timeout_key = 'transfer_timeout_sec' if system_scope else 'timeout_sec'
    allowed = {
        adapter_key,
        'pose_handoff',
        'transition_delay_sec',
        timeout_key,
        'external_action_name',
    }
    for key in value:
        if key not in allowed:
            errors.append(f'{path}.{key} is not a supported transfer option')
    adapter = value.get(adapter_key)
    if adapter is not None and adapter not in {
        'timed_hold',
        'external_action',
    }:
        errors.append(
            f'{path}.{adapter_key} must equal timed_hold or external_action'
        )
    handoff = value.get('pose_handoff')
    if handoff is not None and handoff not in {
        'preserve',
        'set_simulation_pose',
        'wait_for_target',
        'none',
    }:
        errors.append(
            f'{path}.pose_handoff has an unsupported policy {handoff!r}'
        )
    delay = value.get('transition_delay_sec')
    if delay is not None and (not _is_number(delay) or float(delay) < 0.0):
        errors.append(f'{path}.transition_delay_sec must be non-negative')
    timeout = value.get(timeout_key)
    if timeout is not None and (
        not _is_number(timeout) or float(timeout) <= 0.0
    ):
        errors.append(f'{path}.{timeout_key} must be greater than zero')
    action_name = value.get('external_action_name')
    if action_name is not None and (
        not isinstance(action_name, str) or not action_name.strip()
    ):
        errors.append(
            f'{path}.external_action_name must be a non-empty string'
        )


def _pose_in_bounds(
    pose: Sequence[float],
    bounds: Tuple[float, float, float, float],
    path: str,
    errors: List[str],
) -> None:
    x_min, x_max, y_min, y_max = bounds
    if not x_min <= pose[0] <= x_max or not y_min <= pose[1] <= y_max:
        errors.append(
            f'{path} position ({pose[0]}, {pose[1]}) is outside its floor region'
        )


def floor_display_bounds(
    config: Mapping[str, Any],
    floor_id: str,
) -> Tuple[float, float, float, float]:
    """Return display-frame x/y bounds for a configured floor."""
    floor = config['floors'][floor_id]
    offset = floor['simulation_offset']
    simulation = config['simulation']
    half_x = float(simulation['floor_size_x']) / 2.0
    half_y = float(simulation['floor_size_y']) / 2.0
    return (
        float(offset[0]) - half_x,
        float(offset[0]) + half_x,
        float(offset[1]) - half_y,
        float(offset[1]) + half_y,
    )


def display_to_source_xy(
    floor: Mapping[str, Any],
    point: Sequence[float],
) -> Tuple[float, float]:
    """Remove a simulation-only floor offset from an x/y point."""
    offset = floor['simulation_offset']
    return float(point[0]) - float(offset[0]), float(point[1]) - float(offset[1])


def transition_route(
    config: Mapping[str, Any],
    connector_id: str,
    source_floor: str,
    target_floor: str,
) -> Mapping[str, Any]:
    """Return one explicit or legacy directional connector route."""
    connector = config['elevators'][connector_id]
    routes = connector.get('transitions')
    if isinstance(routes, list):
        matches = [
            route
            for route in routes
            if isinstance(route, Mapping)
            and route.get('from') == source_floor
            and route.get('to') == target_floor
        ]
        if len(matches) != 1:
            raise ConfigurationError(
                f'connector {connector_id!r} must define exactly one route '
                f'for {source_floor}->{target_floor}'
            )
        return matches[0]
    if (
        connector.get('source_floor') == source_floor
        and connector.get('target_floor') == target_floor
    ):
        return connector
    if (
        bool(connector.get('reverse_transition_enabled'))
        and connector.get('target_floor') == source_floor
        and connector.get('source_floor') == target_floor
    ):
        return {
            'from': source_floor,
            'to': target_floor,
            'source_trigger_pose': connector.get(
                'reverse_source_trigger_pose',
                config['floors'][source_floor]['elevator_cabin_pose'],
            ),
            'target_release_pose': connector.get(
                'reverse_target_release_pose',
                config['floors'][target_floor]['elevator_lobby_pose'],
            ),
        }
    raise ConfigurationError(
        f'connector {connector_id!r} does not support '
        f'{source_floor}->{target_floor}'
    )


def _validate_mission(
    config: Mapping[str, Any],
    floors: Mapping[str, Any],
    elevators: Mapping[str, Any],
    errors: List[str],
) -> None:
    mission = _mapping(config, 'mission', 'root', errors)
    sequence = mission.get('sequence')
    if not isinstance(sequence, list) or not sequence:
        errors.append('root.mission.sequence must be a non-empty list')
        return

    inspection_names = {
        floor_id: {
            point.get('name')
            for point in floor.get('inspection_points', [])
            if isinstance(point, Mapping)
        }
        for floor_id, floor in floors.items()
    }
    for index, step in enumerate(sequence):
        path = f'root.mission.sequence[{index}]'
        if not isinstance(step, Mapping):
            errors.append(f'{path} must be a mapping')
            continue
        step_type = step.get('type')
        if step_type in {'inspection', 'transit'}:
            floor_id = step.get('floor')
            if floor_id not in floors:
                errors.append(f'{path}.floor references unknown floor {floor_id!r}')
            elif step.get('point') not in inspection_names[floor_id]:
                errors.append(
                    f'{path}.point references unknown point {step.get("point")!r}'
                )
            if step_type == 'transit' and (
                not isinstance(step.get('name'), str)
                or not step['name'].strip()
            ):
                errors.append(f'{path}.name must be a non-empty string')
        elif step_type in {'elevator_transfer', 'floor_transfer'}:
            connector_key = (
                'elevator' if step_type == 'elevator_transfer' else 'connector'
            )
            elevator_id = step.get(connector_key)
            if elevator_id not in elevators:
                errors.append(
                    f'{path}.{connector_key} references unknown connector '
                    f'{elevator_id!r}'
                )
            if step.get('from') not in floors or step.get('to') not in floors:
                errors.append(f'{path} references an unknown floor')
            elif elevator_id in elevators and isinstance(
                elevators[elevator_id], Mapping
            ):
                connector = elevators[elevator_id]
                source = step.get('from')
                target = step.get('to')
                routes = connector.get('transitions')
                if isinstance(routes, list):
                    supported = any(
                        isinstance(route, Mapping)
                        and route.get('from') == source
                        and route.get('to') == target
                        for route in routes
                    )
                else:
                    supported = (
                        connector.get('source_floor') == source
                        and connector.get('target_floor') == target
                    ) or (
                        bool(connector.get('reverse_transition_enabled'))
                        and connector.get('target_floor') == source
                        and connector.get('source_floor') == target
                    )
                if not supported:
                    errors.append(
                        f'{path} references unsupported connector route '
                        f'{source!r}->{target!r}'
                    )
        elif step_type == 'terminal':
            floor_id = step.get('floor')
            if floor_id not in floors:
                errors.append(
                    f'{path}.floor references unknown floor {floor_id!r}'
                )
            if step.get('location') not in {
                'elevator_lobby',
                'initial_pose',
            }:
                errors.append(
                    f'{path}.location must equal elevator_lobby or initial_pose'
                )
            if not isinstance(step.get('name'), str) or not step['name'].strip():
                errors.append(f'{path}.name must be a non-empty string')
        else:
            errors.append(f'{path}.type {step_type!r} is not supported')


def validate_system_config(config: Mapping[str, Any]) -> None:
    """Validate schema, geometry, references, and safety-critical invariants."""
    errors: List[str] = []
    if not isinstance(config, Mapping):
        raise ConfigurationError('configuration root must be a mapping')
    if config.get('schema_version') != 1:
        errors.append('root.schema_version must equal 1')

    system = _mapping(config, 'system', 'root', errors)
    simulation = _mapping(config, 'simulation', 'root', errors)
    generation = _mapping(config, 'map_generation', 'root', errors)
    frames = _mapping(config, 'frames', 'root', errors)
    floors = _mapping(config, 'floors', 'root', errors)
    elevators = _mapping(config, 'elevators', 'root', errors)
    floor_switch = config.get('floor_switch')
    _validate_transfer_settings(
        floor_switch,
        'root.floor_switch',
        errors,
        system_scope=True,
    )
    _mapping(config, 'map_switch_transaction', 'root', errors)
    _mapping(config, 'navigation', 'root', errors)
    _mapping(config, 'safety', 'root', errors)

    deployment_mode = system.get('deployment_mode', 'simulation')
    if deployment_mode not in {'simulation', 'hardware'}:
        errors.append(
            'root.system.deployment_mode must equal simulation or hardware'
        )
    map_asset_kind = system.get('map_asset_kind', 'generated')
    if map_asset_kind not in {'generated', 'surveyed'}:
        errors.append(
            'root.system.map_asset_kind must equal generated or surveyed'
        )
    if deployment_mode == 'simulation' and map_asset_kind != 'generated':
        errors.append(
            'simulation deployment requires generated map assets'
        )
    if deployment_mode == 'hardware' and map_asset_kind != 'surveyed':
        errors.append(
            'hardware deployment requires surveyed map assets'
        )

    layout_mode = simulation.get('layout_mode')
    if layout_mode not in (
        'flat_separated_regions',
        'flat_connected_regions',
    ):
        errors.append(
            'root.simulation.layout_mode must equal flat_separated_regions '
            'or flat_connected_regions'
        )
    ground_z = _number(simulation, 'ground_z', 'root.simulation', errors)
    size_x = _number(
        simulation, 'floor_size_x', 'root.simulation', errors, positive=True
    )
    size_y = _number(
        simulation, 'floor_size_y', 'root.simulation', errors, positive=True
    )
    gap = _number(
        simulation,
        'inter_region_gap',
        'root.simulation',
        errors,
    )
    if gap < 0.0:
        errors.append('root.simulation.inter_region_gap cannot be negative')
    prohibit_crossing = simulation.get('prohibit_direct_region_crossing')
    teleport_on_switch = simulation.get('teleport_on_floor_switch')
    if not isinstance(prohibit_crossing, bool):
        errors.append(
            'root.simulation.prohibit_direct_region_crossing must be boolean'
        )
    if not isinstance(teleport_on_switch, bool):
        errors.append(
            'root.simulation.teleport_on_floor_switch must be boolean'
        )
    if layout_mode == 'flat_separated_regions':
        if prohibit_crossing is not True:
            errors.append(
                'root.simulation.prohibit_direct_region_crossing must be true '
                'for flat_separated_regions'
            )
    elif layout_mode == 'flat_connected_regions':
        if abs(gap) > 1e-9:
            errors.append(
                'root.simulation.inter_region_gap must be zero for '
                'flat_connected_regions'
            )
        if prohibit_crossing is not False:
            errors.append(
                'root.simulation.prohibit_direct_region_crossing must be false '
                'for flat_connected_regions'
            )
        if teleport_on_switch is not False:
            errors.append(
                'root.simulation.teleport_on_floor_switch must be false for '
                'flat_connected_regions'
            )

    obstacle_count = _number(
        generation,
        'obstacle_count_per_floor',
        'root.map_generation',
        errors,
        positive=True,
    )
    _number(
        generation,
        'obstacle_height',
        'root.map_generation',
        errors,
        positive=True,
    )
    _number(
        generation,
        'surface_resolution',
        'root.map_generation',
        errors,
        positive=True,
    )
    boundary_clearance = _number(
        generation,
        'boundary_clearance',
        'root.map_generation',
        errors,
        positive=True,
    )
    _number(
        generation,
        'reserved_zone_clearance',
        'root.map_generation',
        errors,
        positive=True,
    )
    minimum_obstacle_spacing = _number(
        generation,
        'minimum_obstacle_spacing',
        'root.map_generation',
        errors,
        positive=True,
    )

    required_frames = ('overview', 'active_map', 'odom', 'robot_base', 'lidar')
    for frame_name in required_frames:
        if not isinstance(frames.get(frame_name), str) or not frames[frame_name]:
            errors.append(f'root.frames.{frame_name} must be a non-empty string')

    expected_count = system.get('logical_floor_count')
    if not isinstance(expected_count, int) or isinstance(expected_count, bool):
        errors.append('root.system.logical_floor_count must be an integer')
    elif expected_count != len(floors):
        errors.append(
            'root.system.logical_floor_count does not match root.floors'
        )
    if deployment_mode == 'simulation' and len(floors) < 2:
        errors.append('root.floors must contain at least two logical floors')
    if deployment_mode == 'hardware' and len(floors) < 1:
        errors.append('root.floors must contain at least one logical floor')

    base_seeds = set()
    asset_paths = set()
    bounds_by_floor: Dict[str, Tuple[float, float, float, float]] = {}
    gateways_by_floor: Dict[str, Tuple[str, float, float]] = {}
    for floor_id, floor_value in floors.items():
        path = f'root.floors.{floor_id}'
        if not isinstance(floor_id, str) or not floor_id:
            errors.append('root.floors keys must be non-empty strings')
            continue
        if not isinstance(floor_value, Mapping):
            errors.append(f'{path} must be a mapping')
            continue
        floor = floor_value
        seed = floor.get('source_seed')
        replica_of = floor.get('replica_of')
        if not isinstance(seed, int) or isinstance(seed, bool):
            errors.append(f'{path}.source_seed must be an integer')
        if replica_of is None:
            if seed in base_seeds:
                errors.append(f'{path}.source_seed must be unique')
            else:
                base_seeds.add(seed)
        elif not isinstance(replica_of, str) or not replica_of:
            errors.append(f'{path}.replica_of must be a non-empty string')
        elif replica_of == floor_id:
            errors.append(f'{path}.replica_of cannot reference itself')
        elif replica_of not in floors:
            errors.append(
                f'{path}.replica_of references unknown floor {replica_of!r}'
            )
        else:
            template = floors[replica_of]
            if not isinstance(template, Mapping):
                errors.append(f'{path}.replica_of must reference a floor mapping')
            elif template.get('replica_of') is not None:
                errors.append(
                    f'{path}.replica_of must reference a base floor'
                )
            elif seed != template.get('source_seed'):
                errors.append(
                    f'{path}.source_seed must match replica source '
                    f'{replica_of}.source_seed'
                )

        offset = _sequence(
            floor.get('simulation_offset'),
            3,
            f'{path}.simulation_offset',
            errors,
        )
        overview_z = floor.get('overview_z')
        if not _is_number(overview_z):
            errors.append(f'{path}.overview_z must be a number')
        elif abs(float(overview_z) - ground_z) > 1e-9:
            errors.append(f'{path}.overview_z must equal simulation.ground_z')
        if abs(offset[2] - ground_z) > 1e-9:
            errors.append(
                f'{path}.simulation_offset z must equal simulation.ground_z'
            )
        if deployment_mode == 'hardware' and any(
            abs(value) > 1e-9 for value in offset
        ):
            errors.append(
                f'{path}.simulation_offset must be [0, 0, 0] in hardware '
                'mode; surveyed PCD and LIO must share the world frame'
            )

        for asset_key in ('pcd_file', 'metadata_file'):
            value = floor.get(asset_key)
            _safe_relative_path(value, f'{path}.{asset_key}', errors)
            if isinstance(value, str) and value in asset_paths:
                errors.append(f'{path}.{asset_key} must be unique')
            asset_paths.add(value)

        half_x = size_x / 2.0
        half_y = size_y / 2.0
        bounds = (
            offset[0] - half_x,
            offset[0] + half_x,
            offset[1] - half_y,
            offset[1] + half_y,
        )
        bounds_by_floor[floor_id] = bounds

        gateway_value = floor.get('transition_gateway')
        if layout_mode == 'flat_connected_regions':
            if not isinstance(gateway_value, Mapping):
                errors.append(f'{path}.transition_gateway must be a mapping')
            else:
                edge = gateway_value.get('edge')
                if edge not in ('min_x', 'max_x'):
                    errors.append(
                        f'{path}.transition_gateway.edge must equal min_x or max_x'
                    )
                    edge = 'min_x'
                center_y = _number(
                    gateway_value,
                    'center_y',
                    f'{path}.transition_gateway',
                    errors,
                )
                width = _number(
                    gateway_value,
                    'width',
                    f'{path}.transition_gateway',
                    errors,
                    positive=True,
                )
                if (
                    center_y - width / 2.0 < -half_y
                    or center_y + width / 2.0 > half_y
                ):
                    errors.append(
                        f'{path}.transition_gateway exceeds the floor y bounds'
                    )
                gateways_by_floor[floor_id] = (
                    str(edge),
                    center_y,
                    width,
                )

        for pose_key in (
            'initial_pose',
            'elevator_lobby_pose',
            'elevator_cabin_pose',
        ):
            pose = _sequence(floor.get(pose_key), 3, f'{path}.{pose_key}', errors)
            _pose_in_bounds(pose, bounds, f'{path}.{pose_key}', errors)

        reserved_zones = floor.get('reserved_zones')
        if not isinstance(reserved_zones, list) or not reserved_zones:
            errors.append(f'{path}.reserved_zones must be a non-empty list')
        else:
            for index, zone in enumerate(reserved_zones):
                zone_path = f'{path}.reserved_zones[{index}]'
                if not isinstance(zone, Mapping):
                    errors.append(f'{zone_path} must be a mapping')
                    continue
                center = _sequence(
                    zone.get('center'), 2, f'{zone_path}.center', errors
                )
                zone_size = _sequence(
                    zone.get('size'), 2, f'{zone_path}.size', errors
                )
                if any(item <= 0.0 for item in zone_size):
                    errors.append(f'{zone_path}.size values must be positive')
                zone_bounds = (
                    center[0] - zone_size[0] / 2.0,
                    center[0] + zone_size[0] / 2.0,
                    center[1] - zone_size[1] / 2.0,
                    center[1] + zone_size[1] / 2.0,
                )
                if (
                    zone_bounds[0] < bounds[0]
                    or zone_bounds[1] > bounds[1]
                    or zone_bounds[2] < bounds[2]
                    or zone_bounds[3] > bounds[3]
                ):
                    errors.append(f'{zone_path} extends outside its floor region')

        fixed_obstacles = floor.get('fixed_obstacles', [])
        if replica_of is not None and fixed_obstacles:
            errors.append(
                f'{path}.fixed_obstacles must be declared only on its '
                'replica source'
            )
        elif not isinstance(fixed_obstacles, list):
            errors.append(f'{path}.fixed_obstacles must be a list')
        else:
            fixed_names = set()
            fixed_rectangles = []
            for index, obstacle in enumerate(fixed_obstacles):
                obstacle_path = f'{path}.fixed_obstacles[{index}]'
                if not isinstance(obstacle, Mapping):
                    errors.append(f'{obstacle_path} must be a mapping')
                    continue
                name = obstacle.get('name')
                if not isinstance(name, str) or not name:
                    errors.append(
                        f'{obstacle_path}.name must be a non-empty string'
                    )
                elif name in fixed_names:
                    errors.append(
                        f'{obstacle_path}.name must be unique on the floor'
                    )
                fixed_names.add(name)
                center = _sequence(
                    obstacle.get('center'),
                    2,
                    f'{obstacle_path}.center',
                    errors,
                )
                obstacle_size = _sequence(
                    obstacle.get('size'),
                    2,
                    f'{obstacle_path}.size',
                    errors,
                )
                if any(item <= 0.0 for item in obstacle_size):
                    errors.append(
                        f'{obstacle_path}.size values must be positive'
                    )
                rectangle = (
                    center[0] - obstacle_size[0] / 2.0,
                    center[0] + obstacle_size[0] / 2.0,
                    center[1] - obstacle_size[1] / 2.0,
                    center[1] + obstacle_size[1] / 2.0,
                )
                inset_bounds = (
                    bounds[0] + boundary_clearance,
                    bounds[1] - boundary_clearance,
                    bounds[2] + boundary_clearance,
                    bounds[3] - boundary_clearance,
                )
                if (
                    rectangle[0] < inset_bounds[0]
                    or rectangle[1] > inset_bounds[1]
                    or rectangle[2] < inset_bounds[2]
                    or rectangle[3] > inset_bounds[3]
                ):
                    errors.append(
                        f'{obstacle_path} violates boundary clearance'
                    )
                for previous in fixed_rectangles:
                    if not (
                        rectangle[1] + minimum_obstacle_spacing < previous[0]
                        or rectangle[0] - minimum_obstacle_spacing > previous[1]
                        or rectangle[3] + minimum_obstacle_spacing < previous[2]
                        or rectangle[2] - minimum_obstacle_spacing > previous[3]
                    ):
                        errors.append(
                            f'{obstacle_path} violates minimum obstacle '
                            'spacing'
                        )
                        break
                fixed_rectangles.append(rectangle)
            if len(fixed_obstacles) > int(obstacle_count):
                errors.append(
                    f'{path}.fixed_obstacles count exceeds '
                    'map_generation.obstacle_count_per_floor'
                )

        points = floor.get('inspection_points')
        if not isinstance(points, list) or not points:
            errors.append(f'{path}.inspection_points must be a non-empty list')
        else:
            names = set()
            for index, point in enumerate(points):
                point_path = f'{path}.inspection_points[{index}]'
                if not isinstance(point, Mapping):
                    errors.append(f'{point_path} must be a mapping')
                    continue
                name = point.get('name')
                if not isinstance(name, str) or not name:
                    errors.append(f'{point_path}.name must be a non-empty string')
                elif name in names:
                    errors.append(f'{point_path}.name must be unique on the floor')
                names.add(name)
                pose = _sequence(
                    point.get('pose'), 3, f'{point_path}.pose', errors
                )
                _pose_in_bounds(pose, bounds, f'{point_path}.pose', errors)

    floor_items = list(bounds_by_floor.items())
    for index, (left_id, left) in enumerate(floor_items):
        for right_id, right in floor_items[index + 1:]:
            if deployment_mode == 'hardware':
                # Real floor maps may legitimately occupy the same x/y
                # coordinates; only one is active after relocation.
                continue
            overlap_x = min(left[1], right[1]) - max(left[0], right[0])
            overlap_y = min(left[3], right[3]) - max(left[2], right[2])
            if overlap_x > 0.0 and overlap_y > 0.0:
                errors.append(
                    f'floor regions {left_id} and {right_id} overlap in x/y'
                )
            separation_x = max(0.0, max(left[0], right[0]) - min(left[1], right[1]))
            separation_y = max(0.0, max(left[2], right[2]) - min(left[3], right[3]))
            separation = max(separation_x, separation_y)
            if separation + 1e-9 < gap:
                errors.append(
                    f'floor regions {left_id} and {right_id} have less than '
                    'simulation.inter_region_gap separation'
                )

    for elevator_id, elevator_value in elevators.items():
        path = f'root.elevators.{elevator_id}'
        if not isinstance(elevator_value, Mapping):
            errors.append(f'{path} must be a mapping')
            continue
        elevator = elevator_value
        _validate_transfer_settings(
            elevator.get('transfer'), f'{path}.transfer', errors
        )
        simulation_mode = elevator.get('simulation_mode')
        if simulation_mode is not None and (
            simulation_mode != 'continuous_origin_map_switch'
        ):
            errors.append(
                f'{path}.simulation_mode is a legacy field and, when set, '
                'must equal continuous_origin_map_switch'
            )

        transitions = elevator.get('transitions')
        route_items = []
        if transitions is None:
            route_items.append((path, elevator))
        elif not isinstance(transitions, list) or not transitions:
            errors.append(f'{path}.transitions must be a non-empty list')
            continue
        else:
            for index, route in enumerate(transitions):
                route_path = f'{path}.transitions[{index}]'
                if not isinstance(route, Mapping):
                    errors.append(f'{route_path} must be a mapping')
                    continue
                route_items.append((route_path, route))

        seen_directions = set()
        for route_path, route in route_items:
            explicit_routes = transitions is not None
            source_key = 'from' if explicit_routes else 'source_floor'
            target_key = 'to' if explicit_routes else 'target_floor'
            source = route.get(source_key)
            target = route.get(target_key)
            direction = (source, target)
            if direction in seen_directions:
                errors.append(
                    f'{route_path} duplicates route {source!r}->{target!r}'
                )
            seen_directions.add(direction)
            if source not in floors:
                errors.append(
                    f'{route_path}.{source_key} references unknown floor '
                    f'{source!r}'
                )
            if target not in floors:
                errors.append(
                    f'{route_path}.{target_key} references unknown floor '
                    f'{target!r}'
                )
            if source == target:
                errors.append(
                    f'{route_path} source and target floors must differ'
                )

            tolerance_source = dict(elevator)
            tolerance_source.update(route)
            trigger_tolerance_xy = _number(
                tolerance_source,
                'trigger_tolerance_xy',
                route_path,
                errors,
                positive=True,
            )
            trigger_tolerance_yaw = _number(
                tolerance_source,
                'trigger_tolerance_yaw',
                route_path,
                errors,
                positive=True,
            )
            source_pose = _sequence(
                route.get('source_trigger_pose'),
                3,
                f'{route_path}.source_trigger_pose',
                errors,
            )
            target_pose = _sequence(
                route.get('target_release_pose'),
                3,
                f'{route_path}.target_release_pose',
                errors,
            )
            _validate_transfer_settings(
                route.get('transfer'), f'{route_path}.transfer', errors
            )
            if source in bounds_by_floor:
                _pose_in_bounds(
                    source_pose,
                    bounds_by_floor[source],
                    f'{route_path}.source_trigger_pose',
                    errors,
                )
            if target in bounds_by_floor:
                _pose_in_bounds(
                    target_pose,
                    bounds_by_floor[target],
                    f'{route_path}.target_release_pose',
                    errors,
                )

            handoff = (
                'set_simulation_pose'
                if teleport_on_switch
                else 'preserve'
            )
            for policy in (
                floor_switch,
                elevator.get('transfer'),
                route.get('transfer'),
            ):
                if isinstance(policy, Mapping) and 'pose_handoff' in policy:
                    handoff = policy['pose_handoff']
            if layout_mode == 'flat_connected_regions':
                if trigger_tolerance_xy > 0.5:
                    errors.append(
                        f'{route_path}.trigger_tolerance_xy must not exceed '
                        '0.5 m'
                    )
                if (
                    handoff == 'preserve'
                    and trigger_tolerance_yaw + 1e-9 < math.pi
                ):
                    errors.append(
                        f'{route_path}.trigger_tolerance_yaw must allow any '
                        'heading for a point-only shared gateway'
                    )
                if handoff == 'preserve' and any(
                    abs(source_pose[index] - target_pose[index]) > 1e-9
                    for index in range(3)
                ):
                    errors.append(
                        f'{route_path} source trigger and target release must '
                        'be the same world pose for preserve handoff'
                    )
                for floor_id, pose, pose_name in (
                    (source, source_pose, 'source_trigger_pose'),
                    (target, target_pose, 'target_release_pose'),
                ):
                    if (
                        floor_id not in gateways_by_floor
                        or floor_id not in floors
                    ):
                        continue
                    edge, center_y, _ = gateways_by_floor[floor_id]
                    floor = floors[floor_id]
                    offset = floor['simulation_offset']
                    edge_x = (
                        bounds_by_floor[floor_id][0]
                        if edge == 'min_x'
                        else bounds_by_floor[floor_id][1]
                    )
                    gateway_y = float(offset[1]) + center_y
                    if (
                        abs(pose[0] - edge_x) > 1e-9
                        or abs(pose[1] - gateway_y) > 1e-9
                    ):
                        errors.append(
                            f'{route_path}.{pose_name} must equal the center '
                            f'of {floor_id}.transition_gateway'
                        )

    _validate_mission(config, floors, elevators, errors)
    if errors:
        detail = '\n'.join(f'- {error}' for error in errors)
        raise ConfigurationError(f'invalid system configuration:\n{detail}')


def load_system_config(path: Path) -> Dict[str, Any]:
    """Load a YAML file and return it only after complete validation."""
    config_path = Path(path).expanduser().resolve()
    try:
        with config_path.open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigurationError(
            f'cannot load configuration {config_path}: {error}'
        ) from error
    validate_system_config(config)
    return dict(config)


def configured_asset_paths(
    config: Mapping[str, Any],
    package_root: Path,
) -> Iterable[Path]:
    """Yield every floor asset path declared by the configuration."""
    root = Path(package_root).resolve()
    for floor in config['floors'].values():
        for key in ('pcd_file', 'metadata_file'):
            yield root / floor[key]
