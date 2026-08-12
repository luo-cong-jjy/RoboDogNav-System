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

"""Generate deterministic PCD assets for each logical floor."""

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from .configuration import display_to_source_xy, transition_route


Rectangle = Tuple[float, float, float, float]


@dataclass(frozen=True)
class FloorAssets:
    """Paths and summary values for one generated floor."""

    floor_id: str
    pcd_path: Path
    metadata_path: Path
    point_count: int
    obstacle_count: int
    pcd_sha256: str


def sha256_file(path: Path) -> str:
    """Return the SHA-256 digest of a file without loading it all at once."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=path.parent,
        prefix=f'.{path.name}.',
        suffix='.tmp',
        delete=False,
    ) as stream:
        stream.write(content)
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _axis_values(lower: float, upper: float, resolution: float) -> np.ndarray:
    count = max(2, int(math.ceil((upper - lower) / resolution)) + 1)
    return np.linspace(lower, upper, count, dtype=np.float32)


def _plane(
    fixed_axis: int,
    fixed_value: float,
    first_axis: int,
    first_values: np.ndarray,
    second_axis: int,
    second_values: np.ndarray,
) -> np.ndarray:
    first, second = np.meshgrid(
        first_values, second_values, indexing='ij'
    )
    points = np.empty((first.size, 3), dtype=np.float32)
    points[:, fixed_axis] = fixed_value
    points[:, first_axis] = first.ravel()
    points[:, second_axis] = second.ravel()
    return points


def _box_surface_points(
    rectangle: Rectangle,
    height: float,
    resolution: float,
) -> np.ndarray:
    x_min, x_max, y_min, y_max = rectangle
    x_values = _axis_values(x_min, x_max, resolution)
    y_values = _axis_values(y_min, y_max, resolution)
    z_values = _axis_values(0.0, height, resolution)
    faces = [
        _plane(0, x_min, 1, y_values, 2, z_values),
        _plane(0, x_max, 1, y_values, 2, z_values),
        _plane(1, y_min, 0, x_values, 2, z_values),
        _plane(1, y_max, 0, x_values, 2, z_values),
        _plane(2, height, 0, x_values, 1, y_values),
    ]
    return np.concatenate(faces)


def _wall_surface_points(
    half_x: float,
    half_y: float,
    height: float,
    resolution: float,
    gateway: Optional[Mapping[str, Any]] = None,
) -> np.ndarray:
    x_values = _axis_values(-half_x, half_x, resolution)
    y_values = _axis_values(-half_y, half_y, resolution)
    z_values = _axis_values(0.0, height, resolution)
    walls = {
        'min_x': _plane(0, -half_x, 1, y_values, 2, z_values),
        'max_x': _plane(0, half_x, 1, y_values, 2, z_values),
        'min_y': _plane(1, -half_y, 0, x_values, 2, z_values),
        'max_y': _plane(1, half_y, 0, x_values, 2, z_values),
    }
    if gateway is not None:
        edge = str(gateway['edge'])
        center_y = float(gateway['center_y'])
        half_width = float(gateway['width']) / 2.0
        edge_points = walls[edge]
        # Remove the complete vertical wall surface inside the doorway.  This
        # is deliberate collision geometry, not an RViz-only visual mask.
        walls[edge] = edge_points[
            np.abs(edge_points[:, 1] - center_y) > half_width + 1e-6
        ]
    return np.concatenate(list(walls.values()))


def _intersects(left: Rectangle, right: Rectangle) -> bool:
    return not (
        left[1] < right[0]
        or left[0] > right[1]
        or left[3] < right[2]
        or left[2] > right[3]
    )


def _expanded(rectangle: Rectangle, margin: float) -> Rectangle:
    return (
        rectangle[0] - margin,
        rectangle[1] + margin,
        rectangle[2] - margin,
        rectangle[3] + margin,
    )


def _protected_rectangles(
    floor: Mapping[str, Any],
    corridor_clearance: float,
    mission_segments: Sequence[
        Tuple[Tuple[float, float], Tuple[float, float]]
    ] = (),
) -> List[Rectangle]:
    """Build free zones around anchors and native-SCAN straight route legs."""
    protected: List[Rectangle] = []
    for zone in floor['reserved_zones']:
        center_x, center_y = display_to_source_xy(floor, zone['center'])
        half_x = float(zone['size'][0]) / 2.0 + corridor_clearance
        half_y = float(zone['size'][1]) / 2.0 + corridor_clearance
        protected.append(
            (
                center_x - half_x,
                center_x + half_x,
                center_y - half_y,
                center_y + half_y,
            )
        )

    display_anchors = [
        floor['initial_pose'],
        *[point['pose'] for point in floor['inspection_points']],
        floor['elevator_lobby_pose'],
        floor['elevator_cabin_pose'],
    ]
    anchors = [
        display_to_source_xy(floor, pose) for pose in display_anchors
    ]
    # The native SCAN chain receives the final goal directly; it does not
    # follow an artificial Manhattan/global route.  Sample each direct leg
    # with overlapping clearance boxes so generated obstacles cannot trap the
    # local planner along the configured inspection sequence.
    sample_spacing = max(0.10, corridor_clearance * 0.25)
    segments = list(zip(anchors, anchors[1:]))
    segments.extend(mission_segments)
    for start, finish in segments:
        distance = math.hypot(
            finish[0] - start[0],
            finish[1] - start[1],
        )
        sample_count = max(1, int(math.ceil(distance / sample_spacing)))
        for index in range(sample_count + 1):
            ratio = index / sample_count
            x = start[0] + ratio * (finish[0] - start[0])
            y = start[1] + ratio * (finish[1] - start[1])
            protected.append(
                (
                    x - corridor_clearance,
                    x + corridor_clearance,
                    y - corridor_clearance,
                    y + corridor_clearance,
                )
            )
    return protected


def _mission_corridor_segments(
    config: Mapping[str, Any],
) -> Dict[str, List[Tuple[Tuple[float, float], Tuple[float, float]]]]:
    """Return automatic native-SCAN legs grouped by source map asset."""
    floors = config['floors']
    grouped: Dict[
        str,
        List[Tuple[Tuple[float, float], Tuple[float, float]]],
    ] = {}
    current_floor = next(iter(floors))
    current_pose = floors[current_floor]['initial_pose']

    def _source_id(floor_id: str) -> str:
        return str(floors[floor_id].get('replica_of') or floor_id)

    def _append(target_floor: str, target_pose: Sequence[float]) -> None:
        nonlocal current_floor, current_pose
        if current_floor != target_floor:
            current_floor = target_floor
            current_pose = target_pose
            return
        floor = floors[current_floor]
        start = display_to_source_xy(floor, current_pose)
        finish = display_to_source_xy(floor, target_pose)
        grouped.setdefault(_source_id(current_floor), []).append(
            (start, finish)
        )
        current_pose = target_pose

    for step in config['mission']['sequence']:
        step_type = str(step['type'])
        if step_type in {'inspection', 'transit'}:
            floor_id = str(step['floor'])
            point = next(
                item
                for item in floors[floor_id]['inspection_points']
                if item['name'] == step['point']
            )
            _append(floor_id, point['pose'])
        elif step_type in {'elevator_transfer', 'floor_transfer'}:
            connector_key = (
                'elevator' if step_type == 'elevator_transfer' else 'connector'
            )
            source_floor = str(step['from'])
            target_floor = str(step['to'])
            route = transition_route(
                config,
                str(step[connector_key]),
                source_floor,
                target_floor,
            )
            _append(source_floor, route['source_trigger_pose'])
            current_floor = target_floor
            current_pose = route['target_release_pose']
        elif step_type == 'terminal':
            floor_id = str(step['floor'])
            pose_key = {
                'elevator_lobby': 'elevator_lobby_pose',
                'initial_pose': 'initial_pose',
            }[str(step['location'])]
            _append(floor_id, floors[floor_id][pose_key])
    return grouped


def _place_obstacles(
    rng: np.random.Generator,
    count: int,
    half_x: float,
    half_y: float,
    boundary_clearance: float,
    minimum_spacing: float,
    protected: Sequence[Rectangle],
    fixed: Sequence[Rectangle] = (),
) -> List[Rectangle]:
    obstacles = list(fixed)
    if len(obstacles) > count:
        raise RuntimeError(
            f'configured {len(obstacles)} fixed obstacles for a total '
            f'obstacle count of {count}'
        )
    attempts = 0
    maximum_attempts = max(10000, count * 2500)
    while len(obstacles) < count and attempts < maximum_attempts:
        attempts += 1
        width = float(rng.uniform(0.45, 0.85))
        depth = float(rng.uniform(0.45, 0.85))
        center_x = float(
            rng.uniform(
                -half_x + boundary_clearance + width / 2.0,
                half_x - boundary_clearance - width / 2.0,
            )
        )
        center_y = float(
            rng.uniform(
                -half_y + boundary_clearance + depth / 2.0,
                half_y - boundary_clearance - depth / 2.0,
            )
        )
        candidate = (
            center_x - width / 2.0,
            center_x + width / 2.0,
            center_y - depth / 2.0,
            center_y + depth / 2.0,
        )
        if any(_intersects(candidate, area) for area in protected):
            continue
        if any(
            _intersects(_expanded(candidate, minimum_spacing), obstacle)
            for obstacle in obstacles
        ):
            continue
        obstacles.append(candidate)
    if len(obstacles) != count:
        raise RuntimeError(
            f'could only place {len(obstacles)} of {count} obstacles after '
            f'{attempts} deterministic attempts'
        )
    return obstacles


def _configured_fixed_obstacles(
    floor: Mapping[str, Any],
) -> Tuple[List[Rectangle], List[Mapping[str, Any]]]:
    """Convert explicitly placed display-frame obstacles to source rectangles."""
    rectangles: List[Rectangle] = []
    descriptions: List[Mapping[str, Any]] = []
    for obstacle in floor.get('fixed_obstacles', []):
        center_x, center_y = display_to_source_xy(
            floor, obstacle['center']
        )
        width = float(obstacle['size'][0])
        depth = float(obstacle['size'][1])
        rectangle = (
            center_x - width / 2.0,
            center_x + width / 2.0,
            center_y - depth / 2.0,
            center_y + depth / 2.0,
        )
        rectangles.append(rectangle)
        descriptions.append(
            {
                'name': str(obstacle['name']),
                'center': [center_x, center_y],
                'size': [width, depth],
                'x_min': rectangle[0],
                'x_max': rectangle[1],
                'y_min': rectangle[2],
                'y_max': rectangle[3],
            }
        )
    return rectangles, descriptions


def _cell_index(
    coordinate: float,
    lower: float,
    resolution: float,
    limit: int,
) -> int:
    return min(limit - 1, max(0, int(math.floor((coordinate - lower) / resolution))))


def _build_occupancy(
    obstacles: Sequence[Rectangle],
    half_x: float,
    half_y: float,
    resolution: float,
    gateway: Optional[Mapping[str, Any]] = None,
) -> np.ndarray:
    width = int(round(2.0 * half_x / resolution))
    height = int(round(2.0 * half_y / resolution))
    occupancy = np.zeros((height, width), dtype=np.uint8)
    occupancy[0, :] = 100
    occupancy[-1, :] = 100
    occupancy[:, 0] = 100
    occupancy[:, -1] = 100
    for x_min, x_max, y_min, y_max in obstacles:
        x0 = _cell_index(x_min, -half_x, resolution, width)
        x1 = _cell_index(x_max, -half_x, resolution, width)
        y0 = _cell_index(y_min, -half_y, resolution, height)
        y1 = _cell_index(y_max, -half_y, resolution, height)
        occupancy[y0:y1 + 1, x0:x1 + 1] = 100
    if gateway is not None:
        center_y = float(gateway['center_y'])
        half_width = float(gateway['width']) / 2.0
        y0 = _cell_index(
            center_y - half_width,
            -half_y,
            resolution,
            height,
        )
        y1 = _cell_index(
            center_y + half_width,
            -half_y,
            resolution,
            height,
        )
        edge = str(gateway['edge'])
        column = 0 if edge == 'min_x' else width - 1
        occupancy[y0:y1 + 1, column] = 0
    return occupancy


def _validate_connectivity(
    occupancy: np.ndarray,
    floor: Mapping[str, Any],
    half_x: float,
    half_y: float,
    resolution: float,
) -> None:
    display_targets = [
        floor['initial_pose'],
        *[point['pose'] for point in floor['inspection_points']],
        floor['elevator_lobby_pose'],
        floor['elevator_cabin_pose'],
    ]
    height, width = occupancy.shape
    targets = []
    for pose in display_targets:
        x, y = display_to_source_xy(floor, pose)
        targets.append(
            (
                _cell_index(x, -half_x, resolution, width),
                _cell_index(y, -half_y, resolution, height),
            )
        )
    for x, y in targets:
        if occupancy[y, x] != 0:
            raise RuntimeError(f'required free-space cell ({x}, {y}) is occupied')

    start = targets[0]
    queue = deque([start])
    visited = np.zeros_like(occupancy, dtype=np.bool_)
    visited[start[1], start[0]] = True
    while queue:
        x, y = queue.popleft()
        for next_x, next_y in (
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
        ):
            if (
                0 <= next_x < width
                and 0 <= next_y < height
                and not visited[next_y, next_x]
                and occupancy[next_y, next_x] == 0
            ):
                visited[next_y, next_x] = True
                queue.append((next_x, next_y))
    unreachable = [
        target for target in targets if not visited[target[1], target[0]]
    ]
    if unreachable:
        raise RuntimeError(f'configured mission points are disconnected: {unreachable}')


def _write_pcd(path: Path, points: np.ndarray, intensity) -> None:
    values = np.empty((points.shape[0], 4), dtype=np.float32)
    values[:, :3] = points
    values[:, 3] = np.asarray(intensity, dtype=np.float32)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {values.shape[0]}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {values.shape[0]}\n'
        'DATA ascii\n'
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=path.parent,
        prefix=f'.{path.name}.',
        suffix='.tmp',
        delete=False,
    ) as stream:
        stream.write(header)
        np.savetxt(stream, values, fmt='%.4f %.4f %.4f %.1f')
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _floor_asset_paths(
    floor: Mapping[str, Any],
    output_root: Path,
) -> Tuple[Path, Path]:
    pcd_path = output_root / floor['pcd_file']
    metadata_path = output_root / floor['metadata_file']
    return pcd_path, metadata_path


def generate_floor_assets(
    config: Mapping[str, Any],
    floor_id: str,
    output_root: Path,
) -> FloorAssets:
    """Generate and validate one floor's deterministic source-coordinate assets."""
    floor = config['floors'][floor_id]
    replica_of = floor.get('replica_of')
    source_floor = (
        config['floors'][replica_of]
        if replica_of is not None
        else floor
    )
    simulation = config['simulation']
    generation = config['map_generation']
    half_x = float(simulation['floor_size_x']) / 2.0
    half_y = float(simulation['floor_size_y']) / 2.0
    resolution = float(generation['surface_resolution'])
    height = float(generation['obstacle_height'])
    count = int(generation['obstacle_count_per_floor'])
    source_floor_id = str(replica_of) if replica_of is not None else floor_id
    mission_segments = _mission_corridor_segments(config).get(
        source_floor_id, []
    )
    protected = _protected_rectangles(
        source_floor,
        float(generation['reserved_zone_clearance']),
        mission_segments,
    )
    fixed_obstacles, fixed_descriptions = _configured_fixed_obstacles(
        source_floor
    )
    obstacles = _place_obstacles(
        np.random.default_rng(int(source_floor['source_seed'])),
        count,
        half_x,
        half_y,
        float(generation['boundary_clearance']),
        float(generation['minimum_obstacle_spacing']),
        protected,
        fixed_obstacles,
    )

    gateway = floor.get('transition_gateway')
    point_groups = [
        _wall_surface_points(
            half_x,
            half_y,
            height,
            resolution,
            gateway,
        )
    ]
    point_groups.extend(
        _box_surface_points(obstacle, height, resolution)
        for obstacle in obstacles
    )
    points = np.concatenate(point_groups)
    points = np.unique(np.round(points, decimals=4), axis=0)
    occupancy = _build_occupancy(
        obstacles,
        half_x,
        half_y,
        resolution,
        gateway,
    )
    _validate_connectivity(
        occupancy, floor, half_x, half_y, resolution
    )

    root = Path(output_root).resolve()
    pcd_path, metadata_path = _floor_asset_paths(floor, root)
    source_floor_order = list(config['floors']).index(source_floor_id) + 1
    _write_pcd(pcd_path, points, float(source_floor_order))
    pcd_hash = sha256_file(pcd_path)
    offset = [float(value) for value in floor['simulation_offset']]
    metadata: Dict[str, Any] = {
        'schema_version': 1,
        'floor_id': floor_id,
        'source_seed': int(source_floor['source_seed']),
        'replica_of': replica_of,
        'transition_gateway': gateway,
        'coordinate_convention': {
            'asset_frame': 'floor_local',
            'overview_frame': config['frames']['overview'],
            'simulation_offset': offset,
            'ground_z': float(simulation['ground_z']),
        },
        'source_bounds': {
            'x': [-half_x, half_x],
            'y': [-half_y, half_y],
            'z': [0.0, height],
        },
        'overview_bounds': {
            'x': [-half_x + offset[0], half_x + offset[0]],
            'y': [-half_y + offset[1], half_y + offset[1]],
            'z': [0.0 + offset[2], height + offset[2]],
        },
        'surface_resolution': resolution,
        'obstacle_height': height,
        'minimum_obstacle_spacing': float(
            generation['minimum_obstacle_spacing']
        ),
        'fixed_obstacle_count': len(fixed_obstacles),
        'fixed_obstacles': fixed_descriptions,
        'obstacle_count': len(obstacles),
        'obstacles': [
            {
                'x_min': rectangle[0],
                'x_max': rectangle[1],
                'y_min': rectangle[2],
                'y_max': rectangle[3],
            }
            for rectangle in obstacles
        ],
        'protected_areas': [
            {
                'x_min': rectangle[0],
                'x_max': rectangle[1],
                'y_min': rectangle[2],
                'y_max': rectangle[3],
            }
            for rectangle in protected
        ],
        'point_count': int(points.shape[0]),
        # This count comes from an in-memory connectivity check only. No
        # runtime OccupancyGrid asset is written or consumed.
        'connectivity_blocked_cell_count': int(
            np.count_nonzero(occupancy == 100)
        ),
        'files': {
            'pcd': pcd_path.name,
            'pcd_sha256': pcd_hash,
        },
    }
    _atomic_text(
        metadata_path,
        json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
    )
    return FloorAssets(
        floor_id=floor_id,
        pcd_path=pcd_path,
        metadata_path=metadata_path,
        point_count=int(points.shape[0]),
        obstacle_count=len(obstacles),
        pcd_sha256=pcd_hash,
    )


def generate_all_floor_assets(
    config: Mapping[str, Any],
    output_root: Path,
    floor_ids: Optional[Iterable[str]] = None,
) -> List[FloorAssets]:
    """Generate selected floors in stable configuration order."""
    selected = set(floor_ids) if floor_ids is not None else None
    results = []
    for floor_id in config['floors']:
        if selected is None or floor_id in selected:
            results.append(generate_floor_assets(config, floor_id, output_root))
    if selected is not None:
        unknown = selected.difference(config['floors'])
        if unknown:
            raise KeyError(f'unknown floor IDs: {sorted(unknown)}')
    return results


def read_ascii_pcd(path: Path) -> np.ndarray:
    """Read the generated x/y/z/intensity ASCII PCD into float32 values."""
    fields: Optional[List[str]] = None
    point_count: Optional[int] = None
    data_line: Optional[int] = None
    with Path(path).open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream):
            stripped = line.strip()
            if stripped.startswith('FIELDS '):
                fields = stripped.split()[1:]
            elif stripped.startswith('POINTS '):
                point_count = int(stripped.split()[1])
            elif stripped.startswith('DATA '):
                if stripped != 'DATA ascii':
                    raise ValueError(f'{path} is not an ASCII PCD')
                data_line = line_number + 1
                break
    if fields != ['x', 'y', 'z', 'intensity']:
        raise ValueError(f'{path} has unsupported PCD fields {fields}')
    if data_line is None or point_count is None:
        raise ValueError(f'{path} has an incomplete PCD header')
    values = np.loadtxt(
        path,
        dtype=np.float32,
        skiprows=data_line,
        ndmin=2,
    )
    if values.shape != (point_count, 4):
        raise ValueError(
            f'{path} declares {point_count} points but contains {values.shape}'
        )
    if not np.isfinite(values).all():
        raise ValueError(f'{path} contains non-finite point values')
    return values


def validate_generated_assets(
    config: Mapping[str, Any],
    package_root: Path,
) -> List[FloorAssets]:
    """Check generated PCD files, hashes, bounds, and replica contracts."""
    root = Path(package_root).resolve()
    results: List[FloorAssets] = []
    pcd_hashes: Dict[str, str] = {}
    pcd_values: Dict[str, np.ndarray] = {}
    metadata_by_floor: Dict[str, Mapping[str, Any]] = {}
    expected_half_x = float(config['simulation']['floor_size_x']) / 2.0
    expected_half_y = float(config['simulation']['floor_size_y']) / 2.0
    for floor_id, floor in config['floors'].items():
        pcd_path, metadata_path = _floor_asset_paths(floor, root)
        for path in (pcd_path, metadata_path):
            if not path.is_file():
                raise FileNotFoundError(f'missing generated asset: {path}')
        values = read_ascii_pcd(pcd_path)
        with metadata_path.open('r', encoding='utf-8') as stream:
            metadata = json.load(stream)

        pcd_hash = sha256_file(pcd_path)
        replica_of = floor.get('replica_of')
        if metadata['floor_id'] != floor_id:
            raise ValueError(f'{metadata_path} floor_id does not match')
        if metadata.get('replica_of') != replica_of:
            raise ValueError(f'{metadata_path} replica_of does not match config')
        if metadata.get('transition_gateway') != floor.get(
            'transition_gateway'
        ):
            raise ValueError(
                f'{metadata_path} transition_gateway does not match config'
            )
        if metadata['point_count'] != values.shape[0]:
            raise ValueError(f'{metadata_path} point_count does not match PCD')
        if metadata['obstacle_count'] != int(
            config['map_generation']['obstacle_count_per_floor']
        ):
            raise ValueError(f'{metadata_path} obstacle_count does not match config')
        source_floor = (
            config['floors'][replica_of]
            if replica_of is not None
            else floor
        )
        expected_fixed_count = len(source_floor.get('fixed_obstacles', []))
        if metadata.get('fixed_obstacle_count') != expected_fixed_count:
            raise ValueError(
                f'{metadata_path} fixed obstacle count does not match config'
            )
        minimum_spacing = float(
            config['map_generation']['minimum_obstacle_spacing']
        )
        if not math.isclose(
            float(metadata.get('minimum_obstacle_spacing', -1.0)),
            minimum_spacing,
            abs_tol=1e-9,
        ):
            raise ValueError(
                f'{metadata_path} minimum obstacle spacing does not match'
            )
        obstacles = metadata['obstacles']
        for index, left in enumerate(obstacles):
            left_rectangle = (
                float(left['x_min']),
                float(left['x_max']),
                float(left['y_min']),
                float(left['y_max']),
            )
            for right in obstacles[index + 1:]:
                right_rectangle = (
                    float(right['x_min']),
                    float(right['x_max']),
                    float(right['y_min']),
                    float(right['y_max']),
                )
                if _intersects(
                    _expanded(left_rectangle, minimum_spacing - 1e-9),
                    right_rectangle,
                ):
                    raise ValueError(
                        f'{metadata_path} contains obstacles closer than '
                        f'{minimum_spacing:.3f} m'
                    )
        if metadata['files']['pcd_sha256'] != pcd_hash:
            raise ValueError(f'{metadata_path} PCD hash does not match')
        xyz_min = values[:, :3].min(axis=0)
        xyz_max = values[:, :3].max(axis=0)
        expected_min = np.array(
            [-expected_half_x, -expected_half_y, 0.0], dtype=np.float32
        )
        expected_max = np.array(
            [
                expected_half_x,
                expected_half_y,
                float(config['map_generation']['obstacle_height']),
            ],
            dtype=np.float32,
        )
        if not np.allclose(xyz_min, expected_min, atol=1e-4):
            raise ValueError(f'{pcd_path} minimum bounds do not match')
        if not np.allclose(xyz_max, expected_max, atol=1e-4):
            raise ValueError(f'{pcd_path} maximum bounds do not match')
        if replica_of is None:
            if pcd_hash in pcd_hashes.values():
                raise ValueError(
                    'base floors must not have identical PCD hashes'
                )
        else:
            if replica_of not in pcd_values:
                raise ValueError(
                    f'replica source {replica_of!r} must precede {floor_id!r}'
                )
            source_values = pcd_values[replica_of]
            source_interior = source_values[
                np.logical_and(
                    np.abs(source_values[:, 0]) < expected_half_x - 1e-4,
                    np.abs(source_values[:, 1]) < expected_half_y - 1e-4,
                )
            ]
            replica_interior = values[
                np.logical_and(
                    np.abs(values[:, 0]) < expected_half_x - 1e-4,
                    np.abs(values[:, 1]) < expected_half_y - 1e-4,
                )
            ]
            if not np.array_equal(replica_interior, source_interior):
                raise ValueError(
                    f'{floor_id} PCD interior is not a copy of {replica_of}'
                )
            if metadata['obstacles'] != metadata_by_floor[replica_of][
                'obstacles'
            ]:
                raise ValueError(
                    f'{floor_id} obstacle layout is not a copy of {replica_of}'
                )
        pcd_hashes[floor_id] = pcd_hash
        pcd_values[floor_id] = values
        metadata_by_floor[floor_id] = metadata
        results.append(
            FloorAssets(
                floor_id=floor_id,
                pcd_path=pcd_path,
                metadata_path=metadata_path,
                point_count=int(values.shape[0]),
                obstacle_count=int(metadata['obstacle_count']),
                pcd_sha256=pcd_hash,
            )
        )
    return results


def _read_generic_ascii_pcd(path: Path) -> Tuple[np.ndarray, List[str]]:
    """Read an ASCII PCD with arbitrary scalar fields for audited import."""
    fields: Optional[List[str]] = None
    counts: Optional[List[int]] = None
    point_count: Optional[int] = None
    data_line: Optional[int] = None
    with Path(path).open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream):
            stripped = line.strip()
            if stripped.startswith('FIELDS '):
                fields = stripped.split()[1:]
            elif stripped.startswith('COUNT '):
                counts = [int(value) for value in stripped.split()[1:]]
            elif stripped.startswith('POINTS '):
                point_count = int(stripped.split()[1])
            elif stripped.startswith('DATA '):
                if stripped != 'DATA ascii':
                    raise ValueError(
                        f'{path} must first be converted to DATA ascii'
                    )
                data_line = line_number + 1
                break
    if fields is None or point_count is None or data_line is None:
        raise ValueError(f'{path} has an incomplete PCD header')
    counts = counts or [1] * len(fields)
    if len(counts) != len(fields) or any(value != 1 for value in counts):
        raise ValueError(
            f'{path} contains non-scalar PCD fields; flatten before import'
        )
    values = np.loadtxt(path, dtype=np.float64, skiprows=data_line, ndmin=2)
    if values.shape != (point_count, len(fields)):
        raise ValueError(
            f'{path} declares {point_count} points/{len(fields)} fields '
            f'but contains {values.shape}'
        )
    return values, fields


def import_surveyed_ascii_pcd(
    input_path: Path,
    output_pcd_path: Path,
    output_metadata_path: Path,
    floor_id: str,
    *,
    asset_frame: str = 'world',
    source_map_builder: str = 'Elevator-LIO',
    force: bool = False,
) -> FloorAssets:
    """Normalize one surveyed ASCII PCD and write provenance metadata."""
    input_path = Path(input_path).resolve()
    output_pcd_path = Path(output_pcd_path).resolve()
    output_metadata_path = Path(output_metadata_path).resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f'missing input PCD: {input_path}')
    if not force:
        existing = [
            path
            for path in (output_pcd_path, output_metadata_path)
            if path.exists()
        ]
        if existing:
            raise FileExistsError(
                'refusing to overwrite surveyed assets: '
                + ', '.join(str(path) for path in existing)
            )
    values, fields = _read_generic_ascii_pcd(input_path)
    required = {'x', 'y', 'z'}
    if not required.issubset(fields):
        raise ValueError(f'{input_path} is missing x/y/z fields')
    indices = [fields.index(name) for name in ('x', 'y', 'z')]
    xyz = values[:, indices]
    if 'intensity' in fields:
        intensity = values[:, fields.index('intensity')]
    else:
        intensity = np.ones(values.shape[0], dtype=np.float64)
    finite = np.logical_and(
        np.isfinite(xyz).all(axis=1), np.isfinite(intensity)
    )
    removed = int(np.count_nonzero(~finite))
    xyz = xyz[finite]
    intensity = intensity[finite]
    if xyz.shape[0] == 0:
        raise ValueError(f'{input_path} contains no finite points')
    source_hash = sha256_file(input_path)
    _write_pcd(output_pcd_path, xyz, intensity)
    normalized_hash = sha256_file(output_pcd_path)
    metadata = {
        'schema_version': 1,
        'asset_kind': 'surveyed',
        'floor_id': str(floor_id),
        'coordinate_convention': {
            'asset_frame': str(asset_frame),
            'units': 'm',
            'simulation_offset': [0.0, 0.0, 0.0],
        },
        'source': {
            'map_builder': str(source_map_builder),
            'input_file': input_path.name,
            'input_sha256': source_hash,
            'input_fields': fields,
        },
        'normalization': {
            'output_fields': ['x', 'y', 'z', 'intensity'],
            'data_encoding': 'ascii',
            'non_finite_points_removed': removed,
        },
        'point_count': int(xyz.shape[0]),
        'xyz_bounds': {
            'min': [float(value) for value in xyz.min(axis=0)],
            'max': [float(value) for value in xyz.max(axis=0)],
        },
        'files': {
            'pcd': output_pcd_path.name,
            'pcd_sha256': normalized_hash,
        },
    }
    _atomic_text(
        output_metadata_path,
        json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
    )
    return FloorAssets(
        floor_id=str(floor_id),
        pcd_path=output_pcd_path,
        metadata_path=output_metadata_path,
        point_count=int(xyz.shape[0]),
        obstacle_count=0,
        pcd_sha256=normalized_hash,
    )


def validate_surveyed_assets(
    config: Mapping[str, Any],
    package_root: Path,
) -> List[FloorAssets]:
    """Validate real-site PCD/JSON provenance without synthetic assumptions."""
    root = Path(package_root).resolve()
    results = []
    expected_frame = str(config['frames']['active_map'])
    for floor_id, floor in config['floors'].items():
        pcd_path, metadata_path = _floor_asset_paths(floor, root)
        for path in (pcd_path, metadata_path):
            if not path.is_file():
                raise FileNotFoundError(f'missing surveyed asset: {path}')
        values = read_ascii_pcd(pcd_path)
        with metadata_path.open('r', encoding='utf-8') as stream:
            metadata = json.load(stream)
        if metadata.get('schema_version') != 1:
            raise ValueError(f'{metadata_path} schema_version must equal 1')
        if metadata.get('asset_kind') != 'surveyed':
            raise ValueError(f'{metadata_path} asset_kind must be surveyed')
        if metadata.get('floor_id') != floor_id:
            raise ValueError(f'{metadata_path} floor_id does not match')
        convention = metadata.get('coordinate_convention', {})
        if convention.get('asset_frame') != expected_frame:
            raise ValueError(
                f'{metadata_path} asset_frame must equal {expected_frame!r}'
            )
        if convention.get('units') != 'm':
            raise ValueError(f'{metadata_path} units must equal m')
        if metadata.get('point_count') != int(values.shape[0]):
            raise ValueError(f'{metadata_path} point_count does not match PCD')
        pcd_hash = sha256_file(pcd_path)
        if metadata.get('files', {}).get('pcd_sha256') != pcd_hash:
            raise ValueError(f'{metadata_path} PCD hash does not match')
        expected_min = np.asarray(
            metadata.get('xyz_bounds', {}).get('min', []), dtype=np.float32
        )
        expected_max = np.asarray(
            metadata.get('xyz_bounds', {}).get('max', []), dtype=np.float32
        )
        if expected_min.shape != (3,) or not np.allclose(
            expected_min, values[:, :3].min(axis=0), atol=1e-4
        ):
            raise ValueError(f'{metadata_path} minimum bounds do not match')
        if expected_max.shape != (3,) or not np.allclose(
            expected_max, values[:, :3].max(axis=0), atol=1e-4
        ):
            raise ValueError(f'{metadata_path} maximum bounds do not match')
        results.append(
            FloorAssets(
                floor_id=floor_id,
                pcd_path=pcd_path,
                metadata_path=metadata_path,
                point_count=int(values.shape[0]),
                obstacle_count=0,
                pcd_sha256=pcd_hash,
            )
        )
    return results


def validate_map_assets(
    config: Mapping[str, Any],
    package_root: Path,
) -> List[FloorAssets]:
    """Dispatch asset validation by the explicit configuration contract."""
    kind = str(config.get('system', {}).get('map_asset_kind', 'generated'))
    if kind == 'generated':
        return validate_generated_assets(config, package_root)
    if kind == 'surveyed':
        return validate_surveyed_assets(config, package_root)
    raise ValueError(f'unsupported map asset kind {kind!r}')
