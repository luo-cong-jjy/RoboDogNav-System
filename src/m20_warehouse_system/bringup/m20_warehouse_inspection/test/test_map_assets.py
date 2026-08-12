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

"""Tests for deterministic floor-map generation and validation."""

from collections import deque
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from m20_inspection_core.collision_policy import (
    GridGeometry,
    inflate_blocked_grid,
)
from m20_warehouse_inspection.configuration import load_system_config
from m20_warehouse_inspection.map_assets import (
    _build_occupancy,
    generate_all_floor_assets,
    import_surveyed_ascii_pcd,
    read_ascii_pcd,
    validate_surveyed_assets,
    validate_generated_assets,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'flat_multifloor_system.yaml'
DENSE_CONFIG_PATH = (
    PACKAGE_ROOT / 'config' / 'dense_four_corner_system.yaml'
)
CHALLENGE_CONFIG_PATH = (
    PACKAGE_ROOT / 'config' / 'route_challenge_system.yaml'
)


def _assert_inflated_targets_connected(
    assets,
    floor_targets,
    *,
    inflation_radius: float = 0.30,
) -> None:
    """Check that every required target shares one inflated free-space region."""
    for asset in assets:
        with asset.metadata_path.open('r', encoding='utf-8') as stream:
            metadata = json.load(stream)
        obstacles = [
            (
                item['x_min'],
                item['x_max'],
                item['y_min'],
                item['y_max'],
            )
            for item in metadata['obstacles']
        ]
        half_x = float(metadata['source_bounds']['x'][1])
        half_y = float(metadata['source_bounds']['y'][1])
        resolution = float(metadata['surface_resolution'])
        occupancy = _build_occupancy(
            obstacles,
            half_x,
            half_y,
            resolution,
            metadata.get('transition_gateway'),
        )
        geometry = GridGeometry(
            width=occupancy.shape[1],
            height=occupancy.shape[0],
            resolution=resolution,
            origin_x=-half_x,
            origin_y=-half_y,
        )
        blocked = inflate_blocked_grid(
            occupancy.ravel(),
            geometry,
            occupied_threshold=50,
            radius=inflation_radius,
        )

        def _cell(point):
            x = min(
                geometry.width - 1,
                max(
                    0,
                    int(
                        (point[0] - geometry.origin_x)
                        / geometry.resolution
                    ),
                ),
            )
            y = min(
                geometry.height - 1,
                max(
                    0,
                    int(
                        (point[1] - geometry.origin_y)
                        / geometry.resolution
                    ),
                ),
            )
            return x, y

        targets = [_cell(point) for point in floor_targets[asset.floor_id]]
        assert all(not blocked[y, x] for x, y in targets)
        start = targets[0]
        visited = np.zeros_like(blocked)
        visited[start[1], start[0]] = True
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for next_x, next_y in (
                (x - 1, y),
                (x + 1, y),
                (x, y - 1),
                (x, y + 1),
            ):
                if (
                    0 <= next_x < geometry.width
                    and 0 <= next_y < geometry.height
                    and not visited[next_y, next_x]
                    and not blocked[next_y, next_x]
                ):
                    visited[next_y, next_x] = True
                    queue.append((next_x, next_y))
        assert all(visited[y, x] for x, y in targets)


def _small_generation_config():
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['map_generation']['obstacle_count_per_floor'] = 12
    config['map_generation']['surface_resolution'] = 0.25
    return config


def test_surveyed_pcd_import_selects_fields_and_audits_hash(tmp_path):
    source = tmp_path / 'elevator_lio_ascii.pcd'
    source.write_text(
        '# .PCD v0.7\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity normal_x curvature\n'
        'SIZE 4 4 4 4 4 4\n'
        'TYPE F F F F F F\n'
        'COUNT 1 1 1 1 1 1\n'
        'WIDTH 3\nHEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS 3\nDATA ascii\n'
        '0 0 0 10 0 0\n'
        '1 2 0.5 20 0 0\n'
        'nan 3 0.5 30 0 0\n',
        encoding='utf-8',
    )
    pcd = tmp_path / 'maps' / 'static_map.pcd'
    metadata = tmp_path / 'maps' / 'static_map.json'
    asset = import_surveyed_ascii_pcd(
        source, pcd, metadata, 'F1', asset_frame='world'
    )

    assert asset.point_count == 2
    values = read_ascii_pcd(pcd)
    assert values.shape == (2, 4)
    assert np.array_equal(values[:, 3], np.array([10.0, 20.0]))
    document = json.loads(metadata.read_text(encoding='utf-8'))
    assert document['asset_kind'] == 'surveyed'
    assert document['normalization']['non_finite_points_removed'] == 1

    config = {
        'system': {'map_asset_kind': 'surveyed'},
        'frames': {'active_map': 'world'},
        'floors': {
            'F1': {
                'pcd_file': 'maps/static_map.pcd',
                'metadata_file': 'maps/static_map.json',
            }
        },
    }
    validated = validate_surveyed_assets(config, tmp_path)
    assert validated[0].pcd_sha256 == asset.pcd_sha256


def test_shipped_assets_match_configuration() -> None:
    config = load_system_config(CONFIG_PATH)
    assets = validate_generated_assets(config, PACKAGE_ROOT)
    assert [asset.floor_id for asset in assets] == ['F1', 'F2']
    assert assets[0].pcd_sha256 != assets[1].pcd_sha256
    first_points = read_ascii_pcd(assets[0].pcd_path)
    second_points = read_ascii_pcd(assets[1].pcd_path)
    first_interior = first_points[
        np.logical_and(
            np.abs(first_points[:, 0]) < 20.0 - 1e-4,
            np.abs(first_points[:, 1]) < 20.0 - 1e-4,
        )
    ]
    second_interior = second_points[
        np.logical_and(
            np.abs(second_points[:, 0]) < 20.0 - 1e-4,
            np.abs(second_points[:, 1]) < 20.0 - 1e-4,
        )
    ]
    assert np.array_equal(
        first_interior,
        second_interior,
    )
    assert all(asset.obstacle_count == 180 for asset in assets)


def test_shipped_native_scan_mission_legs_have_clearance() -> None:
    """Keep deterministic obstacles clear of every direct automatic leg."""
    local_legs = {
        'floor_1': (
            ((-17.0, 0.0), (-10.0, -10.0)),
            ((-10.0, -10.0), (5.0, 10.0)),
            ((5.0, 10.0), (20.0, 0.0)),
        ),
        'floor_2': (
            ((-20.0, 0.0), (-10.0, -10.0)),
            ((-10.0, -10.0), (5.0, 10.0)),
            ((5.0, 10.0), (-10.0, -10.0)),
            ((-10.0, -10.0), (-20.0, 0.0)),
        ),
    }
    for directory, legs in local_legs.items():
        metadata_path = PACKAGE_ROOT / 'maps' / directory / (
            f'{directory}.json'
        )
        with metadata_path.open('r', encoding='utf-8') as stream:
            obstacles = json.load(stream)['obstacles']
        for start, finish in legs:
            for ratio in np.linspace(0.0, 1.0, 101):
                x = start[0] + ratio * (finish[0] - start[0])
                y = start[1] + ratio * (finish[1] - start[1])
                assert not any(
                    obstacle['x_min'] - 0.80 <= x
                    <= obstacle['x_max'] + 0.80
                    and obstacle['y_min'] - 0.80 <= y
                    <= obstacle['y_max'] + 0.80
                    for obstacle in obstacles
                )


def test_dense_shipped_assets_have_fixed_and_spaced_obstacles() -> None:
    config = load_system_config(DENSE_CONFIG_PATH)
    assets = validate_generated_assets(config, PACKAGE_ROOT)
    assert all(asset.obstacle_count == 246 for asset in assets)
    for asset in assets:
        with asset.metadata_path.open('r', encoding='utf-8') as stream:
            metadata = json.load(stream)
        assert metadata['minimum_obstacle_spacing'] == 0.90
        assert metadata['fixed_obstacle_count'] == 6
        assert {
            obstacle['name'] for obstacle in metadata['fixed_obstacles']
        } == {
            'bottom_aisle_west',
            'bottom_aisle_east',
            'right_aisle_lower',
            'right_aisle_upper',
            'top_aisle_east',
            'top_aisle_west',
        }
        obstacles = metadata['obstacles']
        for index, left in enumerate(obstacles):
            for right in obstacles[index + 1:]:
                separation_x = max(
                    float(right['x_min']) - float(left['x_max']),
                    float(left['x_min']) - float(right['x_max']),
                    0.0,
                )
                separation_y = max(
                    float(right['y_min']) - float(left['y_max']),
                    float(left['y_min']) - float(right['y_max']),
                    0.0,
                )
                assert np.hypot(separation_x, separation_y) >= 0.90 - 1e-9

    _assert_inflated_targets_connected(
        assets,
        {
            'F1': (
                (-17.0, 0.0),
                (-16.0, -16.0),
                (16.0, -16.0),
                (16.0, 16.0),
                (-16.0, 16.0),
                (20.0, 0.0),
            ),
            'F2': (
                (-20.0, 0.0),
                (-16.0, -16.0),
                (16.0, -16.0),
                (16.0, 16.0),
                (-16.0, 16.0),
            ),
        },
    )


def test_route_challenge_obstructs_nominal_legs_but_remains_traversable() -> None:
    """Require real detours while retaining footprint-inflated connectivity."""
    config = load_system_config(CHALLENGE_CONFIG_PATH)
    assets = validate_generated_assets(config, PACKAGE_ROOT)
    assert all(asset.obstacle_count == 252 for asset in assets)
    with assets[0].metadata_path.open(encoding='utf-8') as stream:
        metadata = json.load(stream)

    fixed = metadata['fixed_obstacles']
    assert len(fixed) == 12
    nominal_legs = (
        ((-17.0, 0.0), (-16.0, -16.0)),
        ((-16.0, -16.0), (16.0, -16.0)),
        ((16.0, -16.0), (16.0, 16.0)),
        ((16.0, 16.0), (-16.0, 16.0)),
        ((-16.0, 16.0), (20.0, 0.0)),
        ((-20.0, 0.0), (-16.0, -16.0)),
        ((20.0, 0.0), (-17.0, 0.0)),
    )
    for start, finish in nominal_legs:
        samples = [
            (
                start[0] + ratio * (finish[0] - start[0]),
                start[1] + ratio * (finish[1] - start[1]),
            )
            for ratio in np.linspace(0.0, 1.0, 1001)
        ]
        assert any(
            obstacle['x_min'] <= x <= obstacle['x_max']
            and obstacle['y_min'] <= y <= obstacle['y_max']
            for x, y in samples
            for obstacle in fixed
        )

    floor_targets = {
        'F1': (
            (-17.0, 0.0),
            (-16.0, -16.0),
            (16.0, -16.0),
            (16.0, 16.0),
            (-16.0, 16.0),
            (20.0, 0.0),
        ),
        'F2': (
            (-20.0, 0.0),
            (-16.0, -16.0),
            (16.0, -16.0),
            (16.0, 16.0),
            (-16.0, 16.0),
        ),
    }
    _assert_inflated_targets_connected(assets, floor_targets)


def test_generation_is_reproducible(tmp_path: Path) -> None:
    config = _small_generation_config()
    first_root = tmp_path / 'first'
    second_root = tmp_path / 'second'
    first = generate_all_floor_assets(config, first_root)
    second = generate_all_floor_assets(config, second_root)
    assert [asset.pcd_sha256 for asset in first] == [
        asset.pcd_sha256 for asset in second
    ]
    assert first[0].pcd_sha256 != first[1].pcd_sha256


def test_generated_files_have_expected_geometry(tmp_path: Path) -> None:
    config = _small_generation_config()
    assets = generate_all_floor_assets(config, tmp_path)
    checked = validate_generated_assets(config, tmp_path)
    assert len(checked) == 2
    for asset in assets:
        points = read_ascii_pcd(asset.pcd_path)
        assert tuple(points[:, :3].min(axis=0)) == (-20.0, -20.0, 0.0)
        assert tuple(points[:, :3].max(axis=0)) == (20.0, 20.0, 2.0)
        with asset.metadata_path.open('r', encoding='utf-8') as stream:
            metadata = json.load(stream)
        assert metadata['coordinate_convention']['asset_frame'] == 'floor_local'
        assert metadata['obstacle_count'] == 12
        assert 'occupancy_image' not in metadata['files']
        assert 'occupancy_yaml' not in metadata['files']
    with assets[1].metadata_path.open('r', encoding='utf-8') as stream:
        assert json.load(stream)['replica_of'] == 'F1'
