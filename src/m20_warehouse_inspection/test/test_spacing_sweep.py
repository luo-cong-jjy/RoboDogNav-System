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

"""Regression checks for the controlled obstacle-spacing experiment."""

from collections import deque
import json
import math
from pathlib import Path

import numpy as np

from m20_inspection_core.collision_policy import (
    GridGeometry,
    inflate_blocked_grid,
)
from m20_warehouse_inspection.configuration import load_system_config
from m20_warehouse_inspection.map_assets import (
    read_pgm,
    validate_generated_assets,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILES = (
    ('070', 0.70),
    ('080', 0.80),
    ('090', 0.90),
    ('100', 1.00),
)
LOCAL_LEGS = {
    'F1': (
        ((-17.0, 0.0), (-16.0, -16.0)),
        ((-16.0, -16.0), (16.0, -16.0)),
        ((16.0, -16.0), (16.0, 16.0)),
        ((16.0, 16.0), (-16.0, 16.0)),
        ((-16.0, 16.0), (20.0, 0.0)),
        ((20.0, 0.0), (-17.0, 0.0)),
    ),
    'F2': (
        ((-20.0, 0.0), (-16.0, -16.0)),
        ((-16.0, -16.0), (16.0, -16.0)),
        ((16.0, -16.0), (16.0, 16.0)),
        ((16.0, 16.0), (-16.0, 16.0)),
        ((-16.0, 16.0), (-20.0, 0.0)),
    ),
}


def _separation(left, right) -> float:
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
    return math.hypot(separation_x, separation_y)


def _cell(point):
    return (
        min(399, max(0, int((point[0] + 20.0) / 0.10))),
        min(399, max(0, int((point[1] + 20.0) / 0.10))),
    )


def _reachable(blocked: np.ndarray, start, finish) -> bool:
    source = _cell(start)
    target = _cell(finish)
    if blocked[source[1], source[0]] or blocked[target[1], target[0]]:
        return False
    queue = deque([source])
    visited = np.zeros_like(blocked, dtype=np.bool_)
    visited[source[1], source[0]] = True
    while queue:
        x, y = queue.popleft()
        if (x, y) == target:
            return True
        for next_x, next_y in (
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
        ):
            if (
                0 <= next_x < blocked.shape[1]
                and 0 <= next_y < blocked.shape[0]
                and not visited[next_y, next_x]
                and not blocked[next_y, next_x]
            ):
                visited[next_y, next_x] = True
                queue.append((next_x, next_y))
    return False


def test_spacing_sweep_assets_and_inflated_routes() -> None:
    """Keep every variant controlled, spaced, and 0.30 m traversable."""
    fixed_reference = None
    for tag, requested_spacing in PROFILES:
        config_path = (
            PACKAGE_ROOT
            / 'config'
            / f'spacing_sweep_{tag}_system.yaml'
        )
        config = load_system_config(config_path)
        assert config['experiment']['kind'] == 'obstacle_spacing_sweep'
        assert config['experiment']['value_m'] == requested_spacing
        assert (
            config['map_generation']['minimum_obstacle_spacing']
            == requested_spacing
        )
        assert config['map_generation']['obstacle_count_per_floor'] == 252
        assert config['mission']['id'] == (
            'spacing_sweep_four_corner_patrol'
        )
        fixed = config['floors']['F1']['fixed_obstacles']
        if fixed_reference is None:
            fixed_reference = fixed
        else:
            assert fixed == fixed_reference

        assets = validate_generated_assets(config, PACKAGE_ROOT)
        assert len(assets) == 2
        for asset in assets:
            with asset.metadata_path.open(
                'r', encoding='utf-8'
            ) as stream:
                metadata = json.load(stream)
            obstacles = metadata['obstacles']
            realized = min(
                _separation(left, right)
                for index, left in enumerate(obstacles)
                for right in obstacles[index + 1:]
            )
            assert realized >= requested_spacing - 1e-9

            image = read_pgm(asset.occupancy_image_path)
            occupancy = np.where(np.flipud(image) == 0, 100, 0)
            geometry = GridGeometry(
                width=400,
                height=400,
                resolution=0.10,
                origin_x=-20.0,
                origin_y=-20.0,
            )
            blocked = inflate_blocked_grid(
                occupancy.ravel(),
                geometry,
                occupied_threshold=50,
                radius=0.30,
            )
            assert all(
                _reachable(blocked, start, finish)
                for start, finish in LOCAL_LEGS[asset.floor_id]
            )


def test_spacing_sweep_launch_selects_only_supported_profiles() -> None:
    launch = (
        PACKAGE_ROOT
        / 'launch'
        / 'spacing_sweep_mission_rviz.launch.py'
    ).read_text(encoding='utf-8')
    for tag, spacing in PROFILES:
        assert repr(f'{spacing:.2f}') in launch
        assert repr(tag) in launch
        assert 'spacing_sweep_{tag}_system.yaml' in launch
    assert 'inspection_mission_rviz.launch.py' in launch
