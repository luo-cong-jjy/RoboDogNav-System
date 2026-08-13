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

"""Regression checks for the controlled M20 doorway-width benchmark."""

from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import yaml

from m20_inspection_core.collision_policy import (
    GridGeometry,
    inflate_blocked_grid,
)
from m20_warehouse_inspection.configuration import load_system_config
from m20_warehouse_inspection.clearance_dynamics_probe import (
    load_obstacle_rectangles,
    minimum_double_circle_distance,
)
from m20_warehouse_inspection.map_assets import (
    _build_occupancy,
    validate_generated_assets,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SYSTEM_ROOT = PACKAGE_ROOT.parents[1]
NAVIGATION_ROOT = SYSTEM_ROOT / 'navigation'
SAFETY_ROOT = SYSTEM_ROOT / 'safety_mission'
PROFILES = (
    ('060', 0.60),
    ('065', 0.65),
    ('070', 0.70),
    ('075', 0.75),
    ('080', 0.80),
    ('090', 0.90),
)


def _reachable(blocked: np.ndarray, start, finish) -> bool:
    def cell(point):
        return (
            int(math.floor((point[0] + 10.0) / 0.05)),
            int(math.floor((point[1] + 10.0) / 0.05)),
        )

    source = cell(start)
    target = cell(finish)
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


def test_narrow_passage_assets_have_one_controlled_doorway() -> None:
    inflated_results = {}
    for tag, requested_width in PROFILES:
        config_path = (
            PACKAGE_ROOT
            / 'config'
            / f'narrow_passage_{tag}_system.yaml'
        )
        config = load_system_config(config_path)
        assert config['experiment']['kind'] == 'narrow_passage_sweep'
        assert config['experiment']['value_m'] == requested_width
        assert config['map_generation']['obstacle_count_per_floor'] == 2
        assets = validate_generated_assets(config, PACKAGE_ROOT)
        assert len(assets) == 2

        with assets[0].metadata_path.open(
            'r', encoding='utf-8'
        ) as stream:
            metadata = json.load(stream)
        upper, lower = sorted(
            metadata['fixed_obstacles'],
            key=lambda item: item['center'][1],
            reverse=True,
        )
        realized_width = upper['y_min'] - lower['y_max']
        assert math.isclose(
            realized_width,
            requested_width,
            rel_tol=0.0,
            abs_tol=1e-9,
        )

        obstacle_rectangles = [
            (
                item['x_min'],
                item['x_max'],
                item['y_min'],
                item['y_max'],
            )
            for item in metadata['obstacles']
        ]
        occupancy = _build_occupancy(
            obstacle_rectangles,
            10.0,
            10.0,
            0.05,
            metadata.get('transition_gateway'),
        )
        geometry = GridGeometry(
            width=400,
            height=400,
            resolution=0.05,
            origin_x=-10.0,
            origin_y=-10.0,
        )
        blocked = inflate_blocked_grid(
            occupancy.ravel(),
            geometry,
            occupied_threshold=50,
            radius=0.30,
        )
        inflated_results[tag] = _reachable(
            blocked,
            start=(-8.0, 0.0),
            finish=(3.0, 0.0),
        )

    # Occupied cells and the inclusive 0.30 m kernel add discretization
    # conservatism. At 0.05 m resolution, 0.75 m is the first tested width
    # that retains a centreline cell.
    assert all(
        not inflated_results[tag]
        for tag in ('060', '065', '070')
    )
    assert all(
        inflated_results[tag] for tag in ('075', '080', '090')
    )


def test_narrow_passage_launch_exposes_width_and_clearance() -> None:
    launch = (
        PACKAGE_ROOT
        / 'launch'
        / 'narrow_passage_mission_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    for tag, width in PROFILES:
        assert repr(tag) in launch
        assert repr(f'{width:.2f}') in launch
    for profile in (
        'vendor', 'm20', 'tight', 'balanced', 'conservative'
    ):
        assert repr(profile) in launch
    assert "default_value='m20'" in launch
    assert 'inspection_mission_mujoco.launch.py' in launch


def test_production_guard_keeps_geometry_and_measured_stop_horizon() -> None:
    navigation = NAVIGATION_ROOT / 'm20_scan_navigation'
    core = SAFETY_ROOT / 'm20_inspection_core'
    with (
        navigation / 'config' / 'clearance_conservative.yaml'
    ).open('r', encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    planner = profile['scan_planner_node']['ros__parameters']
    assert planner['grid_map.double_cylinder_radius'] == 0.25
    assert planner['optimization.dist0'] == 0.20
    guard = profile['m20_collision_guard']['ros__parameters']
    assert guard['footprint_radius'] == 0.25
    assert guard['footprint_offset'] == 0.18
    assert guard['safety_margin'] == 0.05
    assert guard['lookahead_sec'] == 0.70
    assert planner['grid_map.double_cylinder_radius'] == guard[
        'footprint_radius'
    ]
    assert math.isclose(
        planner['grid_map.double_cylinder_radius']
        + planner['optimization.dist0'],
        0.45,
        abs_tol=1.0e-9,
    )
    assert 0.45 * guard['lookahead_sec'] >= 2.0 * 0.156

    with (
        core / 'config' / 'collision_guard_scan_native.yaml'
    ).open('r', encoding='utf-8') as stream:
        native_guard = yaml.safe_load(stream)[
            'm20_collision_guard'
        ]['ros__parameters']
    for key in (
        'footprint_radius',
        'footprint_offset',
        'safety_margin',
        'lookahead_sec',
    ):
        assert native_guard[key] == guard[key]

    source = (
        core
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "self.declare_parameter('footprint_radius', 0.25)" in source
    assert "self.declare_parameter('safety_margin', 0.05)" in source
    assert "self.declare_parameter('lookahead_sec', 0.70)" in source


def test_default_m20_clearance_moves_margin_into_hard_geometry() -> None:
    navigation = NAVIGATION_ROOT / 'm20_scan_navigation'
    with (
        navigation / 'config' / 'clearance_m20.yaml'
    ).open('r', encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    planner = profile['scan_planner_node']['ros__parameters']
    guard = profile['m20_collision_guard']['ros__parameters']
    assert planner['grid_map.double_cylinder_radius'] == 0.28
    assert planner['grid_map.double_cylinder_offset'] == 0.18
    assert planner['optimization.dist0'] == 0.17
    assert math.isclose(
        planner['grid_map.double_cylinder_radius']
        + planner['optimization.dist0'],
        0.45,
        abs_tol=1.0e-9,
    )
    assert guard['footprint_radius'] == 0.28
    assert guard['footprint_offset'] == 0.18
    assert guard['safety_margin'] == 0.02


def test_dynamic_probe_measures_world_frame_doorway_clearance() -> None:
    rectangles = load_obstacle_rectangles(
        PACKAGE_ROOT / 'config' / 'narrow_passage_090_system.yaml'
    )
    distance = minimum_double_circle_distance(
        -12.5,
        0.0,
        0.0,
        rectangles,
        footprint_offset=0.18,
    )
    # The circle centres remain on the corridor centreline, 0.45 m from
    # either obstacle surface in the nominal 0.90 m doorway.
    assert math.isclose(distance, 0.45, abs_tol=1.0e-9)


def test_single_obstacle_benchmark_covers_both_sides_and_returns() -> None:
    config = load_system_config(
        PACKAGE_ROOT / 'config' / 'clearance_maneuver_system.yaml'
    )
    assert config['experiment']['kind'] == (
        'single_obstacle_double_bypass'
    )
    sequence = [
        step['point'] for step in config['mission']['sequence']
    ]
    assert sequence == [
        'F1_UPPER_LEFT',
        'F1_UPPER_RIGHT',
        'F1_RIGHT_STAGING',
        'F1_LOWER_RIGHT',
        'F1_LOWER_LEFT',
        'F1_RETURN_START',
    ]
    points = {
        item['name']: item['pose']
        for item in config['floors']['F1']['inspection_points']
    }
    assert points['F1_UPPER_LEFT'][1] > 0.0
    assert points['F1_LOWER_LEFT'][1] < 0.0
    assert points['F1_RETURN_START'][:2] == [-18.0, 0.0]
    assets = validate_generated_assets(config, PACKAGE_ROOT)
    assert all(item.obstacle_count == 1 for item in assets)
    rectangles = load_obstacle_rectangles(
        PACKAGE_ROOT / 'config' / 'clearance_maneuver_system.yaml'
    )
    assert any(
        all(
            math.isclose(actual, expected, abs_tol=1.0e-9)
            for actual, expected in zip(
                rectangle,
                (-13.1, -11.9, -1.0, 1.0),
            )
        )
        for rectangle in rectangles
    )


def test_direct_bypass_reuses_collision_world_without_subgoals() -> None:
    config = load_system_config(
        PACKAGE_ROOT / 'config' / 'clearance_direct_system.yaml'
    )
    assert config['experiment']['kind'] == (
        'single_obstacle_direct_bypass'
    )
    assert config['mission'] == {
        'id': 'single_obstacle_direct_bypass',
        'loop': False,
        'sequence': [
            {
                'type': 'inspection',
                'floor': 'F1',
                'point': 'F1_DIRECT_EXIT',
            }
        ],
    }
    validate_generated_assets(config, PACKAGE_ROOT)
