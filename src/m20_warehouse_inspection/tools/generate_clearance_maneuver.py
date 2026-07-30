#!/usr/bin/env python3
#
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

"""Generate the deterministic single-obstacle clearance benchmark."""

from copy import deepcopy
import os
from pathlib import Path
import sys
import tempfile

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from m20_warehouse_inspection.configuration import (  # noqa: E402
    load_system_config,
    validate_system_config,
)
from m20_warehouse_inspection.map_assets import (  # noqa: E402
    generate_all_floor_assets,
    validate_generated_assets,
)


def _atomic_yaml(path: Path, value) -> None:
    """Write YAML atomically so an interrupted generation stays recoverable."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=path.parent,
        prefix=f'.{path.name}.',
        suffix='.tmp',
        delete=False,
    ) as stream:
        yaml.safe_dump(
            value,
            stream,
            allow_unicode=True,
            sort_keys=False,
        )
        temporary = Path(stream.name)
    os.replace(temporary, path)


def derive_profile(baseline):
    """Build one obstacle with explicit upper and lower bypass routes."""
    config = deepcopy(baseline)
    config['system']['project_id'] = 'm20_clearance_maneuver'
    config['system']['default_profile'] = 'clearance_maneuver'
    config['map_generation'].update(
        {
            'obstacle_count_per_floor': 1,
            'surface_resolution': 0.05,
            'minimum_obstacle_spacing': 0.90,
        }
    )
    config['experiment'] = {
        'kind': 'single_obstacle_double_bypass',
        'controlled_variable': 'double_circle_guard_clearance',
        'baseline': 'narrow_passage_090_system.yaml',
        'map_resolution_m': 0.05,
        'robot_body_width_m': 0.51,
    }

    obstacle = {
        'name': 'isolated_clearance_obstacle',
        # Configuration coordinates are in the shared simulation frame.
        'center': [-12.5, 0.0],
        'size': [1.2, 2.0],
    }
    config['floors']['F1'].update(
        {
            'display_name': 'clearance_maneuver_scene_1',
            'pcd_file': (
                'maps/clearance_maneuver/scene_1/scene_1.pcd'
            ),
            'occupancy_file': (
                'maps/clearance_maneuver/scene_1/scene_1.yaml'
            ),
            'metadata_file': (
                'maps/clearance_maneuver/scene_1/scene_1.json'
            ),
            'initial_pose': [-18.0, 0.0, 0.0],
            'reserved_zones': [
                {
                    'name': 'benchmark_start',
                    'center': [-18.0, 0.0],
                    'size': [2.0, 2.0],
                },
                {
                    'name': 'upper_bypass',
                    'center': [-12.5, 2.0],
                    'size': [5.0, 1.5],
                },
                {
                    'name': 'lower_bypass',
                    'center': [-12.5, -2.0],
                    'size': [5.0, 1.5],
                },
                {
                    'name': 'right_staging',
                    'center': [-7.5, 0.0],
                    'size': [2.0, 5.0],
                },
                {
                    'name': 'shared_gateway_zone',
                    'center': [-1.0, 0.0],
                    'size': [2.0, 4.0],
                },
            ],
            'fixed_obstacles': [obstacle],
            'inspection_points': [
                {
                    'name': 'F1_UPPER_LEFT',
                    'pose': [-14.0, 2.0, 0.0],
                    'dwell_sec': 0.1,
                },
                {
                    'name': 'F1_UPPER_RIGHT',
                    'pose': [-11.0, 2.0, 0.0],
                    'dwell_sec': 0.1,
                },
                {
                    'name': 'F1_RIGHT_STAGING',
                    'pose': [-7.5, 0.0, 0.0],
                    'dwell_sec': 0.1,
                },
                {
                    'name': 'F1_LOWER_RIGHT',
                    'pose': [-11.0, -2.0, 3.141592654],
                    'dwell_sec': 0.1,
                },
                {
                    'name': 'F1_LOWER_LEFT',
                    'pose': [-14.0, -2.0, 3.141592654],
                    'dwell_sec': 0.1,
                },
                {
                    'name': 'F1_RETURN_START',
                    'pose': [-18.0, 0.0, 3.141592654],
                    'dwell_sec': 0.1,
                },
            ],
        }
    )
    config['floors']['F2'].update(
        {
            'display_name': 'clearance_maneuver_scene_2',
            'pcd_file': (
                'maps/clearance_maneuver/scene_2/scene_2.pcd'
            ),
            'occupancy_file': (
                'maps/clearance_maneuver/scene_2/scene_2.yaml'
            ),
            'metadata_file': (
                'maps/clearance_maneuver/scene_2/scene_2.json'
            ),
        }
    )
    config['mission'] = {
        'id': 'single_obstacle_double_bypass',
        'loop': False,
        'sequence': [
            {
                'type': 'inspection',
                'floor': 'F1',
                'point': point,
            }
            for point in (
                'F1_UPPER_LEFT',
                'F1_UPPER_RIGHT',
                'F1_RIGHT_STAGING',
                'F1_LOWER_RIGHT',
                'F1_LOWER_LEFT',
                'F1_RETURN_START',
            )
        ],
    }
    validate_system_config(config)
    return config


def derive_direct_profile(double_bypass):
    """Reuse the same obstacle while letting SCAN choose one bypass."""
    config = deepcopy(double_bypass)
    config['system']['project_id'] = 'm20_clearance_direct'
    config['system']['default_profile'] = 'clearance_direct'
    config['experiment']['kind'] = 'single_obstacle_direct_bypass'
    config['floors']['F1']['inspection_points'].append(
        {
            'name': 'F1_DIRECT_EXIT',
            'pose': [-7.0, 0.0, 0.0],
            'dwell_sec': 0.1,
        }
    )
    config['mission'] = {
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
    validate_system_config(config)
    return config


def main() -> int:
    """Generate and validate the source YAML, PCD, PGM, and metadata."""
    baseline = load_system_config(
        PACKAGE_ROOT / 'config' / 'narrow_passage_090_system.yaml'
    )
    profile = derive_profile(baseline)
    config_path = (
        PACKAGE_ROOT / 'config' / 'clearance_maneuver_system.yaml'
    )
    _atomic_yaml(config_path, profile)
    loaded = load_system_config(config_path)
    generated = generate_all_floor_assets(loaded, PACKAGE_ROOT)
    validate_generated_assets(loaded, PACKAGE_ROOT)
    direct_path = (
        PACKAGE_ROOT / 'config' / 'clearance_direct_system.yaml'
    )
    direct = derive_direct_profile(profile)
    _atomic_yaml(direct_path, direct)
    validate_generated_assets(load_system_config(direct_path), PACKAGE_ROOT)
    for assets in generated:
        print(
            f'{assets.floor_id}: obstacles={assets.obstacle_count}, '
            f'points={assets.point_count}'
        )
    print('direct profile: shared collision assets validated')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
