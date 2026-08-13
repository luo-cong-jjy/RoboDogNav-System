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

"""Generate controlled doorway-width profiles for native SCAN/MuJoCo tests."""

import argparse
from copy import deepcopy
import os
from pathlib import Path
import sys
import tempfile
from typing import Dict, Iterable, Mapping, Sequence, Tuple

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


PROFILES: Tuple[Tuple[str, float], ...] = (
    ('060', 0.60),
    ('065', 0.65),
    ('070', 0.70),
    ('075', 0.75),
    ('080', 0.80),
    ('090', 0.90),
)


def _atomic_yaml(path: Path, value: Mapping) -> None:
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


def derive_profile(
    baseline: Mapping,
    tag: str,
    passage_width: float,
) -> Dict:
    """Derive a two-wall doorway with no alternate M20-sized route."""
    config = deepcopy(baseline)
    config['system']['project_id'] = f'm20_narrow_passage_{tag}'
    config['system']['default_profile'] = f'narrow_passage_{tag}'
    config['simulation']['floor_size_x'] = 20.0
    config['simulation']['floor_size_y'] = 20.0
    config['map_generation'].update(
        {
            'obstacle_count_per_floor': 2,
            'surface_resolution': 0.05,
            'boundary_clearance': 0.50,
            'reserved_zone_clearance': 0.10,
            # The controlled variable is the doorway, not random spacing.
            'minimum_obstacle_spacing': 0.10,
        }
    )
    config['experiment'] = {
        'kind': 'narrow_passage_sweep',
        'controlled_variable': 'doorway_clear_width',
        'value_m': passage_width,
        'baseline': 'dense_four_corner_system.yaml',
        'map_resolution_m': 0.05,
        'robot_body_width_m': 0.51,
    }

    wall_x = -12.5
    outer_y = 9.5
    inner_y = passage_width / 2.0
    wall_depth = outer_y - inner_y
    wall_center_y = (outer_y + inner_y) / 2.0
    config['floors']['F1'].update(
        {
            'display_name': f'narrow_{tag}_scene_1',
            'simulation_offset': [-10.0, 0.0, 0.0],
            'pcd_file': (
                f'maps/narrow_passage/width_{tag}/scene_1/scene_1.pcd'
            ),
            'metadata_file': (
                f'maps/narrow_passage/width_{tag}/scene_1/scene_1.json'
            ),
            'initial_pose': [-18.0, 0.0, 0.0],
            'reserved_zones': [
                {
                    'name': 'benchmark_start',
                    'center': [-18.0, 0.0],
                    'size': [2.0, 2.0],
                },
                {
                    'name': 'benchmark_finish',
                    'center': [-7.0, 0.0],
                    'size': [2.0, 2.0],
                },
                {
                    'name': 'shared_gateway_zone',
                    'center': [-1.0, 0.0],
                    'size': [2.0, 4.0],
                },
            ],
            'fixed_obstacles': [
                {
                    'name': 'doorway_upper_wall',
                    'center': [wall_x, wall_center_y],
                    'size': [1.0, wall_depth],
                },
                {
                    'name': 'doorway_lower_wall',
                    'center': [wall_x, -wall_center_y],
                    'size': [1.0, wall_depth],
                },
            ],
            'inspection_points': [
                {
                    'name': 'F1_DOORWAY_EXIT',
                    'pose': [-7.0, 0.0, 0.0],
                    'dwell_sec': 0.2,
                }
            ],
        }
    )
    config['floors']['F2'].update(
        {
            'display_name': f'narrow_{tag}_scene_2',
            'simulation_offset': [10.0, 0.0, 0.0],
            'pcd_file': (
                f'maps/narrow_passage/width_{tag}/scene_2/scene_2.pcd'
            ),
            'metadata_file': (
                f'maps/narrow_passage/width_{tag}/scene_2/scene_2.json'
            ),
            'initial_pose': [2.0, 0.0, 0.0],
            'reserved_zones': [
                {
                    'name': 'benchmark_start',
                    'center': [2.0, 0.0],
                    'size': [2.0, 2.0],
                },
                {
                    'name': 'benchmark_finish',
                    'center': [13.0, 0.0],
                    'size': [2.0, 2.0],
                },
                {
                    'name': 'shared_gateway_zone',
                    'center': [1.0, 0.0],
                    'size': [2.0, 4.0],
                },
            ],
            'inspection_points': [
                {
                    'name': 'F2_DOORWAY_EXIT',
                    'pose': [13.0, 0.0, 0.0],
                    'dwell_sec': 0.2,
                }
            ],
        }
    )
    config['mission'] = {
        'id': 'narrow_passage_benchmark',
        'loop': False,
        'sequence': [
            {
                'type': 'inspection',
                'floor': 'F1',
                'point': 'F1_DOORWAY_EXIT',
            }
        ],
    }
    validate_system_config(config)
    return config


def generate_profiles(
    package_root: Path,
    baseline_path: Path,
    selected_tags: Iterable[str],
) -> None:
    """Write and validate every requested nominal doorway width."""
    root = package_root.resolve()
    baseline = load_system_config(baseline_path)
    selected = set(selected_tags)
    unknown = selected.difference(tag for tag, _ in PROFILES)
    if unknown:
        raise ValueError(f'unknown passage profile(s): {sorted(unknown)}')

    for tag, width in PROFILES:
        if tag not in selected:
            continue
        config = derive_profile(baseline, tag, width)
        config_path = (
            root / 'config' / f'narrow_passage_{tag}_system.yaml'
        )
        _atomic_yaml(config_path, config)
        loaded = load_system_config(config_path)
        generated = generate_all_floor_assets(loaded, root)
        validate_generated_assets(loaded, root)
        summary = ', '.join(
            f'{asset.floor_id}:{asset.point_count}pts'
            for asset in generated
        )
        print(f'width_{tag}: clear={width:.2f}m, {summary}')


def _parse_arguments(arguments: Sequence[str] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate controlled M20 doorway-width profiles.'
    )
    parser.add_argument(
        '--package-root',
        type=Path,
        default=PACKAGE_ROOT,
    )
    parser.add_argument(
        '--baseline',
        type=Path,
        default=PACKAGE_ROOT / 'config' / 'dense_four_corner_system.yaml',
    )
    parser.add_argument(
        '--profile',
        action='append',
        choices=[tag for tag, _ in PROFILES],
        dest='profiles',
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] = None) -> int:
    options = _parse_arguments(arguments)
    generate_profiles(
        options.package_root,
        options.baseline,
        options.profiles or [tag for tag, _ in PROFILES],
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
