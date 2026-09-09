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

"""Generate controlled 0.70/0.80/0.90/1.00 m obstacle-spacing profiles."""

import argparse
from copy import deepcopy
import json
import math
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
    ('070', 0.70),
    ('080', 0.80),
    ('090', 0.90),
    ('100', 1.00),
)


def _atomic_yaml(path: Path, value: Mapping) -> None:
    """Write one generated profile without exposing a partial YAML file."""
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
    spacing: float,
) -> Dict:
    """Return a spacing-only experiment derived from route_challenge."""
    config = deepcopy(baseline)
    config['system']['project_id'] = f'm20_warehouse_spacing_{tag}'
    config['system']['default_profile'] = f'rviz_spacing_{tag}'
    config['map_generation']['minimum_obstacle_spacing'] = spacing
    config['experiment'] = {
        'kind': 'obstacle_spacing_sweep',
        'controlled_variable': 'minimum_obstacle_spacing',
        'value_m': spacing,
        'baseline': 'route_challenge_system.yaml',
        'constant_obstacle_count_per_floor': int(
            config['map_generation']['obstacle_count_per_floor']
        ),
        'constant_source_seed': int(config['floors']['F1']['source_seed']),
    }

    # The original challenge pair is 0.80 m apart.  Move only the non-route
    # member of that pair so every sweep profile shares one fixed layout that
    # also satisfies the 1.00 m case.  A 0.05 m guard avoids boundary equality
    # being rejected by the configuration validator.
    fixed = {
        obstacle['name']: obstacle
        for obstacle in config['floors']['F1']['fixed_obstacles']
    }
    fixed['inbound_left_pallet']['center'] = [-38.35, -7.7]

    for floor_id, floor in config['floors'].items():
        scene = 'scene_1' if floor_id == 'F1' else 'scene_2'
        relative = Path('maps') / 'spacing_sweep' / f'spacing_{tag}' / scene
        floor['display_name'] = f'spacing_{tag}_{scene}'
        floor['pcd_file'] = str(relative / f'{scene}.pcd')
        floor['metadata_file'] = str(relative / f'{scene}.json')

    # Keep the second command identical while the launch argument selects the
    # active experiment profile.
    config['mission']['id'] = 'spacing_sweep_four_corner_patrol'
    validate_system_config(config)
    return config


def _rectangle_separation(left: Mapping, right: Mapping) -> float:
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


def _minimum_realized_spacing(metadata_path: Path) -> float:
    with metadata_path.open('r', encoding='utf-8') as stream:
        obstacles = json.load(stream)['obstacles']
    return min(
        _rectangle_separation(left, right)
        for index, left in enumerate(obstacles)
        for right in obstacles[index + 1:]
    )


def generate_profiles(
    package_root: Path,
    baseline_path: Path,
    selected_tags: Iterable[str],
) -> None:
    """Write, generate, validate, and summarize each selected profile."""
    root = package_root.resolve()
    baseline = load_system_config(baseline_path)
    selected = set(selected_tags)
    unknown = selected.difference(tag for tag, _ in PROFILES)
    if unknown:
        raise ValueError(f'unknown spacing profile(s): {sorted(unknown)}')

    for tag, spacing in PROFILES:
        if tag not in selected:
            continue
        config = derive_profile(baseline, tag, spacing)
        config_path = root / 'config' / f'spacing_sweep_{tag}_system.yaml'
        _atomic_yaml(config_path, config)
        loaded = load_system_config(config_path)
        generated = generate_all_floor_assets(loaded, root)
        validated = validate_generated_assets(loaded, root)
        realized = min(
            _minimum_realized_spacing(asset.metadata_path)
            for asset in validated
        )
        summaries = ', '.join(
            f'{asset.floor_id}:{asset.obstacle_count}obs/'
            f'{asset.point_count}pts'
            for asset in generated
        )
        print(
            f'spacing_{tag}: requested={spacing:.2f}m, '
            f'realized_min={realized:.6f}m, {summaries}'
        )


def _parse_arguments(arguments: Sequence[str] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Generate four controlled M20 obstacle-spacing profiles.'
    )
    parser.add_argument(
        '--package-root',
        type=Path,
        default=PACKAGE_ROOT,
        help='m20_warehouse_inspection package root',
    )
    parser.add_argument(
        '--baseline',
        type=Path,
        default=PACKAGE_ROOT / 'config' / 'route_challenge_system.yaml',
        help='route challenge YAML used as the controlled baseline',
    )
    parser.add_argument(
        '--profile',
        action='append',
        choices=[tag for tag, _ in PROFILES],
        dest='profiles',
        help='generate one tag; repeat as needed (default: all)',
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] = None) -> int:
    options = _parse_arguments(arguments)
    selected = options.profiles or [tag for tag, _ in PROFILES]
    generate_profiles(
        options.package_root,
        options.baseline,
        selected,
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
