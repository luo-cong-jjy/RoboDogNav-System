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

"""Command-line entry points for validation and deterministic generation."""

import argparse
from pathlib import Path
from typing import Optional, Sequence, Tuple

from .configuration import ConfigurationError, load_system_config
from .map_assets import (
    generate_all_floor_assets,
    validate_generated_assets,
)


def _default_paths() -> Tuple[Path, Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('m20_warehouse_inspection'))
    except (ImportError, LookupError):
        # Keep map generation usable before the first ROS/colcon build.
        share = Path(__file__).resolve().parents[1]
    return share / 'config' / 'flat_multifloor_system.yaml', share


def _common_parser(description: str) -> argparse.ArgumentParser:
    default_config, default_root = _default_paths()
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        '--config',
        type=Path,
        default=default_config,
        help='system YAML configuration',
    )
    parser.add_argument(
        '--package-root',
        type=Path,
        default=default_root,
        help='root used to resolve maps/... asset paths',
    )
    return parser


def main_validate(arguments: Optional[Sequence[str]] = None) -> int:
    """Validate configuration and, optionally, its generated map assets."""
    parser = _common_parser('Validate M20 warehouse system configuration')
    parser.add_argument(
        '--require-assets',
        action='store_true',
        help='also validate all configured PCD, PGM, YAML, and JSON files',
    )
    options = parser.parse_args(arguments)
    try:
        config = load_system_config(options.config)
        print(
            f'configuration valid: {len(config["floors"])} floors, '
            f'layout={config["simulation"]["layout_mode"]}'
        )
        if options.require_assets:
            assets = validate_generated_assets(config, options.package_root)
            for asset in assets:
                print(
                    f'{asset.floor_id}: {asset.point_count} points, '
                    f'{asset.obstacle_count} obstacles, '
                    f'sha256={asset.pcd_sha256}'
                )
    except (ConfigurationError, FileNotFoundError, KeyError, ValueError) as error:
        parser.error(str(error))
    return 0


def main_generate(arguments: Optional[Sequence[str]] = None) -> int:
    """Regenerate all map assets and verify the resulting deterministic set."""
    parser = _common_parser('Generate deterministic M20 warehouse maps')
    parser.add_argument(
        '--floor',
        action='append',
        dest='floors',
        help='generate only this floor ID; may be repeated',
    )
    options = parser.parse_args(arguments)
    try:
        config = load_system_config(options.config)
        generated = generate_all_floor_assets(
            config, options.package_root, options.floors
        )
        for asset in generated:
            print(
                f'generated {asset.floor_id}: {asset.point_count} points, '
                f'{asset.obstacle_count} obstacles, {asset.pcd_path}'
            )
        if options.floors is None:
            validate_generated_assets(config, options.package_root)
            print('generated asset validation: PASS')
    except (
        ConfigurationError,
        FileNotFoundError,
        KeyError,
        RuntimeError,
        ValueError,
    ) as error:
        parser.error(str(error))
    return 0
