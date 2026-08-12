#!/usr/bin/env python3

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

"""Static preflight for the Ubuntu 20.04 / ROS 2 Foxy source closure."""

import argparse
import ast
from collections import defaultdict
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[5]
SOURCE = WORKSPACE / 'src'
SYSTEM = SOURCE / 'm20_warehouse_system'
BACKPACK_DRDDS = SOURCE / 'drdds-背部主机当前版'
PYTHON_RUNTIME_ROOTS = (
    SYSTEM
    / 'safety_mission'
    / 'm20_inspection_core'
    / 'm20_inspection_core',
    SYSTEM
    / 'motion'
    / 'm20_locomotion_control'
    / 'm20_locomotion_control',
    SYSTEM
    / 'navigation'
    / 'm20_multifloor_map'
    / 'm20_multifloor_map',
    SYSTEM
    / 'navigation'
    / 'm20_scan_navigation'
    / 'm20_scan_navigation',
    SYSTEM
    / 'bringup'
    / 'm20_warehouse_inspection'
    / 'm20_warehouse_inspection',
)
DIRECT_SCHEMAS = {
    'MetaType': ('uint64 frame_id', 'builtin_interfaces/Time stamp'),
    'MotionStateValue': ('int32 state',),
    'MotionState': ('MetaType header', 'MotionStateValue data'),
    'GaitValue': ('uint32 gait',),
    'Gait': ('MetaType header', 'GaitValue data'),
    'NavCmdValue': (
        'float32 x_vel', 'float32 y_vel', 'float32 yaw_vel',
    ),
    'NavCmd': ('MetaType header', 'NavCmdValue data'),
    'MotionInfoValue': (
        'float32 vel_x',
        'float32 vel_y',
        'float32 vel_yaw',
        'float32 height',
        'int32 state',
        'uint32 gait',
        'float32 payload',
        'float32 remain_mile',
    ),
    'MotionInfo': ('MetaType header', 'MotionInfoValue data'),
    'StdMsgInt32': ('int32 value',),
}


def _is_colcon_ignored(path: Path) -> bool:
    current = path.parent
    while current != SOURCE.parent and current != current.parent:
        if (current / 'COLCON_IGNORE').exists():
            return True
        if current == SOURCE:
            break
        current = current.parent
    return False


def active_package_names():
    """Return active colcon package paths grouped by declared name."""
    grouped = defaultdict(list)
    for manifest in SOURCE.rglob('package.xml'):
        if _is_colcon_ignored(manifest):
            continue
        try:
            name = ET.parse(str(manifest)).getroot().findtext('name')
        except ET.ParseError:
            continue
        if name:
            grouped[name.strip()].append(manifest.parent)
    return grouped


def risky_python_annotations():
    """Find evaluated annotations that are not safe on Python 3.8."""
    findings = []
    for root in PYTHON_RUNTIME_ROOTS:
        for path in root.rglob('*.py'):
            source = path.read_text(encoding='utf-8')
            if 'from __future__ import annotations' in source:
                continue
            tree = ast.parse(source, filename=str(path))
            for node in ast.walk(tree):
                annotation = None
                if isinstance(node, ast.arg):
                    annotation = node.annotation
                elif isinstance(node, ast.AnnAssign):
                    annotation = node.annotation
                elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    annotation = node.returns
                if annotation is None:
                    continue
                for child in ast.walk(annotation):
                    if (
                        isinstance(child, ast.Subscript)
                        and isinstance(child.value, ast.Name)
                        and child.value.id
                        in {
                            'dict', 'frozenset', 'list',
                            'set', 'tuple', 'type',
                        }
                    ):
                        findings.append((path, child.lineno))
                    if (
                        isinstance(child, ast.BinOp)
                        and isinstance(child.op, ast.BitOr)
                    ):
                        findings.append((path, child.lineno))
    return sorted(set(findings))


def direct_interface_files():
    """Return high-level drdds interface files found in active packages."""
    found = {}
    for name in DIRECT_SCHEMAS:
        for path in SOURCE.rglob('msg/{0}.msg'.format(name)):
            if not _is_colcon_ignored(path):
                found[name] = path
                break
    return found


def direct_schema_errors(found):
    """Validate safety-critical definitions against the deployed baseline."""
    errors = []
    for name, expected in DIRECT_SCHEMAS.items():
        path = found.get(name)
        if path is None:
            continue
        actual = []
        for raw in path.read_text(encoding='utf-8').splitlines():
            line = raw.split('#', 1)[0].strip()
            if line:
                actual.append(line)
        if tuple(actual) != expected:
            errors.append(
                '{0}: expected [{1}], found [{2}]'.format(
                    path,
                    '; '.join(expected),
                    '; '.join(actual),
                )
            )
    return errors


def backpack_drdds_errors():
    """Compare every active message ABI with the latest backpack package."""
    active = SOURCE / 'drdds' / 'msg'
    baseline = BACKPACK_DRDDS / 'msg'
    if not baseline.is_dir():
        return [
            'latest backpack drdds baseline is missing: {0}'.format(
                BACKPACK_DRDDS
            )
        ]

    def normalized(path):
        return tuple(
            line
            for raw in path.read_text(encoding='utf-8').splitlines()
            if (line := ' '.join(raw.split('#', 1)[0].split()))
        )

    active_files = {path.name: path for path in active.glob('*.msg')}
    baseline_files = {path.name: path for path in baseline.glob('*.msg')}
    errors = []
    if set(active_files) != set(baseline_files):
        errors.append(
            'drdds message set differs from backpack baseline: '
            'active_only={0}, baseline_only={1}'.format(
                sorted(set(active_files) - set(baseline_files)),
                sorted(set(baseline_files) - set(active_files)),
            )
        )
    for name in sorted(set(active_files) & set(baseline_files)):
        if normalized(active_files[name]) != normalized(baseline_files[name]):
            errors.append(
                'drdds ABI differs from backpack baseline: {0}'.format(name)
            )
    return errors


def validate_lio_profile():
    root = SOURCE / 'Elevator-LIO' / 'yaml'
    root_path = root / 'root_config_m20_navigation.yaml'
    relocation_root_path = (
        root / 'root_config_m20_navigation_relocation.yaml'
    )
    relocation_runtime_path = root / 'runtime' / 'relocation.yaml'
    sensor_path = root / 'sensors' / 'robosense_m20_navigation.yaml'
    missing_paths = [
        str(path)
        for path in (
            root_path,
            relocation_root_path,
            relocation_runtime_path,
            sensor_path,
        )
        if not path.is_file()
    ]
    if missing_paths:
        return ['missing file: ' + path for path in missing_paths]
    root_text = root_path.read_text(encoding='utf-8')
    relocation_root_text = relocation_root_path.read_text(encoding='utf-8')
    relocation_runtime_text = relocation_runtime_path.read_text(
        encoding='utf-8'
    )
    sensor_text = sensor_path.read_text(encoding='utf-8')
    required = (
        'sensors/robosense_m20_navigation.yaml',
        'world_frame_name: "world"',
        'body_frame_name: "base_link"',
        'lidar_frame_name: "base_link"',
        '"/rslidar_points_front"',
        '"/rslidar_points_rear"',
        'imu_topic_name: "/IMU"',
        'runtime/relocation.yaml',
        'relocation_enable: true',
        'pcd_load_name: "m20_pao_f1_scans.pcd"',
    )
    joined = '\n'.join(
        (
            root_text,
            relocation_root_text,
            relocation_runtime_text,
            sensor_text,
        )
    )
    return [item for item in required if item not in joined]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--transport',
        choices=('basic_server', 'direct_ros'),
        default='basic_server',
    )
    arguments = parser.parse_args(argv)
    errors = []

    duplicates = {
        name: paths
        for name, paths in active_package_names().items()
        if len(paths) > 1
    }
    for name, paths in sorted(duplicates.items()):
        errors.append(
            'duplicate active package {0}: {1}'.format(
                name, ', '.join(str(path) for path in paths)
            )
        )

    for path, line in risky_python_annotations():
        errors.append(
            'Python 3.8 evaluates an unsupported annotation: {0}:{1}'.format(
                path, line
            )
        )

    missing_lio = validate_lio_profile()
    if missing_lio:
        errors.append(
            'Elevator-LIO navigation profile is incomplete: '
            + ', '.join(missing_lio)
        )

    found = direct_interface_files()
    missing_direct = sorted(set(DIRECT_SCHEMAS) - set(found))
    if arguments.transport == 'direct_ros' and missing_direct:
        errors.append(
            'direct_ros requires firmware-matched drdds messages: '
            + ', '.join(missing_direct)
        )
    if arguments.transport == 'direct_ros':
        errors.extend(direct_schema_errors(found))
    errors.extend(backpack_drdds_errors())

    if errors:
        for error in errors:
            print('ERROR: ' + error)
        return 1

    print('Foxy source preflight: PASS')
    print('selected factory transport: ' + arguments.transport)
    if missing_direct:
        print(
            'direct_ros availability: BLOCKED (missing: {0})'.format(
                ', '.join(missing_direct)
            )
        )
    else:
        print(
            'drdds interface schema: MATCHES BACKPACK BASELINE; '
            'direct_ros QoS: TARGET TEST PENDING'
        )
    return 0


if __name__ == '__main__':
    sys.exit(main())
