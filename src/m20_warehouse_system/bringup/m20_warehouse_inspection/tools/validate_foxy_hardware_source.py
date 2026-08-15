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
DEPLOYED_MESSAGE_SOURCE = SOURCE / 'deep-robotics-msg'
DEPLOYED_MESSAGE_PACKAGE = 'drdds'
DEPLOYED_MESSAGE_VERSION = '1.1.0'
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
        'MotionStateValue motion_state',
        'GaitValue gait_state',
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


def post_python38_runtime_calls():
    """Find runtime APIs unavailable in Ubuntu 20.04's Python 3.8."""
    findings = []
    unsupported = ('.removeprefix(', '.removesuffix(', '.is_relative_to(')
    for root in PYTHON_RUNTIME_ROOTS:
        for path in root.rglob('*.py'):
            for line_number, line in enumerate(
                path.read_text(encoding='utf-8').splitlines(), start=1
            ):
                code = line.split('#', 1)[0]
                if any(token in code for token in unsupported):
                    findings.append((path, line_number))
    return findings


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


def deployed_message_source_errors():
    """Validate interfaces used by this project in the deployed source."""
    manifest = DEPLOYED_MESSAGE_SOURCE / 'package.xml'
    baseline = DEPLOYED_MESSAGE_SOURCE / 'msg'
    if not manifest.is_file():
        return [
            'deployed message source is incomplete (package.xml missing): '
            '{0}'.format(DEPLOYED_MESSAGE_SOURCE)
        ]
    if not baseline.is_dir():
        return [
            'deployed message source has no msg directory: {0}'.format(
                DEPLOYED_MESSAGE_SOURCE
            )
        ]
    try:
        manifest_root = ET.parse(str(manifest)).getroot()
        package_name = manifest_root.findtext('name')
        package_version = manifest_root.findtext('version')
    except ET.ParseError as error:
        return [
            'invalid deployed message package.xml: {0}: {1}'.format(
                manifest, error
            )
        ]
    package_name = (package_name or '').strip()
    if package_name != DEPLOYED_MESSAGE_PACKAGE:
        return [
            'deep-robotics-msg declares ROS package {0!r}; the current direct '
            'backend imports drdds.msg, so package dependencies and imports '
            'must be adapted before hardware release'.format(package_name)
        ]
    package_version = (package_version or '').strip()
    if package_version != DEPLOYED_MESSAGE_VERSION:
        return [
            'deep-robotics-msg version is {0!r}; expected the synchronized '
            'M20-PRO ABI baseline {1!r}'.format(
                package_version, DEPLOYED_MESSAGE_VERSION
            )
        ]

    def normalized(path):
        return tuple(
            line
            for raw in path.read_text(encoding='utf-8').splitlines()
            if (line := ' '.join(raw.split('#', 1)[0].split()))
        )

    baseline_files = {path.name: path for path in baseline.glob('*.msg')}
    errors = []
    required_files = {'{0}.msg'.format(name) for name in DIRECT_SCHEMAS}
    missing = sorted(required_files - set(baseline_files))
    if missing:
        errors.append(
            'deployed deep-robotics-msg is missing required interfaces: '
            + ', '.join(missing)
        )
    for name, expected in DIRECT_SCHEMAS.items():
        path = baseline_files.get('{0}.msg'.format(name))
        if path is not None and normalized(path) != expected:
            errors.append(
                'deployed deep-robotics-msg ABI differs from the current '
                'direct backend contract: {0}'.format(name)
            )
    return errors


def deployed_direct_interface_files():
    """Return required message files from the synchronized hardware package."""
    root = DEPLOYED_MESSAGE_SOURCE / 'msg'
    return {
        name: root / '{0}.msg'.format(name)
        for name in DIRECT_SCHEMAS
        if (root / '{0}.msg'.format(name)).is_file()
    }


def validate_lio_profile():
    root = SOURCE / 'Elevator-LIO' / 'yaml'
    runbook_path = (
        SOURCE / 'Elevator-LIO' / 'Virdy-m20-pro-建图定位启动.md'
    )
    root_path = root / 'root_config_m20.yaml'
    relocation_root_path = (
        root / 'root_config_m20_navigation_relocation.yaml'
    )
    relocation_runtime_path = root / 'runtime' / 'relocation.yaml'
    sensor_path = root / 'sensors' / 'robosense_m20.yaml'
    missing_paths = [
        str(path)
        for path in (
            root_path,
            relocation_root_path,
            relocation_runtime_path,
            sensor_path,
            runbook_path,
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
        'sensors/robosense_m20.yaml',
        'world_frame_name: "lio_world"',
        'body_frame_name: "lio_base_link"',
        'lidar_frame_name: "lio_base_link"',
        '"/rslidar_points_front"',
        '"/rslidar_points_rear"',
        'imu_topic_name: "/IMU"',
        'runtime/relocation.yaml',
        'relocation_enable: true',
        'pcd_load_name:',
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


def foxy_launch_errors():
    """Reject static-TF syntax that ROS 2 Foxy cannot parse."""
    launch_root = (
        SYSTEM / 'bringup' / 'm20_warehouse_inspection' / 'launch'
    )
    errors = []
    for name in (
        'f1_scan_rviz.launch.py',
        'multifloor_scan_rviz.launch.py',
    ):
        path = launch_root / name
        if not path.is_file():
            errors.append('missing launch file: {0}'.format(path))
            continue
        text = path.read_text(encoding='utf-8')
        if "'--frame-id'" in text or "'--child-frame-id'" in text:
            errors.append(
                '{0}: named static_transform_publisher arguments are not '
                'supported by ROS 2 Foxy'.format(path)
            )
        if "'map', 'world'" not in text:
            errors.append(
                '{0}: missing positional map -> world static TF'.format(path)
            )
    return errors


def foxy_cpp_compatibility_errors():
    """Guard known ROS header renames between Foxy and Humble."""
    source_root = (
        SYSTEM / 'navigation' / 'm20_scan_planner' / 'src'
    )
    errors = []
    header_pairs = (
        (
            '<tf2_geometry_msgs/tf2_geometry_msgs.hpp>',
            '<tf2_geometry_msgs/tf2_geometry_msgs.h>',
        ),
        ('<tf2/utils.hpp>', '<tf2/utils.h>'),
    )
    for path in source_root.glob('*.cpp'):
        text = path.read_text(encoding='utf-8')
        for humble_header, foxy_header in header_pairs:
            if humble_header in text and foxy_header not in text:
                errors.append(
                    '{0}: Humble header {1} has no Foxy fallback {2}'.format(
                        path, humble_header, foxy_header
                    )
                )
    return errors


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

    for path, line in post_python38_runtime_calls():
        errors.append(
            'Python 3.8 runtime API incompatibility: {0}:{1}'.format(
                path, line
            )
        )

    missing_lio = validate_lio_profile()
    if missing_lio:
        errors.append(
            'Elevator-LIO navigation profile is incomplete: '
            + ', '.join(missing_lio)
        )

    errors.extend(foxy_launch_errors())
    errors.extend(foxy_cpp_compatibility_errors())

    found = direct_interface_files()
    deployed_found = deployed_direct_interface_files()
    missing_direct = sorted(set(DIRECT_SCHEMAS) - set(found))
    if arguments.transport == 'direct_ros' and missing_direct:
        errors.append(
            'direct_ros requires firmware-matched drdds messages: '
            + ', '.join(missing_direct)
        )
    if arguments.transport == 'direct_ros':
        errors.extend(direct_schema_errors(found))
        errors.extend(direct_schema_errors(deployed_found))
    errors.extend(deployed_message_source_errors())

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
            'drdds interface schema: MATCHES DEPLOYED deep-robotics-msg; '
            'direct_ros topics/QoS: MATCH RECORDED M20-PRO ENDPOINTS'
        )
    return 0


if __name__ == '__main__':
    sys.exit(main())
