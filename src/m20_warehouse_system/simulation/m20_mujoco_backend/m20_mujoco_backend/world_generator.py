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

"""Generate a MuJoCo collision world from the warehouse map metadata."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Dict, Iterable, List, Sequence, Tuple
import xml.etree.ElementTree as ET

import yaml


Bounds = Tuple[float, float, float, float]
Segment = Tuple[float, float, float, float]


@dataclass(frozen=True)
class WorldBuildReport:
    """Deterministic summary of one generated physics world."""

    output_path: str
    profile_sha256: str
    floor_count: int
    obstacle_count: int
    wall_count: int
    actuator_count: int


def _number(value: float) -> str:
    """Format XML numbers deterministically without unnecessary precision."""
    return f'{float(value):.9g}'


def _safe_name(value: str) -> str:
    """Convert a floor or object identifier into an MJCF-safe name fragment."""
    result = re.sub(r'[^A-Za-z0-9_]+', '_', value)
    return result.strip('_') or 'unnamed'


def _resolve_asset(root: Path, configured_path: str) -> Path:
    candidate = Path(configured_path).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    return candidate.resolve()


def _read_json(path: Path) -> Dict:
    with path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def _wall_segments(
    bounds: Bounds,
    gateway: Dict,
) -> Iterable[Segment]:
    """Return four boundary edges, splitting the configured gateway edge."""
    min_x, max_x, min_y, max_y = bounds
    edge = str(gateway.get('edge', ''))
    center_y = float(gateway.get('center_y', 0.0))
    width = max(0.0, float(gateway.get('width', 0.0)))
    lower = max(min_y, center_y - width * 0.5)
    upper = min(max_y, center_y + width * 0.5)

    yield (min_x, min_y, max_x, min_y)
    yield (min_x, max_y, max_x, max_y)

    for side, x in (('min_x', min_x), ('max_x', max_x)):
        if edge != side or upper <= lower:
            yield (x, min_y, x, max_y)
            continue
        if lower > min_y:
            yield (x, min_y, x, lower)
        if upper < max_y:
            yield (x, upper, x, max_y)


def _canonical_segment(segment: Segment) -> Segment:
    x1, y1, x2, y2 = segment
    first = (round(x1, 7), round(y1, 7))
    second = (round(x2, 7), round(y2, 7))
    if second < first:
        first, second = second, first
    return first[0], first[1], second[0], second[1]


def _add_box(
    worldbody: ET.Element,
    *,
    name: str,
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
    height: float,
    rgba: str,
) -> None:
    """Add one fixed collision box using MuJoCo half-size convention."""
    ET.SubElement(
        worldbody,
        'geom',
        {
            'name': name,
            'type': 'box',
            'pos': (
                f'{_number(center_x)} {_number(center_y)} '
                f'{_number(height * 0.5)}'
            ),
            'size': (
                f'{_number(size_x * 0.5)} {_number(size_y * 0.5)} '
                f'{_number(height * 0.5)}'
            ),
            'rgba': rgba,
            'contype': '1',
            'conaffinity': '1',
            'condim': '3',
            'friction': '1 0.01 0.001',
            'solref': '0.005 1',
            'group': '0',
        },
    )


def _profile_digest(
    system_config: Path,
    metadata_paths: Sequence[Path],
    robot_template: Path,
) -> str:
    digest = hashlib.sha256()
    for path in [system_config, robot_template, *metadata_paths]:
        digest.update(path.read_bytes())
    return digest.hexdigest()


def generate_world(
    *,
    system_config: Path,
    package_root: Path,
    robot_template: Path,
    mesh_directory: Path,
    output_path: Path,
    wall_thickness: float = 0.10,
    validate_model: bool = True,
) -> WorldBuildReport:
    """Generate an atomic, deterministic MJCF world from map JSON metadata."""
    system_config = system_config.expanduser().resolve()
    package_root = package_root.expanduser().resolve()
    robot_template = robot_template.expanduser().resolve()
    mesh_directory = mesh_directory.expanduser().resolve()
    output_path = output_path.expanduser().resolve()

    with system_config.open('r', encoding='utf-8') as stream:
        system = yaml.safe_load(stream)
    floors = system.get('floors', {})
    if not floors:
        raise ValueError('system configuration contains no floors')

    metadata_paths: List[Path] = []
    floor_metadata: List[Tuple[str, Dict, Dict]] = []
    for floor_id, floor_config in floors.items():
        metadata_path = _resolve_asset(
            package_root,
            str(floor_config['metadata_file']),
        )
        metadata_paths.append(metadata_path)
        floor_metadata.append(
            (str(floor_id), floor_config, _read_json(metadata_path))
        )

    tree = ET.parse(robot_template)
    root = tree.getroot()
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    compiler.set('meshdir', str(mesh_directory))
    compiler.set('angle', 'radian')
    compiler.set('autolimits', 'true')

    option = root.find('option')
    if option is None:
        option = ET.Element('option')
        root.insert(0, option)
    option.set('timestep', '0.001')
    option.set('gravity', '0 0 -9.81')
    option.set('integrator', 'implicitfast')
    option.set('iterations', '50')

    worldbody = root.find('worldbody')
    if worldbody is None:
        raise ValueError('robot template contains no worldbody')
    floor_geom = worldbody.find("./geom[@name='floor']")
    if floor_geom is None:
        floor_geom = ET.SubElement(worldbody, 'geom')
    floor_geom.attrib.update(
        {
            'name': 'warehouse_ground',
            'type': 'plane',
            'pos': '0 0 0',
            'size': '100 100 0.125',
            'condim': '3',
            'friction': '1 0.01 0.001',
            'group': '0',
        }
    )

    robot_body = worldbody.find("./body[@name='base_link']")
    insert_at = (
        list(worldbody).index(robot_body)
        if robot_body is not None
        else len(worldbody)
    )
    generated: List[ET.Element] = []
    obstacle_count = 0
    unique_walls: Dict[Segment, Tuple[Segment, float]] = {}

    for floor_id, floor_config, metadata in floor_metadata:
        convention = metadata.get('coordinate_convention', {})
        offset = convention.get(
            'simulation_offset',
            floor_config.get('simulation_offset', [0.0, 0.0, 0.0]),
        )
        offset_x = float(offset[0])
        offset_y = float(offset[1])
        obstacle_height = float(
            metadata.get(
                'obstacle_height',
                system.get('map_generation', {}).get(
                    'obstacle_height',
                    2.0,
                ),
            )
        )
        for index, obstacle in enumerate(metadata.get('obstacles', [])):
            min_x = float(obstacle['x_min']) + offset_x
            max_x = float(obstacle['x_max']) + offset_x
            min_y = float(obstacle['y_min']) + offset_y
            max_y = float(obstacle['y_max']) + offset_y
            holder = ET.Element('holder')
            _add_box(
                holder,
                name=(
                    f'warehouse_obstacle_{_safe_name(floor_id)}_{index:04d}'
                ),
                center_x=(min_x + max_x) * 0.5,
                center_y=(min_y + max_y) * 0.5,
                size_x=max_x - min_x,
                size_y=max_y - min_y,
                height=obstacle_height,
                rgba='0.35 0.35 0.38 1',
            )
            generated.extend(list(holder))
            obstacle_count += 1

        source_bounds = metadata['source_bounds']
        bounds = (
            float(source_bounds['x'][0]) + offset_x,
            float(source_bounds['x'][1]) + offset_x,
            float(source_bounds['y'][0]) + offset_y,
            float(source_bounds['y'][1]) + offset_y,
        )
        gateway = metadata.get(
            'transition_gateway',
            floor_config.get('transition_gateway', {}),
        )
        for segment in _wall_segments(bounds, gateway):
            canonical = _canonical_segment(segment)
            unique_walls.setdefault(
                canonical,
                (segment, obstacle_height),
            )

    for index, (segment, height) in enumerate(unique_walls.values()):
        x1, y1, x2, y2 = segment
        size_x = abs(x2 - x1)
        size_y = abs(y2 - y1)
        if size_x < 1.0e-7:
            size_x = wall_thickness
        if size_y < 1.0e-7:
            size_y = wall_thickness
        holder = ET.Element('holder')
        _add_box(
            holder,
            name=f'warehouse_wall_{index:03d}',
            center_x=(x1 + x2) * 0.5,
            center_y=(y1 + y2) * 0.5,
            size_x=size_x,
            size_y=size_y,
            height=height,
            rgba='0.15 0.25 0.45 1',
        )
        generated.extend(list(holder))

    for element in generated:
        worldbody.insert(insert_at, element)
        insert_at += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='wb',
        dir=output_path.parent,
        prefix=f'.{output_path.name}.',
        delete=False,
    ) as stream:
        temporary_path = Path(stream.name)
        tree.write(stream, encoding='utf-8', xml_declaration=True)
    os.replace(temporary_path, output_path)

    actuator_count = len(root.findall('./actuator/motor'))
    if actuator_count != 16:
        raise ValueError(
            f'M20 template must contain 16 actuators, got {actuator_count}'
        )
    if validate_model:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(output_path))
        if model.nu != 16:
            raise ValueError(
                f'generated MuJoCo model must expose 16 actuators, got {model.nu}'
            )

    return WorldBuildReport(
        output_path=str(output_path),
        profile_sha256=_profile_digest(
            system_config,
            metadata_paths,
            robot_template,
        ),
        floor_count=len(floor_metadata),
        obstacle_count=obstacle_count,
        wall_count=len(unique_walls),
        actuator_count=actuator_count,
    )


def cached_world_path(system_config: Path) -> Path:
    """Return a stable runtime output path without changing the source tree."""
    digest = hashlib.sha256(
        str(system_config.expanduser().resolve()).encode('utf-8')
    ).hexdigest()[:12]
    return Path('/tmp/m20_mujoco_backend') / f'world_{digest}.xml'


def main(args=None) -> None:
    """Command-line entry point used for diagnostics and offline generation."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--system-config', required=True, type=Path)
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--robot-template', required=True, type=Path)
    parser.add_argument('--mesh-directory', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--skip-validation', action='store_true')
    parsed = parser.parse_args(args)
    package_root = (
        parsed.package_root
        if parsed.package_root
        else parsed.system_config.expanduser().resolve().parent.parent
    )
    output = parsed.output or cached_world_path(parsed.system_config)
    report = generate_world(
        system_config=parsed.system_config,
        package_root=package_root,
        robot_template=parsed.robot_template,
        mesh_directory=parsed.mesh_directory,
        output_path=output,
        validate_model=not parsed.skip_validation,
    )
    print(json.dumps(asdict(report), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
