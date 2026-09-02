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

# ============================================================
# 文件：world_generator.py
# 用途：从仓库地图元数据生成 MuJoCo 碰撞世界（MJCF XML）。
#       - 读取系统配置 YAML（floors 列表），解析每层地图 JSON
#         （障碍物坐标、source_bounds 边界、transition_gateway 通道）；
#       - 以 m20_robot.xml 为模板，注入「地面 + 障碍物盒 + 边界墙」
#         几何体，并覆盖 compiler/option 物理参数；
#       - 输出采用「原子写」（临时文件 + os.replace），结果字节级
#         确定（同一输入产出完全相同的 XML），并生成 SHA256 摘要；
#       - 提供命令行入口用于离线生成与诊断。
# ============================================================

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

# 类型别名：
#   Bounds  = (min_x, max_x, min_y, max_y) 地图边界
#   Segment = (x1, y1, x2, y2) 线段（墙段）
Bounds = Tuple[float, float, float, float]
Segment = Tuple[float, float, float, float]


@dataclass(frozen=True)
class WorldBuildReport:
    """Deterministic summary of one generated physics world."""

    # 生成的 MJCF 文件绝对路径
    output_path: str
    # 输入（系统配置 + 元数据 + 机器人模板）的 SHA256 摘要
    profile_sha256: str
    # 楼层数 / 障碍物数 / 墙段数 / 执行器数
    floor_count: int
    obstacle_count: int
    wall_count: int
    actuator_count: int


def _number(value: float) -> str:
    """Format XML numbers deterministically without unnecessary precision."""
    # 用 %.9g 输出，避免浮点噪声导致两次生成结果字节不同。
    return f'{float(value):.9g}'


def _safe_name(value: str) -> str:
    """Convert a floor or object identifier into an MJCF-safe name fragment."""
    # 把楼层/对象标识符中的非字母数字字符替换为下划线，
    # 得到可用于 MJCF name 属性的安全片段（空则回退为 'unnamed'）。
    result = re.sub(r'[^A-Za-z0-9_]+', '_', value)
    return result.strip('_') or 'unnamed'


def _resolve_asset(root: Path, configured_path: str) -> Path:
    # 相对路径以 root 为基准解析并规范化（支持 ~ 展开）。
    candidate = Path(configured_path).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    return candidate.resolve()


def _read_json(path: Path) -> Dict:
    # 以 UTF-8 读取地图元数据 JSON。
    with path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def _wall_segments(
    bounds: Bounds,
    gateway: Dict,
) -> Iterable[Segment]:
    """Return four boundary edges, splitting the configured gateway edge."""
    # 生成四段边界墙；若配置了 transition_gateway（通道口），
    # 则在对应边缘（min_x 或 max_x）上按 center_y/width 留出缺口，
    # 使相邻楼层之间可以通行。
    min_x, max_x, min_y, max_y = bounds
    edge = str(gateway.get('edge', ''))
    center_y = float(gateway.get('center_y', 0.0))
    width = max(0.0, float(gateway.get('width', 0.0)))
    # 通道口在 y 方向的区间 [lower, upper]，被夹在边界内。
    lower = max(min_y, center_y - width * 0.5)
    upper = min(max_y, center_y + width * 0.5)

    # 南边（y=min_y）与北边（y=max_y）的完整墙段。
    yield (min_x, min_y, max_x, min_y)
    yield (min_x, max_y, max_x, max_y)

    # 西边（x=min_x）与东边（x=max_x）：
    # 若该边是通道边且有有效宽度，则拆成两段（缺口上下各一段），
    # 否则输出完整墙段。
    for side, x in (('min_x', min_x), ('max_x', max_x)):
        if edge != side or upper <= lower:
            yield (x, min_y, x, max_y)
            continue
        if lower > min_y:
            yield (x, min_y, x, lower)
        if upper < max_y:
            yield (x, upper, x, max_y)


def _canonical_segment(segment: Segment) -> Segment:
    # 将墙段规范化为「端点按坐标排序」的表示，用于跨楼层去重
    # （同一段墙被多个楼层共享时只生成一次）。
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
    # 向 worldbody 追加一个固定盒体 geom（type=box）。
    # 注意 MuJoCo 的 size 为半尺寸：pos.z 取 height/2，
    # size 三个分量分别为 size_x/2、size_y/2、height/2。
    # 接触参数：contype/conaffinity=1（与机器人碰撞）、condim=3、
    #           低摩擦（friction="1 0.01 0.001"）、soft 求解器 solref。
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
    # 对「系统配置 + 所有地图元数据 + 机器人模板」的字节内容
    # 计算 SHA256，用于追踪世界来源（profile_sha256 字段）。
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
    # 统一把输入路径展开为绝对路径，避免相对路径歧义。
    system_config = system_config.expanduser().resolve()
    package_root = package_root.expanduser().resolve()
    robot_template = robot_template.expanduser().resolve()
    mesh_directory = mesh_directory.expanduser().resolve()
    output_path = output_path.expanduser().resolve()

    # 读取系统配置 YAML，校验至少有一个楼层。
    with system_config.open('r', encoding='utf-8') as stream:
        system = yaml.safe_load(stream)
    floors = system.get('floors', {})
    if not floors:
        raise ValueError('system configuration contains no floors')

    # 逐楼层解析元数据文件路径（相对 package_root），并读取 JSON。
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

    # 解析机器人模板（m20_robot.xml），并强制覆盖 compiler 属性：
    # meshdir 指向网格目录、角度用弧度、开启自动限位。
    tree = ET.parse(robot_template)
    root = tree.getroot()
    compiler = root.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(root, 'compiler')
    compiler.set('meshdir', str(mesh_directory))
    compiler.set('angle', 'radian')
    compiler.set('autolimits', 'true')

    # 强制覆盖 option 物理参数：1ms 步长、重力、implicitfast 积分器、50 次迭代。
    option = root.find('option')
    if option is None:
        option = ET.Element('option')
        root.insert(0, option)
    option.set('timestep', '0.001')
    option.set('gravity', '0 0 -9.81')
    option.set('integrator', 'implicitfast')
    option.set('iterations', '50')

    # 处理 worldbody：把地面 geom 重命名为 warehouse_ground（大平面）。
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

    # 新生成的几何体统一插到 base_link 之前（insert_at 位置），
    # 保持 MJCF 中 world 几何在前、机器人 body 在后的顺序。
    robot_body = worldbody.find("./body[@name='base_link']")
    insert_at = (
        list(worldbody).index(robot_body)
        if robot_body is not None
        else len(worldbody)
    )
    generated: List[ET.Element] = []
    obstacle_count = 0
    # unique_walls：规范墙段 → (原始墙段, 高度)，用于跨楼层去重。
    unique_walls: Dict[Segment, Tuple[Segment, float]] = {}

    # ---------- 逐楼层生成障碍物盒 ----------
    for floor_id, floor_config, metadata in floor_metadata:
        # 楼层坐标系偏移（simulation_offset），把地图坐标平移到仿真坐标。
        convention = metadata.get('coordinate_convention', {})
        offset = convention.get(
            'simulation_offset',
            floor_config.get('simulation_offset', [0.0, 0.0, 0.0]),
        )
        offset_x = float(offset[0])
        offset_y = float(offset[1])
        # 障碍物高度：优先取元数据 obstacle_height，否则取系统默认 2.0m。
        obstacle_height = float(
            metadata.get(
                'obstacle_height',
                system.get('map_generation', {}).get(
                    'obstacle_height',
                    2.0,
                ),
            )
        )
        # 每个障碍物（x_min/x_max/y_min/y_max 矩形）→ 一个碰撞盒。
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

        # 由 source_bounds 生成四周边界墙（含通道缺口），
        # 规范化后去重（多楼层共享边界只建一次）。
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

    # ---------- 生成边界墙 ----------
    for index, (segment, height) in enumerate(unique_walls.values()):
        x1, y1, x2, y2 = segment
        # 墙段长度在某方向为 0 时，用 wall_thickness 补成细长盒。
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

    # 把生成的几何体按顺序插入 worldbody（在机器人之前）。
    for element in generated:
        worldbody.insert(insert_at, element)
        insert_at += 1

    # ---------- 原子写出 ----------
    # 先写临时文件再 os.replace，避免生成到一半崩溃留下损坏的世界文件。
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

    # ---------- 校验 ----------
    # 模板必须恰好有 16 个电机执行器（12 腿 + 4 轮）。
    actuator_count = len(root.findall('./actuator/motor'))
    if actuator_count != 16:
        raise ValueError(
            f'M20 template must contain 16 actuators, got {actuator_count}'
        )
    # 可选：用 MuJoCo 实际加载生成的文件，确认模型可解析且执行器数为 16。
    if validate_model:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(output_path))
        if model.nu != 16:
            raise ValueError(
                f'generated MuJoCo model must expose 16 actuators, got {model.nu}'
            )

    # 汇总构建报告（路径、SHA256、各数量），供 launch/CLI/测试使用。
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
    # 以系统配置路径的 SHA256 前 12 位为世界文件名，
    # 输出到 /tmp/m20_nav2_backend 下，避免污染源码树。
    digest = hashlib.sha256(
        str(system_config.expanduser().resolve()).encode('utf-8')
    ).hexdigest()[:12]
    return Path('/tmp/m20_nav2_backend') / f'world_{digest}.xml'


def main(args=None) -> None:
    """Command-line entry point used for diagnostics and offline generation."""
    # CLI 入口：--system-config / --robot-template / --mesh-directory 必填，
    # --package-root 与 --output 可选（默认用缓存路径），
    # --skip-validation 跳过 MuJoCo 加载校验。
    parser = argparse.ArgumentParser()
    parser.add_argument('--system-config', required=True, type=Path)
    parser.add_argument('--package-root', type=Path)
    parser.add_argument('--robot-template', required=True, type=Path)
    parser.add_argument('--mesh-directory', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--skip-validation', action='store_true')
    parsed = parser.parse_args(args)
    # package_root 未指定时，推断为 system_config 的上上级目录。
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
    # 以 JSON 形式打印构建报告（保留中文不转义）。
    print(json.dumps(asdict(report), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
