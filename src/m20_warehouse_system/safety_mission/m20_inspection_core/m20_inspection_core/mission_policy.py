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

# ============================================================================
# 【文件职责】mission_policy.py —— 巡检任务配置解析（纯函数，无 ROS 依赖）
# 本文件是 m20_inspection_core 的"任务策略"模块：
#   1) 从 YAML 配置中把命名巡检点（inspection_points）与电梯/连廊引用
#      解析为可执行的步骤序列 MissionStep；
#   2) 支持 5 种步骤类型：inspection（巡检）/ transit（过路点）/
#      elevator_transfer 与 floor_transfer（楼层转移）/ terminal（终点）；
#   3) 提供纯函数 resolve_mission / display_step_index，便于单元测试
#      与确定性执行（不包含任何 ROS 运行时对象）。
# ============================================================================

"""Pure configuration resolver for deterministic inspection mission steps."""

# ------------------------- 标准库导入 -------------------------
from dataclasses import dataclass  # 数据类装饰器：用于定义不可变的值对象 MissionStep
import math                        # 数学库：校验位姿数值是否为有限值（isfinite）
from typing import Any, Mapping, Tuple  # 类型提示：Any 任意值 / Mapping 只读映射 / Tuple 元组


Pose2D = Tuple[float, float, float]  # 二维位姿类型别名：(x, y, yaw) 的元组


@dataclass(frozen=True)  # 冻结数据类：实例创建后不可修改，保证配置解析结果只读
class MissionStep:
    """Resolved mission step without ROS runtime objects."""
    # 【中文】已解析的任务步骤：纯数据对象，不包含 ROS 运行时对象

    step_type: str      # 步骤类型：inspection / transit / elevator_transfer / floor_transfer / terminal
    name: str           # 步骤显示名称（巡检点名 / 连廊:源->目标 / 终点名）
    floor_id: str       # 该步骤所属楼层
    pose: Pose2D        # 目标位姿（x, y, yaw），世界系
    dwell_sec: float    # 巡检停留时长（秒），非巡检步骤为 0
    connector_id: str = ''    # 楼层转移使用的连廊/电梯 id（默认空字符串）
    target_floor: str = ''    # 楼层转移的目标楼层（默认空字符串）


def _pose(values, field: str) -> Pose2D:
    # 【功能】把配置中的位姿值（列表/元组）解析并校验为 Pose2D
    # 【参数】values - 原始位姿值；field - 字段名（用于报错信息）
    # 【返回】校验通过的 (x, y, yaw) 元组
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f'{field} must contain x, y, yaw')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{field} contains a non-finite value')
    return result  # type: ignore[return-value]


def display_step_index(step_index: int, step_count: int) -> int:
    """Clamp an internal cursor to a valid zero-based display step."""
    # 【中文】将内部游标钳制为合法的零基显示步序号（防止越界显示）
    # 【参数】step_index - 内部步骤游标；step_count - 总步数
    # 【返回】合法的零基显示索引；步数为 0 时返回 0
    if step_count <= 0:
        return 0
    return max(0, min(int(step_index), int(step_count) - 1))


def resolve_mission(
    config: Mapping[str, Any], mission_id: str
) -> Tuple[MissionStep, ...]:
    """Resolve named points and elevator references into executable steps."""
    # 【中文】将配置中的命名点位与电梯引用解析为可执行步骤序列
    # 【参数】config - 仓库系统 YAML 配置（含 mission/floors 等）；mission_id - 要解析的任务 id
    # 【返回】MissionStep 元组（按配置顺序排列）；解析失败抛出 ValueError
    mission = config.get('mission', {})
    if mission_id != str(mission.get('id', '')):
        raise ValueError(f'unknown mission {mission_id!r}')
    floors = config.get('floors', {})
    steps = []
    for index, raw in enumerate(mission.get('sequence', [])):
        step_type = raw.get('type')
        if step_type in {'inspection', 'transit'}:
            # 巡检/过路步骤：按"楼层 + 命名点"解析目标位姿与停留时长
            floor_id = str(raw['floor'])
            point_name = str(raw['point'])
            points = {
                str(point['name']): point
                for point in floors[floor_id]['inspection_points']
            }
            if point_name not in points:
                raise ValueError(
                    f'mission step {index} has unknown point {point_name!r}'
                )
            point = points[point_name]
            step_name = (
                point_name
                if step_type == 'inspection'
                else str(raw['name']).strip()
            )
            steps.append(
                MissionStep(
                    step_type=str(step_type),
                    name=step_name,
                    floor_id=floor_id,
                    pose=_pose(point['pose'], f'{point_name}.pose'),
                    dwell_sec=(
                        max(0.0, float(point.get('dwell_sec', 0.0)))
                        if step_type == 'inspection'
                        else 0.0
                    ),
                )
            )
        elif step_type in {'elevator_transfer', 'floor_transfer'}:
            # 楼层转移步骤：解析源/目标楼层、连廊 id 与电梯厅等待位姿
            source = str(raw['from'])
            target = str(raw['to'])
            connector_key = (
                'elevator' if step_type == 'elevator_transfer' else 'connector'
            )
            connector = str(raw[connector_key])
            steps.append(
                MissionStep(
                    step_type=str(step_type),
                    name=f'{connector}:{source}->{target}',
                    floor_id=source,
                    pose=_pose(
                        floors[source]['elevator_lobby_pose'],
                        f'{source}.elevator_lobby_pose',
                    ),
                    dwell_sec=0.0,
                    connector_id=connector,
                    target_floor=target,
                )
            )
        elif step_type == 'terminal':
            # 终点步骤：按 location（电梯厅/初始位姿）选取对应的位姿键
            floor_id = str(raw['floor'])
            location = str(raw.get('location', ''))
            pose_key_by_location = {
                'elevator_lobby': 'elevator_lobby_pose',
                'initial_pose': 'initial_pose',
            }
            if location not in pose_key_by_location:
                raise ValueError(
                    f'mission step {index} has unsupported terminal '
                    f'location {location!r}'
                )
            pose_key = pose_key_by_location[location]
            name = str(raw.get('name', '')).strip()
            if not name:
                raise ValueError(
                    f'mission step {index} terminal name is empty'
                )
            steps.append(
                MissionStep(
                    step_type='terminal',
                    name=name,
                    floor_id=floor_id,
                    pose=_pose(
                        floors[floor_id][pose_key],
                        f'{floor_id}.{pose_key}',
                    ),
                    dwell_sec=0.0,
                )
            )
        else:
            raise ValueError(
                f'mission step {index} has unsupported type {step_type!r}'
            )
    if not steps:
        raise ValueError('mission has no executable steps')
    return tuple(steps)
