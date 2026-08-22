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
# 【文件职责】floor_switch_policy.py —— 楼层切换策略解析（纯函数，无 ROS 依赖）
# 本文件是 m20_inspection_core 的"楼层切换"纯逻辑模块，供
# floor_switch_manager_node.py 与 mission_executor_node.py 调用，主要包括：
#   1) 从 YAML 配置解析电梯/连廊（elevators）的显式路线（transitions）
#      或兼容旧格式的双向连廊，得到可执行的 TransitionPlan；
#   2) 合并系统级 / 连廊级 / 路线级的传输策略（transfer_adapter、
#      pose_handoff、延迟与超时等），支持新旧配置键的兼容回退；
#   3) 提供位姿误差计算 pose_error（平面距离 + 角度差归一化）。
# ============================================================================

"""Resolve floor routes and replaceable transport policies from YAML."""

# ------------------------- 标准库导入 -------------------------
from dataclasses import dataclass  # 数据类装饰器：用于定义不可变的路由计划值对象
import math                        # 数学库：距离/角度计算与有限值校验
from typing import Any, Mapping, Sequence, Tuple  # 类型提示：Any/Mapping/Sequence/Tuple


Pose2D = Tuple[float, float, float]  # 二维位姿类型别名：(x, y, yaw)


@dataclass(frozen=True)  # 冻结数据类：路由计划创建后不可变
class TransitionPlan:
    """Resolved directional route plus its transport/handoff strategy."""
    # 【中文】已解析的定向楼层转移路线及其传输/位姿交接策略

    connector_id: str         # 连廊/电梯 id
    source_floor: str         # 源楼层
    target_floor: str         # 目标楼层
    source_trigger: Pose2D    # 源侧触发位姿（机器人需到达该位姿才允许开始转移）
    target_release: Pose2D    # 目标侧释放位姿（转移完成后机器人应处于该位姿）
    trigger_tolerance_xy: float   # 触发位姿的平面位置容差（米）
    trigger_tolerance_yaw: float  # 触发位姿的航向角容差（弧度）
    transfer_adapter: str         # 传输适配器：timed_hold（定时保持）/ external_action（外部动作）
    pose_handoff: str             # 位姿交接策略：preserve / set_simulation_pose / wait_for_target / none
    transition_delay_sec: float   # 定时保持模式下的保持时长（秒）
    transfer_timeout_sec: float   # 传输过程超时（秒）
    external_action_name: str     # 外部传输 Action 的名称（external_action 模式使用）


TRANSFER_ADAPTERS = frozenset({'timed_hold', 'external_action'})  # 支持的传输适配器集合
POSE_HANDOFFS = frozenset(  # 支持的位姿交接策略集合
    {'preserve', 'set_simulation_pose', 'wait_for_target', 'none'}
)


def _pose3(values: Sequence[float], field: str) -> Pose2D:
    # 【功能】将位姿值序列解析并校验为 Pose2D（必须恰好 3 个有限数值）
    # 【参数】values - 原始位姿序列；field - 字段名（用于报错信息）
    # 【返回】校验通过的 (x, y, yaw) 元组
    if len(values) != 3:
        raise ValueError(f'{field} must contain x, y, yaw')
    pose = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in pose):
        raise ValueError(f'{field} contains a non-finite value')
    return pose  # type: ignore[return-value]


def _finite_number(value: Any, field: str) -> float:
    # 【功能】把任意值转为有限的 float 数值（非数值/非有限值报错）
    # 【参数】value - 原始值；field - 字段名（用于报错信息）
    # 【返回】校验通过的有限浮点数
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field} must be a number') from error
    if not math.isfinite(result):
        raise ValueError(f'{field} contains a non-finite value')
    return result


def _transfer_policy(
    config: Mapping[str, Any],
    connector: Mapping[str, Any],
    route: Mapping[str, Any],
) -> Tuple[str, str, float, float, str]:
    """
    Merge system, connector, and route transport settings.

    The legacy simulation keys remain a compatibility fallback for old test
    profiles. New profiles should use ``floor_switch`` and optional
    ``transfer`` overrides on a connector or an individual route.
    """
    # 【中文】合并系统级 / 连廊级 / 路线级的传输设置（从高到低覆盖）。
    # 旧版 simulation 键保留为兼容回退；新配置应使用 floor_switch 以及
    # 连廊或单条路线上的可选 transfer 覆盖。
    # 【参数】config - 系统配置；connector - 连廊配置；route - 路线配置（可能为空映射）
    # 【返回】(adapter, handoff, delay, timeout, action_name) 五元组
    simulation = config.get('simulation', {})
    transaction = config.get('map_switch_transaction', {})
    settings = {
        'adapter': 'timed_hold',
        'pose_handoff': (
            'set_simulation_pose'
            if bool(simulation.get('teleport_on_floor_switch', False))
            else 'preserve'
        ),
        'transition_delay_sec': simulation.get(
            'elevator_transition_delay_sec', 0.0
        ),
        'timeout_sec': transaction.get('relocate_timeout_sec', 30.0),
        'external_action_name': '/m20/floor_transfer/execute',
    }
    system_policy = config.get('floor_switch', {})
    if isinstance(system_policy, Mapping):
        # 系统级 floor_switch 键 -> 内部设置键的别名映射
        aliases = {
            'transfer_adapter': 'adapter',
            'pose_handoff': 'pose_handoff',
            'transition_delay_sec': 'transition_delay_sec',
            'transfer_timeout_sec': 'timeout_sec',
            'external_action_name': 'external_action_name',
        }
        for source_key, target_key in aliases.items():
            if source_key in system_policy:
                settings[target_key] = system_policy[source_key]
    for owner, path in (
        (connector, 'connector.transfer'),
        (route, 'route.transfer'),
    ):
        # 连廊级与路线级的 transfer 覆盖（优先级递增：系统 < 连廊 < 路线）
        override = owner.get('transfer', {})
        if override is None:
            continue
        if not isinstance(override, Mapping):
            raise ValueError(f'{path} must be a mapping')
        for key in settings:
            if key in override:
                settings[key] = override[key]

    adapter = str(settings['adapter']).strip()
    handoff = str(settings['pose_handoff']).strip()
    action_name = str(settings['external_action_name']).strip()
    delay = _finite_number(
        settings['transition_delay_sec'], 'transition_delay_sec'
    )
    timeout = _finite_number(settings['timeout_sec'], 'transfer timeout_sec')
    if adapter not in TRANSFER_ADAPTERS:
        raise ValueError(
            f'unsupported transfer adapter {adapter!r}; '
            f'expected one of {sorted(TRANSFER_ADAPTERS)}'
        )
    if handoff not in POSE_HANDOFFS:
        raise ValueError(
            f'unsupported pose handoff {handoff!r}; '
            f'expected one of {sorted(POSE_HANDOFFS)}'
        )
    if delay < 0.0:
        raise ValueError('transition_delay_sec cannot be negative')
    if timeout <= 0.0:
        raise ValueError('transfer timeout_sec must be greater than zero')
    if adapter == 'external_action' and not action_name:
        raise ValueError('external_action_name cannot be empty')
    return adapter, handoff, delay, timeout, action_name


def resolve_transition(
    config: Mapping[str, Any],
    connector_id: str,
    source_floor: str,
    target_floor: str,
) -> TransitionPlan:
    """Resolve an explicit route or a legacy bidirectional connector."""
    # 【中文】解析一条显式路线（transitions）或兼容旧格式的双向连廊。
    # 【参数】config - 系统配置；connector_id - 连廊 id；source_floor - 源楼层；target_floor - 目标楼层
    # 【返回】TransitionPlan 路线计划；配置非法时抛出 ValueError
    connectors = config.get('elevators', {})
    floors = config.get('floors', {})
    if connector_id not in connectors:
        raise ValueError(f'unknown floor connector {connector_id!r}')
    if source_floor not in floors or target_floor not in floors:
        raise ValueError('source_floor or target_floor is unknown')
    if source_floor == target_floor:
        raise ValueError('source_floor and target_floor must differ')

    connector = connectors[connector_id]
    if not isinstance(connector, Mapping):
        raise ValueError(f'connector {connector_id!r} must be a mapping')
    route: Mapping[str, Any] = {}
    routes = connector.get('transitions')
    if routes is not None:
        # 显式路线表：按 from/to 精确匹配唯一一条路线
        if not isinstance(routes, list) or not routes:
            raise ValueError(
                f'{connector_id}.transitions must be a non-empty list'
            )
        matches = [
            candidate
            for candidate in routes
            if isinstance(candidate, Mapping)
            and str(candidate.get('from', '')) == source_floor
            and str(candidate.get('to', '')) == target_floor
        ]
        if len(matches) != 1:
            qualifier = 'duplicate' if len(matches) > 1 else 'no'
            raise ValueError(
                f'connector {connector_id!r} has {qualifier} route for '
                f'{source_floor}->{target_floor}'
            )
        route = matches[0]
        trigger = _pose3(
            route['source_trigger_pose'],
            f'{connector_id} {source_floor}->{target_floor} '
            'source_trigger_pose',
        )
        release = _pose3(
            route['target_release_pose'],
            f'{connector_id} {source_floor}->{target_floor} '
            'target_release_pose',
        )
    else:
        # 旧格式双向连廊：判断正向/反向，正向用连廊自带位姿，反向需显式启用
        forward = (
            source_floor == connector['source_floor']
            and target_floor == connector['target_floor']
        )
        reverse = (
            source_floor == connector['target_floor']
            and target_floor == connector['source_floor']
        )
        if forward:
            route = connector
            trigger = _pose3(
                connector['source_trigger_pose'], 'source_trigger_pose'
            )
            release = _pose3(
                connector['target_release_pose'], 'target_release_pose'
            )
        elif reverse and bool(
            connector.get('reverse_transition_enabled', False)
        ):
            # 反向路线：无显式位姿时回退到电梯轿厢位姿 / 电梯厅位姿
            trigger = _pose3(
                connector.get(
                    'reverse_source_trigger_pose',
                    floors[source_floor]['elevator_cabin_pose'],
                ),
                'reverse source trigger pose',
            )
            release = _pose3(
                connector.get(
                    'reverse_target_release_pose',
                    floors[target_floor]['elevator_lobby_pose'],
                ),
                'reverse target release pose',
            )
        else:
            raise ValueError(
                f'connector {connector_id!r} does not support '
                f'{source_floor}->{target_floor}'
            )

    tolerance_xy = _finite_number(
        route.get(
            'trigger_tolerance_xy', connector.get('trigger_tolerance_xy')
        ),
        'trigger_tolerance_xy',
    )
    tolerance_yaw = _finite_number(
        route.get(
            'trigger_tolerance_yaw', connector.get('trigger_tolerance_yaw')
        ),
        'trigger_tolerance_yaw',
    )
    if tolerance_xy <= 0.0 or tolerance_yaw <= 0.0:
        raise ValueError('trigger tolerances must be greater than zero')
    adapter, handoff, delay, timeout, action_name = _transfer_policy(
        config, connector, route
    )

    return TransitionPlan(
        connector_id=connector_id,
        source_floor=source_floor,
        target_floor=target_floor,
        source_trigger=trigger,
        target_release=release,
        trigger_tolerance_xy=tolerance_xy,
        trigger_tolerance_yaw=tolerance_yaw,
        transfer_adapter=adapter,
        pose_handoff=handoff,
        transition_delay_sec=delay,
        transfer_timeout_sec=timeout,
        external_action_name=action_name,
    )


def pose_error(current: Pose2D, target: Pose2D) -> Tuple[float, float]:
    """Return planar distance and wrapped absolute yaw error."""
    # 【中文】计算当前位姿相对目标位姿的平面距离与归一化（[-pi, pi] 包裹）的航向角误差
    # 【参数】current - 当前位姿 (x, y, yaw)；target - 目标位姿 (x, y, yaw)
    # 【返回】(平面距离, 航向角误差绝对值) 二元组
    distance = math.hypot(current[0] - target[0], current[1] - target[1])
    yaw_error = abs(
        math.atan2(
            math.sin(current[2] - target[2]),
            math.cos(current[2] - target[2]),
        )
    )
    return distance, yaw_error
