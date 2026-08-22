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
# 【文件职责】test_floor_switch_policy.py —— 楼层切换策略单元测试
# 测试 floor_switch_policy.py 的纯函数：
#   - resolve_transition：正向/反向电梯转移解析、显式多楼层路线
#     （transitions）与 transfer 覆盖、重复路线/非法适配器拒绝；
#   - pose_error：平面距离与航向角误差（含 2pi 归一化）。
# ============================================================================

"""Tests for directional elevator transition resolution."""

import math  # 数学库：构造测试位姿（pi 等）

import pytest  # 测试框架：异常断言与近似比较

from m20_inspection_core.floor_switch_policy import (  # 导入被测模块的纯函数
    pose_error,
    resolve_transition,
)


CONFIG = {  # 测试用最小双楼层连廊配置（E1：F1 <-> F2，支持反向）
    'floors': {
        'F1': {
            'elevator_lobby_pose': [-8.0, 0.0, 0.0],      # F1 电梯厅位姿
            'elevator_cabin_pose': [-6.5, 0.0, 0.0],      # F1 电梯轿厢位姿
        },
        'F2': {
            'elevator_lobby_pose': [8.0, 0.0, math.pi],   # F2 电梯厅位姿（朝 pi）
            'elevator_cabin_pose': [6.5, 0.0, math.pi],   # F2 电梯轿厢位姿（朝 pi）
        },
    },
    'elevators': {
        'E1': {
            'source_floor': 'F1',                         # 连廊源楼层
            'target_floor': 'F2',                         # 连廊目标楼层
            'source_trigger_pose': [-6.5, 0.0, 0.0],      # 源侧触发位姿
            'target_release_pose': [8.0, 0.0, math.pi],   # 目标侧释放位姿
            'trigger_tolerance_xy': 0.35,                 # 触发平面容差（米）
            'trigger_tolerance_yaw': 0.35,                # 触发航向容差（弧度）
            'reverse_transition_enabled': True,           # 允许反向转移
        }
    },
}


def test_resolves_forward_and_reverse_transition_poses():
    # 【测试】正向与反向转移应解析出正确的触发/释放位姿与默认策略
    forward = resolve_transition(CONFIG, 'E1', 'F1', 'F2')
    reverse = resolve_transition(CONFIG, 'E1', 'F2', 'F1')

    assert forward.source_trigger == (-6.5, 0.0, 0.0)
    assert forward.target_release == (8.0, 0.0, math.pi)
    assert reverse.source_trigger == (6.5, 0.0, math.pi)
    assert reverse.target_release == (-8.0, 0.0, 0.0)
    assert forward.connector_id == 'E1'
    assert forward.transfer_adapter == 'timed_hold'
    assert forward.pose_handoff == 'preserve'


def test_rejects_unsupported_direction_and_wraps_yaw_error():
    # 【测试】同楼层转移应被拒绝；航向角误差应做 2pi 归一化
    with pytest.raises(ValueError):
        resolve_transition(CONFIG, 'E1', 'F1', 'F1')

    distance, yaw_error = pose_error(
        (1.0, 1.0, -math.pi + 0.1),
        (1.0, 1.0, math.pi - 0.1),
    )
    assert distance == 0.0
    assert yaw_error == pytest.approx(0.2)


def test_explicit_routes_support_more_than_two_floors_and_overrides():
    # 【测试】显式路线表支持超过两层的转移，且连廊/路线级 transfer 可覆盖系统级设置
    config = {
        'simulation': {
            'teleport_on_floor_switch': False,
            'elevator_transition_delay_sec': 0.0,
        },
        'map_switch_transaction': {'relocate_timeout_sec': 2.0},
        'floor_switch': {  # 系统级楼层切换默认策略
            'transfer_adapter': 'timed_hold',
            'pose_handoff': 'preserve',
            'transition_delay_sec': 0.25,
            'transfer_timeout_sec': 90.0,
            'external_action_name': '/site/floor_transfer',
        },
        'floors': {'F1': {}, 'F2': {}, 'F3': {}},
        'elevators': {
            'LIFT_A': {
                'trigger_tolerance_xy': 0.4,
                'trigger_tolerance_yaw': 0.5,
                'transitions': [  # 显式路线：仅支持 F2 -> F3
                    {
                        'from': 'F2',
                        'to': 'F3',
                        'source_trigger_pose': [2.0, 1.0, 0.0],
                        'target_release_pose': [3.0, 1.0, 0.0],
                        'transfer': {  # 路线级覆盖：外部动作 + 等待目标位姿
                            'adapter': 'external_action',
                            'pose_handoff': 'wait_for_target',
                            'timeout_sec': 150.0,
                        },
                    }
                ],
            }
        },
    }

    plan = resolve_transition(config, 'LIFT_A', 'F2', 'F3')

    assert plan.source_trigger == (2.0, 1.0, 0.0)
    assert plan.target_release == (3.0, 1.0, 0.0)
    assert plan.transfer_adapter == 'external_action'
    assert plan.pose_handoff == 'wait_for_target'
    assert plan.transition_delay_sec == pytest.approx(0.25)
    assert plan.transfer_timeout_sec == pytest.approx(150.0)
    assert plan.external_action_name == '/site/floor_transfer'


def test_rejects_duplicate_explicit_route_and_unknown_adapter():
    # 【测试】重复路线应报错；不支持的传输适配器应报错
    config = {
        'floors': {'F1': {}, 'F2': {}},
        'floor_switch': {'transfer_adapter': 'unsupported'},
        'elevators': {
            'E1': {
                'trigger_tolerance_xy': 0.3,
                'trigger_tolerance_yaw': 0.3,
                'transitions': [  # 两条相同的 F1->F2 路线（构造重复）
                    {
                        'from': 'F1',
                        'to': 'F2',
                        'source_trigger_pose': [0.0, 0.0, 0.0],
                        'target_release_pose': [0.0, 0.0, 0.0],
                    },
                    {
                        'from': 'F1',
                        'to': 'F2',
                        'source_trigger_pose': [0.0, 0.0, 0.0],
                        'target_release_pose': [0.0, 0.0, 0.0],
                    },
                ],
            }
        },
    }
    with pytest.raises(ValueError, match='duplicate route'):
        resolve_transition(config, 'E1', 'F1', 'F2')

    config['elevators']['E1']['transitions'].pop()  # 去掉重复路线后再测非法适配器
    with pytest.raises(ValueError, match='unsupported transfer adapter'):
        resolve_transition(config, 'E1', 'F1', 'F2')
