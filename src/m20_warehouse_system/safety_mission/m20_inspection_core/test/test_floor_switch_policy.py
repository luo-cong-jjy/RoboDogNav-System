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

"""Tests for directional elevator transition resolution."""

import math

import pytest

from m20_inspection_core.floor_switch_policy import (
    pose_error,
    resolve_transition,
)


CONFIG = {
    'floors': {
        'F1': {
            'elevator_lobby_pose': [-8.0, 0.0, 0.0],
            'elevator_cabin_pose': [-6.5, 0.0, 0.0],
        },
        'F2': {
            'elevator_lobby_pose': [8.0, 0.0, math.pi],
            'elevator_cabin_pose': [6.5, 0.0, math.pi],
        },
    },
    'elevators': {
        'E1': {
            'source_floor': 'F1',
            'target_floor': 'F2',
            'source_trigger_pose': [-6.5, 0.0, 0.0],
            'target_release_pose': [8.0, 0.0, math.pi],
            'trigger_tolerance_xy': 0.35,
            'trigger_tolerance_yaw': 0.35,
            'reverse_transition_enabled': True,
        }
    },
}


def test_resolves_forward_and_reverse_transition_poses():
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
    with pytest.raises(ValueError):
        resolve_transition(CONFIG, 'E1', 'F1', 'F1')

    distance, yaw_error = pose_error(
        (1.0, 1.0, -math.pi + 0.1),
        (1.0, 1.0, math.pi - 0.1),
    )
    assert distance == 0.0
    assert yaw_error == pytest.approx(0.2)


def test_explicit_routes_support_more_than_two_floors_and_overrides():
    config = {
        'simulation': {
            'teleport_on_floor_switch': False,
            'elevator_transition_delay_sec': 0.0,
        },
        'map_switch_transaction': {'relocate_timeout_sec': 2.0},
        'floor_switch': {
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
                'transitions': [
                    {
                        'from': 'F2',
                        'to': 'F3',
                        'source_trigger_pose': [2.0, 1.0, 0.0],
                        'target_release_pose': [3.0, 1.0, 0.0],
                        'transfer': {
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
    config = {
        'floors': {'F1': {}, 'F2': {}},
        'floor_switch': {'transfer_adapter': 'unsupported'},
        'elevators': {
            'E1': {
                'trigger_tolerance_xy': 0.3,
                'trigger_tolerance_yaw': 0.3,
                'transitions': [
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

    config['elevators']['E1']['transitions'].pop()
    with pytest.raises(ValueError, match='unsupported transfer adapter'):
        resolve_transition(config, 'E1', 'F1', 'F2')
