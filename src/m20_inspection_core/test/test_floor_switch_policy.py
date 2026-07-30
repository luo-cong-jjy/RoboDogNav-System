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


def test_rejects_unsupported_direction_and_wraps_yaw_error():
    with pytest.raises(ValueError):
        resolve_transition(CONFIG, 'E1', 'F1', 'F1')

    distance, yaw_error = pose_error(
        (1.0, 1.0, -math.pi + 0.1),
        (1.0, 1.0, math.pi - 0.1),
    )
    assert distance == 0.0
    assert yaw_error == pytest.approx(0.2)
