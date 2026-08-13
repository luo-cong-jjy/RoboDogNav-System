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

"""Unit tests for mission configuration resolution."""

from pathlib import Path

from m20_inspection_core.mission_policy import (
    display_step_index,
    resolve_mission,
)
import pytest
import yaml


CONFIG = (
    Path(__file__).resolve().parents[3]
    / 'bringup'
    / 'm20_warehouse_inspection'
    / 'config'
    / 'flat_multifloor_system.yaml'
)
DENSE_CONFIG = CONFIG.with_name('dense_four_corner_system.yaml')


def _config():
    with CONFIG.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _dense_config():
    with DENSE_CONFIG.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_resolves_complete_two_floor_sequence() -> None:
    steps = resolve_mission(_config(), 'two_floor_warehouse_demo')
    assert [step.name for step in steps] == [
        'F1_A',
        'F1_B',
        'E1:F1->F2',
        'F2_A',
        'F2_B',
        'F2_RETURN_VIA_A',
        'F2_TERMINAL',
    ]
    assert steps[2].pose == (0.0, 0.0, 0.0)
    assert steps[2].target_floor == 'F2'
    assert steps[4].step_type == 'inspection'
    assert steps[5].step_type == 'transit'
    assert steps[5].pose == steps[3].pose
    assert steps[5].dwell_sec == 0.0
    assert steps[6].step_type == 'terminal'
    assert steps[6].pose == (0.0, 0.0, 0.0)


def test_rejects_unknown_mission() -> None:
    with pytest.raises(ValueError, match='unknown mission'):
        resolve_mission(_config(), 'not_configured')


def test_dense_mission_switches_back_and_resolves_the_real_start_pose() -> None:
    steps = resolve_mission(_dense_config(), 'dense_four_corner_patrol')
    assert len(steps) == 11
    assert [step.name for step in steps[:4]] == [
        'F1_LOWER_LEFT',
        'F1_LOWER_RIGHT',
        'F1_UPPER_RIGHT',
        'F1_UPPER_LEFT',
    ]
    assert [step.name for step in steps[5:9]] == [
        'F2_LOWER_LEFT',
        'F2_LOWER_RIGHT',
        'F2_UPPER_RIGHT',
        'F2_UPPER_LEFT',
    ]
    assert steps[-2].name == 'E1:F2->F1'
    assert steps[-2].target_floor == 'F1'
    assert steps[-1].step_type == 'terminal'
    assert steps[-1].floor_id == 'F1'
    assert steps[-1].pose == (-37.0, 0.0, 0.0)


def test_completed_cursor_displays_the_last_real_step() -> None:
    assert display_step_index(7, 7) == 6
    assert display_step_index(6, 7) == 6
    assert display_step_index(-1, 7) == 0
    assert display_step_index(0, 0) == 0


def test_legacy_elevator_transfer_keeps_connector_semantics() -> None:
    config = _config()
    transfer = config['mission']['sequence'][2]
    transfer['type'] = 'elevator_transfer'
    transfer['elevator'] = transfer.pop('connector')

    steps = resolve_mission(config, 'two_floor_warehouse_demo')

    assert steps[2].step_type == 'elevator_transfer'
    assert steps[2].connector_id == 'E1'
    assert steps[2].target_floor == 'F2'
