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

"""Tests for system configuration invariants."""

from copy import deepcopy
from pathlib import Path

import pytest

from m20_warehouse_inspection.configuration import (
    ConfigurationError,
    display_to_source_xy,
    floor_display_bounds,
    load_system_config,
    validate_system_config,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'flat_multifloor_system.yaml'
DENSE_CONFIG_PATH = (
    PACKAGE_ROOT / 'config' / 'dense_four_corner_system.yaml'
)
CHALLENGE_CONFIG_PATH = (
    PACKAGE_ROOT / 'config' / 'route_challenge_system.yaml'
)


def test_shipped_configuration_is_valid_and_flat() -> None:
    config = load_system_config(CONFIG_PATH)
    assert list(config['floors']) == ['F1', 'F2']
    assert config['simulation']['layout_mode'] == 'flat_connected_regions'
    assert config['simulation']['teleport_on_floor_switch'] is False
    assert all(
        floor['simulation_offset'][2] == 0.0
        and floor['overview_z'] == 0.0
        for floor in config['floors'].values()
    )
    assert floor_display_bounds(config, 'F1') == (-40.0, 0.0, -20.0, 20.0)
    assert floor_display_bounds(config, 'F2') == (0.0, 40.0, -20.0, 20.0)
    elevator = config['elevators']['E1']
    assert elevator['source_trigger_pose'] == [0.0, 0.0, 0.0]
    assert elevator['target_release_pose'] == [0.0, 0.0, 0.0]
    sequence = config['mission']['sequence']
    assert [step['type'] for step in sequence[-4:]] == [
        'inspection',
        'inspection',
        'transit',
        'terminal',
    ]
    assert sequence[-3]['point'] == 'F2_B'
    assert sequence[-2]['name'] == 'F2_RETURN_VIA_A'


def test_display_to_source_removes_only_simulation_offset() -> None:
    config = load_system_config(CONFIG_PATH)
    floor = config['floors']['F1']
    assert display_to_source_xy(floor, floor['initial_pose']) == (-17.0, 0.0)
    assert display_to_source_xy(floor, floor['elevator_cabin_pose']) == (
        20.0,
        0.0,
    )


def test_dense_profile_has_ordered_four_corner_patrol_and_home_return() -> None:
    config = load_system_config(DENSE_CONFIG_PATH)
    assert config['map_generation']['obstacle_count_per_floor'] == 246
    assert config['map_generation']['minimum_obstacle_spacing'] == 0.90
    assert len(config['floors']['F1']['fixed_obstacles']) == 6
    assert 'fixed_obstacles' not in config['floors']['F2']
    assert [
        point['name'] for point in config['floors']['F1']['inspection_points']
    ] == [
        'F1_LOWER_LEFT',
        'F1_LOWER_RIGHT',
        'F1_UPPER_RIGHT',
        'F1_UPPER_LEFT',
    ]
    assert [
        point['name'] for point in config['floors']['F2']['inspection_points']
    ] == [
        'F2_LOWER_LEFT',
        'F2_LOWER_RIGHT',
        'F2_UPPER_RIGHT',
        'F2_UPPER_LEFT',
    ]
    sequence = config['mission']['sequence']
    assert [
        step.get('point')
        for step in sequence
        if step['type'] == 'inspection'
    ] == [
        'F1_LOWER_LEFT',
        'F1_LOWER_RIGHT',
        'F1_UPPER_RIGHT',
        'F1_UPPER_LEFT',
        'F2_LOWER_LEFT',
        'F2_LOWER_RIGHT',
        'F2_UPPER_RIGHT',
        'F2_UPPER_LEFT',
    ]
    assert sequence[-2] == {
        'type': 'elevator_transfer',
        'elevator': 'E1',
        'from': 'F2',
        'to': 'F1',
    }
    assert sequence[-1]['location'] == 'initial_pose'


def test_route_challenge_is_isolated_and_preserves_the_mission_contract() -> None:
    config = load_system_config(CHALLENGE_CONFIG_PATH)
    assert config['system']['project_id'] == 'm20_warehouse_route_challenge'
    assert config['map_generation']['obstacle_count_per_floor'] == 252
    assert config['map_generation']['minimum_obstacle_spacing'] == 0.70
    assert len(config['floors']['F1']['fixed_obstacles']) == 12
    assert config['floors']['F2']['replica_of'] == 'F1'
    assert config['simulation']['teleport_on_floor_switch'] is False
    assert config['elevators']['E1']['source_trigger_pose'] == [0.0, 0.0, 0.0]
    assert config['elevators']['E1']['target_release_pose'] == [0.0, 0.0, 0.0]
    assert len(config['mission']['sequence']) == 11
    assert config['mission']['sequence'][-1]['location'] == 'initial_pose'


def test_obstacle_spacing_must_be_positive() -> None:
    config = deepcopy(load_system_config(DENSE_CONFIG_PATH))
    config['map_generation']['minimum_obstacle_spacing'] = 0.0
    with pytest.raises(ConfigurationError, match='minimum_obstacle_spacing'):
        validate_system_config(config)


def test_fixed_aisle_obstacles_must_obey_the_same_spacing_rule() -> None:
    config = deepcopy(load_system_config(DENSE_CONFIG_PATH))
    config['floors']['F1']['fixed_obstacles'][1]['center'] = [-27.0, -16.0]
    with pytest.raises(ConfigurationError, match='minimum obstacle spacing'):
        validate_system_config(config)


def test_f2_declares_an_exact_shifted_f1_replica() -> None:
    config = load_system_config(CONFIG_PATH)
    first = config['floors']['F1']
    second = config['floors']['F2']
    assert second['replica_of'] == 'F1'
    assert second['source_seed'] == first['source_seed']
    for first_point, second_point in zip(
        first['inspection_points'], second['inspection_points']
    ):
        assert display_to_source_xy(first, first_point['pose']) == (
            display_to_source_xy(second, second_point['pose'])
        )


def test_replica_must_share_its_source_seed() -> None:
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['floors']['F2']['source_seed'] += 1
    with pytest.raises(ConfigurationError, match='must match replica source'):
        validate_system_config(config)


def test_overlapping_floor_regions_are_rejected() -> None:
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['floors']['F2']['simulation_offset'][0] = 0.0
    with pytest.raises(ConfigurationError, match='overlap'):
        validate_system_config(config)


def test_nonzero_overview_height_is_rejected() -> None:
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['floors']['F2']['overview_z'] = 4.0
    with pytest.raises(ConfigurationError, match='overview_z'):
        validate_system_config(config)


def test_unknown_mission_reference_is_rejected() -> None:
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['mission']['sequence'][0]['point'] = 'F1_UNKNOWN'
    with pytest.raises(ConfigurationError, match='unknown point'):
        validate_system_config(config)


def test_terminal_must_reference_an_elevator_lobby() -> None:
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['mission']['sequence'][-1]['location'] = 'inter_region_gap'
    with pytest.raises(ConfigurationError, match='elevator_lobby'):
        validate_system_config(config)


def test_transit_requires_a_named_existing_point() -> None:
    config = deepcopy(load_system_config(CONFIG_PATH))
    config['mission']['sequence'][-2]['point'] = 'F2_UNKNOWN'
    config['mission']['sequence'][-2]['name'] = ''
    with pytest.raises(
        ConfigurationError,
        match='unknown point|non-empty string',
    ):
        validate_system_config(config)
