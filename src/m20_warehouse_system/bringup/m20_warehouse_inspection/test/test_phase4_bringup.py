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

"""Guard the phase-4 typed navigation and mission composition contract."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SYSTEM_ROOT = PACKAGE_ROOT.parents[1]
PACKAGE_PATHS = {
    'm20_warehouse_inspection': PACKAGE_ROOT,
    'm20_inspection_core': (
        SYSTEM_ROOT / 'safety_mission' / 'm20_inspection_core'
    ),
    'm20_scan_navigation': (
        SYSTEM_ROOT / 'navigation' / 'm20_scan_navigation'
    ),
}


def _source(package: str, relative: str) -> str:
    return (PACKAGE_PATHS[package] / relative).read_text(encoding='utf-8')


def test_phase4_launch_composes_phase3_and_mission_executor() -> None:
    launch = (
        PACKAGE_ROOT / 'launch' / 'inspection_mission_rviz.launch.py'
    ).read_text(encoding='utf-8')
    assert 'multifloor_scan_rviz.launch.py' in launch
    assert "executable='m20_mission_executor'" in launch
    assert 'use_grid_route' not in launch
    assert "'system_config': system_config" in launch
    assert 'collision_grid_route_enabled' not in launch
    assert 'clearance_m20.yaml' in launch
    assert 'scan_m20_physical_planner.yaml' not in launch
    assert 'planner_config' not in launch


def test_dense_profile_has_an_independent_named_launch_entry() -> None:
    launch = (
        PACKAGE_ROOT
        / 'launch'
        / 'dense_four_corner_mission_rviz.launch.py'
    ).read_text(encoding='utf-8')
    assert 'dense_four_corner_system.yaml' in launch
    assert 'inspection_mission_rviz.launch.py' in launch


def test_route_challenge_has_an_independent_named_launch_entry() -> None:
    launch = (
        PACKAGE_ROOT
        / 'launch'
        / 'route_challenge_mission_rviz.launch.py'
    ).read_text(encoding='utf-8')
    assert 'route_challenge_system.yaml' in launch
    assert 'inspection_mission_rviz.launch.py' in launch


def test_automatic_mission_uses_the_vendor_scan_chain() -> None:
    with (
        PACKAGE_ROOT / 'config' / 'flat_multifloor_system.yaml'
    ).open('r', encoding='utf-8') as stream:
        navigation = yaml.safe_load(stream)['navigation']
    assert navigation['global_route_backend'] == 'scan_native'
    assert navigation['automatic_mission_route_backend'] == 'scan_native'
    assert navigation['local_planner_backend'] == 'scan_planner'


def test_simple_starter_only_triggers_the_running_mission_executor() -> None:
    source = _source(
        'm20_warehouse_inspection',
        'm20_warehouse_inspection/start_inspection_node.py',
    )
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert "RunMission, '/m20/mission/run'" in source
    assert 'send_goal_async(' in source
    assert 'm20_mission_executor' not in source
    assert 'scripts/m20_start_inspection' in cmake


def test_mission_uses_typed_child_actions_and_connector_approach() -> None:
    source = _source(
        'm20_inspection_core',
        'm20_inspection_core/mission_executor_node.py',
    )
    assert 'ActionClient(' in source
    assert 'NavigateFloor' in source
    assert 'SwitchFloor' in source
    assert "(step.pose, 'connector_approach')" in source
    assert "(plan.source_trigger, 'connector_trigger')" in source
    assert 'def _run_transit(' in source
    assert 'def _run_terminal(' in source
    assert 'display_step_index(' in source
    assert "'/m20/control/floor_switch_hold'" in source


def test_navigation_gateway_waits_for_its_own_floor_release_sample() -> None:
    source = _source(
        'm20_scan_navigation',
        'm20_scan_navigation/navigation_gateway_node.py',
    )
    assert "'floor_hold_release_timeout_sec'" in source
    assert "'WAITING_FOR_FLOOR_RELEASE'" in source
    assert 'while self._floor_hold' in source
    assert 'self._floor_hold' in source


def test_pause_fault_and_stop_assert_independent_mission_hold() -> None:
    mission = _source(
        'm20_inspection_core',
        'm20_inspection_core/mission_executor_node.py',
    )
    safety = _source(
        'm20_inspection_core',
        'm20_inspection_core/safety_supervisor_node.py',
    )
    assert "'/m20/control/mission_hold'" in mission
    assert "self._publish_hold(True)" in mission
    assert "'MISSION_HOLD'" in safety


def test_committed_floor_fault_has_a_safe_retry_path() -> None:
    switch = _source(
        'm20_inspection_core',
        'm20_inspection_core/floor_switch_manager_node.py',
    )
    assert "'RECOVERING_COMMITTED_TARGET'" in switch
    assert 'state.floor_id == plan.target_floor' in switch
    assert "'committed target recovered; releasing safety hold'" in switch


def test_pose_transfer_client_is_lazy_and_policy_selected() -> None:
    switch = _source(
        'm20_inspection_core',
        'm20_inspection_core/floor_switch_manager_node.py',
    )
    assert 'self._pose_client = None' in switch
    assert "if plan.pose_handoff != 'set_simulation_pose':" in switch
    assert 'if self._pose_client is None:' in switch
    assert "if plan.pose_handoff == 'preserve':" in switch
