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

"""Protect the SCAN topic and safety boundaries from launch regressions."""

from pathlib import Path
import re

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _canonical_cpp(source: str) -> str:
    """Remove comments and layout for a copied-core parity check."""
    source = re.sub(r'/\*.*?\*/', '', source, flags=re.DOTALL)
    source = re.sub(r'//[^\n]*', '', source)
    return re.sub(r'\s+', '', source)


def test_planner_manager_logic_matches_the_local_upstream_baseline() -> None:
    upstream = (
        PACKAGE_ROOT.parent
        / 'third_party'
        / 'SCAN-Planner'
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    adapted = (
        PACKAGE_ROOT.parent
        / 'm20_scan_planner'
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    assert _canonical_cpp(adapted) == _canonical_cpp(upstream)


def test_controller_only_targets_raw_velocity_interface() -> None:
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert "'cmd_vel', '/m20/navigation/cmd_vel_raw'" in launch_text
    assert "'cmd_vel', '/cmd_vel'" not in launch_text
    assert "'cmd_vel', '/m20/control/cmd_vel_safe'" not in launch_text


def test_f1_planner_is_ground_limited() -> None:
    with (PACKAGE_ROOT / 'config' / 'f1_planner.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        parameters = yaml.safe_load(stream)['/**']['ros__parameters']
    assert parameters['fsm.navi_mode'] == 1
    assert parameters['grid_map.sensor_type'] == 'lidar'
    assert parameters['grid_map.cloud_is_world'] is True
    assert parameters['manager.max_vel'] <= 0.40
    assert parameters['grid_map.body_height'] == 0.59
    assert parameters['fsm.target_reached_tolerance'] == 0.20


def test_native_profile_keeps_upstream_scan_trajectory_parameters() -> None:
    with (PACKAGE_ROOT / 'config' / 'scan_vendor_planner.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        native = yaml.safe_load(stream)['/**']['ros__parameters']
    upstream_path = (
        PACKAGE_ROOT.parent
        / 'third_party'
        / 'SCAN-Planner'
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'config'
        / 'planner.yaml'
    )
    with upstream_path.open('r', encoding='utf-8') as stream:
        upstream = yaml.safe_load(stream)[
            'scan_planner_node'
        ]['ros__parameters']
    for key in upstream:
        assert native[key] == upstream[key]
    assert native['fsm.target_reached_tolerance'] == 0.20


def test_native_controller_matches_upstream_before_safety_limits() -> None:
    with (PACKAGE_ROOT / 'config' / 'scan_vendor_controller.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        native = yaml.safe_load(stream)['/**']['ros__parameters']
    upstream_path = (
        PACKAGE_ROOT.parent
        / 'third_party'
        / 'SCAN-Planner'
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'config'
        / 'controllers.yaml'
    )
    with upstream_path.open('r', encoding='utf-8') as stream:
        upstream = yaml.safe_load(stream)[
            'closed_loop_controller'
        ]['ros__parameters']
    for key, value in upstream.items():
        assert native[key] == value


def test_m20_physical_controller_layers_heading_alignment_after_vendor() -> None:
    with (
        PACKAGE_ROOT
        / 'config'
        / 'scan_m20_physical_controller.yaml'
    ).open('r', encoding='utf-8') as stream:
        physical = yaml.safe_load(stream)[
            'closed_loop_controller'
        ]['ros__parameters']
    assert physical == {
        'heading_error_threshold': 0.55,
        'heading_error_resume_threshold': 0.20,
        'heading_slowdown_threshold': 0.15,
        'heading_alignment_min_hold_sec': 0.40,
    }
    launch = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert 'controller_config = LaunchConfiguration' in launch
    assert (
        "str(share / 'config' / 'scan_vendor_controller.yaml'),\n"
        "            controller_config,"
    ) in launch
    assert "('heading_error', '/m20/navigation/heading_error')" in launch
    assert (
        "('heading_aligning', '/m20/navigation/heading_aligning')"
        in launch
    )


def test_local_sensing_consumes_only_active_map() -> None:
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert '/map_generator/global_cloud' in launch_text
    assert "'/quad_0/cloud'" in launch_text
    assert 'scan_vendor_local_sensing.yaml' in launch_text
    assert '/m20/visualization/all_floors_cloud' not in launch_text


def test_native_scan_is_default_and_grid_route_is_optional() -> None:
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert 'm20_grid_route_planner' in launch_text
    assert "default_value='false'" in launch_text
    assert "package='m20_scan_planner'" in launch_text
    assert "executable='scan_planner_node'" in launch_text
    assert "'scan_vendor_planner.yaml'" in launch_text
    assert "'scan_vendor_controller.yaml'" in launch_text
    assert "'direct_goal_topic': '/move_base_simple/goal'" in launch_text
    assert "'scan_goal_topic': '/move_base_simple/goal'" in launch_text


def test_route_adapter_uses_tighter_final_acceptance() -> None:
    with (PACKAGE_ROOT / 'config' / 'f1_grid_route.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        parameters = yaml.safe_load(stream)[
            'm20_grid_route_planner'
        ]['ros__parameters']
    assert (
        parameters['final_acceptance_radius']
        < parameters['subgoal_acceptance_radius']
    )


def test_adapter_does_not_build_a_duplicate_scan_core() -> None:
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'add_executable(m20_scan_planner_node' not in cmake
    assert 'add_executable(m20_scan_controller' not in cmake
    assert '<exec_depend>m20_scan_planner</exec_depend>' in (
        PACKAGE_ROOT / 'package.xml'
    ).read_text(encoding='utf-8')
    assert not list((PACKAGE_ROOT / 'src').glob('*.cpp'))
    assert not list((PACKAGE_ROOT / 'include').rglob('*.h'))
    assert not list((PACKAGE_ROOT / 'include').rglob('*.hpp'))


def test_replan_state_has_near_goal_exit() -> None:
    source = (
        PACKAGE_ROOT.parent
        / 'm20_scan_planner'
        / 'src'
        / 'scan_replan_fsm.cpp'
    ).read_text(encoding='utf-8')
    assert 'target_reached_tolerance_' in source
    assert 'callEmergencyStop(odom_pos_)' in source
    assert 'changeFSMExecState(WAIT_TARGET, "GOAL_REACHED")' in source


def test_typed_gateway_resets_route_and_scan_on_cancel() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/navigation/navigate'" in source
    assert 'self._route_reset_client' in source
    assert 'self._scan_reset_client' in source
    assert 'floor.generation != goal.map_generation' in source


def test_route_adapter_exposes_typed_reset() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'grid_route_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/navigation/route_reset'" in source
    assert 'state.generation != request.generation' in source
    assert 'self._clear_route()' in source
