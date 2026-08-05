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

from m20_scan_navigation.recovery_replan import (
    collision_replan_available,
    recovery_budget_exhausted,
    recovery_replan_required,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _canonical_cpp(source: str) -> str:
    """Remove comments and layout for a copied-core parity check."""
    source = re.sub(
        r'/\* M20_INTEGRATION_SHORT_ROUTE_TIMING_BEGIN \*/.*?'
        r'/\* M20_INTEGRATION_SHORT_ROUTE_TIMING_END \*/',
        '',
        source,
        flags=re.DOTALL,
    )
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


def test_only_allowed_scan_core_delta_is_bounded_time_scaling() -> None:
    adapted = (
        PACKAGE_ROOT.parent
        / 'm20_scan_planner'
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    assert adapted.count('M20_INTEGRATION_SHORT_ROUTE_TIMING_BEGIN') == 1
    assert adapted.count('M20_INTEGRATION_SHORT_ROUTE_TIMING_END') == 1
    assert 'attempt < 6' in adapted
    assert 'pos.getInterval() * 1.10' in adapted
    assert 'pos = UniformBspline(pos.getControlPoint(), 3, scaled_interval)' in adapted
    assert 'pos.lengthenTime(1.10)' not in adapted
    assert 'while retaining configured dynamic limits' in adapted


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


def test_m20_controller_keeps_vendor_heading_behavior() -> None:
    controller = (
        PACKAGE_ROOT.parent
        / 'm20_scan_planner'
        / 'src'
        / 'closed_loop_controller.cpp'
    ).read_text(encoding='utf-8')
    assert 'heading_error_resume_threshold' not in controller
    assert 'heading_slowdown_threshold' not in controller
    assert 'heading_alignment_min_hold_sec' not in controller
    assert 'headingTranslationScale' not in controller
    assert (
        'if (std::abs(yaw_error) > heading_error_threshold_)'
        in controller
    )
    # This is the only controller extension needed by atomic floor switching
    # and fail-closed collision supervision.
    assert 'execution_hold_topic' in controller
    assert 'if (!external_execution_hold_)' in controller
    assert 'bidirectional_tracking_enabled' in controller
    assert 'reverse_tracking_enter_angle' in controller
    assert 'reverse_tracking_exit_angle' in controller
    assert 'reverse_tracking_entry_alignment' in controller
    assert 'reverse_tracking_exit_alignment' in controller
    assert 'reversePathIsStraight' in controller
    assert 'updateTrackingDirection' in controller
    assert 'return reverse_tracking_ ? reverse_tracking_yaw_' in controller
    assert 'planning/tracking_direction' in controller

    launch = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert 'controller_config = LaunchConfiguration' in launch
    assert (
        "str(share / 'config' / 'scan_vendor_controller.yaml'),\n"
        "            controller_config,"
    ) in launch
    assert "('heading_error'," not in launch
    assert "('heading_aligning'," not in launch
    assert "default_value='false'" in launch
    assert 'bidirectional_tracking_enabled' in launch
    assert 'reverse_tracking_entry_alignment' in launch
    assert 'reverse_tracking_exit_alignment' in launch


def test_scan_launch_has_no_planner_parameter_override() -> None:
    launch = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert 'planner_config' not in launch
    assert (
        "str(share / 'config' / 'scan_vendor_planner.yaml'),\n"
        "            clearance_config,"
    ) in launch


def test_conservative_profile_keeps_vendor_scan_clearance() -> None:
    with (
        PACKAGE_ROOT / 'config' / 'clearance_conservative.yaml'
    ).open('r', encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'scan_planner_node'
        ]['ros__parameters']
    assert parameters == {
        'grid_map.double_cylinder_radius': 0.25,
        'optimization.dist0': 0.20,
    }


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
    assert "'/m20/navigation/scan_goal_internal'" in launch_text
    assert "'manual_goal_topic': '/move_base_simple/goal'" in launch_text
    assert "'manual_goal_monitor_enabled'" in launch_text
    assert "'scan_goal_topic': ParameterValue(" in launch_text
    assert "'direct_goal_topic': ParameterValue(" in launch_text
    assert "'collision_grid_route_enabled'" in launch_text
    assert 'PythonExpression' in launch_text


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
    assert parameters['final_acceptance_radius'] == 0.20
    assert (
        parameters['minimum_scan_goal_distance']
        > parameters['subgoal_acceptance_radius']
    )
    assert parameters['subgoal_stall_timeout_sec'] <= 10.0
    assert parameters['inflation_radius'] == 0.60
    assert parameters['soft_clearance_margin'] == 0.20
    assert parameters['soft_clearance_weight'] == 4.0


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


def test_bounded_collision_replan_policy() -> None:
    assert recovery_budget_exhausted(
        'RECOVERY_BUDGET_EXHAUSTED; reason=TIME_LIMIT'
    )
    assert not recovery_budget_exhausted('RECOVERY_EPISODE_REARM')
    assert not recovery_budget_exhausted('CLEAR')
    assert recovery_replan_required(
        'RECOVERY_BUDGET_EXHAUSTED; reason=TIME_LIMIT'
    )
    assert recovery_replan_required(
        'RECOVERY_UNAVAILABLE; trigger=PREDICTED_FOOTPRINT'
    )
    assert not recovery_replan_required('PREDICTED_FOOTPRINT')
    assert not recovery_replan_required('CURRENT_FOOTPRINT')
    assert collision_replan_available(0, 2)
    assert collision_replan_available(1, 2)
    assert not collision_replan_available(2, 2)
    assert not collision_replan_available(0, 0)


def test_gateway_replans_only_after_guard_rearms() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    assert 'COLLISION_REPLAN_EXHAUSTED' in source
    assert "self._collision_diagnostic == 'CLEAR'" in source
    assert "'WAITING_FOR_RECOVERY_REARM'" in source
    assert 'collision_replan_max_attempts' in source
    assert 'self._collision_route_publisher.publish(target)' in source
    assert 'route_mode_active' in source
    assert 'goal rerouted through M20 clearance subgoals' in source
    assert "'SUBGOAL_STALLED'," in source
    assert "'SUBGOAL_STOP_TIMEOUT'," in source


def test_managed_rviz_goal_uses_same_bounded_fallback() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'navigation_gateway.yaml').read_text(
            encoding='utf-8'
        )
    )['m20_navigation_gateway']['ros__parameters']

    assert "'manual_goal_monitor_enabled'" in source
    assert 'self._manual_goal_callback' in source
    assert 'self._manual_tick' in source
    assert "'TRACKING_NATIVE'" in source
    assert "'TRACKING_ROUTE'" in source
    assert 'self._collision_route_publisher.publish(target)' in source
    assert "'COLLISION_REPLAN_EXHAUSTED'" in source
    assert 'self._reset_navigation(' in source
    assert config['manual_goal_monitor_enabled'] is False
    assert config['manual_goal_topic'] == '/move_base_simple/goal'
    assert config['manual_timeout_sec'] == 300.0
    assert config['rear_goal_route_enabled'] is True
    assert config['rear_goal_trigger_angle'] == 2.10
    assert config['terminal_orientation_mode'] == 'position_only'
    assert 'self._goal_requires_route(target)' in source
    assert 'self._diagnostic_requires_replan()' in source


def test_guard_exposes_missing_safe_recovery_to_gateway() -> None:
    source = (
        PACKAGE_ROOT.parent
        / 'm20_inspection_core'
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "'RECOVERY_UNAVAILABLE; '" in source
    assert 'elif not self._recovery_exhausted_reason:' in source


def test_route_adapter_exposes_typed_reset() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'grid_route_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/navigation/route_reset'" in source
    assert 'state.generation != request.generation' in source
    assert 'self._clear_route()' in source


def test_route_adapter_stops_between_heading_segments() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'grid_route_node.py'
    ).read_text(encoding='utf-8')
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'f1_grid_route.yaml').read_text(
            encoding='utf-8'
        )
    )['m20_grid_route_planner']['ros__parameters']
    assert 'self._waiting_for_intermediate_stop = True' in source
    assert 'self._publish_segment_hold(True)' in source
    assert 'self._segment_hold_release_at = now + max(' in source
    assert "self._publish_state('SUBGOAL_STOP_TIMEOUT')" in source
    assert (
        parameters['segment_hold_topic']
        == '/m20/control/route_segment_hold'
    )
    assert parameters['segment_hold_release_delay_sec'] == 0.20
    assert parameters['intermediate_stop_linear_speed'] == 0.04
    assert parameters['intermediate_stop_angular_speed'] == 0.08
    assert parameters['intermediate_stop_stable_sec'] == 0.30
    assert parameters['intermediate_stop_timeout_sec'] == 6.0
    assert parameters['continuous_handoff_enabled'] is True
    assert parameters['continuous_handoff_radius'] == 0.55
    assert parameters['continuous_handoff_distance'] == 0.70
    assert parameters['continuous_max_heading_change'] == 0.70
    assert parameters['continuous_minimum_turn_radius'] == 0.54
    assert parameters['continuous_minimum_extra_clearance'] == 0.10
    assert 'if continuous:' in source
    assert 'carrying velocity into the next clearance segment' in source

    safety_source = (
        PACKAGE_ROOT.parent
        / 'm20_inspection_core'
        / 'm20_inspection_core'
        / 'safety_supervisor_node.py'
    ).read_text(encoding='utf-8')
    assert "return 'ROUTE_SEGMENT_HOLD'" in safety_source
