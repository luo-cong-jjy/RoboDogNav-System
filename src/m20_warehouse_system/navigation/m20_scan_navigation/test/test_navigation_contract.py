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

from m20_scan_navigation.navigation_gateway_node import guard_allows_motion
from m20_scan_navigation.recovery_replan import (
    collision_replan_available,
    recovery_budget_exhausted,
    recovery_replan_required,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SYSTEM_ROOT = PACKAGE_ROOT.parents[1]
SOURCE_ROOT = SYSTEM_ROOT.parent
PLANNER_ROOT = SYSTEM_ROOT / 'navigation' / 'm20_scan_planner'
SAFETY_ROOT = SYSTEM_ROOT / 'safety_mission' / 'm20_inspection_core'
UPSTREAM_SCAN_ROOT = SOURCE_ROOT / 'third_party' / 'SCAN-Planner'


def test_guard_readiness_follows_execution_profile() -> None:
    assert guard_allows_motion(False, True, '')
    assert guard_allows_motion(False, True, 'STALE_CLOUD')
    assert not guard_allows_motion(True, True, 'CLEAR')
    assert not guard_allows_motion(True, False, 'STALE_CLOUD')
    assert guard_allows_motion(True, False, 'CLEAR')


def _canonical_cpp(source: str) -> str:
    """Remove comments and layout for a copied-core parity check."""
    source = re.sub(r'/\*.*?\*/', '', source, flags=re.DOTALL)
    source = re.sub(r'//[^\n]*', '', source)
    return re.sub(r'\s+', '', source)


def test_planner_manager_logic_matches_the_local_upstream_baseline() -> None:
    upstream = (
        UPSTREAM_SCAN_ROOT
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    adapted = (
        PLANNER_ROOT
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    assert _canonical_cpp(adapted) == _canonical_cpp(upstream)


def test_scan_core_has_no_m20_time_scaling_delta() -> None:
    adapted = (
        PLANNER_ROOT
        / 'src'
        / 'planner_manager.cpp'
    ).read_text(encoding='utf-8')
    assert 'M20_INTEGRATION_SHORT_ROUTE_TIMING' not in adapted
    assert 'scaled_interval' not in adapted


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


def test_native_profile_keeps_upstream_scan_trajectory_parameters() -> None:
    with (PACKAGE_ROOT / 'config' / 'scan_vendor_planner.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        native = yaml.safe_load(stream)['/**']['ros__parameters']
    upstream_path = (
        UPSTREAM_SCAN_ROOT
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
    assert set(native) == set(upstream)


def test_native_controller_matches_upstream_before_safety_limits() -> None:
    with (PACKAGE_ROOT / 'config' / 'scan_vendor_controller.yaml').open(
        'r', encoding='utf-8'
    ) as stream:
        native = yaml.safe_load(stream)['/**']['ros__parameters']
    upstream_path = (
        UPSTREAM_SCAN_ROOT
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
        PLANNER_ROOT
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


def test_scan_launch_keeps_vendor_default_with_an_explicit_override_seam() -> None:
    launch = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert "planner_config = LaunchConfiguration('planner_config')" in launch
    assert (
        "str(share / 'config' / 'scan_vendor_planner.yaml'),\n"
        "            planner_config,\n"
        "            clearance_config,"
    ) in launch
    assert (
        "'planner_config',\n"
        "                default_value=str(\n"
        "                    share / 'config' / 'scan_vendor_planner.yaml'"
        in launch
    )


def test_m20_velocity_overrides_match_the_capability_envelope() -> None:
    planner = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'scan_m20_velocity_planner.yaml')
        .read_text(encoding='utf-8')
    )['scan_planner_node']['ros__parameters']
    controller = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'scan_m20_velocity_controller.yaml')
        .read_text(encoding='utf-8')
    )['closed_loop_controller']['ros__parameters']

    assert planner == {
        'manager.max_vel': 0.45,
        'optimization.max_vel': 0.45,
    }
    assert controller == {
        'max_vx': 0.45,
        'max_vy': 0.20,
        'max_vyaw': 0.65,
        'trajectory_progress_sync': True,
        'projection_samples': 60,
        'max_time_ahead': 0.20,
    }


def test_conservative_profile_keeps_vendor_scan_clearance() -> None:
    with (
        PACKAGE_ROOT / 'config' / 'clearance_conservative.yaml'
    ).open('r', encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'scan_planner_node'
        ]['ros__parameters']
    assert parameters == {
        'grid_map.double_cylinder_radius': 0.25,
        'grid_map.double_cylinder_offset': 0.18,
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


def test_native_scan_is_the_only_runtime_route() -> None:
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert 'm20_grid_route_planner' not in launch_text
    assert 'use_grid_route' not in launch_text
    assert "package='m20_scan_planner'" in launch_text
    assert "executable='scan_planner_node'" in launch_text
    assert "'scan_vendor_planner.yaml'" in launch_text
    assert "'scan_vendor_controller.yaml'" in launch_text
    assert "('move_base_simple/goal', '/move_base_simple/goal')" in launch_text
    assert "'direct_goal_topic': '/move_base_simple/goal'" in launch_text


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


def test_replan_state_keeps_upstream_goal_handling() -> None:
    source = (
        PLANNER_ROOT
        / 'src'
        / 'scan_replan_fsm.cpp'
    ).read_text(encoding='utf-8')
    assert 'target_reached_tolerance_' not in source
    assert 'changeFSMExecState(WAIT_TARGET, "GOAL_REACHED")' not in source


def test_typed_gateway_resets_native_scan_on_cancel() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_scan_navigation'
        / 'navigation_gateway_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/navigation/navigate'" in source
    assert 'self._scan_reset_client' in source
    assert 'self._route_reset_client' not in source
    assert 'floor.generation == goal.map_generation' in source
    assert 'self._floor_matches(goal)' in source


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
    assert "diagnostic == 'CLEAR'" in source
    assert "'WAITING_FOR_RECOVERY_REARM'" in source
    assert 'collision_replan_max_attempts' in source
    assert 'self._publish_goal(target)' in source
    assert 'goal republished to native SCAN' in source
    assert 'route_mode_active' not in source
    assert '_collision_route_publisher' not in source
    assert "self.declare_parameter('collision_guard_required', True)" in source
    assert 'guard_allows_motion(' in source


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
    assert "'TRACKING_ROUTE'" not in source
    assert 'self._publish_goal(target)' in source
    assert "'COLLISION_REPLAN_EXHAUSTED'" in source
    assert 'self._reset_navigation(' in source
    assert config['manual_goal_monitor_enabled'] is False
    assert config['manual_goal_topic'] == '/move_base_simple/goal'
    assert config['manual_timeout_sec'] == 300.0
    assert config['navigation_reset_timeout_sec'] == 10.0
    assert config['collision_guard_required'] is True
    assert config['terminal_orientation_mode'] == 'position_only'
    assert 'self._diagnostic_requires_replan()' in source


def test_integrated_scan_keeps_upstream_visualization_rate() -> None:
    planner_config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'scan_vendor_planner.yaml').read_text(
            encoding='utf-8'
        )
    )['/**']['ros__parameters']
    grid_map_source = (
        UPSTREAM_SCAN_ROOT
        / 'src'
        / 'planner'
        / 'plan_env'
        / 'src'
        / 'grid_map.cpp'
    ).read_text(encoding='utf-8')

    assert 'grid_map.visualization_rate_hz' not in planner_config
    assert 'grid_map.visualization_rate_hz' in grid_map_source
    assert 'visualization_rate_hz, 20.0' in grid_map_source


def test_guard_exposes_missing_safe_recovery_to_gateway() -> None:
    source = (
        SAFETY_ROOT
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "'RECOVERY_UNAVAILABLE; '" in source
    assert 'elif not self._recovery_exhausted_reason:' in source


def test_collision_guard_consumes_scan_online_occupancy() -> None:
    source = (
        SAFETY_ROOT
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "'occupancy_cloud_topic', '/grid_map/occupancy'" in source
    assert 'PointCloud2' in source
    assert 'rasterize_online_occupancy' in source
    assert 'OccupancyGrid' not in source
