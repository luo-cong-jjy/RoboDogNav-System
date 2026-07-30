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

"""Guard the phase-2 integration and command safety boundaries."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


def _launch_text() -> str:
    return (
        PACKAGE_ROOT / 'launch' / 'f1_scan_rviz.launch.py'
    ).read_text(encoding='utf-8')


def test_complete_f1_graph_uses_all_isolated_project_packages() -> None:
    launch = _launch_text()
    for package in (
        'm20_official_description',
        'm20_scan_navigation',
        'm20_warehouse_sim',
        'm20_inspection_core',
        'm20_locomotion_control',
    ):
        assert package in launch
    scan_launch = (
        PACKAGE_ROOT.parent
        / 'm20_scan_navigation'
        / 'launch'
        / 'f1_scan.launch.py'
    ).read_text(encoding='utf-8')
    assert "package='m20_scan_planner'" in scan_launch


def test_backend_is_downstream_of_safety_supervisor() -> None:
    sim_config = (
        PACKAGE_ROOT.parent
        / 'm20_warehouse_sim'
        / 'config'
        / 'rviz_kinematic.yaml'
    ).read_text(encoding='utf-8')
    safety_config = (
        PACKAGE_ROOT.parent
        / 'm20_inspection_core'
        / 'config'
        / 'safety.yaml'
    ).read_text(encoding='utf-8')
    assert 'safe_cmd_topic: /m20/control/cmd_vel_safe' in sim_config
    assert 'safe_topic: /m20/control/cmd_vel_safe' in safety_config
    assert (
        'navigation_topic: /m20/navigation/cmd_vel_candidate'
        in safety_config
    )
    assert 'collision_stop_topic: /m20/control/collision_stop' in safety_config
    assert '/cmd_vel\n' not in sim_config
    launch = _launch_text()
    assert "executable='m20_navigation_adapter'" in launch


def test_integrated_system_uses_m20_heading_tracking_override() -> None:
    launch = (
        PACKAGE_ROOT / 'launch' / 'multifloor_scan_rviz.launch.py'
    ).read_text(encoding='utf-8')
    assert 'scan_m20_physical_controller.yaml' in launch
    assert "'controller_config': controller_config" in launch
    controller = (
        SOURCE_ROOT
        / 'm20_scan_planner'
        / 'src'
        / 'closed_loop_controller.cpp'
    ).read_text(encoding='utf-8')
    assert 'heading_error_resume_threshold' in controller
    assert 'heading_slowdown_threshold' in controller
    assert 'heading_alignment_min_hold_sec' in controller
    assert 'if (heading_aligning_)' in controller
    assert 'vel_world *= headingTranslationScale' in controller


def test_collision_guard_profile_follows_navigation_profile() -> None:
    launch = _launch_text()
    assert 'collision_guard_scan_native.yaml' in launch
    assert "condition=UnlessCondition(use_grid_route)" in launch
    assert "condition=IfCondition(use_grid_route)" in launch
    with (
        SOURCE_ROOT
        / 'm20_inspection_core'
        / 'config'
        / 'collision_guard_scan_native.yaml'
    ).open('r', encoding='utf-8') as stream:
        native = yaml.safe_load(stream)[
            'm20_collision_guard'
        ]['ros__parameters']
    assert native['footprint_radius'] == 0.25
    assert native['footprint_offset'] == 0.18
    assert native['safety_margin'] == 0.05
    assert (
        native['command_topic']
        == '/m20/navigation/cmd_vel_candidate'
    )


def test_scan_world_frame_is_explicitly_connected_to_map() -> None:
    launch = _launch_text()
    assert "'--frame-id', 'map'" in launch
    assert "'--child-frame-id', 'world'" in launch


def test_system_navigation_contract_matches_phase2_graph() -> None:
    with (
        PACKAGE_ROOT / 'config' / 'flat_multifloor_system.yaml'
    ).open('r', encoding='utf-8') as stream:
        navigation = yaml.safe_load(stream)['navigation']
    assert navigation['scan_navigation_mode'] == 1
    assert navigation['global_route_backend'] == 'scan_native'
    assert navigation['optional_global_route_backend'] == 'grid_astar'
    assert navigation['goal_topic'] == '/move_base_simple/goal'
    assert navigation['routed_goal_topic'] == '/m20/navigation/goal_pose'
    assert (
        navigation['global_route_topic']
        == '/m20/navigation/global_route'
    )
    assert navigation['scan_subgoal_topic'] == '/move_base_simple/goal'
    assert navigation['local_cloud_topic'] == '/quad_0/cloud'
    assert navigation['sensor_cloud_topic'] == '/quad_0/sensor_cloud'


def test_rviz_restores_scan_debug_layers_and_live_sensor_cloud() -> None:
    rviz = (
        SOURCE_ROOT / 'm20_scan_planner' / 'launch' / 'default.rviz'
    ).read_text(encoding='utf-8')
    for topic in (
        '/map_generator/global_cloud',
        '/quad_0/cloud',
        '/grid_map/occupancy',
        '/grid_map/occupancy_inflate',
        '/grid_map/sliding_map_bbox',
        '/goal_point',
        '/optimal_list',
        '/move_base_simple/goal',
        '/m20/visualization/inactive_floors_cloud',
    ):
        assert topic in rviz
    assert (
        'Name: Sensor Cloud' in rviz
        and 'Value: /quad_0/cloud' in rviz
    )
    assert 'Name: M20' in rviz
    assert 'Name: Copied Scene (Inactive Floor)' in rviz
    assert 'Hide Left Dock: false' in rviz


def test_scan_pointcloud_and_path_styles_match_upstream_rviz() -> None:
    with (
        SOURCE_ROOT / 'm20_scan_planner' / 'launch' / 'default.rviz'
    ).open('r', encoding='utf-8') as stream:
        integrated = yaml.safe_load(stream)
    upstream_path = (
        SOURCE_ROOT
        / 'third_party'
        / 'SCAN-Planner'
        / 'src'
        / 'planner'
        / 'plan_manage'
        / 'launch'
        / 'default.rviz'
    )
    with upstream_path.open('r', encoding='utf-8') as stream:
        upstream = yaml.safe_load(stream)

    def displays(config):
        return {
            item['Name']: item
            for item in config['Visualization Manager']['Displays']
        }

    current = displays(integrated)
    reference = displays(upstream)
    assert current['Sensor Cloud']['Topic']['Value'] == '/quad_0/cloud'
    names = {
        'Global Map': 'Global Map',
        'Sensor Cloud': 'Sensor Cloud',
        'Occupancy': 'Occupancy',
        'Inflated Occupancy': 'Inflated Occupancy',
        'Robot Path': 'Robot Path',
    }
    style_keys = (
        'Alpha',
        'Class',
        'Color',
        'Color Transformer',
        'Decay Time',
        'Line Style',
        'Line Width',
        'Size (Pixels)',
        'Size (m)',
        'Style',
    )
    for current_name, reference_name in names.items():
        for key in style_keys:
            if key in reference[reference_name]:
                assert (
                    current[current_name][key]
                    == reference[reference_name][key]
                )
