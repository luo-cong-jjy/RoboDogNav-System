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

"""Guard the complete MuJoCo/SDK warehouse composition contract."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
SYSTEM_ROOT = ROOT.parents[1]


def test_complete_launch_composes_scan_sdk_and_physics():
    launch = (
        ROOT / 'launch' / 'inspection_mission_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    assert 'inspection_mission_rviz.launch.py' in launch
    assert 'mujoco_backend.launch.py' in launch
    assert 'sdk_locomotion.launch.py' in launch
    assert "'motion_backend': 'external'" in launch
    assert "'start_sdk': 'true'" in launch
    assert "'require_backend_ready': 'true'" in launch
    assert 'dense_four_corner_system.yaml' in launch
    assert 'clearance_m20.yaml' in launch
    assert 'scan_m20_physical_planner.yaml' not in launch
    assert 'planner_config' not in launch
    assert "default_value='300.0'" in launch
    assert (
        "'use_mujoco_viewer',\n                default_value='true'"
        in launch
    )
    assert "'viewer_distance': mujoco_viewer_distance" in launch
    assert "'mujoco_viewer_elevation'," in launch
    assert "'viewer_elevation': mujoco_viewer_elevation" in launch
    assert "'viewer_max_fps': mujoco_viewer_max_fps" in launch
    assert "'navigation_timeout_sec': navigation_timeout_sec" in launch
    assert (
        "'velocity_feedback_enabled': velocity_feedback_enabled"
        in launch
    )
    assert "'locomotion_capability_config'" in launch
    assert 'm20_policy_v1_capabilities.yaml' in launch


def test_rviz_launch_keeps_its_compatible_default_backend():
    launch = (
        ROOT / 'launch' / 'multifloor_scan_rviz.launch.py'
    ).read_text(encoding='utf-8')
    assert "'motion_backend'," in launch
    assert "default_value='rviz'" in launch
    assert 'PythonExpression(' in launch
    assert '"\' == \'rviz\'"' in launch
    assert (
        "'velocity_feedback_enabled': ParameterValue("
        in launch
    )
    assert 'capability_profile.intent_parameters()' in launch
    assert 'capability_profile.controller_parameters()' in launch
    assert 'capability_profile.collision_guard_parameters()' in launch
    assert 'capability_profile.safety_parameters()' in launch
    assert "default_value='scan_native'" in launch
    assert "'max_linear_x': 0.75" in launch
    assert "'max_linear_y': 0.35" in launch
    assert "'max_angular_z': 1.0" in launch
    assert "'collision_guard_enabled': False" in launch
    assert "'collision_guard_required': str(" in launch


def test_velocity_feedback_is_an_explicit_disabled_by_default_ab_switch():
    mujoco_launch = (
        ROOT / 'launch' / 'inspection_mission_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    rviz_launch = (
        ROOT / 'launch' / 'inspection_mission_rviz.launch.py'
    ).read_text(encoding='utf-8')

    assert "'velocity_feedback_enabled'," in mujoco_launch
    assert "default_value='false'" in mujoco_launch
    assert (
        "'velocity_feedback_enabled': velocity_feedback_enabled"
        in rviz_launch
    )


def test_mujoco_world_uses_map_metadata_not_a_duplicate_scene():
    source = (
        SYSTEM_ROOT
        / 'simulation'
        / 'm20_mujoco_backend'
        / 'm20_mujoco_backend'
        / 'world_generator.py'
    ).read_text(encoding='utf-8')
    assert "floor_config['metadata_file']" in source
    assert "metadata.get('obstacles', [])" in source
    assert '_wall_segments(bounds, gateway)' in source


def test_backend_waits_for_stable_stand_before_system_pose():
    source = (
        SYSTEM_ROOT
        / 'simulation'
        / 'm20_mujoco_backend'
        / 'm20_mujoco_backend'
        / 'backend_node.py'
    ).read_text(encoding='utf-8')
    assert 'def _update_readiness(' in source
    assert 'self._standing_ready_height' in source
    assert 'if not self._ready:' in source
    assert "'/JOINTS_CMD'" in source


def test_motion_envelope_bypasses_scan_and_project_adapter():
    launch = (
        ROOT / 'launch' / 'm20_motion_envelope_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    assert "executable='rl_deploy_cmdvel'" in launch
    assert "executable='m20_command_envelope_probe'" in launch
    assert 'mujoco_backend.launch.py' in launch
    assert 'inspection_mission_mujoco.launch.py' not in launch
    assert 'm20_locomotion_manager' not in launch
    assert 'm20_scan_planner' not in launch


def test_motion_envelope_scene_is_obstacle_free():
    metadata = (
        ROOT
        / 'maps'
        / 'motion_envelope'
        / 'scene_1'
        / 'scene_1.json'
    ).read_text(encoding='utf-8')
    assert '"obstacle_count": 0' in metadata
    assert '"obstacles": []' in metadata


def test_navigation_probe_aborts_and_records_backend_faults():
    probe = (
        ROOT / 'tools' / 'navigation_motion_probe.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/sim/backend_fault'" in probe
    assert 'if node.backend_fault:' in probe
    assert "'backend_fault_reasons'" in probe


def test_navigation_probe_records_velocity_feedback_ab_metrics():
    probe = (
        ROOT / 'tools' / 'navigation_motion_probe.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/navigation/velocity_feedback_state'" in probe
    assert "'velocity_tracking_linear_error_rms_m_s'" in probe
    assert "'velocity_tracking_yaw_error_rms_rad_s'" in probe
    assert "'velocity_feedback_active_sample_fraction'" in probe
    assert "'schema_version': 6" in probe
