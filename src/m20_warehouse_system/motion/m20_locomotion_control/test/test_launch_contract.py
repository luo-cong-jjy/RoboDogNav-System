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

"""Static contract checks for the SDK launch boundary."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_sdk_is_opt_in_until_a_joint_backend_exists():
    launch_text = (
        ROOT / 'launch' / 'sdk_locomotion.launch.py'
    ).read_text(encoding='utf-8')
    assert "default_value='false'" in launch_text
    assert "executable='rl_deploy_cmdvel'" in launch_text
    assert "'locomotion_capability_config'" in launch_text
    assert 'load_capability_profile' in launch_text
    assert 'profile.intent_parameters()' in launch_text
    assert 'profile.sdk_parameters()' in launch_text


def test_navigation_adapter_precedes_safety_and_backend_gate():
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    assert 'input_topic: /m20/navigation/cmd_vel_raw' in config_text
    assert (
        'output_topic: /m20/navigation/cmd_vel_candidate'
        in config_text
    )
    assert 'input_topic: /m20/control/cmd_vel_safe' in config_text
    assert 'output_topic: /m20/locomotion/cmd_vel_sdk' in config_text
    assert 'max_forward:' not in config_text
    assert 'turn_min_forward:' not in config_text


def test_autonomous_rolling_does_not_remove_manual_lateral_control():
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    assert 'safety_state_topic: /m20/control/safety_state' in config_text
    assert 'rolling_navigation_enabled: false' in config_text
    assert 'allow_manual_lateral: true' in config_text


def test_velocity_feedback_is_staged_before_safety_with_fail_open_timeout():
    config_text = (
        ROOT / 'config' / 'sdk_locomotion.yaml'
    ).read_text(encoding='utf-8')
    node_text = (
        ROOT
        / 'm20_locomotion_control'
        / 'navigation_adapter_node.py'
    ).read_text(encoding='utf-8')

    assert 'velocity_feedback_enabled: false' in config_text
    assert (
        'velocity_feedback_odometry_topic: /m20/sim/body_pose'
        in config_text
    )
    assert (
        'velocity_feedback_execution_hold_topic: '
        '/m20/control/execution_hold'
        in config_text
    )
    assert 'velocity_feedback_expected_child_frame: base_link' in config_text
    assert 'velocity_feedback_cruise_only: true' in config_text
    assert (
        'velocity_feedback_max_abs_yaw_reference: 0.08'
        in config_text
    )
    assert "reason = 'MANEUVER_GATED'" in node_text
    assert 'ODOMETRY_STALE' in node_text
    assert 'ODOMETRY_FRAME_MISMATCH' in node_text
    assert 'EXECUTION_HOLD' in node_text
    assert 'feedback_result.output' in node_text


def test_factory_backends_are_guarded_behind_the_common_twist_contract():
    config_text = (
        ROOT / 'config' / 'm20_factory_basic_server.yaml'
    ).read_text(encoding='utf-8')
    launch_text = (
        ROOT / 'launch' / 'official_locomotion.launch.py'
    ).read_text(encoding='utf-8')
    direct_text = (
        ROOT / 'config' / 'm20_factory_direct_ros.yaml'
    ).read_text(encoding='utf-8')

    assert 'input_topic: /m20/control/cmd_vel_safe' in config_text
    assert 'output_topic: /m20/locomotion/cmd_vel_sdk' in config_text
    assert 'input_topic: /m20/locomotion/cmd_vel_sdk' in config_text
    assert 'command_rate_hz: 20.0' in config_text
    assert 'command_timeout_sec: 0.30' in config_text
    assert 'auto_enable_motion: false' in config_text
    assert 'command_ownership_confirmed: false' in config_text
    assert "default_value='false'" in launch_text
    assert 'm20_factory_agile_flat_capabilities.yaml' in launch_text
    assert "{'basic_server', 'direct_ros'}" in launch_text
    assert "else 'm20_direct_ros_backend'" in launch_text
    assert 'nav_cmd_topic: /NAV_CMD' in direct_text
    assert 'motion_info_topic: /MOTION_INFO' in direct_text
    assert 'motion_state_topic: /MOTION_STATE' in direct_text
    assert 'gait_topic: /GAIT' in direct_text
    assert 'command_rate_hz: 20.0' in direct_text
    assert 'command_timeout_sec: 0.30' in direct_text
    assert 'auto_enable_motion: false' in direct_text
    assert 'command_ownership_confirmed: false' in direct_text


def test_feedback_can_use_factory_motion_status_without_fake_odometry():
    node_text = (
        ROOT
        / 'm20_locomotion_control'
        / 'navigation_adapter_node.py'
    ).read_text(encoding='utf-8')

    assert "'velocity_feedback_source', 'odometry'" in node_text
    assert 'velocity_feedback_twist_topic' in node_text
    assert "{'odometry', 'twist'}" in node_text
    assert 'def _twist_callback' in node_text
