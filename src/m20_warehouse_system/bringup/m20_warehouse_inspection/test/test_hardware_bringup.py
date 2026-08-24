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

"""Static fail-closed contracts for the physical M20 bring-up graph."""

from pathlib import Path
import pytest


ROOT = Path(__file__).parents[1]
SYSTEM_ROOT = ROOT.parents[1]
SOURCE_ROOT = SYSTEM_ROOT.parent


def test_hardware_launch_replaces_only_data_and_execution_backends():
    text = (
        ROOT / 'launch' / 'inspection_mission_hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert "'motion_backend': 'external'" in text
    assert "'execution_profile': 'm20_safe'" in text
    assert "'use_local_sensing': 'false'" in text
    assert "default_value='/m20/localization/body_pose'" in text
    assert "default_value='/m20/localization/cloud'" in text
    assert "default_value='/m20/localization/sensor_pose'" in text
    assert "default_value='/LIO/odom_vehicle'" in text
    assert 'm20_factory_agile_flat_capabilities.yaml' in text
    assert "'velocity_feedback_source': 'twist'" in text
    assert '/m20/locomotion/measured_twist' in text
    assert 'official_locomotion.launch.py' in text
    assert 'm20_hardware_localization_adapter' in text
    assert "FindPackageShare('lio')" in text
    assert "default_value='root_config_m20_navigation_relocation.yaml'" in text
    assert "integration / 'config' / 'sites'" in text
    assert "'m20_pao_f1_template.yaml'" in text
    assert 'if not default_system.is_file()' in text


def test_hardware_motion_cannot_start_by_default():
    text = (
        ROOT / 'launch' / 'inspection_mission_hardware.launch.py'
    ).read_text(encoding='utf-8')

    assert (
        "'command_ownership_confirmed', default_value='false'" in text
    )
    assert "'auto_enable_motion', default_value='false'" in text
    assert 'SetSimulationPose' in text


def test_hardware_lio_profile_uses_the_navigation_frame_contract():
    root = SOURCE_ROOT / 'Elevator-LIO' / 'yaml'
    if not root.exists():
        pytest.skip('Elevator-LIO source is not part of this simulation checkout')
    root_config = (
        root / 'root_config_m20.yaml'
    ).read_text(encoding='utf-8')
    sensor_config = (
        root / 'sensors' / 'robosense_m20.yaml'
    ).read_text(encoding='utf-8')

    assert 'sensors/robosense_m20.yaml' in root_config
    assert 'world_frame_name: "lio_world"' in sensor_config
    assert 'body_frame_name: "lio_base_link"' in sensor_config
    assert 'lidar_frame_name: "lio_base_link"' in sensor_config
    assert '"/rslidar_points_front"' in sensor_config
    assert '"/rslidar_points_rear"' in sensor_config


def test_direct_ros_transport_is_selectable_with_guarded_high_level_drdds():
    launch_text = (
        SYSTEM_ROOT
        / 'motion'
        / 'm20_locomotion_control'
        / 'launch'
        / 'official_locomotion.launch.py'
    ).read_text(encoding='utf-8')

    direct_node = (
        SYSTEM_ROOT
        / 'motion'
        / 'm20_locomotion_control'
        / 'm20_locomotion_control'
        / 'direct_ros_backend_node.py'
    ).read_text(encoding='utf-8')

    assert "{'basic_server', 'direct_ros'}" in launch_text
    assert 'm20_direct_ros_backend' in launch_text
    assert 'from drdds.msg import Gait, MotionInfo, MotionState, NavCmd' in (
        direct_node
    )
    assert 'StdMsgInt32' in direct_node
    assert "'hard_estop_topic', '/HES_STATUS'" in direct_node
    assert 'reliability=ReliabilityPolicy.RELIABLE' in direct_node
    assert 'self._hard_estop_callback,\n            latched_qos' in direct_node
    assert "'command_ownership_confirmed', False" in direct_node
    assert "'auto_enable_motion', False" in direct_node
