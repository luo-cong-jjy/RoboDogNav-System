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

"""Static checks for the full-system MuJoCo interface contract."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_backend_owns_current_scan_pose_and_sdk_topics():
    source = (
        ROOT / 'm20_mujoco_backend' / 'backend_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/sim/body_pose'" in source
    assert "'/quad_0/path'" in source
    assert "'/JOINTS_CMD'" in source
    assert "'/JOINTS_DATA'" in source
    assert "'/IMU_DATA'" in source
    assert 'mjtCamera.mjCAMERA_TRACKING' in source
    assert "'/m20/locomotion/mode'" in source
    assert 'apply_wheel_brake(' in source


def test_world_launch_uses_selected_system_configuration():
    source = (
        ROOT / 'launch' / 'mujoco_backend.launch.py'
    ).read_text(encoding='utf-8')
    assert "LaunchConfiguration('system_config')" in source
    assert 'generate_world(' in source
