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
    assert 'mjtCamera.mjCAMERA_FREE' in source
    assert 'def _sync_viewer(' in source
    assert 'self._viewer_step_interval = max(' in source
    assert 'self._last_viewer_sync_step = self._steps' in source
    assert 'self._viewer.cam.lookat[:]' in source
    assert "'/m20/locomotion/mode'" in source
    assert 'apply_wheel_brake(' in source
    assert 'world_vector_to_body(' in source
    assert "'base_linear_velocity_world'" in source
    assert "'base_linear_velocity_body'" in source
    assert "'base_angular_velocity_body'" in source


def test_world_launch_uses_selected_system_configuration():
    source = (
        ROOT / 'launch' / 'mujoco_backend.launch.py'
    ).read_text(encoding='utf-8')
    assert "LaunchConfiguration('system_config')" in source
    assert 'generate_world(' in source
    assert "'viewer_max_fps': LaunchConfiguration(" in source
    assert "'viewer_max_fps'" in source
    assert "executable='m20_mujoco_viewer'" in source
    assert "'use_viewer': False" in source
    assert "prefix='nice -n 10'" in source


def test_viewer_is_a_read_only_ros_state_replica():
    source = (
        ROOT / 'm20_mujoco_backend' / 'viewer_node.py'
    ).read_text(encoding='utf-8')
    assert "'/m20/sim/body_pose'" in source
    assert "'/joint_states'" in source
    assert 'mujoco.mj_step(' not in source
    assert 'mujoco.mj_forward(' in source
    assert 'mjtCamera.mjCAMERA_FREE' in source
    assert "self.declare_parameter('low_cost_render', True)" in source
    assert 'light_castshadow[:]' in source
    assert 'glfw.swap_buffers(' in source
    assert 'mujoco.mjv_updateScene(' in source
    assert 'mujoco.mjr_render(' in source
    assert 'launch_passive(' not in source
    assert 'GUI timing cannot block physics' in source
