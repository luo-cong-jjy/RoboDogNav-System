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
SOURCE = ROOT.parent


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
    assert 'clearance_conservative.yaml' in launch
    assert "default_value='300.0'" in launch
    assert "'use_mujoco_viewer',\n                default_value='true'" in launch
    assert "'navigation_timeout_sec': navigation_timeout_sec" in launch


def test_rviz_launch_keeps_its_compatible_default_backend():
    launch = (
        ROOT / 'launch' / 'multifloor_scan_rviz.launch.py'
    ).read_text(encoding='utf-8')
    assert "'motion_backend'," in launch
    assert "default_value='rviz'" in launch
    assert 'PythonExpression(' in launch
    assert '"\' == \'rviz\'"' in launch


def test_mujoco_world_uses_map_metadata_not_a_duplicate_scene():
    source = (
        SOURCE
        / 'm20_mujoco_backend'
        / 'm20_mujoco_backend'
        / 'world_generator.py'
    ).read_text(encoding='utf-8')
    assert "floor_config['metadata_file']" in source
    assert "metadata.get('obstacles', [])" in source
    assert '_wall_segments(bounds, gateway)' in source


def test_backend_waits_for_stable_stand_before_system_pose():
    source = (
        SOURCE
        / 'm20_mujoco_backend'
        / 'm20_mujoco_backend'
        / 'backend_node.py'
    ).read_text(encoding='utf-8')
    assert 'def _update_readiness(' in source
    assert 'self._standing_ready_height' in source
    assert 'if not self._ready:' in source
    assert "'/JOINTS_CMD'" in source
