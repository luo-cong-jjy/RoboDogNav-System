# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Guard the phase-5 runtime regression composition contract."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_phase5_launch_composes_production_graph_and_regression() -> None:
    launch = (
        PACKAGE_ROOT / 'launch' / 'phase5_regression.launch.py'
    ).read_text(encoding='utf-8')
    assert 'inspection_mission_rviz.launch.py' in launch
    assert "executable='m20_phase5_regression'" in launch
    assert "'switch_iterations', default_value='20'" in launch
    assert "'mission_runs', default_value='10'" in launch
    assert "'memory_growth_limit_mib', default_value='128.0'" in launch
    assert "'memory_slope_limit_mib', default_value='8.0'" in launch
    assert "'system_config': system_config" in launch
    assert "'config_path': system_config" in launch
    assert 'OnProcessExit' in launch
    assert "reason='phase-5 regression finished'" in launch


def test_regression_has_three_bounded_modes() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_warehouse_inspection'
        / 'phase5_regression_node.py'
    ).read_text(encoding='utf-8')
    assert "self.mode == 'switch_stress'" in source
    assert "self.mode == 'mission_stress'" in source
    assert "self.mode == 'fault_injection'" in source
    assert 'memory_growth_is_bounded' in source
    assert 'steady_samples = samples[1:]' in source
    assert "len(self.config['mission']['sequence'])" in source


def test_switch_stress_checks_generation_sensing_hold_and_pose() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_warehouse_inspection'
        / 'phase5_regression_node.py'
    ).read_text(encoding='utf-8')
    assert 'self.safe_hold_violations' in source
    assert 'self.floor_hold_assertions != hold_count + 1' in source
    assert 'self.floor_zero_windows != zero_windows + 1' in source
    assert 'self.sensing.generation == target_generation' in source
    assert 'expected_generation(initial_generation, index + 1)' in source


def test_fault_suite_covers_rejection_cancel_stop_and_retry() -> None:
    source = (
        PACKAGE_ROOT
        / 'm20_warehouse_inspection'
        / 'phase5_regression_node.py'
    ).read_text(encoding='utf-8')
    assert 'ERROR_NOT_AT_TRIGGER' in source
    assert 'ERROR_CANCELLED' in source
    assert 'ERROR_STOPPED' in source
    assert "self.mission.state == 'FAULT_HOLD'" in source
    assert 'ControlMission.Request.RETRY_CURRENT' in source
