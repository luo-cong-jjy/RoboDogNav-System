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

"""Guard the phase-3 atomic floor-switch integration contract."""

from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


def _text(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8')


def test_phase3_launch_uses_vendor_renderer_and_starts_manager() -> None:
    launch = _text('launch/multifloor_scan_rviz.launch.py')
    assert "package='m20_multifloor_map'" in launch
    assert "executable='m20_vendor_sensing_state'" in launch
    assert "'use_local_sensing': 'true'" in launch
    assert "get_package_share_directory('m20_scan_planner')" in launch
    assert "scan_vendor / 'rviz' / 'default.rviz'" in launch
    assert "executable='m20_floor_switch_manager'" in launch
    assert 'collision_guard_scan_native.yaml' in launch
    assert "condition=UnlessCondition(use_grid_route)" in launch
    assert "'collision_grid_route_enabled': (" in launch


def test_map_server_exposes_generation_compare_and_swap() -> None:
    server = _text('m20_warehouse_inspection/map_server_node.py')
    assert "SwitchMap, '/m20/map/switch'" in server
    assert 'expected_current_generation != self._generation' in server
    assert "'LOADING'" in server
    assert "'READY'" in server


def test_floor_transaction_is_fail_closed_and_resets_scan_twice() -> None:
    manager = (
        SOURCE_ROOT
        / 'm20_inspection_core'
        / 'm20_inspection_core'
        / 'floor_switch_manager_node.py'
    ).read_text(encoding='utf-8')
    assert manager.count('self._call_service(') >= 4
    assert manager.count('self._reset_client') >= 3
    assert "self._publish_hold(True)" in manager
    assert "self._publish_hold(False)" in manager
    success_releases = re.findall(
        r'self\._publish_hold\(False\)\s+goal_handle\.succeed\(\)',
        manager,
    )
    # One additional false is the safe node-start default. Every later
    # release is coupled directly to an Action success terminal.
    assert len(success_releases) == 2
    assert manager.count('self._publish_hold(False)') == 3


def test_scan_reset_clears_grid_and_active_waypoints() -> None:
    scan_fsm = (
        SOURCE_ROOT
        / 'm20_scan_planner'
        / 'src'
        / 'scan_replan_fsm.cpp'
    ).read_text(encoding='utf-8')
    assert 'grid_map_->resetBuffer()' in scan_fsm
    assert 'active_waypoints_.clear()' in scan_fsm
    assert 'changeFSMExecState(WAIT_TARGET, "FLOOR_RESET")' in scan_fsm


def test_safety_hold_freezes_execution_and_resets_stale_derivatives() -> None:
    controller = (
        SOURCE_ROOT
        / 'm20_scan_planner'
        / 'src'
        / 'closed_loop_controller.cpp'
    ).read_text(encoding='utf-8')
    scan_fsm = (
        SOURCE_ROOT
        / 'm20_scan_planner'
        / 'src'
        / 'scan_replan_fsm.cpp'
    ).read_text(encoding='utf-8')
    assert 'execution_hold_topic' in controller
    assert 'if (!external_execution_hold_)' in controller
    assert 'reset_start_state_after_hold_ = true' in scan_fsm
    assert 'start_vel_.setZero()' in scan_fsm
