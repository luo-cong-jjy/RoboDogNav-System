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

"""Tests for the explicit factory ROS 2 motion transition contract."""

from types import SimpleNamespace

from m20_locomotion_control.direct_ros_policy import (
    decode_motion_info,
    DirectMotionStatus,
    select_direct_transition,
)


def test_current_deep_robotics_motion_info_nested_abi_is_decoded():
    data = SimpleNamespace(
        vel_x=0.21,
        vel_y=-0.04,
        vel_yaw=0.33,
        motion_state=SimpleNamespace(state=17),
        gait_state=SimpleNamespace(gait=0x3002),
    )

    status = decode_motion_info(data)

    assert status.state == 17
    assert status.gait == 0x3002
    assert status.linear_x == 0.21
    assert status.linear_y == -0.04
    assert status.angular_z == 0.33


def _status(state, gait=0x1001, velocity=0.0):
    return DirectMotionStatus(
        state=state,
        gait=gait,
        linear_x=velocity,
        linear_y=0.0,
        angular_z=0.0,
    )


def test_enable_explicitly_advances_stand_then_rl_control():
    stand = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(4),
        requested_gait=0x3002,
    )
    rl = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(1),
        requested_gait=0x3002,
    )
    assert stand.state_command == 1
    assert rl.state_command == 17


def test_gait_change_waits_until_motion_info_reports_stationary():
    moving = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(17, velocity=0.10),
        requested_gait=0x3002,
    )
    stopped = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(17),
        requested_gait=0x3002,
    )
    assert moving.gait_command is None
    assert not moving.ready
    assert stopped.gait_command == 0x3002


def test_ready_requires_rl_control_and_requested_gait():
    result = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=False,
        status=_status(17, gait=0x3002),
        requested_gait=0x3002,
    )
    assert result.ready
    assert not result.fault


def test_estop_and_ownership_are_fail_closed():
    ownership = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=False,
        e_stop=False,
        status=_status(17, gait=0x3002),
        requested_gait=0x3002,
    )
    estop = select_direct_transition(
        motion_requested=True,
        ownership_confirmed=True,
        e_stop=True,
        status=_status(17, gait=0x3002),
        requested_gait=0x3002,
    )
    assert ownership.fault == 'COMMAND_OWNERSHIP_NOT_CONFIRMED'
    assert estop.fault == 'E_STOP_ASSERTED'
