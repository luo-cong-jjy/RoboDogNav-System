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

"""Tests for command arbitration and limiting."""

import pytest

from m20_inspection_core.safety_policy import (
    TimedCommand,
    clamp_command,
    select_fresh_command,
    slew_command,
)


def test_limits_remove_excess_planar_speed() -> None:
    assert clamp_command((2.0, -1.0, 4.0), (0.45, 0.2, 0.65)) == (
        0.45,
        -0.2,
        0.65,
    )


def test_manual_command_has_priority_when_fresh() -> None:
    command, source = select_fresh_command(
        now=10.0,
        timeout=0.5,
        navigation=TimedCommand((0.4, 0.0, 0.0), 9.9),
        manual=TimedCommand((0.0, 0.1, 0.0), 9.8),
        manual_priority=True,
    )
    assert source == 'MANUAL'
    assert command == (0.0, 0.1, 0.0)


def test_stale_commands_fail_to_zero() -> None:
    command, source = select_fresh_command(
        now=10.0,
        timeout=0.5,
        navigation=TimedCommand((0.4, 0.0, 0.0), 8.0),
        manual=None,
        manual_priority=True,
    )
    assert source == 'COMMAND_TIMEOUT'
    assert command == (0.0, 0.0, 0.0)


def test_ordinary_output_respects_acceleration_limits() -> None:
    output = slew_command(
        current=(0.0, 0.0, 0.0),
        target=(0.45, -0.2, 0.65),
        dt=0.1,
        linear_acceleration=1.0,
        angular_acceleration=1.2,
    )
    assert output == pytest.approx((0.1, -0.1, 0.12))
