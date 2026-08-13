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
    proportional_ramp_command,
    select_fresh_command,
    slew_command,
    valid_collision_recovery_command,
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


def test_recovery_ramp_preserves_command_curvature() -> None:
    target = (0.35, 0.0, 0.65)
    output = proportional_ramp_command(
        current=(0.0, 0.0, 0.0),
        target=target,
        dt=0.2,
        ramp_duration=1.0,
    )

    assert output == pytest.approx((0.07, 0.0, 0.13))
    assert output[0] / output[2] == pytest.approx(
        target[0] / target[2]
    )


def test_recovery_ramp_restarts_when_turn_direction_changes() -> None:
    output = proportional_ramp_command(
        current=(0.175, 0.0, 0.325),
        target=(0.35, 0.0, -0.65),
        dt=0.1,
        ramp_duration=1.0,
    )

    assert output == pytest.approx((0.035, 0.0, -0.065))


@pytest.mark.parametrize(
    ('command', 'expected'),
    (
        ((0.35, 0.0, 0.65), True),
        ((-0.35, 0.0, 0.0), True),
        ((0.35, 0.0, 0.0), True),
        ((-0.35, 0.0, 0.35), False),
        ((0.0, 0.0, 0.65), False),
        ((0.35, 0.01, 0.65), False),
        ((0.0, 0.0, 0.0), False),
    ),
)
def test_collision_recovery_command_shape_is_restricted(
    command: tuple[float, float, float],
    expected: bool,
) -> None:
    assert valid_collision_recovery_command(command) is expected
