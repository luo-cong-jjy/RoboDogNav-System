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

"""Unit tests for the simulator-independent M20 motion-intent policy."""

import math

from m20_locomotion_control.motion_intent import (
    IntentParameters,
    MotionIntent,
    RollingNavigationAdapter,
    constrain_for_intent,
)


def test_zero_command_is_stopped():
    intent, command = constrain_for_intent(
        (0.0, 0.0, 0.0),
        IntentParameters(),
    )
    assert intent is MotionIntent.STOPPED
    assert command == (0.0, 0.0, 0.0)


def test_straight_command_uses_cruise_and_suppresses_small_side_motion():
    intent, command = constrain_for_intent(
        (0.35, 0.03, 0.05),
        IntentParameters(),
    )
    assert intent is MotionIntent.WHEEL_CRUISE
    assert command == (0.35, 0.0, 0.05)


def test_large_curvature_uses_coordinated_turn_and_limits_translation():
    intent, command = constrain_for_intent(
        (0.30, 0.0, 0.60),
        IntentParameters(),
    )
    assert intent is MotionIntent.COORDINATED_TURN
    assert command == (0.12, 0.0, 0.60)


def test_near_in_place_yaw_uses_coordinated_turn():
    intent, command = constrain_for_intent(
        (0.04, 0.0, -0.50),
        IntentParameters(),
    )
    assert intent is MotionIntent.COORDINATED_TURN
    assert command == (0.04, 0.0, -0.50)


def test_lateral_maneuver_limits_forward_motion():
    intent, command = constrain_for_intent(
        (0.40, 0.10, 0.10),
        IntentParameters(),
    )
    assert intent is MotionIntent.LATERAL_MANEUVER
    assert command == (0.10, 0.10, 0.10)


def test_command_envelope_never_increases_components():
    requested = (0.90, -0.60, 1.40)
    _, command = constrain_for_intent(requested, IntentParameters())
    for output, input_value in zip(command, requested):
        assert abs(output) <= abs(input_value)


def test_non_finite_input_fails_to_zero_for_that_component():
    _, command = constrain_for_intent(
        (math.nan, math.inf, -math.inf),
        IntentParameters(),
    )
    assert command == (0.0, 0.0, 0.0)


def test_navigation_adapter_converts_side_error_to_smooth_yaw():
    adapter = RollingNavigationAdapter(IntentParameters())
    intent, command = adapter.update((0.30, 0.03, 0.05), dt=0.10)

    assert intent is MotionIntent.WHEEL_CRUISE
    assert command[0] == 0.10
    assert command[1] == 0.0
    assert 0.0 < command[2] < 0.10


def test_navigation_adapter_enters_turn_before_translating_fast():
    parameters = IntentParameters()
    adapter = RollingNavigationAdapter(parameters)
    intent, command = adapter.update((0.30, 0.20, 0.0), dt=1.0)

    assert intent is MotionIntent.COORDINATED_TURN
    assert adapter.turning
    assert command[0] == parameters.turn_max_forward
    assert command[1] == 0.0
    assert command[2] > parameters.turn_yaw_threshold


def test_navigation_adapter_turn_hysteresis_prevents_mode_chatter():
    adapter = RollingNavigationAdapter(IntentParameters())
    adapter.update((0.30, 0.10, 0.0), dt=1.0)

    intent, _ = adapter.update((0.30, 0.04, 0.0), dt=1.0)
    assert intent is MotionIntent.COORDINATED_TURN
    assert adapter.turning

    intent, _ = adapter.update((0.30, 0.0, 0.02), dt=1.0)
    assert intent is MotionIntent.WHEEL_CRUISE
    assert not adapter.turning


def test_navigation_adapter_turn_holds_before_aligned_release():
    parameters = IntentParameters(turn_min_hold_sec=0.30)
    adapter = RollingNavigationAdapter(parameters)
    adapter.update((0.30, 0.20, 0.0), dt=0.02)

    intent, _ = adapter.update((0.30, 0.0, 0.02), dt=0.10)
    assert intent is MotionIntent.COORDINATED_TURN
    assert adapter.turning

    intent, _ = adapter.update((0.30, 0.0, 0.02), dt=0.20)
    assert intent is MotionIntent.WHEEL_CRUISE
    assert not adapter.turning


def test_navigation_adapter_pure_turn_stops_translation_immediately():
    adapter = RollingNavigationAdapter(IntentParameters())
    adapter.update((0.30, 0.0, 0.0), dt=1.0)

    intent, command = adapter.update((0.0, 0.0, 0.50), dt=0.02)

    assert intent is MotionIntent.COORDINATED_TURN
    assert command[0] == 0.0
    assert command[1] == 0.0
    assert command[2] > 0.0


def test_navigation_adapter_turn_first_profile_cancels_cruise_immediately():
    parameters = IntentParameters(turn_max_forward=0.0)
    adapter = RollingNavigationAdapter(parameters)
    adapter.update((0.30, 0.0, 0.0), dt=1.0)

    intent, command = adapter.update((0.30, 0.20, 0.0), dt=0.02)

    assert intent is MotionIntent.COORDINATED_TURN
    assert command[0] == 0.0
    assert command[1] == 0.0
    assert command[2] > 0.0


def test_navigation_adapter_zero_stop_is_immediate_and_resets_state():
    adapter = RollingNavigationAdapter(IntentParameters())
    adapter.update((0.30, 0.20, 0.0), dt=1.0)

    intent, command = adapter.update((0.0, 0.0, 0.0), dt=0.02)

    assert intent is MotionIntent.STOPPED
    assert command == (0.0, 0.0, 0.0)
    assert not adapter.turning


def test_navigation_adapter_never_requests_autonomous_lateral_motion():
    adapter = RollingNavigationAdapter(IntentParameters())
    commands = (
        (0.40, 0.20, 0.30),
        (0.40, -0.20, -0.30),
        (0.0, 0.20, 0.0),
        (-0.30, 0.15, 0.10),
    )

    for command in commands:
        intent, output = adapter.update(command, dt=0.10)
        assert intent is not MotionIntent.LATERAL_MANEUVER
        assert output[1] == 0.0
