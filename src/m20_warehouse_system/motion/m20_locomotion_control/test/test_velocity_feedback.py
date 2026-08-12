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

"""Unit tests for bounded M20 measured-velocity feedback."""

import math

import pytest

from m20_locomotion_control.velocity_feedback import (
    MeasuredVelocityFeedback,
    VelocityFeedbackParameters,
)


def test_error_inside_deadband_does_not_change_reference() -> None:
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())

    result = controller.update(
        (0.30, 0.0, 0.20),
        (0.28, 0.0, 0.17),
        0.02,
    )

    assert result.output == (0.30, 0.0, 0.20)
    assert result.correction == (0.0, 0.0, 0.0)


def test_under_response_adds_bounded_forward_and_yaw_correction() -> None:
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())

    result = controller.update(
        (0.30, 0.0, 0.30),
        (0.10, 0.0, 0.10),
        0.10,
    )

    assert 0.30 < result.output[0] <= 0.40
    assert 0.30 < result.output[2] <= 0.42
    assert result.integral[0] > 0.0
    assert result.integral[2] > 0.0


def test_correction_and_sdk_envelope_limits_are_both_enforced() -> None:
    parameters = VelocityFeedbackParameters(
        linear_kp=10.0,
        yaw_kp=10.0,
        linear_correction_limit=0.05,
        yaw_correction_limit=0.08,
    )
    controller = MeasuredVelocityFeedback(parameters)

    result = controller.update(
        (0.42, 0.0, 0.60),
        (0.0, 0.0, 0.0),
        0.10,
    )

    assert result.output[0] == parameters.max_forward
    assert result.output[2] == parameters.max_yaw
    assert result.correction[0] == pytest.approx(0.03)
    assert result.correction[2] == pytest.approx(0.05)


def test_feedback_never_reverses_the_requested_motion() -> None:
    controller = MeasuredVelocityFeedback(
        VelocityFeedbackParameters(
            linear_kp=10.0,
            yaw_kp=10.0,
        )
    )

    result = controller.update(
        (0.05, 0.0, -0.05),
        (1.0, 0.0, -1.0),
        0.10,
    )

    assert result.output[0] == 0.0
    assert result.output[2] == 0.0


def test_conditional_integration_prevents_saturation_windup() -> None:
    controller = MeasuredVelocityFeedback(
        VelocityFeedbackParameters(
            linear_kp=1.0,
            linear_ki=1.0,
        )
    )

    first = controller.update(
        (0.44, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        0.10,
    )
    second = controller.update(
        (0.44, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        0.10,
    )

    assert first.output[0] == 0.45
    assert second.output[0] == 0.45
    assert first.integral[0] == 0.0
    assert second.integral[0] == 0.0


def test_reference_sign_change_clears_old_integral() -> None:
    controller = MeasuredVelocityFeedback(
        VelocityFeedbackParameters(
            linear_kp=0.0,
            linear_ki=1.0,
        )
    )
    forward = controller.update(
        (0.20, 0.0, 0.0),
        (0.10, 0.0, 0.0),
        0.10,
    )
    reverse = controller.update(
        (-0.20, 0.0, 0.0),
        (-0.10, 0.0, 0.0),
        0.10,
    )

    assert forward.integral[0] > 0.0
    assert reverse.integral[0] < 0.0
    assert abs(reverse.integral[0]) == pytest.approx(
        abs(forward.integral[0])
    )


def test_zero_reference_resets_axis_integral() -> None:
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())
    controller.update((0.30, 0.0, 0.20), (0.0, 0.0, 0.0), 0.10)

    result = controller.update(
        (0.0, 0.0, 0.0),
        (0.20, 0.0, 0.10),
        0.10,
    )

    assert result.output == (0.0, 0.0, 0.0)
    assert result.integral == (0.0, 0.0, 0.0)


def test_lateral_reference_passes_through_without_feedback() -> None:
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())

    result = controller.update(
        (0.20, 0.07, 0.10),
        (0.10, -0.20, 0.0),
        0.10,
    )

    assert result.output[1] == 0.07
    assert result.correction[1] == 0.0


def test_nonfinite_measurement_resets_and_fails_open() -> None:
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())
    controller.update((0.30, 0.0, 0.20), (0.0, 0.0, 0.0), 0.10)

    result = controller.update(
        (0.25, 0.0, 0.15),
        (math.nan, 0.0, 0.0),
        0.10,
    )

    assert result.output == (0.25, 0.0, 0.15)
    assert result.integral == (0.0, 0.0, 0.0)
