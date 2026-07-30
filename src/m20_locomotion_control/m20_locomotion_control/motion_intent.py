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

"""Pure policy for classifying and constraining M20 body-velocity commands."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Tuple


PlanarCommand = Tuple[float, float, float]


class MotionIntent(str, Enum):
    """High-level intent; the ONNX policy still chooses all 16 joint actions."""

    STOPPED = 'STOPPED'
    BACKEND_HOLD = 'BACKEND_HOLD'
    FAULT_HOLD = 'FAULT_HOLD'
    WHEEL_CRUISE = 'WHEEL_CRUISE'
    COORDINATED_TURN = 'COORDINATED_TURN'
    LATERAL_MANEUVER = 'LATERAL_MANEUVER'


@dataclass(frozen=True)
class IntentParameters:
    """Limits and thresholds used before a safe Twist reaches the RL policy."""

    max_forward: float = 0.45
    max_side: float = 0.20
    max_yaw: float = 0.65
    deadband_linear: float = 0.01
    deadband_yaw: float = 0.02
    lateral_threshold: float = 0.05
    turn_yaw_threshold: float = 0.25
    in_place_linear_threshold: float = 0.08
    turn_curvature_threshold: float = 1.20
    curvature_speed_floor: float = 0.05
    turn_max_forward: float = 0.12
    lateral_max_forward: float = 0.10
    suppress_side_in_cruise: bool = True
    suppress_side_in_turn: bool = True
    course_yaw_gain: float = 0.80
    turn_course_enter: float = 0.25
    turn_course_exit: float = 0.08
    turn_yaw_exit: float = 0.12
    turn_min_hold_sec: float = 0.30
    cruise_yaw_deadband: float = 0.04
    cruise_yaw_filter_time_constant: float = 0.12
    output_linear_accel: float = 1.0
    output_yaw_accel: float = 1.2


def _clamp(value: float, limit: float) -> float:
    """Clamp a finite scalar symmetrically; non-finite input becomes zero."""
    if not math.isfinite(value):
        return 0.0
    safe_limit = max(0.0, float(limit))
    return max(-safe_limit, min(value, safe_limit))


def clamp_command(
    command: PlanarCommand,
    parameters: IntentParameters,
) -> PlanarCommand:
    """Apply the RL command envelope without increasing any component."""
    return (
        _clamp(command[0], parameters.max_forward),
        _clamp(command[1], parameters.max_side),
        _clamp(command[2], parameters.max_yaw),
    )


def classify_intent(
    command: PlanarCommand,
    parameters: IntentParameters,
) -> MotionIntent:
    """Classify the requested maneuver from its planar velocity command."""
    forward, side, yaw = clamp_command(command, parameters)
    if (
        abs(forward) <= parameters.deadband_linear
        and abs(side) <= parameters.deadband_linear
        and abs(yaw) <= parameters.deadband_yaw
    ):
        return MotionIntent.STOPPED

    if abs(side) >= parameters.lateral_threshold:
        return MotionIntent.LATERAL_MANEUVER

    curvature = abs(yaw) / max(
        abs(forward),
        max(1e-6, parameters.curvature_speed_floor),
    )
    if (
        abs(yaw) >= parameters.turn_yaw_threshold
        and (
            abs(forward) <= parameters.in_place_linear_threshold
            or curvature >= parameters.turn_curvature_threshold
        )
    ):
        return MotionIntent.COORDINATED_TURN

    return MotionIntent.WHEEL_CRUISE


def constrain_for_intent(
    command: PlanarCommand,
    parameters: IntentParameters,
) -> Tuple[MotionIntent, PlanarCommand]:
    """Return motion intent and the non-amplified command sent to the SDK."""
    forward, side, yaw = clamp_command(command, parameters)
    intent = classify_intent((forward, side, yaw), parameters)

    if intent is MotionIntent.STOPPED:
        return intent, (0.0, 0.0, 0.0)

    if intent is MotionIntent.WHEEL_CRUISE:
        if parameters.suppress_side_in_cruise:
            side = 0.0
        return intent, (forward, side, yaw)

    if intent is MotionIntent.COORDINATED_TURN:
        forward = _clamp(forward, parameters.turn_max_forward)
        if parameters.suppress_side_in_turn:
            side = 0.0
        return intent, (forward, side, yaw)

    forward = _clamp(forward, parameters.lateral_max_forward)
    return intent, (forward, side, yaw)


def _slew(value: float, target: float, limit: float, dt: float) -> float:
    """Move one command component toward its target at a bounded rate."""
    if dt <= 0.0:
        return value
    maximum_delta = max(0.0, limit) * dt
    delta = max(-maximum_delta, min(maximum_delta, target - value))
    return value + delta


def _soft_deadband(value: float, deadband: float) -> float:
    """Remove small corrections without introducing a step at the threshold."""
    threshold = max(0.0, deadband)
    magnitude = abs(value)
    if magnitude <= threshold:
        return 0.0
    return math.copysign(magnitude - threshold, value)


class RollingNavigationAdapter:
    """Convert holonomic path tracking into stable rolling-and-yaw commands."""

    def __init__(self, parameters: IntentParameters) -> None:
        self._parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Clear turn hysteresis and filtered output after a hard stop."""
        self._turning = False
        self._turn_hold_elapsed = 0.0
        self._last_output = (0.0, 0.0, 0.0)
        self._filtered_cruise_yaw = 0.0

    @property
    def turning(self) -> bool:
        """Return whether turn-first hysteresis is currently active."""
        return self._turning

    def update(
        self,
        command: PlanarCommand,
        dt: float,
    ) -> Tuple[MotionIntent, PlanarCommand]:
        """Adapt one autonomous command while preserving immediate zero stops."""
        forward, side, yaw = clamp_command(command, self._parameters)
        if (
            abs(forward) <= self._parameters.deadband_linear
            and abs(side) <= self._parameters.deadband_linear
            and abs(yaw) <= self._parameters.deadband_yaw
        ):
            self.reset()
            return MotionIntent.STOPPED, (0.0, 0.0, 0.0)

        # SCAN tracks a planar B-spline and can request body-frame lateral
        # velocity while the chassis is not yet tangent to the path. Convert
        # that direction error to yaw instead of asking the wheel-legged policy
        # to crab sideways during an ordinary autonomous patrol.
        direction_sign = (
            -1.0
            if forward < -self._parameters.deadband_linear
            else 1.0
        )
        course_error = direction_sign * math.atan2(
            side,
            max(
                abs(forward),
                self._parameters.curvature_speed_floor,
            ),
        )
        desired_yaw = _clamp(
            yaw + self._parameters.course_yaw_gain * course_error,
            self._parameters.max_yaw,
        )
        curvature = abs(desired_yaw) / max(
            abs(forward),
            self._parameters.curvature_speed_floor,
        )
        enter_turn = (
            abs(course_error) >= self._parameters.turn_course_enter
            or (
                abs(desired_yaw) >= self._parameters.turn_yaw_threshold
                and (
                    abs(forward)
                    <= self._parameters.in_place_linear_threshold
                    or curvature
                    >= self._parameters.turn_curvature_threshold
                )
            )
        )
        exit_turn = (
            abs(course_error) <= self._parameters.turn_course_exit
            and abs(desired_yaw) <= self._parameters.turn_yaw_exit
        )
        if self._turning:
            self._turn_hold_elapsed += max(0.0, dt)
            if (
                exit_turn
                and self._turn_hold_elapsed
                >= self._parameters.turn_min_hold_sec
            ):
                self._turning = False
                self._turn_hold_elapsed = 0.0
        elif enter_turn:
            self._turning = True
            self._turn_hold_elapsed = 0.0

        rolling_speed = math.copysign(
            min(
                math.hypot(forward, side),
                self._parameters.max_forward,
            ),
            forward if abs(forward) > 1.0e-9 else 1.0,
        )
        if self._turning:
            target_forward = _clamp(
                rolling_speed,
                self._parameters.turn_max_forward,
            )
            target_yaw = desired_yaw
            self._filtered_cruise_yaw = target_yaw
            intent = MotionIntent.COORDINATED_TURN
        else:
            target_forward = rolling_speed
            target_yaw = _soft_deadband(
                desired_yaw,
                self._parameters.cruise_yaw_deadband,
            )
            time_constant = max(
                0.0,
                self._parameters.cruise_yaw_filter_time_constant,
            )
            alpha = (
                1.0
                if time_constant <= 0.0
                else max(0.0, dt) / (time_constant + max(0.0, dt))
            )
            self._filtered_cruise_yaw += alpha * (
                target_yaw - self._filtered_cruise_yaw
            )
            target_yaw = self._filtered_cruise_yaw
            intent = MotionIntent.WHEEL_CRUISE

        pure_turn_request = (
            abs(forward) <= self._parameters.deadband_linear
            and abs(side) <= self._parameters.deadband_linear
            and abs(yaw) > self._parameters.deadband_yaw
        )
        turn_without_translation = (
            self._turning
            and self._parameters.turn_max_forward
            <= self._parameters.deadband_linear
        )
        output_forward = (
            0.0
            if pure_turn_request or turn_without_translation
            else _slew(
                self._last_output[0],
                target_forward,
                self._parameters.output_linear_accel,
                dt,
            )
        )
        output = (
            output_forward,
            0.0,
            _slew(
                self._last_output[2],
                target_yaw,
                self._parameters.output_yaw_accel,
                dt,
            ),
        )
        self._last_output = output
        return intent, output
