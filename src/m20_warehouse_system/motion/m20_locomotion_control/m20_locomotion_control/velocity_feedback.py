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

"""Bounded inner-loop velocity feedback for the M20 rolling adapter."""

from dataclasses import dataclass
import math
from typing import Tuple


PlanarCommand = Tuple[float, float, float]


@dataclass(frozen=True)
class VelocityFeedbackParameters:
    """Gains and safety bounds for measured body-velocity compensation."""

    linear_kp: float = 0.30
    linear_ki: float = 0.08
    yaw_kp: float = 0.20
    yaw_ki: float = 0.05
    linear_error_deadband: float = 0.03
    yaw_error_deadband: float = 0.04
    linear_integral_limit: float = 0.20
    yaw_integral_limit: float = 0.25
    linear_correction_limit: float = 0.10
    yaw_correction_limit: float = 0.12
    max_forward: float = 0.45
    max_yaw: float = 0.65
    reference_linear_deadband: float = 0.01
    reference_yaw_deadband: float = 0.02


@dataclass(frozen=True)
class VelocityFeedbackResult:
    """One controller update with values suitable for diagnostics."""

    output: PlanarCommand
    error: PlanarCommand
    correction: PlanarCommand
    integral: PlanarCommand


def _finite(value: float) -> bool:
    """Return whether a scalar is finite."""
    return math.isfinite(float(value))


def _clamp(value: float, limit: float) -> float:
    """Clamp a scalar symmetrically around zero."""
    safe_limit = max(0.0, float(limit))
    return max(-safe_limit, min(float(value), safe_limit))


def _soft_deadband(value: float, deadband: float) -> float:
    """Remove a central error band without a discontinuous correction."""
    threshold = max(0.0, float(deadband))
    magnitude = abs(float(value))
    if magnitude <= threshold:
        return 0.0
    return math.copysign(magnitude - threshold, value)


class MeasuredVelocityFeedback:
    """Apply bounded PI correction without reversing the requested motion."""

    def __init__(self, parameters: VelocityFeedbackParameters) -> None:
        self._parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Clear all integral and reference-sign history."""
        self._linear_integral = 0.0
        self._yaw_integral = 0.0
        self._last_linear_reference = 0.0
        self._last_yaw_reference = 0.0

    def _update_axis(
        self,
        *,
        reference: float,
        measurement: float,
        dt: float,
        kp: float,
        ki: float,
        error_deadband: float,
        reference_deadband: float,
        integral_limit: float,
        correction_limit: float,
        output_limit: float,
        integral: float,
        last_reference: float,
    ) -> Tuple[float, float, float, float]:
        """Update one sign-preserving PI channel with anti-windup."""
        if abs(reference) <= max(0.0, reference_deadband):
            return reference, 0.0, 0.0, 0.0
        if reference * last_reference < 0.0:
            integral = 0.0

        error = _soft_deadband(
            reference - measurement,
            error_deadband,
        )
        proposed_integral = _clamp(
            integral + error * max(0.0, dt),
            integral_limit,
        )

        def output_for(
            candidate_integral: float,
        ) -> Tuple[float, float, float]:
            correction = _clamp(
                max(0.0, kp) * error
                + max(0.0, ki) * candidate_integral,
                correction_limit,
            )
            unconstrained = reference + correction
            bounded = _clamp(unconstrained, output_limit)
            if reference > 0.0:
                bounded = max(0.0, bounded)
            else:
                bounded = min(0.0, bounded)
            return bounded, bounded - reference, unconstrained

        output, correction, unconstrained = output_for(
            proposed_integral
        )
        saturation = unconstrained - output
        if saturation * error > 0.0:
            proposed_integral = integral
            output, correction, _ = output_for(proposed_integral)
        return output, error, correction, proposed_integral

    def update(
        self,
        reference: PlanarCommand,
        measurement: PlanarCommand,
        dt: float,
    ) -> VelocityFeedbackResult:
        """Correct forward and yaw references from measured body velocity."""
        if (
            not all(_finite(value) for value in reference)
            or not all(_finite(value) for value in measurement)
            or not _finite(dt)
        ):
            self.reset()
            safe_reference = tuple(
                float(value) if _finite(value) else 0.0
                for value in reference
            )
            return VelocityFeedbackResult(
                output=safe_reference,
                error=(0.0, 0.0, 0.0),
                correction=(0.0, 0.0, 0.0),
                integral=(0.0, 0.0, 0.0),
            )

        forward, linear_error, linear_correction, linear_integral = (
            self._update_axis(
                reference=float(reference[0]),
                measurement=float(measurement[0]),
                dt=max(0.0, min(0.10, float(dt))),
                kp=self._parameters.linear_kp,
                ki=self._parameters.linear_ki,
                error_deadband=self._parameters.linear_error_deadband,
                reference_deadband=(
                    self._parameters.reference_linear_deadband
                ),
                integral_limit=self._parameters.linear_integral_limit,
                correction_limit=(
                    self._parameters.linear_correction_limit
                ),
                output_limit=self._parameters.max_forward,
                integral=self._linear_integral,
                last_reference=self._last_linear_reference,
            )
        )
        yaw, yaw_error, yaw_correction, yaw_integral = self._update_axis(
            reference=float(reference[2]),
            measurement=float(measurement[2]),
            dt=max(0.0, min(0.10, float(dt))),
            kp=self._parameters.yaw_kp,
            ki=self._parameters.yaw_ki,
            error_deadband=self._parameters.yaw_error_deadband,
            reference_deadband=self._parameters.reference_yaw_deadband,
            integral_limit=self._parameters.yaw_integral_limit,
            correction_limit=self._parameters.yaw_correction_limit,
            output_limit=self._parameters.max_yaw,
            integral=self._yaw_integral,
            last_reference=self._last_yaw_reference,
        )
        self._linear_integral = linear_integral
        self._yaw_integral = yaw_integral
        self._last_linear_reference = float(reference[0])
        self._last_yaw_reference = float(reference[2])
        return VelocityFeedbackResult(
            output=(forward, float(reference[1]), yaw),
            error=(linear_error, 0.0, yaw_error),
            correction=(linear_correction, 0.0, yaw_correction),
            integral=(linear_integral, 0.0, yaw_integral),
        )
