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

"""Validated, versioned M20 motion-capability configuration."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import yaml


def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f'capability profile section {key!r} is missing')
    return value


def _number(parent: Mapping[str, Any], key: str) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'capability profile value {key!r} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'capability profile value {key!r} must be finite')
    return result


def _boolean(parent: Mapping[str, Any], key: str) -> bool:
    value = parent.get(key)
    if not isinstance(value, bool):
        raise ValueError(f'capability profile value {key!r} must be boolean')
    return value


@dataclass(frozen=True)
class M20CapabilityProfile:
    """Single source of truth for M20 command and swept-turn limits."""

    profile_id: str
    body_length: float
    body_width: float
    body_height: float
    max_forward: float
    max_side: float
    max_yaw: float
    supports_autonomous_lateral: bool
    supports_reverse_tracking: bool
    supports_zero_radius_yaw: bool
    deadband_linear: float
    deadband_yaw: float
    lateral_threshold: float
    turn_yaw_threshold: float
    in_place_linear_threshold: float
    turn_curvature_threshold: float
    curvature_speed_floor: float
    turn_min_forward: float
    turn_max_forward: float
    lateral_max_forward: float
    course_yaw_gain: float
    turn_course_enter: float
    turn_course_exit: float
    turn_yaw_exit: float
    turn_min_hold_sec: float
    cruise_yaw_deadband: float
    cruise_yaw_filter_time_constant: float
    reverse_speed_offset: float
    reverse_speed_gain: float
    reverse_yaw_offset: float
    reverse_yaw_gain: float
    output_linear_accel: float
    output_yaw_accel: float
    reverse_tracking_enter_angle: float
    reverse_tracking_exit_angle: float
    reverse_tracking_min_hold_sec: float
    reverse_tracking_entry_alignment: float
    reverse_tracking_exit_alignment: float
    positive_yaw_lateral_drift: float
    negative_yaw_lateral_drift: float
    opposite_lateral_uncertainty: float
    recovery_forward_speed: float
    recovery_lookahead_sec: float
    recovery_min_yaw_rate: float
    recovery_clear_confirm_sec: float
    recovery_rearm_clear_sec: float
    recovery_max_active_sec: float
    recovery_max_displacement_m: float
    recovery_progress_timeout_sec: float
    recovery_min_progress_m: float
    measurement_source: str

    @property
    def minimum_centerline_turn_radius(self) -> float:
        """Smallest calibrated rolling radius at the body centre."""
        return self.turn_min_forward / self.max_yaw

    @property
    def turn_swept_radius(self) -> float:
        """Outer-corner sweep for the minimum-radius rolling turn."""
        lateral_extent = (
            self.minimum_centerline_turn_radius + self.body_width / 2.0
        )
        return math.hypot(lateral_extent, self.body_length / 2.0)

    def intent_parameters(self) -> dict[str, Any]:
        """ROS parameters shared by pre-safety and final command gates."""
        return {
            'capability_profile_id': self.profile_id,
            'minimum_centerline_turn_radius': (
                self.minimum_centerline_turn_radius
            ),
            'turn_swept_radius': self.turn_swept_radius,
            'max_forward': self.max_forward,
            'max_side': self.max_side,
            'max_yaw': self.max_yaw,
            'deadband_linear': self.deadband_linear,
            'deadband_yaw': self.deadband_yaw,
            'lateral_threshold': self.lateral_threshold,
            'turn_yaw_threshold': self.turn_yaw_threshold,
            'in_place_linear_threshold': self.in_place_linear_threshold,
            'turn_curvature_threshold': self.turn_curvature_threshold,
            'curvature_speed_floor': self.curvature_speed_floor,
            'turn_min_forward': self.turn_min_forward,
            'turn_max_forward': self.turn_max_forward,
            'lateral_max_forward': self.lateral_max_forward,
            'suppress_side_in_cruise': (
                not self.supports_autonomous_lateral
            ),
            'suppress_side_in_turn': (
                not self.supports_autonomous_lateral
            ),
            'course_yaw_gain': self.course_yaw_gain,
            'turn_course_enter': self.turn_course_enter,
            'turn_course_exit': self.turn_course_exit,
            'turn_yaw_exit': self.turn_yaw_exit,
            'turn_min_hold_sec': self.turn_min_hold_sec,
            'cruise_yaw_deadband': self.cruise_yaw_deadband,
            'cruise_yaw_filter_time_constant': (
                self.cruise_yaw_filter_time_constant
            ),
            'reverse_speed_offset': self.reverse_speed_offset,
            'reverse_speed_gain': self.reverse_speed_gain,
            'reverse_yaw_offset': self.reverse_yaw_offset,
            'reverse_yaw_gain': self.reverse_yaw_gain,
            'output_linear_accel': self.output_linear_accel,
            'output_yaw_accel': self.output_yaw_accel,
        }

    def collision_guard_parameters(self) -> dict[str, Any]:
        """Motion-model parameters; map-clearance geometry stays separate."""
        return {
            'capability_profile_id': self.profile_id,
            'minimum_centerline_turn_radius': (
                self.minimum_centerline_turn_radius
            ),
            'turn_swept_radius': self.turn_swept_radius,
            'max_linear_x': self.max_forward,
            'max_linear_y': self.max_side,
            'max_angular_z': self.max_yaw,
            'model_positive_yaw_lateral_drift': (
                self.positive_yaw_lateral_drift
            ),
            'model_negative_yaw_lateral_drift': (
                self.negative_yaw_lateral_drift
            ),
            'model_opposite_lateral_uncertainty': (
                self.opposite_lateral_uncertainty
            ),
            'model_reference_yaw_rate': self.max_yaw,
            'recovery_forward_speed': self.recovery_forward_speed,
            'recovery_positive_yaw_lateral_drift': (
                self.positive_yaw_lateral_drift
            ),
            'recovery_negative_yaw_lateral_drift': (
                self.negative_yaw_lateral_drift
            ),
            'recovery_opposite_lateral_uncertainty': (
                self.opposite_lateral_uncertainty
            ),
            'recovery_lookahead_sec': self.recovery_lookahead_sec,
            'recovery_min_yaw_rate': self.recovery_min_yaw_rate,
            'recovery_clear_confirm_sec': self.recovery_clear_confirm_sec,
            'recovery_rearm_clear_sec': self.recovery_rearm_clear_sec,
            'recovery_max_active_sec': self.recovery_max_active_sec,
            'recovery_max_displacement_m': (
                self.recovery_max_displacement_m
            ),
            'recovery_progress_timeout_sec': (
                self.recovery_progress_timeout_sec
            ),
            'recovery_min_progress_m': self.recovery_min_progress_m,
        }

    def controller_parameters(self) -> dict[str, Any]:
        """M20-only execution policy layered after vendor SCAN control."""
        return {
            'bidirectional_tracking_enabled': (
                self.supports_reverse_tracking
            ),
            'reverse_tracking_enter_angle': (
                self.reverse_tracking_enter_angle
            ),
            'reverse_tracking_exit_angle': (
                self.reverse_tracking_exit_angle
            ),
            'reverse_tracking_min_hold_sec': (
                self.reverse_tracking_min_hold_sec
            ),
            'reverse_tracking_entry_alignment': (
                self.reverse_tracking_entry_alignment
            ),
            'reverse_tracking_exit_alignment': (
                self.reverse_tracking_exit_alignment
            ),
        }

    def safety_parameters(self) -> dict[str, Any]:
        """Limits for the command multiplexer without duplicating slew."""
        return {
            'capability_profile_id': self.profile_id,
            'max_linear_x': self.max_forward,
            'max_linear_y': self.max_side,
            'max_angular_z': self.max_yaw,
        }

    def sdk_parameters(self) -> dict[str, float]:
        """Limits understood by the vendored rl_deploy_cmdvel node."""
        return {
            'max_forward': self.max_forward,
            'max_side': self.max_side,
            'max_yaw': self.max_yaw,
        }


def load_capability_profile(path: str | Path) -> M20CapabilityProfile:
    """Load and validate a capability profile without ROS dependencies."""
    profile_path = Path(path).expanduser().resolve()
    with profile_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, Mapping):
        raise ValueError('capability profile root must be a mapping')
    if document.get('schema_version') != 1:
        raise ValueError('capability profile schema_version must be 1')
    profile_id = document.get('profile_id')
    if not isinstance(profile_id, str) or not profile_id.strip():
        raise ValueError('capability profile_id must be a non-empty string')

    body = _mapping(document, 'body')
    command = _mapping(document, 'command')
    turn = _mapping(document, 'turn')
    tracking = _mapping(document, 'tracking')
    reverse = _mapping(document, 'reverse_compensation')
    drift = _mapping(document, 'measured_drift')
    recovery = _mapping(document, 'recovery')
    feedback = _mapping(document, 'feedback')
    measurement_source = feedback.get('measurement_source')
    if not isinstance(measurement_source, str) or not measurement_source:
        raise ValueError('feedback.measurement_source must be a string')

    result = M20CapabilityProfile(
        profile_id=profile_id,
        body_length=_number(body, 'length_m'),
        body_width=_number(body, 'width_m'),
        body_height=_number(body, 'height_m'),
        max_forward=_number(command, 'max_forward_mps'),
        max_side=_number(command, 'max_side_mps'),
        max_yaw=_number(command, 'max_yaw_radps'),
        supports_autonomous_lateral=_boolean(
            command, 'supports_autonomous_lateral'
        ),
        supports_reverse_tracking=_boolean(
            command, 'supports_reverse_tracking'
        ),
        supports_zero_radius_yaw=_boolean(
            turn, 'supports_zero_radius_yaw'
        ),
        deadband_linear=_number(tracking, 'deadband_linear_mps'),
        deadband_yaw=_number(tracking, 'deadband_yaw_radps'),
        lateral_threshold=_number(
            tracking, 'lateral_threshold_mps'
        ),
        turn_yaw_threshold=_number(
            tracking, 'turn_yaw_threshold_radps'
        ),
        in_place_linear_threshold=_number(
            tracking, 'in_place_linear_threshold_mps'
        ),
        turn_curvature_threshold=_number(
            tracking, 'turn_curvature_threshold'
        ),
        curvature_speed_floor=_number(
            tracking, 'curvature_speed_floor_mps'
        ),
        turn_min_forward=_number(turn, 'stable_min_forward_mps'),
        turn_max_forward=_number(turn, 'stable_max_forward_mps'),
        lateral_max_forward=_number(
            tracking, 'lateral_max_forward_mps'
        ),
        course_yaw_gain=_number(tracking, 'course_yaw_gain'),
        turn_course_enter=_number(tracking, 'turn_course_enter_rad'),
        turn_course_exit=_number(tracking, 'turn_course_exit_rad'),
        turn_yaw_exit=_number(tracking, 'turn_yaw_exit_radps'),
        turn_min_hold_sec=_number(tracking, 'turn_min_hold_sec'),
        cruise_yaw_deadband=_number(
            tracking, 'cruise_yaw_deadband_radps'
        ),
        cruise_yaw_filter_time_constant=_number(
            tracking, 'cruise_yaw_filter_time_constant_sec'
        ),
        reverse_speed_offset=_number(reverse, 'speed_offset_mps'),
        reverse_speed_gain=_number(reverse, 'speed_gain'),
        reverse_yaw_offset=_number(reverse, 'yaw_offset_radps'),
        reverse_yaw_gain=_number(reverse, 'yaw_gain'),
        output_linear_accel=_number(
            tracking, 'output_linear_accel_mps2'
        ),
        output_yaw_accel=_number(
            tracking, 'output_yaw_accel_radps2'
        ),
        reverse_tracking_enter_angle=_number(
            tracking, 'reverse_tracking_enter_angle_rad'
        ),
        reverse_tracking_exit_angle=_number(
            tracking, 'reverse_tracking_exit_angle_rad'
        ),
        reverse_tracking_min_hold_sec=_number(
            tracking, 'reverse_tracking_min_hold_sec'
        ),
        reverse_tracking_entry_alignment=_number(
            tracking, 'reverse_tracking_entry_alignment_rad'
        ),
        reverse_tracking_exit_alignment=_number(
            tracking, 'reverse_tracking_exit_alignment_rad'
        ),
        positive_yaw_lateral_drift=_number(
            drift, 'positive_yaw_lateral_mps'
        ),
        negative_yaw_lateral_drift=_number(
            drift, 'negative_yaw_lateral_mps'
        ),
        opposite_lateral_uncertainty=_number(
            drift, 'opposite_lateral_uncertainty_mps'
        ),
        recovery_forward_speed=_number(recovery, 'forward_speed_mps'),
        recovery_lookahead_sec=_number(recovery, 'lookahead_sec'),
        recovery_min_yaw_rate=_number(recovery, 'minimum_yaw_radps'),
        recovery_clear_confirm_sec=_number(
            recovery, 'clear_confirm_sec'
        ),
        recovery_rearm_clear_sec=_number(
            recovery, 'rearm_clear_sec'
        ),
        recovery_max_active_sec=_number(recovery, 'max_active_sec'),
        recovery_max_displacement_m=_number(
            recovery, 'max_displacement_m'
        ),
        recovery_progress_timeout_sec=_number(
            recovery, 'progress_timeout_sec'
        ),
        recovery_min_progress_m=_number(
            recovery, 'minimum_progress_m'
        ),
        measurement_source=measurement_source,
    )
    _validate_profile(result)
    return result


def _validate_profile(profile: M20CapabilityProfile) -> None:
    """Reject internally inconsistent values before any node is started."""
    positive = {
        'body_length': profile.body_length,
        'body_width': profile.body_width,
        'body_height': profile.body_height,
        'max_forward': profile.max_forward,
        'max_side': profile.max_side,
        'max_yaw': profile.max_yaw,
        'turn_min_forward': profile.turn_min_forward,
        'turn_max_forward': profile.turn_max_forward,
        'recovery_forward_speed': profile.recovery_forward_speed,
        'recovery_lookahead_sec': profile.recovery_lookahead_sec,
        'recovery_rearm_clear_sec': profile.recovery_rearm_clear_sec,
        'recovery_max_active_sec': profile.recovery_max_active_sec,
        'recovery_max_displacement_m': (
            profile.recovery_max_displacement_m
        ),
        'recovery_progress_timeout_sec': (
            profile.recovery_progress_timeout_sec
        ),
        'recovery_min_progress_m': profile.recovery_min_progress_m,
        'reverse_tracking_min_hold_sec': (
            profile.reverse_tracking_min_hold_sec
        ),
        'reverse_tracking_entry_alignment': (
            profile.reverse_tracking_entry_alignment
        ),
        'reverse_tracking_exit_alignment': (
            profile.reverse_tracking_exit_alignment
        ),
    }
    for name, value in positive.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
    if not (
        profile.turn_min_forward
        <= profile.turn_max_forward
        <= profile.max_forward
    ):
        raise ValueError(
            'stable rolling speeds must satisfy min <= max <= command max'
        )
    if profile.recovery_forward_speed > profile.max_forward:
        raise ValueError('recovery speed exceeds the command envelope')
    if not (
        0.0
        <= profile.reverse_tracking_exit_angle
        < profile.reverse_tracking_enter_angle
        <= math.pi
    ):
        raise ValueError(
            'reverse tracking angles must satisfy 0 <= exit < enter <= pi'
        )
    if not (
        0.0
        < profile.reverse_tracking_entry_alignment
        < profile.reverse_tracking_exit_alignment
        < math.pi / 2.0
    ):
        raise ValueError(
            'reverse tracking alignment must satisfy '
            '0 < entry < exit < pi/2'
        )
    if (
        profile.recovery_rearm_clear_sec
        < profile.recovery_clear_confirm_sec
    ):
        raise ValueError(
            'recovery rearm clear time must be at least the release time'
        )
    if not profile.supports_zero_radius_yaw and (
        profile.turn_min_forward <= 0.0
    ):
        raise ValueError('rolling-only profile requires positive turn speed')
    if profile.recovery_min_progress_m > (
        profile.recovery_forward_speed
        * profile.recovery_progress_timeout_sec
    ):
        raise ValueError('minimum recovery progress is physically unreachable')
