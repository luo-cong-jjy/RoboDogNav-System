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

"""Tests for the versioned M20 platform capability boundary."""

import math
from pathlib import Path

import pytest

from m20_locomotion_control.capability_profile import (
    load_capability_profile,
)


ROOT = Path(__file__).parents[1]
PROFILE = ROOT / 'config' / 'm20_policy_v1_capabilities.yaml'


def test_default_profile_has_measured_rolling_envelope() -> None:
    profile = load_capability_profile(PROFILE)

    assert profile.profile_id == 'm20_policy_v1'
    assert not profile.supports_zero_radius_yaw
    assert not profile.supports_autonomous_lateral
    assert math.isclose(
        profile.minimum_centerline_turn_radius,
        0.35 / 0.65,
    )
    assert math.isclose(profile.turn_swept_radius, 0.893, abs_tol=0.002)


def test_one_profile_injects_identical_limits_into_all_consumers() -> None:
    profile = load_capability_profile(PROFILE)
    intent = profile.intent_parameters()
    guard = profile.collision_guard_parameters()
    controller = profile.controller_parameters()
    safety = profile.safety_parameters()
    sdk = profile.sdk_parameters()

    assert intent['max_forward'] == guard['max_linear_x']
    assert intent['max_forward'] == sdk['max_forward']
    assert intent['max_forward'] == safety['max_linear_x']
    assert intent['max_side'] == guard['max_linear_y']
    assert intent['max_yaw'] == guard['max_angular_z']
    assert intent['max_yaw'] == safety['max_angular_z']
    assert intent['turn_min_forward'] == guard['recovery_forward_speed']
    assert guard['recovery_rearm_clear_sec'] == 1.50
    assert controller == {
        'bidirectional_tracking_enabled': True,
        'reverse_tracking_enter_angle': 2.10,
        'reverse_tracking_exit_angle': 1.75,
        'reverse_tracking_min_hold_sec': 0.80,
        'reverse_tracking_entry_alignment': 0.20,
        'reverse_tracking_exit_alignment': 0.35,
    }
    assert intent['capability_profile_id'] == 'm20_policy_v1'


def test_invalid_turn_interval_fails_before_launch(tmp_path: Path) -> None:
    invalid = tmp_path / 'invalid.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'stable_min_forward_mps: 0.35',
        'stable_min_forward_mps: 0.55',
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='min <= max <= command max'):
        load_capability_profile(invalid)


def test_recovery_rearm_must_not_be_shorter_than_release(
    tmp_path: Path,
) -> None:
    invalid = tmp_path / 'invalid_rearm.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'rearm_clear_sec: 1.50',
        'rearm_clear_sec: 0.20',
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='at least the release time'):
        load_capability_profile(invalid)


def test_reverse_tracking_hysteresis_must_be_ordered(
    tmp_path: Path,
) -> None:
    invalid = tmp_path / 'invalid_reverse_tracking.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'reverse_tracking_exit_angle_rad: 1.75',
        'reverse_tracking_exit_angle_rad: 2.20',
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='0 <= exit < enter <= pi'):
        load_capability_profile(invalid)


def test_reverse_tracking_alignment_must_have_hysteresis(
    tmp_path: Path,
) -> None:
    invalid = tmp_path / 'invalid_reverse_alignment.yaml'
    text = PROFILE.read_text(encoding='utf-8').replace(
        'reverse_tracking_exit_alignment_rad: 0.35',
        'reverse_tracking_exit_alignment_rad: 0.15',
    )
    invalid.write_text(text, encoding='utf-8')

    with pytest.raises(ValueError, match='0 < entry < exit < pi/2'):
        load_capability_profile(invalid)
