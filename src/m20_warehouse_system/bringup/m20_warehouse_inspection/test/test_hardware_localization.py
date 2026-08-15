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

"""Unit tests for the Foxy-compatible real localization boundary."""

import math
from pathlib import Path

import pytest

from m20_warehouse_inspection.hardware_localization import (
    derive_body_twist,
    PoseSample,
)


ROOT = Path(__file__).parents[1]


def test_localization_adapter_aliases_lio_messages_without_publishing_tf():
    text = (
        ROOT
        / 'm20_warehouse_inspection'
        / 'hardware_localization_adapter_node.py'
    ).read_text(encoding='utf-8')

    assert "'expected_world_frame', 'lio_world'" in text
    assert "'expected_body_frame', 'lio_base_link'" in text
    assert "'expected_sensor_frame', 'lio_imu'" in text
    assert "'output_world_frame', 'world'" in text
    assert "'output_body_frame', 'base_link'" in text
    assert "'output_cloud_topic', '/m20/localization/cloud'" in text
    assert 'TransformBroadcaster' not in text
    assert 'StaticTransformBroadcaster' not in text


def test_pose_difference_is_rotated_into_body_frame():
    previous = PoseSample(1.0, 0.0, 0.0, math.pi / 2.0)
    current = PoseSample(1.1, 0.0, 0.1, math.pi / 2.0)

    twist = derive_body_twist(previous, current)

    assert twist is not None
    assert twist[0] == pytest.approx(1.0)
    assert twist[1] == pytest.approx(0.0, abs=1.0e-9)
    assert twist[2] == pytest.approx(0.0)


def test_pose_difference_wraps_yaw_at_pi():
    previous = PoseSample(2.0, 0.0, 0.0, math.pi - 0.01)
    current = PoseSample(2.1, 0.0, 0.0, -math.pi + 0.01)

    twist = derive_body_twist(previous, current)

    assert twist is not None
    assert twist[2] == pytest.approx(0.2)


def test_pose_difference_rejects_stale_or_reversed_time():
    previous = PoseSample(2.0, 0.0, 0.0, 0.0)

    assert derive_body_twist(
        previous, PoseSample(1.9, 0.1, 0.0, 0.0)
    ) is None
    assert derive_body_twist(
        previous, PoseSample(3.0, 0.1, 0.0, 0.0)
    ) is None
