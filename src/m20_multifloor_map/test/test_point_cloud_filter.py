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

"""Unit tests for generation-independent point selection."""

import numpy as np
import pytest

from m20_multifloor_map.point_cloud_filter import (
    rotating_point_slice,
    select_local_points,
)


def test_selects_only_finite_points_inside_radius():
    points = np.asarray(
        [
            [0.0, 0.0, 1.0, 1.0],
            [3.0, 4.0, 2.0, 2.0],
            [5.01, 0.0, 3.0, 3.0],
            [np.nan, 0.0, 4.0, 4.0],
        ],
        dtype=np.float32,
    )

    selected = select_local_points(points, 0.0, 0.0, 5.0)

    assert selected.shape == (2, 4)
    np.testing.assert_allclose(selected[:, :2], [[0.0, 0.0], [3.0, 4.0]])


def test_rejects_invalid_shape_and_radius():
    with pytest.raises(ValueError):
        select_local_points(np.zeros((2, 3)), 0.0, 0.0, 1.0)
    with pytest.raises(ValueError):
        select_local_points(np.zeros((2, 4)), 0.0, 0.0, 0.0)


def test_rotating_slices_stay_bounded_and_cover_all_points():
    points = np.column_stack(
        [
            np.arange(10, dtype=np.float32),
            np.zeros((10, 3), dtype=np.float32),
        ]
    )

    slices = [
        rotating_point_slice(points, maximum_count=3, phase=phase)
        for phase in range(4)
    ]
    recovered = sorted(
        int(value)
        for point_slice in slices
        for value in point_slice[:, 0]
    )

    assert all(point_slice.shape[0] <= 3 for point_slice in slices)
    assert recovered == list(range(10))


def test_rotating_slice_does_not_mix_disjoint_floor_regions():
    floor_one = np.asarray(
        [[-10.0, 0.0, 0.0, 1.0], [-5.0, 1.0, 0.0, 1.0]],
        dtype=np.float32,
    )

    selected = rotating_point_slice(floor_one, maximum_count=1, phase=0)

    assert selected[:, 0].max() < 0.0
