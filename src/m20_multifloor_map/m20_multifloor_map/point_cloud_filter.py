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

"""Pure NumPy helpers used by the dynamic local sensing node."""

import math

import numpy as np


def select_local_points(
    points: np.ndarray,
    center_x: float,
    center_y: float,
    radius: float,
) -> np.ndarray:
    """Return finite points inside a horizontal circular sensing window."""
    values = np.asarray(points, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError('points must have shape (N, 4)')
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    if values.size == 0:
        return values.copy()
    finite = np.isfinite(values).all(axis=1)
    dx = values[:, 0] - float(center_x)
    dy = values[:, 1] - float(center_y)
    selected = finite & ((dx * dx + dy * dy) <= float(radius) ** 2)
    return np.ascontiguousarray(values[selected], dtype=np.float32)


def rotating_point_slice(
    points: np.ndarray,
    maximum_count: int,
    phase: int,
) -> np.ndarray:
    """Limit one DDS sample while covering every point across later phases."""
    values = np.asarray(points, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError('points must have shape (N, 4)')
    if maximum_count <= 0:
        raise ValueError('maximum_count must be positive')
    if values.shape[0] <= maximum_count:
        return np.ascontiguousarray(values, dtype=np.float32)

    # Each phase publishes one residue class. This keeps every sample below
    # the DDS transport threshold and still reconstructs the full local map
    # in the accumulating SCAN occupancy grid over successive frames.
    stride = int(math.ceil(values.shape[0] / maximum_count))
    start = int(phase) % stride
    indices = np.arange(start, values.shape[0], stride, dtype=np.int64)
    return np.ascontiguousarray(values[indices], dtype=np.float32)
