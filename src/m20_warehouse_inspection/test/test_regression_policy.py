# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test phase-5 generation and memory-trend policies."""

import pytest

from m20_warehouse_inspection.regression_policy import (
    alternating_floor,
    expected_generation,
    memory_growth_is_bounded,
    memory_trend,
    parse_proc_status_memory,
)


def test_alternating_floor_sequence() -> None:
    assert [alternating_floor('F1', index) for index in range(4)] == [
        'F2',
        'F1',
        'F2',
        'F1',
    ]


def test_generation_increments_once_per_switch() -> None:
    assert expected_generation(1, 20) == 21
    with pytest.raises(ValueError):
        expected_generation(0, 1)


def test_memory_trend_reports_linear_growth() -> None:
    trend = memory_trend([100.0, 102.0, 104.0, 106.0])
    assert trend.growth_mib == pytest.approx(6.0)
    assert trend.slope_mib_per_iteration == pytest.approx(2.0)
    assert trend.peak_mib == pytest.approx(106.0)


def test_memory_growth_policy_checks_growth_and_slope() -> None:
    assert memory_growth_is_bounded([100.0, 102.0, 101.0], 8.0, 2.0)
    assert not memory_growth_is_bounded(
        [100.0, 110.0, 120.0], 30.0, 5.0
    )


def test_proc_status_memory_parser() -> None:
    status = 'Name:\tm20_node\nPPid:\t4242\nVmRSS:\t8192 kB\n'
    assert parse_proc_status_memory(status) == (4242, 8192)
    with pytest.raises(ValueError):
        parse_proc_status_memory('Name:\tmissing_parent\n')
