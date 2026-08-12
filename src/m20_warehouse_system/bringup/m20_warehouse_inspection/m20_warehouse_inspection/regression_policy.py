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

"""Pure helpers for repeatable phase-5 stability acceptance."""

from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class MemoryTrend:
    """Summarize process RSS samples captured after stable transactions."""

    first_mib: float
    last_mib: float
    peak_mib: float
    growth_mib: float
    slope_mib_per_iteration: float


def alternating_floor(initial_floor: str, transition_index: int) -> str:
    """Return the target floor for a zero-based alternating transition."""
    if initial_floor not in {'F1', 'F2'}:
        raise ValueError(f'unsupported initial floor {initial_floor!r}')
    if transition_index < 0:
        raise ValueError('transition_index must be non-negative')
    even_target = 'F2' if initial_floor == 'F1' else 'F1'
    return even_target if transition_index % 2 == 0 else initial_floor


def expected_generation(initial_generation: int, transitions: int) -> int:
    """Return the generation after successful single-increment switches."""
    if initial_generation < 1:
        raise ValueError('initial_generation must be positive')
    if transitions < 0:
        raise ValueError('transitions must be non-negative')
    return initial_generation + transitions


def parse_proc_status_memory(status: str) -> Tuple[int, int]:
    """Return parent PID and resident KiB from Linux procfs status text."""
    parent_pid = None
    rss_kib = 0
    for line in status.splitlines():
        if line.startswith('PPid:'):
            parent_pid = int(line.split()[1])
        elif line.startswith('VmRSS:'):
            rss_kib = int(line.split()[1])
    if parent_pid is None:
        raise ValueError('procfs status does not contain PPid')
    return parent_pid, rss_kib


def memory_trend(samples_mib: Sequence[float]) -> MemoryTrend:
    """Calculate an ordinary least-squares RSS slope over iterations."""
    if not samples_mib:
        raise ValueError('at least one memory sample is required')
    values = [float(value) for value in samples_mib]
    if any(value < 0.0 for value in values):
        raise ValueError('memory samples cannot be negative')
    count = len(values)
    if count == 1:
        slope = 0.0
    else:
        mean_x = (count - 1) / 2.0
        mean_y = sum(values) / count
        denominator = sum(
            (index - mean_x) ** 2 for index in range(count)
        )
        slope = sum(
            (index - mean_x) * (value - mean_y)
            for index, value in enumerate(values)
        ) / denominator
    return MemoryTrend(
        first_mib=values[0],
        last_mib=values[-1],
        peak_mib=max(values),
        growth_mib=values[-1] - values[0],
        slope_mib_per_iteration=slope,
    )


def memory_growth_is_bounded(
    samples_mib: Sequence[float],
    growth_limit_mib: float,
    slope_limit_mib_per_iteration: float,
) -> bool:
    """Check both end-to-end RSS growth and sustained linear trend."""
    if growth_limit_mib < 0.0 or slope_limit_mib_per_iteration < 0.0:
        raise ValueError('memory limits cannot be negative')
    trend = memory_trend(samples_mib)
    return (
        trend.growth_mib <= growth_limit_mib
        and trend.slope_mib_per_iteration
        <= slope_limit_mib_per_iteration
    )
