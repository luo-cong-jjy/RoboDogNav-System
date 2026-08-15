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

"""Regression checks for explicit M20 straight-passage profiles."""

from pathlib import Path

import pytest

from m20_scan_navigation.clearance import load_clearance_profile


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ('name', 'nominal'),
    (
        ('tight', 0.70),
        ('balanced', 0.80),
        ('conservative', 0.90),
        ('vendor', 0.90),
    ),
)
def test_clearance_profile_nominal_width(name: str, nominal: float) -> None:
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / f'clearance_{name}.yaml'
    )
    assert profile.native_hard_width == pytest.approx(0.60)
    assert profile.native_nominal_width == pytest.approx(nominal)
    assert profile.classify(0.50) == 'PHYSICALLY_BLOCKED'
    assert profile.classify(0.55) == 'GUARD_BLOCKED'
    assert profile.classify(nominal) == 'NOMINAL_CANDIDATE'


def test_m20_profile_hard_envelope_covers_body_without_widening_nominal_band(
) -> None:
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / 'clearance_m20.yaml'
    )
    assert profile.body_radius == pytest.approx(0.28)
    assert profile.body_offset == pytest.approx(0.18)
    assert profile.optimizer_clearance == pytest.approx(0.17)
    assert profile.scan_hard_length == pytest.approx(0.92)
    assert profile.scan_hard_width == pytest.approx(0.56)
    assert profile.scan_length_margin == pytest.approx(0.10)
    assert profile.scan_width_margin == pytest.approx(0.05)
    assert profile.scan_covers_physical_body is True
    assert profile.native_hard_width == pytest.approx(0.60)
    assert profile.native_nominal_width == pytest.approx(0.90)


def test_balanced_profile_marks_subnominal_widths_as_marginal() -> None:
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / 'clearance_balanced.yaml'
    )
    assert profile.classify(0.65) == 'MARGINAL'
    assert profile.classify(0.70) == 'MARGINAL'
    assert profile.classify(0.75) == 'MARGINAL'
    assert profile.classify(0.80) == 'NOMINAL_CANDIDATE'
    assert profile.raster_guard_exclusive_width(0.05) == pytest.approx(
        0.70
    )
    assert (
        profile.classify(0.70, map_resolution=0.05)
        == 'GRID_GUARD_BLOCKED'
    )
    assert profile.classify(0.75, map_resolution=0.05) == 'MARGINAL'


def test_profile_name_parsing_remains_python38_compatible() -> None:
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / 'clearance_m20.yaml'
    )
    assert profile.name == 'm20'
    source = (
        PACKAGE_ROOT / 'm20_scan_navigation' / 'clearance.py'
    ).read_text(encoding='utf-8')
    code_lines = (line.split('#', 1)[0] for line in source.splitlines())
    assert '.removeprefix(' not in '\n'.join(code_lines)
