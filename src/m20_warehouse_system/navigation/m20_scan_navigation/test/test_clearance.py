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

# =============================================================================
# test_clearance.py —— M20 直线通道净空档位的回归测试
# 所属模块：m20_scan_navigation / test
# 职责：验证 clearance_*.yaml 配置档经 load_clearance_profile() 加载后，
#       其硬宽度/标称宽度/分类结果与预期一致（回归保护，防止调参破坏几何
#       契约）。同时保证 clearance.py 保持 Python 3.8 兼容。
# =============================================================================

"""Regression checks for explicit M20 straight-passage profiles."""

from pathlib import Path      # 跨平台路径处理

import pytest                 # pytest 测试框架（参数化等）

from m20_scan_navigation.clearance import load_clearance_profile   # 被测函数


PACKAGE_ROOT = Path(__file__).resolve().parents[1]   # 本包根目录


@pytest.mark.parametrize(
    ('name', 'nominal'),      # 参数化：配置档名 + 期望标称宽度
    (
        ('tight', 0.70),          # 窄通道档：标称 0.70 米
        ('balanced', 0.80),       # 平衡档：标称 0.80 米
        ('conservative', 0.90),   # 保守档：标称 0.90 米
        ('vendor', 0.90),         # 厂商档：标称 0.90 米
    ),
)
def test_clearance_profile_nominal_width(name: str, nominal: float) -> None:
    """各档位的硬宽度/标称宽度与分类结果必须符合预期。"""
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / f'clearance_{name}.yaml'   # 加载对应配置文件
    )
    # 硬下限统一为 0.60 米（0.25 半径双圆柱 + 0.05 守卫余量的包络）
    assert profile.native_hard_width == pytest.approx(0.60)
    # 标称宽度 = 参数化的期望值
    assert profile.native_nominal_width == pytest.approx(nominal)
    # 0.50 米低于机身宽度 0.51 -> 物理阻挡
    assert profile.classify(0.50) == 'PHYSICALLY_BLOCKED'
    # 0.55 米低于硬下限 -> 守卫阻挡
    assert profile.classify(0.55) == 'GUARD_BLOCKED'
    # 达到标称宽度 -> 标称候选
    assert profile.classify(nominal) == 'NOMINAL_CANDIDATE'


def test_m20_profile_hard_envelope_covers_body_without_widening_nominal_band(
) -> None:
    """M20 档的硬包络必须覆盖机身，且不抬高标称带。"""
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / 'clearance_m20.yaml'
    )
    # 从配置读出的几何参数
    assert profile.body_radius == pytest.approx(0.28)          # 双圆柱半径
    assert profile.body_offset == pytest.approx(0.18)          # 双圆柱轴向偏移
    assert profile.optimizer_clearance == pytest.approx(0.17)  # 软净空
    # 硬包络尺寸（长 0.92 / 宽 0.56）
    assert profile.scan_hard_length == pytest.approx(0.92)
    assert profile.scan_hard_width == pytest.approx(0.56)
    # 相对 M20 物理尺寸的盈余（长 +0.10 / 宽 +0.05）
    assert profile.scan_length_margin == pytest.approx(0.10)
    assert profile.scan_width_margin == pytest.approx(0.05)
    # 硬双圆必须覆盖物理机身
    assert profile.scan_covers_physical_body is True
    # 硬下限 0.60，标称 0.90（与保守/厂商档一致）
    assert profile.native_hard_width == pytest.approx(0.60)
    assert profile.native_nominal_width == pytest.approx(0.90)


def test_balanced_profile_marks_subnominal_widths_as_marginal() -> None:
    """平衡档必须把低于标称的宽度标记为 MARGINAL。"""
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / 'clearance_balanced.yaml'
    )
    # 0.65/0.70/0.75 均低于标称 0.80 -> 临界
    assert profile.classify(0.65) == 'MARGINAL'
    assert profile.classify(0.70) == 'MARGINAL'
    assert profile.classify(0.75) == 'MARGINAL'
    # 0.80 达到标称 -> 标称候选
    assert profile.classify(0.80) == 'NOMINAL_CANDIDATE'
    # 栅格守卫排他宽度（0.05 分辨率下）= 0.70 米
    assert profile.raster_guard_exclusive_width(0.05) == pytest.approx(
        0.70
    )
    # 给出地图分辨率时：0.70 恰被栅格守卫封死
    assert (
        profile.classify(0.70, map_resolution=0.05)
        == 'GRID_GUARD_BLOCKED'
    )
    # 0.75 超出排他宽度但仍低于标称 -> 临界
    assert profile.classify(0.75, map_resolution=0.05) == 'MARGINAL'


def test_profile_name_parsing_remains_python38_compatible() -> None:
    """档位名解析必须保持 Python 3.8 兼容（不用 removeprefix 语法）。"""
    profile = load_clearance_profile(
        PACKAGE_ROOT / 'config' / 'clearance_m20.yaml'
    )
    assert profile.name == 'm20'     # 文件名 clearance_m20 -> 名称 m20
    source = (
        PACKAGE_ROOT / 'm20_scan_navigation' / 'clearance.py'
    ).read_text(encoding='utf-8')
    # 剥离 # 注释后的代码中不得出现 removeprefix 调用（Python 3.8 不支持）
    code_lines = (line.split('#', 1)[0] for line in source.splitlines())
    assert '.removeprefix(' not in '\n'.join(code_lines)
