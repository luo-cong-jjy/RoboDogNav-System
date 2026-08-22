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
# clearance.py —— M20 导航栈的"直线通道净空（clearance）"计算模块
# 所属模块：m20_scan_navigation（纯计算，无 ROS 依赖，可单独运行测试）
# 职责：
#   1. 定义 ClearanceProfile 数据类：把 SCAN 规划器的双圆柱几何参数与在线
#      碰撞守卫的足迹参数统一起来，推导出各种"硬宽度/标称宽度"；
#   2. load_clearance_profile()：从 clearance_*.yaml 参数文件加载并校验参数；
#   3. classify()：把一条直线通道的宽度分类为 物理阻挡/栅格守卫阻挡/守卫阻挡/
#      临界(MARGINAL)/标称候选(NOMINAL_CANDIDATE)。
# 背景：SCAN 规划器用"双圆柱"（半径 body_radius、轴向偏移 body_offset）近似
#       M20 机身；在线碰撞守卫用"足迹半径 + 安全余量"做独立硬停包络。
# =============================================================================

"""Straight-passage clearance calculations for the M20 navigation stack."""

from dataclasses import dataclass   # 数据类装饰器（自动生成 __init__/__repr__ 等）
from pathlib import Path            # 跨平台路径处理
import math                         # 数学函数（ceil 等）
from typing import Iterable, List, Mapping, Optional, Tuple   # 类型标注

import yaml                         # YAML 解析（加载 clearance_*.yaml 参数）


# M20 机身物理尺寸（米）：长 0.82、宽 0.51（来自实测/几何标定）
M20_BODY_LENGTH_M = 0.82
M20_BODY_WIDTH_M = 0.51


@dataclass(frozen=True)
class ClearanceProfile:
    """SCAN 原生规划器与在线碰撞守卫共用的净空参数集合。

    body_radius / body_offset / optimizer_clearance 来自 scan_planner_node 的
    grid_map.double_cylinder_radius / double_cylinder_offset / optimization.dist0；
    guard_footprint_radius / guard_safety_margin 来自 m20_collision_guard。
    frozen=True 使实例不可变，便于安全共享与哈希。
    """

    name: str                       # 配置文件标识名（如 m20/balanced/vendor）
    body_radius: float              # 双圆柱半径（米）
    body_offset: float              # 双圆柱轴向偏移（米）
    optimizer_clearance: float      # 优化器软净空 dist0（米）
    guard_footprint_radius: float   # 碰撞守卫足迹半径（米）
    guard_safety_margin: float      # 碰撞守卫安全余量（米）

    @property
    def physical_minimum_width(self) -> float:
        """仅考虑机身本身的几何极限：完美对正的直线通过最小宽度。"""
        return M20_BODY_WIDTH_M

    @property
    def scan_hard_width(self) -> float:
        """SCAN 双圆柱横截面所表示的宽度（直径）。"""
        return 2.0 * self.body_radius

    @property
    def scan_hard_length(self) -> float:
        """两个有向 SCAN 圆柱所表示的长度（轴向）。"""
        return 2.0 * (self.body_offset + self.body_radius)

    @property
    def scan_width_margin(self) -> float:
        """SCAN 硬包络宽度相对 M20 物理宽度的盈余。"""
        return self.scan_hard_width - M20_BODY_WIDTH_M

    @property
    def scan_length_margin(self) -> float:
        """SCAN 硬包络长度相对 M20 物理长度的盈余。"""
        return self.scan_hard_length - M20_BODY_LENGTH_M

    @property
    def scan_covers_physical_body(self) -> bool:
        """硬双圆是否覆盖机身的轴向范围（两方向均无负盈余）。"""
        return (
            self.scan_width_margin >= -1e-9
            and self.scan_length_margin >= -1e-9
        )

    @property
    def guard_hard_width(self) -> float:
        """独立静态地图守卫必须停车的最小通道宽度。"""
        return 2.0 * (
            self.guard_footprint_radius + self.guard_safety_margin
        )

    @property
    def scan_preferred_width(self) -> float:
        """SCAN 障碍代价所期望的标称宽度。"""
        return 2.0 * (
            self.body_radius + self.optimizer_clearance
        )

    @property
    def native_hard_width(self) -> float:
        """常规原生 SCAN 系统的硬下限：取物理/SCAN/守卫三者的最大值。"""
        return max(
            self.physical_minimum_width,
            self.scan_hard_width,
            self.guard_hard_width,
        )

    @property
    def native_nominal_width(self) -> float:
        """通道达到"标称"前的规划器目标宽度。"""
        return max(self.native_hard_width, self.scan_preferred_width)

    def raster_guard_exclusive_width(self, resolution: float) -> float:
        """
        返回守卫"排他性"占据栅格边界（保守值，米）。

        当前守卫先按整格膨胀、再采样机身中心所在格。每面墙多取 1 个格可
        覆盖障碍栅格化与中心格量化误差。门洞必须严格宽于该值；受控地图以
        此决定第一个可表示的通行宽度。
        """
        cell_size = float(resolution)
        if cell_size <= 0.0:
            raise ValueError('map resolution must be positive')   # 分辨率必须为正
        guard_radius = (
            self.guard_footprint_radius + self.guard_safety_margin
        )
        kernel_cells = int(math.ceil(guard_radius / cell_size))   # 膨胀核格数（向上取整）
        # 两侧各 (kernel_cells + 1) 格：多出的 1 格用于覆盖栅格化误差
        return 2.0 * (kernel_cells + 1) * cell_size

    def classify(
        self,
        passage_width: float,
        map_resolution: Optional[float] = None,
    ) -> str:
        """把一个标称直线宽度分类，但不声称任何动力学可行性。

        返回状态（由紧到松）：
          PHYSICALLY_BLOCKED  物理上不可能通过（小于机身宽度）
          GRID_GUARD_BLOCKED  栅格化后守卫排他宽度已封死通道
          GUARD_BLOCKED       低于硬下限（守卫包络）
          MARGINAL            低于标称宽度但高于硬下限（可行但余量不足）
          NOMINAL_CANDIDATE   达到标称宽度的候选通道
        """
        width = float(passage_width)
        # 低于物理机身宽度：物理阻挡
        if width + 1e-9 < self.physical_minimum_width:
            return 'PHYSICALLY_BLOCKED'
        # 若给出地图分辨率，且宽度不超过栅格守卫排他宽度：栅格守卫阻挡
        if (
            map_resolution is not None
            and width
            <= self.raster_guard_exclusive_width(map_resolution) + 1e-9
        ):
            return 'GRID_GUARD_BLOCKED'
        # 低于硬下限：守卫阻挡
        if width + 1e-9 < self.native_hard_width:
            return 'GUARD_BLOCKED'
        # 低于标称宽度：临界（可行但偏窄）
        if width + 1e-9 < self.native_nominal_width:
            return 'MARGINAL'
        # 达到标称宽度
        return 'NOMINAL_CANDIDATE'


def load_clearance_profile(path: Path) -> ClearanceProfile:
    """加载一份显式 ROS 参数覆盖文件并校验各字段。"""
    profile_path = Path(path).expanduser().resolve()   # 展开 ~ 并取绝对路径
    with profile_path.open('r', encoding='utf-8') as stream:
        root = yaml.safe_load(stream)                  # 解析 YAML
    if not isinstance(root, Mapping):
        raise ValueError(f'{profile_path} must contain a YAML mapping')   # 顶层必须是映射

    def parameters(node_name: str) -> Mapping:
        """取出指定节点下的 ros__parameters 参数表。"""
        node = root.get(node_name)
        if not isinstance(node, Mapping):
            raise ValueError(f'{profile_path} has no {node_name} section')
        values = node.get('ros__parameters')
        if not isinstance(values, Mapping):
            raise ValueError(
                f'{profile_path} has no {node_name}.ros__parameters'
            )
        return values

    # 分别读取 SCAN 规划器节点与碰撞守卫节点的参数
    planner = parameters('scan_planner_node')
    guard = parameters('m20_collision_guard')
    # 从 YAML 深层键名映射到本数据类的数字字段
    numeric_fields = {
        'body_radius': planner.get('grid_map.double_cylinder_radius'),
        'body_offset': planner.get('grid_map.double_cylinder_offset'),
        'optimizer_clearance': planner.get('optimization.dist0'),
        'guard_footprint_radius': guard.get('footprint_radius'),
        'guard_safety_margin': guard.get('safety_margin'),
    }
    # 校验：必须是数字、非布尔、且非负
    for key, value in numeric_fields.items():
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or float(value) < 0.0
        ):
            raise ValueError(
                f'{profile_path}: {key} must be a non-negative number'
            )
    return ClearanceProfile(
        # 名称取自文件名：去掉 "clearance_" 前缀（如 clearance_m20 -> m20）。
        # 注意：不使用 str.removeprefix()，因为真实 M20 主机是 Ubuntu 20.04 /
        # ROS 2 Foxy 自带的 Python 3.8，而 removeprefix 需要 Python 3.9+。
        name=(
            profile_path.stem[len('clearance_'):]
            if profile_path.stem.startswith('clearance_')
            else profile_path.stem
        ),
        **{key: float(value) for key, value in numeric_fields.items()},
    )


def passage_table(
    profile: ClearanceProfile,
    widths: Iterable[float],
    map_resolution: Optional[float] = None,
) -> List[Tuple[float, str]]:
    """返回稳定的 (宽度, 状态) 行，供报告与回归测试使用。"""
    return [
        (
            float(width),
            profile.classify(
                width,
                map_resolution=map_resolution,
            ),
        )
        for width in widths
    ]
