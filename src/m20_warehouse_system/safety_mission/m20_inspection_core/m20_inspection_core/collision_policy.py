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

# ============================================================================
# 【文件职责】collision_policy.py —— 在线占用碰撞预判策略（纯函数）
# 本文件是 m20_inspection_core 的"碰撞预判"纯逻辑模块，供
# collision_guard_node.py 调用。运行时守卫消费 SCAN 的实时 3D 占用体素点云，
# 仅在内存中构建一个以机器人为中心的小型栅格（raster），使既有、经过验证
# 的 M20 足印与滚动恢复检查保持确定性。不涉及 PGM 或地图 YAML。
# 主要功能：
#   1) 在线占用栅格化（rasterize_online_occupancy，按机体高度带过滤）；
#   2) 栅格膨胀（inflate_blocked_grid，保守半径 vs 硬车身半径）；
#   3) 恒定指令前视预测（predict_poses / predict_positions）；
#   4) 名义/实测侧向漂移指令包络（command_drift_variants）；
#   5) 首个阻塞足印采样（first_blocking_footprint / first_blocking_command_envelope）；
#   6) 滚动恢复（safe_rolling_recovery）、光栅外壳逃逸（safe_raster_shell_escape）；
#   7) 恢复清扫/终点校验、连续清零确认与恢复预算（update_* 系列）。
# ============================================================================

"""
Pure online-occupancy collision lookahead policy.

The runtime guard consumes SCAN's live 3-D occupied-voxel cloud.  A small
robot-centred raster is built in memory only so the existing, well-tested M20
footprint and rolling-recovery checks remain deterministic.  No PGM or map
YAML is involved.
"""

from __future__ import annotations  # 延迟注解求值：允许在类型提示中使用 | 与内置泛型

from dataclasses import dataclass, replace  # 数据类与 replace（生成字段修改后的副本）
import math                        # 数学库：hypot/floor/ceil/isfinite 等
from typing import Iterable, Sequence, Tuple  # 类型提示：可迭代/序列/元组

import numpy as np  # 数值数组库：栅格数据的构建与膨胀运算


PlanarCommand = Tuple[float, float, float]  # 平面指令类型别名：(vx, vy, wz)
PlanarPose = Tuple[float, float, float]     # 平面位姿类型别名：(x, y, yaw)


@dataclass(frozen=True)  # 冻结数据类：阻塞采样信息不可变
class BlockingSample:
    """First predicted footprint sample that intersects the blocked grid."""
    # 【中文】第一个与阻塞栅格相交的预测足印采样点（触发停车的"元凶"）

    sample_index: int          # 触发采样的序号（0 表示当前时刻即阻塞）
    time_sec: float            # 触发采样的预测时刻（秒）
    circle: str                # 触发圆：'front'（前圆）/ 'rear'（后圆）
    x: float                   # 触发圆心的 x 坐标（世界系）
    y: float                   # 触发圆心的 y 坐标（世界系）
    yaw: float                 # 触发时刻的航向角（弧度）
    cell_x: int                # 触发栅格单元列号
    cell_y: int                # 触发栅格单元行号
    cause: str                 # 触发原因：'OUT_OF_BOUNDS'（越界）/ 'OCCUPIED'（占用）
    motion_model: str = 'nominal'  # 运动模型：'nominal'（名义）/ 漂移变体名称


@dataclass(frozen=True)  # 冻结数据类：栅格几何参数不可变
class GridGeometry:
    """Metric occupancy-grid geometry used by the safety layer."""
    # 【中文】安全层使用的米制占用栅格几何参数

    width: int                 # 栅格宽度（列数）
    height: int                # 栅格高度（行数）
    resolution: float          # 栅格分辨率（米/格）
    origin_x: float            # 栅格左下角原点的 x 坐标（世界系）
    origin_y: float            # 栅格左下角原点的 y 坐标（世界系）
    edge_tolerance: float = 0.35  # 共享地图边界的边缘容差（米，默认 0.35）

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert a metric position to a grid cell.

        A connected-floor doorway is centred on a shared map edge.  Odometry
        can settle a few centimetres on either side while the active map is
        committed, so clamp a small configurable band to the edge cell.
        Closed edges remain blocked by their occupied boundary cells.  A pose
        beyond the band remains out of bounds and therefore fail-closed.
        """
        # 【中文】把米制位置转换为栅格单元坐标。
        # 连通的楼层门洞以共享地图边界为中心；地图提交时里程计可在边界两侧
        # 偏移几厘米，因此把一小段可配置的容差带钳制到边界单元。
        # 关闭的边界仍由其占用边界单元保持阻塞；超出容差带的位置保持越界，
        # 从而 fail-closed。
        # 【参数】x/y - 米制坐标；【返回】(cell_x, cell_y) 栅格单元坐标
        lower_x = self.origin_x
        lower_y = self.origin_y
        upper_x = self.origin_x + self.width * self.resolution
        upper_y = self.origin_y + self.height * self.resolution
        tolerance = max(0.0, self.edge_tolerance)

        def _axis_cell(  # 单轴单元映射：边界容差带内钳制到边界单元
            value: float,
            lower: float,
            upper: float,
            size: int,
        ) -> int:
            if lower - tolerance <= value < lower:
                return 0
            if upper <= value <= upper + tolerance:
                return size - 1
            return int(math.floor((value - lower) / self.resolution))

        cell_x = _axis_cell(x, lower_x, upper_x, self.width)
        cell_y = _axis_cell(y, lower_y, upper_y, self.height)
        return (
            cell_x,
            cell_y,
        )

    def contains(self, cell_x: int, cell_y: int) -> bool:
        """Return whether a cell index is inside this raster."""
        # 【中文】判断给定单元坐标是否位于本栅格范围内
        # 【参数】cell_x/cell_y - 单元坐标；【返回】True 表示在栅格内
        return (
            0 <= int(cell_x) < self.width
            and 0 <= int(cell_y) < self.height
        )


def rasterize_online_occupancy(
    points: Iterable[Sequence[float]],
    center_xy: Tuple[float, float],
    resolution: float,
    local_size: float,
    minimum_z: float,
    maximum_z: float,
) -> Tuple[np.ndarray, GridGeometry, int]:
    """
    Project SCAN occupied voxels into a robot-centred in-memory raster.

    The input points and ``center_xy`` must use the same world frame.  Only
    voxels intersecting the configured M20 body-height band are projected;
    floor and ceiling returns therefore do not become planar obstacles.
    ``occupied_count`` reports how many finite input voxels were accepted.
    """
    # 【中文】把 SCAN 占用体素投影到以机器人为中心的内存栅格中。
    # 输入点与 center_xy 必须使用同一世界坐标系；仅投影与配置的 M20 机体
    # 高度带相交的体素，因此地面/天花板的回波不会成为平面障碍。
    # occupied_count 报告有多少有限的输入体素被接受。
    # 【参数】points - 输入点序列；center_xy - 栅格中心（世界系）；
    #        resolution - 栅格分辨率；local_size - 局部窗口边长；
    #        minimum_z/maximum_z - 机体高度带的下/上限
    # 【返回】(occupancy 一维占用数组, 栅格几何, 接受点数)
    resolution = float(resolution)
    local_size = float(local_size)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('resolution must be finite and positive')
    if not math.isfinite(local_size) or local_size < 2.0 * resolution:
        raise ValueError('local_size must cover at least two cells')
    if not all(math.isfinite(value) for value in center_xy):
        raise ValueError('center_xy must be finite')
    if not math.isfinite(minimum_z) or not math.isfinite(maximum_z):
        raise ValueError('height limits must be finite')
    if maximum_z < minimum_z:
        raise ValueError('maximum_z must not be below minimum_z')

    cell_count = max(2, int(math.ceil(local_size / resolution)))  # 每边单元数（至少 2）
    extent = cell_count * resolution                              # 栅格实际边长
    geometry = GridGeometry(
        width=cell_count,
        height=cell_count,
        resolution=resolution,
        origin_x=float(center_xy[0]) - 0.5 * extent,  # 中心对齐：原点 = 中心 - 半边长
        origin_y=float(center_xy[1]) - 0.5 * extent,
        # This is a moving local safety window.  Reaching its edge means the
        # online SCAN map is not covering the predicted motion, so fail closed.
        # 【中文】这是移动的局部安全窗口；预测运动到达其边缘意味着在线 SCAN
        # 地图未覆盖该运动，因此 fail-closed（边缘容差为 0）。
        edge_tolerance=0.0,
    )
    occupancy = np.zeros(cell_count * cell_count, dtype=np.int8)  # 占用数组（一维，初值 0）
    accepted = 0
    for point in points:
        if len(point) < 3:
            continue
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue  # 丢弃非有限点（NaN/Inf）
        if z < minimum_z or z > maximum_z:
            continue  # 丢弃高度带之外的点（地面/天花板）
        cell_x, cell_y = geometry.cell(x, y)
        if not geometry.contains(cell_x, cell_y):
            continue  # 丢弃窗口之外的点
        occupancy[cell_y * geometry.width + cell_x] = 100  # 标记占用（值 100）
        accepted += 1
    return occupancy, geometry, accepted


def inflate_blocked_grid(
    values: Sequence[int],
    geometry: GridGeometry,
    occupied_threshold: int,
    radius: float,
) -> np.ndarray:
    """Inflate occupied and unknown cells for the M20 safety footprint."""
    # 【中文】为 M20 安全足印膨胀占用/未知单元（按圆形核膨胀）
    # 【参数】values - 一维栅格值；geometry - 栅格几何；occupied_threshold - 占用阈值；
    #        radius - 膨胀半径（米）
    # 【返回】膨胀后的布尔阻塞栅格（True 表示阻塞）
    raw = np.asarray(values, dtype=np.int16).reshape(
        geometry.height, geometry.width
    )
    blocked = np.logical_or(raw < 0, raw >= occupied_threshold)  # 未知(<0) 或占用(>=阈值)
    cells = int(math.ceil(max(0.0, radius) / geometry.resolution))  # 膨胀核半径（格数）
    if cells == 0:
        return blocked
    # Map-edge collision is still enforced by trajectory_is_blocked.  Padding
    # with occupied cells here would incorrectly seal an intentional doorway
    # whose centre lies on the shared F1/F2 boundary.
    # 【中文】地图边界碰撞仍由 trajectory_is_blocked 强制；此处若用占用单元
    # 填充（padding）会错误封死位于共享 F1/F2 边界上的有意门洞。
    padded = np.pad(blocked, cells, mode='constant', constant_values=False)
    inflated = np.zeros_like(blocked)
    for dy in range(-cells, cells + 1):  # 圆形核：逐偏移叠加膨胀
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy > cells * cells:
                continue  # 圆形核外的偏移跳过
            y0 = cells + dy
            x0 = cells + dx
            inflated |= padded[
                y0:y0 + geometry.height,
                x0:x0 + geometry.width,
            ]
    return inflated


def conservative_raster_radius(
    footprint_radius: float,
    resolution: float,
) -> float:
    """
    Add half a cell diagonal to a continuous footprint before rasterizing.

    A metric circle centre can lie at a cell corner while collision lookup
    uses only the cell index.  Without this allowance a continuous overlap of
    up to half the cell diagonal can still be reported clear.
    """
    # 【中文】在栅格化之前向连续足印半径加上半格对角线长度。
    # 米制圆圆心可能落在格角上，而碰撞查询只使用格索引；若不加此余量，
    # 最多半格对角线的连续重叠仍可能被误报为畅通。
    # 【参数】footprint_radius - 连续足印半径；resolution - 栅格分辨率
    # 【返回】保守栅格化半径（足印半径 + 半格对角线）
    return max(0.0, float(footprint_radius)) + (
        max(0.0, float(resolution)) / math.sqrt(2.0)
    )


def hard_body_raster_radius(
    footprint_radius: float,
    resolution: float,
) -> float:
    """Rasterize the physical body without the conservative half-cell shell."""
    # 【中文】栅格化物理车身：扣除保守半格外壳（仅物理半径）
    # 【参数】footprint_radius - 物理足印半径；resolution - 栅格分辨率
    # 【返回】硬车身栅格化半径（足印半径 - 半格对角线）
    return max(
        0.0,
        float(footprint_radius)
        - max(0.0, float(resolution)) / math.sqrt(2.0),
    )


def predict_poses(
    x: float,
    y: float,
    yaw: float,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
) -> Iterable[PlanarPose]:
    """Yield constant-command body poses across a short safety horizon."""
    # 【中文】在短安全前视内按恒定指令生成机体位姿序列（运动学积分）
    # 【参数】x/y/yaw - 起始位姿；command - (vx, vy, wz) 平面指令；
    #        horizon - 前视时长；sample_period - 采样周期
    # 【返回】位姿生成器：先输出起始位姿，再按周期逐步积分输出预测位姿
    vx, vy, wz = command
    period = max(0.01, sample_period)
    steps = max(1, int(math.ceil(max(0.0, horizon) / period)))
    yield x, y, yaw
    for _ in range(steps):
        world_vx = math.cos(yaw) * vx - math.sin(yaw) * vy  # 机体系速度 -> 世界系
        world_vy = math.sin(yaw) * vx + math.cos(yaw) * vy
        x += world_vx * period
        y += world_vy * period
        yaw += wz * period
        yield x, y, yaw


def predict_positions(
    x: float,
    y: float,
    yaw: float,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
) -> Iterable[Tuple[float, float]]:
    """Yield body-centre positions for legacy single-circle callers."""
    # 【中文】为旧版单圆调用者生成机体中心位置序列（丢弃 yaw）
    # 【参数】同 predict_poses
    # 【返回】(x, y) 位置生成器
    for pose_x, pose_y, _ in predict_poses(
        x, y, yaw, command, horizon, sample_period
    ):
        yield pose_x, pose_y


def first_blocking_footprint(
    blocked: np.ndarray,
    geometry: GridGeometry,
    poses: Iterable[PlanarPose],
    footprint_offset: float,
    sample_period: float,
) -> BlockingSample | None:
    """Return the first blocked front/rear circle centre along a trajectory."""
    # 【中文】返回轨迹上第一个被阻塞的前/后圆圆心采样（双圆模型）
    # 【参数】blocked - 阻塞栅格；geometry - 栅格几何；poses - 预测位姿序列；
    #        footprint_offset - 前后圆相对机体中心的偏移；sample_period - 采样周期
    # 【返回】BlockingSample 或 None（全程畅通）
    offset = max(0.0, float(footprint_offset))
    period = max(0.01, float(sample_period))
    for sample_index, (x, y, yaw) in enumerate(poses):
        heading_x = math.cos(yaw)
        heading_y = math.sin(yaw)
        circles = (  # 前后两个圆心的候选
            ('front', x + offset * heading_x, y + offset * heading_y),
            ('rear', x - offset * heading_x, y - offset * heading_y),
        )
        for name, circle_x, circle_y in circles:
            cell_x, cell_y = geometry.cell(circle_x, circle_y)
            out_of_bounds = (
                cell_x < 0
                or cell_y < 0
                or cell_x >= geometry.width
                or cell_y >= geometry.height
            )
            occupied = (
                not out_of_bounds and blocked[cell_y, cell_x]
            )
            if out_of_bounds or occupied:  # 越界或占用即触发阻塞
                return BlockingSample(
                    sample_index=sample_index,
                    time_sec=sample_index * period,
                    circle=name,
                    x=circle_x,
                    y=circle_y,
                    yaw=yaw,
                    cell_x=cell_x,
                    cell_y=cell_y,
                    cause=(
                        'OUT_OF_BOUNDS'
                        if out_of_bounds
                        else 'OCCUPIED'
                    ),
                )
    return None


def command_drift_variants(
    command: PlanarCommand,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
    reference_yaw_rate: float,
) -> Tuple[Tuple[str, PlanarCommand], ...]:
    """Return nominal and measured M20 lateral-drift command variants."""
    # 【中文】返回名义指令与实测 M20 侧向漂移指令变体
    # 【参数】command - 原始指令；positive/negative_yaw_lateral_drift - 正/负转向
    #        侧向漂移上限；opposite_lateral_uncertainty - 反向漂移不确定性；
    #        reference_yaw_rate - 参考角速度（用于按比例缩放漂移）
    # 【返回】(模型名, 指令) 元组序列：至少包含 ('nominal', 原指令)
    vx, vy, wz = command
    variants = [('nominal', (vx, vy, wz))]
    yaw_reference = max(1.0e-6, abs(float(reference_yaw_rate)))
    yaw_fraction = min(1.0, abs(float(wz)) / yaw_reference)  # 角速度占比
    if yaw_fraction <= 1.0e-6:
        return tuple(variants)

    measured_limit = (  # 按转向方向选择实测漂移上限
        positive_yaw_lateral_drift
        if wz >= 0.0
        else negative_yaw_lateral_drift
    )
    measured = max(0.0, float(measured_limit)) * yaw_fraction
    opposite = (
        max(0.0, float(opposite_lateral_uncertainty)) * yaw_fraction
    )
    if measured > 1.0e-6:
        variants.append(
            ('measured_lateral_drift', (vx, vy + measured, wz))  # 实测漂移变体
        )
    if opposite > 1.0e-6:
        variants.append(
            ('opposite_lateral_uncertainty', (vx, vy - opposite, wz))  # 反向漂移变体
        )
    return tuple(variants)


def first_blocking_command_envelope(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
    reference_yaw_rate: float,
) -> BlockingSample | None:
    """Check nominal and measured-drift sweeps, returning the earliest hit."""
    # 【中文】同时检查名义与实测漂移扫掠，返回最早命中（最危险）的阻塞采样
    # 【参数】见各被调用函数；【返回】最早的 BlockingSample 或 None
    earliest = None
    for model, variant in command_drift_variants(
        command,
        positive_yaw_lateral_drift,
        negative_yaw_lateral_drift,
        opposite_lateral_uncertainty,
        reference_yaw_rate,
    ):
        sample = first_blocking_footprint(
            blocked,
            geometry,
            predict_poses(*pose, variant, horizon, sample_period),
            footprint_offset,
            sample_period,
        )
        if sample is None:
            continue
        sample = replace(sample, motion_model=model)  # 记录触发模型名
        if earliest is None or sample.sample_index < earliest.sample_index:
            earliest = sample
    return earliest


def recovery_sweep_is_clear(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    recovery: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
) -> bool:
    """Check the complete conservative sweep of one recovery command."""
    # 【中文】检查一条恢复指令的完整保守扫掠是否畅通（含漂移变体）
    # 【参数】blocked - 阻塞栅格；geometry - 栅格几何；pose - 当前位姿；
    #        recovery - 恢复指令；horizon/sample_period - 前视与采样；
    #        footprint_offset - 足印偏移；*_lateral_drift - 漂移参数
    # 【返回】True 表示所有变体扫掠均畅通
    forward, _, yaw_rate = recovery
    lateral_speeds = [0.0]
    if abs(yaw_rate) > 1.0e-6:  # 有转向时补充实测漂移与反向不确定性两个横向速度
        measured_drift = (
            max(0.0, float(positive_yaw_lateral_drift))
            if yaw_rate > 0.0
            else max(0.0, float(negative_yaw_lateral_drift))
        )
        lateral_speeds.extend(
            (
                measured_drift,
                -max(0.0, float(opposite_lateral_uncertainty)),
            )
        )
    for lateral_speed in lateral_speeds:
        swept_command = (forward, lateral_speed, yaw_rate)
        recovery_block = first_blocking_footprint(
            blocked,
            geometry,
            predict_poses(
                *pose,
                swept_command,
                horizon,
                sample_period,
            ),
            footprint_offset,
            sample_period,
        )
        if recovery_block is not None:
            return False
    return True


def recovery_endpoint_is_clear(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    recovery: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
) -> bool:
    """Require every recovery motion variant to end in clear space."""
    # 【中文】要求恢复运动的每个变体都结束于自由空间（终点校验）
    # 【参数】同 recovery_sweep_is_clear；【返回】True 表示所有变体终点畅通
    forward, _, yaw_rate = recovery
    lateral_speeds = [0.0]
    if abs(yaw_rate) > 1.0e-6:
        measured_drift = (
            max(0.0, float(positive_yaw_lateral_drift))
            if yaw_rate > 0.0
            else max(0.0, float(negative_yaw_lateral_drift))
        )
        lateral_speeds.extend(
            (
                measured_drift,
                -max(0.0, float(opposite_lateral_uncertainty)),
            )
        )
    for lateral_speed in lateral_speeds:
        poses = list(
            predict_poses(
                *pose,
                (forward, lateral_speed, yaw_rate),
                horizon,
                sample_period,
            )
        )
        if first_blocking_footprint(  # 只检查终点位姿（最后一个采样）
            blocked,
            geometry,
            [poses[-1]],
            footprint_offset,
            sample_period,
        ) is not None:
            return False
    return True


def safe_raster_shell_escape(
    conservative_blocked: np.ndarray,
    hard_body_blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    speed: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
) -> PlanarCommand | None:
    """
    Leave a raster-only safety shell without authorizing physical overlap.

    Normal commands continue to use the conservative grid. This bounded seam
    applies only when that grid marks the current double-circle footprint but
    the hard physical-body grid does not. A straight direction is accepted
    only when its complete hard-body sweep is clear and its endpoint has
    returned to conservative free space. Reverse is tried first because the
    validated M20 policy can leave a front-side shell without zero-radius yaw.
    """
    # 【中文】在"仅光栅外壳"被保守栅格阻塞但硬车身栅格未阻塞时，允许一次
    # 有界的直线逃逸，且不授权物理重叠。普通指令仍使用保守栅格；此接缝
    # 仅适用于：保守栅格标记当前双圆足印、但硬车身栅格未标记的情形。
    # 仅当某直线方向的完整硬车身扫掠畅通、且终点回到保守自由空间时才接受。
    # 先尝试后退，因为经过验证的 M20 策略无需零半径旋转即可离开前侧外壳。
    # 【参数】conservative_blocked - 保守（含外壳）阻塞栅格；hard_body_blocked -
    #        硬车身阻塞栅格；其余同前
    # 【返回】逃逸指令（直线前进/后退）或 None（不可逃逸）
    current_pose = [pose]
    if first_blocking_footprint(  # 当前保守栅格未阻塞：无需逃逸
        conservative_blocked,
        geometry,
        current_pose,
        footprint_offset,
        sample_period,
    ) is None:
        return None
    if first_blocking_footprint(  # 硬车身栅格也阻塞：绝不授权运动
        hard_body_blocked,
        geometry,
        current_pose,
        footprint_offset,
        sample_period,
    ) is not None:
        return None
    escape_speed = abs(float(speed))
    if escape_speed <= 1.0e-3:
        return None
    for recovery in (
        (-escape_speed, 0.0, 0.0),  # 先试直线后退
        (escape_speed, 0.0, 0.0),   # 再试直线前进
    ):
        if not recovery_sweep_is_clear(  # 硬车身完整扫掠必须畅通
            hard_body_blocked,
            geometry,
            pose,
            recovery,
            horizon,
            sample_period,
            footprint_offset,
            positive_yaw_lateral_drift,
            negative_yaw_lateral_drift,
            opposite_lateral_uncertainty,
        ):
            continue
        if recovery_endpoint_is_clear(  # 终点必须回到保守自由空间
            conservative_blocked,
            geometry,
            pose,
            recovery,
            horizon,
            sample_period,
            footprint_offset,
            positive_yaw_lateral_drift,
            negative_yaw_lateral_drift,
            opposite_lateral_uncertainty,
        ):
            return recovery
    return None


def update_clear_confirmation(
    first_clear_time: float | None,
    now: float,
    confirmation_sec: float,
) -> Tuple[float | None, bool]:
    """Track whether recovery has seen a continuous clear interval."""
    # 【中文】跟踪恢复过程是否已出现连续的畅通时间间隔（用于释放确认）
    # 【参数】first_clear_time - 首次畅通时刻（None 表示尚未开始）；now - 当前时刻；
    #        confirmation_sec - 所需确认时长（<=0 表示立即确认）
    # 【返回】(更新后的首次畅通时刻, 是否已确认)
    duration = max(0.0, float(confirmation_sec))
    current_time = float(now)
    if duration <= 0.0:
        return None, True
    if first_clear_time is None or current_time < first_clear_time:
        first_clear_time = current_time
    confirmed = current_time - first_clear_time >= duration
    return first_clear_time, confirmed


def update_recovery_budget(
    start_time: float,
    start_xy: Tuple[float, float],
    progress_time: float,
    progress_xy: Tuple[float, float],
    now: float,
    current_xy: Tuple[float, float],
    max_active_sec: float,
    max_displacement_m: float,
    progress_timeout_sec: float,
    minimum_progress_m: float,
) -> Tuple[float, Tuple[float, float], str | None]:
    """Update bounded-recovery progress and return an exhaustion reason."""
    # 【中文】更新有界恢复的进度，返回恢复预算耗尽的原因（若有）
    # 【参数】start_time/start_xy - 恢复起始时刻与位置；progress_time/progress_xy -
    #        最近进度检查点；now/current_xy - 当前时刻与位置；
    #        max_active_sec - 最大活动时长；max_displacement_m - 最大总位移；
    #        progress_timeout_sec - 无进度超时；minimum_progress_m - 最小进度阈值
    # 【返回】(progress_time, progress_xy, 耗尽原因或 None)
    scalars = (
        start_time,
        start_xy[0],
        start_xy[1],
        progress_time,
        progress_xy[0],
        progress_xy[1],
        now,
        current_xy[0],
        current_xy[1],
    )
    if not all(math.isfinite(float(value)) for value in scalars):
        return progress_time, progress_xy, 'INVALID_STATE'
    if now < start_time or now < progress_time:
        return progress_time, progress_xy, 'CLOCK_REWIND'

    total_displacement = math.hypot(  # 总位移：当前相对恢复起点
        current_xy[0] - start_xy[0],
        current_xy[1] - start_xy[1],
    )
    if total_displacement >= max(0.0, float(max_displacement_m)):
        return progress_time, progress_xy, 'DISTANCE_LIMIT'
    if now - start_time >= max(0.0, float(max_active_sec)):
        return progress_time, progress_xy, 'TIME_LIMIT'

    checkpoint_displacement = math.hypot(  # 相对上一检查点的位移
        current_xy[0] - progress_xy[0],
        current_xy[1] - progress_xy[1],
    )
    if checkpoint_displacement >= max(0.0, float(minimum_progress_m)):
        progress_time = now        # 达到最小进度：刷新检查点
        progress_xy = current_xy
    elif now - progress_time >= max(  # 超过检查点超时：无进展
        0.0, float(progress_timeout_sec)
    ):
        return progress_time, progress_xy, 'NO_PROGRESS'
    return progress_time, progress_xy, None


def safe_rolling_recovery(
    blocked: np.ndarray,
    geometry: GridGeometry,
    pose: PlanarPose,
    command: PlanarCommand,
    horizon: float,
    sample_period: float,
    footprint_offset: float,
    forward_speed: float,
    positive_yaw_lateral_drift: float,
    negative_yaw_lateral_drift: float,
    opposite_lateral_uncertainty: float,
    minimum_yaw_rate: float,
) -> PlanarCommand | None:
    """
    Return a collision-checked M20 command that can break a predicted hold.

    The official M20 policy used by this project is not reliable under a
    zero-forward pure-yaw command.  Recovery therefore uses the validated
    positive-forward rolling envelope.  The opposite yaw direction is tried
    first to turn away from the rejected sweep.  Nominal and measured
    direction-dependent lateral-drift trajectories must remain clear.  A
    small opposite drift also covers residual uncertainty.  A reverse
    tracking request first checks a zero-yaw forward command; this lets the
    robot move inward from a shared map edge without continuing the rejected
    reverse turn. Straight reverse remains the final checked escape.  Reverse
    tracking may use only these zero-yaw checked escapes because the M20
    reverse-yaw drift envelope is not validated.  A currently blocked
    footprint is never recoverable by this policy.
    """
    # 【中文】返回一条经过碰撞检查、可打破预测停车（hold）的 M20 指令。
    # 本项目采用的官方 M20 策略在"零前进 + 纯旋转"指令下不可靠，因此恢复
    # 使用经过验证的正向前进滚动包络：先尝试相反转向以避开被拒绝的扫掠；
    # 名义与实测的、方向相关的侧向漂移轨迹必须保持畅通；小的反向漂移还
    # 覆盖残余不确定性。反向追踪请求先检查零航向的前进指令，使机器人可从
    # 共享地图边界向内移动而不继续被拒绝的反向转弯；直线后退仍是最终受检
    # 逃逸。反向追踪只能使用这些零航向受检逃逸，因为 M20 反向航向漂移包络
    # 未经验证。当前足印已被阻塞的情形本策略永远不可恢复。
    # 【参数】blocked - 阻塞栅格；geometry - 栅格几何；pose - 当前位姿；
    #        command - 被拒绝的原指令；forward_speed - 恢复前进速度；
    #        minimum_yaw_rate - 最小转向角速度阈值；其余同前
    # 【返回】恢复指令或 None（无可用恢复）
    x, y, yaw = pose
    current = first_blocking_footprint(  # 当前足印已阻塞：不可恢复
        blocked,
        geometry,
        [(x, y, yaw)],
        footprint_offset,
        sample_period,
    )
    if current is not None:
        return None
    requested_yaw = float(command[2])
    recovery_forward = float(forward_speed)
    if (
        abs(float(command[0])) <= 1.0e-3  # 原指令无前进分量
        or recovery_forward <= 1.0e-3     # 恢复前进速度非法
    ):
        return None

    if (  # 前进 + 明显转向：尝试相反方向、再同方向的滚动转弯
        float(command[0]) > 1.0e-3
        and abs(requested_yaw) >= max(1.0e-3, minimum_yaw_rate)
    ):
        for yaw_rate in (-requested_yaw, requested_yaw):
            recovery = (recovery_forward, 0.0, yaw_rate)
            if recovery_sweep_is_clear(
                blocked,
                geometry,
                pose,
                recovery,
                horizon,
                sample_period,
                footprint_offset,
                positive_yaw_lateral_drift,
                negative_yaw_lateral_drift,
                opposite_lateral_uncertainty,
            ):
                return recovery

    if float(command[0]) < -1.0e-3:  # 原指令为后退：先检查零航向直线前进
        straight_forward = (recovery_forward, 0.0, 0.0)
        if recovery_sweep_is_clear(
            blocked,
            geometry,
            pose,
            straight_forward,
            horizon,
            sample_period,
            footprint_offset,
            positive_yaw_lateral_drift,
            negative_yaw_lateral_drift,
            opposite_lateral_uncertainty,
        ):
            return straight_forward

    # The cold-start envelope test showed that -0.35 m/s remains posture
    # stable while producing enough measured reverse motion to leave a
    # front-blocked planning boundary.  Keep yaw zero here: reverse-yaw has a
    # larger, direction-dependent lateral drift and is unnecessary for this
    # escape.  As with rolling recovery, authorization is valid only for the
    # current pose and is recomputed by the guard on every update.
    # 【中文】冷启动包络测试表明 -0.35 m/s 保持姿态稳定，同时能产生足够的
    # 实测反向运动以离开前向阻塞的规划边界。此处保持零航向：反向转向的
    # 侧向漂移更大且对本逃逸不必要。与滚动恢复一样，授权仅对当前位姿
    # 有效，守卫每次更新都会重新计算。
    reverse = (-recovery_forward, 0.0, 0.0)  # 最终受检逃逸：直线后退
    if recovery_sweep_is_clear(
        blocked,
        geometry,
        pose,
        reverse,
        horizon,
        sample_period,
        footprint_offset,
        positive_yaw_lateral_drift,
        negative_yaw_lateral_drift,
        opposite_lateral_uncertainty,
    ):
        return reverse
    return None


def trajectory_is_blocked(
    blocked: np.ndarray,
    geometry: GridGeometry,
    positions: Iterable[Tuple[float, float]],
) -> bool:
    """Return true when any predicted position enters a blocked cell."""
    # 【中文】若任一预测位置进入阻塞单元（或越界）则返回 True（旧版单圆调用）
    # 【参数】blocked - 阻塞栅格；geometry - 栅格几何；positions - 预测位置序列
    # 【返回】True 表示轨迹被阻塞
    for x, y in positions:
        cell_x, cell_y = geometry.cell(x, y)
        if (
            cell_x < 0
            or cell_y < 0
            or cell_x >= geometry.width
            or cell_y >= geometry.height
            or blocked[cell_y, cell_x]
        ):
            return True
    return False
