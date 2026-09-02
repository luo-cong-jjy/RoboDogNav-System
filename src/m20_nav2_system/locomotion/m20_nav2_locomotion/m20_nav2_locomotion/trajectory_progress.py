# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：trajectory_progress.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：按"实测空间进度"跟踪 SCAN B 样条轨迹的纯几何模块。
#   - Bspline2D：不依赖第三方库的 de Boor 求值器镜像（与 SCAN 的约定一致）；
#   - SampledPath：对 B 样条密集采样得到的"弧长视角"路径（最近点投影、按弧长插值）；
#   - SpatialProgressFollower：根据实测位姿与固定空间前视距离生成滚动指令
#     （含终点减速、倒车跟踪滞回、偏航比例控制）。
# 本模块不依赖 ROS，可直接单元测试。
# ============================================================================
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

"""Pure geometry for tracking a SCAN B-spline by measured spatial progress."""
# 【中文注释】模块说明：按实测空间进度跟踪 SCAN B 样条的纯几何实现。

from dataclasses import dataclass   # 数据类装饰器
import math                         # 数学库：atan2、hypot、ceil 等
from typing import Sequence, Tuple  # 类型提示：序列、元组


Point2D = Tuple[float, float]               # 【中文注释】二维点类型别名：(x, y)
PlanarCommand = Tuple[float, float, float]  # 【中文注释】平面速度指令类型别名：(vx, vy, yaw)


def normalize_angle(angle: float) -> float:
    """Return an angle in [-pi, pi]."""
    # 【中文注释】把角度归一化到 [-pi, pi] 区间（用 atan2(sin, cos) 实现）。
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_point(point: Sequence[float]) -> Point2D:
    # 【中文注释】把序列转换为有限二维点，校验至少含 x、y 且均为有限数。
    if len(point) < 2:
        raise ValueError('B-spline control points must have x and y')
    result = (float(point[0]), float(point[1]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError('B-spline control points must be finite')
    return result


class Bspline2D:
    """Small dependency-free mirror of SCAN's de Boor evaluator."""
    # 【中文注释】二维 B 样条：SCAN 的 de Boor 求值器的精简无依赖镜像。

    def __init__(
        self,
        control_points: Sequence[Sequence[float]],
        order: int,
        knots: Sequence[float],
    ) -> None:
        # 【中文注释】构造函数：保存控制点、阶数与节点向量，并做一致性校验。
        self.control_points = tuple(
            _finite_point(point) for point in control_points
        )
        self.order = int(order)               # 样条阶数
        self.knots = tuple(float(value) for value in knots)  # 节点向量
        if self.order < 1:
            raise ValueError('B-spline order must be positive')
        if len(self.control_points) < self.order + 1:  # 控制点数量必须 >= 阶数+1
            raise ValueError('B-spline has too few control points')
        expected_knots = len(self.control_points) + self.order + 1
        if len(self.knots) != expected_knots:  # 节点数 = 控制点数 + 阶数 + 1
            raise ValueError(
                'B-spline knot count must equal point count + order + 1'
            )
        if not all(math.isfinite(value) for value in self.knots):
            raise ValueError('B-spline knots must be finite')
        if any(  # 节点必须非递减
            right < left
            for left, right in zip(self.knots, self.knots[1:])
        ):
            raise ValueError('B-spline knots must be nondecreasing')
        self._start_knot = self.knots[self.order]      # 有效区间起点
        self._end_knot = self.knots[-self.order - 1]   # 有效区间终点
        if self._end_knot <= self._start_knot:
            raise ValueError('B-spline duration must be positive')

    @property
    def duration(self) -> float:
        """Return the SCAN trajectory time span."""
        # 【中文注释】返回 SCAN 轨迹的时间跨度（有效区间长度）。
        return self._end_knot - self._start_knot

    def evaluate(self, time_sec: float) -> Point2D:
        """Evaluate at SCAN trajectory time using its de Boor convention."""
        # 【中文注释】按 SCAN 轨迹时间约定求值（de Boor 算法）。
        knot_value = min(  # 把时间夹到有效节点区间内
            self._end_knot,
            max(self._start_knot, self._start_knot + float(time_sec)),
        )
        span = self.order               # 二分查找所在节点区间（线性扫描）
        while self.knots[span + 1] < knot_value:
            span += 1
        working = [  # 取出参与计算的 order+1 个控制点副本
            list(self.control_points[span - self.order + index])
            for index in range(self.order + 1)
        ]
        for level in range(1, self.order + 1):  # de Boor 逐层线性插值
            for index in range(self.order, level - 1, -1):
                left_index = index + span - self.order
                right_index = index + 1 + span - level
                denominator = (
                    self.knots[right_index] - self.knots[left_index]
                )
                if denominator <= 0.0:   # 节点重合时插值权重取 0
                    alpha = 0.0
                else:
                    alpha = (
                        knot_value - self.knots[left_index]
                    ) / denominator
                working[index][0] = (    # x 分量插值
                    (1.0 - alpha) * working[index - 1][0]
                    + alpha * working[index][0]
                )
                working[index][1] = (    # y 分量插值
                    (1.0 - alpha) * working[index - 1][1]
                    + alpha * working[index][1]
                )
        return (working[self.order][0], working[self.order][1])


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class PathPoint:
    """One time and arc-length indexed trajectory sample."""
    # 【中文注释】一个按时间与弧长索引的轨迹采样点。

    x: float            # 路径点 x 坐标
    y: float            # 路径点 y 坐标
    time_sec: float     # 相对轨迹起点的采样时间（s）
    progress_m: float   # 累计弧长进度（m）


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class PathProjection:
    """Closest point on a sampled path."""
    # 【中文注释】采样路径上的最近点投影结果。

    progress_m: float   # 最近点处的弧长进度（m）
    distance_m: float   # 实测点到路径的垂直距离（横向偏差，m）
    x: float            # 最近点 x 坐标
    y: float            # 最近点 y 坐标


class SampledPath:
    """Arc-length view of one SCAN local B-spline."""
    # 【中文注释】一条 SCAN 局部 B 样条的弧长视角视图（密集采样后的路径）。

    def __init__(self, points: Sequence[PathPoint]) -> None:
        # 【中文注释】构造函数：保存采样点并计算路径总长与终点速度。
        self.points = tuple(points)
        if len(self.points) < 2:
            raise ValueError('sampled path requires at least two points')
        self.length_m = self.points[-1].progress_m  # 路径总长 = 最后一点弧长
        if self.length_m <= 0.0:
            raise ValueError('sampled path must have positive length')
        time_delta = max(  # 最后两个采样点的时间差（防零）
            1.0e-6,
            self.points[-1].time_sec - self.points[-2].time_sec,
        )
        self.end_speed_mps = math.hypot(  # 终点速度 = 最后一段长度 / 时间差
            self.points[-1].x - self.points[-2].x,
            self.points[-1].y - self.points[-2].y,
        ) / time_delta

    @classmethod
    def from_spline(
        cls,
        spline: Bspline2D,
        sample_period_sec: float = 0.02,
    ) -> 'SampledPath':
        """Sample the exact B-spline densely enough for spatial projection."""
        # 【中文注释】对精确 B 样条进行足够密集的采样，供空间投影使用。
        period = max(0.005, float(sample_period_sec))  # 采样周期下限 5ms
        sample_count = max(2, math.ceil(spline.duration / period) + 1)
        points = []
        progress = 0.0      # 累计弧长
        previous = None
        for index in range(sample_count):
            time_sec = spline.duration * index / (sample_count - 1)
            current = spline.evaluate(time_sec)
            if previous is not None:
                progress += math.hypot(  # 累加相邻采样点间距 → 弧长
                    current[0] - previous[0],
                    current[1] - previous[1],
                )
            points.append(
                PathPoint(current[0], current[1], time_sec, progress)
            )
            previous = current
        return cls(points)

    def closest_projection(
        self,
        x: float,
        y: float,
        minimum_progress_m: float = 0.0,
    ) -> PathProjection:
        """Project a measured position onto the non-regressing path suffix."""
        # 【中文注释】把实测位置投影到"不倒退"的路径后缀上（弧长只增不减）。
        minimum = max(0.0, min(float(minimum_progress_m), self.length_m))
        best = None
        for left, right in zip(self.points, self.points[1:]):  # 逐段找最近点
            if right.progress_m < minimum:  # 跳过起点进度之前的段
                continue
            dx = right.x - left.x
            dy = right.y - left.y
            segment_sq = dx * dx + dy * dy
            if segment_sq <= 1.0e-12:  # 退化段（长度≈0）跳过
                continue
            ratio = ((x - left.x) * dx + (y - left.y) * dy) / segment_sq  # 垂足参数
            ratio = max(0.0, min(1.0, ratio))  # 夹到 [0,1] 段内
            progress = left.progress_m + ratio * (
                right.progress_m - left.progress_m
            )
            if progress < minimum:  # 垂足落在最小进度之前 → 取最小进度点
                ratio = (
                    (minimum - left.progress_m)
                    / max(1.0e-12, right.progress_m - left.progress_m)
                )
                ratio = max(0.0, min(1.0, ratio))
                progress = minimum
            projected_x = left.x + ratio * dx
            projected_y = left.y + ratio * dy
            distance = math.hypot(x - projected_x, y - projected_y)
            candidate = PathProjection(
                progress, distance, projected_x, projected_y
            )
            if best is None or candidate.distance_m < best.distance_m:
                best = candidate  # 保留距离最小的投影
        if best is not None:
            return best
        final = self.points[-1]  # 无候选（路径退化）时回退到终点
        return PathProjection(
            final.progress_m,
            math.hypot(x - final.x, y - final.y),
            final.x,
            final.y,
        )

    def point_at(self, progress_m: float) -> Point2D:
        """Interpolate a point at arc-length progress."""
        # 【中文注释】按弧长进度插值路径上的点。
        progress = max(0.0, min(float(progress_m), self.length_m))
        for left, right in zip(self.points, self.points[1:]):
            if right.progress_m < progress:
                continue
            span = right.progress_m - left.progress_m
            ratio = 0.0 if span <= 1.0e-12 else (  # 段内线性插值比例
                progress - left.progress_m
            ) / span
            return (
                left.x + ratio * (right.x - left.x),
                left.y + ratio * (right.y - left.y),
            )
        final = self.points[-1]  # 超出末端 → 返回终点
        return (final.x, final.y)


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class ProgressFollowerParameters:
    """Spatial follower settings independent of the M20 output envelope."""
    # 【中文注释】空间跟随器参数（与 M20 输出包络无关）。

    lookahead_m: float = 0.60               # 空间前视距离（m）
    max_speed_mps: float = 0.45             # 最大跟踪速度（m/s）
    yaw_gain: float = 1.50                  # 航向误差 → 偏航比例增益
    max_yaw_radps: float = 0.65             # 最大偏航角速度（rad/s）
    finish_distance_m: float = 0.15         # 终点判定距离（m）
    terminal_speed_threshold_mps: float = 0.10  # 终点速度阈值（低于它视为终点减速段）
    terminal_slowdown_distance_m: float = 0.80 # 终点减速开始距离（m）
    terminal_min_speed_mps: float = 0.10    # 终点最小速度（m/s）
    reverse_tracking_enabled: bool = True   # 是否启用倒车跟踪
    reverse_enter_angle_rad: float = 2.10   # 进入倒车跟踪的航向误差（rad，滞回上限）
    reverse_exit_angle_rad: float = 1.75    # 退出倒车跟踪的航向误差（rad，滞回下限）


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class ProgressCommand:
    """Follower output and diagnostic state for one measured pose."""
    # 【中文注释】一次实测位姿的跟随器输出与诊断状态。

    command: PlanarCommand   # 生成的平面速度指令 (vx, vy, yaw)
    progress_m: float        # 当前弧长进度（m）
    remaining_m: float       # 剩余路径长度（m）
    cross_track_m: float     # 横向偏差（m）
    target: Point2D          # 前视目标点
    heading_error_rad: float # 航向误差（rad）
    reverse_tracking: bool   # 是否处于倒车跟踪
    finished: bool           # 是否到达终点（已完成）


class SpatialProgressFollower:
    """Generate a rolling command from measured path progress."""
    # 【中文注释】空间进度跟随器：根据实测路径进度生成滚动指令。

    def __init__(self, parameters: ProgressFollowerParameters) -> None:
        # 【中文注释】构造函数：保存参数并复位状态。
        self.parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Clear per-trajectory progress and direction state."""
        # 【中文注释】清除单条轨迹的进度与方向状态。
        self.progress_m = 0.0       # 当前弧长进度
        self.reverse_tracking = False  # 是否倒车跟踪

    def start_trajectory(self) -> None:
        """Reset only spatial progress when SCAN publishes a new local path."""
        # 【中文注释】SCAN 发布新局部路径时，只复位空间进度。
        self.progress_m = 0.0

    def update(
        self,
        path: SampledPath,
        x: float,
        y: float,
        yaw: float,
    ) -> ProgressCommand:
        """Track the nearest measured progress and a fixed spatial lookahead."""
        # 【中文注释】跟踪最近的实测进度与固定空间前视点，生成指令。
        projection = path.closest_projection(  # 最近点投影（允许 5cm 倒退容差）
            x,
            y,
            max(0.0, self.progress_m - 0.05),
        )
        self.progress_m = max(self.progress_m, projection.progress_m)  # 进度只增不减
        target_progress = min(  # 前视进度 = 当前进度 + 前视距离
            path.length_m,
            self.progress_m + max(0.05, self.parameters.lookahead_m),
        )
        target = path.point_at(target_progress)  # 前视目标点
        end = path.point_at(path.length_m)       # 路径终点
        distance_to_end = math.hypot(end[0] - x, end[1] - y)  # 到终点的直线距离
        remaining = max(0.0, path.length_m - self.progress_m)  # 剩余弧长
        terminal = (  # 是否为终点减速段（终点速度低于阈值）
            path.end_speed_mps
            <= self.parameters.terminal_speed_threshold_mps
        )
        finished = (  # 完成条件：终点减速段且已到达终点判定距离内
            terminal
            and distance_to_end <= self.parameters.finish_distance_m
        )
        if finished:
            return ProgressCommand(  # 完成：输出零指令
                (0.0, 0.0, 0.0),
                self.progress_m,
                remaining,
                projection.distance_m,
                target,
                0.0,
                self.reverse_tracking,
                True,
            )

        target_dx = target[0] - x   # 目标点相对机体的方向
        target_dy = target[1] - y
        if math.hypot(target_dx, target_dy) < 0.03:  # 前视点太近时改用"身后点"定义航向
            behind = path.point_at(max(0.0, target_progress - 0.10))
            target_dx = target[0] - behind[0]
            target_dy = target[1] - behind[1]
        course_yaw = math.atan2(target_dy, target_dx)  # 目标航向角
        forward_error = normalize_angle(course_yaw - yaw)  # 前向航向误差
        if self.parameters.reverse_tracking_enabled:  # 倒车跟踪滞回（进入/退出阈值不同）
            if (
                not self.reverse_tracking
                and abs(forward_error)
                >= self.parameters.reverse_enter_angle_rad
            ):
                self.reverse_tracking = True
            elif (
                self.reverse_tracking
                and abs(forward_error)
                <= self.parameters.reverse_exit_angle_rad
            ):
                self.reverse_tracking = False
        else:
            self.reverse_tracking = False

        tracking_yaw = normalize_angle(  # 跟踪航向：倒车时加 pi（掉头方向）
            course_yaw + (math.pi if self.reverse_tracking else 0.0)
        )
        heading_error = normalize_angle(tracking_yaw - yaw)  # 用于偏航控制的角度误差
        speed = max(0.0, self.parameters.max_speed_mps)
        if terminal:  # 终点减速：按剩余距离比例降速（不低于最小速度）
            slowdown = max(
                self.parameters.finish_distance_m,
                self.parameters.terminal_slowdown_distance_m,
            )
            ratio = max(0.0, min(1.0, distance_to_end / slowdown))
            speed = min(
                speed,
                max(
                    self.parameters.terminal_min_speed_mps,
                    self.parameters.max_speed_mps * ratio,
                ),
            )
        if self.reverse_tracking:
            speed = -speed  # 倒车跟踪：速度取负
        yaw_rate = max(  # 偏航指令 = 比例控制并限幅
            -self.parameters.max_yaw_radps,
            min(
                self.parameters.max_yaw_radps,
                self.parameters.yaw_gain * heading_error,
            ),
        )
        return ProgressCommand(
            (speed, 0.0, yaw_rate),  # 输出 (速度, 0 横向, 偏航率)
            self.progress_m,
            remaining,
            projection.distance_m,
            target,
            heading_error,
            self.reverse_tracking,
            False,
        )
