# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_trajectory_progress.py
# 所属：m20_nav2_locomotion —— M20 运动控制包的单元测试
# 核心职责：测试"实测进度"SCAN 轨迹执行模块（trajectory_progress.py）。
#   - de Boor 求值器：线性端点/中点一致性、非法节点契约拒绝；
#   - 采样路径：弧长保持与终点速度计算；
#   - 最近点投影与弧长插值；
#   - 空间进度跟随器：进度不倒退、直线速度/航向误差包络、
#     倒车跟踪、终点停止等。
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

"""Unit tests for measured-progress SCAN trajectory execution."""
# 【中文注释】模块说明：实测进度 SCAN 轨迹执行的单元测试。

import math  # 数学库（isclose 等断言）

import pytest  # pytest 测试框架

from m20_nav2_locomotion.trajectory_progress import (  # 被测模块
    Bspline2D,                     #   B 样条求值器
    PathPoint,                     #   路径点数据类
    ProgressFollowerParameters,    #   跟随器参数
    SampledPath,                   #   采样路径
    SpatialProgressFollower,       #   空间进度跟随器
)


def _line_spline() -> Bspline2D:
    # 【中文注释】构造一条 1 阶（线性）B 样条：从 (0,0) 到 (2,0)，时长 2s。
    return Bspline2D(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        1,
        [-1.0, 0.0, 1.0, 2.0, 3.0],
    )


def _line_path() -> SampledPath:
    # 【中文注释】构造一条手工采样路径（3 个点，弧长 0→2m，每米一点）。
    return SampledPath(
        [
            PathPoint(0.0, 0.0, 0.0, 0.0),
            PathPoint(1.0, 0.0, 1.0, 1.0),
            PathPoint(2.0, 0.0, 2.0, 2.0),
        ]
    )


def test_de_boor_evaluator_matches_linear_endpoints_and_midpoint():
    # 【中文注释】校验：de Boor 求值器在线性端点与中点处与理论值一致。
    spline = _line_spline()

    assert spline.duration == 2.0            # 时长 = 2s
    assert spline.evaluate(0.0) == (0.0, 0.0)  # 起点
    assert spline.evaluate(1.0) == (1.0, 0.0)  # 中点
    assert spline.evaluate(2.0) == (2.0, 0.0)  # 终点


def test_invalid_knot_contract_is_rejected():
    # 【中文注释】校验：非法节点数量契约被拒绝（节点数 ≠ 点数 + 阶数 + 1）。
    with pytest.raises(ValueError, match='knot count'):
        Bspline2D([(0.0, 0.0), (1.0, 0.0)], 1, [0.0, 1.0])


def test_sampled_path_preserves_arc_length_and_terminal_speed():
    # 【中文注释】校验：采样路径保持弧长（2m）并正确计算终点速度（1m/s）。
    path = SampledPath.from_spline(_line_spline(), 0.05)

    assert math.isclose(path.length_m, 2.0, abs_tol=1.0e-9)
    assert math.isclose(path.end_speed_mps, 1.0, abs_tol=1.0e-9)


def test_projection_and_arc_length_interpolation_are_spatial():
    # 【中文注释】校验：最近点投影与弧长插值都是空间量的（非时间量）。
    path = _line_path()
    projection = path.closest_projection(0.70, 0.20)  # 投影 (0.70,0.20) 到 x 轴

    assert math.isclose(projection.progress_m, 0.70)  # 垂足弧长
    assert math.isclose(projection.distance_m, 0.20)  # 垂直距离
    assert path.point_at(1.25) == (1.25, 0.0)         # 弧长插值


def test_progress_never_regresses_when_odometry_moves_backwards():
    # 【中文注释】校验：里程计后退时进度不倒退（单调不减）。
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    path = _line_path()

    first = follower.update(path, 1.0, 0.0, 0.0)   # 前进到弧长 1m
    second = follower.update(path, 0.5, 0.0, 0.0)  # 后退到 0.5m

    assert first.progress_m == 1.0
    assert second.progress_m == 1.0  # 进度保持不倒退


def test_straight_path_requests_m20_speed_without_wall_clock_advance():
    # 【中文注释】校验：直线路径请求 M20 速度（0.45 m/s），
    # 不依赖墙钟推进（用固定前视 0.60m）。
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    result = follower.update(_line_path(), 0.0, 0.0, 0.0)

    assert result.command == (0.45, 0.0, 0.0)  # 满速前进
    assert result.progress_m == 0.0
    assert result.target == (0.60, 0.0)        # 前视目标点


def test_heading_error_is_bounded_by_m20_yaw_envelope():
    # 【中文注释】校验：航向误差被 M20 偏航包络限制（-0.65 rad/s）。
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    result = follower.update(_line_path(), 0.0, 0.0, math.pi / 4.0)  # 偏航 45°

    assert result.command == (0.45, 0.0, -0.65)  # 反向纠正且限幅
    assert math.isclose(result.heading_error_rad, -math.pi / 4.0)


def test_path_behind_uses_reverse_without_demanding_a_turnaround():
    # 【中文注释】校验：路径在身后时使用倒车跟踪，而不要求原地掉头。
    follower = SpatialProgressFollower(ProgressFollowerParameters())
    result = follower.update(_line_path(), 0.0, 0.0, math.pi)  # 机头朝后

    assert result.reverse_tracking        # 进入倒车跟踪
    assert result.command == (-0.45, 0.0, 0.0)  # 负速后退


def test_terminal_path_stops_only_near_a_zero_speed_endpoint():
    # 【中文注释】校验：仅在终点速度为 0 的路径末端附近才停止。
    terminal = SampledPath(  # 终点速度≈0（最后一段几乎不动）
        [
            PathPoint(0.0, 0.0, 0.0, 0.0),
            PathPoint(1.0, 0.0, 1.0, 1.0),
            PathPoint(1.01, 0.0, 2.0, 1.01),
        ]
    )
    follower = SpatialProgressFollower(ProgressFollowerParameters())

    result = follower.update(terminal, 1.0, 0.0, 0.0)  # 已到终点附近

    assert result.finished                      # 判定完成
    assert result.command == (0.0, 0.0, 0.0)    # 输出零指令
