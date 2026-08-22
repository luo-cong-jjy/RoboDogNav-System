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
# 【文件职责】test_collision_policy.py —— 碰撞预判策略单元测试
# 测试 collision_policy.py 的全部核心纯函数：
#   - 在线占用栅格化（高度带过滤、局部窗口、世界坐标）；
#   - 栅格膨胀（保守半径 vs 硬车身半径）、轨迹阻塞判断；
#   - 前视采样预测（predict_poses / predict_positions）；
#   - 名义/实测侧向漂移指令包络（command_drift_variants）；
#   - 滚动恢复（safe_rolling_recovery）、光栅外壳逃逸
#     （safe_raster_shell_escape）、恢复清扫与终点校验；
#   - 恢复预算（update_recovery_budget）与连续清零确认
#     （update_clear_confirmation）；
#   - 共享地图边界的 edge_tolerance 门洞语义。
# 另有一个"源码一致性"测试，确保 collision_guard_node.py 的关键
# 参数默认值与策略测试保持同步。
# ============================================================================

"""Tests for independent collision lookahead behavior."""

from pathlib import Path  # 路径对象：定位被测节点源码（源码一致性测试用）

import numpy as np  # 数值数组库：构造占用栅格与断言

from m20_inspection_core.collision_policy import (  # 导入被测模块的全部纯函数与类型
    command_drift_variants,
    conservative_raster_radius,
    first_blocking_command_envelope,
    GridGeometry,
    first_blocking_footprint,
    hard_body_raster_radius,
    inflate_blocked_grid,
    predict_poses,
    predict_positions,
    rasterize_online_occupancy,
    recovery_sweep_is_clear,
    safe_raster_shell_escape,
    safe_rolling_recovery,
    trajectory_is_blocked,
    update_clear_confirmation,
    update_recovery_budget,
)


def test_online_occupancy_filters_height_and_uses_world_coordinates() -> None:
    # 【测试】在线占用栅格化应按机体高度带过滤点，并使用世界坐标系投影
    occupancy, geometry, accepted = rasterize_online_occupancy(
        [
            (10.4, -3.0, 0.40),           # 在高度带内：应被接受
            (10.6, -3.0, 1.10),           # 在高度带内：应被接受
            (10.8, -3.0, 1.80),           # 超出高度带：应被过滤
            (float('nan'), -3.0, 0.60),   # 非有限值：应被过滤
        ],
        (10.0, -3.0),
        0.10,
        4.0,
        0.20,
        1.20,
    )

    assert accepted == 2
    assert occupancy.shape == (geometry.width * geometry.height,)
    for x in (10.4, 10.6):
        cell_x, cell_y = geometry.cell(x, -3.0)
        assert occupancy[cell_y * geometry.width + cell_x] == 100


def test_online_occupancy_drops_points_outside_local_window() -> None:
    # 【测试】落在机器人局部窗口之外的点应被丢弃
    occupancy, _geometry, accepted = rasterize_online_occupancy(
        [(0.5, 0.0, 0.5), (20.0, 0.0, 0.5)],
        (0.0, 0.0),
        0.10,
        4.0,
        0.0,
        1.0,
    )

    assert accepted == 1
    assert int(np.count_nonzero(occupancy)) == 1


def test_guard_source_clamps_prediction_to_backend_limits() -> None:
    # 【测试】守卫节点源码中的限幅与恢复参数默认值应与策略保持一致
    source = (
        Path(__file__).resolve().parents[1]
        / 'm20_inspection_core'
        / 'collision_guard_node.py'
    ).read_text(encoding='utf-8')
    assert "self.declare_parameter('max_linear_x', 0.45)" in source
    assert "self.declare_parameter('max_linear_y', 0.20)" in source
    assert "self.declare_parameter('max_angular_z', 0.65)" in source
    assert "self.declare_parameter('recovery_forward_speed', 0.35)" in source
    assert (
        "self.declare_parameter('recovery_clear_confirm_sec', 0.30)"
        in source
    )
    assert (
        "self.declare_parameter('recovery_rearm_clear_sec', 1.50)"
        in source
    )
    assert "self.declare_parameter('recovery_max_active_sec', 6.0)" in source
    assert (
        "self.declare_parameter('recovery_max_displacement_m', 0.75)"
        in source
    )
    assert (
        "'recovery_positive_yaw_lateral_drift', 0.15"
        in source
    )
    assert 'max(-max_x, min(max_x, float(message.linear.x)))' in source


def test_forward_lookahead_detects_wall_before_contact() -> None:
    # 【测试】前视预测应在接触前检测到墙体（提前停车）
    geometry = GridGeometry(30, 20, 0.1, 0.0, 0.0)
    values = np.zeros((20, 30), dtype=np.int8)
    values[:, 15] = 100  # 在 x=15 处竖立一面墙
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.2
    )
    positions = predict_positions(
        1.0, 1.0, 0.0, (0.5, 0.0, 0.0), 1.0, 0.05
    )
    assert trajectory_is_blocked(blocked, geometry, positions)


def test_conservative_raster_radius_closes_half_cell_aliasing_gap() -> None:
    # 【测试】保守半径应消除"半格走样"间隙（旧六格核会漏检近在咫尺的占用）
    geometry = GridGeometry(40, 20, 0.05, 0.0, 0.0)
    values = np.zeros((20, 40), dtype=np.int8)
    values[10, 20] = 100
    nominal = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.30
    )
    conservative = inflate_blocked_grid(
        values.ravel(),
        geometry,
        50,
        conservative_raster_radius(0.30, geometry.resolution),
    )

    # Seven cells are 0.35 m apart. The old six-cell kernel misses this
    # lookup even though a continuous centre can be less than 0.30 m away.
    # 【中文】七个格相距 0.35 m；旧六格核会漏检，即使连续圆心距离不足 0.30 m。
    assert not nominal[10, 27]
    assert conservative[10, 27]


def test_hard_body_radius_excludes_only_half_cell_shell() -> None:
    # 【测试】硬车身半径只扣除半格外壳（物理车身，不含保守壳）
    assert abs(hard_body_raster_radius(0.25, 0.10) - 0.1792893) < 1e-6


def test_raster_shell_escape_requires_hard_clear_sweep_and_clear_endpoint():
    # 【测试】光栅外壳逃逸要求硬车身清扫完全畅通且终点回到保守自由空间
    geometry = GridGeometry(80, 40, 0.1, -4.0, -2.0)
    conservative = np.zeros((40, 80), dtype=bool)
    hard = np.zeros_like(conservative)
    pose = (0.0, 0.0, 0.0)
    front_x, front_y = geometry.cell(0.18, 0.0)
    conservative[front_y, front_x] = True  # 仅在保守栅格中标记前方障碍

    arguments = (
        conservative,
        hard,
        geometry,
        pose,
        0.90,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
    )
    assert safe_raster_shell_escape(*arguments) == (-0.35, 0.0, 0.0)

    hard[front_y, front_x] = True  # 硬车身栅格也被占用时不再授权任何运动
    assert safe_raster_shell_escape(*arguments) is None


def test_measured_turn_drift_is_checked_beside_nominal_sweep() -> None:
    # 【测试】实测转向侧向漂移轨迹应作为额外包络被检查（名义轨迹不撞、漂移轨迹撞）
    geometry = GridGeometry(100, 100, 0.05, -2.5, -2.5)
    values = np.zeros((100, 100), dtype=np.int8)
    obstacle_x, obstacle_y = geometry.cell(0.14, 0.08)
    values[obstacle_y, obstacle_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )
    pose = (0.0, 0.0, 0.0)
    command = (0.35, 0.0, 0.65)
    nominal = first_blocking_footprint(
        blocked,
        geometry,
        predict_poses(*pose, command, 0.5, 0.05),
        0.0,
        0.05,
    )
    envelope = first_blocking_command_envelope(
        blocked,
        geometry,
        pose,
        command,
        0.5,
        0.05,
        0.0,
        0.15,
        0.10,
        0.05,
        0.65,
    )

    assert nominal is None
    assert envelope is not None
    assert envelope.motion_model == 'measured_lateral_drift'


def test_turn_drift_scales_with_requested_yaw_rate() -> None:
    # 【测试】转向漂移量应按请求角速度比例缩放
    variants = dict(
        command_drift_variants(
            (0.35, 0.0, -0.325),
            0.15,
            0.10,
            0.05,
            0.65,
        )
    )

    assert variants['measured_lateral_drift'] == (
        0.35,
        0.05,
        -0.325,
    )
    assert variants['opposite_lateral_uncertainty'] == (
        0.35,
        -0.025,
        -0.325,
    )


def test_clear_turning_trajectory_is_not_stopped() -> None:
    # 【测试】畅通的转弯轨迹不应被误判为阻塞
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.2
    )
    positions = predict_positions(
        0.0, 0.0, 0.0, (0.2, 0.0, 0.5), 1.0, 0.05
    )
    assert not trajectory_is_blocked(blocked, geometry, positions)


def test_predicted_translation_uses_opposite_rolling_turn() -> None:
    # 【测试】预测到前方碰撞时，恢复应采用相反的滚动转弯（避开阻塞方向）
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    values[:, 25] = 100  # 前方 x=25 处竖墙
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )
    pose = (0.0, 0.0, 0.0)
    command = (0.5, 0.0, 0.5)
    blocked_sample = first_blocking_footprint(
        blocked,
        geometry,
        predict_poses(*pose, command, 1.0, 0.05),
        0.18,
        0.05,
    )

    assert blocked_sample is not None
    assert blocked_sample.sample_index > 0
    assert safe_rolling_recovery(
        blocked,
        geometry,
        pose,
        command,
        1.0,
        0.05,
        0.18,
        0.35,
        0.0,
        0.0,
        0.0,
        0.20,
    ) == (0.35, 0.0, -0.5)


def test_recovery_uses_yaw_when_opposite_arc_is_blocked() -> None:
    # 【测试】反向转弯弧被阻塞时，恢复应使用同向（带角速度）滚动
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    cell_x, cell_y = geometry.cell(0.49, -0.17)
    values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.0,
        0.0,
        0.0,
        0.20,
    ) == (0.35, 0.0, 0.5)


def test_current_footprint_never_authorizes_rolling_recovery() -> None:
    # 【测试】当前足印已被占用时，滚动恢复永不被授权（fail-closed）
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    values = np.zeros((40, 40), dtype=np.int8)
    cell_x, cell_y = geometry.cell(0.18, 0.0)
    values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.1, 0.0, 0.5),
        1.0,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    ) is None


def test_pure_yaw_request_never_authorizes_rolling_recovery() -> None:
    # 【测试】纯旋转请求（无前进速度）永不被授权滚动恢复
    geometry = GridGeometry(40, 40, 0.1, -2.0, -2.0)
    blocked = np.zeros((40, 40), dtype=bool)

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.5),
        1.0,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    ) is None


def test_weak_yaw_forward_request_can_use_clear_reverse() -> None:
    # 【测试】弱转向的前进请求被前方阻塞时，可使用通畅的直线后退逃逸
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    front_x, front_y = geometry.cell(0.55, 0.0)
    values[front_y, front_x] = 100  # 前方障碍
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.03),
        1.0,
        0.02,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    ) == (-0.35, 0.0, 0.0)


def test_shared_edge_prediction_can_recover_straight_inward() -> None:
    # 【测试】共享地图边界外的预测可向内直线恢复（不继续被拒绝的反向转弯）
    geometry = GridGeometry(
        400, 400, 0.1, -40.0, -20.0, edge_tolerance=0.35
    )
    blocked = np.zeros((400, 400), dtype=bool)
    pose = (0.10, 0.12, -2.63)
    command = (-0.16, 0.0, -0.18)

    blocked_sample = first_blocking_footprint(
        blocked,
        geometry,
        predict_poses(*pose, command, 0.90, 0.05),
        0.18,
        0.05,
    )
    recovery = safe_rolling_recovery(
        blocked,
        geometry,
        pose,
        command,
        0.90,
        0.05,
        0.18,
        0.35,
        0.15,
        0.10,
        0.05,
        0.20,
    )

    assert blocked_sample is not None
    assert blocked_sample.cause == 'OUT_OF_BOUNDS'
    assert recovery == (0.35, 0.0, 0.0)


def test_measured_lateral_drift_falls_back_to_clear_reverse() -> None:
    # 【测试】实测漂移使滚动恢复不可用时，应回退到通畅的直线后退
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    for obstacle_x, obstacle_y in (
        (0.53, -0.026),
        (0.458, 0.314),
    ):
        cell_x, cell_y = geometry.cell(obstacle_x, obstacle_y)
        values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    nominal = safe_rolling_recovery(  # 无漂移模型：有滚动恢复可用
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.0,
        0.0,
        0.0,
        0.20,
    )
    conservative = safe_rolling_recovery(  # 含漂移模型：滚动不可用，回退直线后退
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.15,
        0.15,
        0.05,
        0.20,
    )

    assert nominal is not None
    assert conservative == (-0.35, 0.0, 0.0)


def test_reverse_recovery_is_rejected_when_rear_sweep_is_blocked() -> None:
    # 【测试】后方清扫被阻塞时，直线后退恢复也应被拒绝
    geometry = GridGeometry(200, 200, 0.02, -2.0, -2.0)
    values = np.zeros((200, 200), dtype=np.int8)
    # Two front-side obstacles reject both drift-aware rolling arcs.  The
    # rear obstacle also rejects the straight reverse fallback.
    # 【中文】两个前侧障碍拒绝两种考虑漂移的滚动弧；后侧障碍再拒绝直线后退回退。
    for obstacle_x, obstacle_y in (
        (0.53, -0.026),
        (0.458, 0.314),
        (-0.50, 0.0),
    ):
        cell_x, cell_y = geometry.cell(obstacle_x, obstacle_y)
        values[cell_y, cell_x] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )

    assert safe_rolling_recovery(
        blocked,
        geometry,
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.5),
        1.0,
        0.02,
        0.18,
        0.35,
        0.15,
        0.15,
        0.05,
        0.20,
    ) is None


def test_latched_recovery_is_rechecked_against_current_sweep() -> None:
    # 【测试】已锁存的恢复指令应针对当前清扫重新校验（后方出现障碍则失效）
    geometry = GridGeometry(100, 100, 0.05, -2.5, -2.5)
    values = np.zeros((100, 100), dtype=np.int8)
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.0
    )
    recovery = (-0.35, 0.0, 0.0)
    arguments = (
        geometry,
        (0.0, 0.0, 0.0),
        recovery,
        0.9,
        0.05,
        0.18,
        0.15,
        0.10,
        0.05,
    )
    assert recovery_sweep_is_clear(blocked, *arguments)

    rear_x, rear_y = geometry.cell(-0.45, 0.0)
    blocked[rear_y, rear_x] = True  # 在后方路径上新增障碍
    assert not recovery_sweep_is_clear(blocked, *arguments)


def test_recovery_release_requires_continuous_clear_time() -> None:
    # 【测试】恢复释放需要连续清零时间达到确认阈值
    first_clear, confirmed = update_clear_confirmation(None, 10.0, 0.30)
    assert first_clear == 10.0
    assert not confirmed

    first_clear, confirmed = update_clear_confirmation(
        first_clear, 10.29, 0.30
    )
    assert first_clear == 10.0
    assert not confirmed

    first_clear, confirmed = update_clear_confirmation(
        first_clear, 10.30, 0.30
    )
    assert first_clear == 10.0
    assert confirmed


def test_disabled_or_rewound_clear_confirmation_is_deterministic() -> None:
    # 【测试】时钟回退或确认时长被禁用时的行为应确定
    assert update_clear_confirmation(8.0, 7.0, 0.30) == (7.0, False)
    assert update_clear_confirmation(8.0, 8.0, 0.0) == (None, True)


def test_recovery_budget_refreshes_only_after_minimum_progress() -> None:
    # 【测试】恢复预算仅在达到最小前进距离后刷新进度检查点
    progress_time, progress_xy, reason = update_recovery_budget(
        10.0,
        (0.0, 0.0),
        10.0,
        (0.0, 0.0),
        11.0,
        (0.04, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )

    assert progress_time == 11.0
    assert progress_xy == (0.04, 0.0)
    assert reason is None


def test_recovery_budget_stops_a_motionless_command() -> None:
    # 【测试】恢复过程中无有效前进（停滞）时应触发 NO_PROGRESS
    _, _, reason = update_recovery_budget(
        10.0,
        (0.0, 0.0),
        10.0,
        (0.0, 0.0),
        11.5,
        (0.01, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )

    assert reason == 'NO_PROGRESS'


def test_recovery_budget_has_absolute_time_and_distance_limits() -> None:
    # 【测试】恢复预算有绝对时间上限与位移上限
    common = (10.0, (0.0, 0.0), 15.0, (0.50, 0.0))
    assert update_recovery_budget(
        *common,
        16.0,
        (0.60, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )[2] == 'TIME_LIMIT'
    assert update_recovery_budget(
        10.0,
        (0.0, 0.0),
        10.5,
        (0.40, 0.0),
        11.0,
        (0.75, 0.0),
        6.0,
        0.75,
        1.5,
        0.03,
    )[2] == 'DISTANCE_LIMIT'


def test_oriented_double_circle_detects_front_before_body_centre() -> None:
    # 【测试】定向双圆模型应在机体中心之前先检测到前圆碰撞
    geometry = GridGeometry(40, 20, 0.1, 0.0, 0.0)
    values = np.zeros((20, 40), dtype=np.int8)
    values[:, 20] = 100
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 0.30
    )
    poses = predict_poses(
        1.55, 1.0, 0.0, (0.0, 0.0, 0.0), 0.1, 0.05
    )

    sample = first_blocking_footprint(
        blocked, geometry, poses, 0.18, 0.05
    )

    assert sample is not None
    assert sample.sample_index == 0
    assert sample.circle == 'front'
    assert sample.cause == 'OCCUPIED'


def test_shared_edge_tolerance_only_opens_occupied_grid_doorways() -> None:
    # 【测试】共享边界容差只打开占用栅格上的"门洞"（超出容差仍越界即阻塞）
    geometry = GridGeometry(
        10, 10, 1.0, 0.0, 0.0, edge_tolerance=0.35
    )
    values = np.zeros((10, 10), dtype=np.int8)
    values[0, :] = 100
    values[-1, :] = 100
    values[:, 0] = 100
    values[:, -1] = 100
    values[4:7, 0] = 0    # 左侧留出门洞（y=4..6）
    values[4:7, -1] = 0   # 右侧留出门洞（y=4..6）
    blocked = inflate_blocked_grid(
        values.ravel(), geometry, 50, 1.0
    )

    assert geometry.cell(-0.20, 5.0) == (0, 5)
    assert geometry.cell(10.20, 5.0) == (9, 5)
    assert not trajectory_is_blocked(
        blocked, geometry, [(-0.20, 5.0)]
    )
    assert geometry.cell(10.0, 5.0) == (9, 5)
    assert not trajectory_is_blocked(
        blocked, geometry, [(10.20, 5.0)]
    )
    assert trajectory_is_blocked(
        blocked, geometry, [(-0.351, 5.0)]
    )
    assert trajectory_is_blocked(
        blocked, geometry, [(10.351, 5.0)]
    )
    assert trajectory_is_blocked(
        blocked, geometry, [(10.20, 2.0)]
    )
