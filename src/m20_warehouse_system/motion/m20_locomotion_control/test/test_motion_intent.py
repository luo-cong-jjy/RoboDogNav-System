# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_motion_intent.py
# 所属：m20_locomotion_control —— M20 运动控制包的单元测试
# 核心职责：测试与仿真器无关的 M20 运动意图策略（motion_intent.py）。
#   - constrain_for_intent：零指令/直线巡航/大曲率转弯/近原地偏航/横移的分类与约束；
#   - 指令包络不放大任何分量、非有限输入归零；
#   - RollingNavigationAdapter：横向误差转平滑偏航、稳定滚动速度保持、
#     低速转弯投影到稳定滚动、转弯滞回防抖、转弯保持、原地转弯转滚动弧、
#     转弯优先立即取消巡航、零指令立即停止、绝不请求自主横移、
#     倒车死区补偿与倒车偏航补偿、SDK 指令上限尊重。
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

"""Unit tests for the simulator-independent M20 motion-intent policy."""
# 【中文注释】模块说明：与仿真器无关的 M20 运动意图策略单元测试。

import math  # 数学库（nan/inf、hypot 等）

from m20_locomotion_control.motion_intent import (  # 被测模块
    IntentParameters,              #   意图参数
    MotionIntent,                  #   运动意图枚举
    RollingNavigationAdapter,      #   滚动导航适配器
    constrain_for_intent,          #   按意图约束指令
)


def test_zero_command_is_stopped():
    # 【中文注释】校验：零指令分类为 STOPPED 并输出全零。
    intent, command = constrain_for_intent(
        (0.0, 0.0, 0.0),
        IntentParameters(),
    )
    assert intent is MotionIntent.STOPPED
    assert command == (0.0, 0.0, 0.0)


def test_straight_command_uses_cruise_and_suppresses_small_side_motion():
    # 【中文注释】校验：直线指令归为巡航，并抑制小幅横向分量。
    intent, command = constrain_for_intent(
        (0.35, 0.03, 0.05),  # 横向 0.03 < lateral_threshold
        IntentParameters(),
    )
    assert intent is MotionIntent.WHEEL_CRUISE
    assert command == (0.35, 0.0, 0.05)  # 横向被抑制为 0


def test_large_curvature_uses_coordinated_turn_without_low_speed_clamp():
    # 【中文注释】校验：大曲率指令归为协调转弯，且不做低速下限钳制。
    intent, command = constrain_for_intent(
        (0.30, 0.0, 0.60),  # 曲率 2.0 > 阈值 1.2
        IntentParameters(),
    )
    assert intent is MotionIntent.COORDINATED_TURN
    assert command == (0.30, 0.0, 0.60)


def test_near_in_place_yaw_uses_coordinated_turn():
    # 【中文注释】校验：近原地偏航归为协调转弯。
    intent, command = constrain_for_intent(
        (0.04, 0.0, -0.50),  # 前向低于 in_place 阈值且偏航大
        IntentParameters(),
    )
    assert intent is MotionIntent.COORDINATED_TURN
    assert command == (0.04, 0.0, -0.50)


def test_lateral_maneuver_limits_forward_motion():
    # 【中文注释】校验：横移机动时限制前向速度（不超过 lateral_max_forward）。
    intent, command = constrain_for_intent(
        (0.40, 0.10, 0.10),  # 横向 0.10 > lateral_threshold
        IntentParameters(),
    )
    assert intent is MotionIntent.LATERAL_MANEUVER
    assert command == (0.10, 0.10, 0.10)  # 前向被钳到 0.10


def test_command_envelope_never_increases_components():
    # 【中文注释】校验：指令包络绝不放大任何分量（逐分量 |输出| <= |输入|）。
    requested = (0.90, -0.60, 1.40)
    _, command = constrain_for_intent(requested, IntentParameters())
    for output, input_value in zip(command, requested):
        assert abs(output) <= abs(input_value)


def test_non_finite_input_fails_to_zero_for_that_component():
    # 【中文注释】校验：非有限输入分量归零。
    _, command = constrain_for_intent(
        (math.nan, math.inf, -math.inf),
        IntentParameters(),
    )
    assert command == (0.0, 0.0, 0.0)


def test_navigation_adapter_converts_side_error_to_smooth_yaw():
    # 【中文注释】校验：导航适配器把横向误差转换为平滑偏航（不请求侧移）。
    adapter = RollingNavigationAdapter(IntentParameters())
    intent, command = adapter.update((0.30, 0.03, 0.05), dt=0.10)

    assert intent is MotionIntent.WHEEL_CRUISE
    assert command[0] == 0.10        # 斜坡限幅后的前向
    assert command[1] == 0.0         # 无横向
    assert 0.0 < command[2] < 0.10   # 平滑偏航校正


def test_navigation_adapter_preserves_measured_stable_rolling_speed():
    # 【中文注释】校验：导航适配器保持实测稳定滚动速度（vx,vy 合成速度）。
    parameters = IntentParameters()
    adapter = RollingNavigationAdapter(parameters)
    intent, command = adapter.update((0.30, 0.20, 0.0), dt=1.0)

    assert intent is MotionIntent.COORDINATED_TURN
    assert adapter.turning                    # 进入转弯滞回
    assert command[0] == math.hypot(0.30, 0.20)  # 前向 = 合成速度
    assert command[1] == 0.0
    assert command[2] > parameters.turn_yaw_threshold  # 偏航超阈值


def test_navigation_adapter_projects_low_speed_turn_to_stable_roll():
    # 【中文注释】校验：低速转弯被投影到稳定滚动速度（抬升到 turn_min_forward）。
    parameters = IntentParameters()
    adapter = RollingNavigationAdapter(parameters)
    intent, command = adapter.update((0.05, 0.0, 0.65), dt=1.0)

    assert intent is MotionIntent.COORDINATED_TURN
    assert command == (
        parameters.turn_min_forward,  # 低速被抬升到稳定下限
        0.0,
        parameters.max_yaw,
    )


def test_navigation_adapter_turn_hysteresis_prevents_mode_chatter():
    # 【中文注释】校验：转弯滞回防止模式抖动（进入后保持，条件满足才退出）。
    adapter = RollingNavigationAdapter(IntentParameters())
    adapter.update((0.30, 0.10, 0.0), dt=1.0)  # 进入转弯

    intent, _ = adapter.update((0.30, 0.04, 0.0), dt=1.0)  # 横向减小但仍保持
    assert intent is MotionIntent.COORDINATED_TURN
    assert adapter.turning

    intent, _ = adapter.update((0.30, 0.0, 0.02), dt=1.0)  # 完全对齐后退出
    assert intent is MotionIntent.WHEEL_CRUISE
    assert not adapter.turning


def test_navigation_adapter_turn_holds_before_aligned_release():
    # 【中文注释】校验：转弯保持最短时间后才允许对齐释放（turn_min_hold_sec）。
    parameters = IntentParameters(turn_min_hold_sec=0.30)
    adapter = RollingNavigationAdapter(parameters)
    adapter.update((0.30, 0.20, 0.0), dt=0.02)  # 进入转弯

    intent, _ = adapter.update((0.30, 0.0, 0.02), dt=0.10)  # 保持时间不足
    assert intent is MotionIntent.COORDINATED_TURN
    assert adapter.turning

    intent, _ = adapter.update((0.30, 0.0, 0.02), dt=0.20)  # 累计 0.30s 后退出
    assert intent is MotionIntent.WHEEL_CRUISE
    assert not adapter.turning


def test_navigation_adapter_pure_turn_becomes_guarded_rolling_arc():
    # 【中文注释】校验：纯转弯指令变为受保护的滚动圆弧（带前向速度下限）。
    parameters = IntentParameters()
    adapter = RollingNavigationAdapter(parameters)
    intent, command = adapter.update((0.0, 0.0, 0.50), dt=1.0)

    assert intent is MotionIntent.COORDINATED_TURN
    assert command[0] == parameters.turn_min_forward  # 滚动弧需要最低前向
    assert command[1] == 0.0
    assert command[2] == 0.50


def test_navigation_adapter_turn_first_profile_cancels_cruise_immediately():
    # 【中文注释】校验："转弯优先"配置（turn_max_forward=0）立即取消巡航前向。
    parameters = IntentParameters(turn_max_forward=0.0)
    adapter = RollingNavigationAdapter(parameters)
    adapter.update((0.30, 0.0, 0.0), dt=1.0)  # 先巡航

    intent, command = adapter.update((0.30, 0.20, 0.0), dt=0.02)  # 出现横向 → 转转弯

    assert intent is MotionIntent.COORDINATED_TURN
    assert command[0] == 0.0   # 前向立即清零（无平移转弯）
    assert command[1] == 0.0
    assert command[2] > 0.0    # 只保留偏航


def test_navigation_adapter_zero_stop_is_immediate_and_resets_state():
    # 【中文注释】校验：零指令立即停止并复位状态（不经过斜坡）。
    adapter = RollingNavigationAdapter(IntentParameters())
    adapter.update((0.30, 0.20, 0.0), dt=1.0)  # 运动状态

    intent, command = adapter.update((0.0, 0.0, 0.0), dt=0.02)

    assert intent is MotionIntent.STOPPED
    assert command == (0.0, 0.0, 0.0)
    assert not adapter.turning  # 状态已复位


def test_navigation_adapter_never_requests_autonomous_lateral_motion():
    # 【中文注释】校验：导航适配器从不请求自主横向运动（输出横向恒为 0）。
    adapter = RollingNavigationAdapter(IntentParameters())
    commands = (
        (0.40, 0.20, 0.30),
        (0.40, -0.20, -0.30),
        (0.0, 0.20, 0.0),
        (-0.30, 0.15, 0.10),
    )

    for command in commands:
        intent, output = adapter.update(command, dt=0.10)
        assert intent is not MotionIntent.LATERAL_MANEUVER
        assert output[1] == 0.0  # 横向恒为 0


def test_navigation_adapter_compensates_measured_reverse_dead_zone():
    # 【中文注释】校验：倒车时补偿实测官方策略死区
    # （输出幅值 = offset + gain × |请求|）。
    parameters = IntentParameters()
    adapter = RollingNavigationAdapter(parameters)

    intent, command = adapter.update((-0.05, 0.0, 0.0), dt=1.0)

    assert intent is MotionIntent.WHEEL_CRUISE
    assert math.isclose(  # 补偿公式
        command[0],
        -(parameters.reverse_speed_offset + 0.05 * parameters.reverse_speed_gain),
    )
    assert command[1:] == (0.0, 0.0)


def test_navigation_adapter_compensates_reverse_yaw_under_response():
    # 【中文注释】校验：倒车偏航弱响应被补偿（保持方向并放大幅值）。
    parameters = IntentParameters(
        cruise_yaw_deadband=0.0,
        cruise_yaw_filter_time_constant=0.0,
    )
    adapter = RollingNavigationAdapter(parameters)

    _, command = adapter.update((-0.15, 0.0, 0.15), dt=1.0)

    assert command[0] < -0.35  # 倒车速度被补偿放大
    assert math.isclose(  # 偏航补偿公式
        command[2],
        parameters.reverse_yaw_offset
        + 0.15 * parameters.reverse_yaw_gain,
    )


def test_reverse_compensation_respects_sdk_command_limits():
    # 【中文注释】校验：倒车补偿尊重 SDK 指令上限（不超 max_forward/max_yaw）。
    parameters = IntentParameters()
    adapter = RollingNavigationAdapter(parameters)

    _, command = adapter.update((-0.45, 0.0, -0.65), dt=1.0)

    assert command == (
        -parameters.max_forward,  # 不超过负向前限
        0.0,
        -parameters.max_yaw,      # 不超过负偏航限
    )
