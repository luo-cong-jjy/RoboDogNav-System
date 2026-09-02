# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_velocity_feedback.py
# 所属：m20_nav2_locomotion —— M20 运动控制包的单元测试
# 核心职责：测试有界 M20 实测速度反馈（velocity_feedback.py）。
#   - 误差在死区内不改变参考；
#   - 响应不足时施加有界的前向/偏航校正；
#   - 校正限幅与 SDK 包络同时生效；
#   - 反馈绝不反转请求的运动方向；
#   - 条件积分防止饱和 windup、参考符号翻转清积分、零参考复位积分；
#   - 横向参考直通、非有限测量复位并"故障开放"。
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

"""Unit tests for bounded M20 measured-velocity feedback."""
# 【中文注释】模块说明：有界 M20 实测速度反馈的单元测试。

import math  # 数学库（nan 等）

import pytest  # pytest 测试框架

from m20_nav2_locomotion.velocity_feedback import (  # 被测模块
    MeasuredVelocityFeedback,      #   实测速度反馈控制器
    VelocityFeedbackParameters,    #   反馈参数
)


def test_error_inside_deadband_does_not_change_reference() -> None:
    # 【中文注释】校验：误差在死区内时不改变参考（输出 = 参考，校正为 0）。
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())

    result = controller.update(
        (0.30, 0.0, 0.20),
        (0.28, 0.0, 0.17),  # 测量略低于参考，但在死区内
        0.02,
    )

    assert result.output == (0.30, 0.0, 0.20)
    assert result.correction == (0.0, 0.0, 0.0)


def test_under_response_adds_bounded_forward_and_yaw_correction() -> None:
    # 【中文注释】校验：响应不足（测量明显低于参考）时施加有界前向/偏航校正。
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())

    result = controller.update(
        (0.30, 0.0, 0.30),
        (0.10, 0.0, 0.10),  # 实际只有参考的 1/3
        0.10,
    )

    assert 0.30 < result.output[0] <= 0.40  # 前向输出被放大但不超过校正限幅
    assert 0.30 < result.output[2] <= 0.42  # 偏航输出被放大但不超过校正限幅
    assert result.integral[0] > 0.0         # 积分项为正
    assert result.integral[2] > 0.0


def test_correction_and_sdk_envelope_limits_are_both_enforced() -> None:
    # 【中文注释】校验：校正限幅与 SDK 输出包络同时生效
    # （大误差时输出被夹到 max_forward/max_yaw，校正量被夹到 correction_limit）。
    parameters = VelocityFeedbackParameters(
        linear_kp=10.0,
        yaw_kp=10.0,
        linear_correction_limit=0.05,
        yaw_correction_limit=0.08,
    )
    controller = MeasuredVelocityFeedback(parameters)

    result = controller.update(
        (0.42, 0.0, 0.60),
        (0.0, 0.0, 0.0),  # 测量为零
        0.10,
    )

    assert result.output[0] == parameters.max_forward  # 输出被 SDK 包络夹住
    assert result.output[2] == parameters.max_yaw
    assert result.correction[0] == pytest.approx(0.03)  # 校正量被限幅
    assert result.correction[2] == pytest.approx(0.05)


def test_feedback_never_reverses_the_requested_motion() -> None:
    # 【中文注释】校验：反馈绝不反转请求的运动方向（保号）。
    controller = MeasuredVelocityFeedback(
        VelocityFeedbackParameters(
            linear_kp=10.0,
            yaw_kp=10.0,
        )
    )

    result = controller.update(
        (0.05, 0.0, -0.05),   # 小幅正前向 + 负偏航
        (1.0, 0.0, -1.0),     # 测量远超参考
        0.10,
    )

    assert result.output[0] == 0.0  # 正向参考 → 输出不小于 0
    assert result.output[2] == 0.0  # 负向参考 → 输出不大于 0


def test_conditional_integration_prevents_saturation_windup() -> None:
    # 【中文注释】校验：条件积分（anti-windup）防止饱和时的积分累积。
    controller = MeasuredVelocityFeedback(
        VelocityFeedbackParameters(
            linear_kp=1.0,
            linear_ki=1.0,
        )
    )

    first = controller.update(
        (0.44, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        0.10,
    )
    second = controller.update(
        (0.44, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        0.10,
    )

    assert first.output[0] == 0.45     # 输出饱和于上限
    assert second.output[0] == 0.45
    assert first.integral[0] == 0.0    # 饱和时积分不累积
    assert second.integral[0] == 0.0


def test_reference_sign_change_clears_old_integral() -> None:
    # 【中文注释】校验：参考符号翻转时清除旧积分（重新建立反方向积分）。
    controller = MeasuredVelocityFeedback(
        VelocityFeedbackParameters(
            linear_kp=0.0,
            linear_ki=1.0,
        )
    )
    forward = controller.update(
        (0.20, 0.0, 0.0),
        (0.10, 0.0, 0.0),
        0.10,
    )
    reverse = controller.update(
        (-0.20, 0.0, 0.0),
        (-0.10, 0.0, 0.0),
        0.10,
    )

    assert forward.integral[0] > 0.0    # 正向积分
    assert reverse.integral[0] < 0.0    # 反向后从零重建，符号为负
    assert abs(reverse.integral[0]) == pytest.approx(
        abs(forward.integral[0])        # 幅值对称
    )


def test_zero_reference_resets_axis_integral() -> None:
    # 【中文注释】校验：零参考复位对应轴积分。
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())
    controller.update((0.30, 0.0, 0.20), (0.0, 0.0, 0.0), 0.10)

    result = controller.update(
        (0.0, 0.0, 0.0),      # 参考为零
        (0.20, 0.0, 0.10),
        0.10,
    )

    assert result.output == (0.0, 0.0, 0.0)
    assert result.integral == (0.0, 0.0, 0.0)  # 积分被清零


def test_lateral_reference_passes_through_without_feedback() -> None:
    # 【中文注释】校验：横向参考直通、不参与反馈校正。
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())

    result = controller.update(
        (0.20, 0.07, 0.10),
        (0.10, -0.20, 0.0),  # 横向测量差异很大也不校正
        0.10,
    )

    assert result.output[1] == 0.07    # 横向原样直通
    assert result.correction[1] == 0.0


def test_nonfinite_measurement_resets_and_fails_open() -> None:
    # 【中文注释】校验：非有限测量 → 复位并"故障开放"（输出安全参考，不卡在旧输出）。
    controller = MeasuredVelocityFeedback(VelocityFeedbackParameters())
    controller.update((0.30, 0.0, 0.20), (0.0, 0.0, 0.0), 0.10)

    result = controller.update(
        (0.25, 0.0, 0.15),
        (math.nan, 0.0, 0.0),  # 非有限测量
        0.10,
    )

    assert result.output == (0.25, 0.0, 0.15)  # 直通参考
    assert result.integral == (0.0, 0.0, 0.0)  # 积分已复位
