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
# 【文件职责】test_safety_policy.py —— 安全策略单元测试
# 测试 safety_policy.py 中的纯函数：
#   - clamp_command：平面速度指令限幅；
#   - select_fresh_command：指令源选择与超时失效（手动优先）；
#   - slew_command：加速度限制；
#   - proportional_ramp_command：保持曲率的比例斜坡（含转向变化重启）；
#   - valid_collision_recovery_command：恢复指令形状限制。
# ============================================================================

"""Tests for command arbitration and limiting."""

import pytest  # 测试框架：断言、参数化测试（pytest.approx 近似比较）

from m20_inspection_core.safety_policy import (  # 导入被测模块的纯函数与类型
    TimedCommand,
    clamp_command,
    proportional_ramp_command,
    select_fresh_command,
    slew_command,
    valid_collision_recovery_command,
)


def test_limits_remove_excess_planar_speed() -> None:
    # 【测试】限幅函数应把超限的线速度/角速度钳制到限幅绝对值（保留符号）
    assert clamp_command((2.0, -1.0, 4.0), (0.45, 0.2, 0.65)) == (
        0.45,
        -0.2,
        0.65,
    )


def test_manual_command_has_priority_when_fresh() -> None:
    # 【测试】手动指令新鲜时具有最高优先级（manual_priority=True）
    command, source = select_fresh_command(
        now=10.0,
        timeout=0.5,
        navigation=TimedCommand((0.4, 0.0, 0.0), 9.9),
        manual=TimedCommand((0.0, 0.1, 0.0), 9.8),
        manual_priority=True,
    )
    assert source == 'MANUAL'
    assert command == (0.0, 0.1, 0.0)


def test_stale_commands_fail_to_zero() -> None:
    # 【测试】指令超时（超出新鲜窗口）时应失效并输出零指令（fail-closed）
    command, source = select_fresh_command(
        now=10.0,
        timeout=0.5,
        navigation=TimedCommand((0.4, 0.0, 0.0), 8.0),
        manual=None,
        manual_priority=True,
    )
    assert source == 'COMMAND_TIMEOUT'
    assert command == (0.0, 0.0, 0.0)


def test_ordinary_output_respects_acceleration_limits() -> None:
    # 【测试】常规输出应受线/角加速度限制（dt 内变化量不超过 a*dt）
    output = slew_command(
        current=(0.0, 0.0, 0.0),
        target=(0.45, -0.2, 0.65),
        dt=0.1,
        linear_acceleration=1.0,
        angular_acceleration=1.2,
    )
    assert output == pytest.approx((0.1, -0.1, 0.12))


def test_recovery_ramp_preserves_command_curvature() -> None:
    # 【测试】比例斜坡应保持 vx/wz 比值不变（曲率恒定）
    target = (0.35, 0.0, 0.65)
    output = proportional_ramp_command(
        current=(0.0, 0.0, 0.0),
        target=target,
        dt=0.2,
        ramp_duration=1.0,
    )

    assert output == pytest.approx((0.07, 0.0, 0.13))
    assert output[0] / output[2] == pytest.approx(
        target[0] / target[2]
    )


def test_recovery_ramp_restarts_when_turn_direction_changes() -> None:
    # 【测试】转向方向改变时应从零重新起步（避免经过未验证的混合指令）
    output = proportional_ramp_command(
        current=(0.175, 0.0, 0.325),
        target=(0.35, 0.0, -0.65),
        dt=0.1,
        ramp_duration=1.0,
    )

    assert output == pytest.approx((0.035, 0.0, -0.065))


@pytest.mark.parametrize(  # 参数化测试：一组 (指令, 期望合法性) 用例
    ('command', 'expected'),
    (
        ((0.35, 0.0, 0.65), True),    # 前进滚动转弯：合法
        ((-0.35, 0.0, 0.0), True),    # 直线后退：合法
        ((0.35, 0.0, 0.0), True),     # 直线前进：合法
        ((-0.35, 0.0, 0.35), False),  # 后退带转向：非法
        ((0.0, 0.0, 0.65), False),    # 纯旋转：非法
        ((0.35, 0.01, 0.65), False),  # 含横向速度：非法
        ((0.0, 0.0, 0.0), False),     # 零指令：非法
    ),
)
def test_collision_recovery_command_shape_is_restricted(
    command: tuple[float, float, float],
    expected: bool,
) -> None:
    # 【测试】恢复指令形状校验：只允许前进直行/滚动转弯与直线后退
    assert valid_collision_recovery_command(command) is expected
