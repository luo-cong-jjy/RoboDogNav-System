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
# 【文件职责】safety_policy.py —— 速度安全策略（纯函数，无 ROS 依赖）
# 本文件是 m20_inspection_core 的"安全监督"纯逻辑模块，供
# safety_supervisor_node.py 调用，主要包括：
#   1) 平面速度指令的限幅（clamp_command）与线性/角加速度限制（slew_command）；
#   2) 保持指令曲率不变的比例斜坡（proportional_ramp_command）；
#   3) 碰撞恢复指令形状校验（valid_collision_recovery_command）；
#   4) 多来源指令选择（select_fresh_command，支持手动优先与超时失效）。
# ============================================================================

"""Pure command limiting and source selection for the safety supervisor."""

# ------------------------- 标准库导入 -------------------------
from dataclasses import dataclass  # 数据类装饰器：用于定义带时间戳的指令值对象
from typing import Optional, Tuple  # 类型提示：Optional 可空值 / Tuple 元组


PlanarCommand = Tuple[float, float, float]  # 平面指令类型别名：(vx, vy, wz) 即线速度 x/y 与角速度 z


@dataclass(frozen=True)  # 冻结数据类：指令与时间戳打包后不可变
class TimedCommand:
    """A planar command and its monotonic receipt time in seconds."""
    # 【中文】一条平面指令及其单调时钟接收时间（秒），用于判断指令是否超时失效

    command: PlanarCommand  # 平面速度指令 (vx, vy, wz)
    stamp: float            # 指令到达时的单调时钟时间（秒）


def clamp_command(
    command: PlanarCommand,
    limits: PlanarCommand,
) -> PlanarCommand:
    """Clamp x/y/yaw command components symmetrically."""
    # 【中文】将指令的 x/y/yaw 分量按对称限幅钳制到 [-limit, limit]
    # 【参数】command - 原始平面指令；limits - 各分量的最大绝对值 (vx_max, vy_max, wz_max)
    # 【返回】钳制后的平面指令（各分量不超过限幅绝对值）
    return tuple(
        max(-abs(limit), min(abs(limit), value))
        for value, limit in zip(command, limits)
    )


def slew_command(
    current: PlanarCommand,
    target: PlanarCommand,
    dt: float,
    linear_acceleration: float,
    angular_acceleration: float,
) -> PlanarCommand:
    """Apply ordinary linear and angular acceleration limits."""
    # 【中文】对指令应用常规的线加速度与角加速度限制（每步最多变化 a*dt）
    # 【参数】current - 当前输出指令；target - 目标指令；dt - 时间步长（秒）；
    #        linear_acceleration - 线加速度上限；angular_acceleration - 角加速度上限
    # 【返回】受加速度限制后的新输出指令
    if dt <= 0.0:
        return target
    deltas = (
        max(0.0, linear_acceleration) * dt,
        max(0.0, linear_acceleration) * dt,
        max(0.0, angular_acceleration) * dt,
    )
    output = []
    for old, new, maximum_delta in zip(current, target, deltas):
        difference = max(-maximum_delta, min(maximum_delta, new - old))
        output.append(old + difference)
    return tuple(output)


def proportional_ramp_command(
    current: PlanarCommand,
    target: PlanarCommand,
    dt: float,
    ramp_duration: float,
) -> PlanarCommand:
    """
    Ramp all components by one scale so command curvature stays constant.

    Collision recovery starts from a hard stop.  If its direction changes,
    restart from zero rather than crossing through an unvalidated mixed
    command.
    """
    # 【中文】按统一比例因子对所有分量进行斜坡，保持指令曲率不变。
    # 碰撞恢复从硬停状态开始；若方向改变，则从零重新起步，避免经过未验证的混合指令。
    # 【参数】current - 当前输出指令；target - 目标指令；dt - 时间步长；ramp_duration - 斜坡总时长
    # 【返回】按比例斜坡后的指令
    if ramp_duration <= 0.0:
        return target
    if dt <= 0.0:
        return current
    norm_squared = sum(value * value for value in target)
    if norm_squared <= 1.0e-12:
        return target
    alignment = sum(
        old * new for old, new in zip(current, target)
    )
    if alignment <= 0.0:
        current_scale = 0.0
    else:
        current_scale = max(
            0.0,
            min(
                1.0,
                alignment / norm_squared,
            ),
        )
    scale = min(1.0, current_scale + dt / ramp_duration)
    return tuple(scale * value for value in target)


def valid_collision_recovery_command(command: PlanarCommand) -> bool:
    """
    Accept only recovery shapes validated for the current M20 policy.

    Forward recovery may be straight or a rolling turn after the collision
    guard validates its complete sweep. Straight reverse is also allowed as a
    checked escape. Pure yaw, lateral motion, and reverse-yaw remain forbidden.
    """
    # 【中文】仅接受当前 M20 策略已验证的恢复指令形状：
    # 前进恢复可以是直行或经碰撞守卫验证完整扫掠后的滚动转弯；允许受检的直线后退；
    # 纯旋转、横向移动与后退转向一律禁止。
    # 【参数】command - 待校验的平面指令 (vx, vy, wz)
    # 【返回】True 表示指令形状符合 M20 恢复策略
    vx, vy, wz = command
    epsilon = 1.0e-3
    if abs(vy) > 1.0e-9 or abs(vx) <= epsilon:
        return False
    if vx < -epsilon:
        return abs(wz) <= epsilon
    return True


def select_fresh_command(
    now: float,
    timeout: float,
    navigation: Optional[TimedCommand],
    manual: Optional[TimedCommand],
    manual_priority: bool,
) -> Tuple[PlanarCommand, str]:
    """Select the highest-priority command that has not timed out."""
    # 【中文】选择优先级最高且未超时失效的指令源
    # 【参数】now - 当前单调时钟；timeout - 指令最大新鲜时长；navigation - 导航指令；
    #        manual - 手动指令；manual_priority - 手动是否优先
    # 【返回】(指令, 来源名称)；若无新鲜指令则返回 ((0,0,0), 'COMMAND_TIMEOUT')

    def fresh(sample: Optional[TimedCommand]) -> bool:
        # 判断某条指令是否在有效时间窗内（未超时）
        return sample is not None and 0.0 <= now - sample.stamp <= timeout

    ordered = (
        (('MANUAL', manual), ('NAVIGATION', navigation))
        if manual_priority
        else (('NAVIGATION', navigation), ('MANUAL', manual))
    )
    for source, sample in ordered:
        if fresh(sample):
            return sample.command, source
    return (0.0, 0.0, 0.0), 'COMMAND_TIMEOUT'
