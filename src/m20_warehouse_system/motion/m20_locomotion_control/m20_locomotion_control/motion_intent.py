# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：motion_intent.py
# 所属：m20_locomotion_control —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：纯策略逻辑——对 M20 机体速度指令（vx, vy, yaw）进行分类与约束。
#   - MotionIntent 枚举：STOPPED / BACKEND_HOLD / FAULT_HOLD / WHEEL_CRUISE /
#     COORDINATED_TURN / LATERAL_MANEUVER 六种高层运动意图；
#   - IntentParameters：进入 RL 策略前使用的限幅与阈值参数；
#   - clamp_command / classify_intent / constrain_for_intent：指令限幅、意图分类、按意图约束；
#   - RollingNavigationAdapter：把全向（holonomic）路径跟踪转换为稳定的"滚动+偏航"指令
#     （含转弯滞回、航向误差转偏航、倒车死区补偿、巡航偏航滤波、指令斜坡限幅）。
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

"""Pure policy for classifying and constraining M20 body-velocity commands."""
# 【中文注释】模块说明：M20 机体速度指令分类与约束的纯策略（无 ROS 依赖）。

from dataclasses import dataclass   # 数据类装饰器
from enum import Enum               # 枚举基类
import math                         # 数学库：isfinite、atan2、hypot、copysign 等
from typing import Tuple            # 类型提示：元组


PlanarCommand = Tuple[float, float, float]  # 【中文注释】平面速度指令类型别名：(前向速度, 横向速度, 偏航角速度)


class MotionIntent(str, Enum):
    """High-level intent; the ONNX policy still chooses all 16 joint actions."""
    # 【中文注释】高层运动意图枚举；注意：ONNX 策略仍会自行选择全部 16 个关节动作，
    # 这里的意图仅用于指令约束与监控标签，不是离散的 ONNX 步态输入。

    STOPPED = 'STOPPED'                  # 完全停止（指令全零）
    BACKEND_HOLD = 'BACKEND_HOLD'        # 后端未就绪时的保持（零指令）
    FAULT_HOLD = 'FAULT_HOLD'            # 后端故障时的保持（零指令）
    WHEEL_CRUISE = 'WHEEL_CRUISE'        # 轮式巡航（直线/小曲率前进）
    COORDINATED_TURN = 'COORDINATED_TURN'  # 协调转弯（滚动 + 偏航联动）
    LATERAL_MANEUVER = 'LATERAL_MANEUVER'  # 横向机动（横移为主）


@dataclass(frozen=True)  # 【中文注释】冻结数据类：只读的意图参数集
class IntentParameters:
    """Limits and thresholds used before a safe Twist reaches the RL policy."""
    # 【中文注释】安全 Twist 到达 RL 策略之前使用的限幅与阈值参数（默认值供单测/回退使用）。

    max_forward: float = 0.45       # 最大前向速度（m/s）
    max_side: float = 0.20          # 最大横向速度（m/s）
    max_yaw: float = 0.65           # 最大偏航角速度（rad/s）
    deadband_linear: float = 0.01   # 线速度死区（m/s）
    deadband_yaw: float = 0.02      # 偏航角速度死区（rad/s）
    lateral_threshold: float = 0.05 # 横向意图阈值（m/s）
    turn_yaw_threshold: float = 0.25  # 转弯意图偏航阈值（rad/s）
    in_place_linear_threshold: float = 0.08  # 原地转向判定前向速度阈值（m/s）
    turn_curvature_threshold: float = 1.20   # 转弯曲率阈值（1/m）
    curvature_speed_floor: float = 0.05      # 曲率计算速度下限（防除零）
    turn_min_forward: float = 0.35   # 滚动转弯最低稳定前向速度（m/s）
    turn_max_forward: float = 0.45   # 滚动转弯最高前向速度（m/s）
    lateral_max_forward: float = 0.10  # 横移机动最大前向速度（m/s）
    suppress_side_in_cruise: bool = True   # 巡航时是否抑制横向分量
    suppress_side_in_turn: bool = True     # 转弯时是否抑制横向分量
    course_yaw_gain: float = 1.20     # 航向误差 → 偏航增益
    turn_course_enter: float = 0.25   # 进入转弯滞回的航向误差（rad）
    turn_course_exit: float = 0.08    # 退出转弯滞回的航向误差（rad）
    turn_yaw_exit: float = 0.20       # 退出转弯滞回的偏航角速度（rad/s）
    turn_min_hold_sec: float = 0.50   # 转弯滞回最短保持时间（s）
    cruise_yaw_deadband: float = 0.02 # 巡航偏航死区（rad/s）
    cruise_yaw_filter_time_constant: float = 0.12  # 巡航偏航滤波时间常数（s）
    reverse_speed_offset: float = 0.19  # 倒车速度死区补偿偏移（m/s）
    reverse_speed_gain: float = 1.32    # 倒车速度死区补偿增益
    reverse_yaw_offset: float = 0.15    # 倒车偏航补偿偏移（rad/s）
    reverse_yaw_gain: float = 1.00      # 倒车偏航补偿增益
    output_linear_accel: float = 1.0    # 输出线加速度限幅（m/s²）
    output_yaw_accel: float = 1.2       # 输出偏航角加速度限幅（rad/s²）


def _clamp(value: float, limit: float) -> float:
    """Clamp a finite scalar symmetrically; non-finite input becomes zero."""
    # 【中文注释】对称限幅：把标量夹在 [-limit, limit]；非有限输入（NaN/inf）归零。
    if not math.isfinite(value):
        return 0.0
    safe_limit = max(0.0, float(limit))
    return max(-safe_limit, min(value, safe_limit))


def clamp_command(
    command: PlanarCommand,
    parameters: IntentParameters,
) -> PlanarCommand:
    """Apply the RL command envelope without increasing any component."""
    # 【中文注释】对指令施加 RL 指令包络（逐分量对称限幅），任何分量都不会被放大。
    return (
        _clamp(command[0], parameters.max_forward),
        _clamp(command[1], parameters.max_side),
        _clamp(command[2], parameters.max_yaw),
    )


def classify_intent(
    command: PlanarCommand,
    parameters: IntentParameters,
) -> MotionIntent:
    """Classify the requested maneuver from its planar velocity command."""
    # 【中文注释】根据平面速度指令把请求的机动分类为高层意图。
    forward, side, yaw = clamp_command(command, parameters)
    if (  # 三个分量都在死区之内 → 停止
        abs(forward) <= parameters.deadband_linear
        and abs(side) <= parameters.deadband_linear
        and abs(yaw) <= parameters.deadband_yaw
    ):
        return MotionIntent.STOPPED

    if abs(side) >= parameters.lateral_threshold:  # 横向分量超过阈值 → 横移机动
        return MotionIntent.LATERAL_MANEUVER

    curvature = abs(yaw) / max(  # 曲率 = |yaw| / |vx|（分母有下限保护）
        abs(forward),
        max(1e-6, parameters.curvature_speed_floor),
    )
    if (  # 偏航大且（前向速度低 或 曲率大）→ 协调转弯
        abs(yaw) >= parameters.turn_yaw_threshold
        and (
            abs(forward) <= parameters.in_place_linear_threshold
            or curvature >= parameters.turn_curvature_threshold
        )
    ):
        return MotionIntent.COORDINATED_TURN

    return MotionIntent.WHEEL_CRUISE  # 其余情况 → 轮式巡航


def constrain_for_intent(
    command: PlanarCommand,
    parameters: IntentParameters,
) -> Tuple[MotionIntent, PlanarCommand]:
    """Return motion intent and the non-amplified command sent to the SDK."""
    # 【中文注释】返回运动意图以及发给 SDK 的、不会放大的约束后指令。
    forward, side, yaw = clamp_command(command, parameters)
    intent = classify_intent((forward, side, yaw), parameters)

    if intent is MotionIntent.STOPPED:  # 停止：输出全零
        return intent, (0.0, 0.0, 0.0)

    if intent is MotionIntent.WHEEL_CRUISE:  # 巡航：可选抑制横向分量
        if parameters.suppress_side_in_cruise:
            side = 0.0
        return intent, (forward, side, yaw)

    if intent is MotionIntent.COORDINATED_TURN:  # 转弯：前向速度夹到转弯上限，可选抑制横向
        forward = _clamp(forward, parameters.turn_max_forward)
        if parameters.suppress_side_in_turn:
            side = 0.0
        return intent, (forward, side, yaw)

    forward = _clamp(forward, parameters.lateral_max_forward)  # 横移：限制前向分量
    return intent, (forward, side, yaw)


def _slew(value: float, target: float, limit: float, dt: float) -> float:
    """Move one command component toward its target at a bounded rate."""
    # 【中文注释】斜坡限幅：把一个指令分量以有界速率（limit）朝目标移动，防止阶跃。
    if dt <= 0.0:
        return value
    maximum_delta = max(0.0, limit) * dt
    delta = max(-maximum_delta, min(maximum_delta, target - value))
    return value + delta


def _soft_deadband(value: float, deadband: float) -> float:
    """Remove small corrections without introducing a step at the threshold."""
    # 【中文注释】软死区：去掉小幅修正，且在阈值处不产生阶跃（死区外线性过渡）。
    threshold = max(0.0, deadband)
    magnitude = abs(value)
    if magnitude <= threshold:
        return 0.0
    return math.copysign(magnitude - threshold, value)


def _reverse_deadzone_compensation(
    value: float,
    offset: float,
    gain: float,
    limit: float,
) -> float:
    """Map a negative request through the measured official-policy dead zone."""
    # 【中文注释】倒车死区补偿：把负向请求映射穿过实测的官方策略死区。
    # 公式：幅值 = offset + gain·|value|，符号保持为负，并夹在 limit 内。
    if value >= 0.0:
        return value
    magnitude = max(0.0, offset) + max(0.0, gain) * abs(value)
    return -min(max(0.0, limit), magnitude)


def _reverse_yaw_compensation(
    value: float,
    offset: float,
    gain: float,
    limit: float,
    deadband: float,
) -> float:
    """Compensate weak reverse-turn response while preserving yaw direction."""
    # 【中文注释】倒车偏航弱响应补偿：补偿倒车时偏航响应不足，同时保持偏航方向不变。
    if abs(value) <= max(0.0, deadband):
        return 0.0
    magnitude = max(0.0, offset) + max(0.0, gain) * abs(value)
    return math.copysign(min(max(0.0, limit), magnitude), value)


class RollingNavigationAdapter:
    """Convert holonomic path tracking into stable rolling-and-yaw commands."""
    # 【中文注释】滚动导航适配器：把全向路径跟踪转换为稳定的"滚动+偏航"指令。

    def __init__(self, parameters: IntentParameters) -> None:
        # 【中文注释】构造函数：保存参数并复位内部状态。
        self._parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Clear turn hysteresis and filtered output after a hard stop."""
        # 【中文注释】硬停止后清除转弯滞回状态与滤波输出。
        self._turning = False        # 是否处于"转弯优先"滞回状态
        self._turn_hold_elapsed = 0.0  # 转弯滞回已保持时间（s）
        self._last_output = (0.0, 0.0, 0.0)  # 上一次输出指令（供斜坡限幅）
        self._filtered_cruise_yaw = 0.0     # 巡航偏航低通滤波值

    @property
    def turning(self) -> bool:
        """Return whether turn-first hysteresis is currently active."""
        # 【中文注释】返回"转弯优先"滞回是否处于激活状态。
        return self._turning

    def update(
        self,
        command: PlanarCommand,
        dt: float,
    ) -> Tuple[MotionIntent, PlanarCommand]:
        """Adapt one autonomous command while preserving immediate zero stops."""
        # 【中文注释】适配一条自主指令，同时保证零指令能立即停止。
        forward, side, yaw = clamp_command(command, self._parameters)
        if (  # 全部在死区内：立即复位并输出停止
            abs(forward) <= self._parameters.deadband_linear
            and abs(side) <= self._parameters.deadband_linear
            and abs(yaw) <= self._parameters.deadband_yaw
        ):
            self.reset()
            return MotionIntent.STOPPED, (0.0, 0.0, 0.0)

        # SCAN tracks a planar B-spline and can request body-frame lateral
        # velocity while the chassis is not yet tangent to the path. Convert
        # that direction error to yaw instead of asking the wheel-legged policy
        # to crab sideways during an ordinary autonomous patrol.
        # 【中文注释】SCAN 跟踪平面 B 样条，可能在车体尚未与路径相切时请求机体横向速度。
        # 把该方向误差转换为偏航指令，而不是在常规自主巡逻中要求轮腿策略侧移（横移）。
        direction_sign = (  # 前向为负（倒车）时取 -1，否则取 +1
            -1.0
            if forward < -self._parameters.deadband_linear
            else 1.0
        )
        course_error = direction_sign * math.atan2(  # 航向误差 = atan2(side, |forward| 下限保护)
            side,
            max(
                abs(forward),
                self._parameters.curvature_speed_floor,
            ),
        )
        desired_yaw = _clamp(  # 期望偏航 = 原偏航 + 增益 × 航向误差，再限幅
            yaw + self._parameters.course_yaw_gain * course_error,
            self._parameters.max_yaw,
        )
        curvature = abs(desired_yaw) / max(  # 期望偏航对应的曲率
            abs(forward),
            self._parameters.curvature_speed_floor,
        )
        stable_roll_required = (  # 是否要求进入稳定滚动（前向速度低或曲率大且偏航超死区）
            abs(desired_yaw) > self._parameters.deadband_yaw
            and (
                abs(forward)
                <= self._parameters.in_place_linear_threshold
                or curvature
                >= self._parameters.turn_curvature_threshold
            )
        )
        enter_turn = (  # 进入转弯条件：航向误差大或必须稳定滚动
            abs(course_error) >= self._parameters.turn_course_enter
            or stable_roll_required
        )
        exit_turn = (  # 退出转弯条件：不再需要滚动且航向/偏航误差都小
            not stable_roll_required
            and abs(course_error) <= self._parameters.turn_course_exit
            and abs(desired_yaw) <= self._parameters.turn_yaw_exit
        )
        if self._turning:  # 已在转弯滞回中：累计保持时间，满足退出条件且超时后退出
            self._turn_hold_elapsed += max(0.0, dt)
            if (
                exit_turn
                and self._turn_hold_elapsed
                >= self._parameters.turn_min_hold_sec
            ):
                self._turning = False
                self._turn_hold_elapsed = 0.0
        elif enter_turn:  # 满足进入条件：进入转弯滞回
            self._turning = True
            self._turn_hold_elapsed = 0.0

        rolling_speed = math.copysign(  # 滚动速度 = min(|vx,vy| 合成, 最大前向)，带符号
            min(
                math.hypot(forward, side),
                self._parameters.max_forward,
            ),
            forward if abs(forward) > 1.0e-9 else 1.0,
        )
        rolling_speed = _reverse_deadzone_compensation(  # 倒车时补偿官方死区
            rolling_speed,
            self._parameters.reverse_speed_offset,
            self._parameters.reverse_speed_gain,
            self._parameters.max_forward,
        )
        if rolling_speed < -self._parameters.deadband_linear:  # 真正倒车时补偿偏航弱响应
            desired_yaw = _reverse_yaw_compensation(
                desired_yaw,
                self._parameters.reverse_yaw_offset,
                self._parameters.reverse_yaw_gain,
                self._parameters.max_yaw,
                self._parameters.deadband_yaw,
            )
        if self._turning:  # 转弯模式：前向速度夹到转弯上限，必要时抬升到转弯下限
            turn_limit = min(
                self._parameters.max_forward,
                max(0.0, self._parameters.turn_max_forward),
            )
            target_forward = _clamp(rolling_speed, turn_limit)
            if (
                stable_roll_required
                and turn_limit > self._parameters.deadband_linear
            ):
                turn_floor = min(  # 转弯速度下限：保证稳定滚动，避免低速爬行
                    turn_limit,
                    max(0.0, self._parameters.turn_min_forward),
                )
                direction = -1.0 if rolling_speed < 0.0 else 1.0
                target_forward = direction * max(
                    abs(target_forward),
                    turn_floor,
                )
            target_yaw = desired_yaw
            self._filtered_cruise_yaw = target_yaw  # 转弯期间同步刷新巡航滤波值
            intent = MotionIntent.COORDINATED_TURN
        else:  # 巡航模式：前向速度直通，偏航经软死区 + 低通滤波
            target_forward = rolling_speed
            target_yaw = _soft_deadband(
                desired_yaw,
                self._parameters.cruise_yaw_deadband,
            )
            time_constant = max(  # 滤波时间常数（一阶低通 alpha = dt/(T+dt)）
                0.0,
                self._parameters.cruise_yaw_filter_time_constant,
            )
            alpha = (
                1.0
                if time_constant <= 0.0
                else max(0.0, dt) / (time_constant + max(0.0, dt))
            )
            self._filtered_cruise_yaw += alpha * (
                target_yaw - self._filtered_cruise_yaw
            )
            target_yaw = self._filtered_cruise_yaw
            intent = MotionIntent.WHEEL_CRUISE

        turn_without_translation = (  # "原地转向"模式：转弯上限<=死区 → 前向输出 0
            self._turning
            and self._parameters.turn_max_forward
            <= self._parameters.deadband_linear
        )
        output_forward = (
            0.0
            if turn_without_translation
            else _slew(  # 前向输出做加速度斜坡限幅
                self._last_output[0],
                target_forward,
                self._parameters.output_linear_accel,
                dt,
            )
        )
        output = (  # 输出指令：(前向, 横向恒为 0, 偏航斜坡限幅)
            output_forward,
            0.0,
            _slew(
                self._last_output[2],
                target_yaw,
                self._parameters.output_yaw_accel,
                dt,
            ),
        )
        self._last_output = output
        return intent, output
