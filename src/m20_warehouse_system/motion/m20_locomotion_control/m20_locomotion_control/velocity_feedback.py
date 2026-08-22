# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：velocity_feedback.py
# 所属：m20_locomotion_control —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：M20 滚动适配器内环的"有界速度反馈"。
#   - 用实测机体速度（Odometry/TwistStamped 提供的 base_link 线速度与偏航角速度）
#     对前向速度与偏航参考指令做有界 PI 校正；
#   - 特点：保持指令符号不反转、条件积分（anti-windup 抗饱和）、
#     参考死区（参考小于阈值时直通）、误差软死区、输出限幅；
#   - 横向分量不参与反馈（直通）。
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

"""Bounded inner-loop velocity feedback for the M20 rolling adapter."""
# 【中文注释】模块说明：M20 滚动适配器的有界内环速度反馈。

from dataclasses import dataclass   # 数据类装饰器
import math                         # 数学库：isfinite、copysign 等
from typing import Tuple            # 类型提示：元组


PlanarCommand = Tuple[float, float, float]  # 【中文注释】平面速度指令类型别名：(vx, vy, yaw)


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class VelocityFeedbackParameters:
    """Gains and safety bounds for measured body-velocity compensation."""
    # 【中文注释】实测机体速度补偿的增益与安全边界参数。

    linear_kp: float = 0.30            # 线速度比例增益
    linear_ki: float = 0.08            # 线速度积分增益
    yaw_kp: float = 0.20               # 偏航比例增益
    yaw_ki: float = 0.05               # 偏航积分增益
    linear_error_deadband: float = 0.03  # 线速度误差死区（m/s）
    yaw_error_deadband: float = 0.04    # 偏航误差死区（rad/s）
    linear_integral_limit: float = 0.20 # 线速度积分限幅
    yaw_integral_limit: float = 0.25    # 偏航积分限幅
    linear_correction_limit: float = 0.10  # 线速度校正量限幅
    yaw_correction_limit: float = 0.12    # 偏航校正量限幅
    max_forward: float = 0.45          # 输出线速度上限（m/s）
    max_yaw: float = 0.65              # 输出偏航角速度上限（rad/s）
    reference_linear_deadband: float = 0.01  # 线速度参考死区（参考小于它时直通）
    reference_yaw_deadband: float = 0.02     # 偏航参考死区


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class VelocityFeedbackResult:
    """One controller update with values suitable for diagnostics."""
    # 【中文注释】一次控制器更新的输出结果（含便于诊断的中间值）。

    output: PlanarCommand     # 校正后的输出指令
    error: PlanarCommand      # 误差（前向/偏航轴）
    correction: PlanarCommand # 校正量
    integral: PlanarCommand   # 积分项


def _finite(value: float) -> bool:
    """Return whether a scalar is finite."""
    # 【中文注释】判断标量是否有限。
    return math.isfinite(float(value))


def _clamp(value: float, limit: float) -> float:
    """Clamp a scalar symmetrically around zero."""
    # 【中文注释】把标量对称夹到 [-limit, limit]。
    safe_limit = max(0.0, float(limit))
    return max(-safe_limit, min(float(value), safe_limit))


def _soft_deadband(value: float, deadband: float) -> float:
    """Remove a central error band without a discontinuous correction."""
    # 【中文注释】软死区：去掉中心误差带且不产生不连续的校正。
    threshold = max(0.0, float(deadband))
    magnitude = abs(float(value))
    if magnitude <= threshold:
        return 0.0
    return math.copysign(magnitude - threshold, value)


class MeasuredVelocityFeedback:
    """Apply bounded PI correction without reversing the requested motion."""
    # 【中文注释】实测速度反馈控制器：施加有界 PI 校正，且不反转请求的运动方向。

    def __init__(self, parameters: VelocityFeedbackParameters) -> None:
        # 【中文注释】构造函数：保存参数并复位。
        self._parameters = parameters
        self.reset()

    def reset(self) -> None:
        """Clear all integral and reference-sign history."""
        # 【中文注释】清除全部积分与参考符号历史。
        self._linear_integral = 0.0   # 线速度积分项
        self._yaw_integral = 0.0      # 偏航积分项
        self._last_linear_reference = 0.0  # 上一次线速度参考（用于符号变化检测）
        self._last_yaw_reference = 0.0     # 上一次偏航参考

    def _update_axis(
        self,
        *,
        reference: float,
        measurement: float,
        dt: float,
        kp: float,
        ki: float,
        error_deadband: float,
        reference_deadband: float,
        integral_limit: float,
        correction_limit: float,
        output_limit: float,
        integral: float,
        last_reference: float,
    ) -> Tuple[float, float, float, float]:
        """Update one sign-preserving PI channel with anti-windup."""
        # 【中文注释】更新一条"保号"PI 通道（含抗饱和 anti-windup）。
        # 返回 (输出, 误差, 校正量, 积分)。
        if abs(reference) <= max(0.0, reference_deadband):  # 参考低于死区 → 直通
            return reference, 0.0, 0.0, 0.0
        if reference * last_reference < 0.0:  # 参考符号翻转 → 清零积分
            integral = 0.0

        error = _soft_deadband(  # 误差（带软死区）
            reference - measurement,
            error_deadband,
        )
        proposed_integral = _clamp(  # 候选积分 = 旧积分 + 误差 × dt，再限幅
            integral + error * max(0.0, dt),
            integral_limit,
        )

        def output_for(
            candidate_integral: float,
        ) -> Tuple[float, float, float]:
            # 【中文注释】由候选积分计算输出：(输出, 校正量, 未限幅值)。
            correction = _clamp(
                max(0.0, kp) * error
                + max(0.0, ki) * candidate_integral,
                correction_limit,
            )
            unconstrained = reference + correction
            bounded = _clamp(unconstrained, output_limit)
            if reference > 0.0:      # 保号：输出不得越过 0 反转方向
                bounded = max(0.0, bounded)
            else:
                bounded = min(0.0, bounded)
            return bounded, bounded - reference, unconstrained

        output, correction, unconstrained = output_for(
            proposed_integral
        )
        saturation = unconstrained - output  # 饱和量
        if saturation * error > 0.0:  # 条件积分：饱和且误差同号时保持旧积分（抗 windup）
            proposed_integral = integral
            output, correction, _ = output_for(proposed_integral)
        return output, error, correction, proposed_integral

    def update(
        self,
        reference: PlanarCommand,
        measurement: PlanarCommand,
        dt: float,
    ) -> VelocityFeedbackResult:
        """Correct forward and yaw references from measured body velocity."""
        # 【中文注释】用实测机体速度校正前向与偏航参考（横向直通）。
        if (  # 非有限输入 → 复位并"安全直通"（fail-open）
            not all(_finite(value) for value in reference)
            or not all(_finite(value) for value in measurement)
            or not _finite(dt)
        ):
            self.reset()
            safe_reference = tuple(
                float(value) if _finite(value) else 0.0
                for value in reference
            )
            return VelocityFeedbackResult(
                output=safe_reference,
                error=(0.0, 0.0, 0.0),
                correction=(0.0, 0.0, 0.0),
                integral=(0.0, 0.0, 0.0),
            )

        forward, linear_error, linear_correction, linear_integral = (
            self._update_axis(  # 前向轴 PI 校正
                reference=float(reference[0]),
                measurement=float(measurement[0]),
                dt=max(0.0, min(0.10, float(dt))),  # dt 夹到 [0, 0.1]s
                kp=self._parameters.linear_kp,
                ki=self._parameters.linear_ki,
                error_deadband=self._parameters.linear_error_deadband,
                reference_deadband=(
                    self._parameters.reference_linear_deadband
                ),
                integral_limit=self._parameters.linear_integral_limit,
                correction_limit=(
                    self._parameters.linear_correction_limit
                ),
                output_limit=self._parameters.max_forward,
                integral=self._linear_integral,
                last_reference=self._last_linear_reference,
            )
        )
        yaw, yaw_error, yaw_correction, yaw_integral = self._update_axis(  # 偏航轴 PI 校正
            reference=float(reference[2]),
            measurement=float(measurement[2]),
            dt=max(0.0, min(0.10, float(dt))),
            kp=self._parameters.yaw_kp,
            ki=self._parameters.yaw_ki,
            error_deadband=self._parameters.yaw_error_deadband,
            reference_deadband=self._parameters.reference_yaw_deadband,
            integral_limit=self._parameters.yaw_integral_limit,
            correction_limit=self._parameters.yaw_correction_limit,
            output_limit=self._parameters.max_yaw,
            integral=self._yaw_integral,
            last_reference=self._last_yaw_reference,
        )
        self._linear_integral = linear_integral   # 持久化积分与参考符号
        self._yaw_integral = yaw_integral
        self._last_linear_reference = float(reference[0])
        self._last_yaw_reference = float(reference[2])
        return VelocityFeedbackResult(
            output=(forward, float(reference[1]), yaw),  # 横向分量原样直通
            error=(linear_error, 0.0, yaw_error),
            correction=(linear_correction, 0.0, yaw_correction),
            integral=(linear_integral, 0.0, yaw_integral),
        )
