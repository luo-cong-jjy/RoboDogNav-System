# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：capability_profile.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：M20 运动能力配置文件的加载与校验。
#   - 定义 M20CapabilityProfile 数据类：M20 运动指令上限（前进/横移/偏航）、
#     滚动转弯（rolling-turn）包络、倒车跟踪补偿、横向漂移模型、逃逸恢复
#     （recovery）等所有限幅参数的"单一事实来源"；
#   - 从 YAML 文件加载并严格校验（数值必须有限、布尔必须为 bool、
#     内部一致性约束如 0 <= exit < enter <= pi 等）；
#   - 提供 intent_parameters / collision_guard_parameters / controller_parameters /
#     safety_parameters / sdk_parameters 五组 ROS 参数导出，供不同节点使用。
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

"""Validated, versioned M20 motion-capability configuration."""
# 【中文注释】模块说明：经过校验、带版本号的 M20 运动能力配置（纯 Python，无 ROS 依赖）。

from __future__ import annotations  # 延迟求值类型注解（兼容旧版 Python 的类型提示语法）

from dataclasses import dataclass   # 数据类装饰器：自动生成 __init__/__repr__ 等
import math                         # 数学库：isfinite（有限性检查）、hypot（斜边）等
from pathlib import Path            # 跨平台路径对象（扩展用户目录、解析绝对路径）
from typing import Any, Mapping     # 类型提示：Any 任意类型、Mapping 只读映射

import yaml                         # PyYAML：解析 YAML 配置文件


def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    # 【中文注释】从配置字典中取出一个子节（必须是映射/字典），否则抛 ValueError。
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f'capability profile section {key!r} is missing')
    return value


def _number(parent: Mapping[str, Any], key: str) -> float:
    # 【中文注释】从配置字典中取出一个数值（int/float 均可，bool 除外）并转为 float，
    # 同时校验必须为有限数（拒绝 NaN / inf）。
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'capability profile value {key!r} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'capability profile value {key!r} must be finite')
    return result


def _boolean(parent: Mapping[str, Any], key: str) -> bool:
    # 【中文注释】从配置字典中取出一个布尔值（必须是真正的 bool 类型），否则抛 ValueError。
    value = parent.get(key)
    if not isinstance(value, bool):
        raise ValueError(f'capability profile value {key!r} must be boolean')
    return value


@dataclass(frozen=True)  # 【中文注释】冻结数据类：实例创建后不可修改，保证配置线程安全
class M20CapabilityProfile:
    """Single source of truth for M20 command and swept-turn limits."""
    # 【中文注释】M20 能力配置文件：M20 运动指令与转弯扫掠限幅参数的"单一事实来源"。

    profile_id: str                 # 配置文件的唯一标识（版本号，如 m20_policy_v1）
    body_length: float              # 机身长度（米，用于转弯扫掠半径计算）
    body_width: float               # 机身宽度（米）
    body_height: float              # 机身高度（米）
    max_forward: float              # 最大前进/后退速度（m/s）
    max_side: float                 # 最大横向速度（m/s）
    max_yaw: float                  # 最大偏航角速度（rad/s）
    supports_autonomous_lateral: bool  # 是否支持自主横向运动（厂家 Cmd25 支持 Y，但实机未标定前关闭）
    supports_reverse_tracking: bool   # 是否支持倒车跟踪（负向 vx 跟踪）
    supports_zero_radius_yaw: bool    # 是否支持原地零半径偏航（原地转向）
    deadband_linear: float          # 线速度死区（m/s）：小于该值视为零
    deadband_yaw: float             # 偏航角速度死区（rad/s）
    lateral_threshold: float        # 横向意图判定阈值（m/s）：|side| 超过它判为横移机动
    turn_yaw_threshold: float       # 转弯意图判定阈值（rad/s）
    in_place_linear_threshold: float  # 原地转向判定阈值（m/s）：前向速度低于它且 yaw 大时判为转弯
    turn_curvature_threshold: float   # 转弯曲率判定阈值（1/m）：|yaw|/|vx| 超过它判为转弯
    curvature_speed_floor: float    # 曲率计算时的最小速度下限（防止除零）
    turn_min_forward: float         # 滚动转弯最低稳定前向速度（m/s）
    turn_max_forward: float         # 滚动转弯最高前向速度（m/s）
    lateral_max_forward: float      # 横移机动时允许的最大前向速度（m/s）
    course_yaw_gain: float          # 航向误差 → 偏航指令的比例增益（方向误差转偏航）
    turn_course_enter: float        # 进入转弯滞回的航向误差阈值（rad）
    turn_course_exit: float         # 退出转弯滞回的航向误差阈值（rad）
    turn_yaw_exit: float            # 退出转弯滞回的偏航角速度阈值（rad/s）
    turn_min_hold_sec: float        # 转弯滞回最短保持时间（秒）
    cruise_yaw_deadband: float      # 巡航偏航死区（rad/s）
    cruise_yaw_filter_time_constant: float  # 巡航偏航低通滤波时间常数（秒）
    reverse_speed_offset: float     # 倒车速度死区补偿偏移量（m/s）
    reverse_speed_gain: float       # 倒车速度死区补偿增益
    reverse_yaw_offset: float       # 倒车偏航弱响应补偿偏移量（rad/s）
    reverse_yaw_gain: float         # 倒车偏航弱响应补偿增益
    output_linear_accel: float      # 输出线加速度限幅（m/s²，指令斜率限制）
    output_yaw_accel: float         # 输出偏航角加速度限幅（rad/s²）
    reverse_tracking_enter_angle: float  # 进入倒车跟踪的航向误差阈值（rad，滞回上限）
    reverse_tracking_exit_angle: float   # 退出倒车跟踪的航向误差阈值（rad，滞回下限）
    reverse_tracking_min_hold_sec: float # 倒车跟踪最短保持时间（秒）
    reverse_tracking_entry_alignment: float  # 进入倒车跟踪的路径对齐角阈值（rad）
    reverse_tracking_exit_alignment: float   # 退出倒车跟踪的路径对齐角阈值（rad）
    positive_yaw_lateral_drift: float   # 正偏航（左转）时实测的横向漂移速度（m/s）
    negative_yaw_lateral_drift: float   # 负偏航（右转）时实测的横向漂移速度（m/s）
    opposite_lateral_uncertainty: float # 反向横移不确定性（m/s，碰撞预测模型用）
    recovery_forward_speed: float    # 逃逸恢复动作的前进速度（m/s）
    recovery_lookahead_sec: float    # 恢复动作前视时间（秒）
    recovery_min_yaw_rate: float     # 恢复动作最低偏航角速度（rad/s）
    recovery_clear_confirm_sec: float  # 障碍清除确认时间（秒）
    recovery_rearm_clear_sec: float    # 重新布防所需连续清除时间（秒，>= clear_confirm_sec）
    recovery_max_active_sec: float     # 恢复动作最大持续时间（秒，有界逃逸动作）
    recovery_max_displacement_m: float # 恢复动作最大位移（米）
    recovery_progress_timeout_sec: float  # 恢复进展超时（秒）
    recovery_min_progress_m: float    # 恢复最小进展距离（米）
    measurement_source: str          # 速度测量来源（如 factory_motion_status / external_odometry_base_link_twist）

    @property
    def minimum_centerline_turn_radius(self) -> float:
        """Smallest calibrated rolling radius at the body centre."""
        # 【中文注释】最小中心线转弯半径 = 最低稳定滚动速度 / 最大偏航角速度（运动学约束）。
        return self.turn_min_forward / self.max_yaw

    @property
    def turn_swept_radius(self) -> float:
        """Outer-corner sweep for the minimum-radius rolling turn."""
        # 【中文注释】最小半径滚动转弯时，机身外角点的扫掠半径：
        # 横向 = 中心线转弯半径 + 半宽，再与半长合成斜边（勾股定理）。
        lateral_extent = (
            self.minimum_centerline_turn_radius + self.body_width / 2.0
        )
        return math.hypot(lateral_extent, self.body_length / 2.0)

    def intent_parameters(self) -> dict[str, Any]:
        """ROS parameters shared by pre-safety and final command gates."""
        # 【中文注释】导出运动意图参数：安全预测前与最终指令门共用的 ROS 参数集。
        return {
            'capability_profile_id': self.profile_id,
            'minimum_centerline_turn_radius': (
                self.minimum_centerline_turn_radius
            ),
            'turn_swept_radius': self.turn_swept_radius,
            'max_forward': self.max_forward,
            'max_side': self.max_side,
            'max_yaw': self.max_yaw,
            'deadband_linear': self.deadband_linear,
            'deadband_yaw': self.deadband_yaw,
            'lateral_threshold': self.lateral_threshold,
            'turn_yaw_threshold': self.turn_yaw_threshold,
            'in_place_linear_threshold': self.in_place_linear_threshold,
            'turn_curvature_threshold': self.turn_curvature_threshold,
            'curvature_speed_floor': self.curvature_speed_floor,
            'turn_min_forward': self.turn_min_forward,
            'turn_max_forward': self.turn_max_forward,
            'lateral_max_forward': self.lateral_max_forward,
            'suppress_side_in_cruise': (
                not self.supports_autonomous_lateral  # 不支持自主横移 → 巡航时抑制侧向分量
            ),
            'suppress_side_in_turn': (
                not self.supports_autonomous_lateral  # 不支持自主横移 → 转弯时抑制侧向分量
            ),
            'course_yaw_gain': self.course_yaw_gain,
            'turn_course_enter': self.turn_course_enter,
            'turn_course_exit': self.turn_course_exit,
            'turn_yaw_exit': self.turn_yaw_exit,
            'turn_min_hold_sec': self.turn_min_hold_sec,
            'cruise_yaw_deadband': self.cruise_yaw_deadband,
            'cruise_yaw_filter_time_constant': (
                self.cruise_yaw_filter_time_constant
            ),
            'reverse_speed_offset': self.reverse_speed_offset,
            'reverse_speed_gain': self.reverse_speed_gain,
            'reverse_yaw_offset': self.reverse_yaw_offset,
            'reverse_yaw_gain': self.reverse_yaw_gain,
            'output_linear_accel': self.output_linear_accel,
            'output_yaw_accel': self.output_yaw_accel,
        }

    def collision_guard_parameters(self) -> dict[str, Any]:
        """Motion-model parameters; map-clearance geometry stays separate."""
        # 【中文注释】导出碰撞防护模型参数：仅运动模型参数；地图净空几何另由 m20_scan_navigation 负责。
        return {
            'capability_profile_id': self.profile_id,
            'minimum_centerline_turn_radius': (
                self.minimum_centerline_turn_radius
            ),
            'turn_swept_radius': self.turn_swept_radius,
            'max_linear_x': self.max_forward,
            'max_linear_y': self.max_side,
            'max_angular_z': self.max_yaw,
            'model_positive_yaw_lateral_drift': (
                self.positive_yaw_lateral_drift
            ),
            'model_negative_yaw_lateral_drift': (
                self.negative_yaw_lateral_drift
            ),
            'model_opposite_lateral_uncertainty': (
                self.opposite_lateral_uncertainty
            ),
            'model_reference_yaw_rate': self.max_yaw,
            'recovery_forward_speed': self.recovery_forward_speed,
            'recovery_positive_yaw_lateral_drift': (
                self.positive_yaw_lateral_drift
            ),
            'recovery_negative_yaw_lateral_drift': (
                self.negative_yaw_lateral_drift
            ),
            'recovery_opposite_lateral_uncertainty': (
                self.opposite_lateral_uncertainty
            ),
            'recovery_lookahead_sec': self.recovery_lookahead_sec,
            'recovery_min_yaw_rate': self.recovery_min_yaw_rate,
            'recovery_clear_confirm_sec': self.recovery_clear_confirm_sec,
            'recovery_rearm_clear_sec': self.recovery_rearm_clear_sec,
            'recovery_max_active_sec': self.recovery_max_active_sec,
            'recovery_max_displacement_m': (
                self.recovery_max_displacement_m
            ),
            'recovery_progress_timeout_sec': (
                self.recovery_progress_timeout_sec
            ),
            'recovery_min_progress_m': self.recovery_min_progress_m,
        }

    def controller_parameters(self) -> dict[str, Any]:
        """M20-only execution policy layered after vendor SCAN control."""
        # 【中文注释】导出控制器参数：叠加在厂商 SCAN 控制之后的 M20 专属执行策略（双向跟踪）。
        return {
            'bidirectional_tracking_enabled': (
                self.supports_reverse_tracking
            ),
            'reverse_tracking_enter_angle': (
                self.reverse_tracking_enter_angle
            ),
            'reverse_tracking_exit_angle': (
                self.reverse_tracking_exit_angle
            ),
            'reverse_tracking_min_hold_sec': (
                self.reverse_tracking_min_hold_sec
            ),
            'reverse_tracking_entry_alignment': (
                self.reverse_tracking_entry_alignment
            ),
            'reverse_tracking_exit_alignment': (
                self.reverse_tracking_exit_alignment
            ),
        }

    def safety_parameters(self) -> dict[str, Any]:
        """Limits for the command multiplexer without duplicating slew."""
        # 【中文注释】导出安全参数：指令多路复用器的限幅（不重复斜坡限制）。
        return {
            'capability_profile_id': self.profile_id,
            'max_linear_x': self.max_forward,
            'max_linear_y': self.max_side,
            'max_angular_z': self.max_yaw,
        }

    def sdk_parameters(self) -> dict[str, float]:
        """Limits understood by the vendored rl_deploy_cmdvel node."""
        # 【中文注释】导出 SDK 参数：厂商 rl_deploy_cmdvel 节点能识别的限幅值。
        return {
            'max_forward': self.max_forward,
            'max_side': self.max_side,
            'max_yaw': self.max_yaw,
        }


def load_capability_profile(path: str | Path) -> M20CapabilityProfile:
    """Load and validate a capability profile without ROS dependencies."""
    # 【中文注释】加载并校验能力配置文件（无 ROS 依赖）。
    # 参数：path —— YAML 配置文件路径（字符串或 Path）；
    # 返回：校验通过后的 M20CapabilityProfile 对象；任何不一致都会抛 ValueError。
    profile_path = Path(path).expanduser().resolve()  # 展开 ~ 并解析为绝对路径
    with profile_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)            # 安全加载 YAML（不执行任意对象）
    if not isinstance(document, Mapping):
        raise ValueError('capability profile root must be a mapping')
    if document.get('schema_version') != 1:          # 校验 schema 版本号
        raise ValueError('capability profile schema_version must be 1')
    profile_id = document.get('profile_id')
    if not isinstance(profile_id, str) or not profile_id.strip():
        raise ValueError('capability profile_id must be a non-empty string')

    body = _mapping(document, 'body')        # 机身几何尺寸节
    command = _mapping(document, 'command')  # 运动指令上限节
    turn = _mapping(document, 'turn')        # 转弯/滚动节
    tracking = _mapping(document, 'tracking')  # 跟踪/滞回节
    reverse = _mapping(document, 'reverse_compensation')  # 倒车死区补偿节
    drift = _mapping(document, 'measured_drift')           # 实测漂移节
    recovery = _mapping(document, 'recovery')              # 逃逸恢复节
    feedback = _mapping(document, 'feedback')              # 速度反馈来源节
    measurement_source = feedback.get('measurement_source')
    if not isinstance(measurement_source, str) or not measurement_source:
        raise ValueError('feedback.measurement_source must be a string')

    # 【中文注释】逐字段从各节中提取并构造数据类，所有数值均经 _number 校验。
    result = M20CapabilityProfile(
        profile_id=profile_id,
        body_length=_number(body, 'length_m'),
        body_width=_number(body, 'width_m'),
        body_height=_number(body, 'height_m'),
        max_forward=_number(command, 'max_forward_mps'),
        max_side=_number(command, 'max_side_mps'),
        max_yaw=_number(command, 'max_yaw_radps'),
        supports_autonomous_lateral=_boolean(
            command, 'supports_autonomous_lateral'
        ),
        supports_reverse_tracking=_boolean(
            command, 'supports_reverse_tracking'
        ),
        supports_zero_radius_yaw=_boolean(
            turn, 'supports_zero_radius_yaw'
        ),
        deadband_linear=_number(tracking, 'deadband_linear_mps'),
        deadband_yaw=_number(tracking, 'deadband_yaw_radps'),
        lateral_threshold=_number(
            tracking, 'lateral_threshold_mps'
        ),
        turn_yaw_threshold=_number(
            tracking, 'turn_yaw_threshold_radps'
        ),
        in_place_linear_threshold=_number(
            tracking, 'in_place_linear_threshold_mps'
        ),
        turn_curvature_threshold=_number(
            tracking, 'turn_curvature_threshold'
        ),
        curvature_speed_floor=_number(
            tracking, 'curvature_speed_floor_mps'
        ),
        turn_min_forward=_number(turn, 'stable_min_forward_mps'),
        turn_max_forward=_number(turn, 'stable_max_forward_mps'),
        lateral_max_forward=_number(
            tracking, 'lateral_max_forward_mps'
        ),
        course_yaw_gain=_number(tracking, 'course_yaw_gain'),
        turn_course_enter=_number(tracking, 'turn_course_enter_rad'),
        turn_course_exit=_number(tracking, 'turn_course_exit_rad'),
        turn_yaw_exit=_number(tracking, 'turn_yaw_exit_radps'),
        turn_min_hold_sec=_number(tracking, 'turn_min_hold_sec'),
        cruise_yaw_deadband=_number(
            tracking, 'cruise_yaw_deadband_radps'
        ),
        cruise_yaw_filter_time_constant=_number(
            tracking, 'cruise_yaw_filter_time_constant_sec'
        ),
        reverse_speed_offset=_number(reverse, 'speed_offset_mps'),
        reverse_speed_gain=_number(reverse, 'speed_gain'),
        reverse_yaw_offset=_number(reverse, 'yaw_offset_radps'),
        reverse_yaw_gain=_number(reverse, 'yaw_gain'),
        output_linear_accel=_number(
            tracking, 'output_linear_accel_mps2'
        ),
        output_yaw_accel=_number(
            tracking, 'output_yaw_accel_radps2'
        ),
        reverse_tracking_enter_angle=_number(
            tracking, 'reverse_tracking_enter_angle_rad'
        ),
        reverse_tracking_exit_angle=_number(
            tracking, 'reverse_tracking_exit_angle_rad'
        ),
        reverse_tracking_min_hold_sec=_number(
            tracking, 'reverse_tracking_min_hold_sec'
        ),
        reverse_tracking_entry_alignment=_number(
            tracking, 'reverse_tracking_entry_alignment_rad'
        ),
        reverse_tracking_exit_alignment=_number(
            tracking, 'reverse_tracking_exit_alignment_rad'
        ),
        positive_yaw_lateral_drift=_number(
            drift, 'positive_yaw_lateral_mps'
        ),
        negative_yaw_lateral_drift=_number(
            drift, 'negative_yaw_lateral_mps'
        ),
        opposite_lateral_uncertainty=_number(
            drift, 'opposite_lateral_uncertainty_mps'
        ),
        recovery_forward_speed=_number(recovery, 'forward_speed_mps'),
        recovery_lookahead_sec=_number(recovery, 'lookahead_sec'),
        recovery_min_yaw_rate=_number(recovery, 'minimum_yaw_radps'),
        recovery_clear_confirm_sec=_number(
            recovery, 'clear_confirm_sec'
        ),
        recovery_rearm_clear_sec=_number(
            recovery, 'rearm_clear_sec'
        ),
        recovery_max_active_sec=_number(recovery, 'max_active_sec'),
        recovery_max_displacement_m=_number(
            recovery, 'max_displacement_m'
        ),
        recovery_progress_timeout_sec=_number(
            recovery, 'progress_timeout_sec'
        ),
        recovery_min_progress_m=_number(
            recovery, 'minimum_progress_m'
        ),
        measurement_source=measurement_source,
    )
    _validate_profile(result)  # 启动任何节点前先做内部一致性校验
    return result


def _validate_profile(profile: M20CapabilityProfile) -> None:
    """Reject internally inconsistent values before any node is started."""
    # 【中文注释】拒绝内部不一致的配置值（在任何节点启动之前）。
    # 参数：profile —— 待校验的配置对象；无返回值，不满足约束时抛 ValueError。
    positive = {  # 必须严格大于零的字段集合
        'body_length': profile.body_length,
        'body_width': profile.body_width,
        'body_height': profile.body_height,
        'max_forward': profile.max_forward,
        'max_side': profile.max_side,
        'max_yaw': profile.max_yaw,
        'turn_max_forward': profile.turn_max_forward,
        'recovery_forward_speed': profile.recovery_forward_speed,
        'recovery_lookahead_sec': profile.recovery_lookahead_sec,
        'recovery_rearm_clear_sec': profile.recovery_rearm_clear_sec,
        'recovery_max_active_sec': profile.recovery_max_active_sec,
        'recovery_max_displacement_m': (
            profile.recovery_max_displacement_m
        ),
        'recovery_progress_timeout_sec': (
            profile.recovery_progress_timeout_sec
        ),
        'recovery_min_progress_m': profile.recovery_min_progress_m,
        'reverse_tracking_min_hold_sec': (
            profile.reverse_tracking_min_hold_sec
        ),
        'reverse_tracking_entry_alignment': (
            profile.reverse_tracking_entry_alignment
        ),
        'reverse_tracking_exit_alignment': (
            profile.reverse_tracking_exit_alignment
        ),
    }
    for name, value in positive.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
    if profile.turn_min_forward < 0.0:  # 最低滚动速度可为 0（允许原地转向），但不得为负
        raise ValueError('turn_min_forward must be non-negative')
    if not (  # 稳定滚动速度必须满足 最小 <= 最大 <= 指令上限
        profile.turn_min_forward
        <= profile.turn_max_forward
        <= profile.max_forward
    ):
        raise ValueError(
            'stable rolling speeds must satisfy min <= max <= command max'
        )
    if profile.recovery_forward_speed > profile.max_forward:
        raise ValueError('recovery speed exceeds the command envelope')
    if not (  # 倒车跟踪角度滞回：0 <= 退出角 < 进入角 <= pi
        0.0
        <= profile.reverse_tracking_exit_angle
        < profile.reverse_tracking_enter_angle
        <= math.pi
    ):
        raise ValueError(
            'reverse tracking angles must satisfy 0 <= exit < enter <= pi'
        )
    if not (  # 倒车跟踪对齐角滞回：0 < 进入对齐 < 退出对齐 < pi/2
        0.0
        < profile.reverse_tracking_entry_alignment
        < profile.reverse_tracking_exit_alignment
        < math.pi / 2.0
    ):
        raise ValueError(
            'reverse tracking alignment must satisfy '
            '0 < entry < exit < pi/2'
        )
    if (  # 重新布防清除时间不得短于首次清除确认时间
        profile.recovery_rearm_clear_sec
        < profile.recovery_clear_confirm_sec
    ):
        raise ValueError(
            'recovery rearm clear time must be at least the release time'
        )
    if not profile.supports_zero_radius_yaw and (  # 不支持原地转向时，最低滚动速度必须为正
        profile.turn_min_forward <= 0.0
    ):
        raise ValueError('rolling-only profile requires positive turn speed')
    if profile.recovery_min_progress_m > (  # 最小恢复进展必须在物理上可达（速度 × 超时）
        profile.recovery_forward_speed
        * profile.recovery_progress_timeout_sec
    ):
        raise ValueError('minimum recovery progress is physically unreachable')
