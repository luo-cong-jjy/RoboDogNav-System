# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：navigation_adapter_node.py
# 所属：m20_locomotion_control —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：把 SCAN 的全向指令在"安全预测与门控之前"适配成安全候选指令。
#   - 订阅原始 SCAN Twist（/m20/navigation/cmd_vel_raw），经 RollingNavigationAdapter
#     适配后发布候选 Twist（/m20/navigation/cmd_vel_candidate），供碰撞防护检查；
#   - 可选速度反馈内环：用实测机体速度（Odometry 或 TwistStamped）对候选指令做
#     有界 PI 校正（带门控条件：停止/禁用/执行保持/机动/测量缺失/过期时复位旁路）；
#   - 发布候选模式与速度反馈诊断（JSON），并带指令超时看门狗。
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

"""Adapt SCAN's holonomic command before safety prediction and gating."""
# 【中文注释】模块说明：在安全预测与门控之前适配 SCAN 的全向指令。

import json                         # JSON 序列化（诊断发布）
import math                         # 数学库：isfinite 等

from geometry_msgs.msg import Twist, TwistStamped  # 速度指令/带时间戳速度消息
from nav_msgs.msg import Odometry   # 里程计消息（速度反馈测量源）
import rclpy                        # ROS2 Python 客户端库
from rclpy.node import Node         # ROS2 节点基类
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS 策略
from std_msgs.msg import Bool, String  # 标准消息：布尔、字符串

from .motion_intent import IntentParameters, RollingNavigationAdapter  # 意图参数/滚动适配器
from .velocity_feedback import (    # 速度反馈模块
    MeasuredVelocityFeedback,       #   实测速度反馈控制器
    VelocityFeedbackParameters,     #   反馈参数
)


class NavigationAdapter(Node):
    """Publish the exact autonomous candidate command checked by safety."""
    # 【中文注释】导航适配器节点：发布"被安全模块检查的"精确自主候选指令。

    def __init__(self) -> None:
        # 【中文注释】构造函数：声明参数、构建适配器与反馈控制器、创建收发端。
        super().__init__('m20_navigation_adapter')
        self.declare_parameter(      # 输入话题：原始导航指令
            'input_topic', '/m20/navigation/cmd_vel_raw'
        )
        self.declare_parameter(      # 输出话题：安全候选指令
            'output_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(      # 模式话题：候选模式
            'mode_topic', '/m20/navigation/candidate_mode'
        )
        self.declare_parameter('capability_profile_id', 'fallback_defaults')  # 能力配置 ID
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)  # 最小中心线转弯半径
        self.declare_parameter('turn_swept_radius', 0.0)               # 转弯扫掠半径
        self.declare_parameter('command_timeout_sec', 0.30)  # 指令超时
        self.declare_parameter('max_forward', 0.45)  # 最大前向速度
        self.declare_parameter('max_side', 0.20)     # 最大横向速度
        self.declare_parameter('max_yaw', 0.65)      # 最大偏航角速度
        self.declare_parameter('deadband_linear', 0.01)  # 线速度死区
        self.declare_parameter('deadband_yaw', 0.02)     # 偏航死区
        self.declare_parameter('lateral_threshold', 0.05)  # 横向意图阈值
        self.declare_parameter('turn_yaw_threshold', 0.25) # 转弯偏航阈值
        self.declare_parameter('in_place_linear_threshold', 0.08)  # 原地转向前向阈值
        self.declare_parameter('turn_curvature_threshold', 1.20)   # 转弯曲率阈值
        self.declare_parameter('curvature_speed_floor', 0.05)      # 曲率速度下限
        self.declare_parameter('turn_min_forward', 0.35)  # 转弯最低前向速度
        self.declare_parameter('turn_max_forward', 0.45)  # 转弯最高前向速度
        self.declare_parameter('lateral_max_forward', 0.10)  # 横移最大前向速度
        self.declare_parameter('suppress_side_in_cruise', True)  # 巡航抑制横向
        self.declare_parameter('suppress_side_in_turn', True)    # 转弯抑制横向
        self.declare_parameter('course_yaw_gain', 1.20)   # 航向误差增益
        self.declare_parameter('turn_course_enter', 0.25) # 进入转弯航向误差
        self.declare_parameter('turn_course_exit', 0.08)  # 退出转弯航向误差
        self.declare_parameter('turn_yaw_exit', 0.20)     # 退出转弯偏航阈值
        self.declare_parameter('turn_min_hold_sec', 0.50) # 转弯滞回最短保持
        self.declare_parameter('cruise_yaw_deadband', 0.02)  # 巡航偏航死区
        self.declare_parameter(
            'cruise_yaw_filter_time_constant', 0.12  # 巡航偏航滤波时间常数
        )
        self.declare_parameter('reverse_speed_offset', 0.19)  # 倒车速度补偿偏移
        self.declare_parameter('reverse_speed_gain', 1.32)    # 倒车速度补偿增益
        self.declare_parameter('reverse_yaw_offset', 0.15)    # 倒车偏航补偿偏移
        self.declare_parameter('reverse_yaw_gain', 1.00)      # 倒车偏航补偿增益
        self.declare_parameter('output_linear_accel', 1.0)    # 输出线加速度限幅
        self.declare_parameter('output_yaw_accel', 1.2)       # 输出偏航角加速度限幅
        self.declare_parameter('velocity_feedback_enabled', False)  # 是否启用速度反馈
        self.declare_parameter(           # 速度反馈里程计话题
            'velocity_feedback_odometry_topic',
            '/m20/sim/body_pose',
        )
        self.declare_parameter('velocity_feedback_source', 'odometry')  # 测量源：odometry/twist
        self.declare_parameter(           # 速度反馈 TwistStamped 话题
            'velocity_feedback_twist_topic',
            '/m20/locomotion/measured_twist',
        )
        self.declare_parameter(           # 期望的子坐标系（校验测量帧）
            'velocity_feedback_expected_child_frame',
            'base_link',
        )
        self.declare_parameter(           # 执行保持话题
            'velocity_feedback_execution_hold_topic',
            '/m20/control/execution_hold',
        )
        self.declare_parameter(           # 反馈诊断状态话题
            'velocity_feedback_state_topic',
            '/m20/navigation/velocity_feedback_state',
        )
        self.declare_parameter('velocity_feedback_timeout_sec', 0.15)  # 测量新鲜度超时
        self.declare_parameter('velocity_feedback_cruise_only', True)  # 仅巡航时反馈
        self.declare_parameter(           # 巡航反馈允许的最大 |yaw| 参考
            'velocity_feedback_max_abs_yaw_reference',
            0.08,
        )
        self.declare_parameter('velocity_feedback_linear_kp', 0.30)  # 线速度 Kp
        self.declare_parameter('velocity_feedback_linear_ki', 0.08)  # 线速度 Ki
        self.declare_parameter('velocity_feedback_yaw_kp', 0.20)     # 偏航 Kp
        self.declare_parameter('velocity_feedback_yaw_ki', 0.05)     # 偏航 Ki
        self.declare_parameter(           # 线速度误差死区
            'velocity_feedback_linear_error_deadband',
            0.03,
        )
        self.declare_parameter(           # 偏航误差死区
            'velocity_feedback_yaw_error_deadband',
            0.04,
        )
        self.declare_parameter(           # 线速度积分限幅
            'velocity_feedback_linear_integral_limit',
            0.20,
        )
        self.declare_parameter(           # 偏航积分限幅
            'velocity_feedback_yaw_integral_limit',
            0.25,
        )
        self.declare_parameter(           # 线速度校正量限幅
            'velocity_feedback_linear_correction_limit',
            0.10,
        )
        self.declare_parameter(           # 偏航校正量限幅
            'velocity_feedback_yaw_correction_limit',
            0.12,
        )

        parameters = IntentParameters(  # 从 ROS 参数构建意图参数
            max_forward=float(self.get_parameter('max_forward').value),
            max_side=float(self.get_parameter('max_side').value),
            max_yaw=float(self.get_parameter('max_yaw').value),
            deadband_linear=float(
                self.get_parameter('deadband_linear').value
            ),
            deadband_yaw=float(
                self.get_parameter('deadband_yaw').value
            ),
            lateral_threshold=float(
                self.get_parameter('lateral_threshold').value
            ),
            turn_yaw_threshold=float(
                self.get_parameter('turn_yaw_threshold').value
            ),
            in_place_linear_threshold=float(
                self.get_parameter('in_place_linear_threshold').value
            ),
            turn_curvature_threshold=float(
                self.get_parameter('turn_curvature_threshold').value
            ),
            curvature_speed_floor=float(
                self.get_parameter('curvature_speed_floor').value
            ),
            turn_min_forward=float(
                self.get_parameter('turn_min_forward').value
            ),
            turn_max_forward=float(
                self.get_parameter('turn_max_forward').value
            ),
            lateral_max_forward=float(
                self.get_parameter('lateral_max_forward').value
            ),
            suppress_side_in_cruise=bool(
                self.get_parameter('suppress_side_in_cruise').value
            ),
            suppress_side_in_turn=bool(
                self.get_parameter('suppress_side_in_turn').value
            ),
            course_yaw_gain=float(
                self.get_parameter('course_yaw_gain').value
            ),
            turn_course_enter=float(
                self.get_parameter('turn_course_enter').value
            ),
            turn_course_exit=float(
                self.get_parameter('turn_course_exit').value
            ),
            turn_yaw_exit=float(
                self.get_parameter('turn_yaw_exit').value
            ),
            turn_min_hold_sec=float(
                self.get_parameter('turn_min_hold_sec').value
            ),
            cruise_yaw_deadband=float(
                self.get_parameter('cruise_yaw_deadband').value
            ),
            cruise_yaw_filter_time_constant=float(
                self.get_parameter(
                    'cruise_yaw_filter_time_constant'
                ).value
            ),
            reverse_speed_offset=float(
                self.get_parameter('reverse_speed_offset').value
            ),
            reverse_speed_gain=float(
                self.get_parameter('reverse_speed_gain').value
            ),
            reverse_yaw_offset=float(
                self.get_parameter('reverse_yaw_offset').value
            ),
            reverse_yaw_gain=float(
                self.get_parameter('reverse_yaw_gain').value
            ),
            output_linear_accel=float(
                self.get_parameter('output_linear_accel').value
            ),
            output_yaw_accel=float(
                self.get_parameter('output_yaw_accel').value
            ),
        )
        self._adapter = RollingNavigationAdapter(parameters)  # 滚动导航适配器
        self._velocity_feedback_enabled = bool(  # 速度反馈总开关
            self.get_parameter('velocity_feedback_enabled').value
        )
        self._velocity_feedback_source = str(  # 测量源（odometry/twist）
            self.get_parameter('velocity_feedback_source').value
        ).strip().lower()
        if self._velocity_feedback_source not in {'odometry', 'twist'}:
            raise ValueError(
                'velocity_feedback_source must be odometry or twist'
            )
        self._velocity_feedback = MeasuredVelocityFeedback(  # 速度反馈控制器（PI）
            VelocityFeedbackParameters(
                linear_kp=float(
                    self.get_parameter(
                        'velocity_feedback_linear_kp'
                    ).value
                ),
                linear_ki=float(
                    self.get_parameter(
                        'velocity_feedback_linear_ki'
                    ).value
                ),
                yaw_kp=float(
                    self.get_parameter(
                        'velocity_feedback_yaw_kp'
                    ).value
                ),
                yaw_ki=float(
                    self.get_parameter(
                        'velocity_feedback_yaw_ki'
                    ).value
                ),
                linear_error_deadband=float(
                    self.get_parameter(
                        'velocity_feedback_linear_error_deadband'
                    ).value
                ),
                yaw_error_deadband=float(
                    self.get_parameter(
                        'velocity_feedback_yaw_error_deadband'
                    ).value
                ),
                linear_integral_limit=float(
                    self.get_parameter(
                        'velocity_feedback_linear_integral_limit'
                    ).value
                ),
                yaw_integral_limit=float(
                    self.get_parameter(
                        'velocity_feedback_yaw_integral_limit'
                    ).value
                ),
                linear_correction_limit=float(
                    self.get_parameter(
                        'velocity_feedback_linear_correction_limit'
                    ).value
                ),
                yaw_correction_limit=float(
                    self.get_parameter(
                        'velocity_feedback_yaw_correction_limit'
                    ).value
                ),
                max_forward=parameters.max_forward,  # 输出上限取自意图参数
                max_yaw=parameters.max_yaw,
                reference_linear_deadband=parameters.deadband_linear,
                reference_yaw_deadband=parameters.deadband_yaw,
            )
        )
        self._velocity_feedback_timeout_sec = max(  # 测量新鲜度超时（下限 0.05s）
            0.05,
            float(
                self.get_parameter(
                    'velocity_feedback_timeout_sec'
                ).value
            ),
        )
        self._velocity_feedback_cruise_only = bool(  # 仅巡航（小偏航）时启用反馈
            self.get_parameter('velocity_feedback_cruise_only').value
        )
        self._velocity_feedback_max_abs_yaw_reference = max(  # 反馈允许的最大 |yaw| 参考
            0.0,
            float(
                self.get_parameter(
                    'velocity_feedback_max_abs_yaw_reference'
                ).value
            ),
        )
        self._velocity_feedback_expected_child_frame = str(  # 期望测量帧
            self.get_parameter(
                'velocity_feedback_expected_child_frame'
            ).value
        )
        self._timeout_sec = max(  # 指令超时（下限 0.05s）
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._last_command_ns = 0      # 最近指令时间戳
        self._last_update_ns = 0       # 最近适配更新时间戳
        self._last_mode = ''           # 最近发布的模式
        self._timeout_zero_sent = False  # 超时零指令是否已发送
        self._measured_velocity = None   # 最近实测速度 (vx, vy, yaw)
        self._last_measurement_ns = 0    # 最近测量时间戳
        self._measurement_fault_reason = ''  # 测量故障原因
        self._execution_hold = True      # 执行保持（初始为保持）
        self._feedback_diagnostic = {    # 反馈诊断字典（JSON 发布）
            'enabled': self._velocity_feedback_enabled,
            'active': False,
            'reason': 'STARTING',
        }
        self._command_publisher = self.create_publisher(  # 候选指令发布器
            Twist,
            str(self.get_parameter('output_topic').value),
            20,
        )
        self._mode_publisher = self.create_publisher(  # 候选模式发布器
            String,
            str(self.get_parameter('mode_topic').value),
            10,
        )
        self._feedback_state_publisher = self.create_publisher(  # 反馈诊断发布器
            String,
            str(
                self.get_parameter(
                    'velocity_feedback_state_topic'
                ).value
            ),
            10,
        )
        self.create_subscription(  # 订阅原始导航指令
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        latched = QoSProfile(  # 锁存 QoS：TRANSIENT_LOCAL
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if self._velocity_feedback_source == 'odometry':  # 订阅里程计测量
            self.create_subscription(
                Odometry,
                str(
                    self.get_parameter(
                        'velocity_feedback_odometry_topic'
                    ).value
                ),
                self._odometry_callback,
                50,
            )
        else:  # 订阅 TwistStamped 测量
            self.create_subscription(
                TwistStamped,
                str(
                    self.get_parameter(
                        'velocity_feedback_twist_topic'
                    ).value
                ),
                self._twist_callback,
                50,
            )
        self.create_subscription(  # 订阅执行保持
            Bool,
            str(
                self.get_parameter(
                    'velocity_feedback_execution_hold_topic'
                ).value
            ),
            self._execution_hold_callback,
            latched,
        )
        self._watchdog = self.create_timer(0.05, self._check_timeout)  # 指令超时看门狗
        self._feedback_diagnostic_timer = self.create_timer(  # 诊断发布定时器
            0.10,
            self._publish_feedback_diagnostic,
        )
        self._publish_zero()  # 初始发布零指令
        self.get_logger().info(
            'Navigation adapter ready: raw SCAN Twist -> safety candidate '
            'Twist; measured velocity feedback '
            f'{"enabled" if self._velocity_feedback_enabled else "disabled"}; '
            f'source={self._velocity_feedback_source}; '
            'capability profile='
            f'{self.get_parameter("capability_profile_id").value}; '
            'minimum rolling radius='
            f'{float(self.get_parameter("minimum_centerline_turn_radius").value):.3f}m'
        )

    def _odometry_callback(self, message: Odometry) -> None:
        # 【中文注释】里程计回调：校验帧 id 与数值有效性后保存实测速度。
        if (  # 子坐标系与期望不符 → 标记故障并复位反馈
            self._velocity_feedback_expected_child_frame
            and message.child_frame_id
            != self._velocity_feedback_expected_child_frame
        ):
            self._measured_velocity = None
            self._last_measurement_ns = 0
            self._measurement_fault_reason = (
                'ODOMETRY_FRAME_MISMATCH'
            )
            self._velocity_feedback.reset()
            return
        twist = message.twist.twist
        measurement = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        if not all(math.isfinite(value) for value in measurement):  # 非有限 → 故障
            self._measured_velocity = None
            self._last_measurement_ns = 0
            self._measurement_fault_reason = 'ODOMETRY_NONFINITE'
            self._velocity_feedback.reset()
            return
        self._measured_velocity = measurement  # 保存实测速度
        self._last_measurement_ns = self.get_clock().now().nanoseconds
        self._measurement_fault_reason = ''

    def _twist_callback(self, message: TwistStamped) -> None:
        # 【中文注释】TwistStamped 回调：校验帧 id 与数值有效性后保存实测速度。
        if (  # 帧 id 与期望不符 → 标记故障并复位反馈
            self._velocity_feedback_expected_child_frame
            and message.header.frame_id
            != self._velocity_feedback_expected_child_frame
        ):
            self._measured_velocity = None
            self._last_measurement_ns = 0
            self._measurement_fault_reason = 'TWIST_FRAME_MISMATCH'
            self._velocity_feedback.reset()
            return
        twist = message.twist
        measurement = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        if not all(math.isfinite(value) for value in measurement):  # 非有限 → 故障
            self._measured_velocity = None
            self._last_measurement_ns = 0
            self._measurement_fault_reason = 'TWIST_NONFINITE'
            self._velocity_feedback.reset()
            return
        self._measured_velocity = measurement
        self._last_measurement_ns = self.get_clock().now().nanoseconds
        self._measurement_fault_reason = ''

    def _execution_hold_callback(self, message: Bool) -> None:
        # 【中文注释】执行保持回调：保持时复位速度反馈积分。
        self._execution_hold = bool(message.data)
        if self._execution_hold:
            self._velocity_feedback.reset()

    def _command_callback(self, message: Twist) -> None:
        # 【中文注释】原始指令回调：适配 + 可选速度反馈校正，然后发布候选指令。
        now_ns = self.get_clock().now().nanoseconds
        dt = (  # 时间步长（首帧 0.02s，后续夹到 [0.001, 0.10]s）
            0.02
            if self._last_update_ns == 0
            else max(
                0.001,
                min(0.10, (now_ns - self._last_update_ns) / 1e9),
            )
        )
        self._last_update_ns = now_ns
        self._last_command_ns = now_ns
        self._timeout_zero_sent = False
        intent, reference = self._adapter.update(  # 滚动导航适配
            (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            ),
            dt,
        )
        command = reference
        measurement_age_sec = (  # 测量年龄（无测量时为 inf）
            math.inf
            if self._last_measurement_ns == 0
            else max(
                0.0,
                (now_ns - self._last_measurement_ns) / 1e9,
            )
        )
        reason = 'ACTIVE'   # 反馈门控原因（诊断）
        feedback_result = None
        if intent.value == 'STOPPED':  # 停止 → 反馈复位
            reason = 'STOPPED'
            self._velocity_feedback.reset()
        elif not self._velocity_feedback_enabled:  # 未启用 → 直通
            reason = 'DISABLED'
            self._velocity_feedback.reset()
        elif self._execution_hold:  # 执行保持 → 直通
            reason = 'EXECUTION_HOLD'
            self._velocity_feedback.reset()
        elif (  # 仅巡航模式：转弯/横移机动或 |yaw| 过大时旁路反馈
            self._velocity_feedback_cruise_only
            and (
                intent.value != 'WHEEL_CRUISE'
                or abs(reference[2])
                > self._velocity_feedback_max_abs_yaw_reference
            )
        ):
            reason = 'MANEUVER_GATED'
            self._velocity_feedback.reset()
        elif self._measured_velocity is None:  # 无测量 → 直通
            reason = (
                self._measurement_fault_reason
                or 'ODOMETRY_UNAVAILABLE'
            )
            self._velocity_feedback.reset()
        elif measurement_age_sec > self._velocity_feedback_timeout_sec:  # 测量过期 → 直通
            reason = 'ODOMETRY_STALE'
            self._velocity_feedback.reset()
        else:  # 正常：施加速度反馈校正
            feedback_result = self._velocity_feedback.update(
                reference,
                self._measured_velocity,
                dt,
            )
            command = feedback_result.output

        zero = (0.0, 0.0, 0.0)
        self._feedback_diagnostic = {  # 更新诊断数据
            'enabled': self._velocity_feedback_enabled,
            'active': feedback_result is not None,
            'reason': reason,
            'measurement_age_sec': (
                None
                if not math.isfinite(measurement_age_sec)
                else measurement_age_sec
            ),
            'reference': list(reference),
            'measurement': (
                None
                if self._measured_velocity is None
                else list(self._measured_velocity)
            ),
            'error': list(
                zero if feedback_result is None else feedback_result.error
            ),
            'correction': list(
                zero
                if feedback_result is None
                else feedback_result.correction
            ),
            'integral': list(
                zero
                if feedback_result is None
                else feedback_result.integral
            ),
            'output': list(command),
        }
        output = Twist()  # 发布候选指令
        output.linear.x, output.linear.y, output.angular.z = command
        self._command_publisher.publish(output)
        if intent.value != self._last_mode:  # 模式变化时发布
            self._mode_publisher.publish(String(data=intent.value))
            self._last_mode = intent.value

    def _publish_zero(self) -> None:
        # 【中文注释】发布零指令：复位适配器与反馈，发布 STOPPED 模式。
        self._adapter.reset()
        self._velocity_feedback.reset()
        self._last_update_ns = 0
        self._command_publisher.publish(Twist())
        self._feedback_diagnostic = {
            'enabled': self._velocity_feedback_enabled,
            'active': False,
            'reason': 'COMMAND_ZEROED',
            'measurement_age_sec': None,
            'reference': [0.0, 0.0, 0.0],
            'measurement': (
                None
                if self._measured_velocity is None
                else list(self._measured_velocity)
            ),
            'error': [0.0, 0.0, 0.0],
            'correction': [0.0, 0.0, 0.0],
            'integral': [0.0, 0.0, 0.0],
            'output': [0.0, 0.0, 0.0],
        }
        if self._last_mode != 'STOPPED':
            self._mode_publisher.publish(String(data='STOPPED'))
            self._last_mode = 'STOPPED'

    def _publish_feedback_diagnostic(self) -> None:
        # 【中文注释】发布速度反馈诊断（JSON 字符串）。
        self._feedback_state_publisher.publish(
            String(
                data=json.dumps(
                    self._feedback_diagnostic,
                    separators=(',', ':'),
                )
            )
        )

    def _check_timeout(self) -> None:
        # 【中文注释】看门狗：原始指令超时后发布零指令。
        if self._last_command_ns == 0 or self._timeout_zero_sent:
            return
        age_sec = (
            self.get_clock().now().nanoseconds - self._last_command_ns
        ) / 1e9
        if age_sec < self._timeout_sec:
            return
        self._publish_zero()
        self._timeout_zero_sent = True
        self.get_logger().warn(
            f'raw navigation command timed out after {age_sec:.3f}s; '
            'candidate zeroed'
        )


def main(args=None) -> None:
    """Run the pre-safety autonomous navigation adapter."""
    # 【中文注释】节点入口：初始化、自旋；退出时发布零指令并清理。
    rclpy.init(args=args)
    node = NavigationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # SIGINT can invalidate the rclpy context while CycloneDDS is taking
        # a message. Treat that specific shutdown path as a clean stop.
        # 【中文注释】SIGINT 可能在 CycloneDDS 接收消息时使 rclpy 上下文失效；
        # 将该特定关闭路径视为正常停止。
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:  # 退出前发布零指令
                node._command_publisher.publish(Twist())
            except (Exception, KeyboardInterrupt):
                pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
