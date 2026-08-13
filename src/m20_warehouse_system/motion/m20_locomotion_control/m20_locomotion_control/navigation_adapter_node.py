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

import json
import math

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .motion_intent import IntentParameters, RollingNavigationAdapter
from .velocity_feedback import (
    MeasuredVelocityFeedback,
    VelocityFeedbackParameters,
)


class NavigationAdapter(Node):
    """Publish the exact autonomous candidate command checked by safety."""

    def __init__(self) -> None:
        super().__init__('m20_navigation_adapter')
        self.declare_parameter(
            'input_topic', '/m20/navigation/cmd_vel_raw'
        )
        self.declare_parameter(
            'output_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(
            'mode_topic', '/m20/navigation/candidate_mode'
        )
        self.declare_parameter('capability_profile_id', 'fallback_defaults')
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)
        self.declare_parameter('turn_swept_radius', 0.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('max_forward', 0.45)
        self.declare_parameter('max_side', 0.20)
        self.declare_parameter('max_yaw', 0.65)
        self.declare_parameter('deadband_linear', 0.01)
        self.declare_parameter('deadband_yaw', 0.02)
        self.declare_parameter('lateral_threshold', 0.05)
        self.declare_parameter('turn_yaw_threshold', 0.25)
        self.declare_parameter('in_place_linear_threshold', 0.08)
        self.declare_parameter('turn_curvature_threshold', 1.20)
        self.declare_parameter('curvature_speed_floor', 0.05)
        self.declare_parameter('turn_min_forward', 0.35)
        self.declare_parameter('turn_max_forward', 0.45)
        self.declare_parameter('lateral_max_forward', 0.10)
        self.declare_parameter('suppress_side_in_cruise', True)
        self.declare_parameter('suppress_side_in_turn', True)
        self.declare_parameter('course_yaw_gain', 1.20)
        self.declare_parameter('turn_course_enter', 0.25)
        self.declare_parameter('turn_course_exit', 0.08)
        self.declare_parameter('turn_yaw_exit', 0.20)
        self.declare_parameter('turn_min_hold_sec', 0.50)
        self.declare_parameter('cruise_yaw_deadband', 0.02)
        self.declare_parameter(
            'cruise_yaw_filter_time_constant', 0.12
        )
        self.declare_parameter('reverse_speed_offset', 0.19)
        self.declare_parameter('reverse_speed_gain', 1.32)
        self.declare_parameter('reverse_yaw_offset', 0.15)
        self.declare_parameter('reverse_yaw_gain', 1.00)
        self.declare_parameter('output_linear_accel', 1.0)
        self.declare_parameter('output_yaw_accel', 1.2)
        self.declare_parameter('velocity_feedback_enabled', False)
        self.declare_parameter(
            'velocity_feedback_odometry_topic',
            '/m20/sim/body_pose',
        )
        self.declare_parameter('velocity_feedback_source', 'odometry')
        self.declare_parameter(
            'velocity_feedback_twist_topic',
            '/m20/locomotion/measured_twist',
        )
        self.declare_parameter(
            'velocity_feedback_expected_child_frame',
            'base_link',
        )
        self.declare_parameter(
            'velocity_feedback_execution_hold_topic',
            '/m20/control/execution_hold',
        )
        self.declare_parameter(
            'velocity_feedback_state_topic',
            '/m20/navigation/velocity_feedback_state',
        )
        self.declare_parameter('velocity_feedback_timeout_sec', 0.15)
        self.declare_parameter('velocity_feedback_cruise_only', True)
        self.declare_parameter(
            'velocity_feedback_max_abs_yaw_reference',
            0.08,
        )
        self.declare_parameter('velocity_feedback_linear_kp', 0.30)
        self.declare_parameter('velocity_feedback_linear_ki', 0.08)
        self.declare_parameter('velocity_feedback_yaw_kp', 0.20)
        self.declare_parameter('velocity_feedback_yaw_ki', 0.05)
        self.declare_parameter(
            'velocity_feedback_linear_error_deadband',
            0.03,
        )
        self.declare_parameter(
            'velocity_feedback_yaw_error_deadband',
            0.04,
        )
        self.declare_parameter(
            'velocity_feedback_linear_integral_limit',
            0.20,
        )
        self.declare_parameter(
            'velocity_feedback_yaw_integral_limit',
            0.25,
        )
        self.declare_parameter(
            'velocity_feedback_linear_correction_limit',
            0.10,
        )
        self.declare_parameter(
            'velocity_feedback_yaw_correction_limit',
            0.12,
        )

        parameters = IntentParameters(
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
        self._adapter = RollingNavigationAdapter(parameters)
        self._velocity_feedback_enabled = bool(
            self.get_parameter('velocity_feedback_enabled').value
        )
        self._velocity_feedback_source = str(
            self.get_parameter('velocity_feedback_source').value
        ).strip().lower()
        if self._velocity_feedback_source not in {'odometry', 'twist'}:
            raise ValueError(
                'velocity_feedback_source must be odometry or twist'
            )
        self._velocity_feedback = MeasuredVelocityFeedback(
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
                max_forward=parameters.max_forward,
                max_yaw=parameters.max_yaw,
                reference_linear_deadband=parameters.deadband_linear,
                reference_yaw_deadband=parameters.deadband_yaw,
            )
        )
        self._velocity_feedback_timeout_sec = max(
            0.05,
            float(
                self.get_parameter(
                    'velocity_feedback_timeout_sec'
                ).value
            ),
        )
        self._velocity_feedback_cruise_only = bool(
            self.get_parameter('velocity_feedback_cruise_only').value
        )
        self._velocity_feedback_max_abs_yaw_reference = max(
            0.0,
            float(
                self.get_parameter(
                    'velocity_feedback_max_abs_yaw_reference'
                ).value
            ),
        )
        self._velocity_feedback_expected_child_frame = str(
            self.get_parameter(
                'velocity_feedback_expected_child_frame'
            ).value
        )
        self._timeout_sec = max(
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._last_command_ns = 0
        self._last_update_ns = 0
        self._last_mode = ''
        self._timeout_zero_sent = False
        self._measured_velocity = None
        self._last_measurement_ns = 0
        self._measurement_fault_reason = ''
        self._execution_hold = True
        self._feedback_diagnostic = {
            'enabled': self._velocity_feedback_enabled,
            'active': False,
            'reason': 'STARTING',
        }
        self._command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('output_topic').value),
            20,
        )
        self._mode_publisher = self.create_publisher(
            String,
            str(self.get_parameter('mode_topic').value),
            10,
        )
        self._feedback_state_publisher = self.create_publisher(
            String,
            str(
                self.get_parameter(
                    'velocity_feedback_state_topic'
                ).value
            ),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if self._velocity_feedback_source == 'odometry':
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
        else:
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
        self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    'velocity_feedback_execution_hold_topic'
                ).value
            ),
            self._execution_hold_callback,
            latched,
        )
        self._watchdog = self.create_timer(0.05, self._check_timeout)
        self._feedback_diagnostic_timer = self.create_timer(
            0.10,
            self._publish_feedback_diagnostic,
        )
        self._publish_zero()
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
        if (
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
        if not all(math.isfinite(value) for value in measurement):
            self._measured_velocity = None
            self._last_measurement_ns = 0
            self._measurement_fault_reason = 'ODOMETRY_NONFINITE'
            self._velocity_feedback.reset()
            return
        self._measured_velocity = measurement
        self._last_measurement_ns = self.get_clock().now().nanoseconds
        self._measurement_fault_reason = ''

    def _twist_callback(self, message: TwistStamped) -> None:
        if (
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
        if not all(math.isfinite(value) for value in measurement):
            self._measured_velocity = None
            self._last_measurement_ns = 0
            self._measurement_fault_reason = 'TWIST_NONFINITE'
            self._velocity_feedback.reset()
            return
        self._measured_velocity = measurement
        self._last_measurement_ns = self.get_clock().now().nanoseconds
        self._measurement_fault_reason = ''

    def _execution_hold_callback(self, message: Bool) -> None:
        self._execution_hold = bool(message.data)
        if self._execution_hold:
            self._velocity_feedback.reset()

    def _command_callback(self, message: Twist) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt = (
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
        intent, reference = self._adapter.update(
            (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            ),
            dt,
        )
        command = reference
        measurement_age_sec = (
            math.inf
            if self._last_measurement_ns == 0
            else max(
                0.0,
                (now_ns - self._last_measurement_ns) / 1e9,
            )
        )
        reason = 'ACTIVE'
        feedback_result = None
        if intent.value == 'STOPPED':
            reason = 'STOPPED'
            self._velocity_feedback.reset()
        elif not self._velocity_feedback_enabled:
            reason = 'DISABLED'
            self._velocity_feedback.reset()
        elif self._execution_hold:
            reason = 'EXECUTION_HOLD'
            self._velocity_feedback.reset()
        elif (
            self._velocity_feedback_cruise_only
            and (
                intent.value != 'WHEEL_CRUISE'
                or abs(reference[2])
                > self._velocity_feedback_max_abs_yaw_reference
            )
        ):
            reason = 'MANEUVER_GATED'
            self._velocity_feedback.reset()
        elif self._measured_velocity is None:
            reason = (
                self._measurement_fault_reason
                or 'ODOMETRY_UNAVAILABLE'
            )
            self._velocity_feedback.reset()
        elif measurement_age_sec > self._velocity_feedback_timeout_sec:
            reason = 'ODOMETRY_STALE'
            self._velocity_feedback.reset()
        else:
            feedback_result = self._velocity_feedback.update(
                reference,
                self._measured_velocity,
                dt,
            )
            command = feedback_result.output

        zero = (0.0, 0.0, 0.0)
        self._feedback_diagnostic = {
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
        output = Twist()
        output.linear.x, output.linear.y, output.angular.z = command
        self._command_publisher.publish(output)
        if intent.value != self._last_mode:
            self._mode_publisher.publish(String(data=intent.value))
            self._last_mode = intent.value

    def _publish_zero(self) -> None:
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
        self._feedback_state_publisher.publish(
            String(
                data=json.dumps(
                    self._feedback_diagnostic,
                    separators=(',', ':'),
                )
            )
        )

    def _check_timeout(self) -> None:
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
    rclpy.init(args=args)
    node = NavigationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # SIGINT can invalidate the rclpy context while CycloneDDS is taking
        # a message. Treat that specific shutdown path as a clean stop.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:
                node._command_publisher.publish(Twist())
            except (Exception, KeyboardInterrupt):
                pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
