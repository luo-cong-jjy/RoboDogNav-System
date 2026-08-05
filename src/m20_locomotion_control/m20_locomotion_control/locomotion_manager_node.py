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

"""ROS node adapting safe navigation commands to the M20 RL controller."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .motion_intent import (
    IntentParameters,
    MotionIntent,
    RollingNavigationAdapter,
    constrain_for_intent,
)


class LocomotionManager(Node):
    """Constrain safe Twist commands and expose their high-level motion intent."""

    def __init__(self) -> None:
        super().__init__('m20_locomotion_manager')
        self.declare_parameter(
            'input_topic', '/m20/control/cmd_vel_safe'
        )
        self.declare_parameter(
            'output_topic', '/m20/locomotion/cmd_vel_sdk'
        )
        self.declare_parameter(
            'mode_topic', '/m20/locomotion/mode'
        )
        self.declare_parameter(
            'backend_ready_topic', '/m20/sim/backend_ready'
        )
        self.declare_parameter(
            'backend_fault_topic', '/m20/sim/backend_fault'
        )
        self.declare_parameter(
            'safety_state_topic', '/m20/control/safety_state'
        )
        self.declare_parameter('capability_profile_id', 'fallback_defaults')
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)
        self.declare_parameter('turn_swept_radius', 0.0)
        self.declare_parameter('require_backend_ready', False)
        self.declare_parameter('rolling_navigation_enabled', True)
        self.declare_parameter('allow_manual_lateral', True)
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
            'cruise_yaw_filter_time_constant',
            0.12,
        )
        self.declare_parameter('reverse_speed_offset', 0.19)
        self.declare_parameter('reverse_speed_gain', 1.32)
        self.declare_parameter('reverse_yaw_offset', 0.15)
        self.declare_parameter('reverse_yaw_gain', 1.00)
        self.declare_parameter('output_linear_accel', 1.0)
        self.declare_parameter('output_yaw_accel', 1.2)

        # Do not shadow rclpy.Node._parameters, which owns declared ROS
        # parameters internally.
        self._intent_parameters = IntentParameters(
            max_forward=float(self.get_parameter('max_forward').value),
            max_side=float(self.get_parameter('max_side').value),
            max_yaw=float(self.get_parameter('max_yaw').value),
            deadband_linear=float(
                self.get_parameter('deadband_linear').value
            ),
            deadband_yaw=float(self.get_parameter('deadband_yaw').value),
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
        self._rolling_adapter = RollingNavigationAdapter(
            self._intent_parameters
        )
        self._rolling_navigation_enabled = bool(
            self.get_parameter('rolling_navigation_enabled').value
        )
        self._allow_manual_lateral = bool(
            self.get_parameter('allow_manual_lateral').value
        )
        self._timeout_sec = max(
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._last_command_ns = 0
        self._last_adaptation_ns = 0
        self._last_intent = None
        self._timeout_zero_sent = False
        self._require_backend_ready = bool(
            self.get_parameter('require_backend_ready').value
        )
        self._backend_ready = not self._require_backend_ready
        self._backend_fault = ''
        self._safety_state = 'NAVIGATION'

        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('output_topic').value),
            20,
        )
        self._mode_publisher = self.create_publisher(
            String,
            str(self.get_parameter('mode_topic').value),
            mode_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('backend_ready_topic').value),
            self._backend_ready_callback,
            mode_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('backend_fault_topic').value),
            self._backend_fault_callback,
            mode_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('safety_state_topic').value),
            self._safety_state_callback,
            mode_qos,
        )
        self._watchdog = self.create_timer(0.05, self._check_timeout)
        initial_intent = (
            MotionIntent.BACKEND_HOLD
            if self._require_backend_ready
            else MotionIntent.STOPPED
        )
        self._publish(initial_intent, (0.0, 0.0, 0.0))
        self.get_logger().info(
            'Locomotion manager ready: safe Twist -> SDK Twist; '
            'autonomous rolling adapter='
            f'{self._rolling_navigation_enabled}; motion mode is an intent '
            'label, not a discrete ONNX gait input; capability profile='
            f'{self.get_parameter("capability_profile_id").value}'
        )

    def _command_callback(self, message: Twist) -> None:
        if not self._backend_ready or self._backend_fault:
            self._publish_backend_hold()
            return
        command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        now_ns = self.get_clock().now().nanoseconds
        dt = (
            0.02
            if self._last_adaptation_ns == 0
            else max(
                0.001,
                min(0.10, (now_ns - self._last_adaptation_ns) / 1e9),
            )
        )
        self._last_adaptation_ns = now_ns
        if (
            self._rolling_navigation_enabled
            and self._safety_state == 'NAVIGATION'
        ):
            intent, constrained = self._rolling_adapter.update(command, dt)
        elif (
            self._safety_state == 'MANUAL'
            and self._allow_manual_lateral
        ) or not self._rolling_navigation_enabled:
            self._rolling_adapter.reset()
            intent, constrained = constrain_for_intent(
                command,
                self._intent_parameters,
            )
        else:
            self._rolling_adapter.reset()
            intent, constrained = MotionIntent.STOPPED, (0.0, 0.0, 0.0)
        self._last_command_ns = now_ns
        self._timeout_zero_sent = False
        self._publish(intent, constrained)

    def _backend_ready_callback(self, message: Bool) -> None:
        was_ready = self._backend_ready
        self._backend_ready = bool(message.data)
        if not self._backend_ready:
            self._publish_backend_hold()
        elif not was_ready and not self._backend_fault:
            self._publish(MotionIntent.STOPPED, (0.0, 0.0, 0.0))
            self.get_logger().info('MuJoCo backend ready; commands enabled')

    def _backend_fault_callback(self, message: String) -> None:
        previous = self._backend_fault
        self._backend_fault = str(message.data)
        if self._backend_fault:
            self._publish_backend_hold()
            if self._backend_fault != previous:
                self.get_logger().error(
                    f'backend fault hold: {self._backend_fault}'
                )
        elif previous and self._backend_ready:
            self._publish(MotionIntent.STOPPED, (0.0, 0.0, 0.0))
            self.get_logger().info('backend fault cleared; commands enabled')

    def _safety_state_callback(self, message: String) -> None:
        self._safety_state = str(message.data)
        if self._safety_state not in {'NAVIGATION', 'MANUAL'}:
            self._rolling_adapter.reset()
            self._last_adaptation_ns = 0

    def _publish_backend_hold(self) -> None:
        self._rolling_adapter.reset()
        self._last_adaptation_ns = 0
        intent = (
            MotionIntent.FAULT_HOLD
            if self._backend_fault
            else MotionIntent.BACKEND_HOLD
        )
        self._publish(intent, (0.0, 0.0, 0.0))

    def _check_timeout(self) -> None:
        if not self._backend_ready or self._backend_fault:
            return
        if self._last_command_ns == 0 or self._timeout_zero_sent:
            return
        age_sec = (
            self.get_clock().now().nanoseconds - self._last_command_ns
        ) / 1e9
        if age_sec < self._timeout_sec:
            return
        self._rolling_adapter.reset()
        self._last_adaptation_ns = 0
        self._publish(MotionIntent.STOPPED, (0.0, 0.0, 0.0))
        self._timeout_zero_sent = True
        self.get_logger().warn(
            f'safe command timed out after {age_sec:.3f}s; SDK command zeroed'
        )

    def _publish(self, intent: MotionIntent, command) -> None:
        message = Twist()
        message.linear.x = command[0]
        message.linear.y = command[1]
        message.angular.z = command[2]
        self._command_publisher.publish(message)
        if intent != self._last_intent:
            self._mode_publisher.publish(String(data=intent.value))
            self.get_logger().info(f'motion intent -> {intent.value}')
            self._last_intent = intent


def main(args=None) -> None:
    """Run the locomotion manager."""
    rclpy.init(args=args)
    node = LocomotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:
                zero = Twist()
                node._command_publisher.publish(zero)
            except (Exception, KeyboardInterrupt):
                pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
