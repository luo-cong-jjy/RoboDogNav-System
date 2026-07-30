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

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .motion_intent import IntentParameters, RollingNavigationAdapter


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
        self.declare_parameter('turn_max_forward', 0.12)
        self.declare_parameter('lateral_max_forward', 0.10)
        self.declare_parameter('suppress_side_in_cruise', True)
        self.declare_parameter('suppress_side_in_turn', True)
        self.declare_parameter('course_yaw_gain', 0.80)
        self.declare_parameter('turn_course_enter', 0.25)
        self.declare_parameter('turn_course_exit', 0.08)
        self.declare_parameter('turn_yaw_exit', 0.12)
        self.declare_parameter('turn_min_hold_sec', 0.30)
        self.declare_parameter('cruise_yaw_deadband', 0.04)
        self.declare_parameter(
            'cruise_yaw_filter_time_constant', 0.12
        )
        self.declare_parameter('output_linear_accel', 1.0)
        self.declare_parameter('output_yaw_accel', 1.2)

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
            output_linear_accel=float(
                self.get_parameter('output_linear_accel').value
            ),
            output_yaw_accel=float(
                self.get_parameter('output_yaw_accel').value
            ),
        )
        self._adapter = RollingNavigationAdapter(parameters)
        self._timeout_sec = max(
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._last_command_ns = 0
        self._last_update_ns = 0
        self._last_mode = ''
        self._timeout_zero_sent = False
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
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        self._watchdog = self.create_timer(0.05, self._check_timeout)
        self._publish_zero()
        self.get_logger().info(
            'Navigation adapter ready: raw SCAN Twist -> safety candidate '
            'Twist'
        )

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
        intent, command = self._adapter.update(
            (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            ),
            dt,
        )
        output = Twist()
        output.linear.x, output.linear.y, output.angular.z = command
        self._command_publisher.publish(output)
        if intent.value != self._last_mode:
            self._mode_publisher.publish(String(data=intent.value))
            self._last_mode = intent.value

    def _publish_zero(self) -> None:
        self._adapter.reset()
        self._last_update_ns = 0
        self._command_publisher.publish(Twist())
        if self._last_mode != 'STOPPED':
            self._mode_publisher.publish(String(data='STOPPED'))
            self._last_mode = 'STOPPED'

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
        rclpy.try_shutdown()
