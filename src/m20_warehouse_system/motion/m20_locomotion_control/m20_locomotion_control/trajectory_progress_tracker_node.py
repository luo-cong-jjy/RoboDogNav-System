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

"""Execute SCAN geometry using measured M20 path progress."""

from dataclasses import fields
import json
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scan_planner_msgs.msg import Bspline
from std_msgs.msg import Bool, String

from .motion_intent import IntentParameters, RollingNavigationAdapter
from .trajectory_progress import (
    Bspline2D,
    ProgressFollowerParameters,
    SampledPath,
    SpatialProgressFollower,
)


def _odometry_yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    sine = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cosine = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sine, cosine)


class TrajectoryProgressTracker(Node):
    """Publish an M20-safe candidate from SCAN's unchanged spatial curve."""

    def __init__(self) -> None:
        super().__init__('m20_trajectory_progress_tracker')
        self.declare_parameter('trajectory_topic', '/planning/bspline')
        self.declare_parameter('odometry_topic', '/m20/sim/body_pose')
        self.declare_parameter(
            'execution_hold_topic', '/m20/control/execution_hold'
        )
        self.declare_parameter(
            'output_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(
            'mode_topic', '/m20/navigation/candidate_mode'
        )
        self.declare_parameter(
            'state_topic', '/m20/navigation/progress_tracker_state'
        )
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('odometry_timeout_sec', 0.25)
        self.declare_parameter('spline_sample_period_sec', 0.02)
        self.declare_parameter('lookahead_m', 0.60)
        self.declare_parameter('yaw_gain', 1.50)
        self.declare_parameter('finish_distance_m', 0.15)
        self.declare_parameter('terminal_speed_threshold_mps', 0.10)
        self.declare_parameter('terminal_slowdown_distance_m', 0.80)
        self.declare_parameter('terminal_min_speed_mps', 0.10)
        self.declare_parameter('reverse_tracking_enabled', True)
        self.declare_parameter('reverse_tracking_enter_angle', 2.10)
        self.declare_parameter('reverse_tracking_exit_angle', 1.75)
        self.declare_parameter('capability_profile_id', 'fallback_defaults')
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)
        self.declare_parameter('turn_swept_radius', 0.0)

        intent_defaults = IntentParameters()
        for item in fields(IntentParameters):
            self.declare_parameter(
                item.name,
                getattr(intent_defaults, item.name),
            )
        intent_parameters = IntentParameters(
            **{
                item.name: self.get_parameter(item.name).value
                for item in fields(IntentParameters)
            }
        )
        self._adapter = RollingNavigationAdapter(intent_parameters)
        self._follower = SpatialProgressFollower(
            ProgressFollowerParameters(
                lookahead_m=float(
                    self.get_parameter('lookahead_m').value
                ),
                max_speed_mps=intent_parameters.max_forward,
                yaw_gain=float(self.get_parameter('yaw_gain').value),
                max_yaw_radps=intent_parameters.max_yaw,
                finish_distance_m=float(
                    self.get_parameter('finish_distance_m').value
                ),
                terminal_speed_threshold_mps=float(
                    self.get_parameter(
                        'terminal_speed_threshold_mps'
                    ).value
                ),
                terminal_slowdown_distance_m=float(
                    self.get_parameter(
                        'terminal_slowdown_distance_m'
                    ).value
                ),
                terminal_min_speed_mps=float(
                    self.get_parameter('terminal_min_speed_mps').value
                ),
                reverse_tracking_enabled=bool(
                    self.get_parameter('reverse_tracking_enabled').value
                ),
                reverse_enter_angle_rad=float(
                    self.get_parameter(
                        'reverse_tracking_enter_angle'
                    ).value
                ),
                reverse_exit_angle_rad=float(
                    self.get_parameter(
                        'reverse_tracking_exit_angle'
                    ).value
                ),
            )
        )
        self._sample_period_sec = max(
            0.005,
            float(
                self.get_parameter('spline_sample_period_sec').value
            ),
        )
        self._odometry_timeout_sec = max(
            0.05,
            float(self.get_parameter('odometry_timeout_sec').value),
        )
        self._path = None
        self._trajectory_id = -1
        self._pose = None
        self._last_odometry_ns = 0
        self._last_update_ns = 0
        self._execution_hold = True
        self._last_mode = ''
        self._last_state = {'reason': 'STARTING'}

        latched = QoSProfile(
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
            latched,
        )
        self._state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('state_topic').value),
            10,
        )
        self.create_subscription(
            Bspline,
            str(self.get_parameter('trajectory_topic').value),
            self._trajectory_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('execution_hold_topic').value),
            self._execution_hold_callback,
            latched,
        )
        control_rate = max(
            10.0,
            float(self.get_parameter('control_rate_hz').value),
        )
        self._control_timer = self.create_timer(
            1.0 / control_rate,
            self._control_callback,
        )
        self._diagnostic_timer = self.create_timer(
            0.10,
            self._publish_state,
        )
        self._publish_zero('STARTING')
        self.get_logger().info(
            'M20 trajectory progress tracker ready: unchanged SCAN B-spline '
            '-> measured spatial progress -> safety candidate; lookahead='
            f'{self._follower.parameters.lookahead_m:.3f}m; profile='
            f'{self.get_parameter("capability_profile_id").value}'
        )

    def _trajectory_callback(self, message: Bspline) -> None:
        try:
            spline = Bspline2D(
                [(point.x, point.y) for point in message.pos_pts],
                int(message.order),
                list(message.knots),
            )
            path = SampledPath.from_spline(
                spline,
                self._sample_period_sec,
            )
        except (TypeError, ValueError) as error:
            self._path = None
            self._trajectory_id = int(message.traj_id)
            self._adapter.reset()
            self._last_state = {
                'reason': 'INVALID_TRAJECTORY',
                'trajectory_id': self._trajectory_id,
                'detail': str(error),
            }
            self._command_publisher.publish(Twist())
            self.get_logger().warn(
                f'Ignoring invalid/degenerate trajectory: {error}'
            )
            return
        self._path = path
        self._trajectory_id = int(message.traj_id)
        self._follower.start_trajectory()
        self._last_state = {
            'reason': 'TRAJECTORY_READY',
            'trajectory_id': self._trajectory_id,
            'path_length_m': path.length_m,
            'terminal_speed_mps': path.end_speed_mps,
        }

    def _odometry_callback(self, message: Odometry) -> None:
        pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _odometry_yaw(message),
        )
        if not all(math.isfinite(value) for value in pose):
            self._pose = None
            self._last_odometry_ns = 0
            return
        self._pose = pose
        self._last_odometry_ns = self.get_clock().now().nanoseconds

    def _execution_hold_callback(self, message: Bool) -> None:
        hold = bool(message.data)
        if hold and not self._execution_hold:
            self._adapter.reset()
        self._execution_hold = hold

    def _publish_zero(self, reason: str) -> None:
        self._adapter.reset()
        self._command_publisher.publish(Twist())
        if self._last_mode != 'STOPPED':
            self._mode_publisher.publish(String(data='STOPPED'))
            self._last_mode = 'STOPPED'
        self._last_state = {
            'reason': reason,
            'trajectory_id': self._trajectory_id,
        }

    def _control_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt = (
            0.02
            if self._last_update_ns == 0
            else max(
                0.001,
                min(0.10, (now_ns - self._last_update_ns) / 1.0e9),
            )
        )
        self._last_update_ns = now_ns
        if self._execution_hold:
            self._publish_zero('EXECUTION_HOLD')
            return
        if self._path is None:
            self._publish_zero('TRAJECTORY_UNAVAILABLE')
            return
        if self._pose is None or self._last_odometry_ns == 0:
            self._publish_zero('ODOMETRY_UNAVAILABLE')
            return
        odometry_age = (now_ns - self._last_odometry_ns) / 1.0e9
        if odometry_age > self._odometry_timeout_sec:
            self._publish_zero('ODOMETRY_STALE')
            return

        result = self._follower.update(self._path, *self._pose)
        intent, command = self._adapter.update(result.command, dt)
        output = Twist()
        output.linear.x = command[0]
        output.linear.y = command[1]
        output.angular.z = command[2]
        self._command_publisher.publish(output)
        if intent.value != self._last_mode:
            self._mode_publisher.publish(String(data=intent.value))
            self._last_mode = intent.value
        self._last_state = {
            'reason': 'FINISHED' if result.finished else 'TRACKING',
            'trajectory_id': self._trajectory_id,
            'progress_m': result.progress_m,
            'remaining_m': result.remaining_m,
            'cross_track_m': result.cross_track_m,
            'lookahead_target': list(result.target),
            'heading_error_rad': result.heading_error_rad,
            'reverse_tracking': result.reverse_tracking,
            'raw_command': list(result.command),
            'candidate_command': list(command),
            'odometry_age_sec': odometry_age,
        }

    def _publish_state(self) -> None:
        self._state_publisher.publish(
            String(
                data=json.dumps(
                    self._last_state,
                    separators=(',', ':'),
                )
            )
        )


def main(args=None) -> None:
    """Run the optional M20 spatial-progress trajectory executor."""
    rclpy.init(args=args)
    node = TrajectoryProgressTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
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
