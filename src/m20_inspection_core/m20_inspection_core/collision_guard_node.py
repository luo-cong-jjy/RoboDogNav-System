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

"""Independent static-map collision lookahead guard."""

import math
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .collision_policy import (
    GridGeometry,
    first_blocking_footprint,
    inflate_blocked_grid,
    predict_poses,
    safe_rotation_recovery,
)


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class CollisionGuard(Node):
    """Assert a fail-closed stop for occupied current or predicted cells."""

    def __init__(self) -> None:
        super().__init__('m20_collision_guard')
        self.declare_parameter(
            'occupancy_topic', '/m20/map/active_occupancy'
        )
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        self.declare_parameter(
            'command_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(
            'stop_topic', '/m20/control/collision_stop'
        )
        self.declare_parameter(
            'state_topic', '/m20/control/collision_guard_state'
        )
        self.declare_parameter(
            'diagnostic_topic',
            '/m20/control/collision_guard_diagnostic',
        )
        self.declare_parameter(
            'recovery_available_topic',
            '/m20/control/collision_recovery_available',
        )
        self.declare_parameter(
            'recovery_command_topic',
            '/m20/control/collision_recovery_cmd',
        )
        self.declare_parameter('map_ready_topic', '/m20/map/ready')
        self.declare_parameter('occupied_threshold', 50)
        # Safe native-SCAN defaults validated with the official M20 in
        # MuJoCo. The optional grid-route launch overrides these explicitly.
        self.declare_parameter('footprint_radius', 0.25)
        self.declare_parameter('footprint_offset', 0.18)
        self.declare_parameter('safety_margin', 0.05)
        self.declare_parameter('map_edge_tolerance', 0.35)
        self.declare_parameter('lookahead_sec', 0.70)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_linear_x', 0.45)
        self.declare_parameter('max_linear_y', 0.20)
        self.declare_parameter('max_angular_z', 0.65)

        self._geometry: Optional[GridGeometry] = None
        self._blocked = None
        self._odom: Optional[Odometry] = None
        self._command = (0.0, 0.0, 0.0)
        self._command_time: Optional[float] = None
        self._map_ready = False
        self._last_state = ''
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stop_publisher = self.create_publisher(
            Bool, str(self.get_parameter('stop_topic').value), latched_qos
        )
        self._state_publisher = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), latched_qos
        )
        self._diagnostic_publisher = self.create_publisher(
            String,
            str(self.get_parameter('diagnostic_topic').value),
            latched_qos,
        )
        self._recovery_available_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('recovery_available_topic').value),
            latched_qos,
        )
        self._recovery_command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('recovery_command_topic').value),
            20,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('occupancy_topic').value),
            self._map_callback,
            latched_qos,
        )
        self._map_ready_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter('map_ready_topic').value),
            self._map_ready_callback,
            latched_qos,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter('command_topic').value),
            self._command_callback,
            20,
        )
        rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self._timer = self.create_timer(1.0 / rate, self._evaluate)
        self._publish(True, 'NOT_READY')
        self._publish_recovery(None)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _map_callback(self, message: OccupancyGrid) -> None:
        geometry = GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            edge_tolerance=float(
                self.get_parameter('map_edge_tolerance').value
            ),
        )
        radius = (
            float(self.get_parameter('footprint_radius').value)
            + float(self.get_parameter('safety_margin').value)
        )
        self._blocked = inflate_blocked_grid(
            message.data,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            radius,
        )
        self._geometry = geometry
        self.get_logger().info(
            'collision guard map ready; '
            f'double-circle radius={radius:.2f}m, '
            f'offset='
            f'{float(self.get_parameter("footprint_offset").value):.2f}m, '
            f'edge tolerance={geometry.edge_tolerance:.2f}m'
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._odom = message

    def _map_ready_callback(self, message: Bool) -> None:
        self._map_ready = message.data
        if not self._map_ready:
            self._publish(True, 'MAP_NOT_READY')

    def _command_callback(self, message: Twist) -> None:
        # This is the adapted pre-safety candidate, so prediction and backend
        # execution share one body-command model without reading safe output.
        max_x = abs(float(self.get_parameter('max_linear_x').value))
        max_y = abs(float(self.get_parameter('max_linear_y').value))
        max_yaw = abs(float(self.get_parameter('max_angular_z').value))
        self._command = (
            max(-max_x, min(max_x, float(message.linear.x))),
            max(-max_y, min(max_y, float(message.linear.y))),
            max(-max_yaw, min(max_yaw, float(message.angular.z))),
        )
        self._command_time = self._now()

    def _publish(
        self,
        stop: bool,
        state: str,
        diagnostic: str = '',
    ) -> None:
        self._stop_publisher.publish(Bool(data=stop))
        if state != self._last_state:
            self._state_publisher.publish(String(data=state))
            self._diagnostic_publisher.publish(
                String(data=diagnostic or state)
            )
            self.get_logger().info(f'collision guard state: {state}')
            if diagnostic:
                self.get_logger().info(
                    f'collision guard diagnostic: {diagnostic}'
                )
            self._last_state = state

    def _publish_recovery(
        self,
        command: Optional[tuple[float, float, float]],
    ) -> None:
        """Publish only collision-checked, translation-free recovery motion."""
        message = Twist()
        available = command is not None
        if command is not None:
            message.linear.x = 0.0
            message.linear.y = 0.0
            message.angular.z = float(command[2])
        self._recovery_command_publisher.publish(message)
        self._recovery_available_publisher.publish(
            Bool(data=available)
        )

    def _evaluate(self) -> None:
        if (
            not self._map_ready
            or self._geometry is None
            or self._blocked is None
            or self._odom is None
        ):
            self._publish_recovery(None)
            self._publish(True, 'NOT_READY')
            return
        command = self._command
        if (
            self._command_time is None
            or self._now() - self._command_time
            > float(self.get_parameter('command_timeout_sec').value)
        ):
            command = (0.0, 0.0, 0.0)
        position = self._odom.pose.pose.position
        sample_period = float(
            self.get_parameter('sample_period_sec').value
        )
        poses = predict_poses(
            float(position.x),
            float(position.y),
            _yaw_from_odometry(self._odom),
            command,
            float(self.get_parameter('lookahead_sec').value),
            sample_period,
        )
        blocked_sample = first_blocking_footprint(
            self._blocked,
            self._geometry,
            poses,
            float(self.get_parameter('footprint_offset').value),
            sample_period,
        )
        danger = blocked_sample is not None
        diagnostic = 'CLEAR'
        recovery = None
        if blocked_sample is not None:
            trigger = (
                'CURRENT_FOOTPRINT'
                if blocked_sample.sample_index == 0
                else 'PREDICTED_FOOTPRINT'
            )
            diagnostic = (
                f'{trigger}; cause={blocked_sample.cause}; '
                f't={blocked_sample.time_sec:.2f}s; '
                f'circle={blocked_sample.circle}; '
                f'xy=({blocked_sample.x:.2f},'
                f'{blocked_sample.y:.2f}); '
                f'yaw={blocked_sample.yaw:.2f}; '
                f'cell=({blocked_sample.cell_x},'
                f'{blocked_sample.cell_y}); '
                f'cmd=({command[0]:.2f},'
                f'{command[1]:.2f},{command[2]:.2f})'
            )
            if blocked_sample.sample_index > 0:
                recovery = safe_rotation_recovery(
                    self._blocked,
                    self._geometry,
                    (
                        float(position.x),
                        float(position.y),
                        _yaw_from_odometry(self._odom),
                    ),
                    command,
                    float(self.get_parameter('lookahead_sec').value),
                    sample_period,
                    float(
                        self.get_parameter('footprint_offset').value
                    ),
                )
                if recovery is not None:
                    diagnostic += (
                        f'; recovery=(0.00,0.00,{recovery[2]:.2f})'
                    )
        self._publish_recovery(recovery)
        self._publish(
            danger,
            'COLLISION_STOP' if danger else 'CLEAR',
            diagnostic,
        )


def main() -> None:
    """Run the independent collision guard."""
    rclpy.init()
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
