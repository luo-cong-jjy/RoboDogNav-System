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

"""ROS 2 node implementing the RViz planar kinematic backend."""

import math
from typing import List

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from m20_warehouse_interfaces.srv import SetSimulationPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .kinematics import (
    PlanarState,
    clamp_planar_command,
    integrate_planar,
    normalize_angle,
)


JOINT_NAMES: List[str] = [
    'fl_hipx_joint',
    'fl_hipy_joint',
    'fl_knee_joint',
    'fl_wheel_joint',
    'fr_hipx_joint',
    'fr_hipy_joint',
    'fr_knee_joint',
    'fr_wheel_joint',
    'hl_hipx_joint',
    'hl_hipy_joint',
    'hl_knee_joint',
    'hl_wheel_joint',
    'hr_hipx_joint',
    'hr_hipy_joint',
    'hr_knee_joint',
    'hr_wheel_joint',
]
WHEEL_INDICES = (3, 7, 11, 15)


def _quaternion_from_yaw(yaw: float):
    """Return geometry quaternion components for a planar yaw."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class RvizKinematicBackend(Node):
    """Integrate safe commands and publish odometry, TF, joints, and path."""

    def __init__(self) -> None:
        super().__init__('m20_rviz_kinematic_backend')
        self.declare_parameter('initial_x', -37.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.59)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.3)
        self.declare_parameter('max_linear_x', 0.45)
        self.declare_parameter('max_linear_y', 0.20)
        self.declare_parameter('max_angular_z', 0.65)
        self.declare_parameter('wheel_radius', 0.09)
        self.declare_parameter('path_max_poses', 3000)
        self.declare_parameter(
            'safe_cmd_topic', '/m20/control/cmd_vel_safe'
        )
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        self.declare_parameter('path_topic', '/m20/sim/path')

        self._state = PlanarState(
            x=float(self.get_parameter('initial_x').value),
            y=float(self.get_parameter('initial_y').value),
            z=float(self.get_parameter('initial_z').value),
            yaw=float(self.get_parameter('initial_yaw').value),
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._child_frame_id = str(self.get_parameter('child_frame_id').value)
        self._timeout = max(
            0.05, float(self.get_parameter('command_timeout_sec').value)
        )
        self._limits = (
            float(self.get_parameter('max_linear_x').value),
            float(self.get_parameter('max_linear_y').value),
            float(self.get_parameter('max_angular_z').value),
        )
        self._wheel_radius = max(
            0.01, float(self.get_parameter('wheel_radius').value)
        )
        self._path_max_poses = max(
            10, int(self.get_parameter('path_max_poses').value)
        )
        rate = max(1.0, float(self.get_parameter('update_rate_hz').value))
        self._command = (0.0, 0.0, 0.0)
        self._last_command_ns = 0
        self._last_update_ns = self.get_clock().now().nanoseconds
        self._wheel_phase = 0.0
        self._publish_count = 0

        self._odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 20
        )
        self._joint_publisher = self.create_publisher(
            JointState, '/joint_states', 20
        )
        self._path_publisher = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 5
        )
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_publisher = self.create_publisher(
            Bool, '/m20/sim/backend_ready', ready_qos
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter('safe_cmd_topic').value),
            self._command_callback,
            20,
        )
        self._set_pose_service = self.create_service(
            SetSimulationPose,
            '/m20/sim/set_pose',
            self._set_pose_callback,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._path = Path()
        self._path.header.frame_id = self._frame_id
        self._timer = self.create_timer(1.0 / rate, self._update)
        self._ready_publisher.publish(Bool(data=True))
        self.get_logger().info(
            'RViz kinematic backend ready: safe cmd -> '
            f'{self.get_parameter("odom_topic").value}, '
            f'initial=({self._state.x:.2f}, {self._state.y:.2f}, '
            f'{self._state.z:.2f})'
        )

    def _command_callback(self, message: Twist) -> None:
        self._command = clamp_planar_command(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self._limits,
        )
        self._last_command_ns = self.get_clock().now().nanoseconds

    def _set_pose_callback(
        self,
        request: SetSimulationPose.Request,
        response: SetSimulationPose.Response,
    ) -> SetSimulationPose.Response:
        """Teleport the RViz backend while discarding all prior motion."""
        if not all(
            math.isfinite(value) for value in (request.x, request.y, request.yaw)
        ):
            response.success = False
            response.message = 'pose contains a non-finite value'
            return response
        current = self.get_clock().now()
        self._state = PlanarState(
            x=request.x,
            y=request.y,
            z=self._state.z,
            yaw=normalize_angle(request.yaw),
        )
        self._command = (0.0, 0.0, 0.0)
        self._last_command_ns = current.nanoseconds
        self._last_update_ns = current.nanoseconds
        self._path = Path()
        self._path.header.frame_id = self._frame_id
        self._publish_count = 0
        self._publish_state(current.to_msg(), self._command)
        response.success = True
        response.message = (
            f'teleported to floor={request.floor_id}, '
            f'generation={request.generation}'
        )
        self.get_logger().info(response.message)
        return response

    def _update(self) -> None:
        current = self.get_clock().now()
        current_ns = current.nanoseconds
        dt = (current_ns - self._last_update_ns) / 1e9
        self._last_update_ns = current_ns
        if dt < 0.0 or dt > 0.2:
            dt = 0.0
        command = self._command
        if (
            self._last_command_ns == 0
            or (current_ns - self._last_command_ns) / 1e9 > self._timeout
        ):
            command = (0.0, 0.0, 0.0)
        self._state = integrate_planar(self._state, command, dt)
        self._wheel_phase += command[0] / self._wheel_radius * dt
        self._publish_state(current.to_msg(), command)

    def _publish_state(self, stamp, command) -> None:
        qx, qy, qz, qw = _quaternion_from_yaw(self._state.yaw)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self._frame_id
        odometry.child_frame_id = self._child_frame_id
        odometry.pose.pose.position.x = self._state.x
        odometry.pose.pose.position.y = self._state.y
        odometry.pose.pose.position.z = self._state.z
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = self._state.vx_world
        odometry.twist.twist.linear.y = self._state.vy_world
        odometry.twist.twist.angular.z = self._state.wz
        self._odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self._child_frame_id
        transform.transform.translation.x = self._state.x
        transform.transform.translation.y = self._state.y
        transform.transform.translation.z = self._state.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = JOINT_NAMES
        joints.position = [0.0] * len(JOINT_NAMES)
        for index in WHEEL_INDICES:
            joints.position[index] = self._wheel_phase
        joints.velocity = [0.0] * len(JOINT_NAMES)
        for index in WHEEL_INDICES:
            joints.velocity[index] = command[0] / self._wheel_radius
        self._joint_publisher.publish(joints)

        self._publish_count += 1
        if self._publish_count % 5 == 0:
            pose = PoseStamped()
            pose.header = odometry.header
            pose.pose = odometry.pose.pose
            self._path.header.stamp = stamp
            self._path.poses.append(pose)
            if len(self._path.poses) > self._path_max_poses:
                self._path.poses = self._path.poses[-self._path_max_poses:]
            self._path_publisher.publish(self._path)


def main() -> None:
    """Run the RViz kinematic backend."""
    rclpy.init()
    node = RvizKinematicBackend()
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
