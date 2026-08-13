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

"""Fail-closed Elevator-LIO adapter for the common navigation Odometry."""

from copy import deepcopy
import json
import math
from typing import Optional

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .hardware_localization import (
    derive_body_twist,
    finite_planar_twist,
    PoseSample,
    quaternion_yaw,
)


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class HardwareLocalizationAdapter(Node):
    """Overlay factory body velocity on the pose produced by Elevator-LIO."""

    def __init__(self) -> None:
        super().__init__('m20_hardware_localization_adapter')
        self.declare_parameter('input_odometry_topic', '/LIO/odom_vehicle')
        self.declare_parameter(
            'measured_twist_topic', '/m20/locomotion/measured_twist'
        )
        self.declare_parameter(
            'output_odometry_topic', '/m20/localization/body_pose'
        )
        self.declare_parameter(
            'ready_topic', '/m20/localization/ready'
        )
        self.declare_parameter(
            'state_topic', '/m20/localization/adapter_state'
        )
        self.declare_parameter('expected_world_frame', 'world')
        self.declare_parameter('expected_body_frame', 'base_link')
        self.declare_parameter('measured_twist_timeout_sec', 0.50)
        self.declare_parameter('minimum_pose_dt_sec', 0.02)
        self.declare_parameter('maximum_pose_dt_sec', 0.50)
        self.declare_parameter('maximum_derived_linear_speed', 3.0)
        self.declare_parameter('maximum_derived_yaw_rate', 3.0)

        self._world_frame = str(
            self.get_parameter('expected_world_frame').value
        )
        self._body_frame = str(
            self.get_parameter('expected_body_frame').value
        )
        self._twist_timeout = max(
            0.05,
            float(
                self.get_parameter('measured_twist_timeout_sec').value
            ),
        )
        self._minimum_pose_dt = max(
            1.0e-3,
            float(self.get_parameter('minimum_pose_dt_sec').value),
        )
        self._maximum_pose_dt = max(
            self._minimum_pose_dt,
            float(self.get_parameter('maximum_pose_dt_sec').value),
        )
        self._maximum_derived_linear = max(
            0.0,
            float(
                self.get_parameter('maximum_derived_linear_speed').value
            ),
        )
        self._maximum_derived_yaw = max(
            0.0,
            float(self.get_parameter('maximum_derived_yaw_rate').value),
        )
        self._last_pose: Optional[PoseSample] = None
        self._last_measured_twist: Optional[TwistStamped] = None
        self._last_measured_receipt = -1.0e9
        self._ready = False
        self._last_state = ''

        qos = _latched_qos()
        self._odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('output_odometry_topic').value),
            20,
        )
        self._ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), qos
        )
        self._state_publisher = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), qos
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter('measured_twist_topic').value),
            self._measured_twist_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('input_odometry_topic').value),
            self._odometry_callback,
            20,
        )
        self._publish_state('WAITING_FOR_LIO', 'none')
        self.get_logger().info(
            'hardware localization adapter ready: Elevator-LIO pose + '
            'factory measured body velocity'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _stamp_seconds(message: Odometry) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _twist_tuple(message: TwistStamped):
        return (
            float(message.twist.linear.x),
            float(message.twist.linear.y),
            float(message.twist.angular.z),
        )

    def _publish_state(self, state: str, velocity_source: str) -> None:
        payload = json.dumps(
            {
                'ready': self._ready,
                'state': state,
                'velocity_source': velocity_source,
                'world_frame': self._world_frame,
                'body_frame': self._body_frame,
            },
            sort_keys=True,
            separators=(',', ':'),
        )
        self._ready_publisher.publish(Bool(data=self._ready))
        if payload != self._last_state:
            self._state_publisher.publish(String(data=payload))
            self._last_state = payload

    def _measured_twist_callback(self, message: TwistStamped) -> None:
        if message.header.frame_id not in {'', self._body_frame}:
            self.get_logger().warn(
                'ignoring measured TwistStamped with frame '
                f'{message.header.frame_id!r}; expected {self._body_frame!r}'
            )
            return
        if not finite_planar_twist(self._twist_tuple(message)):
            self.get_logger().warn('ignoring non-finite measured body twist')
            return
        self._last_measured_twist = deepcopy(message)
        self._last_measured_receipt = self._now()

    def _pose_sample(self, message: Odometry) -> PoseSample:
        pose = message.pose.pose
        stamp = self._stamp_seconds(message)
        if stamp <= 0.0:
            stamp = self._now()
        return PoseSample(
            stamp_sec=stamp,
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=quaternion_yaw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )

    def _derived_twist(self, sample: PoseSample):
        derived = derive_body_twist(
            self._last_pose,
            sample,
            self._minimum_pose_dt,
            self._maximum_pose_dt,
        )
        if derived is None:
            return None
        if (
            math.hypot(derived[0], derived[1])
            > self._maximum_derived_linear
            or abs(derived[2]) > self._maximum_derived_yaw
        ):
            return None
        return derived

    def _odometry_callback(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self._world_frame
            or message.child_frame_id != self._body_frame
        ):
            self._ready = False
            self._publish_state('FRAME_MISMATCH', 'none')
            self.get_logger().error(
                'Elevator-LIO frame contract mismatch: got '
                f'{message.header.frame_id!r}->{message.child_frame_id!r}, '
                f'expected {self._world_frame!r}->{self._body_frame!r}'
            )
            return

        sample = self._pose_sample(message)
        derived = self._derived_twist(sample)
        output = deepcopy(message)
        velocity_source = 'pose_difference'
        measured = self._last_measured_twist
        if (
            measured is not None
            and self._now() - self._last_measured_receipt
            <= self._twist_timeout
        ):
            output.twist.twist = deepcopy(measured.twist)
            velocity_source = 'factory_motion_status'
        elif derived is not None:
            twist = Twist()
            twist.linear.x = derived[0]
            twist.linear.y = derived[1]
            twist.angular.z = derived[2]
            output.twist.twist = twist
        else:
            # No fabricated motion: the first valid pose remains usable for
            # localization, while the state explicitly reports pose-only.
            output.twist.twist = Twist()
            velocity_source = 'pose_only_initializing'

        self._last_pose = sample
        self._odom_publisher.publish(output)
        self._ready = True
        self._publish_state('READY', velocity_source)


def main(args=None) -> None:
    """Run the real-robot localization adapter."""
    rclpy.init(args=args)
    node = HardwareLocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
