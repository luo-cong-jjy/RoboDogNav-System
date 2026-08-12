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

"""Generate local SCAN sensing from the current map generation."""

import math
from typing import Optional

from m20_warehouse_interfaces.msg import FloorState, LocalSensingState
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .point_cloud_filter import rotating_point_slice, select_local_points


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        depth=5,
        # Reliable publication prevents fragmented PointCloud2 samples from
        # vanishing silently. Best-effort SCAN/RViz readers remain compatible
        # with a reliable writer under DDS requested/offered semantics.
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _cloud_fields():
    return [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='intensity',
            offset=12,
            datatype=PointField.FLOAT32,
            count=1,
        ),
    ]


def _cloud_to_numpy(message: PointCloud2) -> np.ndarray:
    """Decode the x/y/z/intensity layout published by the map server."""
    required_offsets = {'x': 0, 'y': 4, 'z': 8, 'intensity': 12}
    actual_offsets = {field.name: field.offset for field in message.fields}
    if any(
        actual_offsets.get(name) != offset
        for name, offset in required_offsets.items()
    ):
        raise ValueError('global cloud has an unsupported field layout')
    if message.point_step < 16:
        raise ValueError('global cloud point_step is smaller than 16 bytes')
    count = int(message.width) * int(message.height)
    if count == 0:
        return np.empty((0, 4), dtype=np.float32)
    byte_order = '>' if message.is_bigendian else '<'
    dtype = np.dtype(
        {
            'names': ['x', 'y', 'z', 'intensity'],
            'formats': [f'{byte_order}f4'] * 4,
            'offsets': [0, 4, 8, 12],
            'itemsize': int(message.point_step),
        }
    )
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    return np.ascontiguousarray(
        np.column_stack(
            [
                records['x'],
                records['y'],
                records['z'],
                records['intensity'],
            ]
        ),
        dtype=np.float32,
    )


class DynamicLocalSensing(Node):
    """Publish only local points belonging to the committed map generation."""

    def __init__(self) -> None:
        super().__init__('m20_dynamic_local_sensing')
        self.declare_parameter(
            'global_cloud_topic', '/m20/map/active_global_cloud'
        )
        self.declare_parameter('floor_state_topic', '/m20/map/state')
        self.declare_parameter('body_pose_topic', '/m20/sim/body_pose')
        self.declare_parameter('local_cloud_topic', '/m20/sensing/local_cloud')
        self.declare_parameter(
            'sensor_cloud_topic', '/m20/sensing/sensor_cloud'
        )
        self.declare_parameter('lidar_pose_topic', '/m20/sensing/lidar_pose')
        self.declare_parameter('sensing_radius', 20.0)
        self.declare_parameter('max_points_per_cloud', 3500)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lidar_offset_x', 0.18)
        self.declare_parameter('lidar_offset_z', 0.12)
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('lidar_frame', 'lidar_link')

        self._radius = max(
            0.1, float(self.get_parameter('sensing_radius').value)
        )
        self._max_points = max(
            100, int(self.get_parameter('max_points_per_cloud').value)
        )
        self._offset_x = float(self.get_parameter('lidar_offset_x').value)
        self._offset_z = float(self.get_parameter('lidar_offset_z').value)
        self._output_frame = str(self.get_parameter('output_frame').value)
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        self._floor_id = ''
        self._generation = 0
        self._state_ready = False
        self._map_points: Optional[np.ndarray] = None
        self._candidate_points: Optional[np.ndarray] = None
        self._body_pose: Optional[Odometry] = None
        self._fresh_cloud_count = 0
        self._cloud_phase = 0
        self._last_point_count = 0
        self._last_bounds = (0.0, 0.0, 0.0, 0.0)

        self._local_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('local_cloud_topic').value),
            _sensor_qos(),
        )
        # Keep a dedicated RViz/debug stream matching SCAN-Planner's original
        # sensor_cloud convention. Both streams contain the current scan slice
        # in the configured world/map frame.
        self._sensor_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('sensor_cloud_topic').value),
            _sensor_qos(),
        )
        self._lidar_pose_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('lidar_pose_topic').value),
            20,
        )
        self._state_publisher = self.create_publisher(
            LocalSensingState, '/m20/sensing/state', _latched_qos()
        )
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            str(self.get_parameter('global_cloud_topic').value),
            self._cloud_callback,
            _latched_qos(),
        )
        self._state_subscription = self.create_subscription(
            FloorState,
            str(self.get_parameter('floor_state_topic').value),
            self._floor_state_callback,
            _latched_qos(),
        )
        self._pose_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('body_pose_topic').value),
            self._pose_callback,
            20,
        )
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._publish_local_cloud)
        self._publish_sensing_state(False, 'waiting for active map')

    def _floor_state_callback(self, state: FloorState) -> None:
        if not state.ready:
            self._floor_id = state.floor_id
            self._generation = state.generation
            self._state_ready = False
            self._map_points = None
            self._candidate_points = None
            self._fresh_cloud_count = 0
            self._cloud_phase = 0
            self._last_point_count = 0
            self._last_bounds = (0.0, 0.0, 0.0, 0.0)
            self._publish_empty_cloud()
            self._publish_sensing_state(False, 'map transition in progress')
            return

        changed = (
            state.floor_id != self._floor_id
            or state.generation != self._generation
        )
        self._floor_id = state.floor_id
        self._generation = state.generation
        self._state_ready = True
        if changed:
            self._fresh_cloud_count = 0
            self._cloud_phase = 0
        if self._candidate_points is not None:
            self._map_points = self._candidate_points
            self._candidate_points = None
        self._publish_sensing_state(False, 'waiting for a fresh local cloud')

    def _cloud_callback(self, cloud: PointCloud2) -> None:
        try:
            points = _cloud_to_numpy(cloud)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        if self._state_ready:
            self._map_points = points
            self._fresh_cloud_count = 0
            self._cloud_phase = 0
        else:
            self._candidate_points = points

    def _pose_callback(self, pose: Odometry) -> None:
        self._body_pose = pose

    def _publish_empty_cloud(self) -> None:
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._output_frame,
        )
        empty = np.empty((0, 4), dtype=np.float32)
        cloud = point_cloud2.create_cloud(header, _cloud_fields(), empty)
        self._local_publisher.publish(cloud)
        self._sensor_cloud_publisher.publish(cloud)

    def _publish_local_cloud(self) -> None:
        if (
            not self._state_ready
            or self._map_points is None
            or self._body_pose is None
        ):
            return
        position = self._body_pose.pose.pose.position
        local_points = select_local_points(
            self._map_points,
            position.x,
            position.y,
            self._radius,
        )
        local_points = rotating_point_slice(
            local_points,
            self._max_points,
            self._cloud_phase,
        )
        self._cloud_phase += 1
        self._last_point_count = int(local_points.shape[0])
        if self._last_point_count:
            self._last_bounds = (
                float(local_points[:, 0].min()),
                float(local_points[:, 0].max()),
                float(local_points[:, 1].min()),
                float(local_points[:, 1].max()),
            )
        else:
            self._last_bounds = (0.0, 0.0, 0.0, 0.0)
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._output_frame)
        cloud = point_cloud2.create_cloud(
            header, _cloud_fields(), local_points
        )
        self._local_publisher.publish(cloud)
        self._sensor_cloud_publisher.publish(cloud)
        self._publish_lidar_pose(stamp)
        self._fresh_cloud_count += 1
        self._publish_sensing_state(
            True,
            f'published {local_points.shape[0]} current-generation points',
        )

    def _publish_lidar_pose(self, stamp) -> None:
        body = self._body_pose
        orientation = body.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z),
            1.0 - 2.0 * orientation.z * orientation.z,
        )
        pose = Odometry()
        pose.header.stamp = stamp
        pose.header.frame_id = self._output_frame
        pose.child_frame_id = self._lidar_frame
        pose.pose.pose.position.x = (
            body.pose.pose.position.x + self._offset_x * math.cos(yaw)
        )
        pose.pose.pose.position.y = (
            body.pose.pose.position.y + self._offset_x * math.sin(yaw)
        )
        pose.pose.pose.position.z = (
            body.pose.pose.position.z + self._offset_z
        )
        pose.pose.pose.orientation = orientation
        pose.twist = body.twist
        self._lidar_pose_publisher.publish(pose)

    def _publish_sensing_state(self, ready: bool, message: str) -> None:
        state = LocalSensingState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self._output_frame
        state.floor_id = self._floor_id
        state.generation = self._generation
        state.ready = ready
        state.fresh_cloud_count = self._fresh_cloud_count
        state.point_count = self._last_point_count
        (
            state.min_x,
            state.max_x,
            state.min_y,
            state.max_y,
        ) = self._last_bounds
        state.message = message
        self._state_publisher.publish(state)


def main() -> None:
    """Run generation-aware local sensing."""
    rclpy.init()
    node = DynamicLocalSensing()
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
