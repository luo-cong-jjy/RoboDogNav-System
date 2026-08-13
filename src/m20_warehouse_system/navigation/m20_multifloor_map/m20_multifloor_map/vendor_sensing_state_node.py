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

"""Expose SCAN-Planner's native renderer as a generation-aware sensor."""

import time
from typing import Optional

from m20_warehouse_interfaces.msg import FloorState, LocalSensingState
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class VendorSensingState(Node):
    """Count native SCAN cloud frames after each committed floor map."""

    def __init__(self) -> None:
        super().__init__('m20_vendor_sensing_state')
        self.declare_parameter('floor_state_topic', '/m20/map/state')
        self.declare_parameter('cloud_topic', '/quad_0/cloud')
        self.declare_parameter('state_topic', '/m20/sensing/state')
        self.declare_parameter('reload_guard_sec', 0.25)
        self.declare_parameter('output_frame', 'world')

        self._floor: Optional[FloorState] = None
        self._fresh_cloud_count = 0
        self._accept_cloud_after = 0.0
        self._logged_ready_generation = None
        self._reload_guard = max(
            0.0, float(self.get_parameter('reload_guard_sec').value)
        )
        self._output_frame = str(self.get_parameter('output_frame').value)
        self._publisher = self.create_publisher(
            LocalSensingState,
            str(self.get_parameter('state_topic').value),
            _latched_qos(),
        )
        self.create_subscription(
            FloorState,
            str(self.get_parameter('floor_state_topic').value),
            self._floor_callback,
            _latched_qos(),
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            self._cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self._publish(False, 0, 'waiting for active floor and native cloud')

    def _floor_callback(self, state: FloorState) -> None:
        changed = (
            self._floor is None
            or state.floor_id != self._floor.floor_id
            or state.generation != self._floor.generation
        )
        self._floor = state
        if changed or not state.ready:
            self._fresh_cloud_count = 0
            self._accept_cloud_after = time.monotonic() + self._reload_guard
            self._logged_ready_generation = None
        if changed:
            self.get_logger().info(
                'native sensing generation changed: '
                f'floor={state.floor_id}, generation={state.generation}, '
                f'ready={state.ready}'
            )
        self._publish(
            False,
            0,
            (
                'waiting for native renderer map reload'
                if state.ready
                else 'map transition in progress'
            ),
        )

    def _cloud_callback(self, cloud: PointCloud2) -> None:
        if (
            self._floor is None
            or not self._floor.ready
            or time.monotonic() < self._accept_cloud_after
        ):
            return
        self._fresh_cloud_count += 1
        generation = (self._floor.floor_id, self._floor.generation)
        if self._logged_ready_generation != generation:
            self._logged_ready_generation = generation
            self.get_logger().info(
                'first fresh native cloud received: '
                f'floor={self._floor.floor_id}, '
                f'generation={self._floor.generation}, '
                f'points={int(cloud.width) * int(cloud.height)}'
            )
        self._publish(
            True,
            int(cloud.width) * int(cloud.height),
            'native SCAN local cloud is current',
        )

    def _publish(self, ready: bool, point_count: int, message: str) -> None:
        state = LocalSensingState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self._output_frame
        if self._floor is not None:
            state.floor_id = self._floor.floor_id
            state.generation = self._floor.generation
        state.ready = ready
        state.fresh_cloud_count = self._fresh_cloud_count
        state.point_count = point_count
        state.message = message
        self._publisher.publish(state)


def main() -> None:
    """Run the native-renderer readiness adapter."""
    rclpy.init()
    node = VendorSensingState()
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
