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

"""Publish preloaded floor maps and switch the active map atomically."""

from pathlib import Path
import time
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from m20_warehouse_interfaces.msg import FloorState
from m20_warehouse_interfaces.srv import SwitchMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String, UInt64
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .configuration import load_system_config
from .map_assets import (
    read_ascii_pcd,
    read_pgm,
    validate_generated_assets,
)


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _cloud_message(points: np.ndarray, frame_id: str) -> PointCloud2:
    fields = [
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
    return point_cloud2.create_cloud(
        Header(frame_id=frame_id),
        fields,
        np.ascontiguousarray(points, dtype=np.float32),
    )


class FlatMultiFloorMapServer(Node):
    """Preload deterministic maps and own the active-map generation."""

    def __init__(self) -> None:
        super().__init__('m20_flat_multifloor_map_server')
        default_root = Path(
            get_package_share_directory('m20_warehouse_inspection')
        )
        self.declare_parameter(
            'config_path',
            str(default_root / 'config' / 'flat_multifloor_system.yaml'),
        )
        self.declare_parameter('package_root', str(default_root))
        self.declare_parameter('initial_floor', 'F1')
        self.declare_parameter('republish_period_sec', 10.0)
        self.declare_parameter('switch_commit_delay_sec', 0.05)

        config_path = Path(
            self.get_parameter('config_path').get_parameter_value().string_value
        )
        package_root = Path(
            self.get_parameter('package_root').get_parameter_value().string_value
        )
        self._config = load_system_config(config_path)
        validate_generated_assets(self._config, package_root)
        self._active_floor = (
            self.get_parameter('initial_floor').get_parameter_value().string_value
        )
        if self._active_floor not in self._config['floors']:
            raise ValueError(f'unknown initial_floor {self._active_floor!r}')
        self._generation = 1

        overview_frame = self._config['frames']['overview']
        active_frame = self._config['frames']['active_map']
        floor_cloud_values: Dict[str, np.ndarray] = {}
        for floor_id, floor in self._config['floors'].items():
            values = read_ascii_pcd(package_root / floor['pcd_file'])
            values[:, :3] += np.asarray(
                floor['simulation_offset'], dtype=np.float32
            )
            floor_cloud_values[floor_id] = values

        all_floor_values = np.concatenate(
            [
                floor_cloud_values[floor_id]
                for floor_id in self._config['floors']
            ]
        )
        self._all_floors_cloud = _cloud_message(
            all_floor_values, overview_frame
        )
        self._floor_clouds = {
            floor_id: _cloud_message(values, active_frame)
            for floor_id, values in floor_cloud_values.items()
        }
        # RViz must not draw the same active-floor points through both the
        # overview and active displays. Precompute one inactive-only overview
        # per active floor to avoid view-dependent depth/color flicker.
        self._inactive_floor_clouds = {
            active_floor: _cloud_message(
                np.concatenate(
                    [
                        values
                        for floor_id, values in floor_cloud_values.items()
                        if floor_id != active_floor
                    ]
                ),
                overview_frame,
            )
            for active_floor in self._config['floors']
        }
        self._floor_occupancies = {
            floor_id: self._load_occupancy(package_root, floor_id)
            for floor_id in self._config['floors']
        }
        self._active_cloud = self._floor_clouds[self._active_floor]
        self._inactive_floor_cloud = self._inactive_floor_clouds[
            self._active_floor
        ]
        self._active_occupancy = self._floor_occupancies[self._active_floor]
        self._markers = self._build_markers()

        qos = _latched_qos()
        self._all_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/m20/visualization/all_floors_cloud',
            qos,
        )
        self._inactive_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/m20/visualization/inactive_floors_cloud',
            qos,
        )
        self._active_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/m20/map/active_global_cloud',
            qos,
        )
        # Canonical SCAN-Planner map topic. Publishing the same message here
        # lets the native renderer and the unmodified vendor RViz profile run
        # without hiding their original topic graph behind project aliases.
        self._scan_global_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/map_generator/global_cloud',
            qos,
        )
        self._occupancy_publisher = self.create_publisher(
            OccupancyGrid,
            '/m20/map/active_occupancy',
            qos,
        )
        self._floor_publisher = self.create_publisher(
            String, '/m20/map/active_floor', qos
        )
        self._generation_publisher = self.create_publisher(
            UInt64, '/m20/map/generation', qos
        )
        self._ready_publisher = self.create_publisher(
            Bool, '/m20/map/ready', qos
        )
        self._state_publisher = self.create_publisher(
            FloorState, '/m20/map/state', qos
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, '/m20/visualization/floor_markers', qos
        )
        self._switch_service = self.create_service(
            SwitchMap, '/m20/map/switch', self._switch_map
        )
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_identity_map_transform(overview_frame, active_frame)

        self._initial_timer = self.create_timer(0.25, self._publish_initial)
        period = (
            self.get_parameter('republish_period_sec')
            .get_parameter_value()
            .double_value
        )
        self._republish_timer = (
            self.create_timer(period, self.publish_snapshot)
            if period > 0.0
            else None
        )
        self.get_logger().info(
            f'preloaded {len(self._config["floors"])} flat floor maps; '
            f'active_floor={self._active_floor}, generation={self._generation}, '
            f'all_points={all_floor_values.shape[0]}'
        )

    def _publish_identity_map_transform(
        self, overview_frame: str, active_frame: str
    ) -> None:
        if overview_frame == active_frame:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = overview_frame
        transform.child_frame_id = active_frame
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)

    def _load_occupancy(
        self, package_root: Path, floor_id: str
    ) -> OccupancyGrid:
        floor = self._config['floors'][floor_id]
        yaml_path = package_root / floor['occupancy_file']
        with yaml_path.open('r', encoding='utf-8') as stream:
            description = yaml.safe_load(stream)
        image = read_pgm(yaml_path.parent / description['image'])
        source_order = np.flipud(image)
        occupancy = np.full(source_order.shape, -1, dtype=np.int8)
        occupancy[source_order >= 250] = 0
        occupancy[source_order <= 10] = 100

        message = OccupancyGrid()
        message.header.frame_id = self._config['frames']['active_map']
        message.info.resolution = float(description['resolution'])
        message.info.width = int(image.shape[1])
        message.info.height = int(image.shape[0])
        offset = floor['simulation_offset']
        message.info.origin.position.x = float(description['origin'][0] + offset[0])
        message.info.origin.position.y = float(description['origin'][1] + offset[1])
        message.info.origin.position.z = float(description['origin'][2] + offset[2])
        message.info.origin.orientation.w = 1.0
        message.data = occupancy.ravel().tolist()
        return message

    def _floor_boundary(
        self,
        floor_id: str,
        marker_id: int,
        active: bool,
    ) -> Marker:
        floor = self._config['floors'][floor_id]
        simulation = self._config['simulation']
        half_x = float(simulation['floor_size_x']) / 2.0
        half_y = float(simulation['floor_size_y']) / 2.0
        offset_x, offset_y, offset_z = (
            float(value) for value in floor['simulation_offset']
        )
        segments: List[
            Tuple[Tuple[float, float], Tuple[float, float]]
        ] = [
            ((-half_x, -half_y), (half_x, -half_y)),
            ((half_x, -half_y), (half_x, half_y)),
            ((half_x, half_y), (-half_x, half_y)),
            ((-half_x, half_y), (-half_x, -half_y)),
        ]
        gateway = floor.get('transition_gateway')
        if gateway is not None:
            center_y = float(gateway['center_y'])
            half_width = float(gateway['width']) / 2.0
            edge_x = (
                -half_x
                if gateway['edge'] == 'min_x'
                else half_x
            )
            edge_index = 3 if gateway['edge'] == 'min_x' else 1
            segments[edge_index:edge_index + 1] = [
                ((edge_x, -half_y), (edge_x, center_y - half_width)),
                ((edge_x, center_y + half_width), (edge_x, half_y)),
            ]
        marker = Marker()
        marker.header.frame_id = self._config['frames']['overview']
        marker.ns = 'floor_boundaries'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.22 if active else 0.08
        if active:
            marker.color.r = 1.0
            marker.color.g = 0.82
            marker.color.b = 0.1
            marker.color.a = 1.0
        else:
            marker.color.r = 0.25
            marker.color.g = 0.65
            marker.color.b = 1.0
            marker.color.a = 0.8
        marker.points = []
        for start, finish in segments:
            marker.points.extend(
                [
                    Point(
                        x=start[0] + offset_x,
                        y=start[1] + offset_y,
                        z=offset_z + 0.05,
                    ),
                    Point(
                        x=finish[0] + offset_x,
                        y=finish[1] + offset_y,
                        z=offset_z + 0.05,
                    ),
                ]
            )
        return marker

    def _floor_label(
        self,
        floor_id: str,
        marker_id: int,
        active: bool,
    ) -> Marker:
        floor = self._config['floors'][floor_id]
        offset = floor['simulation_offset']
        half_y = float(self._config['simulation']['floor_size_y']) / 2.0
        marker = Marker()
        marker.header.frame_id = self._config['frames']['overview']
        marker.ns = 'floor_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(offset[0])
        marker.pose.position.y = float(offset[1] + half_y + 1.8)
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.8
        marker.color.r = 1.0 if active else 0.45
        marker.color.g = 0.82 if active else 0.75
        marker.color.b = 0.1 if active else 1.0
        marker.color.a = 1.0
        marker.text = (
            f'{floor_id}  ACTIVE' if active else f'{floor_id}  INACTIVE'
        )
        return marker

    def _build_markers(self) -> MarkerArray:
        markers = []
        for index, floor_id in enumerate(self._config['floors']):
            active = floor_id == self._active_floor
            markers.append(self._floor_boundary(floor_id, index, active))
            markers.append(self._floor_label(floor_id, 100 + index, active))
        return MarkerArray(markers=markers)

    def _publish_initial(self) -> None:
        self._initial_timer.cancel()
        self.publish_snapshot()

    def _publish_state(
        self,
        floor_id: str,
        generation: int,
        ready: bool,
        phase: str,
        message: str,
    ) -> None:
        """Publish both typed and phase-1 compatibility state topics."""
        stamp = self.get_clock().now().to_msg()
        state = FloorState()
        state.header.stamp = stamp
        state.header.frame_id = self._config['frames']['active_map']
        state.floor_id = floor_id
        state.generation = generation
        state.ready = ready
        state.phase = phase
        state.message = message
        self._floor_publisher.publish(String(data=floor_id))
        self._generation_publisher.publish(UInt64(data=generation))
        self._ready_publisher.publish(Bool(data=ready))
        self._state_publisher.publish(state)

    def _switch_map(
        self,
        request: SwitchMap.Request,
        response: SwitchMap.Response,
    ) -> SwitchMap.Response:
        """Execute a compare-and-swap transition to a preloaded floor map."""
        target = request.target_floor.strip()
        response.active_floor = self._active_floor
        response.generation = self._generation

        if target not in self._floor_clouds:
            response.success = False
            response.message = f'unknown target floor {target!r}'
            return response
        if request.expected_current_generation != self._generation:
            response.success = False
            response.message = (
                'generation conflict: expected '
                f'{request.expected_current_generation}, active '
                f'{self._generation}'
            )
            return response
        if target == self._active_floor:
            response.success = True
            response.message = 'target floor is already active'
            return response

        next_generation = self._generation + 1
        self._publish_state(
            target,
            next_generation,
            False,
            'LOADING',
            f'preparing preloaded map {target}',
        )
        commit_delay = (
            self.get_parameter('switch_commit_delay_sec')
            .get_parameter_value()
            .double_value
        )
        if commit_delay > 0.0:
            time.sleep(commit_delay)

        # These assignments are the transaction commit point. All large map
        # assets have already been validated and loaded during node startup.
        self._active_floor = target
        self._generation = next_generation
        self._active_cloud = self._floor_clouds[target]
        self._inactive_floor_cloud = self._inactive_floor_clouds[target]
        self._active_occupancy = self._floor_occupancies[target]
        self._markers = self._build_markers()
        self.publish_snapshot()

        response.success = True
        response.active_floor = self._active_floor
        response.generation = self._generation
        response.message = 'active map committed'
        self.get_logger().info(
            f'active map committed: floor={self._active_floor}, '
            f'generation={self._generation}'
        )
        return response

    def publish_snapshot(self) -> None:
        """Publish a consistent, latched snapshot of the committed map."""
        stamp = self.get_clock().now().to_msg()
        self._all_floors_cloud.header.stamp = stamp
        self._inactive_floor_cloud.header.stamp = stamp
        self._active_cloud.header.stamp = stamp
        self._active_occupancy.header.stamp = stamp
        self._active_occupancy.info.map_load_time = stamp
        for marker in self._markers.markers:
            marker.header.stamp = stamp
            marker.lifetime = Duration(seconds=0.0).to_msg()

        self._all_cloud_publisher.publish(self._all_floors_cloud)
        self._inactive_cloud_publisher.publish(
            self._inactive_floor_cloud
        )
        self._active_cloud_publisher.publish(self._active_cloud)
        self._scan_global_cloud_publisher.publish(self._active_cloud)
        self._occupancy_publisher.publish(self._active_occupancy)
        self._marker_publisher.publish(self._markers)
        self._publish_state(
            self._active_floor,
            self._generation,
            True,
            'READY',
            'active map snapshot is ready',
        )


def main() -> None:
    """Run the preloaded flat multi-floor map server."""
    rclpy.init()
    node = None
    try:
        node = FlatMultiFloorMapServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (FileNotFoundError, KeyError, RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'm20_flat_multifloor_map_server: {error}')
        raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
