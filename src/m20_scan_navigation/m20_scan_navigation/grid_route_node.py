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

"""ROS 2 adapter from inspection goals to collision-free SCAN subgoals."""

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from m20_warehouse_interfaces.msg import FloorState
from m20_warehouse_interfaces.srv import ResetNavigation
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .grid_route import (
    GridGeometry,
    make_blocked_grid,
    plan_metric_route,
)


class GridRoutePlanner(Node):
    """Plan a static-map A* route and feed short sequential goals to SCAN."""

    def __init__(self) -> None:
        super().__init__('m20_grid_route_planner')
        self.declare_parameter(
            'occupancy_topic', '/m20/map/active_occupancy'
        )
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        self.declare_parameter(
            'goal_topic', '/m20/navigation/goal_pose'
        )
        self.declare_parameter(
            'route_topic', '/m20/navigation/global_route'
        )
        self.declare_parameter(
            'scan_goal_topic', '/m20/navigation/scan_goal'
        )
        self.declare_parameter(
            'route_state_topic', '/m20/navigation/global_route_state'
        )
        self.declare_parameter('inflation_radius', 0.60)
        self.declare_parameter('nearest_free_radius', 1.50)
        self.declare_parameter('waypoint_spacing', 0.50)
        self.declare_parameter('subgoal_acceptance_radius', 0.30)
        self.declare_parameter('final_acceptance_radius', 0.15)
        self.declare_parameter('occupied_threshold', 50)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._geometry: Optional[GridGeometry] = None
        self._blocked = None
        self._position: Optional[Tuple[float, float]] = None
        self._frame_id = 'world'
        self._subgoals: List[Tuple[float, float]] = []
        self._subgoal_index = 0
        self._floor_hold = False
        self._floor_state: Optional[FloorState] = None
        self._map_ready = False
        self._route_state = 'WAITING_FOR_MAP_AND_ODOM'
        self._route_publisher = self.create_publisher(
            Path, str(self.get_parameter('route_topic').value), 10
        )
        self._state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('route_state_topic').value),
            latched_qos,
        )
        self._scan_goal_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('scan_goal_topic').value),
            10,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('occupancy_topic').value),
            self._map_callback,
            latched_qos,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            10,
        )
        self._hold_subscription = self.create_subscription(
            Bool,
            '/m20/control/floor_switch_hold',
            self._hold_callback,
            latched_qos,
        )
        self._floor_state_subscription = self.create_subscription(
            FloorState,
            '/m20/map/state',
            self._floor_state_callback,
            latched_qos,
        )
        self._reset_service = self.create_service(
            ResetNavigation,
            '/m20/navigation/route_reset',
            self._reset_callback,
        )
        self._publish_state('WAITING_FOR_MAP_AND_ODOM')

    def _publish_state(self, state: str) -> None:
        self._route_state = state
        self._state_publisher.publish(String(data=state))

    def _map_callback(self, message: OccupancyGrid) -> None:
        geometry = GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
        )
        self._blocked = make_blocked_grid(
            message.data,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            float(self.get_parameter('inflation_radius').value),
        )
        self._geometry = geometry
        if (
            not self._floor_hold
            and self._map_ready
            and self._position is not None
            and self._route_state in {
                'WAITING_FOR_MAP_AND_ODOM',
                'MAP_NOT_READY',
                'REJECTED_NOT_READY',
                'READY',
            }
        ):
            self._publish_state('READY')
        self.get_logger().info(
            f'active occupancy prepared: {geometry.width}x{geometry.height}, '
            f'inflation={self.get_parameter("inflation_radius").value:.2f}m'
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if (
            not self._floor_hold
            and self._map_ready
            and self._geometry is not None
            and self._route_state in {
                'WAITING_FOR_MAP_AND_ODOM',
                'MAP_NOT_READY',
                'REJECTED_NOT_READY',
                'READY',
            }
        ):
            self._publish_state('READY')
        self._advance_subgoal_if_reached()

    def _hold_callback(self, message: Bool) -> None:
        self._floor_hold = message.data
        if self._floor_hold:
            self._clear_route()
            self._publish_state('FLOOR_SWITCH_HOLD')
        elif self._geometry is not None and self._position is not None:
            self._publish_state(
                'READY' if self._map_ready else 'MAP_NOT_READY'
            )

    def _floor_state_callback(self, message: FloorState) -> None:
        self._floor_state = message
        self._map_ready = message.ready
        if not message.ready:
            self._clear_route()
            self._geometry = None
            self._blocked = None
            self._publish_state('MAP_NOT_READY')
        elif (
            not self._floor_hold
            and self._geometry is not None
            and self._position is not None
        ):
            self._publish_state('READY')

    def _clear_route(self) -> None:
        self._subgoals = []
        self._subgoal_index = 0
        route = Path()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = self._frame_id
        self._route_publisher.publish(route)

    def _reset_callback(
        self,
        request: ResetNavigation.Request,
        response: ResetNavigation.Response,
    ) -> ResetNavigation.Response:
        state = self._floor_state
        if (
            state is None
            or state.floor_id != request.floor_id
            or state.generation != request.generation
        ):
            response.success = False
            response.message = 'route reset floor/generation mismatch'
            return response
        self._clear_route()
        self._publish_state(
            'FLOOR_SWITCH_HOLD'
            if self._floor_hold
            else 'READY'
            if self._map_ready and self._geometry is not None
            else 'MAP_NOT_READY'
        )
        response.success = True
        response.message = 'route and subgoals cleared'
        return response

    def _publish_current_subgoal(self) -> None:
        if self._subgoal_index >= len(self._subgoals):
            self._subgoals = []
            self._publish_state('GOAL_REACHED')
            self.get_logger().info('A* subgoal sequence completed')
            return
        x, y = self._subgoals[self._subgoal_index]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.59
        goal.pose.orientation.w = 1.0
        self._scan_goal_publisher.publish(goal)
        self._publish_state('TRACKING_ROUTE')
        self.get_logger().info(
            f'SCAN subgoal {self._subgoal_index + 1}/'
            f'{len(self._subgoals)}: ({x:.2f}, {y:.2f})'
        )

    def _advance_subgoal_if_reached(self) -> None:
        if self._floor_hold or not self._subgoals or self._position is None:
            return
        target = self._subgoals[self._subgoal_index]
        is_final = self._subgoal_index == len(self._subgoals) - 1
        radius_parameter = (
            'final_acceptance_radius'
            if is_final
            else 'subgoal_acceptance_radius'
        )
        if math.dist(self._position, target) > float(
            self.get_parameter(radius_parameter).value
        ):
            return
        self._subgoal_index += 1
        self._publish_current_subgoal()

    def _goal_callback(self, message: PoseStamped) -> None:
        if self._floor_hold:
            self._publish_state('REJECTED_FLOOR_SWITCH_HOLD')
            self.get_logger().error('goal rejected: floor switch is active')
            return
        if (
            not self._map_ready
            or self._floor_state is None
            or self._geometry is None
            or self._blocked is None
            or self._position is None
        ):
            self._publish_state('REJECTED_NOT_READY')
            self.get_logger().error('goal rejected: map or odometry not ready')
            return

        goal = (
            float(message.pose.position.x),
            float(message.pose.position.y),
        )
        points = plan_metric_route(
            self._blocked,
            self._geometry,
            self._position,
            goal,
            float(self.get_parameter('nearest_free_radius').value),
            float(self.get_parameter('waypoint_spacing').value),
        )
        if len(points) < 2:
            self._publish_state('NO_ROUTE')
            self.get_logger().error(
                f'no route from {self._position} to {goal}'
            )
            return

        route = Path()
        route.header.stamp = self.get_clock().now().to_msg()
        route.header.frame_id = self._frame_id
        # SCAN prepends the current odometry pose internally. Skip the route's
        # identical first sample to avoid a zero-duration polynomial segment.
        for x, y in points:
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            # SCAN reference-path mode adds the configured M20 body height.
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        self._route_publisher.publish(route)
        length = sum(
            math.dist(points[index - 1], points[index])
            for index in range(1, len(points))
        )
        self._subgoals = list(points[1:])
        self._subgoal_index = 0
        self._publish_current_subgoal()
        self.get_logger().info(
            f'A* route published: {len(points)} poses, {length:.2f}m; '
            f'{len(self._subgoals)} sequential SCAN subgoals'
        )


def main() -> None:
    """Run the F1 grid route adapter."""
    rclpy.init()
    node = GridRoutePlanner()
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
