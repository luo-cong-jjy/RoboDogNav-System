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
import time
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
    classify_continuous_transitions,
    clear_boundary_gateway_inflation,
    clearance_from_blocked,
    compress_route_for_scan,
    GridGeometry,
    line_respects_soft_clearance,
    make_blocked_grid,
    plan_metric_route,
    prepend_reverse_escape_for_heading,
)


class GridRoutePlanner(Node):
    """Plan a static-map A* route and feed short sequential goals to SCAN."""

    def __init__(self) -> None:
        """Create the clearance-biased route and SCAN subgoal adapter."""
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
        self.declare_parameter(
            'segment_hold_topic', '/m20/control/route_segment_hold'
        )
        self.declare_parameter('inflation_radius', 0.60)
        self.declare_parameter('soft_clearance_margin', 0.20)
        self.declare_parameter('soft_clearance_weight', 4.0)
        self.declare_parameter('shared_gateway_x', 0.0)
        self.declare_parameter('shared_gateway_center_y', 0.0)
        self.declare_parameter('shared_gateway_width', 4.0)
        self.declare_parameter('nearest_free_radius', 1.50)
        self.declare_parameter('waypoint_spacing', 0.50)
        self.declare_parameter('subgoal_acceptance_radius', 0.30)
        # Match SCAN's position-only target tolerance.  A tighter adapter-side
        # value can falsely report a stall after SCAN has already stopped.
        self.declare_parameter('final_acceptance_radius', 0.20)
        self.declare_parameter('minimum_scan_goal_distance', 0.85)
        self.declare_parameter('subgoal_stall_timeout_sec', 10.0)
        self.declare_parameter('subgoal_min_progress_m', 0.05)
        self.declare_parameter('intermediate_stop_linear_speed', 0.04)
        self.declare_parameter('intermediate_stop_angular_speed', 0.08)
        self.declare_parameter('intermediate_stop_stable_sec', 0.30)
        self.declare_parameter('intermediate_stop_timeout_sec', 6.0)
        self.declare_parameter('segment_hold_release_delay_sec', 0.20)
        self.declare_parameter('continuous_handoff_enabled', True)
        self.declare_parameter('continuous_handoff_radius', 0.55)
        self.declare_parameter('continuous_handoff_distance', 0.70)
        self.declare_parameter('continuous_max_heading_change', 0.70)
        self.declare_parameter('continuous_minimum_turn_radius', 0.54)
        self.declare_parameter('continuous_minimum_extra_clearance', 0.10)
        self.declare_parameter('reverse_escape_enabled', True)
        self.declare_parameter('reverse_escape_trigger_angle', 1.0)
        self.declare_parameter('reverse_escape_alignment_angle', 0.35)
        self.declare_parameter('reverse_escape_min_distance', 0.90)
        self.declare_parameter('reverse_escape_max_distance', 1.20)
        self.declare_parameter('reverse_escape_sample_step', 0.10)
        self.declare_parameter('occupied_threshold', 50)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._geometry: Optional[GridGeometry] = None
        self._blocked = None
        self._clearance_cells = None
        self._position: Optional[Tuple[float, float]] = None
        self._yaw: Optional[float] = None
        self._linear_speed = math.inf
        self._angular_speed = math.inf
        self._frame_id = 'world'
        self._subgoals: List[Tuple[float, float]] = []
        self._continuous_transitions: List[bool] = []
        self._subgoal_index = 0
        self._floor_hold = False
        self._floor_state: Optional[FloorState] = None
        self._map_ready = False
        self._route_state = 'WAITING_FOR_MAP_AND_ODOM'
        self._last_progress_position: Optional[Tuple[float, float]] = None
        self._last_progress_time = 0.0
        self._waiting_for_intermediate_stop = False
        self._intermediate_stop_started = 0.0
        self._intermediate_stop_stable_since = 0.0
        self._segment_hold: Optional[bool] = None
        self._segment_hold_release_at = 0.0
        self._route_publisher = self.create_publisher(
            Path, str(self.get_parameter('route_topic').value), 10
        )
        self._state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('route_state_topic').value),
            latched_qos,
        )
        self._segment_hold_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('segment_hold_topic').value),
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
        self._stall_timer = self.create_timer(
            0.20, self._check_subgoal_stall
        )
        self._segment_hold_timer = self.create_timer(
            0.05, self._release_segment_hold_if_due
        )
        self._publish_segment_hold(False)
        self._publish_state('WAITING_FOR_MAP_AND_ODOM')

    def _publish_state(self, state: str) -> None:
        self._route_state = state
        self._state_publisher.publish(String(data=state))

    def _publish_segment_hold(self, hold: bool) -> None:
        """Request a safety-layer stop between fallback route segments."""
        hold = bool(hold)
        if hold == self._segment_hold:
            return
        self._segment_hold = hold
        self._segment_hold_publisher.publish(Bool(data=hold))

    def _release_segment_hold_if_due(self) -> None:
        if (
            self._segment_hold
            and self._segment_hold_release_at > 0.0
            and time.monotonic() >= self._segment_hold_release_at
        ):
            self._segment_hold_release_at = 0.0
            self._publish_segment_hold(False)

    def _map_callback(self, message: OccupancyGrid) -> None:
        geometry = GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
        )
        inflation_radius = float(
            self.get_parameter('inflation_radius').value
        )
        self._blocked = make_blocked_grid(
            message.data,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            inflation_radius,
        )
        self._blocked = clear_boundary_gateway_inflation(
            self._blocked,
            message.data,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            float(self.get_parameter('shared_gateway_x').value),
            float(
                self.get_parameter('shared_gateway_center_y').value
            ),
            float(self.get_parameter('shared_gateway_width').value),
            inflation_radius,
        )
        self._clearance_cells = clearance_from_blocked(self._blocked)
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
            f'inflation={self.get_parameter("inflation_radius").value:.2f}m, '
            'soft_margin='
            f'{self.get_parameter("soft_clearance_margin").value:.2f}m'
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        orientation = message.pose.pose.orientation
        self._yaw = math.atan2(
            2.0 * (
                float(orientation.w) * float(orientation.z)
                + float(orientation.x) * float(orientation.y)
            ),
            1.0 - 2.0 * (
                float(orientation.y) ** 2
                + float(orientation.z) ** 2
            ),
        )
        twist = message.twist.twist
        self._linear_speed = math.hypot(
            float(twist.linear.x), float(twist.linear.y)
        )
        self._angular_speed = abs(float(twist.angular.z))
        self._update_subgoal_progress()
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
        self._continue_after_intermediate_stop()

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
            self._clearance_cells = None
            self._publish_state('MAP_NOT_READY')
        elif (
            not self._floor_hold
            and self._geometry is not None
            and self._position is not None
        ):
            self._publish_state('READY')

    def _clear_route(self) -> None:
        self._subgoals = []
        self._continuous_transitions = []
        self._subgoal_index = 0
        self._last_progress_position = None
        self._last_progress_time = 0.0
        self._waiting_for_intermediate_stop = False
        self._intermediate_stop_started = 0.0
        self._intermediate_stop_stable_since = 0.0
        self._segment_hold_release_at = 0.0
        self._publish_segment_hold(False)
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

    def _scan_execution_target(
        self, logical_target: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Extend a short logical target without leaving inflated free space."""
        if (
            self._position is None
            or self._geometry is None
            or self._blocked is None
        ):
            return logical_target
        distance = math.dist(self._position, logical_target)
        minimum = max(
            0.05,
            float(
                self.get_parameter('minimum_scan_goal_distance').value
            ),
        )
        if distance < 1e-6 or distance >= minimum:
            return logical_target

        start_cell = self._geometry.world_to_cell(self._position)
        if (
            not self._geometry.contains(start_cell)
            or self._blocked[start_cell[1], start_cell[0]]
        ):
            return logical_target
        scale = minimum / distance
        extended = (
            self._position[0]
            + (logical_target[0] - self._position[0]) * scale,
            self._position[1]
            + (logical_target[1] - self._position[1]) * scale,
        )
        extended_cell = self._geometry.world_to_cell(extended)
        if (
            not self._geometry.contains(extended_cell)
            or not line_respects_soft_clearance(
                start_cell,
                extended_cell,
                self._blocked,
                self._clearance_cells,
                float(
                    self.get_parameter('soft_clearance_margin').value
                ) / self._geometry.resolution,
            )
        ):
            return logical_target
        return extended

    def _publish_current_subgoal(self) -> None:
        if self._subgoal_index >= len(self._subgoals):
            self._subgoals = []
            self._last_progress_position = None
            self._last_progress_time = 0.0
            self._publish_state('GOAL_REACHED')
            self.get_logger().info('A* subgoal sequence completed')
            return
        logical_x, logical_y = self._subgoals[self._subgoal_index]
        x, y = self._scan_execution_target((logical_x, logical_y))
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.59
        goal.pose.orientation.w = 1.0
        self._scan_goal_publisher.publish(goal)
        self._publish_state('TRACKING_ROUTE')
        self._last_progress_position = self._position
        self._last_progress_time = time.monotonic()
        extension = (
            ''
            if (x, y) == (logical_x, logical_y)
            else f'; command=({x:.2f}, {y:.2f})'
        )
        self.get_logger().info(
            f'SCAN subgoal {self._subgoal_index + 1}/'
            f'{len(self._subgoals)}: ({logical_x:.2f}, {logical_y:.2f})'
            f'{extension}'
        )

    def _update_subgoal_progress(self) -> None:
        if (
            not self._subgoals
            or self._position is None
            or self._last_progress_position is None
        ):
            return
        minimum = max(
            0.01,
            float(self.get_parameter('subgoal_min_progress_m').value),
        )
        if math.dist(
            self._position, self._last_progress_position
        ) >= minimum:
            self._last_progress_position = self._position
            self._last_progress_time = time.monotonic()

    def _check_subgoal_stall(self) -> None:
        if (
            self._floor_hold
            or not self._subgoals
            or self._position is None
            or self._last_progress_position is None
        ):
            return
        if self._waiting_for_intermediate_stop:
            self._continue_after_intermediate_stop()
            return
        timeout = max(
            1.0,
            float(self.get_parameter('subgoal_stall_timeout_sec').value),
        )
        if time.monotonic() - self._last_progress_time <= timeout:
            return
        index = self._subgoal_index
        target = self._subgoals[index]
        self._clear_route()
        self._publish_state('SUBGOAL_STALLED')
        self.get_logger().error(
            f'SCAN subgoal {index + 1} made no progress for '
            f'{timeout:.1f}s: ({target[0]:.2f}, {target[1]:.2f})'
        )

    def _advance_subgoal_if_reached(self) -> None:
        if (
            self._floor_hold
            or not self._subgoals
            or self._position is None
            or self._waiting_for_intermediate_stop
        ):
            return
        target = self._subgoals[self._subgoal_index]
        is_final = self._subgoal_index == len(self._subgoals) - 1
        continuous = (
            not is_final
            and self._subgoal_index < len(self._continuous_transitions)
            and self._continuous_transitions[self._subgoal_index]
        )
        if is_final:
            acceptance_radius = float(
                self.get_parameter('final_acceptance_radius').value
            )
        elif continuous:
            acceptance_radius = max(
                float(
                    self.get_parameter('subgoal_acceptance_radius').value
                ),
                float(
                    self.get_parameter('continuous_handoff_radius').value
                ),
            )
        else:
            acceptance_radius = float(
                self.get_parameter('subgoal_acceptance_radius').value
            )
        if math.dist(self._position, target) > acceptance_radius:
            return
        self._subgoal_index += 1
        if self._subgoal_index >= len(self._subgoals):
            self._publish_current_subgoal()
            return
        if continuous:
            # A new SCAN goal while the old trajectory is still moving keeps
            # its measured/commanded derivative and produces a smooth local
            # connector.  Static geometry classification above limits this
            # to shallow turns with enough M20 rolling-sweep clearance.
            self.get_logger().info(
                f'SCAN subgoal {self._subgoal_index}/'
                f'{len(self._subgoals)} reached continuous handoff; '
                'carrying velocity into the next clearance segment'
            )
            self._publish_current_subgoal()
            return
        # Let the current SCAN trajectory reach its zero-velocity endpoint
        # before issuing a differently oriented segment.  Otherwise SCAN
        # preserves the incoming velocity and produces a curved connector
        # that a rolling M20 can sweep outside the inflated grid route.
        self._waiting_for_intermediate_stop = True
        self._intermediate_stop_started = time.monotonic()
        self._intermediate_stop_stable_since = 0.0
        self._last_progress_time = self._intermediate_stop_started
        self._publish_segment_hold(True)
        self.get_logger().info(
            f'SCAN subgoal {self._subgoal_index}/'
            f'{len(self._subgoals)} reached; waiting for measured stop '
            'before the next heading segment'
        )

    def _continue_after_intermediate_stop(self) -> None:
        """Publish the next route segment only after a bounded measured stop."""
        if not self._waiting_for_intermediate_stop:
            return
        now = time.monotonic()
        timeout = max(
            1.0,
            float(
                self.get_parameter('intermediate_stop_timeout_sec').value
            ),
        )
        if now - self._intermediate_stop_started > timeout:
            index = self._subgoal_index
            self._clear_route()
            self._publish_state('SUBGOAL_STOP_TIMEOUT')
            self.get_logger().error(
                f'SCAN did not stop before subgoal {index + 1} within '
                f'{timeout:.1f}s'
            )
            return
        stopped = (
            self._linear_speed
            <= float(
                self.get_parameter(
                    'intermediate_stop_linear_speed'
                ).value
            )
            and self._angular_speed
            <= float(
                self.get_parameter(
                    'intermediate_stop_angular_speed'
                ).value
            )
        )
        if not stopped:
            self._intermediate_stop_stable_since = 0.0
            return
        if self._intermediate_stop_stable_since <= 0.0:
            self._intermediate_stop_stable_since = now
            return
        stable = max(
            0.0,
            float(
                self.get_parameter('intermediate_stop_stable_sec').value
            ),
        )
        if now - self._intermediate_stop_stable_since < stable:
            return
        self._waiting_for_intermediate_stop = False
        self._intermediate_stop_started = 0.0
        self._intermediate_stop_stable_since = 0.0
        self._publish_current_subgoal()
        self._segment_hold_release_at = now + max(
            0.05,
            float(
                self.get_parameter(
                    'segment_hold_release_delay_sec'
                ).value
            ),
        )

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
            float(self.get_parameter('soft_clearance_margin').value),
            float(self.get_parameter('soft_clearance_weight').value),
            self._clearance_cells,
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
        self._subgoals = compress_route_for_scan(
            points,
            self._blocked,
            self._geometry,
            self._clearance_cells,
            float(self.get_parameter('soft_clearance_margin').value),
        )
        original_first_subgoal = (
            self._subgoals[0] if self._subgoals else None
        )
        if (
            bool(self.get_parameter('reverse_escape_enabled').value)
            and self._yaw is not None
        ):
            self._subgoals = prepend_reverse_escape_for_heading(
                self._position,
                self._yaw,
                self._subgoals,
                self._blocked,
                self._geometry,
                float(
                    self.get_parameter(
                        'reverse_escape_trigger_angle'
                    ).value
                ),
                float(
                    self.get_parameter(
                        'reverse_escape_alignment_angle'
                    ).value
                ),
                float(
                    self.get_parameter(
                        'reverse_escape_min_distance'
                    ).value
                ),
                float(
                    self.get_parameter(
                        'reverse_escape_max_distance'
                    ).value
                ),
                float(
                    self.get_parameter(
                        'reverse_escape_sample_step'
                    ).value
                ),
            )
        reverse_escape_added = (
            original_first_subgoal is not None
            and self._subgoals
            and self._subgoals[0] != original_first_subgoal
        )
        normal_radius = float(
            self.get_parameter('subgoal_acceptance_radius').value
        )
        while (
            len(self._subgoals) > 1
            and math.dist(self._position, self._subgoals[0])
            <= normal_radius
        ):
            self._subgoals.pop(0)
        if not self._subgoals:
            self._publish_state('NO_ROUTE')
            self.get_logger().error(
                'route could not be converted to safe SCAN subgoals'
            )
            return
        if bool(
            self.get_parameter('continuous_handoff_enabled').value
        ):
            self._continuous_transitions = (
                classify_continuous_transitions(
                    self._position,
                    self._subgoals,
                    self._blocked,
                    self._geometry,
                    self._clearance_cells,
                    float(
                        self.get_parameter(
                            'continuous_max_heading_change'
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            'continuous_minimum_turn_radius'
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            'continuous_handoff_distance'
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            'continuous_minimum_extra_clearance'
                        ).value
                    ),
                )
            )
        else:
            self._continuous_transitions = [False] * len(self._subgoals)
        self._subgoal_index = 0
        self._publish_current_subgoal()
        if reverse_escape_added:
            self.get_logger().info(
                'inserted straight reverse escape for M20 rolling-turn '
                f'clearance: ({self._subgoals[0][0]:.2f}, '
                f'{self._subgoals[0][1]:.2f})'
            )
        self.get_logger().info(
            f'A* route published: {len(points)} poses, {length:.2f}m; '
            f'{len(self._subgoals)} SCAN subgoals, '
            f'{sum(self._continuous_transitions)} continuous handoffs'
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
