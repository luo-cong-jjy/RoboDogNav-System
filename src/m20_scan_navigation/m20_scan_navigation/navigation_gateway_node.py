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

"""Typed, floor-aware Action boundary around the SCAN route adapter."""

from collections import deque
import math
import threading
import time
from typing import Deque, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from m20_warehouse_interfaces.action import NavigateFloor
from m20_warehouse_interfaces.msg import FloorState, NavigationState
from m20_warehouse_interfaces.srv import ResetNavigation
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class NavigationGateway(Node):
    """Serialize goals and bind route completion to a map generation."""

    def __init__(self) -> None:
        super().__init__('m20_navigation_gateway')
        self.declare_parameter('action_name', '/m20/navigation/navigate')
        self.declare_parameter('default_timeout_sec', 120.0)
        self.declare_parameter('feedback_period_sec', 0.20)
        self.declare_parameter('position_tolerance', 0.20)
        self.declare_parameter('floor_hold_release_timeout_sec', 2.0)
        self.declare_parameter('use_grid_route', False)
        self.declare_parameter(
            'scan_reset_service', '/m20/navigation/reset'
        )
        self.declare_parameter(
            'route_reset_service', '/m20/navigation/route_reset'
        )
        self.declare_parameter(
            'direct_goal_topic', '/m20/navigation/scan_goal'
        )
        self.declare_parameter(
            'routed_goal_topic', '/m20/navigation/goal_pose'
        )
        self._default_timeout = max(
            1.0, float(self.get_parameter('default_timeout_sec').value)
        )
        self._feedback_period = max(
            0.05, float(self.get_parameter('feedback_period_sec').value)
        )
        self._position_tolerance = max(
            0.05, float(self.get_parameter('position_tolerance').value)
        )
        self._floor_hold_release_timeout = max(
            0.1,
            float(
                self.get_parameter(
                    'floor_hold_release_timeout_sec'
                ).value
            ),
        )
        self._use_grid_route = bool(
            self.get_parameter('use_grid_route').value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._floor_state: Optional[FloorState] = None
        self._odometry: Optional[Odometry] = None
        self._floor_hold = False
        self._route_state = ''
        self._route_update_count = 0
        self._route_events: Deque[Tuple[int, str]] = deque(maxlen=64)
        self._active_lock = threading.Lock()
        self._active = False
        qos = _latched_qos()

        self._goal_publisher = self.create_publisher(
            PoseStamped,
            (
                str(self.get_parameter('routed_goal_topic').value)
                if self._use_grid_route
                else str(self.get_parameter('direct_goal_topic').value)
            ),
            10,
        )
        self._state_publisher = self.create_publisher(
            NavigationState, '/m20/navigation/state', qos
        )
        self.create_subscription(
            FloorState,
            '/m20/map/state',
            self._floor_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry,
            '/m20/sim/body_pose',
            self._odom_callback,
            20,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/m20/navigation/global_route_state',
            self._route_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            '/m20/control/floor_switch_hold',
            self._hold_callback,
            qos,
            callback_group=self._callback_group,
        )
        self._scan_reset_client = self.create_client(
            ResetNavigation,
            str(self.get_parameter('scan_reset_service').value),
            callback_group=self._callback_group,
        )
        self._route_reset_client = self.create_client(
            ResetNavigation,
            str(self.get_parameter('route_reset_service').value),
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            NavigateFloor,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._publish_state('', '', 0, 'IDLE', False, math.inf, 'ready')
        mode = 'grid_route' if self._use_grid_route else 'native_scan'
        self.get_logger().info(
            f'typed navigation action gateway ready: mode={mode}'
        )

    def _floor_callback(self, message: FloorState) -> None:
        self._floor_state = message

    def _odom_callback(self, message: Odometry) -> None:
        self._odometry = message

    def _route_callback(self, message: String) -> None:
        self._route_state = message.data
        self._route_update_count += 1
        self._route_events.append(
            (self._route_update_count, message.data)
        )

    def _hold_callback(self, message: Bool) -> None:
        self._floor_hold = message.data

    def _goal_callback(self, goal) -> GoalResponse:
        pose = goal.target_pose.pose.position
        if (
            not goal.goal_id.strip()
            or not goal.floor_id.strip()
            or not all(math.isfinite(value) for value in (pose.x, pose.y))
        ):
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active:
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _distance(self, target: PoseStamped) -> float:
        if self._odometry is None:
            return math.inf
        position = self._odometry.pose.pose.position
        return math.hypot(
            position.x - target.pose.position.x,
            position.y - target.pose.position.y,
        )

    def _publish_state(
        self,
        goal_id: str,
        floor_id: str,
        generation: int,
        state_name: str,
        active: bool,
        distance: float,
        message: str,
    ) -> None:
        state = NavigationState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.goal_id = goal_id
        state.floor_id = floor_id
        state.map_generation = generation
        state.state = state_name
        state.active = active
        state.distance_remaining = (
            float(distance) if math.isfinite(distance) else -1.0
        )
        state.message = message
        self._state_publisher.publish(state)

    def _feedback(
        self,
        goal_handle,
        state_name: str,
        distance: float,
        message: str,
    ) -> None:
        feedback = NavigateFloor.Feedback()
        feedback.state = state_name
        feedback.distance_remaining = (
            float(distance) if math.isfinite(distance) else -1.0
        )
        feedback.route_update_count = self._route_update_count
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        request = goal_handle.request
        self._publish_state(
            request.goal_id,
            request.floor_id,
            request.map_generation,
            state_name,
            True,
            distance,
            message,
        )

    def _result(
        self,
        goal,
        success: bool,
        error_code: int,
        message: str,
    ):
        result = NavigateFloor.Result()
        result.success = success
        result.goal_id = goal.goal_id
        state = self._floor_state
        result.floor_id = state.floor_id if state is not None else ''
        result.map_generation = state.generation if state is not None else 0
        distance = self._distance(goal.target_pose)
        result.final_distance = (
            float(distance) if math.isfinite(distance) else -1.0
        )
        result.error_code = error_code
        result.message = message
        return result

    @staticmethod
    def _call_service(client, request, timeout: float):
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                return None
            time.sleep(0.02)
        return future.result()

    def _reset_navigation(self, floor_id: str, generation: int) -> bool:
        request = ResetNavigation.Request()
        request.floor_id = floor_id
        request.generation = generation
        route_ok = True
        if self._use_grid_route:
            route = self._call_service(
                self._route_reset_client, request, 2.0
            )
            route_ok = bool(route is not None and route.success)
        scan = self._call_service(
            self._scan_reset_client, request, 2.0
        )
        return bool(
            route_ok
            and scan is not None
            and scan.success
        )

    def _finish_state(
        self,
        goal,
        state_name: str,
        message: str,
    ) -> None:
        self._publish_state(
            goal.goal_id,
            goal.floor_id,
            goal.map_generation,
            state_name,
            False,
            self._distance(goal.target_pose),
            message,
        )

    def _execute(self, goal_handle):
        goal = goal_handle.request
        try:
            floor = self._floor_state
            if floor is None or not floor.ready or self._odometry is None:
                goal_handle.abort()
                self._finish_state(goal, 'MAP_NOT_READY', 'map or odom unavailable')
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_MAP_NOT_READY,
                    'map or odometry is not ready',
                )
            if (
                floor.floor_id != goal.floor_id
                or floor.generation != goal.map_generation
            ):
                goal_handle.abort()
                self._finish_state(
                    goal, 'FLOOR_MISMATCH', 'goal does not match active map'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_FLOOR_MISMATCH,
                    'goal floor/generation does not match active map',
                )
            # The floor-switch manager publishes hold=False immediately
            # before returning its Action result.  DDS delivery to this node
            # and to the mission executor is independent, so a new-floor goal
            # can legitimately arrive while this subscriber still holds the
            # preceding True sample.  Wait briefly for this node's own release
            # sample instead of rejecting that otherwise valid goal.
            release_deadline = (
                time.monotonic() + self._floor_hold_release_timeout
            )
            while (
                self._floor_hold
                and time.monotonic() < release_deadline
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._finish_state(
                        goal,
                        'CANCELLED',
                        'cancelled while waiting for floor release',
                    )
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_CANCELLED,
                        'cancelled while waiting for floor release',
                    )
                floor = self._floor_state
                if (
                    floor is None
                    or not floor.ready
                    or floor.floor_id != goal.floor_id
                    or floor.generation != goal.map_generation
                ):
                    break
                self._feedback(
                    goal_handle,
                    'WAITING_FOR_FLOOR_RELEASE',
                    self._distance(goal.target_pose),
                    'waiting for navigation gateway hold release',
                )
                time.sleep(0.02)
            if self._floor_hold:
                goal_handle.abort()
                self._finish_state(
                    goal, 'FLOOR_SWITCH_HOLD', 'floor switch is active'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_FLOOR_SWITCH,
                    'floor switch hold is active',
                )

            distance = self._distance(goal.target_pose)
            if distance <= self._position_tolerance:
                goal_handle.succeed()
                self._finish_state(goal, 'SUCCEEDED', 'already at goal')
                return self._result(
                    goal,
                    True,
                    NavigateFloor.Result.ERROR_NONE,
                    'already within goal tolerance',
                )

            if not self._reset_navigation(
                goal.floor_id, goal.map_generation
            ):
                goal_handle.abort()
                self._finish_state(
                    goal, 'RESET_FAILED', 'navigation reset failed'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_RESET_FAILED,
                    'navigation reset failed',
                )
            readiness_deadline = time.monotonic() + 10.0
            while self._use_grid_route and self._route_state != 'READY':
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._finish_state(
                        goal, 'CANCELLED', 'cancelled before route start'
                    )
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_CANCELLED,
                        'cancelled before route start',
                    )
                floor = self._floor_state
                if (
                    time.monotonic() >= readiness_deadline
                    or floor is None
                    or not floor.ready
                    or floor.floor_id != goal.floor_id
                    or floor.generation != goal.map_generation
                    or self._floor_hold
                ):
                    goal_handle.abort()
                    self._finish_state(
                        goal,
                        'MAP_NOT_READY',
                        'route adapter did not become ready',
                    )
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_MAP_NOT_READY,
                        'route adapter did not become ready',
                    )
                self._feedback(
                    goal_handle,
                    'WAITING_FOR_ROUTE',
                    distance,
                    self._route_state or 'waiting for route adapter',
                )
                time.sleep(0.05)

            baseline = self._route_update_count
            processed_count = baseline
            target = goal.target_pose
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = target.header.frame_id or 'map'
            self._goal_publisher.publish(target)
            self._feedback(
                goal_handle,
                'PLANNING',
                distance,
                (
                    'goal sent to route adapter'
                    if self._use_grid_route
                    else 'goal sent directly to SCAN'
                ),
            )
            route_started = not self._use_grid_route
            timeout = (
                float(goal.timeout_sec)
                if goal.timeout_sec > 0.0
                else self._default_timeout
            )
            deadline = time.monotonic() + timeout

            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    reset_ok = self._reset_navigation(
                        goal.floor_id, goal.map_generation
                    )
                    goal_handle.canceled()
                    message = (
                        'navigation cancelled and reset'
                        if reset_ok
                        else 'navigation cancelled; reset confirmation failed'
                    )
                    self._finish_state(goal, 'CANCELLED', message)
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_CANCELLED
                        if reset_ok
                        else NavigateFloor.Result.ERROR_RESET_FAILED,
                        message,
                    )

                floor = self._floor_state
                if (
                    floor is None
                    or not floor.ready
                    or floor.floor_id != goal.floor_id
                    or floor.generation != goal.map_generation
                    or self._floor_hold
                ):
                    goal_handle.abort()
                    self._finish_state(
                        goal,
                        'FLOOR_SWITCH',
                        'active floor changed during navigation',
                    )
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_FLOOR_SWITCH,
                        'active floor changed or entered hold',
                    )

                distance = self._distance(goal.target_pose)
                if (
                    not self._use_grid_route
                    and distance <= self._position_tolerance
                ):
                    goal_handle.succeed()
                    self._finish_state(
                        goal, 'SUCCEEDED', 'native SCAN goal reached'
                    )
                    return self._result(
                        goal,
                        True,
                        NavigateFloor.Result.ERROR_NONE,
                        'native SCAN goal reached',
                    )
                events = [
                    event
                    for event in list(self._route_events)
                    if event[0] > processed_count
                ]
                for event_count, route_state in (
                    events if self._use_grid_route else []
                ):
                    processed_count = max(processed_count, event_count)
                    if route_state == 'TRACKING_ROUTE':
                        route_started = True
                    elif route_state == 'NO_ROUTE':
                        self._reset_navigation(
                            goal.floor_id, goal.map_generation
                        )
                        goal_handle.abort()
                        self._finish_state(goal, 'NO_ROUTE', 'route planning failed')
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_NO_ROUTE,
                            'no collision-free route',
                        )
                    elif (
                        route_state == 'GOAL_REACHED'
                        and route_started
                        and distance <= self._position_tolerance
                    ):
                        goal_handle.succeed()
                        self._finish_state(
                            goal, 'SUCCEEDED', 'goal reached'
                        )
                        return self._result(
                            goal,
                            True,
                            NavigateFloor.Result.ERROR_NONE,
                            'goal reached',
                        )
                    elif route_state.startswith('REJECTED'):
                        goal_handle.abort()
                        self._finish_state(
                            goal, 'REJECTED', route_state
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_INVALID_REQUEST,
                            route_state,
                        )
                self._feedback(
                    goal_handle,
                    'TRACKING' if route_started else 'PLANNING',
                    distance,
                    (
                        self._route_state or 'waiting for route state'
                        if self._use_grid_route
                        else 'native SCAN trajectory tracking'
                    ),
                )
                time.sleep(self._feedback_period)

            reset_ok = self._reset_navigation(
                goal.floor_id, goal.map_generation
            )
            goal_handle.abort()
            message = (
                'navigation timed out and reset'
                if reset_ok
                else 'navigation timed out; reset confirmation failed'
            )
            self._finish_state(goal, 'TIMEOUT', message)
            return self._result(
                goal,
                False,
                NavigateFloor.Result.ERROR_TIMEOUT
                if reset_ok
                else NavigateFloor.Result.ERROR_RESET_FAILED,
                message,
            )
        except Exception as error:
            self.get_logger().error(f'navigation gateway failed: {error}')
            goal_handle.abort()
            self._finish_state(goal, 'INTERNAL_ERROR', str(error))
            return self._result(
                goal,
                False,
                NavigateFloor.Result.ERROR_INTERNAL,
                str(error),
            )
        finally:
            with self._active_lock:
                self._active = False

    def destroy_node(self) -> None:
        self._action_server.destroy()
        super().destroy_node()


def main() -> None:
    """Run navigation gateway with concurrent state and service callbacks."""
    rclpy.init()
    node = NavigationGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
