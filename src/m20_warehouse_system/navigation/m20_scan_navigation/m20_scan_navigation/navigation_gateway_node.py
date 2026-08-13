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

"""Floor-aware Action and managed RViz boundary for native SCAN goals."""

from copy import deepcopy
import math
import threading
import time
from typing import Optional

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

from .recovery_replan import (
    collision_replan_available,
    recovery_replan_required,
)


def _latched_qos() -> QoSProfile:
    """Return the reliable transient QoS used by system states."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def guard_allows_motion(
    required: bool,
    collision_stop: bool,
    diagnostic: str,
) -> bool:
    """Apply the execution-profile contract to guard readiness."""
    return bool(
        not required
        or (not collision_stop and diagnostic == 'CLEAR')
    )


class NavigationGateway(Node):
    """Serialize native SCAN goals and bind them to one map generation."""

    def __init__(self) -> None:
        """Create the typed and managed-RViz navigation boundaries."""
        super().__init__('m20_navigation_gateway')
        self.declare_parameter('action_name', '/m20/navigation/navigate')
        self.declare_parameter('default_timeout_sec', 120.0)
        self.declare_parameter('feedback_period_sec', 0.20)
        self.declare_parameter('position_tolerance', 0.20)
        self.declare_parameter('floor_hold_release_timeout_sec', 2.0)
        self.declare_parameter('navigation_reset_timeout_sec', 10.0)
        self.declare_parameter('collision_guard_required', True)
        self.declare_parameter('collision_replan_enabled', True)
        self.declare_parameter('collision_replan_max_attempts', 2)
        self.declare_parameter('collision_replan_rearm_timeout_sec', 5.0)
        self.declare_parameter(
            'collision_stop_topic', '/m20/control/collision_stop'
        )
        self.declare_parameter(
            'collision_diagnostic_topic',
            '/m20/control/collision_guard_diagnostic',
        )
        self.declare_parameter(
            'scan_reset_service', '/m20/navigation/reset'
        )
        self.declare_parameter(
            'direct_goal_topic', '/m20/navigation/scan_goal'
        )
        self.declare_parameter('manual_goal_monitor_enabled', False)
        self.declare_parameter(
            'manual_goal_topic', '/move_base_simple/goal'
        )
        self.declare_parameter(
            'manual_state_topic', '/m20/navigation/manual_state'
        )
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        self.declare_parameter('manual_timeout_sec', 300.0)
        self.declare_parameter(
            'terminal_orientation_mode', 'position_only'
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
        self._navigation_reset_timeout = max(
            0.5,
            float(
                self.get_parameter(
                    'navigation_reset_timeout_sec'
                ).value
            ),
        )
        self._collision_guard_required = bool(
            self.get_parameter('collision_guard_required').value
        )
        self._collision_replan_enabled = bool(
            self.get_parameter('collision_replan_enabled').value
        )
        self._collision_replan_max_attempts = max(
            0,
            int(
                self.get_parameter(
                    'collision_replan_max_attempts'
                ).value
            ),
        )
        self._collision_replan_rearm_timeout = max(
            0.1,
            float(
                self.get_parameter(
                    'collision_replan_rearm_timeout_sec'
                ).value
            ),
        )
        self._manual_goal_monitor_enabled = bool(
            self.get_parameter('manual_goal_monitor_enabled').value
        )
        self._manual_timeout = max(
            1.0, float(self.get_parameter('manual_timeout_sec').value)
        )
        self._terminal_orientation_mode = str(
            self.get_parameter('terminal_orientation_mode').value
        ).strip().lower()
        if self._terminal_orientation_mode != 'position_only':
            raise ValueError(
                'terminal_orientation_mode must be position_only; '
                'strict yaw requires a turn-pocket planner'
            )

        self._callback_group = ReentrantCallbackGroup()
        self._floor_state: Optional[FloorState] = None
        self._odometry: Optional[Odometry] = None
        self._floor_hold = False
        # Safety profiles fail closed until their online guard publishes
        # CLEAR. Native SCAN has no such node and is explicitly permitted by
        # the launch profile instead of waiting for a nonexistent publisher.
        self._collision_stop = self._collision_guard_required
        self._collision_diagnostic = (
            '' if self._collision_guard_required else 'CLEAR'
        )
        self._plan_update_count = 0
        self._active_lock = threading.Lock()
        self._active = False
        self._manual_lock = threading.Lock()
        self._manual_serial = 0
        self._manual_target: Optional[PoseStamped] = None
        self._manual_phase = 'IDLE'
        self._manual_started_at = 0.0
        self._manual_rearm_deadline = 0.0
        self._manual_replan_attempts = 0
        qos = _latched_qos()

        self._goal_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('direct_goal_topic').value),
            10,
        )
        self._state_publisher = self.create_publisher(
            NavigationState, '/m20/navigation/state', qos
        )
        self._manual_state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('manual_state_topic').value),
            qos,
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
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
            callback_group=self._callback_group,
        )
        if self._manual_goal_monitor_enabled:
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter('manual_goal_topic').value),
                self._manual_goal_callback,
                10,
                callback_group=self._callback_group,
            )
        self.create_subscription(
            Bool,
            '/m20/control/floor_switch_hold',
            self._hold_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('collision_stop_topic').value),
            self._collision_stop_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            str(
                self.get_parameter(
                    'collision_diagnostic_topic'
                ).value
            ),
            self._collision_diagnostic_callback,
            qos,
            callback_group=self._callback_group,
        )
        self._scan_reset_client = self.create_client(
            ResetNavigation,
            str(self.get_parameter('scan_reset_service').value),
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
        if self._manual_goal_monitor_enabled:
            self.create_timer(
                0.05,
                self._manual_tick,
                callback_group=self._callback_group,
            )
        self._publish_state('', '', 0, 'IDLE', False, math.inf, 'ready')
        self._publish_manual_state('IDLE', 'ready')
        self.get_logger().info(
            'typed navigation gateway ready: mode=native_scan; '
            f'managed RViz goals={self._manual_goal_monitor_enabled}; '
            f'collision_guard_required={self._collision_guard_required}; '
            f'terminal_orientation={self._terminal_orientation_mode}'
        )

    def _floor_callback(self, message: FloorState) -> None:
        self._floor_state = message

    def _odom_callback(self, message: Odometry) -> None:
        self._odometry = message

    def _hold_callback(self, message: Bool) -> None:
        self._floor_hold = bool(message.data)

    def _collision_stop_callback(self, message: Bool) -> None:
        self._collision_stop = bool(message.data)

    def _collision_diagnostic_callback(self, message: String) -> None:
        self._collision_diagnostic = str(message.data)

    def _diagnostic_requires_replan(self) -> bool:
        """Return true only after one collision recovery episode ends."""
        return bool(
            self._collision_guard_required
            and recovery_replan_required(self._collision_diagnostic)
        )

    def _guard_is_clear(self) -> bool:
        """Require both guard outputs so stale or missing clouds fail closed."""
        return guard_allows_motion(
            self._collision_guard_required,
            self._collision_stop,
            self._collision_diagnostic,
        )

    def _publish_goal(self, target: PoseStamped) -> None:
        """Publish one unchanged position goal to native SCAN."""
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = target.header.frame_id or 'map'
        self._plan_update_count += 1
        self._goal_publisher.publish(target)

    def _publish_manual_state(self, state: str, detail: str = '') -> None:
        """Publish a latched state for RViz-originated free navigation."""
        text = state if not detail else f'{state}; {detail}'
        self._manual_state_publisher.publish(String(data=text))
        self.get_logger().info(f'managed RViz navigation: {text}')

    def _clear_manual_locked(self) -> None:
        """Clear manual state while the caller owns the manual lock."""
        self._manual_target = None
        self._manual_phase = 'IDLE'
        self._manual_started_at = 0.0
        self._manual_rearm_deadline = 0.0
        self._manual_replan_attempts = 0

    def _manual_goal_callback(self, message: PoseStamped) -> None:
        """Reset SCAN, then relay an RViz goal when the guard is clear."""
        point = message.pose.position
        if not all(math.isfinite(value) for value in (point.x, point.y)):
            self._publish_manual_state(
                'REJECTED_INVALID_GOAL', 'target is not finite'
            )
            return
        with self._active_lock:
            if self._active:
                self._publish_manual_state(
                    'REJECTED_ACTION_ACTIVE',
                    'typed navigation owns the planner',
                )
                return
        floor = self._floor_state
        if (
            floor is None
            or not floor.ready
            or self._odometry is None
            or self._floor_hold
        ):
            self._publish_manual_state(
                'REJECTED_NOT_READY',
                'map, odometry, or floor-switch hold is not ready',
            )
            return

        target = deepcopy(message)
        target.header.frame_id = target.header.frame_id or 'map'
        with self._manual_lock:
            self._manual_serial += 1
            serial = self._manual_serial
            self._manual_target = target
            self._manual_phase = 'RESETTING_NATIVE'
            self._manual_started_at = time.monotonic()
            self._manual_replan_attempts = 0
        self._publish_manual_state(
            'RESETTING_NATIVE',
            f'target=({point.x:.2f},{point.y:.2f})',
        )
        reset_ok = self._reset_navigation(
            floor.floor_id, floor.generation
        )
        with self._manual_lock:
            if serial != self._manual_serial:
                return
            if not reset_ok:
                self._clear_manual_locked()
                failed = True
            else:
                self._manual_phase = 'WAITING_FOR_RECOVERY_REARM'
                self._manual_rearm_deadline = (
                    time.monotonic()
                    + self._collision_replan_rearm_timeout
                )
                failed = False
        if failed:
            self._publish_manual_state(
                'RESET_FAILED', 'could not prepare native SCAN'
            )
        else:
            self._publish_manual_state(
                'WAITING_FOR_RECOVERY_REARM',
                'waiting for online point-cloud guard CLEAR',
            )

    def _finish_manual(
        self,
        serial: int,
        state: str,
        detail: str,
        *,
        reset: bool = True,
    ) -> None:
        """Stop one managed manual goal and publish its terminal state."""
        with self._manual_lock:
            if (
                serial != self._manual_serial
                or self._manual_target is None
            ):
                return
            self._manual_phase = 'FINAL_RESETTING'
        floor = self._floor_state
        reset_ok = True
        if reset and floor is not None:
            reset_ok = self._reset_navigation(
                floor.floor_id, floor.generation
            )
        with self._manual_lock:
            if serial != self._manual_serial:
                return
            self._clear_manual_locked()
        suffix = detail
        if reset and not reset_ok:
            suffix += '; navigation reset confirmation failed'
        self._publish_manual_state(state, suffix)

    def _manual_tick(self) -> None:
        """Advance one non-blocking managed RViz navigation state machine."""
        with self._manual_lock:
            target = self._manual_target
            if target is None:
                return
            serial = self._manual_serial
            phase = self._manual_phase
            started_at = self._manual_started_at
            rearm_deadline = self._manual_rearm_deadline
            attempts = self._manual_replan_attempts
        if phase in {
            'RESETTING_NATIVE',
            'RESETTING_RECOVERY',
            'FINAL_RESETTING',
        }:
            return

        floor = self._floor_state
        if (
            floor is None
            or not floor.ready
            or self._floor_hold
            or self._odometry is None
        ):
            self._finish_manual(
                serial,
                'ABORTED_FLOOR_STATE',
                'active map, odometry, or hold changed',
            )
            return
        now = time.monotonic()
        if now - started_at > self._manual_timeout:
            self._finish_manual(
                serial,
                'TIMEOUT',
                f'exceeded {self._manual_timeout:.1f}s',
            )
            return

        if phase == 'WAITING_FOR_RECOVERY_REARM':
            if self._guard_is_clear():
                with self._manual_lock:
                    if serial != self._manual_serial:
                        return
                    self._manual_phase = 'TRACKING_NATIVE'
                self._publish_goal(target)
                self._publish_manual_state(
                    'TRACKING_NATIVE', 'goal relayed unchanged to SCAN'
                )
                return
            if now >= rearm_deadline:
                self._finish_manual(
                    serial,
                    'COLLISION_REARM_TIMEOUT',
                    'online point-cloud guard did not return CLEAR',
                )
            return

        if (
            self._collision_replan_enabled
            and self._diagnostic_requires_replan()
        ):
            if not collision_replan_available(
                attempts, self._collision_replan_max_attempts
            ):
                self._finish_manual(
                    serial,
                    'COLLISION_REPLAN_EXHAUSTED',
                    'bounded native SCAN replan attempts exhausted',
                )
                return
            with self._manual_lock:
                if serial != self._manual_serial:
                    return
                self._manual_phase = 'RESETTING_RECOVERY'
                self._manual_replan_attempts += 1
                attempt = self._manual_replan_attempts
            self._publish_manual_state(
                'RESETTING_RECOVERY',
                f'attempt={attempt}/{self._collision_replan_max_attempts}',
            )
            reset_ok = self._reset_navigation(
                floor.floor_id, floor.generation
            )
            with self._manual_lock:
                if serial != self._manual_serial:
                    return
                if not reset_ok:
                    self._clear_manual_locked()
                    failed = True
                else:
                    self._manual_phase = 'WAITING_FOR_RECOVERY_REARM'
                    self._manual_rearm_deadline = (
                        time.monotonic()
                        + self._collision_replan_rearm_timeout
                    )
                    failed = False
            if failed:
                self._publish_manual_state(
                    'RESET_FAILED', 'collision-triggered SCAN reset failed'
                )
            else:
                self._publish_manual_state(
                    'WAITING_FOR_RECOVERY_REARM',
                    'old trajectory stopped before native SCAN replan',
                )
            return

        distance = self._distance(target)
        if distance <= self._position_tolerance:
            self._finish_manual(
                serial,
                'SUCCEEDED',
                f'final_distance={distance:.3f}m',
            )

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
            with self._manual_lock:
                if self._manual_target is not None:
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
        # The interface field name is frozen for compatibility. In the
        # native-only implementation it counts SCAN goal publications.
        feedback.route_update_count = self._plan_update_count
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
        """Reset only the native SCAN state machine."""
        request = ResetNavigation.Request()
        request.floor_id = floor_id
        request.generation = generation
        scan = self._call_service(
            self._scan_reset_client,
            request,
            self._navigation_reset_timeout,
        )
        return bool(scan is not None and scan.success)

    def _finish_state(self, goal, state_name: str, message: str) -> None:
        self._publish_state(
            goal.goal_id,
            goal.floor_id,
            goal.map_generation,
            state_name,
            False,
            self._distance(goal.target_pose),
            message,
        )

    def _floor_matches(self, goal) -> bool:
        floor = self._floor_state
        return bool(
            floor is not None
            and floor.ready
            and floor.floor_id == goal.floor_id
            and floor.generation == goal.map_generation
        )

    def _wait_for_guard_clear(self, goal_handle, deadline: float) -> bool:
        """Wait for a fresh online-cloud safety decision before moving."""
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested or self._floor_hold:
                return False
            if not self._floor_matches(goal_handle.request):
                return False
            if self._guard_is_clear():
                return True
            self._feedback(
                goal_handle,
                'WAITING_FOR_RECOVERY_REARM',
                self._distance(goal_handle.request.target_pose),
                'waiting for online point-cloud guard CLEAR',
            )
            time.sleep(0.05)
        return False

    def _execute(self, goal_handle):
        goal = goal_handle.request
        try:
            floor = self._floor_state
            if floor is None or not floor.ready or self._odometry is None:
                goal_handle.abort()
                self._finish_state(
                    goal, 'MAP_NOT_READY', 'map or odometry unavailable'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_MAP_NOT_READY,
                    'map or odometry is not ready',
                )
            if not self._floor_matches(goal):
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

            # DDS may deliver a new-floor Action before this subscriber sees
            # the preceding hold=False sample. Absorb only that bounded race.
            release_deadline = (
                time.monotonic() + self._floor_hold_release_timeout
            )
            while self._floor_hold and time.monotonic() < release_deadline:
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
                if not self._floor_matches(goal):
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

            timeout = (
                float(goal.timeout_sec)
                if goal.timeout_sec > 0.0
                else self._default_timeout
            )
            deadline = time.monotonic() + timeout
            if not self._reset_navigation(
                goal.floor_id, goal.map_generation
            ):
                goal_handle.abort()
                self._finish_state(
                    goal, 'RESET_FAILED', 'native SCAN reset failed'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_RESET_FAILED,
                    'native SCAN reset failed',
                )
            rearm_deadline = min(
                deadline,
                time.monotonic() + self._collision_replan_rearm_timeout,
            )
            if not self._wait_for_guard_clear(goal_handle, rearm_deadline):
                goal_handle.abort()
                self._finish_state(
                    goal,
                    'COLLISION_REARM_TIMEOUT',
                    'online point-cloud guard did not become clear',
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_NO_ROUTE,
                    'online point-cloud guard did not become clear',
                )

            target = deepcopy(goal.target_pose)
            self._publish_goal(target)
            self._feedback(
                goal_handle,
                'PLANNING',
                distance,
                'goal sent directly to native SCAN',
            )
            collision_replan_attempts = 0

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

                if not self._floor_matches(goal) or self._floor_hold:
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

                if (
                    self._collision_replan_enabled
                    and self._diagnostic_requires_replan()
                ):
                    if not collision_replan_available(
                        collision_replan_attempts,
                        self._collision_replan_max_attempts,
                    ):
                        self._reset_navigation(
                            goal.floor_id, goal.map_generation
                        )
                        goal_handle.abort()
                        message = (
                            'bounded collision recovery and native SCAN '
                            'replan budget exhausted'
                        )
                        self._finish_state(
                            goal, 'COLLISION_REPLAN_EXHAUSTED', message
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_NO_ROUTE,
                            message,
                        )

                    collision_replan_attempts += 1
                    self._feedback(
                        goal_handle,
                        'RECOVERY_REPLAN',
                        self._distance(goal.target_pose),
                        'collision recovery ended; resetting native SCAN '
                        f'({collision_replan_attempts}/'
                        f'{self._collision_replan_max_attempts})',
                    )
                    if not self._reset_navigation(
                        goal.floor_id, goal.map_generation
                    ):
                        goal_handle.abort()
                        message = 'collision-triggered SCAN reset failed'
                        self._finish_state(
                            goal, 'RESET_FAILED', message
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_RESET_FAILED,
                            message,
                        )
                    rearm_deadline = min(
                        deadline,
                        time.monotonic()
                        + self._collision_replan_rearm_timeout,
                    )
                    if not self._wait_for_guard_clear(
                        goal_handle, rearm_deadline
                    ):
                        goal_handle.abort()
                        message = (
                            'online point-cloud guard did not rearm before '
                            'native SCAN replan'
                        )
                        self._finish_state(
                            goal, 'COLLISION_REARM_TIMEOUT', message
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_NO_ROUTE,
                            message,
                        )
                    self._publish_goal(target)
                    self._feedback(
                        goal_handle,
                        'REPLANNING',
                        self._distance(goal.target_pose),
                        'goal republished to native SCAN after bounded '
                        'recovery',
                    )
                    continue

                distance = self._distance(goal.target_pose)
                if distance <= self._position_tolerance:
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
                self._feedback(
                    goal_handle,
                    'TRACKING',
                    distance,
                    'native SCAN trajectory tracking',
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
        """Destroy the Action server before destroying the node."""
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """Run the native SCAN gateway with concurrent service callbacks."""
    rclpy.init(args=args)
    node = NavigationGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # A launch SIGINT can shut the default context before this finally
        # block runs. Humble raises on a second shutdown; Foxy deployments
        # use the same guard for clean cross-distribution teardown.
        if rclpy.ok():
            rclpy.shutdown()
