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

"""Fail-closed action server coordinating a simulated elevator transfer."""

import math
from pathlib import Path
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from m20_warehouse_interfaces.action import ExecuteFloorTransfer, SwitchFloor
from m20_warehouse_interfaces.msg import FloorState, LocalSensingState
from m20_warehouse_interfaces.srv import (
    ResetNavigation,
    SetSimulationPose,
    SwitchMap,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
import yaml

from .floor_switch_policy import pose_error, resolve_transition


class _SwitchCancelled(RuntimeError):
    """Internal signal used to unwind an action while retaining safety hold."""


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


class FloorSwitchManager(Node):
    """Coordinate stop, reset, map commit, relocation, and sensing readiness."""

    def __init__(self) -> None:
        super().__init__('m20_floor_switch_manager')
        self.declare_parameter('config_path', '')
        self.declare_parameter('action_name', '/m20/floor_switch')
        self.declare_parameter(
            'navigation_reset_service', '/m20/navigation/reset'
        )
        self.declare_parameter('map_switch_service', '/m20/map/switch')
        self.declare_parameter('set_pose_service', '/m20/sim/set_pose')
        self.declare_parameter('body_pose_topic', '/m20/sim/body_pose')
        config_path = Path(str(self.get_parameter('config_path').value))
        if not config_path.is_file():
            raise ValueError(f'config_path is not a file: {config_path}')
        with config_path.open('r', encoding='utf-8') as stream:
            self._config = yaml.safe_load(stream)

        transaction = self._config['map_switch_transaction']
        self._linear_stop_threshold = float(
            transaction['stop_linear_threshold']
        )
        self._angular_stop_threshold = float(
            transaction['stop_angular_threshold']
        )
        self._reset_timeout = float(
            transaction['reset_navigation_timeout_sec']
        )
        self._map_timeout = float(transaction['load_map_timeout_sec'])
        self._pose_timeout = float(transaction['relocate_timeout_sec'])
        self._readiness_timeout = float(
            transaction['readiness_timeout_sec']
        )
        self._minimum_fresh_clouds = int(
            transaction['minimum_fresh_local_cloud_count']
        )
        self._callback_group = ReentrantCallbackGroup()
        self._floor_state: Optional[FloorState] = None
        self._sensing_state: Optional[LocalSensingState] = None
        self._odometry: Optional[Odometry] = None
        self._safe_command: Optional[Twist] = None
        self._goal_lock = threading.Lock()
        self._switch_active = False
        self._hold_asserted = False
        self._external_client_lock = threading.Lock()
        self._external_clients = {}
        qos = _latched_qos()

        self._hold_publisher = self.create_publisher(
            Bool, '/m20/control/floor_switch_hold', qos
        )
        self._state_publisher = self.create_publisher(
            String, '/m20/floor_switch/state', qos
        )
        self.create_subscription(
            FloorState,
            '/m20/map/state',
            self._floor_state_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            LocalSensingState,
            '/m20/sensing/state',
            self._sensing_state_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('body_pose_topic').value),
            self._odometry_callback,
            20,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Twist,
            '/m20/control/cmd_vel_safe',
            self._safe_command_callback,
            20,
            callback_group=self._callback_group,
        )
        self._reset_client = self.create_client(
            ResetNavigation,
            str(self.get_parameter('navigation_reset_service').value),
            callback_group=self._callback_group,
        )
        self._map_client = self.create_client(
            SwitchMap,
            str(self.get_parameter('map_switch_service').value),
            callback_group=self._callback_group,
        )
        self._set_pose_service_name = str(
            self.get_parameter('set_pose_service').value
        )
        self._pose_client = None
        self._action_server = ActionServer(
            self,
            SwitchFloor,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._publish_hold(False)
        self._state_publisher.publish(String(data='READY_FOR_REQUEST'))
        switch_policy = self._config.get('floor_switch', {})
        transfer_mode = str(
            switch_policy.get('transfer_adapter', 'timed_hold')
        )
        pose_handoff = str(
            switch_policy.get(
                'pose_handoff',
                'set_simulation_pose'
                if self._config.get('simulation', {}).get(
                    'teleport_on_floor_switch', False
                )
                else 'preserve',
            )
        )
        self.get_logger().info(
            'floor switch action manager ready; '
            f'transfer_adapter={transfer_mode}, pose_handoff={pose_handoff}'
        )

    def _floor_state_callback(self, message: FloorState) -> None:
        self._floor_state = message

    def _sensing_state_callback(self, message: LocalSensingState) -> None:
        self._sensing_state = message

    def _odometry_callback(self, message: Odometry) -> None:
        self._odometry = message

    def _safe_command_callback(self, message: Twist) -> None:
        self._safe_command = message

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not (
            goal_request.source_floor.strip()
            and goal_request.target_floor.strip()
            and goal_request.elevator_id.strip()
        ):
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._switch_active:
                return GoalResponse.REJECT
            self._switch_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_hold(self, asserted: bool) -> None:
        self._hold_asserted = asserted
        self._hold_publisher.publish(Bool(data=asserted))

    def _feedback(
        self,
        goal_handle,
        phase: str,
        message: str,
        generation: Optional[int] = None,
    ) -> None:
        feedback = SwitchFloor.Feedback()
        feedback.phase = phase
        feedback.map_generation = (
            int(generation)
            if generation is not None
            else int(self._floor_state.generation)
            if self._floor_state is not None
            else 0
        )
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        self._state_publisher.publish(
            String(data=f'{phase}: {message}')
        )
        if self._hold_asserted:
            self._hold_publisher.publish(Bool(data=True))

    def _result(self, success: bool, error_code: int, message: str):
        result = SwitchFloor.Result()
        result.success = success
        result.active_floor = (
            self._floor_state.floor_id
            if self._floor_state is not None
            else ''
        )
        result.map_generation = (
            self._floor_state.generation
            if self._floor_state is not None
            else 0
        )
        result.error_code = error_code
        result.message = message
        return result

    @staticmethod
    def _check_cancel(goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            raise _SwitchCancelled('floor switch cancelled by client')

    def _call_service(self, client, request, timeout: float):
        """Call a service while other executor threads continue callbacks."""
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

    def _wait_stopped(self, goal_handle, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        consecutive = 0
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            odom = self._odometry
            command = self._safe_command
            if odom is not None and command is not None:
                twist = odom.twist.twist
                odom_linear = math.sqrt(
                    twist.linear.x**2
                    + twist.linear.y**2
                    + twist.linear.z**2
                )
                command_linear = math.hypot(
                    command.linear.x, command.linear.y
                )
                stopped = (
                    odom_linear <= self._linear_stop_threshold
                    and abs(twist.angular.z)
                    <= self._angular_stop_threshold
                    and command_linear <= self._linear_stop_threshold
                    and abs(command.angular.z)
                    <= self._angular_stop_threshold
                )
                consecutive = consecutive + 1 if stopped else 0
                if consecutive >= 3:
                    return True
            time.sleep(0.05)
        return False

    def _delay_with_cancel(self, goal_handle, duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            time.sleep(min(0.05, deadline - time.monotonic()))

    def _wait_sensing(
        self,
        goal_handle,
        floor_id: str,
        generation: int,
        minimum_count: int,
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            state = self._sensing_state
            if (
                state is not None
                and state.ready
                and state.floor_id == floor_id
                and state.generation == generation
                and state.fresh_cloud_count >= minimum_count
            ):
                return True
            time.sleep(0.05)
        return False

    def _pose_at_target(self, plan):
        """Return whether current odometry satisfies the release contract."""
        odom = self._odometry
        if odom is None:
            return False, float('inf'), float('inf')
        current_pose = (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            _yaw_from_odometry(odom),
        )
        distance, yaw_error = pose_error(current_pose, plan.target_release)
        return (
            distance <= plan.trigger_tolerance_xy
            and yaw_error <= plan.trigger_tolerance_yaw,
            distance,
            yaw_error,
        )

    def _establish_target_pose(self, goal_handle, plan, generation: int):
        """Apply the configured post-map pose handoff policy."""
        if plan.pose_handoff == 'none':
            return True, 'pose handoff delegated to the transfer adapter'

        if plan.pose_handoff == 'preserve':
            ready, distance, yaw_error = self._pose_at_target(plan)
            if not ready:
                return (
                    False,
                    f'preserved pose differs from target release by '
                    f'{distance:.3f}m/{yaw_error:.3f}rad',
                )
            return True, 'world pose preserved at the shared gateway'

        if plan.pose_handoff == 'wait_for_target':
            deadline = time.monotonic() + self._pose_timeout
            last_distance = float('inf')
            last_yaw_error = float('inf')
            while time.monotonic() < deadline:
                self._check_cancel(goal_handle)
                ready, last_distance, last_yaw_error = self._pose_at_target(
                    plan
                )
                if ready:
                    return True, 'target release pose confirmed'
                time.sleep(0.05)
            return (
                False,
                f'target pose was not confirmed; error '
                f'{last_distance:.3f}m/{last_yaw_error:.3f}rad',
            )

        if plan.pose_handoff != 'set_simulation_pose':
            return False, f'unsupported pose handoff {plan.pose_handoff!r}'

        if self._pose_client is None:
            self._pose_client = self.create_client(
                SetSimulationPose,
                self._set_pose_service_name,
                callback_group=self._callback_group,
            )

        pose_request = SetSimulationPose.Request()
        (
            pose_request.x,
            pose_request.y,
            pose_request.yaw,
        ) = plan.target_release
        pose_request.floor_id = plan.target_floor
        pose_request.generation = generation
        pose_response = self._call_service(
            self._pose_client, pose_request, self._pose_timeout
        )
        if pose_response is None or not pose_response.success:
            return False, 'simulation pose handoff failed'
        return True, 'simulation pose handoff complete'

    def _external_client(self, action_name: str):
        """Create one reusable Action client per configured endpoint."""
        with self._external_client_lock:
            client = self._external_clients.get(action_name)
            if client is None:
                client = ActionClient(
                    self,
                    ExecuteFloorTransfer,
                    action_name,
                    callback_group=self._callback_group,
                )
                self._external_clients[action_name] = client
            return client

    def _execute_external_transfer(self, goal_handle, plan, generation: int):
        client = self._external_client(plan.external_action_name)
        deadline = time.monotonic() + plan.transfer_timeout_sec
        while not client.server_is_ready():
            self._check_cancel(goal_handle)
            if time.monotonic() >= deadline:
                return False, 'external transfer action is unavailable'
            time.sleep(0.05)

        request = ExecuteFloorTransfer.Goal()
        request.connector_id = plan.connector_id
        request.source_floor = plan.source_floor
        request.target_floor = plan.target_floor
        request.source_generation = generation

        def feedback_callback(message) -> None:
            feedback = message.feedback
            phase = str(feedback.phase).strip() or 'RUNNING'
            self._feedback(
                goal_handle,
                f'TRANSFER_{phase}',
                str(feedback.message),
                generation,
            )

        send_future = client.send_goal_async(
            request, feedback_callback=feedback_callback
        )
        while not send_future.done():
            self._check_cancel(goal_handle)
            if time.monotonic() >= deadline:
                send_future.cancel()
                return False, 'external transfer goal timed out'
            time.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, 'external transfer goal was rejected'

        result_future = handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                self._check_cancel(goal_handle)
            if time.monotonic() >= deadline:
                handle.cancel_goal_async()
                return False, 'external transfer execution timed out'
            time.sleep(0.05)
        wrapped = result_future.result()
        if wrapped is None or not wrapped.result.success:
            message = (
                wrapped.result.message
                if wrapped is not None
                else 'external transfer returned no result'
            )
            return False, message
        return True, wrapped.result.message or 'external transfer complete'

    def _execute_transfer(self, goal_handle, plan, generation: int):
        """Run the selected transport adapter without touching map state."""
        if plan.transfer_adapter == 'timed_hold':
            self._delay_with_cancel(goal_handle, plan.transition_delay_sec)
            return True, (
                f'timed hold complete ({plan.transition_delay_sec:.2f}s)'
            )
        if plan.transfer_adapter == 'external_action':
            return self._execute_external_transfer(
                goal_handle, plan, generation
            )
        return False, f'unsupported transfer adapter {plan.transfer_adapter!r}'

    def _execute(self, goal_handle):
        goal = goal_handle.request
        transaction_started = False
        try:
            try:
                plan = resolve_transition(
                    self._config,
                    goal.elevator_id.strip(),
                    goal.source_floor.strip(),
                    goal.target_floor.strip(),
                )
            except (KeyError, TypeError, ValueError) as error:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_INVALID_REQUEST,
                    str(error),
                )

            state = self._floor_state
            odom = self._odometry
            recovering_committed_target = (
                self._hold_asserted
                and state is not None
                and state.ready
                and state.floor_id == plan.target_floor
                and odom is not None
            )
            if recovering_committed_target:
                transaction_started = True
                generation = state.generation
                self._publish_hold(True)
                self._feedback(
                    goal_handle,
                    'RECOVERING_COMMITTED_TARGET',
                    'retrying target pose verification, reset, and sensing',
                    generation,
                )
                pose_ready, pose_message = self._establish_target_pose(
                    goal_handle, plan, generation
                )
                if not pose_ready:
                    goal_handle.abort()
                    return self._result(
                        False,
                        SwitchFloor.Result.ERROR_POSE_TRANSFER,
                        pose_message,
                    )

                reset = ResetNavigation.Request()
                reset.floor_id = plan.target_floor
                reset.generation = generation
                reset_response = self._call_service(
                    self._reset_client, reset, self._reset_timeout
                )
                if reset_response is None or not reset_response.success:
                    goal_handle.abort()
                    return self._result(
                        False,
                        SwitchFloor.Result.ERROR_NAVIGATION_RESET,
                        'target navigation recovery failed',
                    )

                baseline = 0
                sensing = self._sensing_state
                if (
                    sensing is not None
                    and sensing.floor_id == plan.target_floor
                    and sensing.generation == generation
                ):
                    baseline = sensing.fresh_cloud_count
                self._feedback(
                    goal_handle,
                    'WAITING_SENSING',
                    'waiting for post-recovery local clouds',
                    generation,
                )
                if not self._wait_sensing(
                    goal_handle,
                    plan.target_floor,
                    generation,
                    baseline + self._minimum_fresh_clouds,
                    self._readiness_timeout,
                ):
                    goal_handle.abort()
                    return self._result(
                        False,
                        SwitchFloor.Result.ERROR_SENSING_TIMEOUT,
                        'target sensing recovery timed out',
                    )

                self._feedback(
                    goal_handle,
                    'COMPLETE',
                    'committed target recovered; releasing safety hold',
                    generation,
                )
                self._publish_hold(False)
                goal_handle.succeed()
                return self._result(
                    True,
                    SwitchFloor.Result.ERROR_NONE,
                    f'recovered {plan.source_floor}->{plan.target_floor}',
                )

            if (
                state is None
                or not state.ready
                or state.floor_id != plan.source_floor
                or odom is None
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_INVALID_REQUEST,
                    'active source floor, map, or odometry is not ready',
                )
            current_pose = (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                _yaw_from_odometry(odom),
            )
            distance, yaw_error = pose_error(
                current_pose, plan.source_trigger
            )
            if (
                distance > plan.trigger_tolerance_xy
                or yaw_error > plan.trigger_tolerance_yaw
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_NOT_AT_TRIGGER,
                    f'robot is {distance:.3f}m/{yaw_error:.3f}rad '
                    'from the elevator trigger',
                )

            transaction_started = True
            self._publish_hold(True)
            self._feedback(
                goal_handle, 'HOLDING', 'floor switch safety hold asserted'
            )
            if not self._wait_stopped(
                goal_handle,
                float(
                    self._config['map_switch_transaction'][
                        'cancel_navigation_timeout_sec'
                    ]
                ),
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_STOP_TIMEOUT,
                    'robot did not reach a confirmed stop',
                )

            self._feedback(
                goal_handle, 'RESETTING_OLD_NAV', 'clearing old navigation'
            )
            reset = ResetNavigation.Request()
            reset.floor_id = plan.source_floor
            reset.generation = state.generation
            reset_response = self._call_service(
                self._reset_client, reset, self._reset_timeout
            )
            if reset_response is None or not reset_response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_NAVIGATION_RESET,
                    'old-floor navigation reset failed',
                )

            self._feedback(
                goal_handle,
                'TRANSFERRING',
                f'executing {plan.transfer_adapter} transport adapter',
            )
            transfer_ready, transfer_message = self._execute_transfer(
                goal_handle, plan, state.generation
            )
            if not transfer_ready:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_TRANSFER,
                    transfer_message,
                )

            self._feedback(
                goal_handle, 'SWITCHING_MAP', 'committing target floor map'
            )
            switch_request = SwitchMap.Request()
            switch_request.target_floor = plan.target_floor
            switch_request.expected_current_generation = state.generation
            switch_response = self._call_service(
                self._map_client, switch_request, self._map_timeout
            )
            if switch_response is None or not switch_response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_MAP_SWITCH,
                    'target map commit failed',
                )
            generation = switch_response.generation

            self._feedback(
                goal_handle,
                'HANDING_OFF_POSE',
                f'applying {plan.pose_handoff} pose policy',
                generation,
            )
            pose_ready, pose_message = self._establish_target_pose(
                goal_handle, plan, generation
            )
            if not pose_ready:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_POSE_TRANSFER,
                    pose_message,
                )

            self._feedback(
                goal_handle,
                'RESETTING_NEW_NAV',
                'resetting SCAN at the transferred pose',
                generation,
            )
            reset.floor_id = plan.target_floor
            reset.generation = generation
            reset_response = self._call_service(
                self._reset_client, reset, self._reset_timeout
            )
            if reset_response is None or not reset_response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_NAVIGATION_RESET,
                    'target-floor navigation reset failed',
                )

            baseline = 0
            sensing = self._sensing_state
            if (
                sensing is not None
                and sensing.floor_id == plan.target_floor
                and sensing.generation == generation
            ):
                baseline = sensing.fresh_cloud_count
            required_count = baseline + self._minimum_fresh_clouds
            self._feedback(
                goal_handle,
                'WAITING_SENSING',
                f'waiting for {self._minimum_fresh_clouds} post-reset clouds',
                generation,
            )
            if not self._wait_sensing(
                goal_handle,
                plan.target_floor,
                generation,
                required_count,
                self._readiness_timeout,
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_SENSING_TIMEOUT,
                    'target-floor local sensing did not become fresh',
                )

            self._feedback(
                goal_handle,
                'COMPLETE',
                'target floor is ready; releasing safety hold',
                generation,
            )
            self._publish_hold(False)
            goal_handle.succeed()
            return self._result(
                True,
                SwitchFloor.Result.ERROR_NONE,
                f'switched {plan.source_floor}->{plan.target_floor}; '
                f'adapter={plan.transfer_adapter}, '
                f'pose_handoff={plan.pose_handoff}',
            )
        except _SwitchCancelled as error:
            if transaction_started:
                self._publish_hold(True)
            goal_handle.canceled()
            return self._result(
                False,
                SwitchFloor.Result.ERROR_CANCELLED,
                str(error),
            )
        except Exception as error:
            # Defensive boundary: retain hold on unexpected faults.
            if transaction_started:
                self._publish_hold(True)
            self.get_logger().error(f'floor switch failed: {error}')
            goal_handle.abort()
            return self._result(
                False,
                SwitchFloor.Result.ERROR_INTERNAL,
                str(error),
            )
        finally:
            with self._goal_lock:
                self._switch_active = False

    def destroy_node(self) -> None:
        self._action_server.destroy()
        for client in self._external_clients.values():
            client.destroy()
        super().destroy_node()


def main() -> None:
    """Run the floor switch manager with concurrent action callbacks."""
    rclpy.init()
    node = FloorSwitchManager()
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
