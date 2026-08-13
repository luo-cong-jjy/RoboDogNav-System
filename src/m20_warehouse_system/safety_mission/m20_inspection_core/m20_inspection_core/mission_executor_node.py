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

"""Typed mission executor composing navigation and floor-switch Actions."""

import math
from pathlib import Path
import threading
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from m20_warehouse_interfaces.action import (
    NavigateFloor,
    RunMission,
    SwitchFloor,
)
from m20_warehouse_interfaces.msg import FloorState, MissionState
from m20_warehouse_interfaces.srv import ControlMission
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
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import yaml

from .floor_switch_policy import resolve_transition
from .mission_policy import (
    MissionStep,
    display_step_index,
    resolve_mission,
)


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionExecutor(Node):
    """Run one configured mission and retain safe control on all interruptions."""

    def __init__(self) -> None:
        super().__init__('m20_mission_executor')
        self.declare_parameter('config_path', '')
        self.declare_parameter('action_name', '/m20/mission/run')
        self.declare_parameter('control_service', '/m20/mission/control')
        self.declare_parameter(
            'navigation_action', '/m20/navigation/navigate'
        )
        self.declare_parameter('floor_action', '/m20/floor_switch')
        self.declare_parameter('navigation_timeout_sec', 180.0)
        config_path = Path(str(self.get_parameter('config_path').value))
        if not config_path.is_file():
            raise ValueError(f'config_path is not a file: {config_path}')
        with config_path.open('r', encoding='utf-8') as stream:
            self._config = yaml.safe_load(stream)
        self._configured_mission_id = str(self._config['mission']['id'])
        self._navigation_timeout = max(
            1.0,
            float(self.get_parameter('navigation_timeout_sec').value),
        )

        self._callback_group = ReentrantCallbackGroup()
        self._floor_state: Optional[FloorState] = None
        self._goal_lock = threading.Lock()
        self._active = False
        self._has_run = False
        self._mission_id = ''
        self._state_name = 'IDLE'
        self._step_index = 0
        self._steps = ()
        self._paused = False
        self._fault = False
        self._stop_requested = False
        self._retry_requested = False
        self._atomic_transfer = False
        self._child_message = ''
        self._mission_hold = False
        self._floor_switch_hold: Optional[bool] = None
        qos = _latched_qos()

        self._state_publisher = self.create_publisher(
            MissionState, '/m20/mission/state', qos
        )
        self._hold_publisher = self.create_publisher(
            Bool, '/m20/control/mission_hold', qos
        )
        self._marker_publisher = self.create_publisher(
            Marker, '/m20/visualization/mission_marker', qos
        )
        self.create_subscription(
            FloorState,
            '/m20/map/state',
            self._floor_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            '/m20/control/floor_switch_hold',
            self._floor_switch_hold_callback,
            qos,
            callback_group=self._callback_group,
        )
        self._navigation_client = ActionClient(
            self,
            NavigateFloor,
            str(self.get_parameter('navigation_action').value),
            callback_group=self._callback_group,
        )
        self._floor_client = ActionClient(
            self,
            SwitchFloor,
            str(self.get_parameter('floor_action').value),
            callback_group=self._callback_group,
        )
        self._control_service = self.create_service(
            ControlMission,
            str(self.get_parameter('control_service').value),
            self._control_callback,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            RunMission,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._publish_hold(False)
        self._publish_state(None, 'IDLE', 'mission executor ready')
        self.get_logger().info('typed inspection mission executor ready')

    def _floor_callback(self, message: FloorState) -> None:
        self._floor_state = message

    def _floor_switch_hold_callback(self, message: Bool) -> None:
        self._floor_switch_hold = bool(message.data)

    def _goal_callback(self, goal) -> GoalResponse:
        if not goal.mission_id.strip():
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active or (self._has_run and not goal.restart):
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_hold(self, asserted: bool) -> None:
        self._mission_hold = asserted
        self._hold_publisher.publish(Bool(data=asserted))

    def _current_step(self) -> Optional[MissionStep]:
        if 0 <= self._step_index < len(self._steps):
            return self._steps[self._step_index]
        return None

    def _make_state(self, state_name: str, message: str) -> MissionState:
        state = MissionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.mission_id = self._mission_id
        state.state = state_name
        state.step_count = len(self._steps)
        display_index = display_step_index(
            self._step_index, len(self._steps)
        )
        state.step_index = display_index
        step = (
            self._steps[display_index]
            if 0 <= display_index < len(self._steps)
            else None
        )
        if step is not None:
            state.step_type = step.step_type
            state.step_name = step.name
        floor = self._floor_state
        if floor is not None:
            state.active_floor = floor.floor_id
            state.map_generation = floor.generation
        state.paused = self._paused
        state.fault = self._fault
        state.message = message
        return state

    def _publish_marker(self, state: MissionState) -> None:
        marker = Marker()
        marker.header.stamp = state.header.stamp
        marker.header.frame_id = 'warehouse_overview'
        marker.ns = 'mission_state'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 23.0
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.1
        marker.color.a = 1.0
        if state.fault:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.2, 0.2
        elif state.paused:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.8, 0.1
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.2, 1.0, 0.5
        marker.text = (
            f'Mission {state.mission_id or "-"} | {state.state} | '
            f'step {state.step_index + 1}/{max(1, state.step_count)} '
            f'{state.step_name} | {state.active_floor} '
            f'gen {state.map_generation}\n{state.message[:100]}'
        )
        self._marker_publisher.publish(marker)

    def _publish_state(
        self,
        goal_handle,
        state_name: str,
        message: str,
    ) -> None:
        self._state_name = state_name
        state = self._make_state(state_name, message)
        self._state_publisher.publish(state)
        self._publish_marker(state)
        if goal_handle is not None:
            feedback = RunMission.Feedback()
            feedback.state = state
            goal_handle.publish_feedback(feedback)

    def _control_callback(
        self,
        request: ControlMission.Request,
        response: ControlMission.Response,
    ) -> ControlMission.Response:
        if not self._active or request.mission_id != self._mission_id:
            response.success = False
            response.state = self._state_name
            response.message = 'requested mission is not active'
            return response
        if request.command == ControlMission.Request.PAUSE:
            if self._atomic_transfer:
                response.success = False
                response.state = self._state_name
                response.message = 'atomic elevator transfer cannot be paused'
                return response
            self._paused = True
            self._publish_hold(True)
            response.success = True
            response.message = 'pause requested'
        elif request.command == ControlMission.Request.RESUME:
            if not self._paused:
                response.success = False
                response.message = 'mission is not paused'
            else:
                self._paused = False
                response.success = True
                response.message = 'resume requested'
        elif request.command == ControlMission.Request.STOP:
            self._stop_requested = True
            self._publish_hold(True)
            response.success = True
            response.message = 'stop requested; safe hold retained'
        elif request.command == ControlMission.Request.RETRY_CURRENT:
            if not self._fault:
                response.success = False
                response.message = 'mission is not in FAULT_HOLD'
            else:
                self._retry_requested = True
                response.success = True
                response.message = 'retry requested'
        else:
            response.success = False
            response.message = 'unknown mission control command'
        response.state = self._state_name
        return response

    @staticmethod
    def _pose_message(pose, frame_id: str = 'map') -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.pose.position.x = float(pose[0])
        message.pose.position.y = float(pose[1])
        message.pose.position.z = 0.59
        half_yaw = float(pose[2]) / 2.0
        message.pose.orientation.z = math.sin(half_yaw)
        message.pose.orientation.w = math.cos(half_yaw)
        return message

    def _child_feedback(self, feedback) -> None:
        message = feedback.feedback
        self._child_message = str(
            getattr(message, 'message', '')
            or getattr(getattr(message, 'state', None), 'message', '')
        )

    @staticmethod
    def _wait_future(future, timeout: float):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        return future.result()

    def _cancel_child(self, child_handle, result_future=None) -> None:
        if child_handle is None:
            return
        future = child_handle.cancel_goal_async()
        self._wait_future(future, 5.0)
        if result_future is not None:
            self._wait_future(result_future, 5.0)

    def _run_child_action(
        self,
        client,
        request,
        goal_handle,
        *,
        pausable: bool,
        state_name: str,
    ):
        deadline = time.monotonic() + 5.0
        while not client.server_is_ready():
            if time.monotonic() >= deadline:
                return 'FAILED', None, 'child action server unavailable'
            if goal_handle.is_cancel_requested or self._stop_requested:
                return 'STOPPED', None, 'mission stopped'
            time.sleep(0.05)
        self._child_message = ''
        send_future = client.send_goal_async(
            request, feedback_callback=self._child_feedback
        )
        child_handle = self._wait_future(send_future, 5.0)
        if child_handle is None or not child_handle.accepted:
            return 'FAILED', None, 'child action goal rejected'
        result_future = child_handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_child(child_handle, result_future)
                return 'CANCELLED', None, 'mission action cancelled'
            if self._stop_requested:
                self._cancel_child(child_handle, result_future)
                return 'STOPPED', None, 'mission stop requested'
            if self._paused and pausable:
                self._cancel_child(child_handle, result_future)
                return 'PAUSED', None, 'mission paused'
            self._publish_state(
                goal_handle,
                state_name,
                self._child_message or 'child action running',
            )
            time.sleep(0.20)
        response = result_future.result()
        if response is None:
            return 'FAILED', None, 'child action returned no result'
        result = response.result
        if not result.success:
            return 'FAILED', result, result.message
        return 'SUCCEEDED', result, result.message

    def _wait_while_paused(self, goal_handle) -> str:
        self._publish_hold(True)
        while self._paused:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED'
            if self._stop_requested:
                return 'STOPPED'
            self._publish_state(goal_handle, 'PAUSED', 'mission paused')
            time.sleep(0.10)
        self._publish_hold(False)
        return 'RESUMED'

    def _wait_fault_retry(self, goal_handle, message: str) -> str:
        self._fault = True
        self._retry_requested = False
        self._publish_hold(True)
        while not self._retry_requested:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED'
            if self._stop_requested:
                return 'STOPPED'
            self._publish_state(goal_handle, 'FAULT_HOLD', message)
            time.sleep(0.10)
        self._fault = False
        self._retry_requested = False
        self._publish_hold(False)
        return 'RETRY'

    def _navigate(
        self,
        goal_handle,
        pose,
        segment_name: str,
    ):
        floor = self._floor_state
        if floor is None or not floor.ready:
            return 'FAILED', 'active map is not ready'
        request = NavigateFloor.Goal()
        request.goal_id = (
            f'{self._mission_id}:{self._step_index}:{segment_name}'
        )
        request.floor_id = floor.floor_id
        request.map_generation = floor.generation
        request.target_pose = self._pose_message(pose)
        request.timeout_sec = self._navigation_timeout
        outcome, _result, message = self._run_child_action(
            self._navigation_client,
            request,
            goal_handle,
            pausable=True,
            state_name='NAVIGATING',
        )
        return outcome, message

    def _run_inspection(self, goal_handle, step: MissionStep):
        floor = self._floor_state
        if floor is None or floor.floor_id != step.floor_id:
            return 'FAILED', (
                f'inspection {step.name} requires {step.floor_id}'
            )
        outcome, message = self._navigate(
            goal_handle, step.pose, step.name
        )
        if outcome != 'SUCCEEDED':
            return outcome, message
        dwell_deadline = time.monotonic() + step.dwell_sec
        while time.monotonic() < dwell_deadline:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED', 'mission action cancelled'
            if self._stop_requested:
                return 'STOPPED', 'mission stop requested'
            if self._paused:
                pause_start = time.monotonic()
                paused = self._wait_while_paused(goal_handle)
                if paused != 'RESUMED':
                    return paused, 'mission stopped while paused'
                dwell_deadline += time.monotonic() - pause_start
            self._publish_state(
                goal_handle,
                'DWELLING',
                f'inspecting {step.name}',
            )
            time.sleep(0.05)
        return 'SUCCEEDED', f'inspection {step.name} complete'

    def _run_terminal(self, goal_handle, step: MissionStep):
        """Navigate to a named mission endpoint without inspection dwell."""
        floor = self._floor_state
        if floor is None or floor.floor_id != step.floor_id:
            return 'FAILED', (
                f'terminal {step.name} requires {step.floor_id}'
            )
        outcome, message = self._navigate(
            goal_handle, step.pose, step.name
        )
        if outcome != 'SUCCEEDED':
            return outcome, message
        return 'SUCCEEDED', f'terminal {step.name} reached'

    def _run_transit(self, goal_handle, step: MissionStep):
        """Navigate through a route-shaping waypoint without inspection dwell."""
        floor = self._floor_state
        if floor is None or floor.floor_id != step.floor_id:
            return 'FAILED', (
                f'transit {step.name} requires {step.floor_id}'
            )
        outcome, message = self._navigate(
            goal_handle, step.pose, step.name
        )
        if outcome != 'SUCCEEDED':
            return outcome, message
        return 'SUCCEEDED', f'transit {step.name} reached'

    def _run_floor_transfer(self, goal_handle, step: MissionStep):
        plan = resolve_transition(
            self._config,
            step.connector_id,
            step.floor_id,
            step.target_floor,
        )
        floor = self._floor_state
        if floor is None or not floor.ready:
            return 'FAILED', 'active map is not ready for floor transfer'
        recovering_target = floor.floor_id == step.target_floor
        if (
            recovering_target
            and floor.ready
            and self._floor_switch_hold is False
        ):
            return (
                'SUCCEEDED',
                f'{step.target_floor} is already committed and released',
            )
        if not recovering_target:
            if floor.floor_id != step.floor_id:
                return 'FAILED', (
                    f'floor transfer requires {step.floor_id} or committed '
                    f'{step.target_floor}, active={floor.floor_id}'
                )
            for pose, segment in (
                (step.pose, 'connector_approach'),
                (plan.source_trigger, 'connector_trigger'),
            ):
                outcome, message = self._navigate(
                    goal_handle, pose, segment
                )
                if outcome != 'SUCCEEDED':
                    return outcome, message

        request = SwitchFloor.Goal()
        request.source_floor = step.floor_id
        request.target_floor = step.target_floor
        # SwitchFloor keeps the legacy field name for wire compatibility.
        request.elevator_id = step.connector_id
        self._atomic_transfer = True
        try:
            outcome, _result, message = self._run_child_action(
                self._floor_client,
                request,
                goal_handle,
                pausable=False,
                state_name='FLOOR_TRANSFER',
            )
            if outcome != 'SUCCEEDED':
                return outcome, message

            # The navigation gateway performs the authoritative wait for its
            # own hold=False sample when it accepts the next-floor goal.  This
            # executor still observes the hold topic for safe retry detection,
            # but it must not infer delivery to a different DDS subscriber.
            return 'SUCCEEDED', message
        finally:
            self._atomic_transfer = False

    def _result(
        self,
        success: bool,
        completed: int,
        error_code: int,
        message: str,
    ):
        result = RunMission.Result()
        result.success = success
        result.mission_id = self._mission_id
        result.completed_steps = completed
        result.error_code = error_code
        result.message = message
        return result

    def _execute(self, goal_handle):
        completed = 0
        try:
            self._mission_id = goal_handle.request.mission_id.strip()
            try:
                self._steps = resolve_mission(
                    self._config, self._mission_id
                )
            except ValueError as error:
                goal_handle.abort()
                return self._result(
                    False,
                    0,
                    RunMission.Result.ERROR_INVALID_REQUEST,
                    str(error),
                )
            self._step_index = 0
            self._paused = False
            self._fault = False
            self._stop_requested = False
            self._retry_requested = False
            self._publish_hold(False)
            self._publish_state(
                goal_handle, 'STARTING', 'mission accepted'
            )

            while self._step_index < len(self._steps):
                step = self._steps[self._step_index]
                if self._paused:
                    outcome = self._wait_while_paused(goal_handle)
                    if outcome == 'CANCELLED':
                        self._publish_state(
                            goal_handle,
                            'CANCELLED',
                            'mission cancelled; safe hold retained',
                        )
                        goal_handle.canceled()
                        return self._result(
                            False,
                            completed,
                            RunMission.Result.ERROR_CANCELLED,
                            'mission cancelled; safe hold retained',
                        )
                    if outcome == 'STOPPED':
                        self._publish_state(
                            goal_handle,
                            'STOPPED',
                            'mission stopped; safe hold retained',
                        )
                        goal_handle.abort()
                        return self._result(
                            False,
                            completed,
                            RunMission.Result.ERROR_STOPPED,
                            'mission stopped; safe hold retained',
                        )
                if step.step_type == 'inspection':
                    outcome, message = self._run_inspection(
                        goal_handle, step
                    )
                elif step.step_type in {
                    'elevator_transfer',
                    'floor_transfer',
                }:
                    outcome, message = self._run_floor_transfer(
                        goal_handle, step
                    )
                elif step.step_type == 'transit':
                    outcome, message = self._run_transit(
                        goal_handle, step
                    )
                elif step.step_type == 'terminal':
                    outcome, message = self._run_terminal(
                        goal_handle, step
                    )
                else:
                    outcome, message = (
                        'FAILED',
                        f'unsupported mission step {step.step_type!r}',
                    )
                if outcome == 'PAUSED':
                    paused = self._wait_while_paused(goal_handle)
                    if paused == 'RESUMED':
                        continue
                    outcome = paused
                if outcome == 'FAILED':
                    retry = self._wait_fault_retry(goal_handle, message)
                    if retry == 'RETRY':
                        continue
                    outcome = retry
                if outcome in {'STOPPED', 'CANCELLED'}:
                    self._publish_hold(True)
                    if outcome == 'CANCELLED':
                        self._publish_state(
                            goal_handle,
                            'CANCELLED',
                            'mission cancelled; safe hold retained',
                        )
                        goal_handle.canceled()
                        return self._result(
                            False,
                            completed,
                            RunMission.Result.ERROR_CANCELLED,
                            'mission cancelled; safe hold retained',
                        )
                    self._publish_state(
                        goal_handle,
                        'STOPPED',
                        'mission stopped; safe hold retained',
                    )
                    goal_handle.abort()
                    return self._result(
                        False,
                        completed,
                        RunMission.Result.ERROR_STOPPED,
                        'mission stopped; safe hold retained',
                    )
                self._step_index += 1
                completed += 1
                self._publish_state(
                    goal_handle,
                    'STEP_COMPLETE',
                    f'{step.name} complete',
                )

            self._publish_hold(False)
            self._publish_state(
                goal_handle, 'COMPLETED', 'mission complete'
            )
            goal_handle.succeed()
            return self._result(
                True,
                completed,
                RunMission.Result.ERROR_NONE,
                'mission complete',
            )
        except Exception as error:
            self._fault = True
            self._publish_hold(True)
            self.get_logger().error(f'mission executor failed: {error}')
            self._publish_state(
                goal_handle,
                'INTERNAL_ERROR',
                f'{error}; safe hold retained',
            )
            goal_handle.abort()
            return self._result(
                False,
                completed,
                RunMission.Result.ERROR_INTERNAL,
                str(error),
            )
        finally:
            self._atomic_transfer = False
            self._has_run = True
            with self._goal_lock:
                self._active = False

    def destroy_node(self) -> None:
        self._action_server.destroy()
        self._navigation_client.destroy()
        self._floor_client.destroy()
        super().destroy_node()


def main() -> None:
    """Run mission orchestration with concurrent child Action callbacks."""
    rclpy.init()
    node = MissionExecutor()
    executor = MultiThreadedExecutor(num_threads=6)
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
