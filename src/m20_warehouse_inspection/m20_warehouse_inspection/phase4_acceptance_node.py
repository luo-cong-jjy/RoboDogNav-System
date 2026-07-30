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

"""Optional in-graph runtime acceptance client for phase 4."""

import math
from pathlib import Path
import sys
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from m20_inspection_core.mission_policy import resolve_mission
from m20_warehouse_interfaces.action import NavigateFloor, RunMission
from m20_warehouse_interfaces.msg import FloorState, MissionState
from m20_warehouse_interfaces.srv import ControlMission
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
import yaml


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class Phase4Acceptance(Node):
    """Exercise typed navigation or the complete configured mission."""

    def __init__(self) -> None:
        super().__init__('m20_phase4_acceptance')
        self.declare_parameter('mode', 'quick')
        self.declare_parameter('timeout_sec', 900.0)
        self.declare_parameter('config_path', '')
        self.mode = str(self.get_parameter('mode').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        config_path = Path(
            str(self.get_parameter('config_path').value)
        ).expanduser()
        if not config_path.is_file():
            raise ValueError(f'config_path is not a file: {config_path}')
        with config_path.open('r', encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.mission_id = str(self.config['mission']['id'])
        self.steps = resolve_mission(self.config, self.mission_id)
        self.expected_floor = self.steps[-1].floor_id
        self.expected_pose = self.steps[-1].pose
        self.expected_generation = 1 + sum(
            step.step_type == 'elevator_transfer'
            for step in self.steps
        )
        self.floor: Optional[FloorState] = None
        self.mission: Optional[MissionState] = None
        self.odom: Optional[Odometry] = None
        self.safety_state = ''
        self.zero_command_count = 0
        self._last_feedback_state = ''
        self._last_f1_pose = None
        self._first_f2_pose = None
        qos = _latched_qos()
        self.create_subscription(
            FloorState, '/m20/map/state', self._floor_callback, qos
        )
        self.create_subscription(
            MissionState,
            '/m20/mission/state',
            self._mission_callback,
            qos,
        )
        self.create_subscription(
            Odometry, '/m20/sim/body_pose', self._odom_callback, 20
        )
        self.create_subscription(
            Twist,
            '/m20/control/cmd_vel_safe',
            self._command_callback,
            20,
        )
        self.create_subscription(
            String,
            '/m20/control/safety_state',
            self._safety_callback,
            qos,
        )
        self.navigation_client = ActionClient(
            self, NavigateFloor, '/m20/navigation/navigate'
        )
        self.mission_client = ActionClient(
            self, RunMission, '/m20/mission/run'
        )
        self.control_client = self.create_client(
            ControlMission, '/m20/mission/control'
        )

    def _floor_callback(self, message: FloorState) -> None:
        self.floor = message

    def _mission_callback(self, message: MissionState) -> None:
        self.mission = message
        if message.state != self._last_feedback_state:
            self.get_logger().info(
                f'acceptance mission state: {message.state}, '
                f'step={message.step_index + 1}/{message.step_count}, '
                f'name={message.step_name}, floor={message.active_floor}, '
                f'gen={message.map_generation}, message={message.message}'
            )
            self._last_feedback_state = message.state

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message
        position = message.pose.pose.position
        pose_xy = (float(position.x), float(position.y))
        if self.floor is None:
            return
        if self.floor.generation == 1:
            self._last_f1_pose = pose_xy
        elif self.floor.generation == 2 and self._first_f2_pose is None:
            self._first_f2_pose = pose_xy

    def _command_callback(self, message: Twist) -> None:
        magnitude = (
            abs(message.linear.x)
            + abs(message.linear.y)
            + abs(message.angular.z)
        )
        self.zero_command_count = (
            self.zero_command_count + 1 if magnitude < 1e-6 else 0
        )

    def _safety_callback(self, message: String) -> None:
        self.safety_state = message.data

    @staticmethod
    def _wait_future(future, timeout: float):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        return future.result()

    @staticmethod
    def _wait_until(predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def _wait_ready(self) -> None:
        if not self._wait_until(
            lambda: (
                self.floor is not None
                and self.floor.ready
                and self.odom is not None
            ),
            20.0,
        ):
            raise RuntimeError('map and odometry did not become ready')

    def _quick_navigation(self) -> None:
        self._wait_ready()
        request = NavigateFloor.Goal()
        request.goal_id = 'phase4_quick_navigation'
        request.floor_id = self.floor.floor_id
        request.map_generation = self.floor.generation
        request.target_pose.header.frame_id = 'map'
        request.target_pose.pose.position.x = -35.0
        request.target_pose.pose.position.y = 0.0
        request.target_pose.pose.position.z = 0.59
        request.target_pose.pose.orientation.w = 1.0
        request.timeout_sec = 60.0
        if not self.navigation_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('navigation Action server unavailable')
        handle = self._wait_future(
            self.navigation_client.send_goal_async(request), 10.0
        )
        if handle is None or not handle.accepted:
            raise RuntimeError('quick navigation goal rejected')
        response = self._wait_future(handle.get_result_async(), 70.0)
        if response is None or not response.result.success:
            message = (
                'no result' if response is None else response.result.message
            )
            raise RuntimeError(f'quick navigation failed: {message}')
        if response.result.final_distance > 0.20:
            raise RuntimeError('quick navigation stopped outside tolerance')
        self.get_logger().info(
            'PHASE4_QUICK_ACCEPTANCE_PASS: typed navigation reached goal'
        )

    def _control(self, command: int):
        if not self.control_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('mission control service unavailable')
        request = ControlMission.Request()
        request.mission_id = self.mission_id
        request.command = command
        response = self._wait_future(
            self.control_client.call_async(request), 10.0
        )
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(f'mission control failed: {message}')
        return response

    def _full_mission(self) -> None:
        self._wait_ready()
        if not self.mission_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('mission Action server unavailable')
        request = RunMission.Goal()
        request.mission_id = self.mission_id
        request.restart = False
        handle = self._wait_future(
            self.mission_client.send_goal_async(request), 10.0
        )
        if handle is None or not handle.accepted:
            raise RuntimeError('mission goal rejected')
        result_future = handle.get_result_async()
        if not self._wait_until(
            lambda: (
                self.mission is not None
                and self.mission.state == 'NAVIGATING'
            ),
            20.0,
        ):
            raise RuntimeError('mission did not enter navigation')
        time.sleep(2.0)
        self.zero_command_count = 0
        self._control(ControlMission.Request.PAUSE)
        if not self._wait_until(
            lambda: (
                self.mission is not None
                and self.mission.state == 'PAUSED'
                and self.safety_state == 'MISSION_HOLD'
                and self.zero_command_count >= 3
            ),
            15.0,
        ):
            raise RuntimeError(
                'pause did not produce typed PAUSED and three zero commands'
            )
        self.get_logger().info(
            'pause acceptance passed: MISSION_HOLD with zero safe velocity'
        )
        self._control(ControlMission.Request.RESUME)

        response = self._wait_future(result_future, self.timeout)
        if response is None:
            handle.cancel_goal_async()
            raise RuntimeError('mission acceptance timed out')
        result = response.result
        expected_steps = (
            self.mission.step_count if self.mission is not None else 0
        )
        if (
            not result.success
            or expected_steps <= 0
            or result.completed_steps != expected_steps
        ):
            raise RuntimeError(
                f'mission failed: completed={result.completed_steps}, '
                f'expected={expected_steps}, '
                f'code={result.error_code}, message={result.message}'
            )
        if (
            self.floor is None
            or self.floor.floor_id != self.expected_floor
            or self.floor.generation != self.expected_generation
            or not self.floor.ready
        ):
            raise RuntimeError(
                'mission did not finish at the configured floor/generation '
                f'({self.expected_floor}, {self.expected_generation})'
            )
        if self.odom is None:
            raise RuntimeError('final odometry unavailable')
        if self._last_f1_pose is None or self._first_f2_pose is None:
            raise RuntimeError('floor-switch continuity samples unavailable')
        switch_jump = math.hypot(
            self._first_f2_pose[0] - self._last_f1_pose[0],
            self._first_f2_pose[1] - self._last_f1_pose[1],
        )
        trigger_error = max(
            math.hypot(*self._last_f1_pose),
            math.hypot(*self._first_f2_pose),
        )
        if switch_jump > 0.02:
            raise RuntimeError(
                f'floor switch teleported the robot by {switch_jump:.3f}m'
            )
        if trigger_error > 0.35:
            raise RuntimeError(
                f'floor switch occurred {trigger_error:.3f}m from origin'
            )
        position = self.odom.pose.pose.position
        final_distance = math.hypot(
            position.x - self.expected_pose[0],
            position.y - self.expected_pose[1],
        )
        if final_distance > 0.25:
            raise RuntimeError(
                'final configured-terminal distance '
                f'{final_distance:.3f}m exceeds tolerance'
            )
        if self.mission is None or self.mission.state != 'COMPLETED':
            raise RuntimeError('typed mission state is not COMPLETED')
        self.get_logger().info(
            'PHASE4_FULL_ACCEPTANCE_PASS: '
            f'{expected_steps} steps, mission={self.mission_id}, '
            'pause/resume, zero-jump origin map switch, '
            f'final_floor={self.expected_floor}, '
            f'generation={self.expected_generation}, '
            f'switch_jump={switch_jump:.3f}m, '
            f'final_error={final_distance:.3f}m'
        )

    def run(self) -> None:
        """Run the selected acceptance mode."""
        if self.mode == 'quick':
            self._quick_navigation()
        elif self.mode == 'full':
            self._full_mission()
        else:
            raise RuntimeError(f'unknown acceptance mode {self.mode!r}')


def main() -> None:
    """Run acceptance alongside an executor thread and return a process code."""
    rclpy.init()
    node = Phase4Acceptance()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    exit_code = 0
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(f'PHASE4_ACCEPTANCE_FAIL: {error}')
        exit_code = 1
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
