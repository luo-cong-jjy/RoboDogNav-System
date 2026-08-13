# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""In-graph stability and fault regression for the flat multi-floor system."""

import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from m20_warehouse_interfaces.action import (
    NavigateFloor,
    RunMission,
    SwitchFloor,
)
from m20_warehouse_interfaces.msg import (
    FloorState,
    LocalSensingState,
    MissionState,
)
from m20_warehouse_interfaces.srv import (
    ControlMission,
    SetSimulationPose,
)
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
from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2
import yaml

from .configuration import transition_route

from .regression_policy import (
    alternating_floor,
    expected_generation,
    memory_growth_is_bounded,
    memory_trend,
    parse_proc_status_memory,
)


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


class Phase5Regression(Node):
    """Exercise repeated transactions and fail-closed recovery contracts."""

    def __init__(self) -> None:
        super().__init__('m20_phase5_regression')
        self.declare_parameter('config_path', '')
        self.declare_parameter('mode', 'switch_stress')
        self.declare_parameter('switch_iterations', 20)
        self.declare_parameter('mission_runs', 10)
        self.declare_parameter('mission_timeout_sec', 900.0)
        self.declare_parameter('memory_growth_limit_mib', 128.0)
        self.declare_parameter('memory_slope_limit_mib', 8.0)
        config_path = Path(str(self.get_parameter('config_path').value))
        if not config_path.is_file():
            raise ValueError(f'config_path is not a file: {config_path}')
        with config_path.open('r', encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.mode = str(self.get_parameter('mode').value)
        self.switch_iterations = max(
            1, int(self.get_parameter('switch_iterations').value)
        )
        self.mission_runs = max(
            1, int(self.get_parameter('mission_runs').value)
        )
        self.mission_timeout = max(
            30.0,
            float(self.get_parameter('mission_timeout_sec').value),
        )
        self.memory_growth_limit = max(
            0.0,
            float(self.get_parameter('memory_growth_limit_mib').value),
        )
        self.memory_slope_limit = max(
            0.0,
            float(self.get_parameter('memory_slope_limit_mib').value),
        )
        self._launch_parent_pid = self._parent_pid(
            Path('/proc/self/status')
        )

        self.floor: Optional[FloorState] = None
        self.sensing: Optional[LocalSensingState] = None
        self.mission: Optional[MissionState] = None
        self.odom: Optional[Odometry] = None
        self.safety_state = ''
        self.floor_hold = False
        self.mission_hold = False
        self.floor_hold_assertions = 0
        self.floor_zero_windows = 0
        self.mission_zero_windows = 0
        self._floor_zero_confirmed = False
        self._mission_zero_confirmed = False
        self.safe_hold_violations = 0
        self.zero_command_count = 0
        self.raw_cloud_count = 0
        qos = _latched_qos()
        self.create_subscription(
            FloorState, '/m20/map/state', self._floor_callback, qos
        )
        self.create_subscription(
            LocalSensingState,
            '/m20/sensing/state',
            self._sensing_callback,
            qos,
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
            PointCloud2,
            '/quad_0/cloud',
            self._raw_cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            '/m20/control/cmd_vel_safe',
            self._safe_command_callback,
            20,
        )
        self.create_subscription(
            String,
            '/m20/control/safety_state',
            self._safety_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            '/m20/control/floor_switch_hold',
            self._floor_hold_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            '/m20/control/mission_hold',
            self._mission_hold_callback,
            qos,
        )
        self.e_stop_publisher = self.create_publisher(
            Bool, '/m20/control/e_stop', 10
        )
        self.navigation_client = ActionClient(
            self, NavigateFloor, '/m20/navigation/navigate'
        )
        self.switch_client = ActionClient(
            self, SwitchFloor, '/m20/floor_switch'
        )
        self.mission_client = ActionClient(
            self, RunMission, '/m20/mission/run'
        )
        self.mission_control_client = self.create_client(
            ControlMission, '/m20/mission/control'
        )
        self.set_pose_client = self.create_client(
            SetSimulationPose, '/m20/sim/set_pose'
        )

    def _floor_callback(self, message: FloorState) -> None:
        self.floor = message

    def _sensing_callback(self, message: LocalSensingState) -> None:
        self.sensing = message

    def _mission_callback(self, message: MissionState) -> None:
        self.mission = message

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def _raw_cloud_callback(self, _message: PointCloud2) -> None:
        self.raw_cloud_count += 1

    def _safety_callback(self, message: String) -> None:
        self.safety_state = message.data

    def _floor_hold_callback(self, message: Bool) -> None:
        if message.data and not self.floor_hold:
            self.floor_hold_assertions += 1
            self._floor_zero_confirmed = False
        if not message.data:
            self._floor_zero_confirmed = False
        self.floor_hold = message.data

    def _mission_hold_callback(self, message: Bool) -> None:
        if message.data and not self.mission_hold:
            self._mission_zero_confirmed = False
        if not message.data:
            self._mission_zero_confirmed = False
        self.mission_hold = message.data

    def _safe_command_callback(self, message: Twist) -> None:
        magnitude = (
            abs(message.linear.x)
            + abs(message.linear.y)
            + abs(message.angular.z)
        )
        self.zero_command_count = (
            self.zero_command_count + 1 if magnitude <= 1e-6 else 0
        )
        if (
            self.floor_hold
            and not self._floor_zero_confirmed
            and self.zero_command_count >= 3
        ):
            self._floor_zero_confirmed = True
            self.floor_zero_windows += 1
        if (
            self.mission_hold
            and not self._mission_zero_confirmed
            and self.zero_command_count >= 3
        ):
            self._mission_zero_confirmed = True
            self.mission_zero_windows += 1
        if (
            (
                self._floor_zero_confirmed
                or self._mission_zero_confirmed
            )
            and magnitude > 1e-6
        ):
            self.safe_hold_violations += 1

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
        ready = self._wait_until(
            lambda: (
                self.floor is not None
                and self.floor.ready
                and self.sensing is not None
                and self.sensing.ready
                and self.odom is not None
            ),
            30.0,
        )
        if not ready:
            sensing = self.sensing
            sensing_summary = (
                'none'
                if sensing is None
                else (
                    f'{sensing.floor_id}/gen{sensing.generation}/'
                    f'ready={sensing.ready}/'
                    f'fresh={sensing.fresh_cloud_count}/'
                    f'message={sensing.message}'
                )
            )
            cloud_topics = sorted(
                name
                for name, _types in self.get_topic_names_and_types()
                if 'cloud' in name
            )
            raise RuntimeError(
                'map, sensing, and odometry did not become ready: '
                f'raw_clouds={self.raw_cloud_count}, '
                'cloud_publishers='
                f'{self.count_publishers("/quad_0/cloud")}, '
                f'cloud_topics={cloud_topics}, '
                f'sensing={sensing_summary}'
            )

    def _set_e_stop(self, asserted: bool) -> None:
        message = Bool(data=asserted)
        for _index in range(3):
            self.e_stop_publisher.publish(message)
            time.sleep(0.03)
        expected = (
            (lambda: self.safety_state == 'E_STOP')
            if asserted
            else (lambda: self.safety_state != 'E_STOP')
        )
        if not self._wait_until(expected, 3.0):
            raise RuntimeError(
                f'e-stop state did not become {asserted}: '
                f'{self.safety_state}'
            )

    def _pose_for(self, floor_id: str, field: str):
        return tuple(
            float(value) for value in self.config['floors'][floor_id][field]
        )

    def _source_trigger(self, source: str, target: str):
        route = transition_route(self.config, 'E1', source, target)
        values = route['source_trigger_pose']
        return tuple(float(value) for value in values)

    def _target_release(self, source: str, target: str):
        route = transition_route(self.config, 'E1', source, target)
        values = route['target_release_pose']
        return tuple(float(value) for value in values)

    def _teleport_under_e_stop(self, pose) -> None:
        if self.floor is None:
            raise RuntimeError('cannot teleport before floor state')
        self._set_e_stop(True)
        if not self.set_pose_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('simulation pose service unavailable')
        request = SetSimulationPose.Request()
        request.x, request.y, request.yaw = pose
        request.floor_id = self.floor.floor_id
        request.generation = self.floor.generation
        response = self._wait_future(
            self.set_pose_client.call_async(request), 5.0
        )
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(f'test pose preparation failed: {message}')
        if not self._wait_until(
            lambda: (
                self.odom is not None
                and math.hypot(
                    self.odom.pose.pose.position.x - pose[0],
                    self.odom.pose.pose.position.y - pose[1],
                )
                <= 0.05
                and abs(
                    math.atan2(
                        math.sin(_yaw_from_odometry(self.odom) - pose[2]),
                        math.cos(_yaw_from_odometry(self.odom) - pose[2]),
                    )
                )
                <= 0.05
            ),
            3.0,
        ):
            raise RuntimeError('simulation pose did not converge')
        self._set_e_stop(False)

    def _send_switch(self, source: str, target: str, timeout: float = 20.0):
        if not self.switch_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('floor switch Action server unavailable')
        request = SwitchFloor.Goal()
        request.source_floor = source
        request.target_floor = target
        request.elevator_id = 'E1'
        handle = self._wait_future(
            self.switch_client.send_goal_async(request), 5.0
        )
        if handle is None or not handle.accepted:
            raise RuntimeError('floor switch goal was rejected')
        response = self._wait_future(handle.get_result_async(), timeout)
        if response is None:
            handle.cancel_goal_async()
            raise RuntimeError('floor switch result timed out')
        return response.result

    def _switch_successfully(self, source: str, target: str) -> None:
        if self.floor is None:
            raise RuntimeError('floor state unavailable')
        initial_generation = self.floor.generation
        hold_count = self.floor_hold_assertions
        zero_windows = self.floor_zero_windows
        violations = self.safe_hold_violations
        result = self._send_switch(source, target)
        if not result.success:
            raise RuntimeError(
                f'{source}->{target} failed: code={result.error_code}, '
                f'message={result.message}'
            )
        target_generation = initial_generation + 1
        if not self._wait_until(
            lambda: (
                self.floor is not None
                and self.floor.ready
                and self.floor.floor_id == target
                and self.floor.generation == target_generation
                and self.sensing is not None
                and self.sensing.ready
                and self.sensing.floor_id == target
                and self.sensing.generation == target_generation
                and not self.floor_hold
            ),
            8.0,
        ):
            raise RuntimeError('post-switch floor/sensing state is inconsistent')
        if self.floor_hold_assertions != hold_count + 1:
            raise RuntimeError('floor hold was not asserted exactly once')
        if self.floor_zero_windows != zero_windows + 1:
            raise RuntimeError(
                'floor hold did not produce three confirmed zero commands'
            )
        if self.safe_hold_violations != violations:
            raise RuntimeError('non-zero safe command observed during hold')
        release = self._target_release(source, target)
        if self.odom is None or math.hypot(
            self.odom.pose.pose.position.x - release[0],
            self.odom.pose.pose.position.y - release[1],
        ) > 0.10:
            raise RuntimeError('robot is not at target elevator release pose')

    @staticmethod
    def _parent_pid(status_path: Path) -> int:
        """Read one process parent ID from a procfs status file."""
        parent_pid, _rss_kib = parse_proc_status_memory(
            status_path.read_text()
        )
        return parent_pid

    def _project_rss_mib(self) -> float:
        """Read RSS for direct node children of this launch process."""
        total_kib = 0
        proc_root = Path('/proc')
        for process_dir in proc_root.iterdir():
            if not process_dir.name.isdigit():
                continue
            try:
                parent_pid, rss_kib = parse_proc_status_memory(
                    (process_dir / 'status').read_text()
                )
                process_pid = int(process_dir.name)
                if (
                    process_pid != os.getpid()
                    and parent_pid != self._launch_parent_pid
                ):
                    continue
                total_kib += rss_kib
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
        if total_kib <= 0:
            raise RuntimeError('launch process RSS could not be sampled')
        return total_kib / 1024.0

    def _check_memory(self, samples) -> None:
        trend = memory_trend(samples)
        self.get_logger().info(
            'RSS raw trend: '
            f'first={trend.first_mib:.1f}MiB, '
            f'last={trend.last_mib:.1f}MiB, '
            f'peak={trend.peak_mib:.1f}MiB, '
            f'growth={trend.growth_mib:.1f}MiB, '
            f'slope={trend.slope_mib_per_iteration:.2f}MiB/iteration'
        )
        # The first completed transaction initializes DDS/allocator caches.
        # Leak acceptance starts after that bounded one-time warm-up.
        steady_samples = samples[1:] if len(samples) > 1 else samples
        steady = memory_trend(steady_samples)
        self.get_logger().info(
            'RSS steady trend: '
            f'first={steady.first_mib:.1f}MiB, '
            f'last={steady.last_mib:.1f}MiB, '
            f'growth={steady.growth_mib:.1f}MiB, '
            'slope='
            f'{steady.slope_mib_per_iteration:.2f}MiB/iteration'
        )
        if not memory_growth_is_bounded(
            steady_samples,
            self.memory_growth_limit,
            self.memory_slope_limit,
        ):
            raise RuntimeError(
                'steady-state project RSS growth exceeds limits'
            )

    def _switch_stress(self) -> None:
        self._wait_ready()
        if self.floor is None:
            raise RuntimeError('floor state unavailable')
        initial_floor = self.floor.floor_id
        initial_generation = self.floor.generation
        samples = [self._project_rss_mib()]
        for index in range(self.switch_iterations):
            source = self.floor.floor_id
            target = alternating_floor(initial_floor, index)
            self._teleport_under_e_stop(
                self._source_trigger(source, target)
            )
            self._switch_successfully(source, target)
            expected = expected_generation(initial_generation, index + 1)
            if self.floor is None or self.floor.generation != expected:
                raise RuntimeError('map generation did not increment once')
            rss_mib = self._project_rss_mib()
            samples.append(rss_mib)
            self.get_logger().info(
                f'switch stress {index + 1}/{self.switch_iterations}: '
                f'{source}->{target}, generation={expected}, '
                f'rss={rss_mib:.1f}MiB'
            )
        self._check_memory(samples)
        self.get_logger().info(
            'PHASE5_SWITCH_STRESS_PASS: '
            f'{self.switch_iterations} alternating switches, '
            'no hold velocity violations'
        )

    def _send_mission(self, restart: bool):
        if not self.mission_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('mission Action server unavailable')
        request = RunMission.Goal()
        request.mission_id = str(self.config['mission']['id'])
        request.restart = restart
        handle = self._wait_future(
            self.mission_client.send_goal_async(request), 5.0
        )
        if handle is None or not handle.accepted:
            raise RuntimeError('mission goal was rejected')
        return handle, handle.get_result_async()

    def _mission_control(self, command: int):
        if not self.mission_control_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('mission control service unavailable')
        request = ControlMission.Request()
        request.mission_id = str(self.config['mission']['id'])
        request.command = command
        response = self._wait_future(
            self.mission_control_client.call_async(request), 5.0
        )
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            raise RuntimeError(f'mission control failed: {message}')
        return response

    def _return_to_f1(self) -> None:
        if self.floor is None or self.floor.floor_id != 'F2':
            raise RuntimeError('return preparation requires F2')
        self._teleport_under_e_stop(self._source_trigger('F2', 'F1'))
        self._switch_successfully('F2', 'F1')
        self._teleport_under_e_stop(self._pose_for('F1', 'initial_pose'))

    def _mission_stress(self) -> None:
        self._wait_ready()
        if self.floor is None or self.floor.floor_id != 'F1':
            raise RuntimeError('mission stress must start on F1')
        initial_generation = self.floor.generation
        samples = [self._project_rss_mib()]
        for index in range(self.mission_runs):
            self._teleport_under_e_stop(
                self._pose_for('F1', 'initial_pose')
            )
            _handle, result_future = self._send_mission(index > 0)
            response = self._wait_future(
                result_future, self.mission_timeout
            )
            if response is None:
                raise RuntimeError(f'mission run {index + 1} timed out')
            result = response.result
            expected_steps = len(self.config['mission']['sequence'])
            if (
                not result.success
                or result.completed_steps != expected_steps
            ):
                raise RuntimeError(
                    f'mission run {index + 1} failed: '
                    f'completed={result.completed_steps}, '
                    f'expected={expected_steps}, '
                    f'code={result.error_code}, message={result.message}'
                )
            expected = initial_generation + 1 + 2 * index
            if (
                self.floor is None
                or self.floor.floor_id != 'F2'
                or self.floor.generation != expected
            ):
                raise RuntimeError('mission ended on unexpected floor/generation')
            rss_mib = self._project_rss_mib()
            samples.append(rss_mib)
            self.get_logger().info(
                f'mission stress {index + 1}/{self.mission_runs}: '
                f'{expected_steps}/{expected_steps}, '
                f'F2 generation={expected}, '
                f'rss={rss_mib:.1f}MiB'
            )
            if index + 1 < self.mission_runs:
                self._return_to_f1()
        self._check_memory(samples)
        self.get_logger().info(
            'PHASE5_MISSION_STRESS_PASS: '
            f'{self.mission_runs}/{self.mission_runs} complete missions'
        )

    def _fault_injection(self) -> None:
        self._wait_ready()
        if self.floor is None or self.floor.floor_id != 'F1':
            raise RuntimeError('fault injection must start on F1')

        self._teleport_under_e_stop(self._pose_for('F1', 'initial_pose'))
        generation = self.floor.generation
        result = self._send_switch('F1', 'F2')
        if (
            result.success
            or result.error_code != SwitchFloor.Result.ERROR_NOT_AT_TRIGGER
            or self.floor.generation != generation
            or self.floor_hold
        ):
            raise RuntimeError('off-trigger floor switch was not safely rejected')

        request = NavigateFloor.Goal()
        request.goal_id = 'phase5_stale_generation'
        request.floor_id = 'F1'
        request.map_generation = generation + 1
        request.target_pose.header.frame_id = 'map'
        request.target_pose.pose.position.x = -40.0
        request.target_pose.pose.position.z = 0.59
        request.target_pose.pose.orientation.w = 1.0
        request.timeout_sec = 20.0
        if not self.navigation_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('navigation Action server unavailable')
        nav_handle = self._wait_future(
            self.navigation_client.send_goal_async(request), 5.0
        )
        if nav_handle is None or not nav_handle.accepted:
            raise RuntimeError('typed stale-generation goal was not evaluated')
        nav_response = self._wait_future(
            nav_handle.get_result_async(), 8.0
        )
        if nav_response is None or nav_response.result.success:
            raise RuntimeError('stale-generation navigation was not rejected')

        request.goal_id = 'phase5_cancel_navigation'
        request.map_generation = generation
        request.target_pose.pose.position.x = -35.0
        request.target_pose.pose.position.y = -10.0
        nav_handle = self._wait_future(
            self.navigation_client.send_goal_async(request), 5.0
        )
        if nav_handle is None or not nav_handle.accepted:
            raise RuntimeError('cancel test navigation was rejected')
        nav_future = nav_handle.get_result_async()
        time.sleep(2.0)
        cancel_response = self._wait_future(
            nav_handle.cancel_goal_async(), 5.0
        )
        nav_response = self._wait_future(nav_future, 8.0)
        if (
            cancel_response is None
            or nav_response is None
            or nav_response.result.error_code
            != NavigateFloor.Result.ERROR_CANCELLED
        ):
            raise RuntimeError('navigation cancel did not return CANCELLED')

        _mission_handle, mission_future = self._send_mission(False)
        if not self._wait_until(
            lambda: self.mission is not None
            and self.mission.state == 'NAVIGATING',
            15.0,
        ):
            raise RuntimeError('mission stop test did not enter NAVIGATING')
        time.sleep(1.0)
        self._mission_control(ControlMission.Request.STOP)
        mission_response = self._wait_future(mission_future, 10.0)
        if (
            mission_response is None
            or mission_response.result.error_code
            != RunMission.Result.ERROR_STOPPED
            or not self._wait_until(
                lambda: (
                    self.mission_hold
                    and self.safety_state == 'MISSION_HOLD'
                    and self.zero_command_count >= 3
                ),
                5.0,
            )
        ):
            raise RuntimeError('mission stop did not retain safe hold')

        self._teleport_under_e_stop(self._source_trigger('F1', 'F2'))
        self._switch_successfully('F1', 'F2')
        _retry_handle, retry_future = self._send_mission(True)
        if not self._wait_until(
            lambda: self.mission is not None
            and self.mission.state == 'FAULT_HOLD'
            and self.mission.fault,
            8.0,
        ):
            raise RuntimeError('wrong-floor mission did not enter FAULT_HOLD')
        self._teleport_under_e_stop(self._source_trigger('F2', 'F1'))
        self._switch_successfully('F2', 'F1')
        self._teleport_under_e_stop(self._pose_for('F1', 'initial_pose'))
        self._mission_control(ControlMission.Request.RETRY_CURRENT)
        if not self._wait_until(
            lambda: self.mission is not None
            and self.mission.state == 'NAVIGATING'
            and not self.mission.fault,
            15.0,
        ):
            raise RuntimeError('retry-current did not restart navigation')
        self._mission_control(ControlMission.Request.STOP)
        retry_response = self._wait_future(retry_future, 10.0)
        if (
            retry_response is None
            or retry_response.result.error_code
            != RunMission.Result.ERROR_STOPPED
        ):
            raise RuntimeError('retried mission did not stop cleanly')
        if self.safe_hold_violations:
            raise RuntimeError('non-zero safe velocity observed under a hold')
        self.get_logger().info(
            'PHASE5_FAULT_INJECTION_PASS: invalid switch, stale goal, '
            'navigation cancel, mission stop, FAULT_HOLD retry'
        )

    def run(self) -> None:
        """Run the selected regression mode."""
        if self.mode == 'switch_stress':
            self._switch_stress()
        elif self.mode == 'mission_stress':
            self._mission_stress()
        elif self.mode == 'fault_injection':
            self._fault_injection()
        else:
            raise RuntimeError(f'unknown phase-5 mode {self.mode!r}')


def main() -> None:
    """Run regression alongside a concurrent ROS executor."""
    rclpy.init()
    node = Phase5Regression()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    exit_code = 0
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(f'PHASE5_REGRESSION_FAIL: {error}')
        exit_code = 1
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
