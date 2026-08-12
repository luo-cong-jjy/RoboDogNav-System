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

"""Start and monitor the configured warehouse inspection mission."""

import argparse
from typing import Optional, Sequence

from m20_warehouse_interfaces.action import RunMission
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class InspectionStarter(Node):
    """Send one mission goal to the executor already running in bringup."""

    def __init__(self, mission_id: str, restart: bool) -> None:
        super().__init__('m20_start_inspection')
        self._mission_id = mission_id
        self._restart = restart
        self._client = ActionClient(
            self, RunMission, '/m20/mission/run'
        )
        self._last_feedback = ''

    def run(self, server_timeout: float) -> int:
        """Send the mission and block while printing typed feedback."""
        self.get_logger().info(
            f'waiting for inspection system: {self._mission_id}'
        )
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error(
                'inspection system is not running; start '
                'inspection_mission_rviz.launch.py first'
            )
            return 2

        goal = RunMission.Goal()
        goal.mission_id = self._mission_id
        goal.restart = self._restart
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('inspection request was rejected')
            return 3

        self.get_logger().info('inspection started')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error('inspection ended without a result')
            return 4

        result = wrapped.result
        if result.success:
            self.get_logger().info(
                'inspection completed: '
                f'{result.completed_steps} steps'
            )
            return 0
        self.get_logger().error(
            f'inspection failed: code={result.error_code}, '
            f'message={result.message}'
        )
        return 5

    def _feedback_callback(self, message) -> None:
        state = message.feedback.state
        feedback_key = (
            f'{state.state}:{state.step_index}:{state.step_name}:'
            f'{state.active_floor}:{state.map_generation}:{state.message}'
        )
        if feedback_key == self._last_feedback:
            return
        self._last_feedback = feedback_key
        self.get_logger().info(
            f'[{state.step_index + 1}/{state.step_count}] '
            f'{state.step_name}: {state.state}, '
            f'floor={state.active_floor}, '
            f'generation={state.map_generation}, '
            f'message={state.message}'
        )


def _parse_arguments(
    arguments: Optional[Sequence[str]],
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Start the configured M20 warehouse inspection.'
    )
    parser.add_argument(
        '--mission-id',
        default='two_floor_warehouse_demo',
        help='mission id from flat_multifloor_system.yaml',
    )
    parser.add_argument(
        '--no-restart',
        action='store_true',
        help='reject if this mission has already run',
    )
    parser.add_argument(
        '--server-timeout',
        type=float,
        default=15.0,
        help='seconds to wait for the running system',
    )
    return parser.parse_args(arguments)


def main(arguments: Optional[Sequence[str]] = None) -> int:
    """Run the simple inspection trigger command."""
    parsed = _parse_arguments(arguments)
    # Custom command-line options were consumed by argparse above.
    rclpy.init(args=[])
    node = InspectionStarter(
        parsed.mission_id, restart=not parsed.no_restart
    )
    try:
        return node.run(parsed.server_timeout)
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
