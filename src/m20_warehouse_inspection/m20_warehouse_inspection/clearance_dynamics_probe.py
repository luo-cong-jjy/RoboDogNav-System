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

"""Run one full MuJoCo mission and measure obstacle-clearance behaviour."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
from m20_warehouse_interfaces.action import RunMission
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import Bspline
from std_msgs.msg import Bool, String
import yaml


Rectangle = Tuple[float, float, float, float]


def yaw_from_odometry(message: Odometry) -> float:
    """Extract planar yaw from an odometry quaternion."""
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def point_rectangle_distance(
    x: float,
    y: float,
    rectangle: Rectangle,
) -> float:
    """Return zero inside a rectangle and Euclidean distance outside it."""
    min_x, max_x, min_y, max_y = rectangle
    dx = max(min_x - x, 0.0, x - max_x)
    dy = max(min_y - y, 0.0, y - max_y)
    return math.hypot(dx, dy)


def minimum_double_circle_distance(
    x: float,
    y: float,
    yaw: float,
    rectangles: Iterable[Rectangle],
    footprint_offset: float,
) -> float:
    """Measure the nearest obstacle distance from either circle centre."""
    values = []
    offset_x = footprint_offset * math.cos(yaw)
    offset_y = footprint_offset * math.sin(yaw)
    for sign in (-1.0, 1.0):
        center_x = x + sign * offset_x
        center_y = y + sign * offset_y
        values.extend(
            point_rectangle_distance(center_x, center_y, rectangle)
            for rectangle in rectangles
        )
    return min(values, default=math.inf)


def load_obstacle_rectangles(system_config: Path) -> List[Rectangle]:
    """Load generated obstacle rectangles in the shared simulation frame."""
    config_path = system_config.expanduser().resolve()
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    package_root = config_path.parent.parent
    rectangles = []
    for floor in config['floors'].values():
        metadata_path = Path(str(floor['metadata_file'])).expanduser()
        if not metadata_path.is_absolute():
            metadata_path = package_root / metadata_path
        with metadata_path.open('r', encoding='utf-8') as stream:
            metadata = json.load(stream)
        offset = metadata.get('coordinate_convention', {}).get(
            'simulation_offset',
            floor.get('simulation_offset', [0.0, 0.0, 0.0]),
        )
        offset_x = float(offset[0])
        offset_y = float(offset[1])
        for obstacle in metadata.get('obstacles', []):
            rectangles.append(
                (
                    float(obstacle['x_min']) + offset_x,
                    float(obstacle['x_max']) + offset_x,
                    float(obstacle['y_min']) + offset_y,
                    float(obstacle['y_max']) + offset_y,
                )
            )
    return rectangles


class ClearanceDynamicsProbe(Node):
    """Collect mission, command, guard, pose, and physics-contact streams."""

    def __init__(self) -> None:
        super().__init__('m20_clearance_dynamics_probe')
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.client = ActionClient(self, RunMission, '/m20/mission/run')
        self.pose: Optional[Odometry] = None
        self.candidate = Twist()
        self.safe = Twist()
        self.collision_stop = True
        self.guard_state = 'NOT_READY'
        self.guard_diagnostic = 'NOT_READY'
        self.guard_events: List[str] = []
        self.dynamics: Dict = {}
        self.trajectory_count = 0
        self.feedback: List[Dict] = []
        self.create_subscription(
            Odometry,
            '/m20/sim/body_pose',
            lambda message: setattr(self, 'pose', message),
            50,
        )
        self.create_subscription(
            Twist,
            '/m20/navigation/cmd_vel_candidate',
            lambda message: setattr(self, 'candidate', message),
            50,
        )
        self.create_subscription(
            Twist,
            '/m20/control/cmd_vel_safe',
            lambda message: setattr(self, 'safe', message),
            50,
        )
        self.create_subscription(
            Bool,
            '/m20/control/collision_stop',
            lambda message: setattr(
                self,
                'collision_stop',
                bool(message.data),
            ),
            latched,
        )
        self.create_subscription(
            String,
            '/m20/control/collision_guard_state',
            lambda message: setattr(
                self,
                'guard_state',
                str(message.data),
            ),
            latched,
        )
        self.create_subscription(
            String,
            '/m20/control/collision_guard_diagnostic',
            self._guard_diagnostic_callback,
            latched,
        )
        self.create_subscription(
            String,
            '/m20/sim/dynamics_state',
            self._dynamics_callback,
            20,
        )
        self.create_subscription(
            Bspline,
            '/planning/bspline',
            lambda _message: setattr(
                self,
                'trajectory_count',
                self.trajectory_count + 1,
            ),
            10,
        )

    def _guard_diagnostic_callback(self, message: String) -> None:
        diagnostic = str(message.data)
        self.guard_diagnostic = diagnostic
        if (
            diagnostic.startswith('CURRENT_FOOTPRINT')
            or diagnostic.startswith('PREDICTED_FOOTPRINT')
        ):
            self.guard_events.append(diagnostic)

    def _dynamics_callback(self, message: String) -> None:
        try:
            self.dynamics = json.loads(message.data)
        except json.JSONDecodeError:
            self.dynamics = {}

    def _feedback_callback(self, message) -> None:
        state = message.feedback.state
        item = {
            'state': str(state.state),
            'step_index': int(state.step_index),
            'step_name': str(state.step_name),
            'message': str(state.message),
        }
        if not self.feedback or item != self.feedback[-1]:
            self.feedback.append(item)

    def current_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return the latest planar body pose."""
        if self.pose is None:
            return None
        return (
            float(self.pose.pose.pose.position.x),
            float(self.pose.pose.pose.position.y),
            yaw_from_odometry(self.pose),
        )

    def sample(
        self,
        elapsed_sec: float,
        rectangles: Sequence[Rectangle],
        footprint_offset: float,
        hard_radius: float,
        guard_radius: float,
    ) -> Optional[Dict]:
        """Create one flat CSV-compatible observation."""
        pose = self.current_pose()
        if pose is None:
            return None
        circle_distance = minimum_double_circle_distance(
            *pose,
            rectangles,
            footprint_offset,
        )
        base_velocity = self.dynamics.get('base_velocity', [0.0] * 6)
        return {
            'elapsed_sec': elapsed_sec,
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
            'candidate_vx': float(self.candidate.linear.x),
            'candidate_vy': float(self.candidate.linear.y),
            'candidate_wz': float(self.candidate.angular.z),
            'safe_vx': float(self.safe.linear.x),
            'safe_vy': float(self.safe.linear.y),
            'safe_wz': float(self.safe.angular.z),
            'base_vx': float(base_velocity[0]),
            'base_vy': float(base_velocity[1]),
            'collision_stop': self.collision_stop,
            'guard_state': self.guard_state,
            'circle_center_obstacle_distance_m': circle_distance,
            'scan_hard_clearance_m': circle_distance - hard_radius,
            'guard_clearance_m': circle_distance - guard_radius,
            'contact_count': int(self.dynamics.get('contact_count', 0)),
            'obstacle_contact_count': int(
                self.dynamics.get('obstacle_contact_count', 0)
            ),
            'obstacle_contact_event_count': int(
                self.dynamics.get('obstacle_contact_event_count', 0)
            ),
            'obstacle_contact_peak_force_n': float(
                self.dynamics.get('obstacle_contact_peak_force_n', 0.0)
            ),
        }


def longest_commanded_stationary_period(
    samples: Sequence[Dict],
    displacement_tolerance: float = 0.03,
) -> float:
    """Measure the longest commanded period with negligible translation."""
    longest = 0.0
    anchor = None
    started = None
    for sample in samples:
        commanded = (
            math.hypot(
                sample['candidate_vx'],
                sample['candidate_vy'],
            ) > 0.03
            or abs(sample['candidate_wz']) > 0.08
        )
        if not commanded:
            anchor = None
            started = None
            continue
        point = (sample['x'], sample['y'])
        if anchor is None:
            anchor = point
            started = sample['elapsed_sec']
            continue
        if math.hypot(point[0] - anchor[0], point[1] - anchor[1]) > (
            displacement_tolerance
        ):
            anchor = point
            started = sample['elapsed_sec']
            continue
        longest = max(longest, sample['elapsed_sec'] - started)
    return longest


def summarise(
    *,
    samples: Sequence[Dict],
    action_success: bool,
    action_message: str,
    target: Tuple[float, float],
    doorway_x: float,
    guard_events: Sequence[str],
    trajectory_count: int,
    feedback: Sequence[Dict],
) -> Dict:
    """Build the stable machine-readable acceptance summary."""
    final = samples[-1]
    doorway = min(samples, key=lambda item: abs(item['x'] - doorway_x))
    return {
        'schema_version': 1,
        'action_success': action_success,
        'action_message': action_message,
        'duration_sec': final['elapsed_sec'],
        'final_error_m': math.hypot(
            target[0] - final['x'],
            target[1] - final['y'],
        ),
        'minimum_circle_center_obstacle_distance_m': min(
            item['circle_center_obstacle_distance_m'] for item in samples
        ),
        'minimum_scan_hard_clearance_m': min(
            item['scan_hard_clearance_m'] for item in samples
        ),
        'minimum_guard_clearance_m': min(
            item['guard_clearance_m'] for item in samples
        ),
        'doorway_sample': {
            'x': doorway['x'],
            'y': doorway['y'],
            'yaw': doorway['yaw'],
            'distance_from_doorway_x_m': abs(
                doorway['x'] - doorway_x
            ),
            'guard_clearance_m': doorway['guard_clearance_m'],
        },
        'collision_stop_sample_count': sum(
            bool(item['collision_stop']) for item in samples
        ),
        'predicted_stop_event_count': sum(
            item.startswith('PREDICTED_FOOTPRINT')
            for item in guard_events
        ),
        'rotation_recovery_event_count': sum(
            'recovery=(' in item for item in guard_events
        ),
        'current_stop_event_count': sum(
            item.startswith('CURRENT_FOOTPRINT')
            for item in guard_events
        ),
        'guard_events': list(guard_events),
        'obstacle_contact_event_count': max(
            item['obstacle_contact_event_count'] for item in samples
        ),
        'obstacle_contact_peak_force_n': max(
            item['obstacle_contact_peak_force_n'] for item in samples
        ),
        'longest_commanded_stationary_sec': (
            longest_commanded_stationary_period(samples)
        ),
        'trajectory_messages': trajectory_count,
        'feedback': list(feedback),
    }


def parse_arguments(
    arguments: Optional[Sequence[str]] = None,
) -> argparse.Namespace:
    """Parse probe-only arguments without leaking them into ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system-config', required=True, type=Path)
    parser.add_argument(
        '--mission-id',
        default='narrow_passage_benchmark',
    )
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--timeout-sec', type=float, default=150.0)
    parser.add_argument('--target-x', type=float, default=-7.0)
    parser.add_argument('--target-y', type=float, default=0.0)
    parser.add_argument('--doorway-x', type=float, default=-12.5)
    parser.add_argument('--footprint-offset', type=float, default=0.18)
    parser.add_argument('--hard-radius', type=float, default=0.25)
    parser.add_argument('--guard-radius', type=float, default=0.30)
    return parser.parse_args(arguments)


def main(arguments: Optional[Sequence[str]] = None) -> int:
    """Run the configured mission, save evidence, and return acceptance."""
    args = parse_arguments(arguments)
    args.output_directory.mkdir(parents=True, exist_ok=True)
    rectangles = load_obstacle_rectangles(args.system_config)
    if not rectangles:
        raise RuntimeError('system configuration has no obstacle rectangles')

    rclpy.init(args=[])
    node = ClearanceDynamicsProbe()
    samples: List[Dict] = []
    action_success = False
    action_message = 'probe did not receive a mission result'
    try:
        ready_deadline = time.monotonic() + 45.0
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose is not None and node.client.server_is_ready():
                break
        if node.pose is None or not node.client.server_is_ready():
            raise RuntimeError('timed out waiting for the complete system')

        goal = RunMission.Goal()
        goal.mission_id = args.mission_id
        goal.restart = True
        send_future = node.client.send_goal_async(
            goal,
            feedback_callback=node._feedback_callback,
        )
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('mission request was rejected')

        result_future = goal_handle.get_result_async()
        started = time.monotonic()
        next_sample = started
        while (
            not result_future.done()
            and time.monotonic() - started < args.timeout_sec
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_sample:
                continue
            sample = node.sample(
                now - started,
                rectangles,
                args.footprint_offset,
                args.hard_radius,
                args.guard_radius,
            )
            if sample is not None:
                samples.append(sample)
            next_sample = now + 0.05

        if result_future.done() and result_future.result() is not None:
            result = result_future.result().result
            action_success = bool(result.success)
            action_message = str(result.message)
        else:
            action_message = f'probe timeout after {args.timeout_sec:.1f}s'
            cancel_future = goal_handle.cancel_goal_async()
            cancel_deadline = time.monotonic() + 2.0
            while (
                not cancel_future.done()
                and time.monotonic() < cancel_deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
        final_sample = node.sample(
            time.monotonic() - started,
            rectangles,
            args.footprint_offset,
            args.hard_radius,
            args.guard_radius,
        )
        if final_sample is not None:
            samples.append(final_sample)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if not samples:
        raise RuntimeError('no dynamics samples were collected')
    with (
        args.output_directory / 'clearance_samples.csv'
    ).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    report = summarise(
        samples=samples,
        action_success=action_success,
        action_message=action_message,
        target=(args.target_x, args.target_y),
        doorway_x=args.doorway_x,
        guard_events=node.guard_events,
        trajectory_count=node.trajectory_count,
        feedback=node.feedback,
    )
    (
        args.output_directory / 'clearance_summary.json'
    ).write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    clean = (
        action_success
        and report['obstacle_contact_event_count'] == 0
        and report['current_stop_event_count'] == 0
    )
    return 0 if clean else 2


if __name__ == '__main__':
    raise SystemExit(main())
