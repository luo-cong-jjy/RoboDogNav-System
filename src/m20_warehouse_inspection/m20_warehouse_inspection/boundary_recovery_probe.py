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

"""Cold-start probe for M20 recovery at warehouse map boundaries."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, Twist
from m20_warehouse_interfaces.action import NavigateFloor
from m20_warehouse_interfaces.msg import FloorState, LocalSensingState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .clearance_dynamics_probe import (
    load_obstacle_rectangles,
    minimum_double_circle_distance,
    yaw_from_odometry,
)


def latched_qos() -> QoSProfile:
    """Return the project's reliable transient-local state QoS."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def longest_period(
    samples: Sequence[Dict],
    key: str,
    expected: bool = True,
) -> float:
    """Return the longest contiguous sampled period matching a boolean."""
    longest = 0.0
    started: Optional[float] = None
    previous: Optional[float] = None
    for sample in samples:
        elapsed = float(sample['elapsed_sec'])
        if bool(sample[key]) == expected:
            if started is None:
                started = elapsed
            previous = elapsed
            longest = max(longest, previous - started)
        else:
            started = None
            previous = None
    return longest


def fallback_scan_goals(
    scan_goals: Sequence[Dict],
    route_states: Sequence[Dict],
) -> List[Dict]:
    """
    Keep goals emitted by the grid fallback, excluding the direct goal.

    The production launch intentionally remaps the route planner output onto
    the native SCAN goal topic.  The first subgoal is published immediately
    before TRACKING_ROUTE, so retain a short scheduling tolerance.
    """
    tracking_times = [
        float(item['elapsed_sec'])
        for item in route_states
        if str(item['state']) == 'TRACKING_ROUTE'
    ]
    if not tracking_times:
        return []
    fallback_started = min(tracking_times)
    return [
        dict(item)
        for item in scan_goals
        if float(item['elapsed_sec']) >= fallback_started - 0.20
    ]


def build_report(
    *,
    case_name: str,
    samples: Sequence[Dict],
    target: Tuple[float, float],
    action_success: bool,
    action_error_code: int,
    action_message: str,
    guard_events: Sequence[str],
    route_states: Sequence[Dict],
    scan_goals: Sequence[Dict],
    feedback: Sequence[Dict],
    backend_faults: Sequence[str],
    stop_stable_sec: float,
) -> Dict:
    """Create a stable machine-readable risk and recovery summary."""
    if not samples:
        raise ValueError('at least one dynamics sample is required')
    scan_goals = fallback_scan_goals(scan_goals, route_states)
    final = samples[-1]
    later_scan_goals = list(scan_goals[1:])
    stopped_scan_goals = [
        item
        for item in later_scan_goals
        if bool(item.get('stop_required_before_publish', True))
    ]
    continuous_scan_goals = [
        item
        for item in later_scan_goals
        if not bool(item.get('stop_required_before_publish', True))
    ]
    stop_floor = max(0.0, stop_stable_sec - 0.05)
    stop_violations = [
        item
        for item in stopped_scan_goals
        if float(item['stationary_before_publish_sec']) < stop_floor
    ]
    route_state_names = [str(item['state']) for item in route_states]
    obstacle_contacts = max(
        int(item['obstacle_contact_event_count']) for item in samples
    )
    report = {
        'schema_version': 1,
        'case': case_name,
        'action_success': bool(action_success),
        'action_error_code': int(action_error_code),
        'action_message': action_message,
        'duration_sec': float(final['elapsed_sec']),
        'start_pose': {
            'x': float(samples[0]['x']),
            'y': float(samples[0]['y']),
            'yaw': float(samples[0]['yaw']),
        },
        'final_pose': {
            'x': float(final['x']),
            'y': float(final['y']),
            'yaw': float(final['yaw']),
        },
        'final_error_m': math.hypot(
            target[0] - float(final['x']),
            target[1] - float(final['y']),
        ),
        'minimum_obstacle_body_clearance_m': min(
            float(item['obstacle_body_clearance_m']) for item in samples
        ),
        'minimum_scan_inflation_clearance_m': min(
            float(item['scan_inflation_clearance_m']) for item in samples
        ),
        'minimum_guard_clearance_m': min(
            float(item['guard_clearance_m']) for item in samples
        ),
        'scan_inflated_region_sample_count': sum(
            bool(item['inside_scan_inflation']) for item in samples
        ),
        'longest_scan_inflated_region_sec': longest_period(
            samples, 'inside_scan_inflation'
        ),
        'obstacle_contact_event_count': obstacle_contacts,
        'obstacle_contact_peak_force_n': max(
            float(item['obstacle_contact_peak_force_n'])
            for item in samples
        ),
        'obstacle_contact_pairs': [
            item
            for item in str(final['obstacle_contact_pairs']).split('|')
            if item
        ],
        'current_footprint_guard_event_count': sum(
            item.startswith('CURRENT_FOOTPRINT') for item in guard_events
        ),
        'predicted_footprint_guard_event_count': sum(
            item.startswith('PREDICTED_FOOTPRINT') for item in guard_events
        ),
        'recovery_budget_exhausted_event_count': sum(
            item.startswith('RECOVERY_BUDGET_EXHAUSTED')
            for item in guard_events
        ),
        'raster_shell_escape_event_count': sum(
            item.startswith('RASTER_SHELL_ESCAPE')
            for item in guard_events
        ),
        'map_edge_inward_recovery_event_count': sum(
            item.startswith('MAP_EDGE_INWARD_RECOVERY')
            for item in guard_events
        ),
        'guard_events': list(guard_events),
        'fallback_triggered': (
            bool(scan_goals) or 'TRACKING_ROUTE' in route_state_names
        ),
        'route_states': list(route_states),
        'scan_goal_count': len(scan_goals),
        'scan_goals': list(scan_goals),
        'continuous_handoff_count': len(continuous_scan_goals),
        'intermediate_stop_check_count': len(stopped_scan_goals),
        'intermediate_stop_violation_count': len(stop_violations),
        'intermediate_stop_violations': stop_violations,
        'backend_faults': list(backend_faults),
        'feedback': list(feedback),
    }
    report['accepted'] = bool(
        report['action_success']
        and report['final_error_m'] <= 0.35
        and report['obstacle_contact_event_count'] == 0
        and not report['backend_faults']
        and report['intermediate_stop_violation_count'] == 0
    )
    return report


def aggregate_reports(reports: Sequence[Dict]) -> Dict:
    """Aggregate independent cold starts without hiding individual failures."""
    grouped: Dict[str, List[Dict]] = {}
    for report in reports:
        grouped.setdefault(str(report['case']), []).append(report)

    cases = {}
    for case_name, case_reports in sorted(grouped.items()):
        total = len(case_reports)
        accepted = sum(
            bool(item.get('accepted', False)) for item in case_reports
        )
        durations = [float(item['duration_sec']) for item in case_reports]
        cases[case_name] = {
            'runs': total,
            'accepted_runs': accepted,
            'observed_success_rate': accepted / total,
            'action_success_runs': sum(
                bool(item.get('action_success', False))
                for item in case_reports
            ),
            'contact_free_runs': sum(
                int(item.get('obstacle_contact_event_count', 0)) == 0
                for item in case_reports
            ),
            'fallback_coverage_runs': sum(
                bool(item.get('fallback_triggered', False))
                for item in case_reports
            ),
            'raster_shell_escape_runs': sum(
                int(item.get('raster_shell_escape_event_count', 0)) > 0
                for item in case_reports
            ),
            'map_edge_inward_recovery_runs': sum(
                int(item.get('map_edge_inward_recovery_event_count', 0)) > 0
                for item in case_reports
            ),
            'scan_inflation_entry_runs': sum(
                int(item.get('scan_inflated_region_sample_count', 0)) > 0
                for item in case_reports
            ),
            'intermediate_stop_clean_runs': sum(
                int(item.get('intermediate_stop_violation_count', 0)) == 0
                for item in case_reports
            ),
            'maximum_obstacle_contact_events': max(
                int(item.get('obstacle_contact_event_count', 0))
                for item in case_reports
            ),
            'worst_scan_inflation_clearance_m': min(
                float(item['minimum_scan_inflation_clearance_m'])
                for item in case_reports
            ),
            'longest_scan_inflated_region_sec': max(
                float(item.get('longest_scan_inflated_region_sec', 0.0))
                for item in case_reports
            ),
            'median_duration_sec': statistics.median(durations),
            'maximum_duration_sec': max(durations),
        }
    total = len(reports)
    accepted = sum(bool(item.get('accepted', False)) for item in reports)
    return {
        'schema_version': 1,
        'independent_cold_starts': total,
        'accepted_runs': accepted,
        'observed_success_rate': accepted / total if total else 0.0,
        'all_runs_accepted': bool(total and accepted == total),
        'cases': cases,
    }


class BoundaryRecoveryProbe(Node):
    """Observe navigation, fallback subgoals, guard state, and physics."""

    def __init__(self) -> None:
        """Subscribe to physical, planning, and safety evidence streams."""
        super().__init__('m20_boundary_recovery_probe')
        qos = latched_qos()
        self.client = ActionClient(
            self, NavigateFloor, '/m20/navigation/navigate'
        )
        self.floor: Optional[FloorState] = None
        self.sensing: Optional[LocalSensingState] = None
        self.pose: Optional[Odometry] = None
        self.backend_ready = False
        self.backend_faults: List[str] = []
        self.dynamics: Dict = {}
        self.candidate = Twist()
        self.safe = Twist()
        self.collision_stop = True
        self.guard_state = 'NOT_READY'
        self.guard_events: List[str] = []
        self.route_states: List[Dict] = []
        self.scan_goals: List[Dict] = []
        self.feedback: List[Dict] = []
        self.segment_hold = False
        self._segment_hold_seen_since_goal = False
        self._started_monotonic: Optional[float] = None
        self._last_moving_monotonic = time.monotonic()
        self._last_guard_diagnostic = ''

        self.create_subscription(
            FloorState,
            '/m20/map/state',
            lambda message: setattr(self, 'floor', message),
            qos,
        )
        self.create_subscription(
            LocalSensingState,
            '/m20/sensing/state',
            lambda message: setattr(self, 'sensing', message),
            qos,
        )
        self.create_subscription(
            Odometry,
            '/m20/sim/body_pose',
            self._pose_callback,
            50,
        )
        self.create_subscription(
            Bool,
            '/m20/sim/backend_ready',
            lambda message: setattr(
                self, 'backend_ready', bool(message.data)
            ),
            qos,
        )
        self.create_subscription(
            String,
            '/m20/sim/backend_fault',
            self._backend_fault_callback,
            qos,
        )
        self.create_subscription(
            String,
            '/m20/sim/dynamics_state',
            self._dynamics_callback,
            20,
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
                self, 'collision_stop', bool(message.data)
            ),
            qos,
        )
        self.create_subscription(
            String,
            '/m20/control/collision_guard_state',
            lambda message: setattr(self, 'guard_state', str(message.data)),
            qos,
        )
        self.create_subscription(
            String,
            '/m20/control/collision_guard_diagnostic',
            self._guard_callback,
            qos,
        )
        self.create_subscription(
            String,
            '/m20/navigation/global_route_state',
            self._route_state_callback,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            '/m20/navigation/scan_goal_internal',
            self._scan_goal_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/m20/control/route_segment_hold',
            self._segment_hold_callback,
            qos,
        )

    def elapsed(self) -> float:
        """Return probe elapsed time, or zero before the action starts."""
        if self._started_monotonic is None:
            return 0.0
        return time.monotonic() - self._started_monotonic

    def _pose_callback(self, message: Odometry) -> None:
        self.pose = message
        twist = message.twist.twist
        if (
            math.hypot(float(twist.linear.x), float(twist.linear.y)) > 0.04
            or abs(float(twist.angular.z)) > 0.08
        ):
            self._last_moving_monotonic = time.monotonic()

    def _backend_fault_callback(self, message: String) -> None:
        fault = str(message.data).strip()
        if fault and fault not in self.backend_faults:
            self.backend_faults.append(fault)

    def _dynamics_callback(self, message: String) -> None:
        try:
            self.dynamics = json.loads(message.data)
        except json.JSONDecodeError:
            self.dynamics = {}

    def _guard_callback(self, message: String) -> None:
        diagnostic = str(message.data)
        if diagnostic == self._last_guard_diagnostic:
            return
        self._last_guard_diagnostic = diagnostic
        if diagnostic.startswith(
            ('CURRENT_FOOTPRINT', 'PREDICTED_FOOTPRINT',
             'RECOVERY_BUDGET_EXHAUSTED', 'RASTER_SHELL_ESCAPE',
             'MAP_EDGE_INWARD_RECOVERY')
        ):
            self.guard_events.append(diagnostic)

    def _route_state_callback(self, message: String) -> None:
        state = str(message.data)
        if self.route_states and self.route_states[-1]['state'] == state:
            return
        self.route_states.append(
            {'elapsed_sec': self.elapsed(), 'state': state}
        )

    def _segment_hold_callback(self, message: Bool) -> None:
        self.segment_hold = bool(message.data)
        if self.segment_hold:
            self._segment_hold_seen_since_goal = True

    def _scan_goal_callback(self, message: PoseStamped) -> None:
        now = time.monotonic()
        x = float(message.pose.position.x)
        y = float(message.pose.position.y)
        if self.scan_goals:
            previous = self.scan_goals[-1]
            if (
                math.hypot(x - previous['x'], y - previous['y']) <= 0.01
                and self.elapsed() - previous['elapsed_sec'] <= 0.50
            ):
                return
        speed = self.pose.twist.twist if self.pose is not None else Twist()
        self.scan_goals.append(
            {
                'elapsed_sec': self.elapsed(),
                'x': x,
                'y': y,
                'stationary_before_publish_sec': max(
                    0.0, now - self._last_moving_monotonic
                ),
                'linear_speed_mps': math.hypot(
                    float(speed.linear.x), float(speed.linear.y)
                ),
                'angular_speed_radps': abs(float(speed.angular.z)),
                'stop_required_before_publish': bool(
                    self._segment_hold_seen_since_goal
                ),
            }
        )
        self._segment_hold_seen_since_goal = False

    def _feedback_callback(self, message) -> None:
        feedback = message.feedback
        item = {
            'elapsed_sec': self.elapsed(),
            'state': str(feedback.state),
            'distance_remaining_m': float(feedback.distance_remaining),
            'route_update_count': int(feedback.route_update_count),
            'message': str(feedback.message),
        }
        if (
            not self.feedback
            or self.feedback[-1]['state'] != item['state']
            or self.feedback[-1]['message'] != item['message']
        ):
            self.feedback.append(item)

    def ready(self) -> bool:
        """Return whether the complete physical navigation graph is ready."""
        return bool(
            self.floor is not None
            and self.floor.ready
            and self.floor.floor_id == 'F1'
            and self.sensing is not None
            and self.sensing.ready
            and self.sensing.floor_id == 'F1'
            and self.sensing.generation == self.floor.generation
            and self.pose is not None
            and self.backend_ready
            and bool(self.dynamics)
            and self.client.server_is_ready()
        )

    def sample(
        self,
        rectangles,
        footprint_offset: float,
        hard_radius: float,
        guard_radius: float,
    ) -> Optional[Dict]:
        """Capture one flat state sample for later independent inspection."""
        if self.pose is None:
            return None
        x = float(self.pose.pose.pose.position.x)
        y = float(self.pose.pose.pose.position.y)
        yaw = yaw_from_odometry(self.pose)
        centre_distance = minimum_double_circle_distance(
            x, y, yaw, rectangles, footprint_offset
        )
        twist = self.pose.twist.twist
        return {
            'elapsed_sec': self.elapsed(),
            'x': x,
            'y': y,
            'yaw': yaw,
            'body_linear_speed_mps': math.hypot(
                float(twist.linear.x), float(twist.linear.y)
            ),
            'body_angular_speed_radps': abs(float(twist.angular.z)),
            'candidate_vx': float(self.candidate.linear.x),
            'candidate_vy': float(self.candidate.linear.y),
            'candidate_wz': float(self.candidate.angular.z),
            'safe_vx': float(self.safe.linear.x),
            'safe_vy': float(self.safe.linear.y),
            'safe_wz': float(self.safe.angular.z),
            'collision_stop': bool(self.collision_stop),
            'guard_state': self.guard_state,
            'obstacle_body_clearance_m': centre_distance,
            'scan_inflation_clearance_m': centre_distance - hard_radius,
            'guard_clearance_m': centre_distance - guard_radius,
            'inside_scan_inflation': centre_distance < hard_radius,
            'obstacle_contact_count': int(
                self.dynamics.get('obstacle_contact_count', 0)
            ),
            'obstacle_contact_event_count': int(
                self.dynamics.get('obstacle_contact_event_count', 0)
            ),
            'obstacle_contact_peak_force_n': float(
                self.dynamics.get('obstacle_contact_peak_force_n', 0.0)
            ),
            'obstacle_contact_pairs': '|'.join(
                str(item)
                for item in self.dynamics.get('obstacle_contact_pairs', [])
            ),
        }


def parse_arguments(
    arguments: Optional[Sequence[str]] = None,
) -> argparse.Namespace:
    """Parse probe-only arguments without leaking them into ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True)
    parser.add_argument('--system-config', required=True, type=Path)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--target-x', required=True, type=float)
    parser.add_argument('--target-y', required=True, type=float)
    parser.add_argument('--target-yaw', type=float, default=0.0)
    parser.add_argument('--timeout-sec', type=float, default=240.0)
    parser.add_argument('--footprint-offset', type=float, default=0.18)
    parser.add_argument('--hard-radius', type=float, default=0.25)
    parser.add_argument('--guard-radius', type=float, default=0.30)
    parser.add_argument('--stop-stable-sec', type=float, default=0.30)
    raw_arguments = (
        list(sys.argv)
        if arguments is None
        else [sys.argv[0], *arguments]
    )
    return parser.parse_args(
        rclpy.utilities.remove_ros_args(raw_arguments)[1:]
    )


def main(arguments: Optional[Sequence[str]] = None) -> int:
    """Run one cold-start case, persist evidence, and return acceptance."""
    args = parse_arguments(arguments)
    args.output_directory.mkdir(parents=True, exist_ok=True)
    rectangles = load_obstacle_rectangles(args.system_config)
    if not rectangles:
        raise RuntimeError('system configuration has no obstacle rectangles')

    rclpy.init(args=[])
    node = BoundaryRecoveryProbe()
    samples: List[Dict] = []
    action_success = False
    action_error_code = NavigateFloor.Result.ERROR_INTERNAL
    action_message = 'probe did not receive a navigation result'
    try:
        ready_deadline = time.monotonic() + 60.0
        while time.monotonic() < ready_deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.10)
        if not node.ready() or node.floor is None:
            raise RuntimeError(
                'timed out waiting for complete MuJoCo navigation'
            )

        goal = NavigateFloor.Goal()
        goal.goal_id = f'boundary_recovery_{args.case}'
        goal.floor_id = node.floor.floor_id
        goal.map_generation = node.floor.generation
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = args.target_x
        goal.target_pose.pose.position.y = args.target_y
        goal.target_pose.pose.position.z = 0.59
        half_yaw = 0.5 * args.target_yaw
        goal.target_pose.pose.orientation.z = math.sin(half_yaw)
        goal.target_pose.pose.orientation.w = math.cos(half_yaw)
        goal.timeout_sec = float(args.timeout_sec)

        node._started_monotonic = time.monotonic()
        node._last_moving_monotonic = node._started_monotonic
        send_future = node.client.send_goal_async(
            goal, feedback_callback=node._feedback_callback
        )
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('boundary navigation request was rejected')

        result_future = goal_handle.get_result_async()
        next_sample = time.monotonic()
        while (
            not result_future.done()
            and node.elapsed() < args.timeout_sec + 10.0
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_sample:
                continue
            sample = node.sample(
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
            action_error_code = int(result.error_code)
            action_message = str(result.message)
        else:
            action_message = f'probe timeout after {args.timeout_sec:.1f}s'
            cancel_future = goal_handle.cancel_goal_async()
            cancel_deadline = time.monotonic() + 3.0
            while (
                not cancel_future.done()
                and time.monotonic() < cancel_deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)

        for _index in range(20):
            rclpy.spin_once(node, timeout_sec=0.02)
        final_sample = node.sample(
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
        raise RuntimeError('no boundary-recovery samples were collected')
    samples_path = args.output_directory / 'boundary_recovery_samples.csv'
    with samples_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    report = build_report(
        case_name=args.case,
        samples=samples,
        target=(args.target_x, args.target_y),
        action_success=action_success,
        action_error_code=action_error_code,
        action_message=action_message,
        guard_events=node.guard_events,
        route_states=node.route_states,
        scan_goals=node.scan_goals,
        feedback=node.feedback,
        backend_faults=node.backend_faults,
        stop_stable_sec=args.stop_stable_sec,
    )
    (args.output_directory / 'boundary_recovery_summary.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 0 if report['accepted'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
