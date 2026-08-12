#!/usr/bin/env python3
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

"""Record repeatable free-navigation and wheel-leg motion metrics."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String


DEFAULT_TARGETS = (
    ('straight_forward_2p5m', -34.5, 0.0),
    ('reverse_heading_return_2p5m', -37.0, 0.0),
    ('right_turn_90_and_straight_6m', -37.0, -6.0),
    ('left_turn_90_and_straight_3m', -34.0, -6.0),
    ('reverse_heading_return_3m', -37.0, -6.0),
    ('left_turn_90_return_home_6m', -37.0, 0.0),
)

LEG_INDICES = (0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14)
WHEEL_INDICES = (3, 7, 11, 15)


def _yaw_from_odometry(message: Odometry) -> float:
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


def _normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _percentile(values, ratio: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, round(ratio * (len(ordered) - 1))),
    )
    return float(ordered[index])


def _rms(values) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


class NavigationMotionProbe(Node):
    """Collect the command, body, joint, mode, and collision streams."""

    def __init__(self) -> None:
        super().__init__('m20_navigation_motion_probe')
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            10,
        )
        self.pose = None
        self.raw_command = Twist()
        self.safe_command = Twist()
        self.sdk_command = Twist()
        self.joint_state = None
        self.mode = 'UNKNOWN'
        self.tracking_direction = 'UNKNOWN'
        self.collision_stop = False
        self.guard_diagnostic = 'NOT_READY'
        self.guard_events = []
        self.heading_error = 0.0
        self.heading_aligning = False
        self.backend_fault = ''
        self.dynamics = {}
        self.velocity_feedback = {}
        self.trajectory_count = 0
        self.create_subscription(
            Odometry,
            '/m20/sim/body_pose',
            self._pose_callback,
            50,
        )
        self.create_subscription(
            Twist,
            '/m20/navigation/cmd_vel_raw',
            lambda message: setattr(self, 'raw_command', message),
            50,
        )
        self.create_subscription(
            Twist,
            '/m20/control/cmd_vel_safe',
            lambda message: setattr(self, 'safe_command', message),
            50,
        )
        self.create_subscription(
            Twist,
            '/m20/locomotion/cmd_vel_sdk',
            lambda message: setattr(self, 'sdk_command', message),
            50,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            lambda message: setattr(self, 'joint_state', message),
            50,
        )
        self.create_subscription(
            String,
            '/m20/locomotion/mode',
            lambda message: setattr(self, 'mode', str(message.data)),
            latched,
        )
        self.create_subscription(
            String,
            '/planning/tracking_direction',
            lambda message: setattr(
                self,
                'tracking_direction',
                str(message.data),
            ),
            10,
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
            '/m20/control/collision_guard_diagnostic',
            self._guard_diagnostic_callback,
            latched,
        )
        self.create_subscription(
            Float64,
            '/m20/navigation/heading_error',
            lambda message: setattr(
                self,
                'heading_error',
                float(message.data),
            ),
            50,
        )
        self.create_subscription(
            Bool,
            '/m20/navigation/heading_aligning',
            lambda message: setattr(
                self,
                'heading_aligning',
                bool(message.data),
            ),
            latched,
        )
        self.create_subscription(
            String,
            '/m20/sim/dynamics_state',
            self._dynamics_callback,
            20,
        )
        self.create_subscription(
            String,
            '/m20/sim/backend_fault',
            lambda message: setattr(
                self,
                'backend_fault',
                str(message.data),
            ),
            latched,
        )
        self.create_subscription(
            String,
            '/m20/navigation/velocity_feedback_state',
            self._velocity_feedback_callback,
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

    def _pose_callback(self, message: Odometry) -> None:
        self.pose = message

    def _dynamics_callback(self, message: String) -> None:
        try:
            self.dynamics = json.loads(message.data)
        except json.JSONDecodeError:
            self.dynamics = {}

    def _velocity_feedback_callback(self, message: String) -> None:
        try:
            diagnostic = json.loads(message.data)
            self.velocity_feedback = (
                diagnostic if isinstance(diagnostic, dict) else {}
            )
        except json.JSONDecodeError:
            self.velocity_feedback = {}

    def _feedback_component(self, field: str, index: int):
        vector = self.velocity_feedback.get(field)
        if not isinstance(vector, list) or len(vector) <= index:
            return None
        try:
            value = float(vector[index])
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def _guard_diagnostic_callback(self, message: String) -> None:
        diagnostic = str(message.data)
        self.guard_diagnostic = diagnostic
        if diagnostic.startswith(
            (
                'CURRENT_FOOTPRINT',
                'PREDICTED_FOOTPRINT',
                'RECOVERY_BUDGET_EXHAUSTED',
            )
        ):
            self.guard_events.append(diagnostic)

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = 0.0
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        self.goal_publisher.publish(message)

    def current_pose(self):
        if self.pose is None:
            return None
        return (
            float(self.pose.pose.pose.position.x),
            float(self.pose.pose.pose.position.y),
            _yaw_from_odometry(self.pose),
        )

    def sample(self, test_name: str, elapsed: float):
        pose = self.current_pose()
        if pose is None:
            return None
        joint_velocity = (
            list(self.joint_state.velocity)
            if self.joint_state is not None
            and len(self.joint_state.velocity) >= 16
            else [0.0] * 16
        )
        leg_max = max(abs(joint_velocity[index]) for index in LEG_INDICES)
        wheel_max = max(
            abs(joint_velocity[index]) for index in WHEEL_INDICES
        )
        return {
            'test': test_name,
            'elapsed_sec': elapsed,
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
            'raw_vx': float(self.raw_command.linear.x),
            'raw_vy': float(self.raw_command.linear.y),
            'raw_wz': float(self.raw_command.angular.z),
            'safe_vx': float(self.safe_command.linear.x),
            'safe_vy': float(self.safe_command.linear.y),
            'safe_wz': float(self.safe_command.angular.z),
            'sdk_vx': float(self.sdk_command.linear.x),
            'sdk_vy': float(self.sdk_command.linear.y),
            'sdk_wz': float(self.sdk_command.angular.z),
            'mode': self.mode,
            'tracking_direction': self.tracking_direction,
            'leg_velocity_max': leg_max,
            'wheel_velocity_max': wheel_max,
            'collision_stop': self.collision_stop,
            'guard_diagnostic': self.guard_diagnostic,
            'heading_error': self.heading_error,
            'heading_aligning': self.heading_aligning,
            'backend_fault': self.backend_fault,
            'feedback_enabled': bool(
                self.velocity_feedback.get('enabled', False)
            ),
            'feedback_active': bool(
                self.velocity_feedback.get('active', False)
            ),
            'feedback_reason': str(
                self.velocity_feedback.get('reason', 'UNAVAILABLE')
            ),
            'feedback_reference_vx': self._feedback_component(
                'reference', 0
            ),
            'feedback_reference_wz': self._feedback_component(
                'reference', 2
            ),
            'feedback_measurement_vx': self._feedback_component(
                'measurement', 0
            ),
            'feedback_measurement_wz': self._feedback_component(
                'measurement', 2
            ),
            'feedback_correction_vx': self._feedback_component(
                'correction', 0
            ),
            'feedback_correction_wz': self._feedback_component(
                'correction', 2
            ),
            # Total MuJoCo contacts include normal wheel/foot-ground contacts.
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


def _cross_track_distance(start, finish, point) -> float:
    dx = finish[0] - start[0]
    dy = finish[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1.0e-9:
        return 0.0
    return abs(
        dy * point[0] - dx * point[1]
        + finish[0] * start[1] - finish[1] * start[0]
    ) / length


def _summarise_test(
    name: str,
    start,
    target,
    samples,
    duration: float,
    success: bool,
    trajectories: int,
    guard_events,
):
    moving = [
        sample for sample in samples
        if abs(sample['sdk_vx']) > 0.01
        or abs(sample['sdk_vy']) > 0.01
        or abs(sample['sdk_wz']) > 0.02
    ]
    direct_yaw = math.atan2(target[1] - start[1], target[0] - start[0])
    yaw_errors = [
        abs(_normalise_angle(sample['yaw'] - direct_yaw))
        for sample in moving
    ]
    raw_wz = [abs(sample['raw_wz']) for sample in moving]
    path_length = 0.0
    for left, right in zip(samples, samples[1:]):
        path_length += math.hypot(
            right['x'] - left['x'],
            right['y'] - left['y'],
        )
    direct_length = math.hypot(target[0] - start[0], target[1] - start[1])
    simultaneous = [
        sample for sample in moving
        if sample['leg_velocity_max'] > 0.05
        and sample['wheel_velocity_max'] > 0.20
    ]
    modes = {}
    for sample in moving:
        modes[sample['mode']] = modes.get(sample['mode'], 0) + 1
    mode_transitions = sum(
        left['mode'] != right['mode']
        for left, right in zip(moving, moving[1:])
    )
    heading_errors = [
        abs(sample['heading_error']) for sample in moving
    ]
    alignment_events = sum(
        not left['heading_aligning'] and right['heading_aligning']
        for left, right in zip(samples, samples[1:])
    )
    if samples and samples[0]['heading_aligning']:
        alignment_events += 1
    mode_fraction = {
        mode: count / max(1, len(moving))
        for mode, count in sorted(modes.items())
    }
    directions = {}
    for sample in moving:
        direction = sample['tracking_direction']
        directions[direction] = directions.get(direction, 0) + 1
    tracking_direction_fraction = {
        direction: count / max(1, len(moving))
        for direction, count in sorted(directions.items())
    }
    feedback_samples = [
        sample for sample in moving
        if sample['feedback_reference_vx'] is not None
        and sample['feedback_reference_wz'] is not None
        and sample['feedback_measurement_vx'] is not None
        and sample['feedback_measurement_wz'] is not None
    ]
    linear_tracking_errors = [
        sample['feedback_reference_vx']
        - sample['feedback_measurement_vx']
        for sample in feedback_samples
    ]
    yaw_tracking_errors = [
        sample['feedback_reference_wz']
        - sample['feedback_measurement_wz']
        for sample in feedback_samples
    ]
    linear_corrections = [
        float(sample['feedback_correction_vx'] or 0.0)
        for sample in feedback_samples
    ]
    yaw_corrections = [
        float(sample['feedback_correction_wz'] or 0.0)
        for sample in feedback_samples
    ]
    feedback_reasons = {}
    for sample in moving:
        reason = sample['feedback_reason']
        feedback_reasons[reason] = feedback_reasons.get(reason, 0) + 1
    final = samples[-1]
    collision_spans = []
    collision_started = None
    collision_last = None
    for sample in samples:
        if sample['collision_stop']:
            if collision_started is None:
                collision_started = sample['elapsed_sec']
            collision_last = sample['elapsed_sec']
        elif collision_started is not None:
            collision_spans.append(collision_last - collision_started)
            collision_started = None
            collision_last = None
    if collision_started is not None:
        collision_spans.append(collision_last - collision_started)
    return {
        'name': name,
        'success': success,
        'duration_sec': duration,
        'start': [start[0], start[1], start[2]],
        'target': [target[0], target[1]],
        'final': [final['x'], final['y'], final['yaw']],
        'final_error_m': math.hypot(
            target[0] - final['x'],
            target[1] - final['y'],
        ),
        'direct_length_m': direct_length,
        'travel_length_m': path_length,
        'path_length_ratio': path_length / max(direct_length, 1.0e-9),
        'max_cross_track_m': max(
            _cross_track_distance(
                (start[0], start[1]),
                target,
                (sample['x'], sample['y']),
            )
            for sample in samples
        ),
        'moving_sample_count': len(moving),
        'mean_abs_yaw_error_deg': math.degrees(
            statistics.fmean(yaw_errors) if yaw_errors else 0.0
        ),
        'p95_abs_yaw_error_deg': math.degrees(
            _percentile(yaw_errors, 0.95)
        ),
        'raw_wz_rms': _rms(raw_wz),
        'raw_wz_p95': _percentile(raw_wz, 0.95),
        'raw_wz_above_0p02_fraction': (
            sum(value > 0.02 for value in raw_wz) / max(1, len(raw_wz))
        ),
        'controller_heading_error_mean_deg': math.degrees(
            statistics.fmean(heading_errors) if heading_errors else 0.0
        ),
        'controller_heading_error_p95_deg': math.degrees(
            _percentile(heading_errors, 0.95)
        ),
        'controller_heading_error_max_deg': math.degrees(
            max(heading_errors, default=0.0)
        ),
        'heading_alignment_sample_fraction': (
            sum(bool(sample['heading_aligning']) for sample in moving)
            / max(1, len(moving))
        ),
        'heading_alignment_events': alignment_events,
        'leg_velocity_rms': _rms(
            [sample['leg_velocity_max'] for sample in moving]
        ),
        'wheel_velocity_rms': _rms(
            [sample['wheel_velocity_max'] for sample in moving]
        ),
        'wheel_leg_simultaneous_fraction': (
            len(simultaneous) / max(1, len(moving))
        ),
        'collision_stop_samples': sum(
            bool(sample['collision_stop']) for sample in samples
        ),
        'longest_collision_stop_sec': max(collision_spans, default=0.0),
        'predicted_stop_event_count': sum(
            item.startswith('PREDICTED_FOOTPRINT')
            for item in guard_events
        ),
        'current_stop_event_count': sum(
            item.startswith('CURRENT_FOOTPRINT')
            for item in guard_events
        ),
        'recovery_event_count': sum(
            'recovery=(' in item for item in guard_events
        ),
        'recovery_budget_exhausted_event_count': sum(
            item.startswith('RECOVERY_BUDGET_EXHAUSTED')
            for item in guard_events
        ),
        'guard_events': list(guard_events),
        'max_contact_count': max(
            sample['contact_count'] for sample in samples
        ),
        'max_obstacle_contact_count': max(
            sample['obstacle_contact_count'] for sample in samples
        ),
        'obstacle_contact_event_count': max(
            sample['obstacle_contact_event_count'] for sample in samples
        ),
        'obstacle_contact_peak_force_n': max(
            sample['obstacle_contact_peak_force_n'] for sample in samples
        ),
        'backend_fault_samples': sum(
            bool(sample['backend_fault']) for sample in samples
        ),
        'backend_fault_reasons': sorted({
            sample['backend_fault']
            for sample in samples
            if sample['backend_fault']
        }),
        'trajectory_messages': trajectories,
        'mode_transitions': mode_transitions,
        'mode_fraction': mode_fraction,
        'tracking_direction_fraction': tracking_direction_fraction,
        'velocity_feedback_valid_samples': len(feedback_samples),
        'velocity_feedback_enabled_sample_fraction': (
            sum(bool(sample['feedback_enabled']) for sample in moving)
            / max(1, len(moving))
        ),
        'velocity_feedback_active_sample_fraction': (
            sum(bool(sample['feedback_active']) for sample in moving)
            / max(1, len(moving))
        ),
        'velocity_feedback_reason_counts': dict(
            sorted(feedback_reasons.items())
        ),
        'velocity_tracking_linear_error_rms_m_s': _rms(
            linear_tracking_errors
        ),
        'velocity_tracking_yaw_error_rms_rad_s': _rms(
            yaw_tracking_errors
        ),
        'velocity_feedback_linear_correction_rms_m_s': _rms(
            linear_corrections
        ),
        'velocity_feedback_yaw_correction_rms_rad_s': _rms(
            yaw_corrections
        ),
        'velocity_feedback_linear_correction_peak_m_s': max(
            (abs(value) for value in linear_corrections),
            default=0.0,
        ),
        'velocity_feedback_yaw_correction_peak_rad_s': max(
            (abs(value) for value in yaw_corrections),
            default=0.0,
        ),
    }


def _parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output-directory',
        type=Path,
        required=True,
    )
    parser.add_argument('--goal-timeout-sec', type=float, default=90.0)
    parser.add_argument('--settle-sec', type=float, default=1.5)
    parser.add_argument(
        '--target',
        action='append',
        nargs=3,
        metavar=('NAME', 'X', 'Y'),
        help=(
            'run only this named map-frame target; repeat for multiple goals'
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_arguments()
    targets = (
        tuple(
            (str(item[0]), float(item[1]), float(item[2]))
            for item in args.target
        )
        if args.target
        else DEFAULT_TARGETS
    )
    args.output_directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = NavigationMotionProbe()
    all_samples = []
    summaries = []
    try:
        wait_deadline = time.monotonic() + 30.0
        while node.pose is None and time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            raise RuntimeError('timed out waiting for /m20/sim/body_pose')

        time.sleep(1.0)
        for name, target_x, target_y in targets:
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            start = node.current_pose()
            target = (target_x, target_y)
            direct_yaw = math.atan2(
                target_y - start[1],
                target_x - start[0],
            )
            trajectories_before = node.trajectory_count
            guard_events_before = len(node.guard_events)
            node.publish_goal(target_x, target_y, direct_yaw)
            print(
                f'START {name}: pose=({start[0]:.3f},'
                f'{start[1]:.3f},{math.degrees(start[2]):.1f}deg) '
                f'target=({target_x:.3f},{target_y:.3f})',
                flush=True,
            )
            started_at = time.monotonic()
            next_sample = started_at
            settled_since = None
            samples = []
            success = False
            while time.monotonic() - started_at < args.goal_timeout_sec:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if now >= next_sample:
                    sample = node.sample(name, now - started_at)
                    if sample is not None:
                        samples.append(sample)
                        all_samples.append(sample)
                    next_sample = now + 0.05
                pose = node.current_pose()
                distance = math.hypot(
                    target_x - pose[0],
                    target_y - pose[1],
                )
                dynamics_velocity = node.dynamics.get(
                    'base_velocity',
                    [0.0] * 6,
                )
                planar_speed = math.hypot(
                    float(dynamics_velocity[0]),
                    float(dynamics_velocity[1]),
                )
                if distance <= 0.25 and planar_speed <= 0.08:
                    if settled_since is None:
                        settled_since = now
                    elif now - settled_since >= args.settle_sec:
                        success = True
                        break
                else:
                    settled_since = None
                if node.backend_fault:
                    print(
                        f'ABORT {name}: backend fault='
                        f'{node.backend_fault}',
                        flush=True,
                    )
                    break
            duration = time.monotonic() - started_at
            if not samples:
                raise RuntimeError(f'no samples collected for {name}')
            summary = _summarise_test(
                name,
                start,
                target,
                samples,
                duration,
                success,
                node.trajectory_count - trajectories_before,
                node.guard_events[guard_events_before:],
            )
            summaries.append(summary)
            print(
                f'END {name}: success={success} '
                f'duration={duration:.2f}s '
                f'error={summary["final_error_m"]:.3f}m '
                f'cross_track={summary["max_cross_track_m"]:.3f}m '
                'simultaneous='
                f'{summary["wheel_leg_simultaneous_fraction"]:.3f}',
                flush=True,
            )
            if not success:
                break
            settle_deadline = time.monotonic() + 1.0
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    fieldnames = list(all_samples[0])
    with (
        args.output_directory / 'navigation_motion_samples.csv'
    ).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_samples)
    report = {
        'schema_version': 6,
        'sample_period_sec': 0.05,
        'leg_motion_threshold_rad_s': 0.05,
        'wheel_motion_threshold_rad_s': 0.20,
        'tests': summaries,
    }
    (
        args.output_directory / 'navigation_motion_summary.json'
    ).write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    passed = (
        bool(summaries)
        and all(item['success'] for item in summaries)
        and all(
            item['current_stop_event_count'] == 0
            and item['recovery_budget_exhausted_event_count'] == 0
            and item['backend_fault_samples'] == 0
            and item['longest_collision_stop_sec'] <= 8.0
            and item['obstacle_contact_event_count'] == 0
            for item in summaries
        )
    )
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
