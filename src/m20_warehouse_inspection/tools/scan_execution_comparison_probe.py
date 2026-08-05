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

"""Compare native SCAN kinematics with the M20 wheel-leg execution chain."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scan_planner_msgs.msg import Bspline
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String


# The shape matches the existing M20 motion baseline, but is translated from
# the measured initial pose so both 40 m x 40 m scenes receive the same task.
RELATIVE_TARGETS = (
    ('straight_inward_2p5m', 2.5, 0.0),
    ('return_2p5m', 0.0, 0.0),
    ('right_turn_90_and_straight_6m', 0.0, -6.0),
    ('left_turn_90_and_straight_3m', 3.0, -6.0),
    ('return_3m', 0.0, -6.0),
    ('left_turn_90_return_home_6m', 0.0, 0.0),
)


PROFILE_DEFAULTS = {
    'original_scan': {
        'pose_topic': '/quad_0/body_pose',
        'map_topic': '/map_generator/global_cloud',
        'raw_topic': '/quad_0/cmd_vel',
        'candidate_topic': '',
        'safe_topic': '',
        'applied_topic': '/quad_0/cmd_vel',
        'mode_topic': '',
        'collision_topic': '',
        'fault_topic': '',
        'goal_frame': 'world',
    },
    'm20_mujoco': {
        'pose_topic': '/m20/sim/body_pose',
        'map_topic': '/m20/map/active_global_cloud',
        'raw_topic': '/m20/navigation/cmd_vel_raw',
        'candidate_topic': '/m20/navigation/cmd_vel_candidate',
        'safe_topic': '/m20/control/cmd_vel_safe',
        'applied_topic': '/m20/locomotion/cmd_vel_sdk',
        'mode_topic': '/m20/navigation/candidate_mode',
        'collision_topic': '/m20/control/collision_stop',
        'fault_topic': '/m20/sim/backend_fault',
        'goal_frame': 'map',
    },
}


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


def _rms(values) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _percentile(values, ratio: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values), 100.0 * ratio))


def _twist_tuple(message: Twist) -> Tuple[float, float, float]:
    return (
        float(message.linear.x),
        float(message.linear.y),
        float(message.angular.z),
    )


def _world_velocity(
    command: Tuple[float, float, float],
    yaw: float,
) -> Tuple[float, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * command[0] - sine * command[1],
        sine * command[0] + cosine * command[1],
    )


def _vector_error(
    left: Tuple[float, float],
    right: Tuple[float, float],
) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _cross_track_distance(start, finish, point) -> float:
    delta_x = finish[0] - start[0]
    delta_y = finish[1] - start[1]
    length = math.hypot(delta_x, delta_y)
    if length < 1.0e-9:
        return 0.0
    return abs(
        delta_y * point[0] - delta_x * point[1]
        + finish[0] * start[1] - finish[1] * start[0]
    ) / length


class ExecutionProbe(Node):
    """Subscribe to one execution profile without changing its command path."""

    def __init__(self, arguments) -> None:
        """Create subscriptions and state for one comparison profile."""
        super().__init__(f'{arguments.profile}_execution_probe')
        self.arguments = arguments
        self.pose: Optional[Odometry] = None
        self.raw = Twist()
        self.candidate = Twist()
        self.safe = Twist()
        self.applied = Twist()
        self.mode = (
            'DIRECT'
            if arguments.profile == 'original_scan'
            else 'UNKNOWN'
        )
        self.collision_stop = False
        self.backend_fault = ''
        self.trajectory_count = 0
        self.map_tree: Optional[cKDTree] = None
        self.map_point_count = 0
        self._last_pose_for_velocity = None
        self._last_pose_time_ns = 0
        self.realized_world_velocity = (0.0, 0.0)

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
        self.create_subscription(
            Odometry,
            arguments.pose_topic,
            self._pose_callback,
            50,
        )
        self.create_subscription(
            PointCloud2,
            arguments.map_topic,
            self._map_callback,
            latched,
        )
        self.create_subscription(
            Twist,
            arguments.raw_topic,
            lambda message: setattr(self, 'raw', message),
            50,
        )
        self._optional_twist_subscription(
            arguments.candidate_topic, 'candidate'
        )
        self._optional_twist_subscription(arguments.safe_topic, 'safe')
        if arguments.applied_topic == arguments.raw_topic:
            self.applied = self.raw
            self.create_subscription(
                Twist,
                arguments.applied_topic,
                self._direct_command_callback,
                50,
            )
        else:
            self._optional_twist_subscription(
                arguments.applied_topic, 'applied'
            )
        if arguments.mode_topic:
            self.create_subscription(
                String,
                arguments.mode_topic,
                lambda message: setattr(self, 'mode', str(message.data)),
                10,
            )
        if arguments.collision_topic:
            self.create_subscription(
                Bool,
                arguments.collision_topic,
                lambda message: setattr(
                    self, 'collision_stop', bool(message.data)
                ),
                latched,
            )
        if arguments.fault_topic:
            self.create_subscription(
                String,
                arguments.fault_topic,
                lambda message: setattr(
                    self, 'backend_fault', str(message.data)
                ),
                latched,
            )
        self.create_subscription(
            Bspline,
            '/planning/bspline',
            self._trajectory_callback,
            10,
        )

    def _optional_twist_subscription(
        self,
        topic: str,
        attribute: str,
    ) -> None:
        if not topic:
            return
        self.create_subscription(
            Twist,
            topic,
            lambda message: setattr(self, attribute, message),
            50,
        )

    def _direct_command_callback(self, message: Twist) -> None:
        self.applied = message

    def _trajectory_callback(self, _message: Bspline) -> None:
        self.trajectory_count += 1

    def _pose_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp
        now_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if now_ns <= 0:
            now_ns = self.get_clock().now().nanoseconds
        current = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if self._last_pose_for_velocity is not None:
            delta_time = (now_ns - self._last_pose_time_ns) / 1.0e9
            if 1.0e-4 < delta_time < 0.5:
                self.realized_world_velocity = (
                    (current[0] - self._last_pose_for_velocity[0])
                    / delta_time,
                    (current[1] - self._last_pose_for_velocity[1])
                    / delta_time,
                )
        self._last_pose_for_velocity = current
        self._last_pose_time_ns = now_ns
        self.pose = message

    def _map_callback(self, message: PointCloud2) -> None:
        if self.map_tree is not None:
            return
        points = point_cloud2.read_points_numpy(
            message,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )
        if points.size == 0:
            return
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        # Ground-only points do not constrain the planar body.  Every scene
        # obstacle and fence reaches this band.
        points = points[(points[:, 2] >= 0.15) & (points[:, 2] <= 1.20)]
        if points.size == 0:
            return
        planar = np.unique(np.round(points[:, :2], decimals=4), axis=0)
        self.map_tree = cKDTree(planar)
        self.map_point_count = int(planar.shape[0])
        self.get_logger().info(
            f'planar obstacle index ready: {self.map_point_count} points'
        )

    def current_pose(self):
        """Return the latest planar pose, or ``None`` before first odometry."""
        if self.pose is None:
            return None
        return (
            float(self.pose.pose.pose.position.x),
            float(self.pose.pose.pose.position.y),
            _yaw_from_odometry(self.pose),
        )

    def publish_goal(self, x: float, y: float) -> None:
        """Publish one planar goal facing away from the current pose."""
        current = self.current_pose()
        yaw = math.atan2(y - current[1], x - current[0])
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.arguments.goal_frame
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        self.goal_publisher.publish(message)

    def _footprint_surface_distance(self, pose) -> float:
        if self.map_tree is None:
            return math.inf
        cosine = math.cos(pose[2])
        sine = math.sin(pose[2])
        offset = self.arguments.footprint_offset
        centers = np.asarray(
            [
                [pose[0] + offset * cosine, pose[1] + offset * sine],
                [pose[0] - offset * cosine, pose[1] - offset * sine],
            ]
        )
        distances, _ = self.map_tree.query(centers, k=1)
        return float(np.min(distances))

    def select_safe_target(
        self,
        desired_x: float,
        desired_y: float,
    ) -> Tuple[float, float, float]:
        """Return the nearest target satisfying the common footprint margin."""
        current = self.current_pose()
        step = max(0.02, self.arguments.target_search_step)
        count = int(math.ceil(
            self.arguments.target_search_radius / step
        ))
        offsets = [
            (x_index * step, y_index * step)
            for x_index in range(-count, count + 1)
            for y_index in range(-count, count + 1)
            if math.hypot(x_index * step, y_index * step)
            <= self.arguments.target_search_radius + 1.0e-9
        ]
        offsets.sort(key=lambda value: math.hypot(value[0], value[1]))
        for offset_x, offset_y in offsets:
            candidate_x = desired_x + offset_x
            candidate_y = desired_y + offset_y
            yaw = math.atan2(
                candidate_y - current[1],
                candidate_x - current[0],
            )
            distance = self._footprint_surface_distance(
                (candidate_x, candidate_y, yaw)
            )
            if distance >= self.arguments.target_clearance:
                return candidate_x, candidate_y, distance
        raise RuntimeError(
            'no collision-free target found within '
            f'{self.arguments.target_search_radius:.2f} m of '
            f'({desired_x:.3f}, {desired_y:.3f})'
        )

    def sample(self, test_name: str, elapsed: float) -> Optional[Dict]:
        """Capture one synchronized best-effort execution sample."""
        pose = self.current_pose()
        if pose is None:
            return None
        raw = _twist_tuple(self.raw)
        candidate = (
            _twist_tuple(self.candidate)
            if self.arguments.candidate_topic else raw
        )
        safe = (
            _twist_tuple(self.safe)
            if self.arguments.safe_topic else candidate
        )
        applied = (
            raw
            if self.arguments.applied_topic == self.arguments.raw_topic
            else _twist_tuple(self.applied)
        )
        raw_world = _world_velocity(raw, pose[2])
        applied_world = _world_velocity(applied, pose[2])
        surface_distance = self._footprint_surface_distance(pose)
        return {
            'profile': self.arguments.profile,
            'test': test_name,
            'elapsed_sec': elapsed,
            'x': pose[0],
            'y': pose[1],
            'yaw': pose[2],
            'realized_world_vx': self.realized_world_velocity[0],
            'realized_world_vy': self.realized_world_velocity[1],
            'raw_vx': raw[0],
            'raw_vy': raw[1],
            'raw_wz': raw[2],
            'candidate_vx': candidate[0],
            'candidate_vy': candidate[1],
            'candidate_wz': candidate[2],
            'safe_vx': safe[0],
            'safe_vy': safe[1],
            'safe_wz': safe[2],
            'applied_vx': applied[0],
            'applied_vy': applied[1],
            'applied_wz': applied[2],
            'raw_to_applied_linear_error': _vector_error(
                raw_world, applied_world
            ),
            'applied_to_realized_linear_error': _vector_error(
                applied_world, self.realized_world_velocity
            ),
            'surface_distance_m': surface_distance,
            'scan_hard_margin_m': (
                surface_distance - self.arguments.hard_radius
            ),
            'guard_margin_m': (
                surface_distance - self.arguments.guard_radius
            ),
            'mode': self.mode,
            'collision_stop': self.collision_stop,
            'backend_fault': self.backend_fault,
        }


def _summarise(
    arguments,
    name: str,
    start,
    requested_target,
    target,
    samples,
    duration: float,
    success: bool,
    trajectories: int,
) -> Dict:
    moving = [
        sample for sample in samples
        if math.hypot(sample['applied_vx'], sample['applied_vy']) > 0.01
        or abs(sample['applied_wz']) > 0.02
    ]
    path_length = sum(
        math.hypot(right['x'] - left['x'], right['y'] - left['y'])
        for left, right in zip(samples, samples[1:])
    )
    direct_length = math.hypot(target[0] - start[0], target[1] - start[1])
    direct_yaw = math.atan2(target[1] - start[1], target[0] - start[0])
    yaw_errors = [
        abs(_normalise_angle(sample['yaw'] - direct_yaw))
        for sample in moving
    ]
    mode_transitions = sum(
        left['mode'] != right['mode']
        for left, right in zip(moving, moving[1:])
    )
    mode_counts: Dict[str, int] = {}
    for sample in moving:
        mode_counts[sample['mode']] = mode_counts.get(sample['mode'], 0) + 1
    final = samples[-1]
    return {
        'profile': arguments.profile,
        'name': name,
        'success': success,
        'duration_sec': duration,
        'start': [start[0], start[1], start[2]],
        'requested_target': [
            requested_target[0], requested_target[1]
        ],
        'target': [target[0], target[1]],
        'target_adjustment_m': math.hypot(
            target[0] - requested_target[0],
            target[1] - requested_target[1],
        ),
        'final': [final['x'], final['y'], final['yaw']],
        'final_error_m': math.hypot(
            target[0] - final['x'], target[1] - final['y']
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
        'mean_abs_body_to_segment_yaw_deg': math.degrees(
            statistics.fmean(yaw_errors) if yaw_errors else 0.0
        ),
        'p95_abs_body_to_segment_yaw_deg': math.degrees(
            _percentile(yaw_errors, 0.95)
        ),
        'raw_lateral_command_fraction': (
            sum(abs(sample['raw_vy']) > 0.02 for sample in moving)
            / max(1, len(moving))
        ),
        'applied_lateral_command_fraction': (
            sum(abs(sample['applied_vy']) > 0.02 for sample in moving)
            / max(1, len(moving))
        ),
        'raw_to_applied_linear_error_rms': _rms(
            [sample['raw_to_applied_linear_error'] for sample in moving]
        ),
        'applied_to_realized_linear_error_rms': _rms(
            [
                sample['applied_to_realized_linear_error']
                for sample in moving
            ]
        ),
        'minimum_surface_distance_m': min(
            sample['surface_distance_m'] for sample in samples
        ),
        'minimum_scan_hard_margin_m': min(
            sample['scan_hard_margin_m'] for sample in samples
        ),
        'minimum_guard_margin_m': min(
            sample['guard_margin_m'] for sample in samples
        ),
        'scan_hard_margin_negative_samples': sum(
            sample['scan_hard_margin_m'] < 0.0 for sample in samples
        ),
        'guard_margin_negative_samples': sum(
            sample['guard_margin_m'] < 0.0 for sample in samples
        ),
        'collision_stop_samples': sum(
            bool(sample['collision_stop']) for sample in samples
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
        'mode_fraction': {
            mode: count / max(1, len(moving))
            for mode, count in sorted(mode_counts.items())
        },
        'sample_count': len(samples),
        'moving_sample_count': len(moving),
    }


def _parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--profile',
        choices=sorted(PROFILE_DEFAULTS),
        required=True,
    )
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--goal-timeout-sec', type=float, default=180.0)
    parser.add_argument('--settle-sec', type=float, default=1.0)
    parser.add_argument('--sample-period-sec', type=float, default=0.05)
    parser.add_argument('--hard-radius', type=float, default=0.25)
    parser.add_argument('--guard-radius', type=float, default=0.30)
    parser.add_argument('--footprint-offset', type=float, default=0.18)
    parser.add_argument('--target-clearance', type=float, default=0.55)
    parser.add_argument('--target-search-radius', type=float, default=1.5)
    parser.add_argument('--target-search-step', type=float, default=0.10)
    for name in (
        'pose_topic',
        'map_topic',
        'raw_topic',
        'candidate_topic',
        'safe_topic',
        'applied_topic',
        'mode_topic',
        'collision_topic',
        'fault_topic',
        'goal_frame',
    ):
        parser.add_argument(f'--{name.replace("_", "-")}', default=None)
    arguments = parser.parse_args()
    defaults = PROFILE_DEFAULTS[arguments.profile]
    for name, default in defaults.items():
        if getattr(arguments, name) is None:
            setattr(arguments, name, default)
    return arguments


def main() -> int:
    """Run the selected target sequence and save CSV plus JSON results."""
    arguments = _parse_arguments()
    arguments.output_directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = ExecutionProbe(arguments)
    all_samples = []
    summaries = []
    try:
        wait_deadline = time.monotonic() + 45.0
        while (
            (node.pose is None or node.map_tree is None)
            and time.monotonic() < wait_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            raise RuntimeError(f'timed out waiting for {arguments.pose_topic}')
        if node.map_tree is None:
            raise RuntimeError(f'timed out waiting for {arguments.map_topic}')
        initial = node.current_pose()
        targets = [
            (name, initial[0] + delta_x, initial[1] + delta_y)
            for name, delta_x, delta_y in RELATIVE_TARGETS
        ]
        for name, requested_x, requested_y in targets:
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.02)
            start = node.current_pose()
            target_x, target_y, target_clearance = (
                node.select_safe_target(requested_x, requested_y)
            )
            trajectories_before = node.trajectory_count
            node.publish_goal(target_x, target_y)
            print(
                f'START {arguments.profile}/{name}: '
                f'pose=({start[0]:.3f},{start[1]:.3f},'
                f'{math.degrees(start[2]):.1f}deg) '
                f'requested=({requested_x:.3f},{requested_y:.3f}) '
                f'target=({target_x:.3f},{target_y:.3f}) '
                f'clearance={target_clearance:.3f}m',
                flush=True,
            )
            started_at = time.monotonic()
            next_sample = started_at
            settled_since = None
            samples = []
            success = False
            while (
                time.monotonic() - started_at
                < arguments.goal_timeout_sec
            ):
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if now >= next_sample:
                    sample = node.sample(name, now - started_at)
                    if sample is not None:
                        samples.append(sample)
                        all_samples.append(sample)
                    next_sample = now + arguments.sample_period_sec
                pose = node.current_pose()
                distance = math.hypot(
                    target_x - pose[0], target_y - pose[1]
                )
                realized_speed = math.hypot(
                    node.realized_world_velocity[0],
                    node.realized_world_velocity[1],
                )
                if distance <= 0.25 and realized_speed <= 0.08:
                    if settled_since is None:
                        settled_since = now
                    elif now - settled_since >= arguments.settle_sec:
                        success = True
                        break
                else:
                    settled_since = None
                if node.backend_fault:
                    print(
                        f'ABORT {arguments.profile}/{name}: '
                        f'backend fault={node.backend_fault}',
                        flush=True,
                    )
                    break
            duration = time.monotonic() - started_at
            if not samples:
                raise RuntimeError(f'no samples collected for {name}')
            summary = _summarise(
                arguments,
                name,
                start,
                (requested_x, requested_y),
                (target_x, target_y),
                samples,
                duration,
                success,
                node.trajectory_count - trajectories_before,
            )
            summaries.append(summary)
            print(
                f'END {arguments.profile}/{name}: success={success} '
                f'duration={duration:.2f}s '
                f'error={summary["final_error_m"]:.3f}m '
                f'cross_track={summary["max_cross_track_m"]:.3f}m '
                f'hard_margin={summary["minimum_scan_hard_margin_m"]:.3f}m',
                flush=True,
            )
            if not success:
                break
            settle_deadline = time.monotonic() + 0.5
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if not all_samples:
        raise RuntimeError('probe produced no samples')
    csv_path = arguments.output_directory / 'execution_samples.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_samples[0]))
        writer.writeheader()
        writer.writerows(all_samples)
    report = {
        'schema_version': 2,
        'profile': arguments.profile,
        'sample_period_sec': arguments.sample_period_sec,
        'map_point_count': node.map_point_count,
        'topics': {
            name: getattr(arguments, name)
            for name in (
                'pose_topic',
                'map_topic',
                'raw_topic',
                'candidate_topic',
                'safe_topic',
                'applied_topic',
                'mode_topic',
                'collision_topic',
                'fault_topic',
            )
        },
        'footprint': {
            'offset_m': arguments.footprint_offset,
            'scan_hard_radius_m': arguments.hard_radius,
            'guard_radius_m': arguments.guard_radius,
        },
        'tests': summaries,
    }
    (
        arguments.output_directory / 'execution_summary.json'
    ).write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    return 0 if summaries and all(item['success'] for item in summaries) else 2


if __name__ == '__main__':
    raise SystemExit(main())
