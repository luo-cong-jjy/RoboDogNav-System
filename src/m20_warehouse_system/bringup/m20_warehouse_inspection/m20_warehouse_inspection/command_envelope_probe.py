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

"""Measure the official M20 SDK body-command envelope without SCAN."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String


Command = Tuple[float, float, float]


@dataclass(frozen=True)
class TestCase:
    """One exact body-velocity target sent to the official SDK interface."""

    name: str
    command: Command


CASES = {
    case.name: case
    for case in (
        TestCase('idle', (0.0, 0.0, 0.0)),
        TestCase('forward_010', (0.10, 0.0, 0.0)),
        TestCase('forward_020', (0.20, 0.0, 0.0)),
        TestCase('forward_030', (0.30, 0.0, 0.0)),
        TestCase('forward_045', (0.45, 0.0, 0.0)),
        TestCase('reverse_010', (-0.10, 0.0, 0.0)),
        TestCase('reverse_020', (-0.20, 0.0, 0.0)),
        TestCase('reverse_025', (-0.25, 0.0, 0.0)),
        TestCase('reverse_030', (-0.30, 0.0, 0.0)),
        TestCase('reverse_035', (-0.35, 0.0, 0.0)),
        TestCase('reverse_045', (-0.45, 0.0, 0.0)),
        TestCase('reverse_turn_035_015', (-0.35, 0.0, 0.15)),
        TestCase('reverse_turn_035_035', (-0.35, 0.0, 0.35)),
        TestCase('reverse_turn_035_050', (-0.35, 0.0, 0.50)),
        TestCase('reverse_turn_035_065', (-0.35, 0.0, 0.65)),
        TestCase('lateral_005', (0.0, 0.05, 0.0)),
        TestCase('lateral_010', (0.0, 0.10, 0.0)),
        TestCase('lateral_020', (0.0, 0.20, 0.0)),
        TestCase('yaw_005', (0.0, 0.0, 0.05)),
        TestCase('yaw_010', (0.0, 0.0, 0.10)),
        TestCase('yaw_015', (0.0, 0.0, 0.15)),
        TestCase('yaw_020', (0.0, 0.0, 0.20)),
        TestCase('yaw_035', (0.0, 0.0, 0.35)),
        TestCase('yaw_050', (0.0, 0.0, 0.50)),
        TestCase('yaw_065', (0.0, 0.0, 0.65)),
        TestCase('arc_020_015', (0.20, 0.0, 0.15)),
        TestCase('arc_030_025', (0.30, 0.0, 0.25)),
        TestCase('arc_045_015', (0.45, 0.0, 0.15)),
        TestCase('turn_005_020', (0.05, 0.0, 0.20)),
        TestCase('turn_010_020', (0.10, 0.0, 0.20)),
        TestCase('turn_015_020', (0.15, 0.0, 0.20)),
        TestCase('turn_020_020', (0.20, 0.0, 0.20)),
        TestCase('turn_010_035', (0.10, 0.0, 0.35)),
        TestCase('turn_020_035', (0.20, 0.0, 0.35)),
        TestCase('turn_030_035', (0.30, 0.0, 0.35)),
        TestCase('turn_020_050', (0.20, 0.0, 0.50)),
        TestCase('turn_030_050', (0.30, 0.0, 0.50)),
        TestCase('turn_045_050', (0.45, 0.0, 0.50)),
        TestCase('turn_005_050', (0.05, 0.0, 0.50)),
        TestCase('turn_005_065', (0.05, 0.0, 0.65)),
        TestCase('turn_030_065', (0.30, 0.0, 0.65)),
        TestCase('turn_035_065', (0.35, 0.0, 0.65)),
        TestCase('turn_035_neg065', (0.35, 0.0, -0.65)),
        TestCase('turn_045_065', (0.45, 0.0, 0.65)),
    )
}

SUITES = {
    'smoke': [
        'idle',
        'forward_010',
        'yaw_020',
        'arc_020_015',
    ],
    'axial': [
        'idle',
        'forward_010',
        'forward_020',
        'forward_030',
        'forward_045',
        'lateral_005',
        'lateral_010',
        'lateral_020',
        'yaw_005',
        'yaw_010',
        'yaw_015',
        'yaw_020',
        'yaw_035',
        'yaw_050',
        'yaw_065',
    ],
    'navigation': [
        'idle',
        'forward_020',
        'forward_045',
        'arc_020_015',
        'arc_030_025',
        'arc_045_015',
        'turn_010_035',
        'turn_005_050',
        'turn_005_065',
    ],
    # These commands reproduce the correction branch observed after a
    # near-obstacle trajectory overshoot. Keep the cases cold-startable so
    # residual state from another manoeuvre cannot mask policy behaviour.
    'reverse': [
        'reverse_010',
        'reverse_020',
        'reverse_025',
        'reverse_030',
        'reverse_035',
        'reverse_045',
        'reverse_turn_035_015',
        'reverse_turn_035_035',
        'reverse_turn_035_050',
        'reverse_turn_035_065',
    ],
    # Ordered from the already-failing pure-yaw neighbourhood toward larger
    # rolling radii. Run individual cases cold before accepting a boundary.
    'turn_matrix': [
        'turn_005_020',
        'turn_010_020',
        'turn_015_020',
        'turn_020_020',
        'turn_010_035',
        'turn_020_035',
        'turn_030_035',
        'turn_005_050',
        'turn_020_050',
        'turn_030_050',
        'turn_045_050',
        'turn_005_065',
        'turn_030_065',
        'turn_035_065',
        'turn_045_065',
    ],
}
SUITES['full'] = list(
    dict.fromkeys(SUITES['axial'] + SUITES['navigation'])
)


def wrap_angle(value: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_odometry(message: Odometry) -> float:
    """Extract yaw from the backend odometry quaternion."""
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CommandEnvelopeProbe(Node):
    """Publish exact SDK commands and collect physics diagnostics."""

    def __init__(self) -> None:
        super().__init__('m20_command_envelope_probe')
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready = False
        self.fault = ''
        self.pose: Optional[Odometry] = None
        self.dynamics: Dict = {}
        self.command_publisher = self.create_publisher(
            Twist,
            '/m20/locomotion/cmd_vel_sdk',
            20,
        )
        self.mode_publisher = self.create_publisher(
            String,
            '/m20/locomotion/mode',
            latched,
        )
        self.create_subscription(
            Bool,
            '/m20/sim/backend_ready',
            lambda message: setattr(self, 'ready', bool(message.data)),
            latched,
        )
        self.create_subscription(
            String,
            '/m20/sim/backend_fault',
            lambda message: setattr(self, 'fault', str(message.data)),
            latched,
        )
        self.create_subscription(
            Odometry,
            '/m20/sim/body_pose',
            lambda message: setattr(self, 'pose', message),
            50,
        )
        self.create_subscription(
            String,
            '/m20/sim/dynamics_state',
            self._dynamics_callback,
            20,
        )

    def _dynamics_callback(self, message: String) -> None:
        try:
            self.dynamics = json.loads(message.data)
        except json.JSONDecodeError:
            self.dynamics = {}

    def publish(self, command: Command) -> None:
        """Publish command and the matching wheel-brake intent."""
        message = Twist()
        message.linear.x = command[0]
        message.linear.y = command[1]
        message.angular.z = command[2]
        self.command_publisher.publish(message)
        if abs(command[1]) > 1.0e-6:
            mode = 'LATERAL_MANEUVER'
        elif abs(command[2]) > 0.25 and abs(command[0]) < 0.08:
            mode = 'COORDINATED_TURN'
        elif any(abs(value) > 1.0e-6 for value in command):
            mode = 'WHEEL_CRUISE'
        else:
            mode = 'STOPPED'
        self.mode_publisher.publish(String(data=mode))

    def sample(
        self,
        *,
        elapsed_sec: float,
        case_name: str,
        phase: str,
        target: Command,
        published: Command,
    ) -> Dict:
        """Create one flat, CSV-compatible observation."""
        position = self.dynamics.get('base_position', [math.nan] * 3)
        rpy = self.dynamics.get('base_rpy', [math.nan] * 3)
        velocity = self.dynamics.get('base_velocity', [math.nan] * 6)
        diagnostic_linear_body = self.dynamics.get(
            'base_linear_velocity_body',
            [math.nan] * 3,
        )
        diagnostic_angular_body = self.dynamics.get(
            'base_angular_velocity_body',
            [math.nan] * 3,
        )
        yaw = float(rpy[2])
        world_vx = float(velocity[0])
        world_vy = float(velocity[1])
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
        odometry_twist = (
            self.pose.twist.twist
            if self.pose is not None
            else None
        )
        odometry_linear_body = [
            float(odometry_twist.linear.x),
            float(odometry_twist.linear.y),
            float(odometry_twist.linear.z),
        ] if odometry_twist is not None else [math.nan] * 3
        odometry_angular_body = [
            float(odometry_twist.angular.x),
            float(odometry_twist.angular.y),
            float(odometry_twist.angular.z),
        ] if odometry_twist is not None else [math.nan] * 3
        linear_contract_error = math.sqrt(
            sum(
                (
                    odometry_linear_body[index]
                    - float(diagnostic_linear_body[index])
                ) ** 2
                for index in range(3)
            )
        )
        angular_contract_error = math.sqrt(
            sum(
                (
                    odometry_angular_body[index]
                    - float(diagnostic_angular_body[index])
                ) ** 2
                for index in range(3)
            )
        )
        return {
            'elapsed_sec': elapsed_sec,
            'case': case_name,
            'phase': phase,
            'target_vx': target[0],
            'target_vy': target[1],
            'target_wz': target[2],
            'published_vx': published[0],
            'published_vy': published[1],
            'published_wz': published[2],
            'x': float(position[0]),
            'y': float(position[1]),
            'z': float(position[2]),
            'roll': float(rpy[0]),
            'pitch': float(rpy[1]),
            'yaw': yaw,
            'world_vx': world_vx,
            'world_vy': world_vy,
            'body_vx': body_vx,
            'body_vy': body_vy,
            'body_wz': float(velocity[5]),
            'odom_body_vx': odometry_linear_body[0],
            'odom_body_vy': odometry_linear_body[1],
            'odom_body_vz': odometry_linear_body[2],
            'odom_body_wx': odometry_angular_body[0],
            'odom_body_wy': odometry_angular_body[1],
            'odom_body_wz': odometry_angular_body[2],
            'twist_linear_contract_error_mps': linear_contract_error,
            'twist_angular_contract_error_radps': angular_contract_error,
            'ready': bool(self.dynamics.get('ready', self.ready)),
            'fault': str(self.dynamics.get('fault', self.fault)),
            'parking_brake_active': bool(
                self.dynamics.get('parking_brake_active', True)
            ),
            'contact_count': int(
                self.dynamics.get('contact_count', 0)
            ),
            'obstacle_contact_event_count': int(
                self.dynamics.get(
                    'obstacle_contact_event_count',
                    0,
                )
            ),
            'max_abs_torque': float(
                self.dynamics.get('max_abs_torque', math.nan)
            ),
        }


def ramp_command(target: Command, fraction: float) -> Command:
    """Scale every component by a bounded linear-ramp fraction."""
    value = max(0.0, min(1.0, fraction))
    return tuple(component * value for component in target)


def mean(values: Sequence[float]) -> float:
    """Return a finite arithmetic mean or NaN for an empty sequence."""
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else math.nan


def summarise_case(
    test_case: TestCase,
    samples: Sequence[Dict],
    *,
    safety_abort: str,
    stable_tilt_rad: float,
    stable_height_m: float,
) -> Dict:
    """Summarise stability and command tracking for one test case."""
    active = [
        sample for sample in samples if sample['phase'] == 'hold'
    ]
    start = samples[0]
    final = samples[-1]
    peak_tilt = max(
        max(abs(sample['roll']), abs(sample['pitch']))
        for sample in samples
    )
    minimum_height = min(sample['z'] for sample in samples)
    fault = next(
        (
            sample['fault']
            for sample in samples
            if sample['fault']
        ),
        '',
    )
    stable = (
        not fault
        and not safety_abort
        and peak_tilt <= stable_tilt_rad
        and minimum_height >= stable_height_m
        and max(
            sample['obstacle_contact_event_count']
            for sample in samples
        ) == 0
    )
    return {
        'name': test_case.name,
        'command': {
            'vx': test_case.command[0],
            'vy': test_case.command[1],
            'wz': test_case.command[2],
        },
        'status': (
            'FAULT'
            if fault
            else 'ABORT'
            if safety_abort
            else 'PASS'
            if stable
            else 'WARN'
        ),
        'stable': stable,
        'fault': fault,
        'safety_abort': safety_abort,
        'sample_count': len(samples),
        'peak_abs_tilt_rad': peak_tilt,
        'minimum_base_height_m': minimum_height,
        'mean_hold_body_vx_mps': mean(
            [sample['body_vx'] for sample in active]
        ),
        'mean_hold_body_vy_mps': mean(
            [sample['body_vy'] for sample in active]
        ),
        'mean_hold_body_wz_radps': mean(
            [sample['body_wz'] for sample in active]
        ),
        'mean_hold_odometry_body_vx_mps': mean(
            [sample['odom_body_vx'] for sample in active]
        ),
        'mean_hold_odometry_body_vy_mps': mean(
            [sample['odom_body_vy'] for sample in active]
        ),
        'mean_hold_odometry_body_wz_radps': mean(
            [sample['odom_body_wz'] for sample in active]
        ),
        # Odometry (200 Hz) and diagnostics (10 Hz) are sampled
        # asynchronously. These means are wiring checks, not controller
        # tracking thresholds.
        'mean_hold_twist_linear_contract_error_mps': mean(
            [
                sample['twist_linear_contract_error_mps']
                for sample in active
            ]
        ),
        'mean_hold_twist_angular_contract_error_radps': mean(
            [
                sample['twist_angular_contract_error_radps']
                for sample in active
            ]
        ),
        'planar_displacement_m': math.hypot(
            final['x'] - start['x'],
            final['y'] - start['y'],
        ),
        'yaw_change_rad': wrap_angle(final['yaw'] - start['yaw']),
        'maximum_abs_torque': max(
            sample['max_abs_torque'] for sample in samples
        ),
        'obstacle_contact_event_count': max(
            sample['obstacle_contact_event_count']
            for sample in samples
        ),
    }


def parse_arguments(
    arguments: Optional[Sequence[str]] = None,
) -> argparse.Namespace:
    """Parse probe arguments without passing them to rclpy."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=sorted(SUITES), default='smoke')
    parser.add_argument(
        '--case',
        choices=['', *sorted(CASES)],
        default='',
        dest='case_name',
    )
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--ready-timeout-sec', type=float, default=45.0)
    parser.add_argument('--ramp-sec', type=float, default=1.0)
    parser.add_argument('--hold-sec', type=float, default=2.0)
    parser.add_argument('--settle-sec', type=float, default=1.5)
    parser.add_argument('--publish-rate-hz', type=float, default=50.0)
    parser.add_argument('--sample-rate-hz', type=float, default=20.0)
    parser.add_argument('--stable-tilt-rad', type=float, default=0.25)
    parser.add_argument('--stable-height-m', type=float, default=0.48)
    parser.add_argument('--abort-tilt-rad', type=float, default=0.50)
    parser.add_argument('--abort-height-m', type=float, default=0.40)
    clean_arguments = (
        list(arguments)
        if arguments is not None
        else remove_ros_args(args=sys.argv)[1:]
    )
    return parser.parse_args(clean_arguments)


def run_phase(
    node: CommandEnvelopeProbe,
    *,
    case_name: str,
    phase: str,
    target: Command,
    duration_sec: float,
    start_fraction: float,
    end_fraction: float,
    test_started: float,
    publish_period: float,
    sample_period: float,
    abort_tilt_rad: float,
    abort_height_m: float,
    samples: List[Dict],
) -> str:
    """Run one ramp/hold/settle phase and return a safety-abort reason."""
    started = time.monotonic()
    next_publish = started
    next_sample = started
    while rclpy.ok() and time.monotonic() - started < duration_sec:
        now = time.monotonic()
        elapsed = now - started
        fraction = (
            end_fraction
            if duration_sec <= 0.0
            else start_fraction
            + (end_fraction - start_fraction)
            * min(1.0, elapsed / duration_sec)
        )
        command = ramp_command(target, fraction)
        if now >= next_publish:
            node.publish(command)
            next_publish = now + publish_period
        rclpy.spin_once(node, timeout_sec=min(0.005, publish_period))
        if now < next_sample or not node.dynamics:
            continue
        sample = node.sample(
            elapsed_sec=now - test_started,
            case_name=case_name,
            phase=phase,
            target=target,
            published=command,
        )
        samples.append(sample)
        next_sample = now + sample_period
        if sample['fault']:
            return f"backend fault: {sample['fault']}"
        tilt = max(abs(sample['roll']), abs(sample['pitch']))
        if tilt > abort_tilt_rad:
            return f'tilt {tilt:.3f} rad exceeded abort threshold'
        if sample['z'] < abort_height_m:
            return (
                f"height {sample['z']:.3f} m fell below abort threshold"
            )
    return ''


def main(arguments: Optional[Sequence[str]] = None) -> int:
    """Run a staged command suite and save raw plus summary evidence."""
    args = parse_arguments(arguments)
    args.output_directory.mkdir(parents=True, exist_ok=True)
    names = [args.case_name] if args.case_name else SUITES[args.suite]
    cases = [CASES[name] for name in names]
    publish_period = 1.0 / max(1.0, args.publish_rate_hz)
    sample_period = 1.0 / max(1.0, args.sample_rate_hz)

    rclpy.init(args=[])
    node = CommandEnvelopeProbe()
    all_samples: List[Dict] = []
    results: List[Dict] = []
    test_started = time.monotonic()
    try:
        deadline = time.monotonic() + args.ready_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish((0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.ready and node.pose is not None and node.dynamics:
                break
        if not node.ready or node.pose is None or not node.dynamics:
            raise RuntimeError(
                'timed out waiting for stable M20 SDK stand-up'
            )

        for test_case in cases:
            node.get_logger().info(
                f'case {test_case.name}: command={test_case.command}'
            )
            case_samples: List[Dict] = []
            safety_abort = ''
            phases = (
                ('ramp_up', args.ramp_sec, 0.0, 1.0),
                ('hold', args.hold_sec, 1.0, 1.0),
                ('ramp_down', args.ramp_sec, 1.0, 0.0),
                ('settle', args.settle_sec, 0.0, 0.0),
            )
            for phase, duration, start_fraction, end_fraction in phases:
                safety_abort = run_phase(
                    node,
                    case_name=test_case.name,
                    phase=phase,
                    target=test_case.command,
                    duration_sec=duration,
                    start_fraction=start_fraction,
                    end_fraction=end_fraction,
                    test_started=test_started,
                    publish_period=publish_period,
                    sample_period=sample_period,
                    abort_tilt_rad=args.abort_tilt_rad,
                    abort_height_m=args.abort_height_m,
                    samples=case_samples,
                )
                if safety_abort:
                    break
            for _ in range(50):
                node.publish((0.0, 0.0, 0.0))
                rclpy.spin_once(node, timeout_sec=0.01)
            if not case_samples:
                raise RuntimeError(
                    f'no dynamics samples collected for {test_case.name}'
                )
            all_samples.extend(case_samples)
            result = summarise_case(
                test_case,
                case_samples,
                safety_abort=safety_abort,
                stable_tilt_rad=args.stable_tilt_rad,
                stable_height_m=args.stable_height_m,
            )
            results.append(result)
            node.get_logger().info(
                f"{test_case.name}: {result['status']}, "
                f"peak_tilt={result['peak_abs_tilt_rad']:.3f}rad, "
                f"min_height={result['minimum_base_height_m']:.3f}m"
            )
            if result['status'] in {'FAULT', 'ABORT'}:
                break
    finally:
        for _ in range(10):
            node.publish((0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()
        rclpy.try_shutdown()

    csv_path = args.output_directory / 'command_envelope_samples.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(all_samples[0]),
        )
        writer.writeheader()
        writer.writerows(all_samples)
    report = {
        'schema_version': 1,
        'suite': args.suite,
        'requested_cases': names,
        'completed_cases': len(results),
        'thresholds': {
            'stable_tilt_rad': args.stable_tilt_rad,
            'stable_height_m': args.stable_height_m,
            'abort_tilt_rad': args.abort_tilt_rad,
            'abort_height_m': args.abort_height_m,
        },
        'all_stable': len(results) == len(cases)
        and all(result['stable'] for result in results),
        'results': results,
    }
    summary_path = args.output_directory / 'command_envelope_summary.json'
    summary_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 0 if report['all_stable'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
