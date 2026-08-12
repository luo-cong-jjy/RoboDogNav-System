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

"""Fail-closed velocity supervisor and command multiplexer."""

from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .safety_policy import (
    PlanarCommand,
    TimedCommand,
    clamp_command,
    proportional_ramp_command,
    select_fresh_command,
    slew_command,
    valid_collision_recovery_command,
)


class SafetySupervisor(Node):
    """Gate every command before it reaches a simulation or real backend."""

    def __init__(self) -> None:
        super().__init__('m20_safety_supervisor')
        self.declare_parameter(
            'navigation_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(
            'manual_topic', '/m20/control/cmd_vel_manual'
        )
        self.declare_parameter('safe_topic', '/m20/control/cmd_vel_safe')
        self.declare_parameter('e_stop_topic', '/m20/control/e_stop')
        self.declare_parameter(
            'floor_hold_topic', '/m20/control/floor_switch_hold'
        )
        self.declare_parameter(
            'mission_hold_topic', '/m20/control/mission_hold'
        )
        self.declare_parameter(
            'collision_stop_topic', '/m20/control/collision_stop'
        )
        self.declare_parameter('collision_guard_enabled', True)
        self.declare_parameter(
            'collision_recovery_available_topic',
            '/m20/control/collision_recovery_available',
        )
        self.declare_parameter(
            'collision_recovery_command_topic',
            '/m20/control/collision_recovery_cmd',
        )
        self.declare_parameter('collision_recovery_timeout_sec', 0.15)
        self.declare_parameter('collision_recovery_ramp_sec', 1.0)
        self.declare_parameter(
            'execution_hold_topic', '/m20/control/execution_hold'
        )
        self.declare_parameter('map_ready_topic', '/m20/map/ready')
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        self.declare_parameter('capability_profile_id', 'fallback_defaults')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('max_linear_x', 0.45)
        self.declare_parameter('max_linear_y', 0.20)
        self.declare_parameter('max_angular_z', 0.65)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 1.2)
        self.declare_parameter('manual_priority', True)

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._command_timeout = max(
            0.05, float(self.get_parameter('command_timeout_sec').value)
        )
        self._odom_timeout = max(
            0.05, float(self.get_parameter('odom_timeout_sec').value)
        )
        self._collision_recovery_timeout = max(
            0.05,
            float(
                self.get_parameter(
                    'collision_recovery_timeout_sec'
                ).value
            ),
        )
        self._collision_recovery_ramp = max(
            0.0,
            float(
                self.get_parameter(
                    'collision_recovery_ramp_sec'
                ).value
            ),
        )
        self._limits: PlanarCommand = (
            float(self.get_parameter('max_linear_x').value),
            float(self.get_parameter('max_linear_y').value),
            float(self.get_parameter('max_angular_z').value),
        )
        self._linear_acceleration = float(
            self.get_parameter('max_linear_accel').value
        )
        self._angular_acceleration = float(
            self.get_parameter('max_angular_accel').value
        )
        self._manual_priority = bool(
            self.get_parameter('manual_priority').value
        )
        self._navigation: Optional[TimedCommand] = None
        self._manual: Optional[TimedCommand] = None
        self._last_odom_time: Optional[float] = None
        self._map_ready = False
        self._e_stop = False
        self._floor_hold = False
        self._mission_hold = False
        self._collision_guard_enabled = bool(
            self.get_parameter('collision_guard_enabled').value
        )
        # The native SCAN execution profile intentionally has no downstream
        # footprint veto.  Keep the multi-floor/e-stop command gate, but do
        # not fail closed waiting for a guard node that is not launched.
        self._collision_stop = self._collision_guard_enabled
        self._collision_recovery_available = False
        self._collision_recovery: Optional[TimedCommand] = None
        self._last_output: PlanarCommand = (0.0, 0.0, 0.0)
        self._last_publish_time = self._now()
        self._last_state = ''

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._safe_publisher = self.create_publisher(
            Twist, str(self.get_parameter('safe_topic').value), 20
        )
        self._state_publisher = self.create_publisher(
            String, '/m20/control/safety_state', latched_qos
        )
        self._execution_hold_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('execution_hold_topic').value),
            latched_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('navigation_topic').value),
            self._navigation_callback,
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('manual_topic').value),
            self._manual_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('e_stop_topic').value),
            self._e_stop_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('floor_hold_topic').value),
            self._floor_hold_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('mission_hold_topic').value),
            self._mission_hold_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('collision_stop_topic').value),
            self._collision_stop_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    'collision_recovery_available_topic'
                ).value
            ),
            self._collision_recovery_available_callback,
            latched_qos,
        )
        self.create_subscription(
            Twist,
            str(
                self.get_parameter(
                    'collision_recovery_command_topic'
                ).value
            ),
            self._collision_recovery_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('map_ready_topic').value),
            self._map_ready_callback,
            latched_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._publish_immediate_zero('MAP_NOT_READY')
        self.get_logger().info(
            'Safety supervisor ready: candidate/manual -> safe; '
            'fail-closed gates enabled; capability profile='
            f'{self.get_parameter("capability_profile_id").value}; '
            f'limits=({self._limits[0]:.2f},'
            f'{self._limits[1]:.2f},{self._limits[2]:.2f})'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _command_from_message(message: Twist) -> PlanarCommand:
        return message.linear.x, message.linear.y, message.angular.z

    @staticmethod
    def _message_from_command(command: PlanarCommand) -> Twist:
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        return message

    def _navigation_callback(self, message: Twist) -> None:
        self._navigation = TimedCommand(
            clamp_command(self._command_from_message(message), self._limits),
            self._now(),
        )

    def _manual_callback(self, message: Twist) -> None:
        self._manual = TimedCommand(
            clamp_command(self._command_from_message(message), self._limits),
            self._now(),
        )

    def _e_stop_callback(self, message: Bool) -> None:
        changed = self._e_stop != message.data
        self._e_stop = message.data
        if self._e_stop:
            self._publish_immediate_zero('E_STOP')
        if changed:
            self.get_logger().warn(
                f'e-stop {"asserted" if self._e_stop else "released"}'
            )

    def _floor_hold_callback(self, message: Bool) -> None:
        self._floor_hold = message.data
        if self._floor_hold:
            self._publish_immediate_zero('FLOOR_SWITCH_HOLD')

    def _mission_hold_callback(self, message: Bool) -> None:
        self._mission_hold = message.data
        if self._mission_hold:
            self._publish_immediate_zero('MISSION_HOLD')

    def _map_ready_callback(self, message: Bool) -> None:
        self._map_ready = message.data
        if not self._map_ready:
            self._publish_immediate_zero('MAP_NOT_READY')

    def _collision_stop_callback(self, message: Bool) -> None:
        if not self._collision_guard_enabled:
            return
        changed = self._collision_stop != bool(message.data)
        self._collision_stop = bool(message.data)
        if changed and self._collision_stop:
            self._publish_immediate_zero('COLLISION_STOP')

    def _collision_recovery_available_callback(
        self,
        message: Bool,
    ) -> None:
        self._collision_recovery_available = bool(message.data)

    def _collision_recovery_callback(self, message: Twist) -> None:
        # The guard may authorize only swept-clear forward rolling motion or
        # straight reverse. The final gate validates that shape before SDK.
        command = clamp_command(
            (
                float(message.linear.x),
                0.0,
                float(message.angular.z),
            ),
            self._limits,
        )
        self._collision_recovery = TimedCommand(command, self._now())

    def _odom_callback(self, _message: Odometry) -> None:
        self._last_odom_time = self._now()

    def _hard_stop_reason(self, now: float) -> Optional[str]:
        if self._e_stop:
            return 'E_STOP'
        if self._floor_hold:
            return 'FLOOR_SWITCH_HOLD'
        if self._mission_hold:
            return 'MISSION_HOLD'
        if not self._map_ready:
            return 'MAP_NOT_READY'
        if (
            self._last_odom_time is None
            or now - self._last_odom_time > self._odom_timeout
        ):
            return 'ODOM_STALE'
        return None

    def _fresh_collision_recovery(
        self,
        now: float,
    ) -> Optional[PlanarCommand]:
        if (
            not self._collision_recovery_available
            or self._collision_recovery is None
            or now - self._collision_recovery.stamp
            > self._collision_recovery_timeout
        ):
            return None
        command = self._collision_recovery.command
        if not valid_collision_recovery_command(command):
            return None
        return command

    def _publish_state(self, state: str) -> None:
        if state != self._last_state:
            self._state_publisher.publish(String(data=state))
            # Only autonomous navigation advances the SCAN trajectory clock.
            # Manual override and every fail-closed state freeze execution.
            self._execution_hold_publisher.publish(
                Bool(data=state != 'NAVIGATION')
            )
            self.get_logger().info(f'safety state: {state}')
            self._last_state = state

    def _publish_immediate_zero(self, state: str) -> None:
        self._last_output = (0.0, 0.0, 0.0)
        self._safe_publisher.publish(Twist())
        self._publish_state(state)

    def _publish(self) -> None:
        now = self._now()
        dt = max(0.0, min(0.2, now - self._last_publish_time))
        self._last_publish_time = now
        hard_stop = self._hard_stop_reason(now)
        if hard_stop is not None:
            self._publish_immediate_zero(hard_stop)
            return
        if self._collision_guard_enabled and self._collision_stop:
            recovery = self._fresh_collision_recovery(now)
            if recovery is None:
                self._publish_immediate_zero('COLLISION_STOP')
                return
            self._last_output = proportional_ramp_command(
                self._last_output,
                recovery,
                dt,
                self._collision_recovery_ramp,
            )
            self._safe_publisher.publish(
                self._message_from_command(self._last_output)
            )
            self._publish_state('COLLISION_RECOVERY')
            return
        target, state = select_fresh_command(
            now,
            self._command_timeout,
            self._navigation,
            self._manual,
            self._manual_priority,
        )
        target = clamp_command(target, self._limits)
        output = slew_command(
            self._last_output,
            target,
            dt,
            self._linear_acceleration,
            self._angular_acceleration,
        )
        self._safe_publisher.publish(self._message_from_command(output))
        self._last_output = output
        self._publish_state(state)


def main() -> None:
    """Run the velocity safety supervisor."""
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # CycloneDDS can invalidate a subscription while SIGINT is being
        # handled. Treat only that shutdown path as a clean stop.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
