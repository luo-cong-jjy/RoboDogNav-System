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

"""Independent static-map collision lookahead guard."""

import math
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .collision_policy import (
    conservative_raster_radius,
    first_blocking_command_envelope,
    GridGeometry,
    hard_body_raster_radius,
    inflate_blocked_grid,
    recovery_sweep_is_clear,
    safe_raster_shell_escape,
    safe_rolling_recovery,
    update_clear_confirmation,
    update_recovery_budget,
)


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


class CollisionGuard(Node):
    """Assert a fail-closed stop for occupied current or predicted cells."""

    def __init__(self) -> None:
        super().__init__('m20_collision_guard')
        self.declare_parameter(
            'occupancy_topic', '/m20/map/active_occupancy'
        )
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        self.declare_parameter(
            'command_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(
            'stop_topic', '/m20/control/collision_stop'
        )
        self.declare_parameter(
            'state_topic', '/m20/control/collision_guard_state'
        )
        self.declare_parameter(
            'diagnostic_topic',
            '/m20/control/collision_guard_diagnostic',
        )
        self.declare_parameter(
            'recovery_available_topic',
            '/m20/control/collision_recovery_available',
        )
        self.declare_parameter(
            'recovery_command_topic',
            '/m20/control/collision_recovery_cmd',
        )
        self.declare_parameter('map_ready_topic', '/m20/map/ready')
        self.declare_parameter('capability_profile_id', 'fallback_defaults')
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)
        self.declare_parameter('turn_swept_radius', 0.0)
        self.declare_parameter('occupied_threshold', 50)
        # Safe native-SCAN defaults validated with the official M20 in
        # MuJoCo. The optional grid-route launch overrides these explicitly.
        self.declare_parameter('footprint_radius', 0.25)
        self.declare_parameter('footprint_offset', 0.18)
        self.declare_parameter('safety_margin', 0.05)
        self.declare_parameter('map_edge_tolerance', 0.35)
        self.declare_parameter('lookahead_sec', 0.70)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_linear_x', 0.45)
        self.declare_parameter('max_linear_y', 0.20)
        self.declare_parameter('max_angular_z', 0.65)
        self.declare_parameter(
            'model_positive_yaw_lateral_drift', 0.15
        )
        self.declare_parameter(
            'model_negative_yaw_lateral_drift', 0.10
        )
        self.declare_parameter(
            'model_opposite_lateral_uncertainty', 0.05
        )
        self.declare_parameter('model_reference_yaw_rate', 0.65)
        # The official-policy cold matrix validated this rolling command.
        # Pure-yaw recovery is intentionally forbidden.
        self.declare_parameter('recovery_forward_speed', 0.35)
        self.declare_parameter(
            'recovery_positive_yaw_lateral_drift', 0.15
        )
        self.declare_parameter(
            'recovery_negative_yaw_lateral_drift', 0.10
        )
        self.declare_parameter(
            'recovery_opposite_lateral_uncertainty', 0.05
        )
        self.declare_parameter('recovery_lookahead_sec', 0.90)
        self.declare_parameter('recovery_min_yaw_rate', 0.20)
        self.declare_parameter('recovery_clear_confirm_sec', 0.30)
        self.declare_parameter('recovery_rearm_clear_sec', 1.50)
        self.declare_parameter('recovery_max_active_sec', 6.0)
        self.declare_parameter('recovery_max_displacement_m', 0.75)
        self.declare_parameter('recovery_progress_timeout_sec', 1.50)
        self.declare_parameter('recovery_min_progress_m', 0.03)

        self._geometry: Optional[GridGeometry] = None
        self._blocked = None
        self._hard_body_blocked = None
        self._odom: Optional[Odometry] = None
        self._command = (0.0, 0.0, 0.0)
        self._command_time: Optional[float] = None
        self._map_ready = False
        self._active_recovery: Optional[tuple[float, float, float]] = None
        self._recovery_clear_since: Optional[float] = None
        self._recovery_rearm_clear_since: Optional[float] = None
        self._recovery_started_at: Optional[float] = None
        self._recovery_start_xy: Optional[tuple[float, float]] = None
        self._recovery_progress_at: Optional[float] = None
        self._recovery_progress_xy: Optional[tuple[float, float]] = None
        self._recovery_exhausted_reason = ''
        self._last_state = ''
        self._last_diagnostic_key = ''
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stop_publisher = self.create_publisher(
            Bool, str(self.get_parameter('stop_topic').value), latched_qos
        )
        self._state_publisher = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), latched_qos
        )
        self._diagnostic_publisher = self.create_publisher(
            String,
            str(self.get_parameter('diagnostic_topic').value),
            latched_qos,
        )
        self._recovery_available_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('recovery_available_topic').value),
            latched_qos,
        )
        self._recovery_command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('recovery_command_topic').value),
            20,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('occupancy_topic').value),
            self._map_callback,
            latched_qos,
        )
        self._map_ready_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter('map_ready_topic').value),
            self._map_ready_callback,
            latched_qos,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter('command_topic').value),
            self._command_callback,
            20,
        )
        rate = max(
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self._timer = self.create_timer(1.0 / rate, self._evaluate)
        self._publish(True, 'NOT_READY')
        self._publish_recovery(None)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _map_callback(self, message: OccupancyGrid) -> None:
        geometry = GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            edge_tolerance=float(
                self.get_parameter('map_edge_tolerance').value
            ),
        )
        physical_radius = (
            float(self.get_parameter('footprint_radius').value)
            + float(self.get_parameter('safety_margin').value)
        )
        hard_body_radius = hard_body_raster_radius(
            float(self.get_parameter('footprint_radius').value),
            geometry.resolution,
        )
        raster_radius = conservative_raster_radius(
            physical_radius,
            geometry.resolution,
        )
        self._hard_body_blocked = inflate_blocked_grid(
            message.data,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            hard_body_radius,
        )
        self._blocked = inflate_blocked_grid(
            message.data,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            raster_radius,
        )
        self._geometry = geometry
        self._reset_recovery()
        self.get_logger().info(
            'collision guard map ready; '
            f'double-circle radius={physical_radius:.2f}m, '
            f'conservative raster radius={raster_radius:.3f}m, '
            f'hard-body raster radius={hard_body_radius:.3f}m, '
            f'offset='
            f'{float(self.get_parameter("footprint_offset").value):.2f}m, '
            f'edge tolerance={geometry.edge_tolerance:.2f}m'
        )
        minimum_turn_radius = float(
            self.get_parameter('minimum_centerline_turn_radius').value
        )
        self.get_logger().info(
            'M20 capability profile='
            f'{self.get_parameter("capability_profile_id").value}; '
            'minimum centreline turn radius='
            f'{minimum_turn_radius:.3f}m; '
            'outer body sweep radius='
            f'{float(self.get_parameter("turn_swept_radius").value):.3f}m'
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._odom = message

    def _map_ready_callback(self, message: Bool) -> None:
        self._map_ready = message.data
        if not self._map_ready:
            self._reset_recovery()
            self._publish(True, 'MAP_NOT_READY')

    def _command_callback(self, message: Twist) -> None:
        # This is the adapted pre-safety candidate, so prediction and backend
        # execution share one body-command model without reading safe output.
        max_x = abs(float(self.get_parameter('max_linear_x').value))
        max_y = abs(float(self.get_parameter('max_linear_y').value))
        max_yaw = abs(float(self.get_parameter('max_angular_z').value))
        self._command = (
            max(-max_x, min(max_x, float(message.linear.x))),
            max(-max_y, min(max_y, float(message.linear.y))),
            max(-max_yaw, min(max_yaw, float(message.angular.z))),
        )
        self._command_time = self._now()

    def _publish(
        self,
        stop: bool,
        state: str,
        diagnostic: str = '',
    ) -> None:
        self._stop_publisher.publish(Bool(data=stop))
        state_changed = state != self._last_state
        diagnostic_text = diagnostic or state
        diagnostic_key = diagnostic_text.split(';', 1)[0]
        if state_changed:
            self._state_publisher.publish(String(data=state))
            self.get_logger().info(f'collision guard state: {state}')
            self._last_state = state
        if state_changed or diagnostic_key != self._last_diagnostic_key:
            self._diagnostic_publisher.publish(
                String(data=diagnostic_text)
            )
            if diagnostic:
                self.get_logger().info(
                    f'collision guard diagnostic: {diagnostic}'
                )
            self._last_diagnostic_key = diagnostic_key

    def _reset_recovery(self) -> None:
        """Clear active command, budget and any exhaustion latch."""
        self._clear_active_recovery()
        self._recovery_rearm_clear_since = None
        self._recovery_started_at = None
        self._recovery_start_xy = None
        self._recovery_progress_at = None
        self._recovery_progress_xy = None
        self._recovery_exhausted_reason = ''

    def _clear_active_recovery(self) -> None:
        """End the current escape command without rearming its budget."""
        self._active_recovery = None
        self._recovery_clear_since = None

    def _recovery_episode_exists(self) -> bool:
        """Return whether a recovery budget or exhaustion latch is live."""
        return (
            self._recovery_started_at is not None
            or bool(self._recovery_exhausted_reason)
        )

    def _start_recovery_budget(
        self,
        now: float,
        xy: tuple[float, float],
    ) -> None:
        """Start once; changing recovery direction cannot reset the budget."""
        if self._recovery_started_at is not None:
            return
        self._recovery_started_at = now
        self._recovery_start_xy = xy
        self._recovery_progress_at = now
        self._recovery_progress_xy = xy

    def _update_recovery_budget(
        self,
        now: float,
        xy: tuple[float, float],
    ) -> str | None:
        """Return a latched reason when recovery must stop."""
        if self._recovery_exhausted_reason:
            return self._recovery_exhausted_reason
        if (
            self._recovery_started_at is None
            or self._recovery_start_xy is None
            or self._recovery_progress_at is None
            or self._recovery_progress_xy is None
        ):
            return None
        progress_at, progress_xy, reason = update_recovery_budget(
            self._recovery_started_at,
            self._recovery_start_xy,
            self._recovery_progress_at,
            self._recovery_progress_xy,
            now,
            xy,
            float(self.get_parameter('recovery_max_active_sec').value),
            float(
                self.get_parameter('recovery_max_displacement_m').value
            ),
            float(
                self.get_parameter(
                    'recovery_progress_timeout_sec'
                ).value
            ),
            float(self.get_parameter('recovery_min_progress_m').value),
        )
        self._recovery_progress_at = progress_at
        self._recovery_progress_xy = progress_xy
        if reason:
            self._recovery_exhausted_reason = reason
            self._active_recovery = None
        return reason

    def _publish_recovery(
        self,
        command: Optional[tuple[float, float, float]],
    ) -> None:
        """Publish only a collision-checked rolling recovery command."""
        message = Twist()
        available = command is not None
        if command is not None:
            message.linear.x = float(command[0])
            message.linear.y = 0.0
            message.angular.z = float(command[2])
        self._recovery_command_publisher.publish(message)
        self._recovery_available_publisher.publish(
            Bool(data=available)
        )

    def _evaluate(self) -> None:
        if (
            not self._map_ready
            or self._geometry is None
            or self._blocked is None
            or self._hard_body_blocked is None
            or self._odom is None
        ):
            self._reset_recovery()
            self._publish_recovery(None)
            self._publish(True, 'NOT_READY')
            return
        now = self._now()
        command = self._command
        if (
            self._command_time is None
            or now - self._command_time
            > float(self.get_parameter('command_timeout_sec').value)
        ):
            command = (0.0, 0.0, 0.0)
        position = self._odom.pose.pose.position
        current_xy = (float(position.x), float(position.y))
        sample_period = float(
            self.get_parameter('sample_period_sec').value
        )
        blocked_sample = first_blocking_command_envelope(
            self._blocked,
            self._geometry,
            (
                float(position.x),
                float(position.y),
                _yaw_from_odometry(self._odom),
            ),
            command,
            float(self.get_parameter('lookahead_sec').value),
            sample_period,
            float(self.get_parameter('footprint_offset').value),
            float(
                self.get_parameter(
                    'model_positive_yaw_lateral_drift'
                ).value
            ),
            float(
                self.get_parameter(
                    'model_negative_yaw_lateral_drift'
                ).value
            ),
            float(
                self.get_parameter(
                    'model_opposite_lateral_uncertainty'
                ).value
            ),
            float(
                self.get_parameter('model_reference_yaw_rate').value
            ),
        )
        danger = blocked_sample is not None
        diagnostic = 'CLEAR'
        recovery = None
        if blocked_sample is not None:
            self._recovery_clear_since = None
            # Any renewed prediction breaks the continuous-clear rearm timer.
            self._recovery_rearm_clear_since = None
            trigger = (
                'CURRENT_FOOTPRINT'
                if blocked_sample.sample_index == 0
                else 'PREDICTED_FOOTPRINT'
            )
            diagnostic = (
                f'{trigger}; cause={blocked_sample.cause}; '
                f'model={blocked_sample.motion_model}; '
                f't={blocked_sample.time_sec:.2f}s; '
                f'circle={blocked_sample.circle}; '
                f'xy=({blocked_sample.x:.2f},'
                f'{blocked_sample.y:.2f}); '
                f'yaw={blocked_sample.yaw:.2f}; '
                f'cell=({blocked_sample.cell_x},'
                f'{blocked_sample.cell_y}); '
                f'cmd=({command[0]:.2f},'
                f'{command[1]:.2f},{command[2]:.2f})'
            )
            if blocked_sample.sample_index > 0:
                pose = (
                    float(position.x),
                    float(position.y),
                    _yaw_from_odometry(self._odom),
                )
                recovery_horizon = float(
                    self.get_parameter('recovery_lookahead_sec').value
                )
                footprint_offset = float(
                    self.get_parameter('footprint_offset').value
                )
                positive_drift = float(
                    self.get_parameter(
                        'recovery_positive_yaw_lateral_drift'
                    ).value
                )
                negative_drift = float(
                    self.get_parameter(
                        'recovery_negative_yaw_lateral_drift'
                    ).value
                )
                opposite_drift = float(
                    self.get_parameter(
                        'recovery_opposite_lateral_uncertainty'
                    ).value
                )
                budget_reason = self._update_recovery_budget(
                    now, current_xy
                )
                # Preserve a selected direction while its full sweep is safe.
                # Direction changes share one budget and cannot restart it.
                if budget_reason:
                    diagnostic = (
                        'RECOVERY_BUDGET_EXHAUSTED; '
                        f'reason={budget_reason}; trigger={diagnostic}'
                    )
                elif (
                    self._active_recovery is not None
                    and recovery_sweep_is_clear(
                        self._blocked,
                        self._geometry,
                        pose,
                        self._active_recovery,
                        recovery_horizon,
                        sample_period,
                        footprint_offset,
                        positive_drift,
                        negative_drift,
                        opposite_drift,
                    )
                ):
                    recovery = self._active_recovery
                elif not self._recovery_exhausted_reason:
                    recovery = safe_rolling_recovery(
                        self._blocked,
                        self._geometry,
                        pose,
                        command,
                        recovery_horizon,
                        sample_period,
                        footprint_offset,
                        float(
                            self.get_parameter(
                                'recovery_forward_speed'
                            ).value
                        ),
                        positive_drift,
                        negative_drift,
                        opposite_drift,
                        float(
                            self.get_parameter(
                                'recovery_min_yaw_rate'
                            ).value
                        ),
                    )
                self._active_recovery = recovery
                if recovery is not None:
                    self._start_recovery_budget(now, current_xy)
                    recovery_text = (
                        f'recovery=({recovery[0]:.2f},'
                        f'0.00,{recovery[2]:.2f})'
                    )
                    if (
                        blocked_sample.cause == 'OUT_OF_BOUNDS'
                        and command[0] < -1.0e-3
                        and recovery[0] > 1.0e-3
                        and abs(recovery[2]) <= 1.0e-6
                    ):
                        diagnostic = (
                            'MAP_EDGE_INWARD_RECOVERY; '
                            f'trigger={diagnostic}; {recovery_text}'
                        )
                    else:
                        diagnostic += f'; {recovery_text}'
                elif not self._recovery_exhausted_reason:
                    # A predicted collision with neither validated rolling
                    # escape direction must be observable by the navigation
                    # gateway.  Without this terminal diagnostic the guard
                    # safely holds zero speed forever, but no recovery budget
                    # ever starts and the active SCAN goal cannot be reset.
                    diagnostic = (
                        'RECOVERY_UNAVAILABLE; '
                        f'trigger={diagnostic}'
                    )
            else:
                # A hard-body overlap never authorizes motion. A false current
                # hit created only by the conservative half-cell shell may use
                # one bounded straight escape whose hard sweep is clear and
                # whose endpoint returns to conservative free space.
                recovery = safe_raster_shell_escape(
                    self._blocked,
                    self._hard_body_blocked,
                    self._geometry,
                    (
                        float(position.x),
                        float(position.y),
                        _yaw_from_odometry(self._odom),
                    ),
                    float(
                        self.get_parameter('recovery_lookahead_sec').value
                    ),
                    sample_period,
                    float(self.get_parameter('footprint_offset').value),
                    float(
                        self.get_parameter('recovery_forward_speed').value
                    ),
                    float(
                        self.get_parameter(
                            'recovery_positive_yaw_lateral_drift'
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            'recovery_negative_yaw_lateral_drift'
                        ).value
                    ),
                    float(
                        self.get_parameter(
                            'recovery_opposite_lateral_uncertainty'
                        ).value
                    ),
                )
                self._active_recovery = recovery
                if recovery is not None:
                    self._start_recovery_budget(now, current_xy)
                    diagnostic = (
                        'RASTER_SHELL_ESCAPE; '
                        f'trigger={diagnostic}; '
                        f'recovery=({recovery[0]:.2f},0.00,0.00)'
                    )
                else:
                    # Preserve any spent episode for the gateway; reset only
                    # after a continuous conservative-clear interval.
                    self._clear_active_recovery()
        else:
            # A single clear grid lookup at an obstacle-cell boundary is not
            # enough to release recovery.  Require a continuous clear period
            # while the already selected recovery sweep remains safe.
            if self._active_recovery is not None:
                pose = (
                    float(position.x),
                    float(position.y),
                    _yaw_from_odometry(self._odom),
                )
                recovery_horizon = float(
                    self.get_parameter('recovery_lookahead_sec').value
                )
                footprint_offset = float(
                    self.get_parameter('footprint_offset').value
                )
                positive_drift = float(
                    self.get_parameter(
                        'recovery_positive_yaw_lateral_drift'
                    ).value
                )
                negative_drift = float(
                    self.get_parameter(
                        'recovery_negative_yaw_lateral_drift'
                    ).value
                )
                opposite_drift = float(
                    self.get_parameter(
                        'recovery_opposite_lateral_uncertainty'
                    ).value
                )
                budget_reason = self._update_recovery_budget(
                    now, current_xy
                )
                recovery_clear = False
                if not budget_reason:
                    recovery_clear = recovery_sweep_is_clear(
                        self._blocked,
                        self._geometry,
                        pose,
                        self._active_recovery,
                        recovery_horizon,
                        sample_period,
                        footprint_offset,
                        positive_drift,
                        negative_drift,
                        opposite_drift,
                    )
                clear_since, confirmed = update_clear_confirmation(
                    self._recovery_clear_since,
                    now,
                    float(
                        self.get_parameter(
                            'recovery_clear_confirm_sec'
                        ).value
                    ),
                )
                self._recovery_clear_since = clear_since
                if budget_reason:
                    self._clear_active_recovery()
                    self._recovery_rearm_clear_since = clear_since
                    elapsed = max(0.0, now - float(clear_since))
                    diagnostic = (
                        'RECOVERY_EPISODE_REARM; '
                        f'clear_for={elapsed:.2f}s; '
                        f'reason={budget_reason}'
                    )
                elif recovery_clear and not confirmed:
                    danger = True
                    recovery = self._active_recovery
                    elapsed = max(0.0, now - float(clear_since))
                    diagnostic = (
                        'RECOVERY_CLEAR_CONFIRM; '
                        f'clear_for={elapsed:.2f}s; '
                        f'recovery=({recovery[0]:.2f},'
                        f'0.00,{recovery[2]:.2f})'
                    )
                else:
                    # End active recovery after the short release confirmation
                    # (or immediately when its own sweep becomes unsafe), but
                    # retain the episode budget until the longer rearm period.
                    self._clear_active_recovery()
                    self._recovery_rearm_clear_since = clear_since
                    elapsed = max(0.0, now - float(clear_since))
                    diagnostic = (
                        'RECOVERY_EPISODE_REARM; '
                        f'clear_for={elapsed:.2f}s'
                    )
            if (
                self._active_recovery is None
                and self._recovery_episode_exists()
            ):
                rearm_since, rearmed = update_clear_confirmation(
                    self._recovery_rearm_clear_since,
                    now,
                    float(
                        self.get_parameter(
                            'recovery_rearm_clear_sec'
                        ).value
                    ),
                )
                self._recovery_rearm_clear_since = rearm_since
                if rearmed:
                    self._reset_recovery()
                    diagnostic = 'CLEAR'
                else:
                    elapsed = max(0.0, now - float(rearm_since))
                    suffix = (
                        f'; reason={self._recovery_exhausted_reason}'
                        if self._recovery_exhausted_reason
                        else ''
                    )
                    diagnostic = (
                        'RECOVERY_EPISODE_REARM; '
                        f'clear_for={elapsed:.2f}s'
                        f'{suffix}'
                    )
        self._publish_recovery(recovery)
        self._publish(
            danger,
            'COLLISION_STOP' if danger else 'CLEAR',
            diagnostic,
        )


def main() -> None:
    """Run the independent collision guard."""
    rclpy.init()
    node = CollisionGuard()
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
