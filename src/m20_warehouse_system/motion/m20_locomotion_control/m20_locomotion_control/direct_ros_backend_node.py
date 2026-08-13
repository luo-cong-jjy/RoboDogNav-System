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

"""Real M20 backend using the official DrDDS ROS 2 motion topics."""

from __future__ import annotations

import json
from typing import Optional, Tuple

from drdds.msg import Gait, MotionInfo, MotionState, NavCmd, StdMsgInt32
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .basic_server_protocol import apply_vendor_velocity_envelope
from .direct_ros_policy import (
    DirectMotionStatus,
    SOFT_ESTOP,
    select_direct_transition,
)


class DirectRosBackend(Node):
    """Translate the common safe Twist into ``/NAV_CMD`` DrDDS messages."""

    def __init__(self) -> None:
        super().__init__('m20_direct_ros_backend')
        self.declare_parameter('input_topic', '/m20/locomotion/cmd_vel_sdk')
        self.declare_parameter('e_stop_topic', '/m20/control/e_stop')
        self.declare_parameter('ready_topic', '/m20/locomotion/backend_ready')
        self.declare_parameter('fault_topic', '/m20/locomotion/backend_fault')
        self.declare_parameter(
            'status_topic', '/m20/locomotion/backend_status'
        )
        self.declare_parameter(
            'measured_twist_topic', '/m20/locomotion/measured_twist'
        )
        self.declare_parameter(
            'enable_service', '/m20/hardware/enable_motion'
        )
        self.declare_parameter('nav_cmd_topic', '/NAV_CMD')
        self.declare_parameter('motion_info_topic', '/MOTION_INFO')
        self.declare_parameter('motion_state_topic', '/MOTION_STATE')
        self.declare_parameter('gait_topic', '/GAIT')
        self.declare_parameter('hard_estop_topic', '/HES_STATUS')
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('status_timeout_sec', 1.25)
        self.declare_parameter('hard_estop_timeout_sec', 2.50)
        self.declare_parameter('state_command_period_sec', 1.0)
        self.declare_parameter('requested_gait', 0x3002)
        self.declare_parameter('auto_enable_motion', False)
        self.declare_parameter('lie_down_on_disable', False)
        self.declare_parameter('command_ownership_confirmed', False)
        self.declare_parameter('subthreshold_policy', 'zero')

        self._command_period = 1.0 / max(
            20.0, float(self.get_parameter('command_rate_hz').value)
        )
        self._command_timeout = min(
            0.45,
            max(
                0.05,
                float(self.get_parameter('command_timeout_sec').value),
            ),
        )
        self._status_timeout = max(
            0.50, float(self.get_parameter('status_timeout_sec').value)
        )
        self._hard_estop_timeout = max(
            1.25,
            float(self.get_parameter('hard_estop_timeout_sec').value),
        )
        self._state_command_period = max(
            0.25,
            float(self.get_parameter('state_command_period_sec').value),
        )
        self._requested_gait = int(
            self.get_parameter('requested_gait').value
        )
        self._ownership_confirmed = bool(
            self.get_parameter('command_ownership_confirmed').value
        )
        self._lie_down_on_disable = bool(
            self.get_parameter('lie_down_on_disable').value
        )
        self._subthreshold_policy = str(
            self.get_parameter('subthreshold_policy').value
        ).lower()

        self._frame_id = 0
        self._command = (0.0, 0.0, 0.0)
        self._last_command_time = -1.0e9
        self._last_velocity_send = -1.0e9
        self._last_state_send = -1.0e9
        self._last_gait_send = -1.0e9
        self._last_info_time = -1.0e9
        self._last_hard_estop_time = -1.0e9
        self._enable_requested_at = -1.0e9
        self._status: Optional[DirectMotionStatus] = None
        self._motion_requested = bool(
            self.get_parameter('auto_enable_motion').value
        )
        self._e_stop = False
        self._hard_estop = False
        self._hard_estop_known = False
        self._hard_estop_latched = False
        self._ready = False
        self._fault = ''
        self._last_status_text = ''
        self._last_suppressed_axes: Tuple[str, ...] = ()

        command_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        feedback_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._nav_publisher = self.create_publisher(
            NavCmd, str(self.get_parameter('nav_cmd_topic').value), command_qos
        )
        self._state_publisher = self.create_publisher(
            MotionState,
            str(self.get_parameter('motion_state_topic').value),
            command_qos,
        )
        self._gait_publisher = self.create_publisher(
            Gait, str(self.get_parameter('gait_topic').value), command_qos
        )
        self._ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), latched_qos
        )
        self._fault_publisher = self.create_publisher(
            String, str(self.get_parameter('fault_topic').value), latched_qos
        )
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), latched_qos
        )
        self._twist_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('measured_twist_topic').value),
            20,
        )
        self.create_subscription(
            MotionInfo,
            str(self.get_parameter('motion_info_topic').value),
            self._motion_info_callback,
            feedback_qos,
        )
        self.create_subscription(
            StdMsgInt32,
            str(self.get_parameter('hard_estop_topic').value),
            self._hard_estop_callback,
            feedback_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('e_stop_topic').value),
            self._e_stop_callback,
            latched_qos,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter('enable_service').value),
            self._enable_callback,
        )
        self._timer = self.create_timer(0.02, self._update)
        if self._motion_requested:
            self._enable_requested_at = self._now()
        self._publish_health(force=True)
        self.get_logger().info(
            'Official M20 direct ROS backend configured; motion starts '
            'disabled unless explicitly enabled'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _fill_header(self, message) -> None:
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFFFFFFFFFF
        stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.header.stamp = stamp

    def _command_callback(self, message: Twist) -> None:
        self._command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self._last_command_time = self._now()

    def _motion_info_callback(self, message: MotionInfo) -> None:
        data = message.data
        self._status = DirectMotionStatus(
            state=int(data.state),
            gait=int(data.gait),
            linear_x=float(data.vel_x),
            linear_y=float(data.vel_y),
            angular_z=float(data.vel_yaw),
        )
        self._last_info_time = self._now()
        measured = TwistStamped()
        measured.header.stamp = self.get_clock().now().to_msg()
        measured.header.frame_id = 'base_link'
        measured.twist.linear.x = self._status.linear_x
        measured.twist.linear.y = self._status.linear_y
        measured.twist.angular.z = self._status.angular_z
        self._twist_publisher.publish(measured)

    def _hard_estop_callback(self, message: StdMsgInt32) -> None:
        """Latch the physical tail-button state until an operator resets it."""
        asserted = bool(message.value)
        self._hard_estop_known = True
        self._last_hard_estop_time = self._now()
        if asserted:
            self._hard_estop_latched = True
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            for _ in range(3):
                self._publish_velocity((0.0, 0.0, 0.0))
            if not self._hard_estop:
                self.get_logger().error(
                    'physical M20 hard e-stop asserted; manual reset required'
                )
        elif self._hard_estop:
            self.get_logger().warn(
                'physical M20 hard e-stop released; robot remains disabled '
                'until enable_motion false then true'
            )
        self._hard_estop = asserted
        self._publish_health(force=True)

    def _e_stop_callback(self, message: Bool) -> None:
        asserted = bool(message.data)
        if asserted and not self._e_stop:
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            for _ in range(3):
                self._publish_velocity((0.0, 0.0, 0.0))
            self._publish_state(SOFT_ESTOP)
            self.get_logger().error(
                'e-stop asserted: zero /NAV_CMD and soft e-stop published'
            )
        self._e_stop = asserted
        self._publish_health(force=True)

    def _enable_callback(self, request, response):
        enable = bool(request.data)
        if enable and self._e_stop:
            response.success = False
            response.message = 'release /m20/control/e_stop first'
            return response
        hard_estop_fresh = bool(
            self._hard_estop_known
            and self._now() - self._last_hard_estop_time
            <= self._hard_estop_timeout
        )
        if enable and not hard_estop_fresh:
            response.success = False
            response.message = (
                'waiting for fresh /HES_STATUS before motion enable'
            )
            return response
        if enable and self._hard_estop:
            response.success = False
            response.message = 'physical M20 hard e-stop is asserted'
            return response
        if enable and self._hard_estop_latched:
            response.success = False
            response.message = (
                'hard e-stop release is latched; call enable_motion false '
                'once, inspect the robot, then request true'
            )
            return response
        if enable and not self._ownership_confirmed:
            response.success = False
            response.message = (
                'command ownership not confirmed; stop the onboard planner '
                'and autonomous charging, select navigation mode, then set '
                'command_ownership_confirmed:=true'
            )
            return response
        self._motion_requested = enable
        self._command = (0.0, 0.0, 0.0)
        self._last_command_time = self._now()
        self._enable_requested_at = self._now() if enable else -1.0e9
        if not enable:
            for _ in range(3):
                self._publish_velocity((0.0, 0.0, 0.0))
            if (
                self._lie_down_on_disable
                and self._status is not None
                and self._status.state == 17
                and self._status.stationary
            ):
                self._publish_state(4)
            hard_estop_fresh = bool(
                self._hard_estop_known
                and self._now() - self._last_hard_estop_time
                <= self._hard_estop_timeout
            )
            if hard_estop_fresh and not self._hard_estop:
                self._hard_estop_latched = False
        response.success = True
        response.message = (
            'direct ROS motion enable sequence requested'
            if enable
            else 'motion disabled; zero /NAV_CMD published'
        )
        self._publish_health(force=True)
        return response

    def _publish_state(self, value: int) -> None:
        message = MotionState()
        self._fill_header(message)
        message.data.state = int(value)
        self._state_publisher.publish(message)

    def _publish_gait(self, value: int) -> None:
        message = Gait()
        self._fill_header(message)
        message.data.gait = int(value)
        self._gait_publisher.publish(message)

    def _publish_velocity(self, command) -> None:
        message = NavCmd()
        self._fill_header(message)
        message.data.x_vel = float(command[0])
        message.data.y_vel = float(command[1])
        message.data.yaw_vel = float(command[2])
        self._nav_publisher.publish(message)

    def _publish_periodic_velocity(self, now: float) -> None:
        if now - self._last_velocity_send < self._command_period:
            return
        self._last_velocity_send = now
        command = (0.0, 0.0, 0.0)
        fresh = now - self._last_command_time <= self._command_timeout
        if self._ready and fresh:
            command, suppressed = apply_vendor_velocity_envelope(
                self._command,
                self._requested_gait,
                self._subthreshold_policy,
            )
            if suppressed != self._last_suppressed_axes:
                if suppressed:
                    self.get_logger().warn(
                        'official velocity dead zone suppressed axes: '
                        + ','.join(suppressed)
                    )
                self._last_suppressed_axes = suppressed
        else:
            self._last_suppressed_axes = ()
        if self._motion_requested or self._ready:
            self._publish_velocity(command)

    def _publish_health(self, force: bool = False) -> None:
        status = self._status
        age = (
            max(0.0, self._now() - self._last_info_time)
            if status is not None
            else None
        )
        data = {
            'transport': 'direct_ros',
            'motion_requested': self._motion_requested,
            'ready': self._ready,
            'fault': self._fault,
            'e_stop': self._e_stop,
            'hard_estop': self._hard_estop,
            'hard_estop_known': self._hard_estop_known,
            'hard_estop_latched': self._hard_estop_latched,
            'command_ownership_confirmed': self._ownership_confirmed,
            'motion_info_age_sec': age,
            'motion_state': status.state if status else None,
            'gait': status.gait if status else None,
            'nav_cmd_subscribers': self._nav_publisher.get_subscription_count(),
        }
        text = json.dumps(data, sort_keys=True, separators=(',', ':'))
        if not force and text == self._last_status_text:
            return
        self._ready_publisher.publish(Bool(data=self._ready))
        self._fault_publisher.publish(String(data=self._fault))
        self._status_publisher.publish(String(data=text))
        self._last_status_text = text

    def _update(self) -> None:
        now = self._now()
        fresh_status = self._status
        if now - self._last_info_time > self._status_timeout:
            fresh_status = None
        hard_estop_fresh = bool(
            self._hard_estop_known
            and now - self._last_hard_estop_time <= self._hard_estop_timeout
        )
        decision = select_direct_transition(
            motion_requested=self._motion_requested,
            ownership_confirmed=self._ownership_confirmed,
            e_stop=self._e_stop,
            status=fresh_status,
            requested_gait=self._requested_gait,
        )
        fault = decision.fault
        if not hard_estop_fresh:
            fault = 'HARD_ESTOP_STATUS_TIMEOUT'
        elif self._hard_estop:
            fault = 'M20_HARD_ESTOP_ASSERTED'
        elif self._hard_estop_latched:
            fault = 'M20_HARD_ESTOP_RELEASE_LATCHED'
        elif (
            self._motion_requested
            and fresh_status is None
            and now - self._enable_requested_at > self._status_timeout
        ):
            fault = 'MOTION_INFO_TIMEOUT'
        self._fault = fault
        self._ready = decision.ready and not fault
        if (
            not fault
            and decision.state_command is not None
            and now - self._last_state_send >= self._state_command_period
        ):
            self._publish_state(decision.state_command)
            self._last_state_send = now
        if (
            not fault
            and decision.gait_command is not None
            and now - self._last_gait_send >= self._state_command_period
        ):
            self._publish_gait(decision.gait_command)
            self._last_gait_send = now
        self._publish_periodic_velocity(now)
        self._publish_health()

    def stop(self) -> None:
        """Publish redundant zero commands before ROS teardown."""
        self._ready = False
        for _ in range(3):
            self._publish_velocity((0.0, 0.0, 0.0))
        self._publish_health(force=True)


def main(args=None) -> None:
    """Run the M20 direct ROS factory motion backend."""
    rclpy.init(args=args)
    node = DirectRosBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
