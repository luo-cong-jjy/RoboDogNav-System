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

"""Real M20 motion backend using the official ``basic_server`` protocol."""

import errno
import json
import socket
from typing import Optional, Tuple

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .basic_server_protocol import (
    ApduStreamDecoder,
    BasicStatus,
    MotionStatus,
    apply_vendor_velocity_envelope,
    encode_json_apdu,
    make_patrol_message,
    parse_basic_status,
    parse_device_errors,
    parse_error,
    parse_motion_status,
)


class BasicServerBackend(Node):
    """Translate the common safe Twist stream to M20 factory motion control."""

    def __init__(self) -> None:
        super().__init__('m20_basic_server_backend')
        self.declare_parameter('robot_host', '10.21.31.103')
        self.declare_parameter('udp_port', 30000)
        self.declare_parameter('tcp_port', 30001)
        self.declare_parameter(
            'input_topic', '/m20/locomotion/cmd_vel_sdk'
        )
        self.declare_parameter(
            'e_stop_topic', '/m20/control/e_stop'
        )
        self.declare_parameter(
            'ready_topic', '/m20/locomotion/backend_ready'
        )
        self.declare_parameter(
            'fault_topic', '/m20/locomotion/backend_fault'
        )
        self.declare_parameter(
            'status_topic', '/m20/locomotion/backend_status'
        )
        self.declare_parameter(
            'measured_twist_topic',
            '/m20/locomotion/measured_twist',
        )
        self.declare_parameter(
            'enable_service', '/m20/hardware/enable_motion'
        )
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.30)
        self.declare_parameter('status_timeout_sec', 1.25)
        self.declare_parameter('connect_retry_sec', 2.0)
        self.declare_parameter('requested_control_usage_mode', 1)
        self.declare_parameter('requested_gait', 0x3002)
        self.declare_parameter('auto_enable_motion', False)
        self.declare_parameter('lie_down_on_disable', False)
        self.declare_parameter('command_ownership_confirmed', False)
        self.declare_parameter('subthreshold_policy', 'zero')

        self._host = str(self.get_parameter('robot_host').value)
        self._udp_port = int(self.get_parameter('udp_port').value)
        self._tcp_port = int(self.get_parameter('tcp_port').value)
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
        self._connect_retry = max(
            0.25, float(self.get_parameter('connect_retry_sec').value)
        )
        self._requested_usage_mode = int(
            self.get_parameter('requested_control_usage_mode').value
        )
        self._requested_gait = int(
            self.get_parameter('requested_gait').value
        )
        self._lie_down_on_disable = bool(
            self.get_parameter('lie_down_on_disable').value
        )
        self._ownership_confirmed = bool(
            self.get_parameter('command_ownership_confirmed').value
        )
        self._subthreshold_policy = str(
            self.get_parameter('subthreshold_policy').value
        ).lower()

        self._tcp: Optional[socket.socket] = None
        self._udp: Optional[socket.socket] = None
        self._tcp_decoder = ApduStreamDecoder()
        self._udp_decoder = ApduStreamDecoder()
        self._frame_id = 0
        self._last_connect_attempt = -1.0e9
        self._connected_at = 0.0
        self._last_heartbeat = -1.0e9
        self._last_state_command = -1.0e9
        self._last_gait_command = -1.0e9
        self._last_mode_command = -1.0e9
        self._last_velocity_send = -1.0e9
        self._last_command_time = -1.0e9
        self._last_basic_status_time = -1.0e9
        self._last_motion_status_time = -1.0e9
        self._basic_status: Optional[BasicStatus] = None
        self._motion_status: Optional[MotionStatus] = None
        self._command = (0.0, 0.0, 0.0)
        self._motion_requested = bool(
            self.get_parameter('auto_enable_motion').value
        )
        self._e_stop = False
        self._hard_estop = False
        self._hard_estop_known = False
        self._hard_estop_latched = False
        self._device_fault_latched = False
        self._device_fault_text = ''
        self._command_fault = ''
        self._soft_estop_sent = False
        self._ready = False
        self._fault = ''
        self._last_status_text = ''
        self._last_suppressed_axes: Tuple[str, ...] = ()

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), latched_qos
        )
        self._fault_publisher = self.create_publisher(
            String, str(self.get_parameter('fault_topic').value), latched_qos
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            latched_qos,
        )
        self._twist_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('measured_twist_topic').value),
            20,
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
        self._publish_health(force=True)
        self.get_logger().info(
            'Official M20 basic_server backend configured for '
            f'{self._host}:{self._udp_port}/{self._tcp_port}; motion starts '
            'disabled unless explicitly enabled'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _command_callback(self, message: Twist) -> None:
        self._command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self._last_command_time = self._now()

    def _e_stop_callback(self, message: Bool) -> None:
        asserted = bool(message.data)
        if asserted and not self._e_stop:
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            self._send_velocity((0.0, 0.0, 0.0))
            self._send_tcp_command(2, 22, {'MotionParam': 2})
            self._soft_estop_sent = True
            self.get_logger().error(
                'e-stop asserted: zero velocity and M20 soft e-stop sent'
            )
        self._e_stop = asserted
        self._publish_health(force=True)

    def _enable_callback(self, request, response):
        enable = bool(request.data)
        if enable and self._e_stop:
            response.success = False
            response.message = 'release /m20/control/e_stop first'
            return response
        status_fresh = bool(
            self._basic_status is not None
            and self._now() - self._last_basic_status_time
            <= self._status_timeout
        )
        if enable and (not status_fresh or not self._hard_estop_known):
            response.success = False
            response.message = (
                'waiting for a fresh BasicStatus with HES before motion enable'
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
        if enable and self._device_fault_latched:
            response.success = False
            response.message = (
                'a factory device fault is latched; resolve the vendor '
                'error, restart this backend, and repeat read-only checks'
            )
            return response
        if enable and not self._ownership_confirmed:
            response.success = False
            response.message = (
                'command ownership not confirmed; stop the onboard planner '
                'and autonomous charging, then set '
                'command_ownership_confirmed:=true'
            )
            return response
        self._motion_requested = enable
        self._command = (0.0, 0.0, 0.0)
        self._last_command_time = self._now()
        if not enable:
            self._send_velocity((0.0, 0.0, 0.0))
            if (
                self._lie_down_on_disable
                and self._basic_status is not None
                and self._basic_status.motion_state == 17
                and self._is_stationary()
            ):
                self._send_tcp_command(2, 22, {'MotionParam': 4})
            if status_fresh and not self._hard_estop:
                self._hard_estop_latched = False
            self._command_fault = ''
        response.success = True
        response.message = (
            'motion enable sequence requested'
            if enable
            else 'motion disabled; zero command sent'
        )
        self._publish_health(force=True)
        return response

    def _connect(self, now: float) -> None:
        if (
            self._tcp is not None
            or now - self._last_connect_attempt < self._connect_retry
        ):
            return
        self._last_connect_attempt = now
        tcp = None
        udp = None
        try:
            tcp = socket.create_connection(
                (self._host, self._tcp_port), timeout=0.10
            )
            tcp.setblocking(False)
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp.setblocking(False)
            udp.connect((self._host, self._udp_port))
        except OSError as exc:
            for channel in (tcp, udp):
                if channel is not None:
                    try:
                        channel.close()
                    except OSError:
                        pass
            self._set_fault(f'BASIC_SERVER_CONNECT:{exc.errno or "IO"}')
            return
        self._tcp = tcp
        self._udp = udp
        self._tcp_decoder = ApduStreamDecoder()
        self._udp_decoder = ApduStreamDecoder()
        self._connected_at = now
        self._last_heartbeat = -1.0e9
        self._basic_status = None
        self._motion_status = None
        self._clear_fault()
        self.get_logger().info('connected to M20 basic_server')

    def _close_sockets(self) -> None:
        for channel in (self._tcp, self._udp):
            if channel is not None:
                try:
                    channel.close()
                except OSError:
                    pass
        self._tcp = None
        self._udp = None
        self._ready = False

    def _next_frame_id(self) -> int:
        self._frame_id = (self._frame_id + 1) & 0xFFFF
        return self._frame_id

    def _packet(self, message_type: int, command: int, items) -> bytes:
        message = make_patrol_message(message_type, command, items)
        return encode_json_apdu(message, self._next_frame_id())

    def _send_tcp_command(self, message_type: int, command: int, items) -> bool:
        if self._tcp is None:
            return False
        packet = self._packet(message_type, command, items)
        try:
            self._tcp.sendall(packet)
            return True
        except OSError as exc:
            self._connection_failed(exc)
            return False

    def _send_velocity(self, command) -> bool:
        if self._udp is None:
            return False
        packet = self._packet(
            2,
            25,
            {
                'X': float(command[0]),
                'Y': float(command[1]),
                'Z': 0.0,
                'Roll': 0.0,
                'Pitch': 0.0,
                'Yaw': float(command[2]),
            },
        )
        try:
            self._udp.send(packet)
            return True
        except OSError as exc:
            self._connection_failed(exc)
            return False

    def _connection_failed(self, exc: OSError) -> None:
        self._close_sockets()
        self._set_fault(f'BASIC_SERVER_IO:{exc.errno or "IO"}')

    def _drain_socket(self, channel, decoder) -> None:
        if channel is None:
            return
        while True:
            try:
                chunk = channel.recv(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in {errno.EAGAIN, errno.EWOULDBLOCK}:
                    return
                self._connection_failed(exc)
                return
            if not chunk:
                self._connection_failed(OSError(errno.ECONNRESET, 'closed'))
                return
            for _, _, message in decoder.feed(chunk):
                self._handle_message(message)

    def _handle_message(self, message) -> None:
        now = self._now()
        basic = parse_basic_status(message)
        if basic is not None:
            previous_hard_estop = self._hard_estop
            self._basic_status = basic
            self._last_basic_status_time = now
            self._hard_estop_known = basic.hard_estop in {0, 1}
            self._hard_estop = basic.hard_estop == 1
            if self._hard_estop:
                self._hard_estop_latched = True
                self._motion_requested = False
                self._command = (0.0, 0.0, 0.0)
                for _ in range(3):
                    self._send_velocity((0.0, 0.0, 0.0))
                if not previous_hard_estop:
                    self.get_logger().error(
                        'physical M20 hard e-stop asserted; manual reset '
                        'and operator acknowledgement required'
                    )
            elif previous_hard_estop:
                self.get_logger().warn(
                    'physical M20 hard e-stop released; robot remains '
                    'disabled until enable_motion false then true'
                )
        motion = parse_motion_status(message)
        if motion is not None:
            self._motion_status = motion
            self._last_motion_status_time = now
            measured = TwistStamped()
            measured.header.stamp = self.get_clock().now().to_msg()
            measured.header.frame_id = 'base_link'
            measured.twist.linear.x = motion.linear_x
            measured.twist.linear.y = motion.linear_y
            measured.twist.angular.z = motion.omega_z
            self._twist_publisher.publish(measured)
        error = parse_error(message)
        if error is not None and error[0] != 0:
            self._command_fault = (
                f'BASIC_SERVER_ERROR_{error[0]}:{error[1]}'
            )
        device_errors = parse_device_errors(message)
        if device_errors:
            summary = ','.join(
                f'{item.code}@{item.component}' for item in device_errors
            )
            self._device_fault_latched = True
            self._device_fault_text = f'M20_DEVICE_ERROR:{summary}'
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            for _ in range(3):
                self._send_velocity((0.0, 0.0, 0.0))
            self.get_logger().error(
                'factory device fault asserted; motion disabled: ' + summary
            )

    def _is_stationary(self) -> bool:
        motion = self._motion_status
        if motion is None:
            return False
        return (
            abs(motion.linear_x) < 0.03
            and abs(motion.linear_y) < 0.03
            and abs(motion.omega_z) < 0.05
        )

    def _advance_factory_state(self, now: float) -> None:
        status = self._basic_status
        if (
            not self._motion_requested
            or status is None
            or self._e_stop
            or self._hard_estop
            or self._hard_estop_latched
            or self._device_fault_latched
            or self._fault
        ):
            return
        if not self._ownership_confirmed:
            self._set_fault('COMMAND_OWNERSHIP_NOT_CONFIRMED')
            return
        if status.motion_state == 2:
            self._command_fault = 'M20_SOFT_ESTOP_LATCHED'
            self._motion_requested = False
            return
        if status.control_usage_mode != self._requested_usage_mode:
            if now - self._last_mode_command >= 1.0:
                self._send_tcp_command(
                    1101, 5, {'Mode': self._requested_usage_mode}
                )
                self._last_mode_command = now
            return
        if status.motion_state != 17:
            if (
                status.motion_state in {0, 3, 4}
                and now - self._last_state_command >= 1.0
            ):
                # basic_server automatically advances stand (1) to RL (17).
                self._send_tcp_command(2, 22, {'MotionParam': 1})
                self._last_state_command = now
            return
        if status.gait != self._requested_gait:
            if self._is_stationary() and now - self._last_gait_command >= 1.0:
                self._send_tcp_command(
                    2, 23, {'GaitParam': self._requested_gait}
                )
                self._last_gait_command = now

    def _desired_ready(self, now: float) -> bool:
        status = self._basic_status
        return bool(
            self._tcp is not None
            and self._udp is not None
            and self._motion_requested
            and not self._e_stop
            and self._hard_estop_known
            and not self._hard_estop
            and not self._hard_estop_latched
            and not self._device_fault_latched
            and self._ownership_confirmed
            and status is not None
            and now - self._last_basic_status_time <= self._status_timeout
            and status.control_usage_mode == self._requested_usage_mode
            and status.motion_state == 17
            and status.gait == self._requested_gait
            and not self._fault
        )

    def _update_faults(self, now: float) -> None:
        if self._tcp is None:
            self._set_fault('BASIC_SERVER_DISCONNECTED')
            return
        if (
            now - self._connected_at > self._status_timeout
            and now - self._last_basic_status_time > self._status_timeout
        ):
            self._set_fault('BASIC_STATUS_TIMEOUT')
            return
        if not self._hard_estop_known:
            self._set_fault('HARD_ESTOP_STATUS_UNAVAILABLE')
            return
        if self._hard_estop:
            self._set_fault('M20_HARD_ESTOP_ASSERTED')
            return
        if self._hard_estop_latched:
            self._set_fault('M20_HARD_ESTOP_RELEASE_LATCHED')
            return
        if self._device_fault_latched:
            self._set_fault(self._device_fault_text or 'M20_DEVICE_ERROR')
            return
        if self._command_fault:
            self._set_fault(self._command_fault)
            return
        self._clear_fault()

    def _send_periodic_velocity(self, now: float) -> None:
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
        # While an enable sequence is active, zero frames keep the UDP client
        # identity and the vendor 500 ms watchdog deterministic.
        if self._motion_requested or self._ready:
            self._send_velocity(command)

    def _set_fault(self, text: str) -> None:
        if text == self._fault:
            return
        self._fault = text
        self._ready = False
        self._publish_health(force=True)

    def _clear_fault(self) -> None:
        if not self._fault:
            return
        self._fault = ''
        self._publish_health(force=True)

    def _publish_health(self, force: bool = False) -> None:
        status = self._basic_status
        data = {
            'transport': 'basic_server',
            'connected': self._tcp is not None and self._udp is not None,
            'motion_requested': self._motion_requested,
            'ready': self._ready,
            'fault': self._fault,
            'e_stop': self._e_stop,
            'hard_estop': self._hard_estop,
            'hard_estop_known': self._hard_estop_known,
            'hard_estop_latched': self._hard_estop_latched,
            'device_fault_latched': self._device_fault_latched,
            'command_ownership_confirmed': self._ownership_confirmed,
            'motion_state': status.motion_state if status else None,
            'gait': status.gait if status else None,
            'control_usage_mode': (
                status.control_usage_mode if status else None
            ),
            'firmware_version': status.version if status else '',
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
        self._connect(now)
        self._drain_socket(self._tcp, self._tcp_decoder)
        self._drain_socket(self._udp, self._udp_decoder)
        if self._tcp is not None and now - self._last_heartbeat >= 1.0:
            self._send_tcp_command(100, 100, {})
            self._last_heartbeat = now
        self._update_faults(now)
        self._advance_factory_state(now)
        new_ready = self._desired_ready(now)
        if new_ready != self._ready:
            self._ready = new_ready
            self.get_logger().info(
                f'factory motion backend ready={self._ready}'
            )
        self._send_periodic_velocity(now)
        self._publish_health()

    def stop(self) -> None:
        """Best-effort zero command before closing both transports."""
        self._ready = False
        for _ in range(3):
            self._send_velocity((0.0, 0.0, 0.0))
        self._close_sockets()
        self._publish_health(force=True)


def main(args=None) -> None:
    """Run the real M20 factory motion backend."""
    rclpy.init(args=args)
    node = BasicServerBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        # ``ok`` and ``shutdown`` are shared by Foxy and Humble.
        if rclpy.ok():
            rclpy.shutdown()
