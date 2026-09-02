# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：basic_server_backend_node.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：使用官方 ``basic_server`` 协议的真实 M20 运动后端。
#   - 通过 TCP(30001)/UDP(30000) 与机器人 AOS basic_server 通信；
#   - 周期性发送速度指令（Type=2, Cmd=25）与心跳/状态指令，解码 BasicStatus(1002/6)、
#     MotionStatus(1002/4)、设备错误(1002/3)；
#   - 状态机推进：控制模式 → 站立(1) → RL 控制(17) → 目标步态；
#   - 安全锁存：软急停、硬急停(HES)、设备故障、所有权确认、指令超时看门狗；
#   - 发布后端就绪/故障/状态 JSON 与实测速度 TwistStamped。
# ============================================================================
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
# 【中文注释】模块说明：使用官方 basic_server 协议的真实 M20 运动后端。

import errno                         # 标准错误码（EAGAIN/EWOULDBLOCK 等）
import json                          # JSON 序列化（状态发布）
import socket                        # 套接字（TCP/UDP 通信）
from typing import Optional, Tuple   # 类型提示

from geometry_msgs.msg import Twist, TwistStamped  # 速度指令/带时间戳速度消息
import rclpy                         # ROS2 Python 客户端库
from rclpy.node import Node          # ROS2 节点基类
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS 策略
from std_msgs.msg import Bool, String  # 标准消息：布尔、字符串
from std_srvs.srv import SetBool     # 标准服务：设置布尔（启用/禁用运动）

from .basic_server_protocol import ( # 导入 basic_server 协议模块
    ApduStreamDecoder,               #   APDU 流解码器
    BasicStatus,                     #   基本状态数据类
    MotionStatus,                    #   运动状态数据类
    apply_vendor_velocity_envelope,  #   厂商速度区间应用
    encode_json_apdu,                #   JSON APDU 编码
    make_patrol_message,             #   Patrol 消息构造
    parse_basic_status,              #   解析基本状态
    parse_device_errors,             #   解析设备错误
    parse_error,                     #   解析指令应答错误
    parse_motion_status,             #   解析运动状态
)


class BasicServerBackend(Node):
    """Translate the common safe Twist stream to M20 factory motion control."""
    # 【中文注释】basic_server 后端节点：把公共安全 Twist 流翻译为 M20 工厂运动控制。

    def __init__(self) -> None:
        # 【中文注释】构造函数：声明参数、创建套接字与收发端。
        super().__init__('m20_basic_server_backend')
        self.declare_parameter('robot_host', '10.21.31.103')  # 机器人 IP
        self.declare_parameter('udp_port', 30000)  # UDP 端口（速度指令）
        self.declare_parameter('tcp_port', 30001)  # TCP 端口（状态/指令）
        self.declare_parameter(                    # 输入话题：SDK 指令
            'input_topic', '/m20/locomotion/cmd_vel_sdk'
        )
        self.declare_parameter(                    # 软急停话题
            'e_stop_topic', '/m20/control/e_stop'
        )
        self.declare_parameter(                    # 就绪话题
            'ready_topic', '/m20/locomotion/backend_ready'
        )
        self.declare_parameter(                    # 故障话题
            'fault_topic', '/m20/locomotion/backend_fault'
        )
        self.declare_parameter(                    # 状态话题
            'status_topic', '/m20/locomotion/backend_status'
        )
        self.declare_parameter(                    # 实测速度话题
            'measured_twist_topic',
            '/m20/locomotion/measured_twist',
        )
        self.declare_parameter(                    # 启用运动服务
            'enable_service', '/m20/hardware/enable_motion'
        )
        self.declare_parameter('command_rate_hz', 20.0)   # 指令频率
        self.declare_parameter('command_timeout_sec', 0.30)  # 指令超时
        self.declare_parameter('status_timeout_sec', 1.25)   # 状态超时
        self.declare_parameter('connect_retry_sec', 2.0)     # 重连间隔
        self.declare_parameter('requested_control_usage_mode', 1)  # 请求的控制模式
        self.declare_parameter('requested_gait', 0x3002)     # 请求的步态（敏捷平地步态）
        self.declare_parameter('auto_enable_motion', False)  # 是否自动启用运动
        self.declare_parameter('lie_down_on_disable', False) # 禁用时是否卧倒
        self.declare_parameter('command_ownership_confirmed', False)  # 指令所有权确认
        self.declare_parameter('subthreshold_policy', 'zero')  # 阈值以下策略

        self._host = str(self.get_parameter('robot_host').value)  # 机器人主机
        self._udp_port = int(self.get_parameter('udp_port').value)  # UDP 端口
        self._tcp_port = int(self.get_parameter('tcp_port').value)  # TCP 端口
        self._command_period = 1.0 / max(  # 指令发送周期
            20.0, float(self.get_parameter('command_rate_hz').value)
        )
        self._command_timeout = min(  # 指令超时（夹到 [0.05, 0.45]s）
            0.45,
            max(
                0.05,
                float(self.get_parameter('command_timeout_sec').value),
            ),
        )
        self._status_timeout = max(  # 状态超时（下限 0.50s）
            0.50, float(self.get_parameter('status_timeout_sec').value)
        )
        self._connect_retry = max(  # 重连间隔（下限 0.25s）
            0.25, float(self.get_parameter('connect_retry_sec').value)
        )
        self._requested_usage_mode = int(  # 请求的控制使用模式
            self.get_parameter('requested_control_usage_mode').value
        )
        self._requested_gait = int(  # 请求的步态
            self.get_parameter('requested_gait').value
        )
        self._lie_down_on_disable = bool(  # 禁用时是否卧倒
            self.get_parameter('lie_down_on_disable').value
        )
        self._ownership_confirmed = bool(  # 指令所有权是否确认
            self.get_parameter('command_ownership_confirmed').value
        )
        self._subthreshold_policy = str(  # 阈值以下策略（zero/passthrough）
            self.get_parameter('subthreshold_policy').value
        ).lower()

        self._tcp: Optional[socket.socket] = None  # TCP 套接字
        self._udp: Optional[socket.socket] = None  # UDP 套接字
        self._tcp_decoder = ApduStreamDecoder()  # TCP APDU 解码器
        self._udp_decoder = ApduStreamDecoder()  # UDP APDU 解码器
        self._frame_id = 0            # 报文帧号（自增）
        self._last_connect_attempt = -1.0e9  # 上次连接尝试时间
        self._connected_at = 0.0      # 连接建立时间
        self._last_heartbeat = -1.0e9 # 上次心跳时间
        self._last_state_command = -1.0e9  # 上次状态指令时间
        self._last_gait_command = -1.0e9   # 上次步态指令时间
        self._last_mode_command = -1.0e9   # 上次模式指令时间
        self._last_velocity_send = -1.0e9  # 上次速度发送时间
        self._last_command_time = -1.0e9   # 上次指令接收时间
        self._last_basic_status_time = -1.0e9  # 上次基本状态时间
        self._last_motion_status_time = -1.0e9 # 上次运动状态时间
        self._basic_status: Optional[BasicStatus] = None    # 最近基本状态
        self._motion_status: Optional[MotionStatus] = None  # 最近运动状态
        self._command = (0.0, 0.0, 0.0)  # 当前速度指令
        self._motion_requested = bool(    # 是否请求运动
            self.get_parameter('auto_enable_motion').value
        )
        self._e_stop = False              # 软急停
        self._hard_estop = False          # 硬急停状态
        self._hard_estop_known = False    # 硬急停是否已知
        self._hard_estop_latched = False  # 硬急停释放锁存
        self._device_fault_latched = False  # 设备故障锁存
        self._device_fault_text = ''        # 设备故障文本
        self._command_fault = ''            # 指令应答故障
        self._soft_estop_sent = False       # 软急停是否已发送
        self._ready = False                 # 后端就绪
        self._fault = ''                    # 当前故障
        self._last_status_text = ''         # 上次状态文本（去重）
        self._last_suppressed_axes: Tuple[str, ...] = ()  # 上次被抑制的轴

        latched_qos = QoSProfile(  # 锁存 QoS：TRANSIENT_LOCAL
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_publisher = self.create_publisher(  # 就绪发布器
            Bool, str(self.get_parameter('ready_topic').value), latched_qos
        )
        self._fault_publisher = self.create_publisher(  # 故障发布器
            String, str(self.get_parameter('fault_topic').value), latched_qos
        )
        self._status_publisher = self.create_publisher(  # 状态发布器
            String,
            str(self.get_parameter('status_topic').value),
            latched_qos,
        )
        self._twist_publisher = self.create_publisher(  # 实测速度发布器
            TwistStamped,
            str(self.get_parameter('measured_twist_topic').value),
            20,
        )
        self.create_subscription(  # 订阅 SDK 指令
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        self.create_subscription(  # 订阅软急停
            Bool,
            str(self.get_parameter('e_stop_topic').value),
            self._e_stop_callback,
            latched_qos,
        )
        self.create_service(  # 启用运动服务
            SetBool,
            str(self.get_parameter('enable_service').value),
            self._enable_callback,
        )
        self._timer = self.create_timer(0.02, self._update)  # 50Hz 主循环
        self._publish_health(force=True)
        self.get_logger().info(
            'Official M20 basic_server backend configured for '
            f'{self._host}:{self._udp_port}/{self._tcp_port}; motion starts '
            'disabled unless explicitly enabled'
        )

    def _now(self) -> float:
        # 【中文注释】当前 ROS 时间（秒）。
        return self.get_clock().now().nanoseconds / 1e9

    def _command_callback(self, message: Twist) -> None:
        # 【中文注释】SDK 指令回调：保存指令与接收时间。
        self._command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self._last_command_time = self._now()

    def _e_stop_callback(self, message: Bool) -> None:
        # 【中文注释】软急停回调：触发时清空指令、发零速度与软急停指令（MotionParam=2）。
        asserted = bool(message.data)
        if asserted and not self._e_stop:
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            self._send_velocity((0.0, 0.0, 0.0))
            self._send_tcp_command(2, 22, {'MotionParam': 2})  # 软急停指令
            self._soft_estop_sent = True
            self.get_logger().error(
                'e-stop asserted: zero velocity and M20 soft e-stop sent'
            )
        self._e_stop = asserted
        self._publish_health(force=True)

    def _enable_callback(self, request, response):
        # 【中文注释】启用/禁用运动服务回调：执行完整的前置安全检查（fail-closed）。
        enable = bool(request.data)
        if enable and self._e_stop:  # 急停未释放
            response.success = False
            response.message = 'release /m20/control/e_stop first'
            return response
        status_fresh = bool(  # 基本状态是否新鲜
            self._basic_status is not None
            and self._now() - self._last_basic_status_time
            <= self._status_timeout
        )
        if enable and (not status_fresh or not self._hard_estop_known):  # 无新鲜 HES 状态
            response.success = False
            response.message = (
                'waiting for a fresh BasicStatus with HES before motion enable'
            )
            return response
        if enable and self._hard_estop:  # 硬急停触发
            response.success = False
            response.message = 'physical M20 hard e-stop is asserted'
            return response
        if enable and self._hard_estop_latched:  # 硬急停释放锁存未复位
            response.success = False
            response.message = (
                'hard e-stop release is latched; call enable_motion false '
                'once, inspect the robot, then request true'
            )
            return response
        if enable and self._device_fault_latched:  # 设备故障锁存
            response.success = False
            response.message = (
                'a factory device fault is latched; resolve the vendor '
                'error, restart this backend, and repeat read-only checks'
            )
            return response
        if enable and not self._ownership_confirmed:  # 指令所有权未确认
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
        if not enable:  # 禁用：发送零速度，可选卧倒，复位锁存
            self._send_velocity((0.0, 0.0, 0.0))
            if (  # 可选：RL 控制状态且静止时发送卧倒指令（MotionParam=4）
                self._lie_down_on_disable
                and self._basic_status is not None
                and self._basic_status.motion_state == 17
                and self._is_stationary()
            ):
                self._send_tcp_command(2, 22, {'MotionParam': 4})
            if status_fresh and not self._hard_estop:
                self._hard_estop_latched = False  # 硬急停锁存复位（需先 false 后 true）
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
        # 【中文注释】连接机器人的 TCP 与 UDP 通道（带重连节流）。
        if (  # 已连接或未到重连间隔则跳过
            self._tcp is not None
            or now - self._last_connect_attempt < self._connect_retry
        ):
            return
        self._last_connect_attempt = now
        tcp = None
        udp = None
        try:
            tcp = socket.create_connection(  # TCP 连接（0.1s 超时）
                (self._host, self._tcp_port), timeout=0.10
            )
            tcp.setblocking(False)  # 非阻塞
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP 套接字
            udp.setblocking(False)
            udp.connect((self._host, self._udp_port))
        except OSError as exc:  # 连接失败：清理并设置故障
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
        self._tcp_decoder = ApduStreamDecoder()  # 重置解码器
        self._udp_decoder = ApduStreamDecoder()
        self._connected_at = now
        self._last_heartbeat = -1.0e9
        self._basic_status = None
        self._motion_status = None
        self._clear_fault()
        self.get_logger().info('connected to M20 basic_server')

    def _close_sockets(self) -> None:
        # 【中文注释】关闭 TCP/UDP 套接字并复位就绪标志。
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
        # 【中文注释】自增帧号（16 位回绕）。
        self._frame_id = (self._frame_id + 1) & 0xFFFF
        return self._frame_id

    def _packet(self, message_type: int, command: int, items) -> bytes:
        # 【中文注释】构造 Patrol 消息并编码为 APDU 字节串。
        message = make_patrol_message(message_type, command, items)
        return encode_json_apdu(message, self._next_frame_id())

    def _send_tcp_command(self, message_type: int, command: int, items) -> bool:
        # 【中文注释】通过 TCP 发送指令；失败时触发连接故障处理。
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
        # 【中文注释】通过 UDP 发送速度指令（Type=2, Cmd=25，XYZ/RPY 六自由度指令）。
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
        # 【中文注释】连接失败：关闭套接字并设置 IO 故障。
        self._close_sockets()
        self._set_fault(f'BASIC_SERVER_IO:{exc.errno or "IO"}')

    def _drain_socket(self, channel, decoder) -> None:
        # 【中文注释】非阻塞读取套接字数据并送入解码器，解出的消息逐条处理。
        if channel is None:
            return
        while True:
            try:
                chunk = channel.recv(65535)
            except BlockingIOError:
                return  # 无更多数据
            except OSError as exc:
                if exc.errno in {errno.EAGAIN, errno.EWOULDBLOCK}:
                    return  # 暂时无数据
                self._connection_failed(exc)
                return
            if not chunk:  # 对端关闭
                self._connection_failed(OSError(errno.ECONNRESET, 'closed'))
                return
            for _, _, message in decoder.feed(chunk):  # 逐条处理消息
                self._handle_message(message)

    def _handle_message(self, message) -> None:
        # 【中文注释】处理一条解码后的消息：状态解析 + 锁存 + 实测速度发布。
        now = self._now()
        basic = parse_basic_status(message)  # 解析基本状态
        if basic is not None:
            previous_hard_estop = self._hard_estop
            self._basic_status = basic
            self._last_basic_status_time = now
            self._hard_estop_known = basic.hard_estop in {0, 1}  # HES 可知
            self._hard_estop = basic.hard_estop == 1
            if self._hard_estop:  # 硬急停触发：锁存、停运动、连发零速度
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
            elif previous_hard_estop:  # 硬急停释放：仍需操作员确认
                self.get_logger().warn(
                    'physical M20 hard e-stop released; robot remains '
                    'disabled until enable_motion false then true'
                )
        motion = parse_motion_status(message)  # 解析运动状态 → 发布实测速度
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
        error = parse_error(message)  # 指令应答错误 → 命令故障
        if error is not None and error[0] != 0:
            self._command_fault = (
                f'BASIC_SERVER_ERROR_{error[0]}:{error[1]}'
            )
        device_errors = parse_device_errors(message)  # 设备错误 → 锁存停运动
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
        # 【中文注释】是否静止（各速度分量低于阈值）。
        motion = self._motion_status
        if motion is None:
            return False
        return (
            abs(motion.linear_x) < 0.03
            and abs(motion.linear_y) < 0.03
            and abs(motion.omega_z) < 0.05
        )

    def _advance_factory_state(self, now: float) -> None:
        # 【中文注释】推进工厂状态机：模式 → 站立 → RL 控制 → 步态（带节流）。
        status = self._basic_status
        if (  # 任何保护条件触发则不推进
            not self._motion_requested
            or status is None
            or self._e_stop
            or self._hard_estop
            or self._hard_estop_latched
            or self._device_fault_latched
            or self._fault
        ):
            return
        if not self._ownership_confirmed:  # 所有权未确认 → 故障
            self._set_fault('COMMAND_OWNERSHIP_NOT_CONFIRMED')
            return
        if status.motion_state == 2:  # 软急停锁存
            self._command_fault = 'M20_SOFT_ESTOP_LATCHED'
            self._motion_requested = False
            return
        if status.control_usage_mode != self._requested_usage_mode:  # 先切控制模式
            if now - self._last_mode_command >= 1.0:
                self._send_tcp_command(
                    1101, 5, {'Mode': self._requested_usage_mode}
                )
                self._last_mode_command = now
            return
        if status.motion_state != 17:  # 尚未进入 RL 控制
            if (
                status.motion_state in {0, 3, 4}  # 空闲/开机阻尼/卧倒
                and now - self._last_state_command >= 1.0
            ):
                # basic_server automatically advances stand (1) to RL (17).
                # 【中文注释】basic_server 会自动把站立(1)推进到 RL 控制(17)。
                self._send_tcp_command(2, 22, {'MotionParam': 1})  # 站立指令
                self._last_state_command = now
            return
        if status.gait != self._requested_gait:  # 步态不符：静止时切换步态
            if self._is_stationary() and now - self._last_gait_command >= 1.0:
                self._send_tcp_command(
                    2, 23, {'GaitParam': self._requested_gait}
                )
                self._last_gait_command = now

    def _desired_ready(self, now: float) -> bool:
        # 【中文注释】计算期望就绪状态：所有条件同时满足才算就绪（fail-closed）。
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
        # 【中文注释】故障检测：按优先级设置当前故障（未连接/状态超时/HES/设备错误等）。
        if self._tcp is None:
            self._set_fault('BASIC_SERVER_DISCONNECTED')
            return
        if (  # 连接建立后状态超时
            now - self._connected_at > self._status_timeout
            and now - self._last_basic_status_time > self._status_timeout
        ):
            self._set_fault('BASIC_STATUS_TIMEOUT')
            return
        if not self._hard_estop_known:  # HES 状态不可知
            self._set_fault('HARD_ESTOP_STATUS_UNAVAILABLE')
            return
        if self._hard_estop:  # 硬急停触发
            self._set_fault('M20_HARD_ESTOP_ASSERTED')
            return
        if self._hard_estop_latched:  # 硬急停释放锁存
            self._set_fault('M20_HARD_ESTOP_RELEASE_LATCHED')
            return
        if self._device_fault_latched:  # 设备故障锁存
            self._set_fault(self._device_fault_text or 'M20_DEVICE_ERROR')
            return
        if self._command_fault:  # 指令应答错误
            self._set_fault(self._command_fault)
            return
        self._clear_fault()

    def _send_periodic_velocity(self, now: float) -> None:
        # 【中文注释】周期性发送速度指令（20Hz），应用厂商速度区间；就绪且指令新鲜才发指令。
        if now - self._last_velocity_send < self._command_period:
            return
        self._last_velocity_send = now
        command = (0.0, 0.0, 0.0)
        fresh = now - self._last_command_time <= self._command_timeout
        if self._ready and fresh:
            command, suppressed = apply_vendor_velocity_envelope(  # 应用官方速度区间
                self._command,
                self._requested_gait,
                self._subthreshold_policy,
            )
            if suppressed != self._last_suppressed_axes:  # 抑制轴变化时告警一次
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
        # 【中文注释】启用序列进行中时，零帧保持 UDP 客户端身份与厂商 500ms 看门狗确定性。
        if self._motion_requested or self._ready:
            self._send_velocity(command)

    def _set_fault(self, text: str) -> None:
        # 【中文注释】设置故障（去重）并发布健康状态。
        if text == self._fault:
            return
        self._fault = text
        self._ready = False
        self._publish_health(force=True)

    def _clear_fault(self) -> None:
        # 【中文注释】清除故障并发布健康状态。
        if not self._fault:
            return
        self._fault = ''
        self._publish_health(force=True)

    def _publish_health(self, force: bool = False) -> None:
        # 【中文注释】发布后端健康状态（就绪/故障/状态 JSON），内容变化时才发布。
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
        # 【中文注释】50Hz 主循环：连接/读取/心跳/故障/状态机/就绪/速度/健康发布。
        now = self._now()
        self._connect(now)
        self._drain_socket(self._tcp, self._tcp_decoder)  # 读取 TCP 数据
        self._drain_socket(self._udp, self._udp_decoder)  # 读取 UDP 数据
        if self._tcp is not None and now - self._last_heartbeat >= 1.0:  # 心跳
            self._send_tcp_command(100, 100, {})
            self._last_heartbeat = now
        self._update_faults(now)
        self._advance_factory_state(now)
        new_ready = self._desired_ready(now)  # 就绪状态变化时记录日志
        if new_ready != self._ready:
            self._ready = new_ready
            self.get_logger().info(
                f'factory motion backend ready={self._ready}'
            )
        self._send_periodic_velocity(now)
        self._publish_health()

    def stop(self) -> None:
        """Best-effort zero command before closing both transports."""
        # 【中文注释】停止：尽力发送零速度后关闭两条传输通道。
        self._ready = False
        for _ in range(3):
            self._send_velocity((0.0, 0.0, 0.0))
        self._close_sockets()
        self._publish_health(force=True)


def main(args=None) -> None:
    """Run the real M20 factory motion backend."""
    # 【中文注释】节点入口：初始化、自旋；退出时发送零指令并清理。
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
        # 【中文注释】ok 与 shutdown 在 Foxy 与 Humble 中通用。
        if rclpy.ok():
            rclpy.shutdown()
