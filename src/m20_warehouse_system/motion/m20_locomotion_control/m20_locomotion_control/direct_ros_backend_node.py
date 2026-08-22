# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：direct_ros_backend_node.py
# 所属：m20_locomotion_control —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：使用官方 DrDDS ROS 2 运动话题的真实 M20 后端。
#   - 把公共安全 Twist 翻译为 /NAV_CMD（NavCmd）DrDDS 消息；
#   - 订阅 /MOTION_INFO（MotionInfo）解码运动状态/步态/实测速度，发布 /MOTION_STATE、
#     /GAIT 状态指令与实测 TwistStamped；
#   - 订阅 /HES_STATUS（StdMsgInt32）锁存物理硬急停；
#   - 主循环按 select_direct_transition 决策推进状态机（站立→RL 控制→步态），
#     并周期性发布速度指令（带厂商速度区间应用）；
#   - 提供启用服务（前置安全检查：急停/HES/所有权确认），
#     SIGINT/SIGTERM 优雅停止（先发零指令再销毁节点）。
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

"""Real M20 backend using the official DrDDS ROS 2 motion topics."""
# 【中文注释】模块说明：使用官方 DrDDS ROS 2 运动话题的真实 M20 后端。

from __future__ import annotations  # 延迟求值类型注解

import json                         # JSON 序列化（状态发布）
import signal                       # 信号处理（SIGINT/SIGTERM 优雅停止）
from typing import Optional, Tuple  # 类型提示

from drdds.msg import Gait, MotionInfo, MotionState, NavCmd, StdMsgInt32  # 厂商 DrDDS 消息
from geometry_msgs.msg import Twist, TwistStamped  # 速度指令/带时间戳速度消息
import rclpy                         # ROS2 Python 客户端库
from rclpy.node import Node          # ROS2 节点基类
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS 策略
from std_msgs.msg import Bool, String  # 标准消息：布尔、字符串
from std_srvs.srv import SetBool     # 标准服务：设置布尔（启用/禁用运动）

from .basic_server_protocol import apply_vendor_velocity_envelope  # 厂商速度区间应用
from .direct_ros_policy import (     # 导入直接 ROS 转移策略模块
    decode_motion_info,              #   解码 MotionInfo
    DirectMotionStatus,              #   运动状态数据类
    SOFT_ESTOP,                      #   软急停状态常量
    select_direct_transition,        #   选择状态机转移
)


class DirectRosBackend(Node):
    """Translate the common safe Twist into ``/NAV_CMD`` DrDDS messages."""
    # 【中文注释】直接 ROS 后端节点：把公共安全 Twist 翻译为 /NAV_CMD DrDDS 消息。

    def __init__(self) -> None:
        # 【中文注释】构造函数：声明参数、创建发布/订阅/服务。
        super().__init__('m20_direct_ros_backend')
        self.declare_parameter('input_topic', '/m20/locomotion/cmd_vel_sdk')  # SDK 指令话题
        self.declare_parameter('e_stop_topic', '/m20/control/e_stop')        # 软急停话题
        self.declare_parameter('ready_topic', '/m20/locomotion/backend_ready')  # 就绪话题
        self.declare_parameter('fault_topic', '/m20/locomotion/backend_fault')  # 故障话题
        self.declare_parameter(       # 状态话题
            'status_topic', '/m20/locomotion/backend_status'
        )
        self.declare_parameter(       # 实测速度话题
            'measured_twist_topic', '/m20/locomotion/measured_twist'
        )
        self.declare_parameter(       # 启用运动服务
            'enable_service', '/m20/hardware/enable_motion'
        )
        self.declare_parameter('nav_cmd_topic', '/NAV_CMD')        # 速度指令话题
        self.declare_parameter('motion_info_topic', '/MOTION_INFO')  # 运动信息话题
        self.declare_parameter('motion_state_topic', '/MOTION_STATE')  # 运动状态话题
        self.declare_parameter('gait_topic', '/GAIT')              # 步态话题
        self.declare_parameter('hard_estop_topic', '/HES_STATUS')  # 硬急停话题
        self.declare_parameter('command_rate_hz', 20.0)            # 指令频率
        self.declare_parameter('command_timeout_sec', 0.30)        # 指令超时
        self.declare_parameter('status_timeout_sec', 1.25)         # 状态超时
        self.declare_parameter('hard_estop_timeout_sec', 2.50)     # 硬急停状态超时
        self.declare_parameter('state_command_period_sec', 1.0)    # 状态指令周期
        self.declare_parameter('requested_gait', 0x3002)           # 请求步态（敏捷平地步态）
        self.declare_parameter('auto_enable_motion', False)        # 是否自动启用运动
        self.declare_parameter('lie_down_on_disable', False)       # 禁用时是否卧倒
        self.declare_parameter('command_ownership_confirmed', False)  # 指令所有权确认
        self.declare_parameter('subthreshold_policy', 'zero')      # 阈值以下策略

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
        self._hard_estop_timeout = max(  # 硬急停状态超时（下限 1.25s，HES 文档频率 1Hz）
            1.25,
            float(self.get_parameter('hard_estop_timeout_sec').value),
        )
        self._state_command_period = max(  # 状态指令周期（下限 0.25s）
            0.25,
            float(self.get_parameter('state_command_period_sec').value),
        )
        self._requested_gait = int(  # 请求的步态
            self.get_parameter('requested_gait').value
        )
        self._ownership_confirmed = bool(  # 指令所有权是否确认
            self.get_parameter('command_ownership_confirmed').value
        )
        self._lie_down_on_disable = bool(  # 禁用时是否卧倒
            self.get_parameter('lie_down_on_disable').value
        )
        self._subthreshold_policy = str(  # 阈值以下策略
            self.get_parameter('subthreshold_policy').value
        ).lower()

        self._frame_id = 0               # 报文帧号（自增）
        self._command = (0.0, 0.0, 0.0)  # 当前速度指令
        self._last_command_time = -1.0e9 # 上次指令接收时间
        self._last_velocity_send = -1.0e9  # 上次速度发送时间
        self._last_state_send = -1.0e9   # 上次状态指令发送时间
        self._last_gait_send = -1.0e9    # 上次步态指令发送时间
        self._last_info_time = -1.0e9    # 上次运动信息时间
        self._last_hard_estop_time = -1.0e9  # 上次硬急停状态时间
        self._enable_requested_at = -1.0e9   # 请求启用时间
        self._status: Optional[DirectMotionStatus] = None  # 最近运动状态
        self._motion_requested = bool(   # 是否请求运动
            self.get_parameter('auto_enable_motion').value
        )
        self._e_stop = False             # 软急停
        self._hard_estop = False         # 硬急停状态
        self._hard_estop_known = False   # 硬急停是否已知
        self._hard_estop_latched = False # 硬急停释放锁存
        self._ready = False              # 后端就绪
        self._fault = ''                 # 当前故障
        self._last_status_text = ''      # 上次状态文本（去重）
        self._last_suppressed_axes: Tuple[str, ...] = ()  # 上次被抑制的轴

        command_qos = QoSProfile(  # 指令 QoS：RELIABLE + VOLATILE（匹配记录端点）
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        feedback_qos = QoSProfile(  # 反馈 QoS：RELIABLE + VOLATILE
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(  # 锁存 QoS：TRANSIENT_LOCAL（HES/健康状态）
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._nav_publisher = self.create_publisher(  # 速度指令发布器
            NavCmd, str(self.get_parameter('nav_cmd_topic').value), command_qos
        )
        self._state_publisher = self.create_publisher(  # 运动状态发布器
            MotionState,
            str(self.get_parameter('motion_state_topic').value),
            command_qos,
        )
        self._gait_publisher = self.create_publisher(  # 步态发布器
            Gait, str(self.get_parameter('gait_topic').value), command_qos
        )
        self._ready_publisher = self.create_publisher(  # 就绪发布器（锁存）
            Bool, str(self.get_parameter('ready_topic').value), latched_qos
        )
        self._fault_publisher = self.create_publisher(  # 故障发布器（锁存）
            String, str(self.get_parameter('fault_topic').value), latched_qos
        )
        self._status_publisher = self.create_publisher(  # 状态发布器（锁存）
            String, str(self.get_parameter('status_topic').value), latched_qos
        )
        self._twist_publisher = self.create_publisher(  # 实测速度发布器
            TwistStamped,
            str(self.get_parameter('measured_twist_topic').value),
            20,
        )
        self.create_subscription(  # 订阅运动信息
            MotionInfo,
            str(self.get_parameter('motion_info_topic').value),
            self._motion_info_callback,
            feedback_qos,
        )
        self.create_subscription(  # 订阅硬急停状态
            StdMsgInt32,
            str(self.get_parameter('hard_estop_topic').value),
            self._hard_estop_callback,
            latched_qos,
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
        if self._motion_requested:
            self._enable_requested_at = self._now()
        self._publish_health(force=True)
        self.get_logger().info(
            'Official M20 direct ROS backend configured; motion starts '
            'disabled unless explicitly enabled'
        )

    def _now(self) -> float:
        # 【中文注释】当前 ROS 时间（秒）。
        return self.get_clock().now().nanoseconds / 1e9

    def _fill_header(self, message) -> None:
        # 【中文注释】填充消息头：自增帧号 + 当前时间戳。
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFFFFFFFFFF
        stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.header.stamp = stamp

    def _command_callback(self, message: Twist) -> None:
        # 【中文注释】SDK 指令回调：保存指令与接收时间。
        self._command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self._last_command_time = self._now()

    def _motion_info_callback(self, message: MotionInfo) -> None:
        # 【中文注释】运动信息回调：解码状态并发布实测速度。
        self._status = decode_motion_info(message.data)
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
        # 【中文注释】锁存机尾物理急停状态，直到操作员复位。
        asserted = bool(message.value)
        self._hard_estop_known = True
        self._last_hard_estop_time = self._now()
        if asserted:  # 硬急停触发：锁存、停运动、连发零速度
            self._hard_estop_latched = True
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            for _ in range(3):
                self._publish_velocity((0.0, 0.0, 0.0))
            if not self._hard_estop:
                self.get_logger().error(
                    'physical M20 hard e-stop asserted; manual reset required'
                )
        elif self._hard_estop:  # 硬急停释放：仍需操作员确认
            self.get_logger().warn(
                'physical M20 hard e-stop released; robot remains disabled '
                'until enable_motion false then true'
            )
        self._hard_estop = asserted
        self._publish_health(force=True)

    def _e_stop_callback(self, message: Bool) -> None:
        # 【中文注释】软急停回调：触发时清空指令、连发零速度并发布软急停状态。
        asserted = bool(message.data)
        if asserted and not self._e_stop:
            self._motion_requested = False
            self._command = (0.0, 0.0, 0.0)
            for _ in range(3):
                self._publish_velocity((0.0, 0.0, 0.0))
            self._publish_state(SOFT_ESTOP)  # 发布软急停状态指令
            self.get_logger().error(
                'e-stop asserted: zero /NAV_CMD and soft e-stop published'
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
        hard_estop_fresh = bool(  # 硬急停状态是否新鲜
            self._hard_estop_known
            and self._now() - self._last_hard_estop_time
            <= self._hard_estop_timeout
        )
        if enable and not hard_estop_fresh:  # 无新鲜 HES 状态
            response.success = False
            response.message = (
                'waiting for fresh /HES_STATUS before motion enable'
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
        if enable and not self._ownership_confirmed:  # 指令所有权未确认
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
        if not enable:  # 禁用：连发零速度，可选卧倒，复位锁存
            for _ in range(3):
                self._publish_velocity((0.0, 0.0, 0.0))
            if (  # 可选：RL 控制状态且静止时发布卧倒状态指令(4)
                self._lie_down_on_disable
                and self._status is not None
                and self._status.state == 17
                and self._status.stationary
            ):
                self._publish_state(4)
            hard_estop_fresh = bool(  # 重新检查 HES 新鲜度
                self._hard_estop_known
                and self._now() - self._last_hard_estop_time
                <= self._hard_estop_timeout
            )
            if hard_estop_fresh and not self._hard_estop:
                self._hard_estop_latched = False  # 硬急停锁存复位
        response.success = True
        response.message = (
            'direct ROS motion enable sequence requested'
            if enable
            else 'motion disabled; zero /NAV_CMD published'
        )
        self._publish_health(force=True)
        return response

    def _publish_state(self, value: int) -> None:
        # 【中文注释】发布运动状态指令（MotionState 消息）。
        message = MotionState()
        self._fill_header(message)
        message.data.state = int(value)
        self._state_publisher.publish(message)

    def _publish_gait(self, value: int) -> None:
        # 【中文注释】发布步态指令（Gait 消息）。
        message = Gait()
        self._fill_header(message)
        message.data.gait = int(value)
        self._gait_publisher.publish(message)

    def _publish_velocity(self, command) -> None:
        # 【中文注释】发布速度指令（NavCmd 消息，x/y/yaw 三轴）。
        message = NavCmd()
        self._fill_header(message)
        message.data.x_vel = float(command[0])
        message.data.y_vel = float(command[1])
        message.data.yaw_vel = float(command[2])
        self._nav_publisher.publish(message)

    def _publish_periodic_velocity(self, now: float) -> None:
        # 【中文注释】周期性发布速度指令（20Hz），应用厂商速度区间；就绪且指令新鲜才发指令。
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
        if self._motion_requested or self._ready:
            self._publish_velocity(command)

    def _publish_health(self, force: bool = False) -> None:
        # 【中文注释】发布后端健康状态（就绪/故障/状态 JSON），内容变化时才发布。
        status = self._status
        age = (  # 运动信息年龄
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
        # 【中文注释】50Hz 主循环：状态新鲜度检查 → 转移决策 → 状态/步态/速度发布。
        now = self._now()
        fresh_status = self._status
        if now - self._last_info_time > self._status_timeout:  # 状态过期
            fresh_status = None
        hard_estop_fresh = bool(  # 硬急停状态是否新鲜
            self._hard_estop_known
            and now - self._last_hard_estop_time <= self._hard_estop_timeout
        )
        decision = select_direct_transition(  # 状态机转移决策
            motion_requested=self._motion_requested,
            ownership_confirmed=self._ownership_confirmed,
            e_stop=self._e_stop,
            status=fresh_status,
            requested_gait=self._requested_gait,
        )
        fault = decision.fault
        if not hard_estop_fresh:  # HES 状态过期 → 故障
            fault = 'HARD_ESTOP_STATUS_TIMEOUT'
        elif self._hard_estop:  # 硬急停触发
            fault = 'M20_HARD_ESTOP_ASSERTED'
        elif self._hard_estop_latched:  # 硬急停释放锁存
            fault = 'M20_HARD_ESTOP_RELEASE_LATCHED'
        elif (  # 请求运动但状态超时
            self._motion_requested
            and fresh_status is None
            and now - self._enable_requested_at > self._status_timeout
        ):
            fault = 'MOTION_INFO_TIMEOUT'
        self._fault = fault
        self._ready = decision.ready and not fault  # 就绪 = 决策就绪且无故障
        if (  # 有状态指令待发且到周期
            not fault
            and decision.state_command is not None
            and now - self._last_state_send >= self._state_command_period
        ):
            self._publish_state(decision.state_command)
            self._last_state_send = now
        if (  # 有步态指令待发且到周期
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
        # 【中文注释】停止：在 ROS 拆除前发布冗余零速度指令。
        self._ready = False
        for _ in range(3):
            self._publish_velocity((0.0, 0.0, 0.0))
        self._publish_health(force=True)


def main(args=None) -> None:
    """Run the M20 direct ROS factory motion backend."""
    # 【中文注释】节点入口：注册信号处理器，确保零指令发布后再关闭。
    rclpy.init(args=args)
    node = DirectRosBackend()
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        """Defer shutdown until zero commands have been published."""
        # 【中文注释】延迟关闭：等零指令发布完成后再退出。
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)  # 注册 SIGINT 处理
    signal.signal(signal.SIGTERM, request_stop)  # 注册 SIGTERM 处理
    try:
        while rclpy.ok() and not stop_requested:  # 手动自旋循环
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()  # 发布零指令
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
