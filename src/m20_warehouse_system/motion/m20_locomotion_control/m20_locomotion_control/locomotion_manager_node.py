# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：locomotion_manager_node.py
# 所属：m20_locomotion_control —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：ROS 节点——把"安全导航指令"适配到 M20 RL 控制器。
#   - 订阅安全 Twist（/m20/control/cmd_vel_safe），按能力参数约束后发布 SDK Twist
#     （/m20/locomotion/cmd_vel_sdk）；
#   - 依据后端就绪/故障、安全状态（NAVIGATION/MANUAL）、运动意图发布模式标签
#     （/m20/locomotion/mode）；
#   - 内置看门狗：指令超时后自动输出零指令（fail-safe）；
#   - 不在此层执行导航滚动适配；该职责唯一归 m20_navigation_adapter。
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

"""ROS node adapting safe navigation commands to the M20 RL controller."""
# 【中文注释】模块说明：把安全导航指令适配到 M20 RL 控制器的 ROS 节点。

from geometry_msgs.msg import Twist  # ROS2 速度指令消息（线速度 + 角速度）
import rclpy                         # ROS2 Python 客户端库
from rclpy.node import Node          # ROS2 节点基类
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS 策略
from std_msgs.msg import Bool, String  # 标准消息：布尔、字符串

from .motion_intent import (         # 导入运动意图策略模块
    IntentParameters,                #   意图参数
    MotionIntent,                    #   运动意图枚举
    RollingNavigationAdapter,        #   滚动导航适配器
    constrain_for_intent,            #   按意图约束指令
)


class LocomotionManager(Node):
    """Constrain safe Twist commands and expose their high-level motion intent."""
    # 【中文注释】运动管理器节点：约束安全 Twist 指令并发布高层运动意图。

    def __init__(self) -> None:
        # 【中文注释】构造函数：声明参数、构建适配器、创建发布/订阅/定时器。
        super().__init__('m20_locomotion_manager')
        self.declare_parameter(       # 输入话题：安全指令
            'input_topic', '/m20/control/cmd_vel_safe'
        )
        self.declare_parameter(       # 输出话题：SDK 指令
            'output_topic', '/m20/locomotion/cmd_vel_sdk'
        )
        self.declare_parameter(       # 模式话题：运动意图标签
            'mode_topic', '/m20/locomotion/mode'
        )
        self.declare_parameter(       # 后端就绪话题
            'backend_ready_topic', '/m20/sim/backend_ready'
        )
        self.declare_parameter(       # 后端故障话题
            'backend_fault_topic', '/m20/sim/backend_fault'
        )
        self.declare_parameter(       # 安全状态话题（NAVIGATION/MANUAL）
            'safety_state_topic', '/m20/control/safety_state'
        )
        self.declare_parameter('capability_profile_id', 'fallback_defaults')  # 能力配置 ID
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)  # 最小中心线转弯半径
        self.declare_parameter('turn_swept_radius', 0.0)               # 转弯扫掠半径
        self.declare_parameter('require_backend_ready', False)  # 是否要求后端就绪才放行
        self.declare_parameter('rolling_navigation_enabled', True)  # 是否启用滚动导航适配
        self.declare_parameter('allow_manual_lateral', True)    # 手动模式下是否允许横向
        self.declare_parameter('command_timeout_sec', 0.30)     # 指令超时（看门狗）
        self.declare_parameter('max_forward', 0.45)  # 最大前向速度
        self.declare_parameter('max_side', 0.20)     # 最大横向速度
        self.declare_parameter('max_yaw', 0.65)      # 最大偏航角速度
        self.declare_parameter('deadband_linear', 0.01)  # 线速度死区
        self.declare_parameter('deadband_yaw', 0.02)     # 偏航死区
        self.declare_parameter('lateral_threshold', 0.05)  # 横向意图阈值
        self.declare_parameter('turn_yaw_threshold', 0.25) # 转弯偏航阈值
        self.declare_parameter('in_place_linear_threshold', 0.08)  # 原地转向前向速度阈值
        self.declare_parameter('turn_curvature_threshold', 1.20)   # 转弯曲率阈值
        self.declare_parameter('curvature_speed_floor', 0.05)      # 曲率计算速度下限
        self.declare_parameter('turn_min_forward', 0.35)  # 转弯最低前向速度
        self.declare_parameter('turn_max_forward', 0.45)  # 转弯最高前向速度
        self.declare_parameter('lateral_max_forward', 0.10)  # 横移最大前向速度
        self.declare_parameter('suppress_side_in_cruise', True)  # 巡航抑制横向
        self.declare_parameter('suppress_side_in_turn', True)    # 转弯抑制横向
        self.declare_parameter('course_yaw_gain', 1.20)   # 航向误差增益
        self.declare_parameter('turn_course_enter', 0.25) # 进入转弯航向误差
        self.declare_parameter('turn_course_exit', 0.08)  # 退出转弯航向误差
        self.declare_parameter('turn_yaw_exit', 0.20)     # 退出转弯偏航阈值
        self.declare_parameter('turn_min_hold_sec', 0.50) # 转弯滞回最短保持
        self.declare_parameter('cruise_yaw_deadband', 0.02)  # 巡航偏航死区
        self.declare_parameter(
            'cruise_yaw_filter_time_constant',  # 巡航偏航滤波时间常数
            0.12,
        )
        self.declare_parameter('reverse_speed_offset', 0.19)  # 倒车速度补偿偏移
        self.declare_parameter('reverse_speed_gain', 1.32)    # 倒车速度补偿增益
        self.declare_parameter('reverse_yaw_offset', 0.15)    # 倒车偏航补偿偏移
        self.declare_parameter('reverse_yaw_gain', 1.00)      # 倒车偏航补偿增益
        self.declare_parameter('output_linear_accel', 1.0)    # 输出线加速度限幅
        self.declare_parameter('output_yaw_accel', 1.2)       # 输出偏航角加速度限幅

        # Do not shadow rclpy.Node._parameters, which owns declared ROS
        # parameters internally.
        # 【中文注释】不要遮蔽 rclpy.Node._parameters（rclpy 内部持有已声明的 ROS 参数）。
        self._intent_parameters = IntentParameters(  # 从 ROS 参数构建意图参数
            max_forward=float(self.get_parameter('max_forward').value),
            max_side=float(self.get_parameter('max_side').value),
            max_yaw=float(self.get_parameter('max_yaw').value),
            deadband_linear=float(
                self.get_parameter('deadband_linear').value
            ),
            deadband_yaw=float(self.get_parameter('deadband_yaw').value),
            lateral_threshold=float(
                self.get_parameter('lateral_threshold').value
            ),
            turn_yaw_threshold=float(
                self.get_parameter('turn_yaw_threshold').value
            ),
            in_place_linear_threshold=float(
                self.get_parameter('in_place_linear_threshold').value
            ),
            turn_curvature_threshold=float(
                self.get_parameter('turn_curvature_threshold').value
            ),
            curvature_speed_floor=float(
                self.get_parameter('curvature_speed_floor').value
            ),
            turn_min_forward=float(
                self.get_parameter('turn_min_forward').value
            ),
            turn_max_forward=float(
                self.get_parameter('turn_max_forward').value
            ),
            lateral_max_forward=float(
                self.get_parameter('lateral_max_forward').value
            ),
            suppress_side_in_cruise=bool(
                self.get_parameter('suppress_side_in_cruise').value
            ),
            suppress_side_in_turn=bool(
                self.get_parameter('suppress_side_in_turn').value
            ),
            course_yaw_gain=float(
                self.get_parameter('course_yaw_gain').value
            ),
            turn_course_enter=float(
                self.get_parameter('turn_course_enter').value
            ),
            turn_course_exit=float(
                self.get_parameter('turn_course_exit').value
            ),
            turn_yaw_exit=float(
                self.get_parameter('turn_yaw_exit').value
            ),
            turn_min_hold_sec=float(
                self.get_parameter('turn_min_hold_sec').value
            ),
            cruise_yaw_deadband=float(
                self.get_parameter('cruise_yaw_deadband').value
            ),
            cruise_yaw_filter_time_constant=float(
                self.get_parameter(
                    'cruise_yaw_filter_time_constant'
                ).value
            ),
            reverse_speed_offset=float(
                self.get_parameter('reverse_speed_offset').value
            ),
            reverse_speed_gain=float(
                self.get_parameter('reverse_speed_gain').value
            ),
            reverse_yaw_offset=float(
                self.get_parameter('reverse_yaw_offset').value
            ),
            reverse_yaw_gain=float(
                self.get_parameter('reverse_yaw_gain').value
            ),
            output_linear_accel=float(
                self.get_parameter('output_linear_accel').value
            ),
            output_yaw_accel=float(
                self.get_parameter('output_yaw_accel').value
            ),
        )
        self._rolling_adapter = RollingNavigationAdapter(  # 滚动导航适配器实例
            self._intent_parameters
        )
        # Deprecated compatibility parameter. Navigation adaptation is owned
        # exclusively by m20_navigation_adapter and is never repeated here.
        self._rolling_navigation_enabled = False
        self._allow_manual_lateral = bool(  # 手动模式是否允许横向
            self.get_parameter('allow_manual_lateral').value
        )
        self._timeout_sec = max(  # 指令超时时间（下限 0.05s）
            0.05,
            float(self.get_parameter('command_timeout_sec').value),
        )
        self._last_command_ns = 0        # 最近一次指令时间戳（ns）
        self._last_adaptation_ns = 0     # 最近一次适配更新时间戳（ns）
        self._last_intent = None         # 最近发布的意图（避免重复发布）
        self._timeout_zero_sent = False  # 超时零指令是否已发送
        self._require_backend_ready = bool(  # 是否要求后端就绪
            self.get_parameter('require_backend_ready').value
        )
        self._backend_ready = not self._require_backend_ready  # 后端就绪标志
        self._backend_fault = ''         # 后端故障文本
        self._safety_state = 'NAVIGATION'  # 安全状态（NAVIGATION/MANUAL）

        mode_qos = QoSProfile(  # 模式话题 QoS：RELIABLE + TRANSIENT_LOCAL（新订阅者能收到最新值）
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_publisher = self.create_publisher(  # SDK 指令发布器
            Twist,
            str(self.get_parameter('output_topic').value),
            20,
        )
        self._mode_publisher = self.create_publisher(  # 运动意图模式发布器
            String,
            str(self.get_parameter('mode_topic').value),
            mode_qos,
        )
        self.create_subscription(  # 订阅安全指令
            Twist,
            str(self.get_parameter('input_topic').value),
            self._command_callback,
            20,
        )
        self.create_subscription(  # 订阅后端就绪
            Bool,
            str(self.get_parameter('backend_ready_topic').value),
            self._backend_ready_callback,
            mode_qos,
        )
        self.create_subscription(  # 订阅后端故障
            String,
            str(self.get_parameter('backend_fault_topic').value),
            self._backend_fault_callback,
            mode_qos,
        )
        self.create_subscription(  # 订阅安全状态
            String,
            str(self.get_parameter('safety_state_topic').value),
            self._safety_state_callback,
            mode_qos,
        )
        self._watchdog = self.create_timer(0.05, self._check_timeout)  # 看门狗定时器
        initial_intent = (  # 初始意图：要求后端就绪时用 BACKEND_HOLD，否则 STOPPED
            MotionIntent.BACKEND_HOLD
            if self._require_backend_ready
            else MotionIntent.STOPPED
        )
        self._publish(initial_intent, (0.0, 0.0, 0.0))
        self.get_logger().info(
            'Locomotion manager ready: safe Twist -> SDK Twist; '
            'autonomous rolling adapter='
            f'{self._rolling_navigation_enabled}; motion mode is an intent '
            'label, not a discrete ONNX gait input; capability profile='
            f'{self.get_parameter("capability_profile_id").value}'
        )

    def _command_callback(self, message: Twist) -> None:
        # 【中文注释】安全指令回调：约束指令并发布（后端未就绪/故障时发布保持零指令）。
        if not self._backend_ready or self._backend_fault:
            self._publish_backend_hold()
            return
        command = (  # 提取平面速度指令 (vx, vy, yaw)
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        now_ns = self.get_clock().now().nanoseconds
        dt = (  # 时间步长：首帧用 0.02s，后续夹到 [0.001, 0.10]s
            0.02
            if self._last_adaptation_ns == 0
            else max(
                0.001,
                min(0.10, (now_ns - self._last_adaptation_ns) / 1e9),
            )
        )
        self._last_adaptation_ns = now_ns
        self._rolling_adapter.reset()
        intent, constrained = constrain_for_intent(
            command,
            self._intent_parameters,
        )
        self._last_command_ns = now_ns
        self._timeout_zero_sent = False
        self._publish(intent, constrained)

    def _backend_ready_callback(self, message: Bool) -> None:
        # 【中文注释】后端就绪回调：就绪状态变化时更新发布。
        was_ready = self._backend_ready
        self._backend_ready = bool(message.data)
        if not self._backend_ready:  # 后端不再就绪 → 保持零指令
            self._publish_backend_hold()
        elif not was_ready and not self._backend_fault:  # 首次就绪 → 停止并通知
            self._publish(MotionIntent.STOPPED, (0.0, 0.0, 0.0))
            self.get_logger().info('motion backend ready; commands enabled')

    def _backend_fault_callback(self, message: String) -> None:
        # 【中文注释】后端故障回调：故障出现时保持零指令，故障清除且就绪时恢复。
        previous = self._backend_fault
        self._backend_fault = str(message.data)
        if self._backend_fault:  # 出现故障
            self._publish_backend_hold()
            if self._backend_fault != previous:
                self.get_logger().error(
                    f'backend fault hold: {self._backend_fault}'
                )
        elif previous and self._backend_ready:  # 故障清除且后端就绪
            self._publish(MotionIntent.STOPPED, (0.0, 0.0, 0.0))
            self.get_logger().info('backend fault cleared; commands enabled')

    def _safety_state_callback(self, message: String) -> None:
        # 【中文注释】安全状态回调：非 NAVIGATION/MANUAL 时复位适配器状态。
        self._safety_state = str(message.data)
        if self._safety_state not in {'NAVIGATION', 'MANUAL'}:
            self._rolling_adapter.reset()
            self._last_adaptation_ns = 0

    def _publish_backend_hold(self) -> None:
        # 【中文注释】发布后端保持：复位适配器并发布零指令（故障时用 FAULT_HOLD）。
        self._rolling_adapter.reset()
        self._last_adaptation_ns = 0
        intent = (
            MotionIntent.FAULT_HOLD
            if self._backend_fault
            else MotionIntent.BACKEND_HOLD
        )
        self._publish(intent, (0.0, 0.0, 0.0))

    def _check_timeout(self) -> None:
        # 【中文注释】看门狗：安全指令超时后发布零指令（fail-safe）。
        if not self._backend_ready or self._backend_fault:
            return
        if self._last_command_ns == 0 or self._timeout_zero_sent:
            return
        age_sec = (  # 指令年龄（s）
            self.get_clock().now().nanoseconds - self._last_command_ns
        ) / 1e9
        if age_sec < self._timeout_sec:
            return
        self._rolling_adapter.reset()
        self._last_adaptation_ns = 0
        self._publish(MotionIntent.STOPPED, (0.0, 0.0, 0.0))
        self._timeout_zero_sent = True
        self.get_logger().warn(
            f'safe command timed out after {age_sec:.3f}s; SDK command zeroed'
        )

    def _publish(self, intent: MotionIntent, command) -> None:
        # 【中文注释】发布指令与意图：填充 Twist 消息，意图变化时发布模式标签。
        message = Twist()
        message.linear.x = command[0]
        message.linear.y = command[1]
        message.angular.z = command[2]
        self._command_publisher.publish(message)
        if intent != self._last_intent:  # 意图变化才发布模式（避免刷屏）
            self._mode_publisher.publish(String(data=intent.value))
            self.get_logger().info(f'motion intent -> {intent.value}')
            self._last_intent = intent


def main(args=None) -> None:
    """Run the locomotion manager."""
    # 【中文注释】节点入口：初始化 rclpy、构建节点并自旋；退出时发布零指令并清理。
    rclpy.init(args=args)
    node = LocomotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:  # 退出前发布零指令，确保机器人停止
                zero = Twist()
                node._command_publisher.publish(zero)
            except (Exception, KeyboardInterrupt):
                pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
