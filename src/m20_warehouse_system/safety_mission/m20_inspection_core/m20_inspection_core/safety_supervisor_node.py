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

# ============================================================================
# 【文件职责】safety_supervisor_node.py —— 速度安全监督节点（ROS2 Node）
# 本节点是"安全门"（fail-closed velocity supervisor）：
#   1) 订阅导航候选指令 / 手动指令 / 急停 / 楼层切换保持 / 任务保持 /
#      碰撞停止 / 碰撞恢复 / 地图就绪 / 里程计等话题；
#   2) 对指令做限幅（clamp）、加速度限制（slew）与比例斜坡（碰撞恢复）；
#   3) 按优先级选择未超时的指令源（手动优先或导航优先）；
#   4) 任何安全门条件（急停/保持/地图未就绪/里程计超时/碰撞停止）触发
#      时立即输出零速度指令（fail-closed），并在状态变化时发布执行保持；
#   5) 最终把安全指令发布到 /m20/control/cmd_vel_safe 供后端执行。
# ============================================================================

"""Fail-closed velocity supervisor and command multiplexer."""

# ------------------------- 第三方/标准库导入 -------------------------
from typing import Optional  # 类型提示：Optional 可空值

from geometry_msgs.msg import Twist   # Twist 消息：线速度/角速度指令
from nav_msgs.msg import Odometry     # Odometry 消息：里程计（判断新鲜度）
import rclpy                          # ROS2 Python 客户端库
from rclpy.node import Node           # ROS2 节点基类
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS 策略（锁存发布）
from std_msgs.msg import Bool, String # 标准消息：Bool 门控标志 / String 状态

from .safety_policy import (  # 导入安全策略纯函数（限幅/斜坡/选择/校验）
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
    # 【中文】安全监督节点：在每条指令到达仿真/真实后端之前进行门控

    def __init__(self) -> None:
        super().__init__('m20_safety_supervisor')
        # ------------------------- 参数声明（话题名/阈值） -------------------------
        self.declare_parameter(
            'navigation_topic', '/m20/navigation/cmd_vel_candidate'  # 导航候选指令话题
        )
        self.declare_parameter(
            'manual_topic', '/m20/control/cmd_vel_manual'            # 手动指令话题
        )
        self.declare_parameter('safe_topic', '/m20/control/cmd_vel_safe')  # 安全指令输出话题
        self.declare_parameter('e_stop_topic', '/m20/control/e_stop')      # 急停话题
        self.declare_parameter(
            'floor_hold_topic', '/m20/control/floor_switch_hold'           # 楼层切换保持话题
        )
        self.declare_parameter(
            'mission_hold_topic', '/m20/control/mission_hold'              # 任务保持话题
        )
        self.declare_parameter(
            'collision_stop_topic', '/m20/control/collision_stop'          # 碰撞停止话题
        )
        self.declare_parameter('collision_guard_enabled', True)   # 是否启用碰撞守卫门控
        self.declare_parameter(
            'collision_recovery_available_topic',                          # 碰撞恢复可用标志话题
            '/m20/control/collision_recovery_available',
        )
        self.declare_parameter(
            'collision_recovery_command_topic',                            # 碰撞恢复指令话题
            '/m20/control/collision_recovery_cmd',
        )
        self.declare_parameter('collision_recovery_timeout_sec', 0.15)     # 恢复指令新鲜时长
        self.declare_parameter('collision_recovery_ramp_sec', 1.0)         # 恢复指令斜坡时长
        self.declare_parameter(
            'execution_hold_topic', '/m20/control/execution_hold'          # 执行保持话题
        )
        self.declare_parameter('map_ready_topic', '/m20/map/ready')        # 地图就绪话题
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')          # 里程计话题
        self.declare_parameter('capability_profile_id', 'fallback_defaults')  # 平台能力档案 id
        self.declare_parameter('publish_rate_hz', 50.0)                     # 发布频率
        self.declare_parameter('command_timeout_sec', 0.5)                  # 指令超时
        self.declare_parameter('odom_timeout_sec', 0.5)                     # 里程计超时
        self.declare_parameter('max_linear_x', 0.45)                        # 线速度 x 限幅
        self.declare_parameter('max_linear_y', 0.20)                        # 线速度 y 限幅
        self.declare_parameter('max_angular_z', 0.65)                       # 角速度限幅
        self.declare_parameter('max_linear_accel', 1.0)                     # 线加速度上限
        self.declare_parameter('max_angular_accel', 1.2)                    # 角加速度上限
        self.declare_parameter('manual_priority', True)                     # 手动指令是否优先

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._command_timeout = max(          # 指令新鲜窗口（下限 0.05 s）
            0.05, float(self.get_parameter('command_timeout_sec').value)
        )
        self._odom_timeout = max(             # 里程计新鲜窗口（下限 0.05 s）
            0.05, float(self.get_parameter('odom_timeout_sec').value)
        )
        self._collision_recovery_timeout = max(  # 碰撞恢复指令新鲜窗口
            0.05,
            float(
                self.get_parameter(
                    'collision_recovery_timeout_sec'
                ).value
            ),
        )
        self._collision_recovery_ramp = max(  # 碰撞恢复比例斜坡时长
            0.0,
            float(
                self.get_parameter(
                    'collision_recovery_ramp_sec'
                ).value
            ),
        )
        self._limits: PlanarCommand = (       # 指令限幅 (vx_max, vy_max, wz_max)
            float(self.get_parameter('max_linear_x').value),
            float(self.get_parameter('max_linear_y').value),
            float(self.get_parameter('max_angular_z').value),
        )
        self._linear_acceleration = float(    # 线加速度上限
            self.get_parameter('max_linear_accel').value
        )
        self._angular_acceleration = float(   # 角加速度上限
            self.get_parameter('max_angular_accel').value
        )
        self._manual_priority = bool(         # 手动优先标志
            self.get_parameter('manual_priority').value
        )
        # ------------------------- 运行时状态 -------------------------
        self._navigation: Optional[TimedCommand] = None  # 最近导航指令（带时间戳）
        self._manual: Optional[TimedCommand] = None      # 最近手动指令（带时间戳）
        self._last_odom_time: Optional[float] = None     # 最近里程计时间
        self._map_ready = False              # 地图是否就绪
        self._e_stop = False                 # 急停是否触发
        self._floor_hold = False             # 楼层切换保持是否触发
        self._mission_hold = False           # 任务保持是否触发
        self._collision_guard_enabled = bool(  # 碰撞守卫门控开关
            self.get_parameter('collision_guard_enabled').value
        )
        # The native SCAN execution profile intentionally has no downstream
        # footprint veto.  Keep the multi-floor/e-stop command gate, but do
        # not fail closed waiting for a guard node that is not launched.
        # 【中文】原生 SCAN 执行档案有意不设下游足印否决权。保留多楼层/急停
        # 指令门，但不因等待一个未启动的守卫节点而 fail-closed。
        self._collision_stop = self._collision_guard_enabled  # 碰撞停止标志（未收到消息时默认按启用处理）
        self._collision_recovery_available = False  # 碰撞恢复可用标志
        self._collision_recovery: Optional[TimedCommand] = None  # 最近碰撞恢复指令
        self._last_output: PlanarCommand = (0.0, 0.0, 0.0)  # 上次输出指令（用于斜坡）
        self._last_publish_time = self._now()  # 上次发布时刻
        self._last_state = ''                  # 上次发布的安全状态（用于状态变化检测）

        latched_qos = QoSProfile(  # 锁存 QoS：新订阅者立即收到最近值
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # ------------------------- 发布器 -------------------------
        self._safe_publisher = self.create_publisher(  # 安全指令发布器
            Twist, str(self.get_parameter('safe_topic').value), 20
        )
        self._state_publisher = self.create_publisher(  # 安全状态发布器（锁存）
            String, '/m20/control/safety_state', latched_qos
        )
        self._execution_hold_publisher = self.create_publisher(  # 执行保持发布器（锁存）
            Bool,
            str(self.get_parameter('execution_hold_topic').value),
            latched_qos,
        )
        # ------------------------- 订阅器 -------------------------
        self.create_subscription(  # 导航候选指令订阅
            Twist,
            str(self.get_parameter('navigation_topic').value),
            self._navigation_callback,
            20,
        )
        self.create_subscription(  # 手动指令订阅
            Twist,
            str(self.get_parameter('manual_topic').value),
            self._manual_callback,
            20,
        )
        self.create_subscription(  # 急停状态订阅
            Bool,
            str(self.get_parameter('e_stop_topic').value),
            self._e_stop_callback,
            10,
        )
        self.create_subscription(  # 楼层切换保持订阅（锁存）
            Bool,
            str(self.get_parameter('floor_hold_topic').value),
            self._floor_hold_callback,
            latched_qos,
        )
        self.create_subscription(  # 任务保持订阅（锁存）
            Bool,
            str(self.get_parameter('mission_hold_topic').value),
            self._mission_hold_callback,
            latched_qos,
        )
        self.create_subscription(  # 碰撞停止订阅（锁存）
            Bool,
            str(self.get_parameter('collision_stop_topic').value),
            self._collision_stop_callback,
            latched_qos,
        )
        self.create_subscription(  # 碰撞恢复可用标志订阅（锁存）
            Bool,
            str(
                self.get_parameter(
                    'collision_recovery_available_topic'
                ).value
            ),
            self._collision_recovery_available_callback,
            latched_qos,
        )
        self.create_subscription(  # 碰撞恢复指令订阅
            Twist,
            str(
                self.get_parameter(
                    'collision_recovery_command_topic'
                ).value
            ),
            self._collision_recovery_callback,
            20,
        )
        self.create_subscription(  # 地图就绪订阅（锁存）
            Bool,
            str(self.get_parameter('map_ready_topic').value),
            self._map_ready_callback,
            latched_qos,
        )
        self.create_subscription(  # 里程计订阅（用于新鲜度判定）
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        # ------------------------- 主定时器 -------------------------
        self._timer = self.create_timer(1.0 / rate, self._publish)  # 周期评估并发布安全指令
        self._publish_immediate_zero('MAP_NOT_READY')  # 启动即输出零指令（地图未就绪）
        self.get_logger().info(
            'Safety supervisor ready: candidate/manual -> safe; '
            'fail-closed gates enabled; capability profile='
            f'{self.get_parameter("capability_profile_id").value}; '
            f'limits=({self._limits[0]:.2f},'
            f'{self._limits[1]:.2f},{self._limits[2]:.2f})'
        )

    def _now(self) -> float:
        # 【功能】返回当前单调时钟时间（秒）
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _command_from_message(message: Twist) -> PlanarCommand:
        # 【功能】从 Twist 消息提取平面指令 (vx, vy, wz)
        return message.linear.x, message.linear.y, message.angular.z

    @staticmethod
    def _message_from_command(command: PlanarCommand) -> Twist:
        # 【功能】把平面指令打包为 Twist 消息
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        return message

    def _navigation_callback(self, message: Twist) -> None:
        # 【回调】接收导航候选指令：限幅后连同时间戳保存
        self._navigation = TimedCommand(
            clamp_command(self._command_from_message(message), self._limits),
            self._now(),
        )

    def _manual_callback(self, message: Twist) -> None:
        # 【回调】接收手动指令：限幅后连同时间戳保存
        self._manual = TimedCommand(
            clamp_command(self._command_from_message(message), self._limits),
            self._now(),
        )

    def _e_stop_callback(self, message: Bool) -> None:
        # 【回调】接收急停状态：触发时立即输出零指令
        changed = self._e_stop != message.data
        self._e_stop = message.data
        if self._e_stop:
            self._publish_immediate_zero('E_STOP')
        if changed:
            self.get_logger().warn(
                f'e-stop {"asserted" if self._e_stop else "released"}'
            )

    def _floor_hold_callback(self, message: Bool) -> None:
        # 【回调】接收楼层切换保持：触发时立即输出零指令
        self._floor_hold = message.data
        if self._floor_hold:
            self._publish_immediate_zero('FLOOR_SWITCH_HOLD')

    def _mission_hold_callback(self, message: Bool) -> None:
        # 【回调】接收任务保持：触发时立即输出零指令
        self._mission_hold = message.data
        if self._mission_hold:
            self._publish_immediate_zero('MISSION_HOLD')

    def _map_ready_callback(self, message: Bool) -> None:
        # 【回调】接收地图就绪状态：未就绪时立即输出零指令
        self._map_ready = message.data
        if not self._map_ready:
            self._publish_immediate_zero('MAP_NOT_READY')

    def _collision_stop_callback(self, message: Bool) -> None:
        # 【回调】接收碰撞停止：守卫未启用时忽略；启用时触发立即零输出
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
        # 【回调】接收碰撞恢复可用标志
        self._collision_recovery_available = bool(message.data)

    def _collision_recovery_callback(self, message: Twist) -> None:
        # The guard may authorize only swept-clear forward rolling motion or
        # straight reverse. The final gate validates that shape before SDK.
        # 【中文】守卫只授权"扫掠畅通的前进滚动"或"直线后退"；最终门在交给
        # 后端之前再次校验该形状。
        # 【回调】接收碰撞恢复指令：限幅（横向分量强制为 0）后保存
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
        # 【回调】接收里程计：仅更新时间戳（用于新鲜度判定）
        self._last_odom_time = self._now()

    def _hard_stop_reason(self, now: float) -> Optional[str]:
        # 【功能】检查所有"硬停车"门控条件，返回触发原因（None 表示无）
        # 【参数】now - 当前时刻；【返回】触发原因字符串或 None
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
        # 【功能】获取未超时且形状合法的碰撞恢复指令
        # 【参数】now - 当前时刻；【返回】合法的恢复指令或 None
        if (
            not self._collision_recovery_available
            or self._collision_recovery is None
            or now - self._collision_recovery.stamp
            > self._collision_recovery_timeout
        ):
            return None
        command = self._collision_recovery.command
        if not valid_collision_recovery_command(command):  # 最终形状校验
            return None
        return command

    def _publish_state(self, state: str) -> None:
        # 【功能】发布安全状态（仅在状态变化时发布），并同步执行保持标志
        # 【参数】state - 安全状态名（如 NAVIGATION / COLLISION_STOP / E_STOP）
        if state != self._last_state:
            self._state_publisher.publish(String(data=state))
            # Only autonomous navigation advances the SCAN trajectory clock.
            # Manual override and every fail-closed state freeze execution.
            # 【中文】只有自主导航推进 SCAN 轨迹时钟；手动接管与所有
            # fail-closed 状态都会冻结执行。
            self._execution_hold_publisher.publish(
                Bool(data=state != 'NAVIGATION')
            )
            self.get_logger().info(f'safety state: {state}')
            self._last_state = state

    def _publish_immediate_zero(self, state: str) -> None:
        # 【功能】立即发布零速度指令（fail-closed）并更新状态
        # 【参数】state - 触发零指令的安全状态名
        self._last_output = (0.0, 0.0, 0.0)
        self._safe_publisher.publish(Twist())
        self._publish_state(state)

    def _publish(self) -> None:
        # 【功能】周期评估入口：按门控优先级输出安全指令
        now = self._now()
        dt = max(0.0, min(0.2, now - self._last_publish_time))  # 限幅步长（防时钟跳变）
        self._last_publish_time = now
        hard_stop = self._hard_stop_reason(now)  # 1) 硬停车门控
        if hard_stop is not None:
            self._publish_immediate_zero(hard_stop)
            return
        if self._collision_guard_enabled and self._collision_stop:  # 2) 碰撞停止门控
            recovery = self._fresh_collision_recovery(now)
            if recovery is None:
                self._publish_immediate_zero('COLLISION_STOP')
                return
            self._last_output = proportional_ramp_command(  # 按比例斜坡输出恢复指令（保持曲率）
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
        target, state = select_fresh_command(  # 3) 指令源选择（手动/导航，超时置零）
            now,
            self._command_timeout,
            self._navigation,
            self._manual,
            self._manual_priority,
        )
        target = clamp_command(target, self._limits)  # 再次限幅（双保险）
        output = slew_command(  # 4) 加速度限制
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
    # 【中文】入口：运行速度安全监督节点
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # CycloneDDS can invalidate a subscription while SIGINT is being
        # handled. Treat only that shutdown path as a clean stop.
        # 【中文】CycloneDDS 在处理 SIGINT 时可能使订阅失效；
        # 仅把该关闭路径视为干净停止。
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
