# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：trajectory_progress_tracker_node.py
# 所属：m20_locomotion_control —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：按实测 M20 路径进度执行 SCAN 几何轨迹的 ROS 节点。
#   - 订阅 SCAN B 样条（/planning/bspline），采样为弧长路径；
#   - 订阅里程计（/m20/sim/body_pose），用 SpatialProgressFollower 按实测空间进度
#     生成滚动指令，再经 RollingNavigationAdapter 适配为安全候选指令
#     （/m20/navigation/cmd_vel_candidate）；
#   - 发布候选模式与 JSON 状态诊断（进度/剩余/横向偏差/航向误差/倒车跟踪等）；
#   - 含执行保持、里程计超时/过期、轨迹无效等零指令保护。
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

"""Execute SCAN geometry using measured M20 path progress."""
# 【中文注释】模块说明：按实测 M20 路径进度执行 SCAN 几何轨迹。

from dataclasses import fields      # 数据类字段遍历（用于声明意图参数）
import json                         # JSON 序列化（状态诊断发布）
import math                         # 数学库：atan2 等

from geometry_msgs.msg import Twist  # 速度指令消息
from nav_msgs.msg import Odometry    # 里程计消息
import rclpy                         # ROS2 Python 客户端库
from rclpy.node import Node          # ROS2 节点基类
from rclpy.qos import (              # QoS 策略
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,         # 传感器数据 QoS（里程计）
)
from scan_planner_msgs.msg import Bspline  # SCAN 规划器 B 样条消息
from std_msgs.msg import Bool, String      # 标准消息：布尔、字符串

from .motion_intent import IntentParameters, RollingNavigationAdapter  # 意图参数/滚动适配器
from .trajectory_progress import (   # 轨迹进度模块
    Bspline2D,                       #   B 样条求值器
    ProgressFollowerParameters,      #   进度跟随器参数
    SampledPath,                     #   采样路径
    SpatialProgressFollower,         #   空间进度跟随器
)


def _odometry_yaw(message: Odometry) -> float:
    # 【中文注释】从里程计四元数提取偏航角（yaw）：用 z 轴旋转的 atan2 公式。
    orientation = message.pose.pose.orientation
    sine = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cosine = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sine, cosine)


class TrajectoryProgressTracker(Node):
    """Publish an M20-safe candidate from SCAN's unchanged spatial curve."""
    # 【中文注释】轨迹进度跟踪器节点：基于 SCAN 未改动的空间曲线发布 M20 安全候选。

    def __init__(self) -> None:
        # 【中文注释】构造函数：声明参数、构建跟随器与适配器、创建收发端。
        super().__init__('m20_trajectory_progress_tracker')
        self.declare_parameter('trajectory_topic', '/planning/bspline')  # B 样条轨迹话题
        self.declare_parameter('odometry_topic', '/m20/sim/body_pose')   # 里程计话题
        self.declare_parameter(       # 执行保持话题
            'execution_hold_topic', '/m20/control/execution_hold'
        )
        self.declare_parameter(       # 输出话题：安全候选指令
            'output_topic', '/m20/navigation/cmd_vel_candidate'
        )
        self.declare_parameter(       # 模式话题
            'mode_topic', '/m20/navigation/candidate_mode'
        )
        self.declare_parameter(       # 状态诊断话题
            'state_topic', '/m20/navigation/progress_tracker_state'
        )
        self.declare_parameter('control_rate_hz', 50.0)      # 控制频率
        self.declare_parameter('odometry_timeout_sec', 0.25) # 里程计新鲜度超时
        self.declare_parameter('spline_sample_period_sec', 0.02)  # 样条采样周期
        self.declare_parameter('lookahead_m', 0.60)  # 空间前视距离
        self.declare_parameter('yaw_gain', 1.50)     # 航向误差增益
        self.declare_parameter('finish_distance_m', 0.15)  # 终点判定距离
        self.declare_parameter('terminal_speed_threshold_mps', 0.10)  # 终点速度阈值
        self.declare_parameter('terminal_slowdown_distance_m', 0.80)  # 终点减速距离
        self.declare_parameter('terminal_min_speed_mps', 0.10)        # 终点最小速度
        self.declare_parameter('reverse_tracking_enabled', True)      # 是否启用倒车跟踪
        self.declare_parameter('reverse_tracking_enter_angle', 2.10)  # 进入倒车角度
        self.declare_parameter('reverse_tracking_exit_angle', 1.75)   # 退出倒车角度
        self.declare_parameter('capability_profile_id', 'fallback_defaults')  # 能力配置 ID
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)  # 最小中心线转弯半径
        self.declare_parameter('turn_swept_radius', 0.0)               # 转弯扫掠半径

        intent_defaults = IntentParameters()  # 用默认值声明全部意图参数
        for item in fields(IntentParameters):
            self.declare_parameter(
                item.name,
                getattr(intent_defaults, item.name),
            )
        intent_parameters = IntentParameters(  # 从 ROS 参数构建意图参数
            **{
                item.name: self.get_parameter(item.name).value
                for item in fields(IntentParameters)
            }
        )
        self._adapter = RollingNavigationAdapter(intent_parameters)  # 滚动导航适配器
        self._follower = SpatialProgressFollower(  # 空间进度跟随器
            ProgressFollowerParameters(
                lookahead_m=float(
                    self.get_parameter('lookahead_m').value
                ),
                max_speed_mps=intent_parameters.max_forward,  # 最大速度取自意图参数
                yaw_gain=float(self.get_parameter('yaw_gain').value),
                max_yaw_radps=intent_parameters.max_yaw,      # 最大偏航取自意图参数
                finish_distance_m=float(
                    self.get_parameter('finish_distance_m').value
                ),
                terminal_speed_threshold_mps=float(
                    self.get_parameter(
                        'terminal_speed_threshold_mps'
                    ).value
                ),
                terminal_slowdown_distance_m=float(
                    self.get_parameter(
                        'terminal_slowdown_distance_m'
                    ).value
                ),
                terminal_min_speed_mps=float(
                    self.get_parameter('terminal_min_speed_mps').value
                ),
                reverse_tracking_enabled=bool(
                    self.get_parameter('reverse_tracking_enabled').value
                ),
                reverse_enter_angle_rad=float(
                    self.get_parameter(
                        'reverse_tracking_enter_angle'
                    ).value
                ),
                reverse_exit_angle_rad=float(
                    self.get_parameter(
                        'reverse_tracking_exit_angle'
                    ).value
                ),
            )
        )
        self._sample_period_sec = max(  # 样条采样周期（下限 5ms）
            0.005,
            float(
                self.get_parameter('spline_sample_period_sec').value
            ),
        )
        self._odometry_timeout_sec = max(  # 里程计超时（下限 0.05s）
            0.05,
            float(self.get_parameter('odometry_timeout_sec').value),
        )
        self._path = None            # 当前采样路径
        self._trajectory_id = -1     # 当前轨迹 ID
        self._pose = None            # 最近位姿 (x, y, yaw)
        self._last_odometry_ns = 0   # 最近里程计时间戳
        self._last_update_ns = 0     # 最近控制更新时间戳
        self._execution_hold = True  # 执行保持（初始为保持）
        self._last_mode = ''         # 最近发布的模式
        self._last_state = {'reason': 'STARTING'}  # 最近状态诊断

        latched = QoSProfile(  # 锁存 QoS：TRANSIENT_LOCAL
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_publisher = self.create_publisher(  # 候选指令发布器
            Twist,
            str(self.get_parameter('output_topic').value),
            20,
        )
        self._mode_publisher = self.create_publisher(  # 模式发布器（锁存）
            String,
            str(self.get_parameter('mode_topic').value),
            latched,
        )
        self._state_publisher = self.create_publisher(  # 状态诊断发布器
            String,
            str(self.get_parameter('state_topic').value),
            10,
        )
        self.create_subscription(  # 订阅 B 样条轨迹
            Bspline,
            str(self.get_parameter('trajectory_topic').value),
            self._trajectory_callback,
            10,
        )
        self.create_subscription(  # 订阅里程计（传感器 QoS）
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(  # 订阅执行保持
            Bool,
            str(self.get_parameter('execution_hold_topic').value),
            self._execution_hold_callback,
            latched,
        )
        control_rate = max(  # 控制频率（下限 10Hz）
            10.0,
            float(self.get_parameter('control_rate_hz').value),
        )
        self._control_timer = self.create_timer(  # 控制循环定时器
            1.0 / control_rate,
            self._control_callback,
        )
        self._diagnostic_timer = self.create_timer(  # 诊断发布定时器
            0.10,
            self._publish_state,
        )
        self._publish_zero('STARTING')  # 初始发布零指令
        self.get_logger().info(
            'M20 trajectory progress tracker ready: unchanged SCAN B-spline '
            '-> measured spatial progress -> safety candidate; lookahead='
            f'{self._follower.parameters.lookahead_m:.3f}m; profile='
            f'{self.get_parameter("capability_profile_id").value}'
        )

    def _trajectory_callback(self, message: Bspline) -> None:
        # 【中文注释】B 样条轨迹回调：采样为弧长路径，无效轨迹则清零保护。
        try:
            spline = Bspline2D(  # 构造 B 样条（控制点 x/y、阶数、节点向量）
                [(point.x, point.y) for point in message.pos_pts],
                int(message.order),
                list(message.knots),
            )
            path = SampledPath.from_spline(  # 密集采样为弧长路径
                spline,
                self._sample_period_sec,
            )
        except (TypeError, ValueError) as error:  # 无效/退化轨迹 → 零指令
            self._path = None
            self._trajectory_id = int(message.traj_id)
            self._adapter.reset()
            self._last_state = {
                'reason': 'INVALID_TRAJECTORY',
                'trajectory_id': self._trajectory_id,
                'detail': str(error),
            }
            self._command_publisher.publish(Twist())
            self.get_logger().warn(
                f'Ignoring invalid/degenerate trajectory: {error}'
            )
            return
        self._path = path                  # 保存有效路径
        self._trajectory_id = int(message.traj_id)
        self._follower.start_trajectory()  # 复位进度
        self._last_state = {               # 更新状态诊断
            'reason': 'TRAJECTORY_READY',
            'trajectory_id': self._trajectory_id,
            'path_length_m': path.length_m,
            'terminal_speed_mps': path.end_speed_mps,
        }

    def _odometry_callback(self, message: Odometry) -> None:
        # 【中文注释】里程计回调：保存位姿 (x, y, yaw)，非有限值则置空。
        pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _odometry_yaw(message),
        )
        if not all(math.isfinite(value) for value in pose):  # 非有限 → 无位姿
            self._pose = None
            self._last_odometry_ns = 0
            return
        self._pose = pose
        self._last_odometry_ns = self.get_clock().now().nanoseconds

    def _execution_hold_callback(self, message: Bool) -> None:
        # 【中文注释】执行保持回调：进入保持时复位适配器。
        hold = bool(message.data)
        if hold and not self._execution_hold:
            self._adapter.reset()
        self._execution_hold = hold

    def _publish_zero(self, reason: str) -> None:
        # 【中文注释】发布零指令与 STOPPED 模式，并记录原因。
        self._adapter.reset()
        self._command_publisher.publish(Twist())
        if self._last_mode != 'STOPPED':
            self._mode_publisher.publish(String(data='STOPPED'))
            self._last_mode = 'STOPPED'
        self._last_state = {
            'reason': reason,
            'trajectory_id': self._trajectory_id,
        }

    def _control_callback(self) -> None:
        # 【中文注释】控制循环：检查保持/轨迹/里程计后更新跟随器与适配器并发布。
        now_ns = self.get_clock().now().nanoseconds
        dt = (  # 时间步长（首帧 0.02s，后续夹到 [0.001, 0.10]s）
            0.02
            if self._last_update_ns == 0
            else max(
                0.001,
                min(0.10, (now_ns - self._last_update_ns) / 1.0e9),
            )
        )
        self._last_update_ns = now_ns
        if self._execution_hold:  # 执行保持 → 零指令
            self._publish_zero('EXECUTION_HOLD')
            return
        if self._path is None:  # 无轨迹 → 零指令
            self._publish_zero('TRAJECTORY_UNAVAILABLE')
            return
        if self._pose is None or self._last_odometry_ns == 0:  # 无里程计 → 零指令
            self._publish_zero('ODOMETRY_UNAVAILABLE')
            return
        odometry_age = (now_ns - self._last_odometry_ns) / 1.0e9
        if odometry_age > self._odometry_timeout_sec:  # 里程计过期 → 零指令
            self._publish_zero('ODOMETRY_STALE')
            return

        result = self._follower.update(self._path, *self._pose)  # 进度跟随
        intent, command = self._adapter.update(result.command, dt)  # 滚动适配
        output = Twist()  # 发布候选指令
        output.linear.x = command[0]
        output.linear.y = command[1]
        output.angular.z = command[2]
        self._command_publisher.publish(output)
        if intent.value != self._last_mode:  # 模式变化时发布
            self._mode_publisher.publish(String(data=intent.value))
            self._last_mode = intent.value
        self._last_state = {  # 更新状态诊断
            'reason': 'FINISHED' if result.finished else 'TRACKING',
            'trajectory_id': self._trajectory_id,
            'progress_m': result.progress_m,
            'remaining_m': result.remaining_m,
            'cross_track_m': result.cross_track_m,
            'lookahead_target': list(result.target),
            'heading_error_rad': result.heading_error_rad,
            'reverse_tracking': result.reverse_tracking,
            'raw_command': list(result.command),
            'candidate_command': list(command),
            'odometry_age_sec': odometry_age,
        }

    def _publish_state(self) -> None:
        # 【中文注释】发布状态诊断（JSON 字符串）。
        self._state_publisher.publish(
            String(
                data=json.dumps(
                    self._last_state,
                    separators=(',', ':'),
                )
            )
        )


def main(args=None) -> None:
    """Run the optional M20 spatial-progress trajectory executor."""
    # 【中文注释】节点入口：初始化、自旋；退出时发布零指令并清理。
    rclpy.init(args=args)
    node = TrajectoryProgressTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:  # 退出前发布零指令
                node._command_publisher.publish(Twist())
            except (Exception, KeyboardInterrupt):
                pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
