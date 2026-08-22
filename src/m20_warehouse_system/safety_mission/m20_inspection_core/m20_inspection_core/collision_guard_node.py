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
# 【文件职责】collision_guard_node.py —— 独立碰撞守卫节点（ROS2 Node）
# 本节点基于 SCAN 的在线占用体素点云实现独立的 M20 碰撞守卫：
#   1) 订阅 SCAN 占用体素点云（世界系），按机体高度带投影到以机器人为
#      中心的在线栅格（不依赖 PGM/YAML 静态地图）；
#   2) 用保守双圆足印对候选指令做前视预测（含名义/实测侧向漂移包络），
#      任一预测采样越界或占用即输出碰撞停止（fail-closed）；
#   3) 预测碰撞但当前足印未被硬车身栅格占用时，可输出经过完整扫掠校验
#      的滚动恢复指令（前进滚动 / 直线后退），并受有界恢复预算约束；
#   4) 维护恢复的连续清零确认与重新武装（rearm）计时，确保只在持续畅通
#      后释放；
#   5) 发布状态 / 诊断 / 恢复可用 / 恢复指令等话题供导航网关与安全监督使用。
# ============================================================================

"""Independent M20 collision guard over SCAN's online occupied voxels."""

from __future__ import annotations  # 延迟注解求值：允许内置泛型（tuple[...] 等）

# ------------------------- 第三方/标准库导入 -------------------------
import math                # 数学库：航向角计算
from typing import Optional  # 类型提示：Optional 可空值

from geometry_msgs.msg import Twist   # Twist 消息：候选指令/恢复指令
from nav_msgs.msg import Odometry     # 里程计消息：位姿与速度
import rclpy                          # ROS2 Python 客户端库
from rclpy.node import Node           # ROS2 节点基类
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # QoS 策略（锁存发布）
from sensor_msgs.msg import PointCloud2  # 点云消息：SCAN 占用体素
from sensor_msgs_py import point_cloud2  # 点云读取工具（read_points）
from std_msgs.msg import Bool, String # 标准消息：停止标志 / 状态与诊断字符串

from .collision_policy import (  # 导入碰撞预判纯函数
    conservative_raster_radius,
    first_blocking_command_envelope,
    GridGeometry,
    hard_body_raster_radius,
    inflate_blocked_grid,
    rasterize_online_occupancy,
    recovery_sweep_is_clear,
    safe_raster_shell_escape,
    safe_rolling_recovery,
    update_clear_confirmation,
    update_recovery_budget,
)


def _yaw_from_odometry(message: Odometry) -> float:
    # 【功能】从里程计四元数提取航向角 yaw
    # 【参数】message - 里程计消息；【返回】航向角（弧度）
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
    """Fail closed when the M20 sweep meets SCAN's live occupied voxels."""
    # 【中文】碰撞守卫节点：当 M20 足印扫掠与 SCAN 实时占用体素相交时 fail-closed

    def __init__(self) -> None:
        super().__init__('m20_collision_guard')
        # ------------------------- 话题/框架参数 -------------------------
        self.declare_parameter(
            'occupancy_cloud_topic', '/grid_map/occupancy'  # SCAN 在线占用点云话题
        )
        self.declare_parameter('expected_cloud_frame', 'world')  # 点云期望坐标系
        self.declare_parameter('occupancy_cloud_timeout_sec', 0.75)  # 点云超时
        self.declare_parameter('online_grid_resolution', 0.10)  # 在线栅格分辨率（米/格）
        self.declare_parameter('online_grid_size', 16.0)  # 在线栅格边长（米）
        self.declare_parameter('minimum_obstacle_z_from_body', -0.50)  # 障碍高度带下限
        self.declare_parameter('maximum_obstacle_z_from_body', 0.70)   # 障碍高度带上限
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')  # 里程计话题
        self.declare_parameter(
            'command_topic', '/m20/navigation/cmd_vel_candidate'  # 候选指令话题（预安全）
        )
        self.declare_parameter(
            'stop_topic', '/m20/control/collision_stop'  # 碰撞停止输出话题
        )
        self.declare_parameter(
            'state_topic', '/m20/control/collision_guard_state'  # 状态话题
        )
        self.declare_parameter(
            'diagnostic_topic',  # 诊断话题
            '/m20/control/collision_guard_diagnostic',
        )
        self.declare_parameter(
            'recovery_available_topic',  # 恢复可用标志话题
            '/m20/control/collision_recovery_available',
        )
        self.declare_parameter(
            'recovery_command_topic',  # 恢复指令话题
            '/m20/control/collision_recovery_cmd',
        )
        self.declare_parameter('capability_profile_id', 'fallback_defaults')  # 平台能力档案 id
        self.declare_parameter('minimum_centerline_turn_radius', 0.0)  # 最小中心线转弯半径（仅记录）
        self.declare_parameter('turn_swept_radius', 0.0)  # 外圈扫掠半径（仅记录）
        self.declare_parameter('occupied_threshold', 50)  # 占用判定阈值
        # Safe native-SCAN defaults validated with the official M20 in
        # MuJoCo. The optional grid-route launch overrides these explicitly.
        # 【中文】与官方 M20 在 MuJoCo 中验证的安全原生 SCAN 默认值；
        # 可选的栅格路线启动脚本会显式覆盖这些值。
        self.declare_parameter('footprint_radius', 0.25)  # 足印半径（米）
        self.declare_parameter('footprint_offset', 0.18)  # 前后圆偏移（米）
        self.declare_parameter('safety_margin', 0.05)     # 安全裕量（米）
        self.declare_parameter('lookahead_sec', 0.70)     # 前视时长（秒）
        self.declare_parameter('sample_period_sec', 0.05) # 预测采样周期（秒）
        self.declare_parameter('command_timeout_sec', 0.5)  # 候选指令超时（秒）
        self.declare_parameter('publish_rate_hz', 20.0)     # 评估/发布频率（Hz）
        self.declare_parameter('max_linear_x', 0.45)  # 线速度 x 限幅（与后端一致）
        self.declare_parameter('max_linear_y', 0.20)  # 线速度 y 限幅
        self.declare_parameter('max_angular_z', 0.65) # 角速度限幅
        self.declare_parameter(  # 正转向（左转）实测侧向漂移上限（米/秒）
            'model_positive_yaw_lateral_drift', 0.15
        )
        self.declare_parameter(  # 负转向（右转）实测侧向漂移上限（米/秒）
            'model_negative_yaw_lateral_drift', 0.10
        )
        self.declare_parameter(  # 反向侧向漂移不确定性（米/秒）
            'model_opposite_lateral_uncertainty', 0.05
        )
        self.declare_parameter('model_reference_yaw_rate', 0.65)  # 参考角速度（漂移缩放基准）
        # The official-policy cold matrix validated this rolling command.
        # Pure-yaw recovery is intentionally forbidden.
        # 【中文】官方策略冷矩阵验证了该滚动指令；纯旋转恢复被有意禁止。
        self.declare_parameter('recovery_forward_speed', 0.35)  # 恢复前进速度（米/秒）
        self.declare_parameter(  # 恢复：正转向侧向漂移上限
            'recovery_positive_yaw_lateral_drift', 0.15
        )
        self.declare_parameter(  # 恢复：负转向侧向漂移上限
            'recovery_negative_yaw_lateral_drift', 0.10
        )
        self.declare_parameter(  # 恢复：反向漂移不确定性
            'recovery_opposite_lateral_uncertainty', 0.05
        )
        self.declare_parameter('recovery_lookahead_sec', 0.90)  # 恢复前视时长
        self.declare_parameter('recovery_min_yaw_rate', 0.20)   # 恢复最小转向角速度
        self.declare_parameter('recovery_clear_confirm_sec', 0.30)  # 恢复释放确认时长
        self.declare_parameter('recovery_rearm_clear_sec', 1.50)    # 恢复重新武装确认时长
        self.declare_parameter('recovery_max_active_sec', 6.0)      # 恢复最大活动时长
        self.declare_parameter('recovery_max_displacement_m', 0.75) # 恢复最大总位移
        self.declare_parameter('recovery_progress_timeout_sec', 1.50)  # 恢复无进度超时
        self.declare_parameter('recovery_min_progress_m', 0.03)        # 恢复最小进度阈值

        # ------------------------- 运行时状态 -------------------------
        self._geometry: Optional[GridGeometry] = None  # 当前在线栅格几何
        self._blocked = None              # 保守（含外壳）阻塞栅格
        self._hard_body_blocked = None    # 硬车身阻塞栅格
        self._odom: Optional[Odometry] = None  # 最近里程计
        self._command = (0.0, 0.0, 0.0)   # 最近候选指令（限幅后）
        self._command_time: Optional[float] = None  # 候选指令时间戳
        self._last_cloud_time: Optional[float] = None  # 最近点云时间
        self._cloud_frame_valid = False   # 点云坐标系是否匹配
        self._online_accepted_points = 0  # 最近一次栅格化接受的占用点数
        self._online_map_announced = False  # 是否已打印在线地图就绪日志
        self._active_recovery: Optional[tuple[float, float, float]] = None  # 当前激活的恢复指令
        self._recovery_clear_since: Optional[float] = None  # 恢复畅通确认起始时刻
        self._recovery_rearm_clear_since: Optional[float] = None  # 恢复重新武装计时起始
        self._recovery_started_at: Optional[float] = None  # 恢复预算起始时刻
        self._recovery_start_xy: Optional[tuple[float, float]] = None  # 恢复预算起始位置
        self._recovery_progress_at: Optional[float] = None  # 恢复进度检查点时刻
        self._recovery_progress_xy: Optional[tuple[float, float]] = None  # 恢复进度检查点位置
        self._recovery_exhausted_reason = ''  # 恢复预算耗尽原因（锁存）
        self._last_state = ''             # 最近发布的状态（变化检测）
        self._last_diagnostic_key = ''    # 最近发布的诊断主键（变化检测）
        latched_qos = QoSProfile(  # 锁存 QoS：新订阅者立即收到最近值
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # ------------------------- 发布器 -------------------------
        self._stop_publisher = self.create_publisher(  # 碰撞停止发布器（锁存）
            Bool, str(self.get_parameter('stop_topic').value), latched_qos
        )
        self._state_publisher = self.create_publisher(  # 状态发布器（锁存）
            String, str(self.get_parameter('state_topic').value), latched_qos
        )
        self._diagnostic_publisher = self.create_publisher(  # 诊断发布器（锁存）
            String,
            str(self.get_parameter('diagnostic_topic').value),
            latched_qos,
        )
        self._recovery_available_publisher = self.create_publisher(  # 恢复可用发布器（锁存）
            Bool,
            str(self.get_parameter('recovery_available_topic').value),
            latched_qos,
        )
        self._recovery_command_publisher = self.create_publisher(  # 恢复指令发布器
            Twist,
            str(self.get_parameter('recovery_command_topic').value),
            20,
        )
        # ------------------------- 订阅器 -------------------------
        self._map_subscription = self.create_subscription(  # 占用点云订阅
            PointCloud2,
            str(self.get_parameter('occupancy_cloud_topic').value),
            self._cloud_callback,
            10,
        )
        self._odom_subscription = self.create_subscription(  # 里程计订阅
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self._command_subscription = self.create_subscription(  # 候选指令订阅
            Twist,
            str(self.get_parameter('command_topic').value),
            self._command_callback,
            20,
        )
        rate = max(  # 评估频率（下限 1 Hz）
            1.0, float(self.get_parameter('publish_rate_hz').value)
        )
        self._timer = self.create_timer(1.0 / rate, self._evaluate)  # 周期评估定时器
        self._publish(True, 'NOT_READY')  # 初始：未就绪即停止
        self._publish_recovery(None)      # 初始：无恢复指令

    def _now(self) -> float:
        # 【功能】返回当前单调时钟时间（秒）
        return self.get_clock().now().nanoseconds / 1e9

    def _cloud_callback(self, message: PointCloud2) -> None:
        """Build an in-memory safety raster from SCAN occupied voxels."""
        # 【中文】从 SCAN 占用体素构建内存安全栅格。
        # 【参数】message - SCAN 点云消息
        expected_frame = str(  # 校验点云坐标系
            self.get_parameter('expected_cloud_frame').value
        )
        if message.header.frame_id != expected_frame:
            self._cloud_frame_valid = False
            self._last_cloud_time = None
            self._publish(  # 坐标系不匹配：fail-closed
                True,
                'CLOUD_FRAME_MISMATCH',
                f'CLOUD_FRAME_MISMATCH; got={message.header.frame_id}; '
                f'expected={expected_frame}',
            )
            return
        if self._odom is None:
            return  # 尚无里程计：无法定位栅格中心

        position = self._odom.pose.pose.position
        minimum_z = float(position.z) + float(  # 机体高度带下限（世界系 z）
            self.get_parameter('minimum_obstacle_z_from_body').value
        )
        maximum_z = float(position.z) + float(  # 机体高度带上限（世界系 z）
            self.get_parameter('maximum_obstacle_z_from_body').value
        )
        try:
            points = point_cloud2.read_points(  # 读取点云 (x, y, z)
                message,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
            occupancy, geometry, accepted = rasterize_online_occupancy(  # 栅格化
                points,
                (float(position.x), float(position.y)),
                float(
                    self.get_parameter('online_grid_resolution').value
                ),
                float(self.get_parameter('online_grid_size').value),
                minimum_z,
                maximum_z,
            )
        except (KeyError, TypeError, ValueError) as error:
            self._cloud_frame_valid = False
            self._last_cloud_time = None
            self._publish(  # 点云无效：fail-closed
                True,
                'INVALID_OCCUPANCY_CLOUD',
                f'INVALID_OCCUPANCY_CLOUD; {error}',
            )
            return

        physical_radius = (  # 足印物理半径 = 足印半径 + 安全裕量
            float(self.get_parameter('footprint_radius').value)
            + float(self.get_parameter('safety_margin').value)
        )
        hard_body_radius = hard_body_raster_radius(  # 硬车身栅格化半径
            float(self.get_parameter('footprint_radius').value),
            geometry.resolution,
        )
        raster_radius = conservative_raster_radius(  # 保守栅格化半径（含半格对角线）
            physical_radius,
            geometry.resolution,
        )
        self._hard_body_blocked = inflate_blocked_grid(  # 硬车身阻塞栅格
            occupancy,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            hard_body_radius,
        )
        self._blocked = inflate_blocked_grid(  # 保守阻塞栅格
            occupancy,
            geometry,
            int(self.get_parameter('occupied_threshold').value),
            raster_radius,
        )
        self._geometry = geometry
        self._online_accepted_points = accepted
        self._last_cloud_time = self._now()
        self._cloud_frame_valid = True
        if not self._online_map_announced:  # 首次就绪：打印在线地图参数
            self._online_map_announced = True
            self.get_logger().info(
                'online SCAN collision map ready; '
                f'window={geometry.width * geometry.resolution:.1f}m; '
                f'resolution={geometry.resolution:.2f}m; '
                f'height_band=[{minimum_z:.2f},{maximum_z:.2f}]m; '
                f'double-circle radius={physical_radius:.2f}m; '
                f'conservative raster radius={raster_radius:.3f}m; '
                f'hard-body raster radius={hard_body_radius:.3f}m; '
                f'offset='
                f'{float(self.get_parameter("footprint_offset").value):.2f}m'
            )
            minimum_turn_radius = float(  # 打印平台能力档案信息
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
        # 【回调】缓存最近里程计
        self._odom = message

    def _command_callback(self, message: Twist) -> None:
        # This is the adapted pre-safety candidate, so prediction and backend
        # execution share one body-command model without reading safe output.
        # 【中文】这是适配后的预安全候选指令，因此预测与后端执行共用同一
        # 机体指令模型，无需读取安全输出。
        # 【回调】接收候选指令：按后端限幅钳制后保存
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
        # 【功能】发布停止标志、状态与诊断（仅在变化时发布，避免刷屏）
        # 【参数】stop - 是否停止；state - 状态名；diagnostic - 诊断文本（可空）
        self._stop_publisher.publish(Bool(data=stop))
        state_changed = state != self._last_state
        diagnostic_text = diagnostic or state
        diagnostic_key = diagnostic_text.split(';', 1)[0]  # 诊断主键（分号前部分）
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
        # 【中文】清除激活指令、恢复预算与任何耗尽锁存
        self._clear_active_recovery()
        self._recovery_rearm_clear_since = None
        self._recovery_started_at = None
        self._recovery_start_xy = None
        self._recovery_progress_at = None
        self._recovery_progress_xy = None
        self._recovery_exhausted_reason = ''

    def _clear_active_recovery(self) -> None:
        """End the current escape command without rearming its budget."""
        # 【中文】结束当前逃逸指令（不重新武装其预算）
        self._active_recovery = None
        self._recovery_clear_since = None

    def _recovery_episode_exists(self) -> bool:
        """Return whether a recovery budget or exhaustion latch is live."""
        # 【中文】判断是否存在活跃的恢复预算或耗尽锁存（恢复阶段尚未完全结束）
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
        # 【中文】启动恢复预算（仅一次）；改变恢复方向不得重置预算。
        # 【参数】now - 当前时刻；xy - 当前平面位置
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
        # 【中文】更新恢复预算，返回必须停止恢复的（锁存）原因
        # 【参数】now - 当前时刻；xy - 当前平面位置
        # 【返回】耗尽原因字符串或 None
        if self._recovery_exhausted_reason:
            return self._recovery_exhausted_reason
        if (
            self._recovery_started_at is None
            or self._recovery_start_xy is None
            or self._recovery_progress_at is None
            or self._recovery_progress_xy is None
        ):
            return None
        progress_at, progress_xy, reason = update_recovery_budget(  # 调用纯函数更新预算
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
            self._recovery_exhausted_reason = reason  # 锁存耗尽原因
            self._active_recovery = None
        return reason

    def _publish_recovery(
        self,
        command: Optional[tuple[float, float, float]],
    ) -> None:
        """Publish only a collision-checked rolling recovery command."""
        # 【中文】仅发布经过碰撞检查的滚动恢复指令（横向速度恒为 0）。
        # 【参数】command - 恢复指令 (vx, 0, wz) 或 None（无恢复）
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
        # 【主流程】周期评估：判断是否碰撞停止 / 输出恢复指令 / 释放恢复
        now = self._now()
        cloud_timeout = max(  # 点云超时（下限 0.05 s）
            0.05,
            float(
                self.get_parameter('occupancy_cloud_timeout_sec').value
            ),
        )
        if (  # 前置条件不满足：地图未就绪/过期/缺栅格/缺里程计 -> fail-closed
            not self._cloud_frame_valid
            or self._last_cloud_time is None
            or now - self._last_cloud_time > cloud_timeout
            or self._geometry is None
            or self._blocked is None
            or self._hard_body_blocked is None
            or self._odom is None
        ):
            self._reset_recovery()
            self._publish_recovery(None)
            state = (
                'ONLINE_MAP_STALE'
                if self._last_cloud_time is not None
                and now - self._last_cloud_time > cloud_timeout
                else 'NOT_READY'
            )
            self._publish(True, state)
            return
        command = self._command
        if (  # 候选指令超时：按零指令预测
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
        blocked_sample = first_blocking_command_envelope(  # 前视包络检查（含漂移变体）
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
        if blocked_sample is not None:  # ------------- 预测碰撞分支 -------------
            self._recovery_clear_since = None
            # Any renewed prediction breaks the continuous-clear rearm timer.
            # 【中文】任何新一次的预测碰撞都会打断"连续畅通"的重新武装计时。
            self._recovery_rearm_clear_since = None
            trigger = (
                'CURRENT_FOOTPRINT'
                if blocked_sample.sample_index == 0  # 当前足印即被阻塞
                else 'PREDICTED_FOOTPRINT'          # 预测足印被阻塞
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
            if blocked_sample.sample_index > 0:  # 预测（而非当前）碰撞：可尝试滚动恢复
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
                positive_drift = float(  # 恢复：正转向漂移
                    self.get_parameter(
                        'recovery_positive_yaw_lateral_drift'
                    ).value
                )
                negative_drift = float(  # 恢复：负转向漂移
                    self.get_parameter(
                        'recovery_negative_yaw_lateral_drift'
                    ).value
                )
                opposite_drift = float(  # 恢复：反向漂移不确定性
                    self.get_parameter(
                        'recovery_opposite_lateral_uncertainty'
                    ).value
                )
                budget_reason = self._update_recovery_budget(  # 更新恢复预算
                    now, current_xy
                )
                # Preserve a selected direction while its full sweep is safe.
                # Direction changes share one budget and cannot restart it.
                # 【中文】只要选定方向的完整扫掠畅通就保持该方向；
                # 方向改变共享同一预算，不能重启预算。
                if budget_reason:
                    diagnostic = (
                        'RECOVERY_BUDGET_EXHAUSTED; '
                        f'reason={budget_reason}; trigger={diagnostic}'
                    )
                elif (  # 已有激活恢复且其扫掠仍畅通：继续使用
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
                elif not self._recovery_exhausted_reason:  # 预算未耗尽：重新计算恢复
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
                    self._start_recovery_budget(now, current_xy)  # 启动/保持预算
                    recovery_text = (
                        f'recovery=({recovery[0]:.2f},'
                        f'0.00,{recovery[2]:.2f})'
                    )
                    if (  # 地图边界内向恢复：特殊标记
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
                    # 【中文】预测碰撞但两个经验证的滚动逃逸方向都不可用时，
                    # 必须让导航网关可观察到该终态诊断；否则守卫将永远安全
                    # 保持零速度，但恢复预算永不启动，活动的 SCAN 目标无法重置。
                    diagnostic = (
                        'RECOVERY_UNAVAILABLE; '
                        f'trigger={diagnostic}'
                    )
            else:
                # A hard-body overlap never authorizes motion. A false current
                # hit created only by the conservative half-cell shell may use
                # one bounded straight escape whose hard sweep is clear and
                # whose endpoint returns to conservative free space.
                # 【中文】硬车身重叠绝不授权运动；仅由保守半格外壳造成的
                # 当前命中可使用一次有界直线逃逸（其硬车身扫掠畅通且终点
                # 回到保守自由空间）。
                recovery = safe_raster_shell_escape(  # 光栅外壳逃逸
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
                    # 【中文】为网关保留任何已耗尽的恢复阶段；仅在持续
                    # "保守畅通"时间间隔之后才重置。
                    self._clear_active_recovery()
        else:  # ------------- 畅通分支（释放恢复） -------------
            # A single clear grid lookup at an obstacle-cell boundary is not
            # enough to release recovery.  Require a continuous clear period
            # while the already selected recovery sweep remains safe.
            # 【中文】在障碍单元边界的单次畅通查询不足以释放恢复；
            # 要求已选恢复扫掠保持安全的同时出现一段连续畅通时间。
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
                budget_reason = self._update_recovery_budget(  # 更新预算
                    now, current_xy
                )
                recovery_clear = False
                if not budget_reason:
                    recovery_clear = recovery_sweep_is_clear(  # 当前扫掠是否仍畅通
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
                clear_since, confirmed = update_clear_confirmation(  # 连续畅通确认
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
                elif recovery_clear and not confirmed:  # 扫掠畅通但尚未确认：继续恢复
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
                    # 【中文】在短释放确认（或扫掠本身不安全时立即）结束激活
                    # 恢复，但在更长的重新武装周期内保留阶段预算。
                    self._clear_active_recovery()
                    self._recovery_rearm_clear_since = clear_since
                    elapsed = max(0.0, now - float(clear_since))
                    diagnostic = (
                        'RECOVERY_EPISODE_REARM; '
                        f'clear_for={elapsed:.2f}s'
                    )
            if (  # 无激活恢复但存在恢复阶段：等待重新武装后彻底重置
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
    # 【中文】入口：运行独立碰撞守卫节点
    rclpy.init()
    node = CollisionGuard()
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
