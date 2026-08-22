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

# =============================================================================
# navigation_gateway_node.py —— 楼层感知的"类型化"导航网关节点
# 所属模块：m20_scan_navigation（m20_navigation_gateway 可执行节点）
# 职责：
#   1. 对外提供 NavigateFloor action 服务（类型化接口：goal_id + floor_id +
#      map_generation + 目标位姿），串行化处理原生 SCAN 目标；
#   2. 把目标原样转发给原生 SCAN 规划器（direct_goal_topic），并在每次
#      导航前调用 /m20/navigation/reset 服务复位 SCAN 状态机；
#   3. 对接楼层状态(/m20/map/state)、里程计、碰撞守卫(/m20/control/*)，
#      实现"守卫未 CLEAR 不动"的 fail-closed 安全契约；
#   4. 可选接管 RViz 手点目标（/move_base_simple/goal），与类型化 action
#      互斥，走同一套"复位 -> 等守卫 -> 转发"的有界重规划流程；
#   5. 碰撞恢复后按硬性次数上限触发 SCAN 重规划（策略见 recovery_replan.py）。
# 运行：main() 使用 4 线程 MultiThreadedExecutor，服务回调可并发。
# =============================================================================

"""Floor-aware Action and managed RViz boundary for native SCAN goals."""

from copy import deepcopy      # 深拷贝目标消息，避免修改原始消息
import math                    # 数学函数（hypot/isfinite/inf）
import threading               # 线程锁（跨回调保护共享状态）
import time                    # 单调时钟（超时/延迟判定）
from typing import Optional    # 可选类型标注

from geometry_msgs.msg import PoseStamped                       # 带时间戳的位姿消息
from m20_warehouse_interfaces.action import NavigateFloor       # 类型化导航 action 定义
from m20_warehouse_interfaces.msg import FloorState, NavigationState   # 楼层状态/导航状态消息
from m20_warehouse_interfaces.srv import ResetNavigation        # 复位导航服务
from nav_msgs.msg import Odometry                               # 里程计消息
import rclpy                                                    # ROS2 Python 客户端库
from rclpy.action import ActionServer, CancelResponse, GoalResponse   # action 服务端
from rclpy.callback_groups import ReentrantCallbackGroup        # 可重入回调组（允许并发回调）
from rclpy.executors import MultiThreadedExecutor               # 多线程执行器
from rclpy.node import Node                                     # ROS2 节点基类
from rclpy.qos import (
    DurabilityPolicy,       # QoS 持久性策略（TRANSIENT_LOCAL 等）
    QoSProfile,             # QoS 配置类
    ReliabilityPolicy,      # QoS 可靠性策略（RELIABLE 等）
)
from std_msgs.msg import Bool, String       # 布尔/字符串消息

# 引入碰撞重规划策略函数（本包内相对导入）
from .recovery_replan import (
    collision_replan_available,      # 当前目标内是否还可再重规划
    recovery_replan_required,        # 诊断是否要求重新规划
)


def _latched_qos() -> QoSProfile:
    """返回系统状态话题使用的可靠 + transient-local QoS。"""
    return QoSProfile(
        depth=1,                                             # 队列深度 1（只保留最新）
        reliability=ReliabilityPolicy.RELIABLE,              # 可靠传输（不丢消息）
        durability=DurabilityPolicy.TRANSIENT_LOCAL,         # 迟订阅者也能收到最近一帧
    )


def guard_allows_motion(
    required: bool,
    collision_stop: bool,
    diagnostic: str,
) -> bool:
    """按执行档位契约判定碰撞守卫是否允许运动。"""
    return bool(
        not required                       # 不要求守卫（scan_native 档）：直接放行
        or (not collision_stop and diagnostic == 'CLEAR')   # 要求守卫时：无停车标志且诊断为 CLEAR
    )


class NavigationGateway(Node):
    """串行化原生 SCAN 目标，并把它们绑定到同一份地图代次（generation）。"""

    def __init__(self) -> None:
        """创建类型化与受管 RViz 两种导航边界。"""
        super().__init__('m20_navigation_gateway')
        # ---- 声明全部 ROS2 参数（默认值与 navigation_gateway.yaml 对应）----
        self.declare_parameter('action_name', '/m20/navigation/navigate')          # 类型化 action 名
        self.declare_parameter('default_timeout_sec', 120.0)                        # 默认导航超时（秒）
        self.declare_parameter('feedback_period_sec', 0.20)                         # 反馈周期（秒）
        self.declare_parameter('position_tolerance', 0.20)                          # 到点位置容差（米）
        self.declare_parameter('floor_hold_release_timeout_sec', 2.0)               # 楼层保持释放超时（秒）
        self.declare_parameter('navigation_reset_timeout_sec', 10.0)                # SCAN 复位服务超时（秒）
        self.declare_parameter('collision_guard_required', True)                    # 是否强制要求碰撞守卫
        self.declare_parameter('collision_replan_enabled', True)                    # 是否启用碰撞重规划
        self.declare_parameter('collision_replan_max_attempts', 2)                  # 单目标内重规划次数上限
        self.declare_parameter('collision_replan_rearm_timeout_sec', 5.0)           # 守卫重新就绪(CLEAR)等待超时
        self.declare_parameter(
            'collision_stop_topic', '/m20/control/collision_stop'                   # 碰撞停车标志话题
        )
        self.declare_parameter(
            'collision_diagnostic_topic',
            '/m20/control/collision_guard_diagnostic',                              # 碰撞守卫诊断话题
        )
        self.declare_parameter(
            'scan_reset_service', '/m20/navigation/reset'                           # SCAN 复位服务名
        )
        self.declare_parameter(
            'direct_goal_topic', '/m20/navigation/scan_goal'                        # 发给 SCAN 的目标话题
        )
        self.declare_parameter('manual_goal_monitor_enabled', False)                # 是否接管 RViz 手点目标
        self.declare_parameter(
            'manual_goal_topic', '/move_base_simple/goal'                           # RViz 手点目标话题
        )
        self.declare_parameter(
            'manual_state_topic', '/m20/navigation/manual_state'                    # 手点导航状态话题
        )
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')                  # 里程计/机身位姿话题
        self.declare_parameter('manual_timeout_sec', 300.0)                         # 手点导航超时（秒）
        self.declare_parameter(
            'terminal_orientation_mode', 'position_only'                            # 末端朝向模式（仅位置）
        )

        # ---- 读取参数并做下限钳制（防止配置出 0/负值）----
        self._default_timeout = max(
            1.0, float(self.get_parameter('default_timeout_sec').value)
        )
        self._feedback_period = max(
            0.05, float(self.get_parameter('feedback_period_sec').value)
        )
        self._position_tolerance = max(
            0.05, float(self.get_parameter('position_tolerance').value)
        )
        self._floor_hold_release_timeout = max(
            0.1,
            float(
                self.get_parameter(
                    'floor_hold_release_timeout_sec'
                ).value
            ),
        )
        self._navigation_reset_timeout = max(
            0.5,
            float(
                self.get_parameter(
                    'navigation_reset_timeout_sec'
                ).value
            ),
        )
        self._collision_guard_required = bool(
            self.get_parameter('collision_guard_required').value
        )
        self._collision_replan_enabled = bool(
            self.get_parameter('collision_replan_enabled').value
        )
        self._collision_replan_max_attempts = max(
            0,
            int(
                self.get_parameter(
                    'collision_replan_max_attempts'
                ).value
            ),
        )
        self._collision_replan_rearm_timeout = max(
            0.1,
            float(
                self.get_parameter(
                    'collision_replan_rearm_timeout_sec'
                ).value
            ),
        )
        self._manual_goal_monitor_enabled = bool(
            self.get_parameter('manual_goal_monitor_enabled').value
        )
        self._manual_timeout = max(
            1.0, float(self.get_parameter('manual_timeout_sec').value)
        )
        self._terminal_orientation_mode = str(
            self.get_parameter('terminal_orientation_mode').value
        ).strip().lower()
        # 目前只支持"仅位置"模式；严格 yaw 需要带转向口袋的规划器
        if self._terminal_orientation_mode != 'position_only':
            raise ValueError(
                'terminal_orientation_mode must be position_only; '
                'strict yaw requires a turn-pocket planner'
            )

        # ---- 内部状态初始化 ----
        self._callback_group = ReentrantCallbackGroup()      # 可重入回调组：允许并发执行
        self._floor_state: Optional[FloorState] = None       # 最近一帧楼层状态
        self._odometry: Optional[Odometry] = None            # 最近一帧里程计
        self._floor_hold = False                             # 楼层切换保持标志（外部开关）
        # Safety profiles fail closed until their online guard publishes
        # CLEAR. Native SCAN has no such node and is explicitly permitted by
        # the launch profile instead of waiting for a nonexistent publisher.
        # （安全档位在在线守卫发布 CLEAR 前默认"关闭即禁行"；原生 SCAN 档没有
        #   该守卫节点，由 launch 档显式放行，而不是等待一个不存在的发布者。）
        self._collision_stop = self._collision_guard_required   # 初始停车标志 = 是否要求守卫
        self._collision_diagnostic = (
            '' if self._collision_guard_required else 'CLEAR'   # 不要求守卫时直接视为 CLEAR
        )
        self._plan_update_count = 0                          # 已发布目标计数（反馈用）
        self._active_lock = threading.Lock()                 # 保护 _active 标志
        self._active = False                                 # 是否有 action 正在执行
        self._manual_lock = threading.Lock()                 # 保护手点导航状态
        self._manual_serial = 0                              # 手点导航的序号（防止过期回调）
        self._manual_target: Optional[PoseStamped] = None    # 当前手点目标
        self._manual_phase = 'IDLE'                          # 手点导航阶段
        self._manual_started_at = 0.0                        # 手点导航开始时刻
        self._manual_rearm_deadline = 0.0                    # 守卫重就绪等待截止时刻
        self._manual_replan_attempts = 0                     # 手点导航重规划次数
        qos = _latched_qos()

        # ---- 发布器：目标、导航状态、手点状态 ----
        self._goal_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('direct_goal_topic').value),
            10,
        )
        self._state_publisher = self.create_publisher(
            NavigationState, '/m20/navigation/state', qos
        )
        self._manual_state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('manual_state_topic').value),
            qos,
        )
        # ---- 订阅器：楼层状态 / 里程计 / 保持开关 / 碰撞守卫 ----
        self.create_subscription(
            FloorState,
            '/m20/map/state',
            self._floor_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
            callback_group=self._callback_group,
        )
        if self._manual_goal_monitor_enabled:
            # 可选：接管 RViz 手点目标
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter('manual_goal_topic').value),
                self._manual_goal_callback,
                10,
                callback_group=self._callback_group,
            )
        self.create_subscription(
            Bool,
            '/m20/control/floor_switch_hold',
            self._hold_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('collision_stop_topic').value),
            self._collision_stop_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            str(
                self.get_parameter(
                    'collision_diagnostic_topic'
                ).value
            ),
            self._collision_diagnostic_callback,
            qos,
            callback_group=self._callback_group,
        )
        # ---- SCAN 复位服务客户端 ----
        self._scan_reset_client = self.create_client(
            ResetNavigation,
            str(self.get_parameter('scan_reset_service').value),
            callback_group=self._callback_group,
        )
        # ---- 类型化 action 服务器 ----
        self._action_server = ActionServer(
            self,
            NavigateFloor,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,      # 目标执行回调
            goal_callback=self._goal_callback,   # 目标接收判定回调
            cancel_callback=self._cancel_callback,  # 取消响应回调
            callback_group=self._callback_group,
        )
        if self._manual_goal_monitor_enabled:
            # 手点导航的周期推进定时器（20 Hz）
            self.create_timer(
                0.05,
                self._manual_tick,
                callback_group=self._callback_group,
            )
        # 启动时发布初始状态
        self._publish_state('', '', 0, 'IDLE', False, math.inf, 'ready')
        self._publish_manual_state('IDLE', 'ready')
        self.get_logger().info(
            'typed navigation gateway ready: mode=native_scan; '
            f'managed RViz goals={self._manual_goal_monitor_enabled}; '
            f'collision_guard_required={self._collision_guard_required}; '
            f'terminal_orientation={self._terminal_orientation_mode}'
        )

    # ---- 各话题回调：仅缓存最新数据 ----
    def _floor_callback(self, message: FloorState) -> None:
        self._floor_state = message              # 缓存楼层状态

    def _odom_callback(self, message: Odometry) -> None:
        self._odometry = message                 # 缓存里程计

    def _hold_callback(self, message: Bool) -> None:
        self._floor_hold = bool(message.data)    # 缓存楼层切换保持开关

    def _collision_stop_callback(self, message: Bool) -> None:
        self._collision_stop = bool(message.data)   # 缓存碰撞停车标志

    def _collision_diagnostic_callback(self, message: String) -> None:
        self._collision_diagnostic = str(message.data)  # 缓存碰撞守卫诊断

    def _diagnostic_requires_replan(self) -> bool:
        """仅在一次碰撞恢复过程结束后返回 True（需要重规划）。"""
        return bool(
            self._collision_guard_required                       # 只有要求守卫的档位才考虑
            and recovery_replan_required(self._collision_diagnostic)   # 诊断命中恢复前缀
        )

    def _guard_is_clear(self) -> bool:
        """同时要求两个守卫输出，过期/缺失的点云必须 fail-closed。"""
        return guard_allows_motion(
            self._collision_guard_required,
            self._collision_stop,
            self._collision_diagnostic,
        )

    def _publish_goal(self, target: PoseStamped) -> None:
        """把一份"不做任何修改"的位置目标发布给原生 SCAN。"""
        target.header.stamp = self.get_clock().now().to_msg()   # 刷新时间戳（避免过期 TF）
        target.header.frame_id = target.header.frame_id or 'map'  # frame 为空则补 map
        self._plan_update_count += 1
        self._goal_publisher.publish(target)

    def _publish_manual_state(self, state: str, detail: str = '') -> None:
        """发布受管 RViz 自由导航的状态（latched 话题）。"""
        text = state if not detail else f'{state}; {detail}'
        self._manual_state_publisher.publish(String(data=text))
        self.get_logger().info(f'managed RViz navigation: {text}')

    def _clear_manual_locked(self) -> None:
        """在调用方持有 manual 锁的前提下清空手点导航状态。"""
        self._manual_target = None
        self._manual_phase = 'IDLE'
        self._manual_started_at = 0.0
        self._manual_rearm_deadline = 0.0
        self._manual_replan_attempts = 0

    def _manual_goal_callback(self, message: PoseStamped) -> None:
        """收到 RViz 手点目标：先复位 SCAN，守卫 CLEAR 后再转发。"""
        point = message.pose.position
        # 校验目标坐标有限（拒绝 NaN/Inf）
        if not all(math.isfinite(value) for value in (point.x, point.y)):
            self._publish_manual_state(
                'REJECTED_INVALID_GOAL', 'target is not finite'
            )
            return
        # 类型化 action 正在执行时拒绝手点目标（互斥）
        with self._active_lock:
            if self._active:
                self._publish_manual_state(
                    'REJECTED_ACTION_ACTIVE',
                    'typed navigation owns the planner',
                )
                return
        floor = self._floor_state
        # 前置条件：楼层就绪、有里程计、无保持开关
        if (
            floor is None
            or not floor.ready
            or self._odometry is None
            or self._floor_hold
        ):
            self._publish_manual_state(
                'REJECTED_NOT_READY',
                'map, odometry, or floor-switch hold is not ready',
            )
            return

        target = deepcopy(message)                       # 深拷贝，避免改到原始消息
        target.header.frame_id = target.header.frame_id or 'map'
        with self._manual_lock:
            self._manual_serial += 1                     # 递增序号：使旧回调失效
            serial = self._manual_serial
            self._manual_target = target
            self._manual_phase = 'RESETTING_NATIVE'      # 阶段：正在复位 SCAN
            self._manual_started_at = time.monotonic()
            self._manual_replan_attempts = 0
        self._publish_manual_state(
            'RESETTING_NATIVE',
            f'target=({point.x:.2f},{point.y:.2f})',
        )
        # 调用 SCAN 复位服务（同步等待）
        reset_ok = self._reset_navigation(
            floor.floor_id, floor.generation
        )
        with self._manual_lock:
            if serial != self._manual_serial:            # 期间有新目标：放弃本次
                return
            if not reset_ok:
                self._clear_manual_locked()
                failed = True
            else:
                # 复位成功：等待在线守卫重新发布 CLEAR
                self._manual_phase = 'WAITING_FOR_RECOVERY_REARM'
                self._manual_rearm_deadline = (
                    time.monotonic()
                    + self._collision_replan_rearm_timeout
                )
                failed = False
        if failed:
            self._publish_manual_state(
                'RESET_FAILED', 'could not prepare native SCAN'
            )
        else:
            self._publish_manual_state(
                'WAITING_FOR_RECOVERY_REARM',
                'waiting for online point-cloud guard CLEAR',
            )

    def _finish_manual(
        self,
        serial: int,
        state: str,
        detail: str,
        *,
        reset: bool = True,
    ) -> None:
        """停止一个受管手点目标并发布终态（默认同时复位 SCAN）。"""
        with self._manual_lock:
            # 序号不匹配或已无目标：忽略过期回调
            if (
                serial != self._manual_serial
                or self._manual_target is None
            ):
                return
            self._manual_phase = 'FINAL_RESETTING'       # 阶段：收尾复位
        floor = self._floor_state
        reset_ok = True
        if reset and floor is not None:
            reset_ok = self._reset_navigation(
                floor.floor_id, floor.generation
            )
        with self._manual_lock:
            if serial != self._manual_serial:
                return
            self._clear_manual_locked()                  # 清空手点状态
        suffix = detail
        if reset and not reset_ok:
            suffix += '; navigation reset confirmation failed'
        self._publish_manual_state(state, suffix)

    def _manual_tick(self) -> None:
        """推进受管 RViz 导航的非阻塞状态机（定时器回调）。"""
        # 快照当前手点状态（锁内读取，锁外处理）
        with self._manual_lock:
            target = self._manual_target
            if target is None:
                return
            serial = self._manual_serial
            phase = self._manual_phase
            started_at = self._manual_started_at
            rearm_deadline = self._manual_rearm_deadline
            attempts = self._manual_replan_attempts
        # 复位类阶段在 _manual_goal_callback / _finish_manual 中同步完成，跳过
        if phase in {
            'RESETTING_NATIVE',
            'RESETTING_RECOVERY',
            'FINAL_RESETTING',
        }:
            return

        floor = self._floor_state
        # 运行中前置条件被破坏（楼层/里程计/保持开关变化）则中止
        if (
            floor is None
            or not floor.ready
            or self._floor_hold
            or self._odometry is None
        ):
            self._finish_manual(
                serial,
                'ABORTED_FLOOR_STATE',
                'active map, odometry, or hold changed',
            )
            return
        now = time.monotonic()
        # 超时判定
        if now - started_at > self._manual_timeout:
            self._finish_manual(
                serial,
                'TIMEOUT',
                f'exceeded {self._manual_timeout:.1f}s',
            )
            return

        # 阶段：等待碰撞守卫重新 CLEAR（复位后）
        if phase == 'WAITING_FOR_RECOVERY_REARM':
            if self._guard_is_clear():               # 守卫已放行：转发目标
                with self._manual_lock:
                    if serial != self._manual_serial:
                        return
                    self._manual_phase = 'TRACKING_NATIVE'
                self._publish_goal(target)
                self._publish_manual_state(
                    'TRACKING_NATIVE', 'goal relayed unchanged to SCAN'
                )
                return
            if now >= rearm_deadline:                # 等待超时：终止
                self._finish_manual(
                    serial,
                    'COLLISION_REARM_TIMEOUT',
                    'online point-cloud guard did not return CLEAR',
                )
            return

        # 碰撞恢复结束后需要重规划（有界次数）
        if (
            self._collision_replan_enabled
            and self._diagnostic_requires_replan()
        ):
            if not collision_replan_available(
                attempts, self._collision_replan_max_attempts
            ):
                # 重规划预算耗尽：终止
                self._finish_manual(
                    serial,
                    'COLLISION_REPLAN_EXHAUSTED',
                    'bounded native SCAN replan attempts exhausted',
                )
                return
            with self._manual_lock:
                if serial != self._manual_serial:
                    return
                self._manual_phase = 'RESETTING_RECOVERY'   # 阶段：恢复后复位
                self._manual_replan_attempts += 1
                attempt = self._manual_replan_attempts
            self._publish_manual_state(
                'RESETTING_RECOVERY',
                f'attempt={attempt}/{self._collision_replan_max_attempts}',
            )
            reset_ok = self._reset_navigation(
                floor.floor_id, floor.generation
            )
            with self._manual_lock:
                if serial != self._manual_serial:
                    return
                if not reset_ok:
                    self._clear_manual_locked()
                    failed = True
                else:
                    self._manual_phase = 'WAITING_FOR_RECOVERY_REARM'
                    self._manual_rearm_deadline = (
                        time.monotonic()
                        + self._collision_replan_rearm_timeout
                    )
                    failed = False
            if failed:
                self._publish_manual_state(
                    'RESET_FAILED', 'collision-triggered SCAN reset failed'
                )
            else:
                self._publish_manual_state(
                    'WAITING_FOR_RECOVERY_REARM',
                    'old trajectory stopped before native SCAN replan',
                )
            return

        # 常规阶段：检查是否已到点
        distance = self._distance(target)
        if distance <= self._position_tolerance:
            self._finish_manual(
                serial,
                'SUCCEEDED',
                f'final_distance={distance:.3f}m',
            )

    def _goal_callback(self, goal) -> GoalResponse:
        """目标接收判定：校验字段、检查互斥，决定 ACCEPT/REJECT。"""
        pose = goal.target_pose.pose.position
        # 字段校验：goal_id / floor_id 非空，目标坐标有限
        if (
            not goal.goal_id.strip()
            or not goal.floor_id.strip()
            or not all(math.isfinite(value) for value in (pose.x, pose.y))
        ):
            return GoalResponse.REJECT
        with self._active_lock:
            # 已有 action 正在执行：拒绝新目标（串行化）
            if self._active:
                return GoalResponse.REJECT
            with self._manual_lock:
                # 手点导航进行中：拒绝（互斥）
                if self._manual_target is not None:
                    return GoalResponse.REJECT
            self._active = True        # 占用执行权
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT      # 无条件接受取消请求

    def _distance(self, target: PoseStamped) -> float:
        """计算机器人当前位置到目标位置的平面距离（米）。"""
        if self._odometry is None:
            return math.inf                # 无里程计：视为无穷远
        position = self._odometry.pose.pose.position
        return math.hypot(
            position.x - target.pose.position.x,
            position.y - target.pose.position.y,
        )

    def _publish_state(
        self,
        goal_id: str,
        floor_id: str,
        generation: int,
        state_name: str,
        active: bool,
        distance: float,
        message: str,
    ) -> None:
        """发布 NavigationState 导航状态消息。"""
        state = NavigationState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.goal_id = goal_id
        state.floor_id = floor_id
        state.map_generation = generation
        state.state = state_name
        state.active = active
        state.distance_remaining = (
            float(distance) if math.isfinite(distance) else -1.0   # 无效距离用 -1
        )
        state.message = message
        self._state_publisher.publish(state)

    def _feedback(
        self,
        goal_handle,
        state_name: str,
        distance: float,
        message: str,
    ) -> None:
        """发布 action 反馈并同步发布导航状态。"""
        feedback = NavigateFloor.Feedback()
        feedback.state = state_name
        feedback.distance_remaining = (
            float(distance) if math.isfinite(distance) else -1.0
        )
        # The interface field name is frozen for compatibility. In the
        # native-only implementation it counts SCAN goal publications.
        # （接口字段名因兼容性冻结；在纯原生实现里它统计 SCAN 目标发布次数。）
        feedback.route_update_count = self._plan_update_count
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        request = goal_handle.request
        self._publish_state(
            request.goal_id,
            request.floor_id,
            request.map_generation,
            state_name,
            True,
            distance,
            message,
        )

    def _result(
        self,
        goal,
        success: bool,
        error_code: int,
        message: str,
    ):
        """构造 action 结果对象。"""
        result = NavigateFloor.Result()
        result.success = success
        result.goal_id = goal.goal_id
        state = self._floor_state
        result.floor_id = state.floor_id if state is not None else ''
        result.map_generation = state.generation if state is not None else 0
        distance = self._distance(goal.target_pose)
        result.final_distance = (
            float(distance) if math.isfinite(distance) else -1.0
        )
        result.error_code = error_code
        result.message = message
        return result

    @staticmethod
    def _call_service(client, request, timeout: float):
        """带超时的同步服务调用（等待服务上线 -> 异步调用 -> 等待结果）。"""
        deadline = time.monotonic() + timeout
        # 等待服务上线
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                return None            # 超时：返回 None
            time.sleep(0.02)
        future = client.call_async(request)
        # 等待结果或超时
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()        # 超时：取消调用
                return None
            time.sleep(0.02)
        return future.result()

    def _reset_navigation(self, floor_id: str, generation: int) -> bool:
        """只复位原生 SCAN 状态机（带楼层/代次信息）。"""
        request = ResetNavigation.Request()
        request.floor_id = floor_id
        request.generation = generation
        scan = self._call_service(
            self._scan_reset_client,
            request,
            self._navigation_reset_timeout,
        )
        # 服务可用且返回 success 才视为复位成功
        return bool(scan is not None and scan.success)

    def _finish_state(self, goal, state_name: str, message: str) -> None:
        """发布目标结束时的最终导航状态（active=False）。"""
        self._publish_state(
            goal.goal_id,
            goal.floor_id,
            goal.map_generation,
            state_name,
            False,
            self._distance(goal.target_pose),
            message,
        )

    def _floor_matches(self, goal) -> bool:
        """目标要求的楼层与代次是否与当前激活地图一致。"""
        floor = self._floor_state
        return bool(
            floor is not None
            and floor.ready
            and floor.floor_id == goal.floor_id
            and floor.generation == goal.map_generation
        )

    def _wait_for_guard_clear(self, goal_handle, deadline: float) -> bool:
        """运动前等待一份新鲜的在线点云安全判定（CLEAR）。"""
        while time.monotonic() < deadline:
            # 取消请求或楼层保持：立即失败
            if goal_handle.is_cancel_requested or self._floor_hold:
                return False
            # 执行期间楼层/代次变化：失败
            if not self._floor_matches(goal_handle.request):
                return False
            if self._guard_is_clear():          # 守卫 CLEAR：放行
                return True
            self._feedback(
                goal_handle,
                'WAITING_FOR_RECOVERY_REARM',
                self._distance(goal_handle.request.target_pose),
                'waiting for online point-cloud guard CLEAR',
            )
            time.sleep(0.05)                    # 50ms 轮询
        return False                            # 超时

    def _execute(self, goal_handle):
        """类型化 action 的执行回调：完整导航流程。"""
        goal = goal_handle.request
        try:
            # ---- 前置条件检查：地图与里程计 ----
            floor = self._floor_state
            if floor is None or not floor.ready or self._odometry is None:
                goal_handle.abort()
                self._finish_state(
                    goal, 'MAP_NOT_READY', 'map or odometry unavailable'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_MAP_NOT_READY,
                    'map or odometry is not ready',
                )
            # ---- 楼层/代次匹配检查 ----
            if not self._floor_matches(goal):
                goal_handle.abort()
                self._finish_state(
                    goal, 'FLOOR_MISMATCH', 'goal does not match active map'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_FLOOR_MISMATCH,
                    'goal floor/generation does not match active map',
                )

            # DDS may deliver a new-floor Action before this subscriber sees
            # the preceding hold=False sample. Absorb only that bounded race.
            # （DDS 可能在新楼层 action 到达时本订阅器还没收到 hold=False，
            #   这里只吸收这一有界竞态：等待保持释放。）
            release_deadline = (
                time.monotonic() + self._floor_hold_release_timeout
            )
            while self._floor_hold and time.monotonic() < release_deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._finish_state(
                        goal,
                        'CANCELLED',
                        'cancelled while waiting for floor release',
                    )
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_CANCELLED,
                        'cancelled while waiting for floor release',
                    )
                if not self._floor_matches(goal):
                    break                  # 楼层已切换：跳出等待
                self._feedback(
                    goal_handle,
                    'WAITING_FOR_FLOOR_RELEASE',
                    self._distance(goal.target_pose),
                    'waiting for navigation gateway hold release',
                )
                time.sleep(0.02)
            if self._floor_hold:
                # 等待超时仍保持：楼层切换保持生效
                goal_handle.abort()
                self._finish_state(
                    goal, 'FLOOR_SWITCH_HOLD', 'floor switch is active'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_FLOOR_SWITCH,
                    'floor switch hold is active',
                )

            # ---- 起点已在目标容差内：直接成功 ----
            distance = self._distance(goal.target_pose)
            if distance <= self._position_tolerance:
                goal_handle.succeed()
                self._finish_state(goal, 'SUCCEEDED', 'already at goal')
                return self._result(
                    goal,
                    True,
                    NavigateFloor.Result.ERROR_NONE,
                    'already within goal tolerance',
                )

            # ---- 确定本次导航超时（目标指定或默认值）----
            timeout = (
                float(goal.timeout_sec)
                if goal.timeout_sec > 0.0
                else self._default_timeout
            )
            deadline = time.monotonic() + timeout
            # ---- 导航前先复位 SCAN 状态机 ----
            if not self._reset_navigation(
                goal.floor_id, goal.map_generation
            ):
                goal_handle.abort()
                self._finish_state(
                    goal, 'RESET_FAILED', 'native SCAN reset failed'
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_RESET_FAILED,
                    'native SCAN reset failed',
                )
            # ---- 等碰撞守卫重新 CLEAR（带截止时间）----
            rearm_deadline = min(
                deadline,
                time.monotonic() + self._collision_replan_rearm_timeout,
            )
            if not self._wait_for_guard_clear(goal_handle, rearm_deadline):
                goal_handle.abort()
                self._finish_state(
                    goal,
                    'COLLISION_REARM_TIMEOUT',
                    'online point-cloud guard did not become clear',
                )
                return self._result(
                    goal,
                    False,
                    NavigateFloor.Result.ERROR_NO_ROUTE,
                    'online point-cloud guard did not become clear',
                )

            # ---- 发布目标给原生 SCAN ----
            target = deepcopy(goal.target_pose)
            self._publish_goal(target)
            self._feedback(
                goal_handle,
                'PLANNING',
                distance,
                'goal sent directly to native SCAN',
            )
            collision_replan_attempts = 0      # 本目标内重规划计数

            # ---- 主循环：跟踪直到到点/超时/取消/楼层变化 ----
            while time.monotonic() < deadline:
                # 取消请求处理
                if goal_handle.is_cancel_requested:
                    reset_ok = self._reset_navigation(
                        goal.floor_id, goal.map_generation
                    )
                    goal_handle.canceled()
                    message = (
                        'navigation cancelled and reset'
                        if reset_ok
                        else 'navigation cancelled; reset confirmation failed'
                    )
                    self._finish_state(goal, 'CANCELLED', message)
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_CANCELLED
                        if reset_ok
                        else NavigateFloor.Result.ERROR_RESET_FAILED,
                        message,
                    )

                # 执行中楼层变化或保持开关生效：中止
                if not self._floor_matches(goal) or self._floor_hold:
                    goal_handle.abort()
                    self._finish_state(
                        goal,
                        'FLOOR_SWITCH',
                        'active floor changed during navigation',
                    )
                    return self._result(
                        goal,
                        False,
                        NavigateFloor.Result.ERROR_FLOOR_SWITCH,
                        'active floor changed or entered hold',
                    )

                # ---- 碰撞恢复结束 -> 有界重规划 ----
                if (
                    self._collision_replan_enabled
                    and self._diagnostic_requires_replan()
                ):
                    if not collision_replan_available(
                        collision_replan_attempts,
                        self._collision_replan_max_attempts,
                    ):
                        # 重规划预算耗尽：复位并中止
                        self._reset_navigation(
                            goal.floor_id, goal.map_generation
                        )
                        goal_handle.abort()
                        message = (
                            'bounded collision recovery and native SCAN '
                            'replan budget exhausted'
                        )
                        self._finish_state(
                            goal, 'COLLISION_REPLAN_EXHAUSTED', message
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_NO_ROUTE,
                            message,
                        )

                    collision_replan_attempts += 1
                    self._feedback(
                        goal_handle,
                        'RECOVERY_REPLAN',
                        self._distance(goal.target_pose),
                        'collision recovery ended; resetting native SCAN '
                        f'({collision_replan_attempts}/'
                        f'{self._collision_replan_max_attempts})',
                    )
                    # 复位 SCAN 以停止旧轨迹
                    if not self._reset_navigation(
                        goal.floor_id, goal.map_generation
                    ):
                        goal_handle.abort()
                        message = 'collision-triggered SCAN reset failed'
                        self._finish_state(
                            goal, 'RESET_FAILED', message
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_RESET_FAILED,
                            message,
                        )
                    # 等守卫重新 CLEAR 后再重发目标
                    rearm_deadline = min(
                        deadline,
                        time.monotonic()
                        + self._collision_replan_rearm_timeout,
                    )
                    if not self._wait_for_guard_clear(
                        goal_handle, rearm_deadline
                    ):
                        goal_handle.abort()
                        message = (
                            'online point-cloud guard did not rearm before '
                            'native SCAN replan'
                        )
                        self._finish_state(
                            goal, 'COLLISION_REARM_TIMEOUT', message
                        )
                        return self._result(
                            goal,
                            False,
                            NavigateFloor.Result.ERROR_NO_ROUTE,
                            message,
                        )
                    self._publish_goal(target)     # 重新发布同一目标
                    self._feedback(
                        goal_handle,
                        'REPLANNING',
                        self._distance(goal.target_pose),
                        'goal republished to native SCAN after bounded '
                        'recovery',
                    )
                    continue                        # 继续跟踪循环

                # ---- 到点判定 ----
                distance = self._distance(goal.target_pose)
                if distance <= self._position_tolerance:
                    goal_handle.succeed()
                    self._finish_state(
                        goal, 'SUCCEEDED', 'native SCAN goal reached'
                    )
                    return self._result(
                        goal,
                        True,
                        NavigateFloor.Result.ERROR_NONE,
                        'native SCAN goal reached',
                    )
                self._feedback(
                    goal_handle,
                    'TRACKING',
                    distance,
                    'native SCAN trajectory tracking',
                )
                time.sleep(self._feedback_period)   # 按反馈周期轮询

            # ---- 超时：复位 SCAN 并中止 ----
            reset_ok = self._reset_navigation(
                goal.floor_id, goal.map_generation
            )
            goal_handle.abort()
            message = (
                'navigation timed out and reset'
                if reset_ok
                else 'navigation timed out; reset confirmation failed'
            )
            self._finish_state(goal, 'TIMEOUT', message)
            return self._result(
                goal,
                False,
                NavigateFloor.Result.ERROR_TIMEOUT
                if reset_ok
                else NavigateFloor.Result.ERROR_RESET_FAILED,
                message,
            )
        except Exception as error:
            # 兜底异常处理：记录日志并中止
            self.get_logger().error(f'navigation gateway failed: {error}')
            goal_handle.abort()
            self._finish_state(goal, 'INTERNAL_ERROR', str(error))
            return self._result(
                goal,
                False,
                NavigateFloor.Result.ERROR_INTERNAL,
                str(error),
            )
        finally:
            # 无论结果如何，释放执行权
            with self._active_lock:
                self._active = False

    def destroy_node(self) -> None:
        """先销毁 action 服务器，再销毁节点本体。"""
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """以并发服务回调运行原生 SCAN 网关。"""
    rclpy.init(args=args)
    node = NavigationGateway()
    executor = MultiThreadedExecutor(num_threads=4)   # 4 线程：服务/订阅回调并发
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass                         # Ctrl+C 正常退出
    finally:
        executor.shutdown()
        node.destroy_node()
        # A launch SIGINT can shut the default context before this finally
        # block runs. Humble raises on a second shutdown; Foxy deployments
        # use the same guard for clean cross-distribution teardown.
        # （launch 的 SIGINT 可能先关闭默认上下文再执行本 finally 块；
        #   Humble 对二次 shutdown 会抛异常，Foxy 部署用同一守卫实现跨发行版
        #   的干净收尾。）
        if rclpy.ok():
            rclpy.shutdown()
