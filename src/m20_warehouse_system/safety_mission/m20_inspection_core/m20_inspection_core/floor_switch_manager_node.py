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
# 【文件职责】floor_switch_manager_node.py —— 楼层切换管理节点（ROS2 Node）
# 本节点以"故障闭合"（fail-closed）的方式协调一次模拟电梯转移，核心职责：
#   1) 提供 SwitchFloor Action 服务：源楼层确认 -> 断言安全保持 -> 确认停车
#      -> 重置旧楼层导航 -> 执行传输适配器（定时保持 / 外部动作）-> 提交目标
#      楼层地图 -> 位姿交接 -> 重置新楼层导航 -> 等待感知就绪 -> 释放保持；
#   2) 在转移期间持续发布安全保持（/m20/control/floor_switch_hold），任何
#      取消/异常都保持或恢复断言，绝不自动放行；
#   3) 支持已提交目标的恢复（崩溃/重启后重试位姿验证、重置与感知就绪）；
#   4) 多线程执行器并发处理订阅回调与服务/Action 调用。
# ============================================================================

"""Fail-closed action server coordinating a simulated elevator transfer."""

# ------------------------- 标准库导入 -------------------------
import math                # 数学库：航向角（yaw）计算
from pathlib import Path   # 路径对象：定位配置文件
import threading           # 线程库：目标锁与外部客户端锁
import time                # 时间库：单调时钟超时等待
from typing import Optional  # 类型提示：Optional 可空值

from geometry_msgs.msg import Twist   # Twist 消息：安全指令（停车确认）
from m20_warehouse_interfaces.action import ExecuteFloorTransfer, SwitchFloor  # 自定义 Action：外部转移执行 / 楼层切换
from m20_warehouse_interfaces.msg import FloorState, LocalSensingState  # 楼层状态 / 局部感知状态消息
from m20_warehouse_interfaces.srv import (  # 自定义服务接口
    ResetNavigation,   # 重置导航服务
    SetSimulationPose, # 设置仿真位姿服务
    SwitchMap,         # 切换/提交地图服务
)
from nav_msgs.msg import Odometry     # 里程计消息（停车判定与位姿）
import rclpy                          # ROS2 Python 客户端库
from rclpy.action import (            # ROS2 Action 客户端/服务端组件
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup  # 可重入回调组（并发回调）
from rclpy.executors import MultiThreadedExecutor         # 多线程执行器
from rclpy.node import Node           # ROS2 节点基类
from rclpy.qos import (               # QoS 策略（锁存发布）
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String # 标准消息：保持标志 / 状态字符串
import yaml                           # YAML 解析：加载配置

from .floor_switch_policy import pose_error, resolve_transition  # 楼层切换策略纯函数


class _SwitchCancelled(RuntimeError):
    """Internal signal used to unwind an action while retaining safety hold."""
    # 【中文】内部信号异常：用于展开（取消）动作流程，同时保留安全保持


def _latched_qos() -> QoSProfile:
    # 【功能】构造锁存 QoS（深度 1、可靠、瞬态本地），供状态/保持话题使用
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _yaw_from_odometry(message: Odometry) -> float:
    # 【功能】从里程计四元数提取航向角 yaw（atan2 形式）
    # 【参数】message - 里程计消息；【返回】航向角（弧度）
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


class FloorSwitchManager(Node):
    """Coordinate stop, reset, map commit, relocation, and sensing readiness."""
    # 【中文】楼层切换管理节点：协调停车、重置、地图提交、位姿交接与感知就绪

    def __init__(self) -> None:
        super().__init__('m20_floor_switch_manager')
        # ------------------------- 参数声明 -------------------------
        self.declare_parameter('config_path', '')  # 配置文件路径
        self.declare_parameter('action_name', '/m20/floor_switch')  # SwitchFloor Action 名称
        self.declare_parameter(
            'navigation_reset_service', '/m20/navigation/reset'  # 导航重置服务名
        )
        self.declare_parameter('map_switch_service', '/m20/map/switch')  # 地图切换服务名
        self.declare_parameter('set_pose_service', '/m20/sim/set_pose')  # 仿真位姿服务名
        self.declare_parameter('body_pose_topic', '/m20/sim/body_pose')  # 机体位姿话题
        config_path = Path(str(self.get_parameter('config_path').value))
        if not config_path.is_file():
            raise ValueError(f'config_path is not a file: {config_path}')
        with config_path.open('r', encoding='utf-8') as stream:
            self._config = yaml.safe_load(stream)  # 加载仓库系统配置

        # ------------------------- 事务超时与阈值参数 -------------------------
        transaction = self._config['map_switch_transaction']
        self._linear_stop_threshold = float(  # 停车判定线速度阈值（米/秒）
            transaction['stop_linear_threshold']
        )
        self._angular_stop_threshold = float(  # 停车判定角速度阈值（弧度/秒）
            transaction['stop_angular_threshold']
        )
        self._reset_timeout = float(  # 导航重置服务超时（秒）
            transaction['reset_navigation_timeout_sec']
        )
        self._map_timeout = float(transaction['load_map_timeout_sec'])  # 地图提交超时
        self._pose_timeout = float(transaction['relocate_timeout_sec'])  # 位姿交接超时
        self._readiness_timeout = float(  # 感知就绪等待超时
            transaction['readiness_timeout_sec']
        )
        self._minimum_fresh_clouds = int(  # 要求的新鲜点云最小数量
            transaction['minimum_fresh_local_cloud_count']
        )
        # ------------------------- 运行时状态 -------------------------
        self._callback_group = ReentrantCallbackGroup()  # 可重入回调组
        self._floor_state: Optional[FloorState] = None        # 最近楼层状态
        self._sensing_state: Optional[LocalSensingState] = None  # 最近感知状态
        self._odometry: Optional[Odometry] = None             # 最近里程计
        self._safe_command: Optional[Twist] = None            # 最近安全指令（停车确认）
        self._goal_lock = threading.Lock()    # 目标接受互斥锁
        self._switch_active = False           # 是否有切换动作进行中
        self._hold_asserted = False           # 安全保持当前是否已断言
        self._external_client_lock = threading.Lock()  # 外部客户端字典锁
        self._external_clients = {}           # 按 Action 名缓存的外部客户端
        qos = _latched_qos()

        # ------------------------- 发布器/订阅器 -------------------------
        self._hold_publisher = self.create_publisher(  # 安全保持发布器（锁存）
            Bool, '/m20/control/floor_switch_hold', qos
        )
        self._state_publisher = self.create_publisher(  # 切换状态发布器（锁存）
            String, '/m20/floor_switch/state', qos
        )
        self.create_subscription(  # 楼层状态订阅
            FloorState,
            '/m20/map/state',
            self._floor_state_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(  # 局部感知状态订阅
            LocalSensingState,
            '/m20/sensing/state',
            self._sensing_state_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(  # 里程计订阅
            Odometry,
            str(self.get_parameter('body_pose_topic').value),
            self._odometry_callback,
            20,
            callback_group=self._callback_group,
        )
        self.create_subscription(  # 安全指令订阅（停车确认）
            Twist,
            '/m20/control/cmd_vel_safe',
            self._safe_command_callback,
            20,
            callback_group=self._callback_group,
        )
        # ------------------------- 服务客户端 -------------------------
        self._reset_client = self.create_client(  # 导航重置服务客户端
            ResetNavigation,
            str(self.get_parameter('navigation_reset_service').value),
            callback_group=self._callback_group,
        )
        self._map_client = self.create_client(  # 地图切换服务客户端
            SwitchMap,
            str(self.get_parameter('map_switch_service').value),
            callback_group=self._callback_group,
        )
        self._set_pose_service_name = str(  # 仿真位姿服务名（延迟创建客户端）
            self.get_parameter('set_pose_service').value
        )
        self._pose_client = None
        # ------------------------- Action 服务端 -------------------------
        self._action_server = ActionServer(  # SwitchFloor Action 服务端
            self,
            SwitchFloor,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._publish_hold(False)  # 初始：不保持
        self._state_publisher.publish(String(data='READY_FOR_REQUEST'))
        switch_policy = self._config.get('floor_switch', {})  # 楼层切换策略配置
        transfer_mode = str(  # 传输适配器模式（默认 timed_hold）
            switch_policy.get('transfer_adapter', 'timed_hold')
        )
        pose_handoff = str(  # 位姿交接模式（默认按仿真配置推导）
            switch_policy.get(
                'pose_handoff',
                'set_simulation_pose'
                if self._config.get('simulation', {}).get(
                    'teleport_on_floor_switch', False
                )
                else 'preserve',
            )
        )
        self.get_logger().info(
            'floor switch action manager ready; '
            f'transfer_adapter={transfer_mode}, pose_handoff={pose_handoff}'
        )

    # ------------------------- 订阅回调 -------------------------
    def _floor_state_callback(self, message: FloorState) -> None:
        # 【回调】缓存最近楼层状态
        self._floor_state = message

    def _sensing_state_callback(self, message: LocalSensingState) -> None:
        # 【回调】缓存最近局部感知状态
        self._sensing_state = message

    def _odometry_callback(self, message: Odometry) -> None:
        # 【回调】缓存最近里程计
        self._odometry = message

    def _safe_command_callback(self, message: Twist) -> None:
        # 【回调】缓存最近安全指令（用于确认机器人是否真正停车）
        self._safe_command = message

    # ------------------------- Action 回调 -------------------------
    def _goal_callback(self, goal_request) -> GoalResponse:
        # 【回调】目标接受策略：源/目标楼层与电梯 id 均非空、且无活动切换才接受
        if not (
            goal_request.source_floor.strip()
            and goal_request.target_floor.strip()
            and goal_request.elevator_id.strip()
        ):
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._switch_active:
                return GoalResponse.REJECT
            self._switch_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        # 【回调】取消请求总是接受（取消语义在执行回调中处理）
        return CancelResponse.ACCEPT

    def _publish_hold(self, asserted: bool) -> None:
        # 【功能】发布安全保持标志（asserted=True 表示要求安全门停车）
        self._hold_asserted = asserted
        self._hold_publisher.publish(Bool(data=asserted))

    def _feedback(
        self,
        goal_handle,
        phase: str,
        message: str,
        generation: Optional[int] = None,
    ) -> None:
        # 【功能】发布切换阶段反馈（Action 反馈 + 状态话题），保持期间周期性重申保持
        # 【参数】goal_handle - 目标句柄；phase - 阶段名；message - 说明；
        #        generation - 地图代数（缺省取当前楼层状态）
        feedback = SwitchFloor.Feedback()
        feedback.phase = phase
        feedback.map_generation = (
            int(generation)
            if generation is not None
            else int(self._floor_state.generation)
            if self._floor_state is not None
            else 0
        )
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        self._state_publisher.publish(
            String(data=f'{phase}: {message}')
        )
        if self._hold_asserted:  # 保持期间周期性重申保持（防丢包）
            self._hold_publisher.publish(Bool(data=True))

    def _result(self, success: bool, error_code: int, message: str):
        # 【功能】构造 SwitchFloor 结果消息（含当前楼层与地图代数）
        result = SwitchFloor.Result()
        result.success = success
        result.active_floor = (
            self._floor_state.floor_id
            if self._floor_state is not None
            else ''
        )
        result.map_generation = (
            self._floor_state.generation
            if self._floor_state is not None
            else 0
        )
        result.error_code = error_code
        result.message = message
        return result

    @staticmethod
    def _check_cancel(goal_handle) -> None:
        # 【功能】检查取消请求：已请求则抛出 _SwitchCancelled（中断当前流程）
        if goal_handle.is_cancel_requested:
            raise _SwitchCancelled('floor switch cancelled by client')

    def _call_service(self, client, request, timeout: float):
        """Call a service while other executor threads continue callbacks."""
        # 【中文】调用服务（异步等待，带超时），期间其他执行器线程继续处理回调。
        # 【参数】client - 服务客户端；request - 请求；timeout - 超时（秒）
        # 【返回】服务响应或 None（超时/服务不可用）
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():  # 等待服务端就绪
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        future = client.call_async(request)
        while not future.done():  # 等待响应
            if time.monotonic() >= deadline:
                future.cancel()
                return None
            time.sleep(0.02)
        return future.result()

    def _wait_stopped(self, goal_handle, timeout: float) -> bool:
        # 【功能】等待机器人真正停车：同时检查里程计速度与安全指令速度
        # 【参数】goal_handle - 目标句柄；timeout - 超时（秒）
        # 【返回】True 表示已连续 3 次确认停车
        deadline = time.monotonic() + timeout
        consecutive = 0
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            odom = self._odometry
            command = self._safe_command
            if odom is not None and command is not None:
                twist = odom.twist.twist
                odom_linear = math.sqrt(  # 里程计线速度大小
                    twist.linear.x**2
                    + twist.linear.y**2
                    + twist.linear.z**2
                )
                command_linear = math.hypot(  # 安全指令线速度大小
                    command.linear.x, command.linear.y
                )
                stopped = (
                    odom_linear <= self._linear_stop_threshold
                    and abs(twist.angular.z)
                    <= self._angular_stop_threshold
                    and command_linear <= self._linear_stop_threshold
                    and abs(command.angular.z)
                    <= self._angular_stop_threshold
                )
                consecutive = consecutive + 1 if stopped else 0
                if consecutive >= 3:  # 连续 3 次确认才算停稳
                    return True
            time.sleep(0.05)
        return False

    def _delay_with_cancel(self, goal_handle, duration: float) -> None:
        # 【功能】可取消的延时等待（用于定时保持适配器）
        deadline = time.monotonic() + max(0.0, duration)
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            time.sleep(min(0.05, deadline - time.monotonic()))

    def _wait_sensing(
        self,
        goal_handle,
        floor_id: str,
        generation: int,
        minimum_count: int,
        timeout: float,
    ) -> bool:
        # 【功能】等待目标楼层感知就绪：楼层/代数匹配且新鲜点云数达标
        # 【返回】True 表示感知已就绪
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            state = self._sensing_state
            if (
                state is not None
                and state.ready
                and state.floor_id == floor_id
                and state.generation == generation
                and state.fresh_cloud_count >= minimum_count
            ):
                return True
            time.sleep(0.05)
        return False

    def _pose_at_target(self, plan):
        """Return whether current odometry satisfies the release contract."""
        # 【中文】判断当前里程计位姿是否满足释放契约（与目标释放位姿足够接近）。
        # 【参数】plan - 转移计划；【返回】(是否就绪, 平面距离, 航向误差)
        odom = self._odometry
        if odom is None:
            return False, float('inf'), float('inf')
        current_pose = (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            _yaw_from_odometry(odom),
        )
        distance, yaw_error = pose_error(current_pose, plan.target_release)
        return (
            distance <= plan.trigger_tolerance_xy
            and yaw_error <= plan.trigger_tolerance_yaw,
            distance,
            yaw_error,
        )

    def _establish_target_pose(self, goal_handle, plan, generation: int):
        """Apply the configured post-map pose handoff policy."""
        # 【中文】应用配置的"地图提交后位姿交接"策略。
        # 【参数】goal_handle - 目标句柄；plan - 转移计划；generation - 目标地图代数
        # 【返回】(是否成功, 说明消息)
        if plan.pose_handoff == 'none':  # 无交接：交给传输适配器处理
            return True, 'pose handoff delegated to the transfer adapter'

        if plan.pose_handoff == 'preserve':  # 保持位姿：验证当前位姿满足释放契约
            ready, distance, yaw_error = self._pose_at_target(plan)
            if not ready:
                return (
                    False,
                    f'preserved pose differs from target release by '
                    f'{distance:.3f}m/{yaw_error:.3f}rad',
                )
            return True, 'world pose preserved at the shared gateway'

        if plan.pose_handoff == 'wait_for_target':  # 等待目标位姿：轮询确认
            deadline = time.monotonic() + self._pose_timeout
            last_distance = float('inf')
            last_yaw_error = float('inf')
            while time.monotonic() < deadline:
                self._check_cancel(goal_handle)
                ready, last_distance, last_yaw_error = self._pose_at_target(
                    plan
                )
                if ready:
                    return True, 'target release pose confirmed'
                time.sleep(0.05)
            return (
                False,
                f'target pose was not confirmed; error '
                f'{last_distance:.3f}m/{last_yaw_error:.3f}rad',
            )

        if plan.pose_handoff != 'set_simulation_pose':  # 未知策略
            return False, f'unsupported pose handoff {plan.pose_handoff!r}'

        # set_simulation_pose：调用仿真位姿服务直接瞬移
        if self._pose_client is None:  # 延迟创建位姿服务客户端
            self._pose_client = self.create_client(
                SetSimulationPose,
                self._set_pose_service_name,
                callback_group=self._callback_group,
            )

        pose_request = SetSimulationPose.Request()
        (
            pose_request.x,
            pose_request.y,
            pose_request.yaw,
        ) = plan.target_release
        pose_request.floor_id = plan.target_floor
        pose_request.generation = generation
        pose_response = self._call_service(
            self._pose_client, pose_request, self._pose_timeout
        )
        if pose_response is None or not pose_response.success:
            return False, 'simulation pose handoff failed'
        return True, 'simulation pose handoff complete'

    def _external_client(self, action_name: str):
        """Create one reusable Action client per configured endpoint."""
        # 【中文】按配置的端点名创建可复用的 Action 客户端（每个端点只建一次）。
        # 【参数】action_name - 外部转移 Action 名称；【返回】ActionClient 实例
        with self._external_client_lock:
            client = self._external_clients.get(action_name)
            if client is None:
                client = ActionClient(
                    self,
                    ExecuteFloorTransfer,
                    action_name,
                    callback_group=self._callback_group,
                )
                self._external_clients[action_name] = client
            return client

    def _execute_external_transfer(self, goal_handle, plan, generation: int):
        # 【功能】执行外部传输适配器：调用 ExecuteFloorTransfer Action
        # 【参数】goal_handle - 目标句柄；plan - 转移计划；generation - 源地图代数
        # 【返回】(是否成功, 消息)
        client = self._external_client(plan.external_action_name)
        deadline = time.monotonic() + plan.transfer_timeout_sec
        while not client.server_is_ready():  # 等待外部服务端就绪
            self._check_cancel(goal_handle)
            if time.monotonic() >= deadline:
                return False, 'external transfer action is unavailable'
            time.sleep(0.05)

        request = ExecuteFloorTransfer.Goal()
        request.connector_id = plan.connector_id
        request.source_floor = plan.source_floor
        request.target_floor = plan.target_floor
        request.source_generation = generation

        def feedback_callback(message) -> None:  # 转发外部转移反馈
            feedback = message.feedback
            phase = str(feedback.phase).strip() or 'RUNNING'
            self._feedback(
                goal_handle,
                f'TRANSFER_{phase}',
                str(feedback.message),
                generation,
            )

        send_future = client.send_goal_async(  # 异步发送外部转移目标
            request, feedback_callback=feedback_callback
        )
        while not send_future.done():
            self._check_cancel(goal_handle)
            if time.monotonic() >= deadline:
                send_future.cancel()
                return False, 'external transfer goal timed out'
            time.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, 'external transfer goal was rejected'

        result_future = handle.get_result_async()
        while not result_future.done():  # 等待外部转移完成（响应取消）
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                self._check_cancel(goal_handle)
            if time.monotonic() >= deadline:
                handle.cancel_goal_async()
                return False, 'external transfer execution timed out'
            time.sleep(0.05)
        wrapped = result_future.result()
        if wrapped is None or not wrapped.result.success:
            message = (
                wrapped.result.message
                if wrapped is not None
                else 'external transfer returned no result'
            )
            return False, message
        return True, wrapped.result.message or 'external transfer complete'

    def _execute_transfer(self, goal_handle, plan, generation: int):
        """Run the selected transport adapter without touching map state."""
        # 【中文】运行选定的传输适配器（不触碰地图状态）。
        # 【参数】goal_handle - 目标句柄；plan - 转移计划；generation - 源地图代数
        # 【返回】(是否成功, 消息)
        if plan.transfer_adapter == 'timed_hold':  # 定时保持：等待设定时长
            self._delay_with_cancel(goal_handle, plan.transition_delay_sec)
            return True, (
                f'timed hold complete ({plan.transition_delay_sec:.2f}s)'
            )
        if plan.transfer_adapter == 'external_action':  # 外部动作：调用外部 Action
            return self._execute_external_transfer(
                goal_handle, plan, generation
            )
        return False, f'unsupported transfer adapter {plan.transfer_adapter!r}'

    def _execute(self, goal_handle):
        # 【主流程】SwitchFloor Action 执行回调：完整的楼层切换事务
        goal = goal_handle.request
        transaction_started = False
        try:
            try:
                plan = resolve_transition(  # 解析转移计划（配置校验）
                    self._config,
                    goal.elevator_id.strip(),
                    goal.source_floor.strip(),
                    goal.target_floor.strip(),
                )
            except (KeyError, TypeError, ValueError) as error:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_INVALID_REQUEST,
                    str(error),
                )

            state = self._floor_state
            odom = self._odometry
            recovering_committed_target = (  # 恢复已提交目标：保持已断言且目标楼层已就绪
                self._hold_asserted
                and state is not None
                and state.ready
                and state.floor_id == plan.target_floor
                and odom is not None
            )
            if recovering_committed_target:
                # ---------------- 已提交目标恢复分支 ----------------
                transaction_started = True
                generation = state.generation
                self._publish_hold(True)
                self._feedback(
                    goal_handle,
                    'RECOVERING_COMMITTED_TARGET',
                    'retrying target pose verification, reset, and sensing',
                    generation,
                )
                pose_ready, pose_message = self._establish_target_pose(  # 位姿验证
                    goal_handle, plan, generation
                )
                if not pose_ready:
                    goal_handle.abort()
                    return self._result(
                        False,
                        SwitchFloor.Result.ERROR_POSE_TRANSFER,
                        pose_message,
                    )

                reset = ResetNavigation.Request()  # 重置目标楼层导航
                reset.floor_id = plan.target_floor
                reset.generation = generation
                reset_response = self._call_service(
                    self._reset_client, reset, self._reset_timeout
                )
                if reset_response is None or not reset_response.success:
                    goal_handle.abort()
                    return self._result(
                        False,
                        SwitchFloor.Result.ERROR_NAVIGATION_RESET,
                        'target navigation recovery failed',
                    )

                baseline = 0  # 以当前新鲜点云数为基线
                sensing = self._sensing_state
                if (
                    sensing is not None
                    and sensing.floor_id == plan.target_floor
                    and sensing.generation == generation
                ):
                    baseline = sensing.fresh_cloud_count
                self._feedback(
                    goal_handle,
                    'WAITING_SENSING',
                    'waiting for post-recovery local clouds',
                    generation,
                )
                if not self._wait_sensing(  # 等待恢复后的感知就绪
                    goal_handle,
                    plan.target_floor,
                    generation,
                    baseline + self._minimum_fresh_clouds,
                    self._readiness_timeout,
                ):
                    goal_handle.abort()
                    return self._result(
                        False,
                        SwitchFloor.Result.ERROR_SENSING_TIMEOUT,
                        'target sensing recovery timed out',
                    )

                self._feedback(  # 恢复完成：释放安全保持
                    goal_handle,
                    'COMPLETE',
                    'committed target recovered; releasing safety hold',
                    generation,
                )
                self._publish_hold(False)
                goal_handle.succeed()
                return self._result(
                    True,
                    SwitchFloor.Result.ERROR_NONE,
                    f'recovered {plan.source_floor}->{plan.target_floor}',
                )

            # ---------------- 前置条件检查 ----------------
            if (
                state is None
                or not state.ready
                or state.floor_id != plan.source_floor
                or odom is None
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_INVALID_REQUEST,
                    'active source floor, map, or odometry is not ready',
                )
            current_pose = (  # 检查是否位于电梯触发位姿
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                _yaw_from_odometry(odom),
            )
            distance, yaw_error = pose_error(
                current_pose, plan.source_trigger
            )
            if (
                distance > plan.trigger_tolerance_xy
                or yaw_error > plan.trigger_tolerance_yaw
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_NOT_AT_TRIGGER,
                    f'robot is {distance:.3f}m/{yaw_error:.3f}rad '
                    'from the elevator trigger',
                )

            # ---------------- 正常转移事务 ----------------
            transaction_started = True
            self._publish_hold(True)  # 断言安全保持
            self._feedback(
                goal_handle, 'HOLDING', 'floor switch safety hold asserted'
            )
            if not self._wait_stopped(  # 等待确认停车
                goal_handle,
                float(
                    self._config['map_switch_transaction'][
                        'cancel_navigation_timeout_sec'
                    ]
                ),
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_STOP_TIMEOUT,
                    'robot did not reach a confirmed stop',
                )

            self._feedback(  # 重置旧楼层导航
                goal_handle, 'RESETTING_OLD_NAV', 'clearing old navigation'
            )
            reset = ResetNavigation.Request()
            reset.floor_id = plan.source_floor
            reset.generation = state.generation
            reset_response = self._call_service(
                self._reset_client, reset, self._reset_timeout
            )
            if reset_response is None or not reset_response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_NAVIGATION_RESET,
                    'old-floor navigation reset failed',
                )

            self._feedback(  # 执行传输适配器
                goal_handle,
                'TRANSFERRING',
                f'executing {plan.transfer_adapter} transport adapter',
            )
            transfer_ready, transfer_message = self._execute_transfer(
                goal_handle, plan, state.generation
            )
            if not transfer_ready:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_TRANSFER,
                    transfer_message,
                )

            self._feedback(  # 提交目标楼层地图
                goal_handle, 'SWITCHING_MAP', 'committing target floor map'
            )
            switch_request = SwitchMap.Request()
            switch_request.target_floor = plan.target_floor
            switch_request.expected_current_generation = state.generation
            switch_response = self._call_service(
                self._map_client, switch_request, self._map_timeout
            )
            if switch_response is None or not switch_response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_MAP_SWITCH,
                    'target map commit failed',
                )
            generation = switch_response.generation  # 新地图代数

            self._feedback(  # 位姿交接
                goal_handle,
                'HANDING_OFF_POSE',
                f'applying {plan.pose_handoff} pose policy',
                generation,
            )
            pose_ready, pose_message = self._establish_target_pose(
                goal_handle, plan, generation
            )
            if not pose_ready:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_POSE_TRANSFER,
                    pose_message,
                )

            self._feedback(  # 在交接位姿处重置新楼层导航
                goal_handle,
                'RESETTING_NEW_NAV',
                'resetting SCAN at the transferred pose',
                generation,
            )
            reset.floor_id = plan.target_floor
            reset.generation = generation
            reset_response = self._call_service(
                self._reset_client, reset, self._reset_timeout
            )
            if reset_response is None or not reset_response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_NAVIGATION_RESET,
                    'target-floor navigation reset failed',
                )

            baseline = 0  # 以当前新鲜点云数为基线
            sensing = self._sensing_state
            if (
                sensing is not None
                and sensing.floor_id == plan.target_floor
                and sensing.generation == generation
            ):
                baseline = sensing.fresh_cloud_count
            required_count = baseline + self._minimum_fresh_clouds
            self._feedback(  # 等待目标楼层感知就绪
                goal_handle,
                'WAITING_SENSING',
                f'waiting for {self._minimum_fresh_clouds} post-reset clouds',
                generation,
            )
            if not self._wait_sensing(
                goal_handle,
                plan.target_floor,
                generation,
                required_count,
                self._readiness_timeout,
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    SwitchFloor.Result.ERROR_SENSING_TIMEOUT,
                    'target-floor local sensing did not become fresh',
                )

            self._feedback(  # 完成：释放安全保持
                goal_handle,
                'COMPLETE',
                'target floor is ready; releasing safety hold',
                generation,
            )
            self._publish_hold(False)
            goal_handle.succeed()
            return self._result(
                True,
                SwitchFloor.Result.ERROR_NONE,
                f'switched {plan.source_floor}->{plan.target_floor}; '
                f'adapter={plan.transfer_adapter}, '
                f'pose_handoff={plan.pose_handoff}',
            )
        except _SwitchCancelled as error:
            # 取消：若事务已开始则保持安全保持（不自动放行）
            if transaction_started:
                self._publish_hold(True)
            goal_handle.canceled()
            return self._result(
                False,
                SwitchFloor.Result.ERROR_CANCELLED,
                str(error),
            )
        except Exception as error:
            # Defensive boundary: retain hold on unexpected faults.
            # 【中文】防御性边界：意外故障时保持安全保持。
            if transaction_started:
                self._publish_hold(True)
            self.get_logger().error(f'floor switch failed: {error}')
            goal_handle.abort()
            return self._result(
                False,
                SwitchFloor.Result.ERROR_INTERNAL,
                str(error),
            )
        finally:
            with self._goal_lock:
                self._switch_active = False  # 释放活动标志

    def destroy_node(self) -> None:
        # 【功能】销毁节点资源（Action 服务端与外部客户端）
        self._action_server.destroy()
        for client in self._external_clients.values():
            client.destroy()
        super().destroy_node()


def main() -> None:
    """Run the floor switch manager with concurrent action callbacks."""
    # 【中文】入口：以并发 Action 回调方式运行楼层切换管理器
    rclpy.init()
    node = FloorSwitchManager()
    executor = MultiThreadedExecutor(num_threads=4)  # 多线程执行器（4 线程）
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
