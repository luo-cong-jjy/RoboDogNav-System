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
# 【文件职责】mission_executor_node.py —— 巡检任务执行器节点（ROS2 Node）
# 本节点把配置化的巡检任务（MissionStep 序列）编排为对导航 Action
# （NavigateFloor）与楼层切换 Action（SwitchFloor）的调用：
#   1) 提供 RunMission Action 服务：按步骤执行巡检 / 过路 / 楼层转移 / 终点；
#   2) 支持暂停/恢复/停止/重试当前步骤（ControlMission 服务）；
#   3) 支持多线程执行（MultiThreadedExecutor），子 Action 回调并发处理；
#   4) 在取消/停止/故障/暂停等所有中断情形下保持"安全保持"（mission_hold），
#      保证任务执行器绝不放弃安全控制；
#   5) 发布任务状态（MissionState）与可视化 Marker。
# ============================================================================

"""Typed mission executor composing navigation and floor-switch Actions."""

# ------------------------- 标准库/第三方导入 -------------------------
import math                # 数学库：位姿消息的四元数计算
from pathlib import Path   # 路径对象：定位配置文件
import threading           # 线程库：目标锁（goal_lock）
import time                # 时间库：单调时钟超时等待
from typing import Optional  # 类型提示：Optional 可空值

from geometry_msgs.msg import PoseStamped   # 带姿态的位姿消息（导航目标）
from m20_warehouse_interfaces.action import (  # 本仓库自定义 Action 接口
    NavigateFloor,   # 导航到指定楼层的 Action
    RunMission,      # 运行任务 Action（本节点提供服务）
    SwitchFloor,     # 楼层切换 Action（客户端，调楼层切换管理节点）
)
from m20_warehouse_interfaces.msg import FloorState, MissionState  # 楼层状态 / 任务状态消息
from m20_warehouse_interfaces.srv import ControlMission  # 任务控制服务（暂停/恢复/停止/重试）
import rclpy                          # ROS2 Python 客户端库
from rclpy.action import (            # ROS2 Action 客户端/服务端组件
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup  # 可重入回调组（允许并发回调）
from rclpy.executors import MultiThreadedExecutor         # 多线程执行器
from rclpy.node import Node           # ROS2 节点基类
from rclpy.qos import (               # QoS 策略（锁存发布）
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool         # 标准消息：保持标志
from visualization_msgs.msg import Marker  # 可视化标记：任务状态文字
import yaml                           # YAML 解析：加载任务配置

from .floor_switch_policy import resolve_transition  # 楼层转移路线解析（纯函数）
from .mission_policy import (         # 任务解析纯函数与数据类
    MissionStep,
    display_step_index,
    resolve_mission,
)


def _latched_qos() -> QoSProfile:
    # 【功能】构造锁存 QoS（深度 1、可靠、瞬态本地），供状态/保持话题使用
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionExecutor(Node):
    """Run one configured mission and retain safe control on all interruptions."""
    # 【中文】任务执行器节点：运行一个配置化任务，并在所有中断情形下保留安全控制

    def __init__(self) -> None:
        super().__init__('m20_mission_executor')
        # ------------------------- 参数声明 -------------------------
        self.declare_parameter('config_path', '')   # 配置文件路径
        self.declare_parameter('action_name', '/m20/mission/run')  # RunMission Action 名称
        self.declare_parameter('control_service', '/m20/mission/control')  # 控制服务名
        self.declare_parameter(
            'navigation_action', '/m20/navigation/navigate'  # 导航 Action 名
        )
        self.declare_parameter('floor_action', '/m20/floor_switch')  # 楼层切换 Action 名
        self.declare_parameter('navigation_timeout_sec', 180.0)  # 单段导航超时
        config_path = Path(str(self.get_parameter('config_path').value))
        if not config_path.is_file():
            raise ValueError(f'config_path is not a file: {config_path}')
        with config_path.open('r', encoding='utf-8') as stream:
            self._config = yaml.safe_load(stream)  # 加载任务/楼层/电梯配置
        self._configured_mission_id = str(self._config['mission']['id'])  # 配置的任务 id
        self._navigation_timeout = max(
            1.0,
            float(self.get_parameter('navigation_timeout_sec').value),
        )

        # ------------------------- 运行时状态 -------------------------
        self._callback_group = ReentrantCallbackGroup()  # 可重入回调组（Action 并发）
        self._floor_state: Optional[FloorState] = None   # 最近楼层状态
        self._goal_lock = threading.Lock()               # 目标接受互斥锁
        self._active = False               # 是否有任务正在运行
        self._has_run = False              # 是否已运行过任务（限制单次，除非 restart）
        self._mission_id = ''              # 当前任务 id
        self._state_name = 'IDLE'          # 当前状态名
        self._step_index = 0               # 当前步骤游标
        self._steps = ()                   # 解析后的步骤序列
        self._paused = False               # 是否暂停
        self._fault = False                # 是否处于故障保持
        self._stop_requested = False       # 是否已请求停止
        self._retry_requested = False      # 是否已请求重试当前步骤
        self._atomic_transfer = False      # 是否处于原子楼层转移（不可暂停）
        self._child_message = ''           # 最近子 Action 的反馈消息
        self._mission_hold = False         # 任务保持标志（发布给安全门）
        self._floor_switch_hold: Optional[bool] = None  # 最近楼层切换保持状态（订阅）
        qos = _latched_qos()

        # ------------------------- 发布器 -------------------------
        self._state_publisher = self.create_publisher(  # 任务状态发布器
            MissionState, '/m20/mission/state', qos
        )
        self._hold_publisher = self.create_publisher(  # 任务保持发布器
            Bool, '/m20/control/mission_hold', qos
        )
        self._marker_publisher = self.create_publisher(  # 可视化 Marker 发布器
            Marker, '/m20/visualization/mission_marker', qos
        )
        # ------------------------- 订阅器 -------------------------
        self.create_subscription(  # 楼层状态订阅
            FloorState,
            '/m20/map/state',
            self._floor_callback,
            qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(  # 楼层切换保持订阅
            Bool,
            '/m20/control/floor_switch_hold',
            self._floor_switch_hold_callback,
            qos,
            callback_group=self._callback_group,
        )
        # ------------------------- Action 客户端/服务端 -------------------------
        self._navigation_client = ActionClient(  # 导航 Action 客户端
            self,
            NavigateFloor,
            str(self.get_parameter('navigation_action').value),
            callback_group=self._callback_group,
        )
        self._floor_client = ActionClient(  # 楼层切换 Action 客户端
            self,
            SwitchFloor,
            str(self.get_parameter('floor_action').value),
            callback_group=self._callback_group,
        )
        self._control_service = self.create_service(  # 任务控制服务
            ControlMission,
            str(self.get_parameter('control_service').value),
            self._control_callback,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(  # RunMission Action 服务端
            self,
            RunMission,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._publish_hold(False)
        self._publish_state(None, 'IDLE', 'mission executor ready')
        self.get_logger().info('typed inspection mission executor ready')

    # ------------------------- 回调方法 -------------------------
    def _floor_callback(self, message: FloorState) -> None:
        # 【回调】缓存最近楼层状态
        self._floor_state = message

    def _floor_switch_hold_callback(self, message: Bool) -> None:
        # 【回调】缓存最近楼层切换保持标志
        self._floor_switch_hold = bool(message.data)

    def _goal_callback(self, goal) -> GoalResponse:
        # 【回调】新任务目标接受策略：任务 id 非空、当前无活动任务、
        # 且（首次运行或显式 restart）才接受
        if not goal.mission_id.strip():
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active or (self._has_run and not goal.restart):
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        # 【回调】取消请求总是接受（取消语义在执行回调中处理）
        return CancelResponse.ACCEPT

    def _publish_hold(self, asserted: bool) -> None:
        # 【功能】发布任务保持标志（asserted=True 表示要求安全门停车）
        self._mission_hold = asserted
        self._hold_publisher.publish(Bool(data=asserted))

    def _current_step(self) -> Optional[MissionStep]:
        # 【功能】返回当前步骤（游标越界时返回 None）
        if 0 <= self._step_index < len(self._steps):
            return self._steps[self._step_index]
        return None

    def _make_state(self, state_name: str, message: str) -> MissionState:
        # 【功能】构造任务状态消息（含时间戳/任务 id/当前步骤/楼层/暂停故障标志）
        state = MissionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.mission_id = self._mission_id
        state.state = state_name
        state.step_count = len(self._steps)
        display_index = display_step_index(  # 显示游标（钳制到合法范围）
            self._step_index, len(self._steps)
        )
        state.step_index = display_index
        step = (
            self._steps[display_index]
            if 0 <= display_index < len(self._steps)
            else None
        )
        if step is not None:
            state.step_type = step.step_type
            state.step_name = step.name
        floor = self._floor_state
        if floor is not None:
            state.active_floor = floor.floor_id
            state.map_generation = floor.generation
        state.paused = self._paused
        state.fault = self._fault
        state.message = message
        return state

    def _publish_marker(self, state: MissionState) -> None:
        # 【功能】在仓库总览中发布任务状态文字 Marker（颜色按故障/暂停/运行区分）
        marker = Marker()
        marker.header.stamp = state.header.stamp
        marker.header.frame_id = 'warehouse_overview'
        marker.ns = 'mission_state'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 23.0
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.1
        marker.color.a = 1.0
        if state.fault:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.2, 0.2  # 故障：红
        elif state.paused:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.8, 0.1  # 暂停：黄
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.2, 1.0, 0.5  # 运行：绿
        marker.text = (
            f'Mission {state.mission_id or "-"} | {state.state} | '
            f'step {state.step_index + 1}/{max(1, state.step_count)} '
            f'{state.step_name} | {state.active_floor} '
            f'gen {state.map_generation}\n{state.message[:100]}'
        )
        self._marker_publisher.publish(marker)

    def _publish_state(
        self,
        goal_handle,
        state_name: str,
        message: str,
    ) -> None:
        # 【功能】发布任务状态（话题 + Marker + Action 反馈）
        self._state_name = state_name
        state = self._make_state(state_name, message)
        self._state_publisher.publish(state)
        self._publish_marker(state)
        if goal_handle is not None:
            feedback = RunMission.Feedback()
            feedback.state = state
            goal_handle.publish_feedback(feedback)

    def _control_callback(
        self,
        request: ControlMission.Request,
        response: ControlMission.Response,
    ) -> ControlMission.Response:
        # 【服务】任务控制：暂停 / 恢复 / 停止 / 重试当前步骤
        if not self._active or request.mission_id != self._mission_id:
            response.success = False
            response.state = self._state_name
            response.message = 'requested mission is not active'
            return response
        if request.command == ControlMission.Request.PAUSE:
            if self._atomic_transfer:  # 原子楼层转移期间不可暂停
                response.success = False
                response.state = self._state_name
                response.message = 'atomic elevator transfer cannot be paused'
                return response
            self._paused = True
            self._publish_hold(True)
            response.success = True
            response.message = 'pause requested'
        elif request.command == ControlMission.Request.RESUME:
            if not self._paused:
                response.success = False
                response.message = 'mission is not paused'
            else:
                self._paused = False
                response.success = True
                response.message = 'resume requested'
        elif request.command == ControlMission.Request.STOP:
            self._stop_requested = True
            self._publish_hold(True)  # 停止请求：保留安全保持
            response.success = True
            response.message = 'stop requested; safe hold retained'
        elif request.command == ControlMission.Request.RETRY_CURRENT:
            if not self._fault:
                response.success = False
                response.message = 'mission is not in FAULT_HOLD'
            else:
                self._retry_requested = True
                response.success = True
                response.message = 'retry requested'
        else:
            response.success = False
            response.message = 'unknown mission control command'
        response.state = self._state_name
        return response

    @staticmethod
    def _pose_message(pose, frame_id: str = 'map') -> PoseStamped:
        # 【功能】把 (x, y, yaw) 位姿转换为 PoseStamped 消息（z 固定 0.59，四元数由 yaw 计算）
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.pose.position.x = float(pose[0])
        message.pose.position.y = float(pose[1])
        message.pose.position.z = 0.59
        half_yaw = float(pose[2]) / 2.0
        message.pose.orientation.z = math.sin(half_yaw)
        message.pose.orientation.w = math.cos(half_yaw)
        return message

    def _child_feedback(self, feedback) -> None:
        # 【回调】记录子 Action 的反馈消息（用于转发到任务状态）
        message = feedback.feedback
        self._child_message = str(
            getattr(message, 'message', '')
            or getattr(getattr(message, 'state', None), 'message', '')
        )

    @staticmethod
    def _wait_future(future, timeout: float):
        # 【功能】阻塞等待 future 完成（带超时），返回结果或 None
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)
        return future.result()

    def _cancel_child(self, child_handle, result_future=None) -> None:
        # 【功能】取消子 Action（并可选等待其结果 future 结束）
        if child_handle is None:
            return
        future = child_handle.cancel_goal_async()
        self._wait_future(future, 5.0)
        if result_future is not None:
            self._wait_future(result_future, 5.0)

    def _run_child_action(
        self,
        client,
        request,
        goal_handle,
        *,
        pausable: bool,
        state_name: str,
    ):
        # 【功能】运行一个子 Action（导航/楼层切换），统一处理取消/停止/暂停
        # 【参数】client - Action 客户端；request - 目标请求；goal_handle - 父任务目标句柄；
        #        pausable - 是否可被暂停（楼层转移不可暂停）；state_name - 运行中状态名
        # 【返回】(结果码, 子结果, 消息)：结果码为 SUCCEEDED/FAILED/STOPPED/CANCELLED/PAUSED
        deadline = time.monotonic() + 5.0
        while not client.server_is_ready():  # 等待子 Action 服务端就绪
            if time.monotonic() >= deadline:
                return 'FAILED', None, 'child action server unavailable'
            if goal_handle.is_cancel_requested or self._stop_requested:
                return 'STOPPED', None, 'mission stopped'
            time.sleep(0.05)
        self._child_message = ''
        send_future = client.send_goal_async(  # 异步发送子目标
            request, feedback_callback=self._child_feedback
        )
        child_handle = self._wait_future(send_future, 5.0)
        if child_handle is None or not child_handle.accepted:
            return 'FAILED', None, 'child action goal rejected'
        result_future = child_handle.get_result_async()
        while not result_future.done():  # 等待子目标完成，期间响应取消/停止/暂停
            if goal_handle.is_cancel_requested:
                self._cancel_child(child_handle, result_future)
                return 'CANCELLED', None, 'mission action cancelled'
            if self._stop_requested:
                self._cancel_child(child_handle, result_future)
                return 'STOPPED', None, 'mission stop requested'
            if self._paused and pausable:
                self._cancel_child(child_handle, result_future)
                return 'PAUSED', None, 'mission paused'
            self._publish_state(  # 周期性转发子 Action 运行状态
                goal_handle,
                state_name,
                self._child_message or 'child action running',
            )
            time.sleep(0.20)
        response = result_future.result()
        if response is None:
            return 'FAILED', None, 'child action returned no result'
        result = response.result
        if not result.success:
            return 'FAILED', result, result.message
        return 'SUCCEEDED', result, result.message

    def _wait_while_paused(self, goal_handle) -> str:
        # 【功能】暂停等待循环：保持安全保持，直到恢复/取消/停止
        # 【返回】'RESUMED'（已恢复）/ 'CANCELLED' / 'STOPPED'
        self._publish_hold(True)
        while self._paused:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED'
            if self._stop_requested:
                return 'STOPPED'
            self._publish_state(goal_handle, 'PAUSED', 'mission paused')
            time.sleep(0.10)
        self._publish_hold(False)
        return 'RESUMED'

    def _wait_fault_retry(self, goal_handle, message: str) -> str:
        # 【功能】故障保持等待循环：保持安全保持，直到操作员请求重试/取消/停止
        # 【参数】message - 故障说明；【返回】'RETRY'（已请求重试）/ 'CANCELLED' / 'STOPPED'
        self._fault = True
        self._retry_requested = False
        self._publish_hold(True)
        while not self._retry_requested:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED'
            if self._stop_requested:
                return 'STOPPED'
            self._publish_state(goal_handle, 'FAULT_HOLD', message)
            time.sleep(0.10)
        self._fault = False
        self._retry_requested = False
        self._publish_hold(False)
        return 'RETRY'

    def _navigate(
        self,
        goal_handle,
        pose,
        segment_name: str,
    ):
        # 【功能】发起一次导航（子 Action），返回 (结果码, 消息)
        # 【参数】goal_handle - 父任务目标；pose - 目标位姿；segment_name - 段名（用于 goal_id）
        floor = self._floor_state
        if floor is None or not floor.ready:
            return 'FAILED', 'active map is not ready'
        request = NavigateFloor.Goal()
        request.goal_id = (
            f'{self._mission_id}:{self._step_index}:{segment_name}'
        )
        request.floor_id = floor.floor_id
        request.map_generation = floor.generation
        request.target_pose = self._pose_message(pose)
        request.timeout_sec = self._navigation_timeout
        outcome, _result, message = self._run_child_action(
            self._navigation_client,
            request,
            goal_handle,
            pausable=True,       # 导航可暂停
            state_name='NAVIGATING',
        )
        return outcome, message

    def _run_inspection(self, goal_handle, step: MissionStep):
        # 【功能】执行巡检步骤：导航到点位并停留 dwell_sec（支持暂停续时）
        # 【返回】(结果码, 消息)
        floor = self._floor_state
        if floor is None or floor.floor_id != step.floor_id:
            return 'FAILED', (
                f'inspection {step.name} requires {step.floor_id}'
            )
        outcome, message = self._navigate(
            goal_handle, step.pose, step.name
        )
        if outcome != 'SUCCEEDED':
            return outcome, message
        dwell_deadline = time.monotonic() + step.dwell_sec  # 停留截止时刻
        while time.monotonic() < dwell_deadline:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED', 'mission action cancelled'
            if self._stop_requested:
                return 'STOPPED', 'mission stop requested'
            if self._paused:  # 暂停期间不消耗停留时长（恢复后顺延）
                pause_start = time.monotonic()
                paused = self._wait_while_paused(goal_handle)
                if paused != 'RESUMED':
                    return paused, 'mission stopped while paused'
                dwell_deadline += time.monotonic() - pause_start
            self._publish_state(
                goal_handle,
                'DWELLING',
                f'inspecting {step.name}',
            )
            time.sleep(0.05)
        return 'SUCCEEDED', f'inspection {step.name} complete'

    def _run_terminal(self, goal_handle, step: MissionStep):
        """Navigate to a named mission endpoint without inspection dwell."""
        # 【中文】执行终点步骤：导航到命名终点（无停留）
        # 【返回】(结果码, 消息)
        floor = self._floor_state
        if floor is None or floor.floor_id != step.floor_id:
            return 'FAILED', (
                f'terminal {step.name} requires {step.floor_id}'
            )
        outcome, message = self._navigate(
            goal_handle, step.pose, step.name
        )
        if outcome != 'SUCCEEDED':
            return outcome, message
        return 'SUCCEEDED', f'terminal {step.name} reached'

    def _run_transit(self, goal_handle, step: MissionStep):
        """Navigate through a route-shaping waypoint without inspection dwell."""
        # 【中文】执行过路步骤：导航经过路线整形途经点（无停留）
        # 【返回】(结果码, 消息)
        floor = self._floor_state
        if floor is None or floor.floor_id != step.floor_id:
            return 'FAILED', (
                f'transit {step.name} requires {step.floor_id}'
            )
        outcome, message = self._navigate(
            goal_handle, step.pose, step.name
        )
        if outcome != 'SUCCEEDED':
            return outcome, message
        return 'SUCCEEDED', f'transit {step.name} reached'

    def _run_floor_transfer(self, goal_handle, step: MissionStep):
        # 【功能】执行楼层转移步骤：解析转移计划、导航到触发位姿、调用
        # SwitchFloor Action（原子转移，不可暂停）
        # 【返回】(结果码, 消息)
        plan = resolve_transition(
            self._config,
            step.connector_id,
            step.floor_id,
            step.target_floor,
        )
        floor = self._floor_state
        if floor is None or not floor.ready:
            return 'FAILED', 'active map is not ready for floor transfer'
        recovering_target = floor.floor_id == step.target_floor  # 是否已在目标楼层（恢复场景）
        if (
            recovering_target
            and floor.ready
            and self._floor_switch_hold is False  # 目标已提交且保持已释放：视为成功
        ):
            return (
                'SUCCEEDED',
                f'{step.target_floor} is already committed and released',
            )
        if not recovering_target:
            if floor.floor_id != step.floor_id:
                return 'FAILED', (
                    f'floor transfer requires {step.floor_id} or committed '
                    f'{step.target_floor}, active={floor.floor_id}'
                )
            for pose, segment in (  # 依次导航到连廊接近点与触发点
                (step.pose, 'connector_approach'),
                (plan.source_trigger, 'connector_trigger'),
            ):
                outcome, message = self._navigate(
                    goal_handle, pose, segment
                )
                if outcome != 'SUCCEEDED':
                    return outcome, message

        request = SwitchFloor.Goal()
        request.source_floor = step.floor_id
        request.target_floor = step.target_floor
        # SwitchFloor keeps the legacy field name for wire compatibility.
        # 【中文】SwitchFloor 为保持线级兼容保留旧字段名 elevator_id。
        request.elevator_id = step.connector_id
        self._atomic_transfer = True  # 进入原子转移区间（不可暂停）
        try:
            outcome, _result, message = self._run_child_action(
                self._floor_client,
                request,
                goal_handle,
                pausable=False,       # 楼层转移不可暂停
                state_name='FLOOR_TRANSFER',
            )
            if outcome != 'SUCCEEDED':
                return outcome, message

            # The navigation gateway performs the authoritative wait for its
            # own hold=False sample when it accepts the next-floor goal.  This
            # executor still observes the hold topic for safe retry detection,
            # but it must not infer delivery to a different DDS subscriber.
            # 【中文】导航网关在接受下一楼层目标时会权威地等待其自身的
            # hold=False 采样；本执行器仍订阅保持话题用于安全的重试检测，
            # 但不得推断已投递到另一个 DDS 订阅者。
            return 'SUCCEEDED', message
        finally:
            self._atomic_transfer = False  # 退出原子转移区间

    def _result(
        self,
        success: bool,
        completed: int,
        error_code: int,
        message: str,
    ):
        # 【功能】构造 RunMission 结果消息
        result = RunMission.Result()
        result.success = success
        result.mission_id = self._mission_id
        result.completed_steps = completed
        result.error_code = error_code
        result.message = message
        return result

    def _execute(self, goal_handle):
        # 【主流程】RunMission Action 执行回调：按步骤循环执行任务
        completed = 0
        try:
            self._mission_id = goal_handle.request.mission_id.strip()
            try:
                self._steps = resolve_mission(  # 解析任务步骤序列
                    self._config, self._mission_id
                )
            except ValueError as error:
                goal_handle.abort()
                return self._result(
                    False,
                    0,
                    RunMission.Result.ERROR_INVALID_REQUEST,
                    str(error),
                )
            self._step_index = 0
            self._paused = False
            self._fault = False
            self._stop_requested = False
            self._retry_requested = False
            self._publish_hold(False)
            self._publish_state(
                goal_handle, 'STARTING', 'mission accepted'
            )

            while self._step_index < len(self._steps):  # 步骤主循环
                step = self._steps[self._step_index]
                if self._paused:  # 每步开始时若处于暂停则等待恢复
                    outcome = self._wait_while_paused(goal_handle)
                    if outcome == 'CANCELLED':
                        self._publish_state(
                            goal_handle,
                            'CANCELLED',
                            'mission cancelled; safe hold retained',
                        )
                        goal_handle.canceled()
                        return self._result(
                            False,
                            completed,
                            RunMission.Result.ERROR_CANCELLED,
                            'mission cancelled; safe hold retained',
                        )
                    if outcome == 'STOPPED':
                        self._publish_state(
                            goal_handle,
                            'STOPPED',
                            'mission stopped; safe hold retained',
                        )
                        goal_handle.abort()
                        return self._result(
                            False,
                            completed,
                            RunMission.Result.ERROR_STOPPED,
                            'mission stopped; safe hold retained',
                        )
                # 按步骤类型分派执行
                if step.step_type == 'inspection':
                    outcome, message = self._run_inspection(
                        goal_handle, step
                    )
                elif step.step_type in {
                    'elevator_transfer',
                    'floor_transfer',
                }:
                    outcome, message = self._run_floor_transfer(
                        goal_handle, step
                    )
                elif step.step_type == 'transit':
                    outcome, message = self._run_transit(
                        goal_handle, step
                    )
                elif step.step_type == 'terminal':
                    outcome, message = self._run_terminal(
                        goal_handle, step
                    )
                else:
                    outcome, message = (
                        'FAILED',
                        f'unsupported mission step {step.step_type!r}',
                    )
                if outcome == 'PAUSED':  # 被暂停：等待恢复后重试本步骤
                    paused = self._wait_while_paused(goal_handle)
                    if paused == 'RESUMED':
                        continue
                    outcome = paused
                if outcome == 'FAILED':  # 失败：进入故障保持，等待重试
                    retry = self._wait_fault_retry(goal_handle, message)
                    if retry == 'RETRY':
                        continue
                    outcome = retry
                if outcome in {'STOPPED', 'CANCELLED'}:  # 停止/取消：保持安全保持并返回
                    self._publish_hold(True)
                    if outcome == 'CANCELLED':
                        self._publish_state(
                            goal_handle,
                            'CANCELLED',
                            'mission cancelled; safe hold retained',
                        )
                        goal_handle.canceled()
                        return self._result(
                            False,
                            completed,
                            RunMission.Result.ERROR_CANCELLED,
                            'mission cancelled; safe hold retained',
                        )
                    self._publish_state(
                        goal_handle,
                        'STOPPED',
                        'mission stopped; safe hold retained',
                    )
                    goal_handle.abort()
                    return self._result(
                        False,
                        completed,
                        RunMission.Result.ERROR_STOPPED,
                        'mission stopped; safe hold retained',
                    )
                self._step_index += 1  # 前进到下一步
                completed += 1
                self._publish_state(
                    goal_handle,
                    'STEP_COMPLETE',
                    f'{step.name} complete',
                )

            self._publish_hold(False)  # 全部完成：释放安全保持
            self._publish_state(
                goal_handle, 'COMPLETED', 'mission complete'
            )
            goal_handle.succeed()
            return self._result(
                True,
                completed,
                RunMission.Result.ERROR_NONE,
                'mission complete',
            )
        except Exception as error:  # 防御性边界：意外故障保持安全保持
            self._fault = True
            self._publish_hold(True)
            self.get_logger().error(f'mission executor failed: {error}')
            self._publish_state(
                goal_handle,
                'INTERNAL_ERROR',
                f'{error}; safe hold retained',
            )
            goal_handle.abort()
            return self._result(
                False,
                completed,
                RunMission.Result.ERROR_INTERNAL,
                str(error),
            )
        finally:
            self._atomic_transfer = False
            self._has_run = True
            with self._goal_lock:
                self._active = False  # 释放活动标志（允许下一次目标）

    def destroy_node(self) -> None:
        # 【功能】销毁节点资源（Action 服务端与客户端）
        self._action_server.destroy()
        self._navigation_client.destroy()
        self._floor_client.destroy()
        super().destroy_node()


def main() -> None:
    """Run mission orchestration with concurrent child Action callbacks."""
    # 【中文】入口：以并发子 Action 回调方式运行任务编排
    rclpy.init()
    node = MissionExecutor()
    executor = MultiThreadedExecutor(num_threads=6)  # 多线程执行器（6 线程）
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
