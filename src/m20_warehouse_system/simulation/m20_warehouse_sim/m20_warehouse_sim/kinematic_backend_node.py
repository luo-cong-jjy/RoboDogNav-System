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

# ======================================================================
# kinematic_backend_node.py —— RViz 平面运动学后端节点（中文注释版）
# 作用：作为纯 RViz 仿真后端：订阅安全速度指令（cmd_vel_safe），按平面运动学
#       积分更新机身位姿，并发布里程计（odometry）、TF 变换、关节状态（joint_states）
#       与历史轨迹（path）；同时提供 set_pose 服务用于瞬移复位。
# 订阅话题：/m20/control/cmd_vel_safe（安全速度指令，Twist）
# 发布话题：/m20/sim/body_pose（里程计）、/joint_states（关节）、/m20/sim/path（轨迹）、
#            /m20/sim/backend_ready（就绪标志）
# 服务：/m20/sim/set_pose（设置仿真位姿）
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""ROS 2 node implementing the RViz planar kinematic backend."""

# 数学库：三角函数
import math
# 类型标注：List（列表）
from typing import List

# 几何消息：PoseStamped（带时间戳位姿）/ TransformStamped（TF 变换）/ Twist（速度指令）
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
# 自定义接口服务：SetSimulationPose（设置仿真位姿）
from m20_warehouse_interfaces.srv import SetSimulationPose
# 导航消息：Odometry（里程计）/ Path（路径）
from nav_msgs.msg import Odometry, Path
# ROS2 Python 客户端库
import rclpy
# ROS2 节点基类
from rclpy.node import Node
# QoS 相关：持久化策略 / QoS 描述 / 可靠性策略
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# 传感器消息：JointState（关节状态）
from sensor_msgs.msg import JointState
# 标准消息：Bool（布尔标志）
from std_msgs.msg import Bool
# TF2 广播器：发布坐标变换
from tf2_ros import TransformBroadcaster

# 本包内运动学工具函数
from .kinematics import (
    PlanarState,             # 机身平面状态（位姿 + 速度）
    clamp_planar_command,    # 速度指令限幅
    integrate_planar,        # 平面运动学积分
    normalize_angle,         # 角度归一化
)


# 四足机器人关节名列表：fl/fr/hl/hr = 左前/右前/左后/右后，
# 每个腿含 hipx / hipy / knee / wheel（轮）四个关节，共 16 个
JOINT_NAMES: List[str] = [
    'fl_hipx_joint',
    'fl_hipy_joint',
    'fl_knee_joint',
    'fl_wheel_joint',
    'fr_hipx_joint',
    'fr_hipy_joint',
    'fr_knee_joint',
    'fr_wheel_joint',
    'hl_hipx_joint',
    'hl_hipy_joint',
    'hl_knee_joint',
    'hl_wheel_joint',
    'hr_hipx_joint',
    'hr_hipy_joint',
    'hr_knee_joint',
    'hr_wheel_joint',
]
# 四个轮子关节在 JOINT_NAMES 中的索引（fl/fr/hl/hr 的 wheel_joint）
WHEEL_INDICES = (3, 7, 11, 15)


# 由平面航向角 yaw 生成四元数分量（x/y/z/w）
def _quaternion_from_yaw(yaw: float):
    """Return geometry quaternion components for a planar yaw."""
    # 绕 z 轴旋转：qx=0, qy=0, qz=sin(yaw/2), qw=cos(yaw/2)
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# RViz 平面运动学后端节点：积分安全指令并发布里程计/TF/关节/轨迹
class RvizKinematicBackend(Node):
    """Integrate safe commands and publish odometry, TF, joints, and path."""

    def __init__(self) -> None:
        # 节点名：m20_rviz_kinematic_backend
        super().__init__('m20_rviz_kinematic_backend')
        # 声明参数：初始 x 位置（米）
        self.declare_parameter('initial_x', -37.0)
        # 声明参数：初始 y 位置（米）
        self.declare_parameter('initial_y', 0.0)
        # 声明参数：初始 z 高度（米）
        self.declare_parameter('initial_z', 0.59)
        # 声明参数：初始航向角（弧度）
        self.declare_parameter('initial_yaw', 0.0)
        # 声明参数：里程计/TF 的父坐标系
        self.declare_parameter('frame_id', 'map')
        # 声明参数：机身坐标系
        self.declare_parameter('child_frame_id', 'base_link')
        # 声明参数：状态更新频率（Hz）
        self.declare_parameter('update_rate_hz', 50.0)
        # 声明参数：速度指令超时时间（秒），超时未收到新指令则停车
        self.declare_parameter('command_timeout_sec', 0.3)
        # 声明参数：x 方向最大线速度（米/秒）
        self.declare_parameter('max_linear_x', 0.45)
        # 声明参数：y 方向最大线速度（米/秒）
        self.declare_parameter('max_linear_y', 0.20)
        # 声明参数：最大角速度（弧度/秒）
        self.declare_parameter('max_angular_z', 0.65)
        # 声明参数：轮子半径（米），用于计算轮子转角
        self.declare_parameter('wheel_radius', 0.09)
        # 声明参数：轨迹最多保留的位姿数
        self.declare_parameter('path_max_poses', 3000)
        # 声明参数：安全速度指令话题
        self.declare_parameter(
            'safe_cmd_topic', '/m20/control/cmd_vel_safe'
        )
        # 声明参数：里程计发布话题
        self.declare_parameter('odom_topic', '/m20/sim/body_pose')
        # 声明参数：轨迹发布话题
        self.declare_parameter('path_topic', '/m20/sim/path')

        # 用初始参数构造初始平面状态
        self._state = PlanarState(
            x=float(self.get_parameter('initial_x').value),
            y=float(self.get_parameter('initial_y').value),
            z=float(self.get_parameter('initial_z').value),
            yaw=float(self.get_parameter('initial_yaw').value),
        )
        # 读取父坐标系
        self._frame_id = str(self.get_parameter('frame_id').value)
        # 读取机身坐标系
        self._child_frame_id = str(self.get_parameter('child_frame_id').value)
        # 读取指令超时时间（下限 0.05 秒）
        self._timeout = max(
            0.05, float(self.get_parameter('command_timeout_sec').value)
        )
        # 读取速度上限三元组 (max_vx, max_vy, max_wz)
        self._limits = (
            float(self.get_parameter('max_linear_x').value),
            float(self.get_parameter('max_linear_y').value),
            float(self.get_parameter('max_angular_z').value),
        )
        # 读取轮子半径（下限 0.01 米）
        self._wheel_radius = max(
            0.01, float(self.get_parameter('wheel_radius').value)
        )
        # 读取轨迹最大位姿数（下限 10）
        self._path_max_poses = max(
            10, int(self.get_parameter('path_max_poses').value)
        )
        # 更新频率（下限 1 Hz）
        rate = max(1.0, float(self.get_parameter('update_rate_hz').value))
        # 当前速度指令（机身系 vx, vy, wz）
        self._command = (0.0, 0.0, 0.0)
        # 最近一次收到指令的时刻（纳秒）
        self._last_command_ns = 0
        # 最近一次状态更新的时刻（纳秒）
        self._last_update_ns = self.get_clock().now().nanoseconds
        # 轮子累计转角（弧度，用于关节显示）
        self._wheel_phase = 0.0
        # 发布计数（每 5 次记录一个轨迹点）
        self._publish_count = 0

        # 创建里程计发布器
        self._odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 20
        )
        # 创建关节状态发布器（/joint_states，供 RViz 显示模型动作）
        self._joint_publisher = self.create_publisher(
            JointState, '/joint_states', 20
        )
        # 创建轨迹发布器
        self._path_publisher = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 5
        )
        # 构造锁存 QoS（供就绪标志使用）
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # 创建后端就绪标志发布器（锁存：新订阅者立即可见）
        self._ready_publisher = self.create_publisher(
            Bool, '/m20/sim/backend_ready', ready_qos
        )
        # 订阅安全速度指令（Twist）
        self._command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter('safe_cmd_topic').value),
            self._command_callback,
            20,
        )
        # 提供瞬移服务 /m20/sim/set_pose
        self._set_pose_service = self.create_service(
            SetSimulationPose,
            '/m20/sim/set_pose',
            self._set_pose_callback,
        )
        # TF 广播器：发布 map -> base_link 变换
        self._tf_broadcaster = TransformBroadcaster(self)
        # 初始化轨迹消息（父坐标系）
        self._path = Path()
        self._path.header.frame_id = self._frame_id
        # 创建定时器：周期性更新状态
        self._timer = self.create_timer(1.0 / rate, self._update)
        # 发布就绪标志
        self._ready_publisher.publish(Bool(data=True))
        # 打印启动日志
        self.get_logger().info(
            'RViz kinematic backend ready: safe cmd -> '
            f'{self.get_parameter("odom_topic").value}, '
            f'initial=({self._state.x:.2f}, {self._state.y:.2f}, '
            f'{self._state.z:.2f})'
        )

    # 速度指令回调：限幅后保存指令与时间戳
    def _command_callback(self, message: Twist) -> None:
        # 对指令做防御性限幅
        self._command = clamp_planar_command(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self._limits,
        )
        # 记录收到指令的时刻（用于超时判断）
        self._last_command_ns = self.get_clock().now().nanoseconds

    # set_pose 服务回调：瞬移机器人并清空此前运动
    def _set_pose_callback(
        self,
        request: SetSimulationPose.Request,
        response: SetSimulationPose.Response,
    ) -> SetSimulationPose.Response:
        """Teleport the RViz backend while discarding all prior motion."""
        # 校验请求位姿全部有限（拒绝 NaN/Inf）
        if not all(
            math.isfinite(value)
            for value in (request.x, request.y, request.yaw)
        ):
            # 非法位姿：返回失败
            response.success = False
            response.message = 'pose contains a non-finite value'
            return response
        # 当前时刻
        current = self.get_clock().now()
        # 直接覆盖状态（保留 z 高度，航向归一化）
        self._state = PlanarState(
            x=request.x,
            y=request.y,
            z=self._state.z,
            yaw=normalize_angle(request.yaw),
        )
        # 清空速度指令与运动历史
        self._command = (0.0, 0.0, 0.0)
        self._last_command_ns = current.nanoseconds
        self._last_update_ns = current.nanoseconds
        self._path = Path()
        self._path.header.frame_id = self._frame_id
        self._publish_count = 0
        # 立即发布一帧新状态
        self._publish_state(current.to_msg(), self._command)
        # 返回成功
        response.success = True
        response.message = (
            f'teleported to floor={request.floor_id}, '
            f'generation={request.generation}'
        )
        self.get_logger().info(response.message)
        return response

    # 定时更新：计算时间步长，超时停车，积分运动，发布状态
    def _update(self) -> None:
        # 当前时刻
        current = self.get_clock().now()
        current_ns = current.nanoseconds
        # 时间步长 dt（秒）
        dt = (current_ns - self._last_update_ns) / 1e9
        self._last_update_ns = current_ns
        # 防御性检查：dt 为负或过大（>0.2s，如卡顿）则按 0 处理，避免跳变
        if dt < 0.0 or dt > 0.2:
            dt = 0.0
        command = self._command
        # 指令超时判断：从未收到指令 或 距上次指令超过超时时间 -> 停车
        if (
            self._last_command_ns == 0
            or (current_ns - self._last_command_ns) / 1e9 > self._timeout
        ):
            command = (0.0, 0.0, 0.0)
        # 平面运动学积分：更新位姿与速度
        self._state = integrate_planar(self._state, command, dt)
        # 累加轮子转角（线速度 / 轮半径 = 轮角速度）
        self._wheel_phase += command[0] / self._wheel_radius * dt
        # 发布里程计/TF/关节/轨迹
        self._publish_state(current.to_msg(), command)

    # 发布一帧完整状态：里程计 + TF + 关节状态 +（周期性的）轨迹
    def _publish_state(self, stamp, command) -> None:
        # 由航向角生成四元数
        qx, qy, qz, qw = _quaternion_from_yaw(self._state.yaw)
        # 组装里程计消息
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self._frame_id
        odometry.child_frame_id = self._child_frame_id
        odometry.pose.pose.position.x = self._state.x
        odometry.pose.pose.position.y = self._state.y
        odometry.pose.pose.position.z = self._state.z
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        # ROS Odometry expresses twist in child_frame_id. Keep the
        # world-frame values only inside PlanarState for pose integration.
        # 注释（原文）：ROS 里程计的 twist 表达在 child_frame_id（机身系）中；
        # 世界系速度只保存在 PlanarState 内部用于位姿积分。
        odometry.twist.twist.linear.x = self._state.vx_body
        odometry.twist.twist.linear.y = self._state.vy_body
        odometry.twist.twist.angular.z = self._state.wz
        # 发布里程计
        self._odom_publisher.publish(odometry)

        # 组装并广播 TF 变换（map -> base_link）
        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self._child_frame_id
        transform.transform.translation.x = self._state.x
        transform.transform.translation.y = self._state.y
        transform.transform.translation.z = self._state.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        # 组装关节状态：16 个关节，仅轮子关节有位置/速度
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = JOINT_NAMES
        # 关节位置默认全 0，轮子填入累计转角
        joints.position = [0.0] * len(JOINT_NAMES)
        for index in WHEEL_INDICES:
            joints.position[index] = self._wheel_phase
        # 关节速度默认全 0，轮子填入线速度/轮半径
        joints.velocity = [0.0] * len(JOINT_NAMES)
        for index in WHEEL_INDICES:
            joints.velocity[index] = command[0] / self._wheel_radius
        # 发布关节状态
        self._joint_publisher.publish(joints)

        # 每 5 帧记录一个轨迹点并发布（控制话题流量）
        self._publish_count += 1
        if self._publish_count % 5 == 0:
            pose = PoseStamped()
            pose.header = odometry.header
            pose.pose = odometry.pose.pose
            self._path.header.stamp = stamp
            self._path.poses.append(pose)
            # 超出上限时丢弃最老的位姿，保持轨迹长度受控
            if len(self._path.poses) > self._path_max_poses:
                self._path.poses = self._path.poses[-self._path_max_poses:]
            # 发布轨迹
            self._path_publisher.publish(self._path)


# 节点入口函数
def main() -> None:
    """Run the RViz kinematic backend."""
    # 初始化 ROS2
    rclpy.init()
    # 创建节点
    node = RvizKinematicBackend()
    try:
        # 进入事件循环
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 正常退出
        pass
    finally:
        try:
            # 清理节点与 ROS2 环境
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
