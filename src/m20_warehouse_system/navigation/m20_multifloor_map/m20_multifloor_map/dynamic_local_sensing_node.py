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
# dynamic_local_sensing_node.py —— 动态局部感知节点（中文注释版）
# 作用：根据当前"已提交（committed）"的地图代次（generation），从全局活动地图
#       点云中裁剪出机器人周围 sensing_radius 范围内的局部点，并按旋转分片方式
#       发布，生成 SCAN 规划器需要的局部激光感知。
# 订阅话题：/m20/map/active_global_cloud（全局地图点云）、/m20/map/state（楼层状态）、
#            /m20/sim/body_pose（机身位姿）
# 发布话题：/m20/sensing/local_cloud、/m20/sensing/sensor_cloud（局部点云）、
#            /m20/sensing/lidar_pose（雷达位姿）、/m20/sensing/state（感知状态）
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Generate local SCAN sensing from the current map generation."""

# 数学库：提供 atan2、三角函数等
import math
# 类型标注：Optional 表示参数/返回值可为 None
from typing import Optional

# 自定义接口消息：FloorState（楼层状态）/ LocalSensingState（局部感知状态）
from m20_warehouse_interfaces.msg import FloorState, LocalSensingState
# ROS2 导航消息：Odometry（里程计，携带位姿与速度）
from nav_msgs.msg import Odometry
# NumPy：点云数据的数组运算
import numpy as np
# ROS2 Python 客户端库（初始化/关闭/节点运行）
import rclpy
# ROS2 节点基类
from rclpy.node import Node
# QoS（服务质量）相关：持久化策略 / QoS 描述 / 可靠性策略
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
# 传感器消息：PointCloud2（点云）/ PointField（点字段描述）
from sensor_msgs.msg import PointCloud2, PointField
# sensor_msgs 的 Python 工具库：点云消息的创建/解析
from sensor_msgs_py import point_cloud2
# 标准消息：Header（时间戳 + 坐标系 frame_id）
from std_msgs.msg import Header

# 本包内工具函数：rotating_point_slice（旋转分片）/ select_local_points（局部点选择）
from .point_cloud_filter import rotating_point_slice, select_local_points


# 生成"锁存型（latched）"QoS：用于地图点云、楼层状态等慢变话题
def _latched_qos() -> QoSProfile:
    return QoSProfile(
        # 队列深度：1（只保留最新一帧）
        depth=1,
        # 可靠性：可靠传输（保证消息不丢）
        reliability=ReliabilityPolicy.RELIABLE,
        # 持久性：局部瞬态保持（新订阅者连接后能立即收到最近一帧）
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


# 生成传感器数据流 QoS：用于高频发布的点云话题
def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        # 队列深度：5（允许短暂积压）
        depth=5,
        # Reliable publication prevents fragmented PointCloud2 samples from
        # vanishing silently. Best-effort SCAN/RViz readers remain compatible
        # with a reliable writer under DDS requested/offered semantics.
        # 注释（原文）：可靠发布可防止分片的 PointCloud2 样本静默丢失；
        # best-effort 的 SCAN/RViz 订阅者与可靠发布者在 DDS requested/offered 语义下仍兼容
        reliability=ReliabilityPolicy.RELIABLE,
        # 持久性：易失（不保留历史帧）
        durability=DurabilityPolicy.VOLATILE,
    )


# 定义点云字段布局：x/y/z 坐标 + intensity 强度，每个字段 FLOAT32（4 字节）
def _cloud_fields():
    return [
        # x 坐标字段：偏移 0 字节
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        # y 坐标字段：偏移 4 字节
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        # z 坐标字段：偏移 8 字节
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        # 强度字段：偏移 12 字节（单个点共 16 字节）
        PointField(
            name='intensity',
            offset=12,
            datatype=PointField.FLOAT32,
            count=1,
        ),
    ]


# 把地图服务器发布的 PointCloud2 消息解码为 numpy 数组（N x 4：x/y/z/intensity）
def _cloud_to_numpy(message: PointCloud2) -> np.ndarray:
    """Decode the x/y/z/intensity layout published by the map server."""
    # 期望的字段偏移布局（x/y/z/intensity 各占 4 字节）
    required_offsets = {'x': 0, 'y': 4, 'z': 8, 'intensity': 12}
    # 消息中实际的字段偏移
    actual_offsets = {field.name: field.offset for field in message.fields}
    # 若字段偏移与期望不符，则抛异常（布局不兼容）
    if any(
        actual_offsets.get(name) != offset
        for name, offset in required_offsets.items()
    ):
        raise ValueError('global cloud has an unsupported field layout')
    # 单点字节数必须 >= 16，否则无法容纳四个 float32 字段
    if message.point_step < 16:
        raise ValueError('global cloud point_step is smaller than 16 bytes')
    # 总点数 = 宽度 x 高度
    count = int(message.width) * int(message.height)
    # 空点云直接返回空数组 (0, 4)
    if count == 0:
        return np.empty((0, 4), dtype=np.float32)
    # 根据字节序（大端/小端）构造 numpy dtype
    byte_order = '>' if message.is_bigendian else '<'
    dtype = np.dtype(
        {
            # 字段名
            'names': ['x', 'y', 'z', 'intensity'],
            # 每个字段均为 float32
            'formats': [f'{byte_order}f4'] * 4,
            # 字段字节偏移
            'offsets': [0, 4, 8, 12],
            # 单个点的总字节数（使用消息中的 point_step，兼容额外填充字段）
            'itemsize': int(message.point_step),
        }
    )
    # 用结构化 dtype 直接解释消息原始字节
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    # 按列拼接为 (N, 4) 的连续数组并返回
    return np.ascontiguousarray(
        np.column_stack(
            [
                records['x'],
                records['y'],
                records['z'],
                records['intensity'],
            ]
        ),
        dtype=np.float32,
    )


# 动态局部感知节点：只发布属于"已提交地图代次"的局部点云
class DynamicLocalSensing(Node):
    """Publish only local points belonging to the committed map generation."""

    def __init__(self) -> None:
        # 节点名：m20_dynamic_local_sensing
        super().__init__('m20_dynamic_local_sensing')
        # 声明参数：全局活动地图点云话题（默认 /m20/map/active_global_cloud）
        self.declare_parameter(
            'global_cloud_topic', '/m20/map/active_global_cloud'
        )
        # 声明参数：楼层状态话题
        self.declare_parameter('floor_state_topic', '/m20/map/state')
        # 声明参数：机身位姿话题
        self.declare_parameter('body_pose_topic', '/m20/sim/body_pose')
        # 声明参数：局部点云发布话题
        self.declare_parameter('local_cloud_topic', '/m20/sensing/local_cloud')
        # 声明参数：传感器点云发布话题（兼容 SCAN 规划器约定）
        self.declare_parameter(
            'sensor_cloud_topic', '/m20/sensing/sensor_cloud'
        )
        # 声明参数：雷达位姿发布话题
        self.declare_parameter('lidar_pose_topic', '/m20/sensing/lidar_pose')
        # 声明参数：感知半径（米）
        self.declare_parameter('sensing_radius', 20.0)
        # 声明参数：每帧点云最大点数（受 DDS 传输上限约束）
        self.declare_parameter('max_points_per_cloud', 3500)
        # 声明参数：发布频率（Hz）
        self.declare_parameter('publish_rate_hz', 10.0)
        # 声明参数：雷达相对机身的 x 方向偏移（米）
        self.declare_parameter('lidar_offset_x', 0.18)
        # 声明参数：雷达相对机身的 z 方向偏移（米）
        self.declare_parameter('lidar_offset_z', 0.12)
        # 声明参数：输出坐标系
        self.declare_parameter('output_frame', 'map')
        # 声明参数：雷达坐标系
        self.declare_parameter('lidar_frame', 'lidar_link')

        # 读取感知半径（下限保护为 0.1 米）
        self._radius = max(
            0.1, float(self.get_parameter('sensing_radius').value)
        )
        # 读取每帧最大点数（下限保护为 100 点）
        self._max_points = max(
            100, int(self.get_parameter('max_points_per_cloud').value)
        )
        # 读取雷达 x 偏移
        self._offset_x = float(self.get_parameter('lidar_offset_x').value)
        # 读取雷达 z 偏移
        self._offset_z = float(self.get_parameter('lidar_offset_z').value)
        # 读取输出坐标系
        self._output_frame = str(self.get_parameter('output_frame').value)
        # 读取雷达坐标系
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        # 当前楼层 ID（初始为空）
        self._floor_id = ''
        # 当前地图代次（generation）
        self._generation = 0
        # 地图状态是否就绪（是否有已提交的地图）
        self._state_ready = False
        # 当前生效的地图点云（已提交代次）
        self._map_points: Optional[np.ndarray] = None
        # 待提交的候选点云（地图切换中收到的点云先暂存）
        self._candidate_points: Optional[np.ndarray] = None
        # 机身位姿（最新一帧）
        self._body_pose: Optional[Odometry] = None
        # 已发布的新鲜点云帧计数
        self._fresh_cloud_count = 0
        # 旋转分片相位（每帧递增，实现轮流覆盖全部点）
        self._cloud_phase = 0
        # 上一帧发布的点数
        self._last_point_count = 0
        # 上一帧发布点的包围盒（min_x, max_x, min_y, max_y）
        self._last_bounds = (0.0, 0.0, 0.0, 0.0)

        # 创建局部点云发布器（使用传感器 QoS）
        self._local_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('local_cloud_topic').value),
            _sensor_qos(),
        )
        # Keep a dedicated RViz/debug stream matching SCAN-Planner's original
        # sensor_cloud convention. Both streams contain the current scan slice
        # in the configured world/map frame.
        # 注释（原文）：保留一个专用的 RViz/调试流，与 SCAN 规划器原有的 sensor_cloud
        # 约定一致；两个流都发布同一份扫描分片，坐标系为配置的 world/map 帧
        self._sensor_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('sensor_cloud_topic').value),
            _sensor_qos(),
        )
        # 创建雷达位姿发布器（Odometry，队列深度 20）
        self._lidar_pose_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('lidar_pose_topic').value),
            20,
        )
        # 创建感知状态发布器（锁存 QoS，供监测节点读取）
        self._state_publisher = self.create_publisher(
            LocalSensingState, '/m20/sensing/state', _latched_qos()
        )
        # 订阅全局活动地图点云（锁存 QoS）
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            str(self.get_parameter('global_cloud_topic').value),
            self._cloud_callback,
            _latched_qos(),
        )
        # 订阅楼层状态（锁存 QoS）
        self._state_subscription = self.create_subscription(
            FloorState,
            str(self.get_parameter('floor_state_topic').value),
            self._floor_state_callback,
            _latched_qos(),
        )
        # 订阅机身位姿（队列深度 20）
        self._pose_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('body_pose_topic').value),
            self._pose_callback,
            20,
        )
        # 发布频率（下限 1 Hz）
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        # 创建定时器，周期性调用局部点云发布逻辑
        self._timer = self.create_timer(1.0 / rate, self._publish_local_cloud)
        # 初始状态：等待活动地图
        self._publish_sensing_state(False, 'waiting for active map')

    # 楼层状态回调：处理楼层/代次切换
    def _floor_state_callback(self, state: FloorState) -> None:
        # 地图未就绪：进入"地图切换中"状态，清空所有缓存
        if not state.ready:
            self._floor_id = state.floor_id
            self._generation = state.generation
            self._state_ready = False
            self._map_points = None
            self._candidate_points = None
            self._fresh_cloud_count = 0
            self._cloud_phase = 0
            self._last_point_count = 0
            self._last_bounds = (0.0, 0.0, 0.0, 0.0)
            # 发布空点云，避免下游使用过期数据
            self._publish_empty_cloud()
            # 发布状态：地图切换中
            self._publish_sensing_state(False, 'map transition in progress')
            return

        # 判断楼层或代次是否发生变化
        changed = (
            state.floor_id != self._floor_id
            or state.generation != self._generation
        )
        self._floor_id = state.floor_id
        self._generation = state.generation
        # 地图就绪
        self._state_ready = True
        if changed:
            # 代次变化：重置分片相位与计数
            self._fresh_cloud_count = 0
            self._cloud_phase = 0
        # 若切换期间收到了候选点云，则将其提升为正式地图点云
        if self._candidate_points is not None:
            self._map_points = self._candidate_points
            self._candidate_points = None
        # 发布状态：等待一帧新鲜局部点云
        self._publish_sensing_state(False, 'waiting for a fresh local cloud')

    # 全局点云回调：解码并缓存地图点云
    def _cloud_callback(self, cloud: PointCloud2) -> None:
        try:
            # 解码点云为 numpy 数组
            points = _cloud_to_numpy(cloud)
        except ValueError as error:
            # 布局不兼容时记录错误并丢弃
            self.get_logger().error(str(error))
            return
        if self._state_ready:
            # 地图就绪：直接作为当前地图点云，并重置计数
            self._map_points = points
            self._fresh_cloud_count = 0
            self._cloud_phase = 0
        else:
            # 地图切换中：暂存为候选点云，待状态就绪后提交
            self._candidate_points = points

    # 机身位姿回调：仅保存最新位姿
    def _pose_callback(self, pose: Odometry) -> None:
        self._body_pose = pose

    # 发布空点云（两个点云话题都发），用于地图切换期间的占位
    def _publish_empty_cloud(self) -> None:
        # 构造消息头（当前时间戳 + 输出坐标系）
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._output_frame,
        )
        # 空点云数组 (0, 4)
        empty = np.empty((0, 4), dtype=np.float32)
        # 创建并发布空点云
        cloud = point_cloud2.create_cloud(header, _cloud_fields(), empty)
        self._local_publisher.publish(cloud)
        self._sensor_cloud_publisher.publish(cloud)

    # 定时发布局部点云：裁剪 + 旋转分片 + 发布
    def _publish_local_cloud(self) -> None:
        # 任一前置条件不满足（地图未就绪/无地图点/无机身位姿）则跳过
        if (
            not self._state_ready
            or self._map_points is None
            or self._body_pose is None
        ):
            return
        # 取机身位置作为感知中心
        position = self._body_pose.pose.pose.position
        # 第一步：按水平半径裁剪出机身周围的局部点
        local_points = select_local_points(
            self._map_points,
            position.x,
            position.y,
            self._radius,
        )
        # 第二步：按相位旋转分片，保证每帧点数不超过 DDS 上限
        local_points = rotating_point_slice(
            local_points,
            self._max_points,
            self._cloud_phase,
        )
        # 相位递增（下一帧换一个分片）
        self._cloud_phase += 1
        # 记录本帧点数
        self._last_point_count = int(local_points.shape[0])
        if self._last_point_count:
            # 计算本帧点云的 xy 包围盒（用于监测显示）
            self._last_bounds = (
                float(local_points[:, 0].min()),
                float(local_points[:, 0].max()),
                float(local_points[:, 1].min()),
                float(local_points[:, 1].max()),
            )
        else:
            # 空分片时包围盒清零
            self._last_bounds = (0.0, 0.0, 0.0, 0.0)
        # 构造消息头与点云并发布
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._output_frame)
        cloud = point_cloud2.create_cloud(
            header, _cloud_fields(), local_points
        )
        self._local_publisher.publish(cloud)
        self._sensor_cloud_publisher.publish(cloud)
        # 同步发布雷达位姿
        self._publish_lidar_pose(stamp)
        # 新鲜帧计数 +1
        self._fresh_cloud_count += 1
        # 发布感知状态
        self._publish_sensing_state(
            True,
            f'published {local_points.shape[0]} current-generation points',
        )

    # 根据机身位姿推算雷达位姿并发布（考虑 x/z 安装偏移）
    def _publish_lidar_pose(self, stamp) -> None:
        body = self._body_pose
        # 取机身四元数姿态
        orientation = body.pose.pose.orientation
        # 由四元数反解航向角 yaw（绕 z 轴）
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z),
            1.0 - 2.0 * orientation.z * orientation.z,
        )
        # 构造雷达位姿消息
        pose = Odometry()
        pose.header.stamp = stamp
        pose.header.frame_id = self._output_frame
        pose.child_frame_id = self._lidar_frame
        # x 偏移沿机身朝向投影到世界系
        pose.pose.pose.position.x = (
            body.pose.pose.position.x + self._offset_x * math.cos(yaw)
        )
        pose.pose.pose.position.y = (
            body.pose.pose.position.y + self._offset_x * math.sin(yaw)
        )
        # z 偏移直接叠加
        pose.pose.pose.position.z = (
            body.pose.pose.position.z + self._offset_z
        )
        # 姿态与机身一致
        pose.pose.pose.orientation = orientation
        # 速度沿用机身速度
        pose.twist = body.twist
        self._lidar_pose_publisher.publish(pose)

    # 发布感知状态消息（就绪标志 + 楼层/代次 + 统计信息）
    def _publish_sensing_state(self, ready: bool, message: str) -> None:
        state = LocalSensingState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self._output_frame
        state.floor_id = self._floor_id
        state.generation = self._generation
        state.ready = ready
        state.fresh_cloud_count = self._fresh_cloud_count
        state.point_count = self._last_point_count
        # 写入本帧点云包围盒
        (
            state.min_x,
            state.max_x,
            state.min_y,
            state.max_y,
        ) = self._last_bounds
        state.message = message
        self._state_publisher.publish(state)


# 节点入口函数
def main() -> None:
    """Run generation-aware local sensing."""
    # 初始化 ROS2
    rclpy.init()
    # 创建节点
    node = DynamicLocalSensing()
    try:
        # 进入事件循环，持续处理回调
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
