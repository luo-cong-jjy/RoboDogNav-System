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
# vendor_sensing_state_node.py —— 厂商（原生）渲染器感知状态适配节点（中文注释版）
# 作用：把 SCAN-Planner 的原生渲染器（发布 /quad_0/cloud 点云）包装成一个
#       "代次感知"的传感器：统计每个已提交楼层地图之后收到的原生 SCAN 点云帧数，
#       并发布 /m20/sensing/state 状态，供上层判断感知数据是否已与当前地图同步。
# 订阅话题：/m20/map/state（楼层状态）、/quad_0/cloud（原生 SCAN 点云）
# 发布话题：/m20/sensing/state（感知状态，锁存 QoS）
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Expose SCAN-Planner's native renderer as a generation-aware sensor."""

# 时间库：使用单调时钟做重载保护计时
import time
# 类型标注：Optional 表示值可为 None
from typing import Optional

# 自定义接口消息：FloorState（楼层状态）/ LocalSensingState（局部感知状态）
from m20_warehouse_interfaces.msg import FloorState, LocalSensingState
# ROS2 Python 客户端库
import rclpy
# ROS2 节点基类
from rclpy.node import Node
# QoS 相关：持久化策略 / QoS 描述 / 可靠性策略
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
# 传感器消息：点云 PointCloud2
from sensor_msgs.msg import PointCloud2


# 生成锁存型 QoS：用于楼层状态/感知状态等慢变话题
def _latched_qos() -> QoSProfile:
    return QoSProfile(
        # 队列深度：1
        depth=1,
        # 可靠性：可靠传输
        reliability=ReliabilityPolicy.RELIABLE,
        # 持久性：局部瞬态保持（新订阅者立即收到最近一帧）
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


# 感知状态节点：统计每个已提交楼层地图之后收到的原生 SCAN 点云帧数
class VendorSensingState(Node):
    """Count native SCAN cloud frames after each committed floor map."""

    def __init__(self) -> None:
        # 节点名：m20_vendor_sensing_state
        super().__init__('m20_vendor_sensing_state')
        # 声明参数：楼层状态话题
        self.declare_parameter('floor_state_topic', '/m20/map/state')
        # 声明参数：原生 SCAN 点云话题
        self.declare_parameter('cloud_topic', '/quad_0/cloud')
        # 声明参数：感知状态发布话题
        self.declare_parameter('state_topic', '/m20/sensing/state')
        # 声明参数：地图重载保护时间（秒），防止把重载过程中的旧帧误计
        self.declare_parameter('reload_guard_sec', 0.25)
        # 声明参数：输出坐标系
        self.declare_parameter('output_frame', 'world')

        # 最近一帧楼层状态（初始为 None）
        self._floor: Optional[FloorState] = None
        # 新鲜点云帧计数
        self._fresh_cloud_count = 0
        # 允许开始计数的单调时钟时间点（重载保护）
        self._accept_cloud_after = 0.0
        # 已记录"就绪代次"（避免重复打印日志）
        self._logged_ready_generation = None
        # 读取重载保护时长（下限 0）
        self._reload_guard = max(
            0.0, float(self.get_parameter('reload_guard_sec').value)
        )
        # 读取输出坐标系
        self._output_frame = str(self.get_parameter('output_frame').value)
        # 创建感知状态发布器（锁存 QoS）
        self._publisher = self.create_publisher(
            LocalSensingState,
            str(self.get_parameter('state_topic').value),
            _latched_qos(),
        )
        # 订阅楼层状态（锁存 QoS）
        self.create_subscription(
            FloorState,
            str(self.get_parameter('floor_state_topic').value),
            self._floor_callback,
            _latched_qos(),
        )
        # 订阅原生 SCAN 点云（使用传感器默认 QoS）
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            self._cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        # 初始状态：等待活动楼层与原生点云
        self._publish(False, 0, 'waiting for active floor and native cloud')

    # 楼层状态回调：检测楼层/代次变化并重置计数
    def _floor_callback(self, state: FloorState) -> None:
        # 判断楼层/代次是否变化（首帧或不同楼层/代次）
        changed = (
            self._floor is None
            or state.floor_id != self._floor.floor_id
            or state.generation != self._floor.generation
        )
        self._floor = state
        if changed or not state.ready:
            # 楼层变化或地图未就绪：重置计数，并进入重载保护窗口
            self._fresh_cloud_count = 0
            self._accept_cloud_after = time.monotonic() + self._reload_guard
            self._logged_ready_generation = None
        if changed:
            # 打印代次变化日志
            self.get_logger().info(
                'native sensing generation changed: '
                f'floor={state.floor_id}, generation={state.generation}, '
                f'ready={state.ready}'
            )
        # 发布等待状态：若就绪则在等原生渲染器重载地图，否则在地图切换中
        self._publish(
            False,
            0,
            (
                'waiting for native renderer map reload'
                if state.ready
                else 'map transition in progress'
            ),
        )

    # 原生点云回调：在保护窗口外统计新鲜帧
    def _cloud_callback(self, cloud: PointCloud2) -> None:
        # 无楼层状态 / 地图未就绪 / 仍处重载保护窗口：忽略该帧
        if (
            self._floor is None
            or not self._floor.ready
            or time.monotonic() < self._accept_cloud_after
        ):
            return
        # 通过保护窗口：视为当前代次的新鲜帧，计数 +1
        self._fresh_cloud_count += 1
        generation = (self._floor.floor_id, self._floor.generation)
        # 首个新鲜帧到达时打印一次日志（含点数）
        if self._logged_ready_generation != generation:
            self._logged_ready_generation = generation
            self.get_logger().info(
                'first fresh native cloud received: '
                f'floor={self._floor.floor_id}, '
                f'generation={self._floor.generation}, '
                f'points={int(cloud.width) * int(cloud.height)}'
            )
        # 发布"已就绪"状态（带当前帧点数）
        self._publish(
            True,
            int(cloud.width) * int(cloud.height),
            'native SCAN local cloud is current',
        )

    # 组装并发布感知状态消息
    def _publish(self, ready: bool, point_count: int, message: str) -> None:
        state = LocalSensingState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self._output_frame
        # 若有楼层状态则写入楼层 ID 与代次
        if self._floor is not None:
            state.floor_id = self._floor.floor_id
            state.generation = self._floor.generation
        state.ready = ready
        state.fresh_cloud_count = self._fresh_cloud_count
        state.point_count = point_count
        state.message = message
        self._publisher.publish(state)


# 节点入口函数
def main() -> None:
    """Run the native-renderer readiness adapter."""
    # 初始化 ROS2
    rclpy.init()
    # 创建节点
    node = VendorSensingState()
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
