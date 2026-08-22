#!/usr/bin/env python3
"""Follow a traversability support path directly with /cmd_vel.

This is a Gazebo validation controller for legged M20 Building tests. It bypasses
SCAN-Planner's aerial-style B-spline collision optimizer and tracks /initial_path
as a ground/support-surface reference.
"""
# ============================================================
# 文件职责：支撑路径跟踪控制器（support_path_follower）。
#   这是 M20 四足机器人楼宇测试用的 Gazebo 验证控制器：
#   绕过 SCAN-Planner 的空中式 B 样条碰撞优化，直接订阅 /initial_path
#   作为地面/支撑面参考路径，用纯跟踪（前视点 + P 控制器）输出 /cmd_vel
#   速度指令，驱动机器人沿支撑路径行走。
#   支持前向行驶（forward_only）、原地转向、到达终点自动停止。
# ============================================================

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def clamp(value, low, high):
    # 函数：把数值限制在 [low, high] 区间
    return max(low, min(high, value))


def normalize_angle(angle):
    # 函数：把角度归一化到 (-pi, pi] 区间
    # 参数：angle - 输入角度（弧度）；返回：归一化后的角度
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    # 函数：从四元数提取偏航角 yaw（绕 z 轴）
    # 参数：q - 四元数（w, x, y, z）；返回：yaw 角（弧度）
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SupportPathFollower(Node):
    def __init__(self):
        # 构造函数：声明参数、创建订阅/发布器与定时器
        super().__init__("m20_support_path_follower")
        self.declare_parameter("use_sim_time", False) # 是否使用仿真时间
        self.lookahead_distance = self.declare_parameter("lookahead_distance", 0.70).value # 前视距离（米）
        self.waypoint_reached_distance = self.declare_parameter(
            "waypoint_reached_distance", 0.35 # 判定"已到达路点"的距离
        ).value
        self.finish_distance = self.declare_parameter("finish_distance", 0.40).value # 到达终点的判定距离
        self.rotate_in_place_yaw_error = self.declare_parameter(
            "rotate_in_place_yaw_error", 0.75 # 原地转向的偏航误差阈值（弧度）
        ).value
        self.kp_xy = self.declare_parameter("kp_xy", 0.85).value # XY 方向 P 增益
        self.kp_yaw = self.declare_parameter("kp_yaw", 1.6).value # 偏航 P 增益
        self.max_vx = self.declare_parameter("max_vx", 0.45).value # 最大前向速度
        self.max_vy = self.declare_parameter("max_vy", 0.08).value # 最大横向速度
        self.max_vyaw = self.declare_parameter("max_vyaw", 0.8).value # 最大偏航角速度
        self.forward_only = self.declare_parameter("forward_only", True).value # 是否仅前向行驶

        self.path = []           # 收到的路径路点列表 [(x, y, z), ...]
        self.current_index = 0   # 当前进度索引（最近的路点）
        self.odom = None         # 最新里程计
        self.active = False      # 是否正在跟踪路径

        # 路径使用 TRANSIENT_LOCAL 持久化 QoS（订阅方晚于发布方也能收到）
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(
            Path, "initial_path", self.path_callback, path_qos # 订阅支撑路径
        )
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 10 # 订阅里程计
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 20) # 发布速度指令
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz 控制周期
        self.get_logger().info("Support path follower ready")

    def path_callback(self, msg):
        # 回调：接收新路径，转为路点列表并激活跟踪
        self.path = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in msg.poses
        ]
        self.current_index = 0
        self.active = len(self.path) >= 2 # 少于 2 个路点无法跟踪
        if self.active:
            self.get_logger().info(
                "Received support path: %d waypoints, start_z=%.2f, goal_z=%.2f"
                % (len(self.path), self.path[0][2], self.path[-1][2]) # 打印起止高度
            )
        else:
            self.get_logger().warning("Ignoring support path with fewer than 2 waypoints")

    def odom_callback(self, msg):
        # 回调：缓存最新里程计
        self.odom = msg

    def publish_stop(self):
        # 函数：发布零速度指令（停车）
        self.cmd_pub.publish(Twist())

    def timer_callback(self):
        # 函数：周期控制回调——计算速度指令并发布
        if not self.active or self.odom is None:
            self.publish_stop() # 未激活或无里程计：停车
            return

        pos = self.odom.pose.pose.position
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation) # 当前偏航角
        px, py = pos.x, pos.y # 当前位置

        final = self.path[-1] # 终点
        if math.hypot(final[0] - px, final[1] - py) <= self.finish_distance:
            # 到达终点：停步并结束
            self.active = False
            self.publish_stop()
            self.get_logger().info("Support path goal reached")
            return

        self.current_index = self.find_progress_index(px, py) # 更新进度索引
        target_index = self.find_lookahead_index(px, py, self.current_index) # 前视点索引
        tx, ty, _ = self.path[target_index] # 前视点坐标
        dx = tx - px # 指向前视点的向量
        dy = ty - py

        desired_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-4 else yaw # 期望航向
        yaw_error = normalize_angle(desired_yaw - yaw) # 偏航误差（归一化）
        yaw_cmd = clamp(self.kp_yaw * yaw_error, -self.max_vyaw, self.max_vyaw) # 偏航角速度指令

        cmd = Twist()
        if abs(yaw_error) > self.rotate_in_place_yaw_error:
            # 偏航误差过大：先原地转向（只转不前进）
            cmd.angular.z = yaw_cmd
            self.cmd_pub.publish(cmd)
            return

        # 把指向前视点的向量转到机器人坐标系（body frame）
        c = math.cos(yaw)
        s = math.sin(yaw)
        x_body = c * dx + s * dy # 前方分量
        y_body = -s * dx + c * dy # 侧向分量

        if self.forward_only:
            cmd.linear.x = clamp(self.kp_xy * x_body, 0.0, self.max_vx) # 仅前向（不倒退）
        else:
            cmd.linear.x = clamp(self.kp_xy * x_body, -self.max_vx, self.max_vx) # 可前进后退
        cmd.linear.y = clamp(self.kp_xy * y_body, -self.max_vy, self.max_vy) # 横向速度
        cmd.angular.z = yaw_cmd # 偏航角速度
        self.cmd_pub.publish(cmd) # 发布指令

    def find_progress_index(self, px, py):
        # 函数：找到离当前位置最近的路径点索引（在上一索引附近局部搜索）
        # 参数：px, py - 当前位置；返回：最近的路径点索引
        start = max(0, self.current_index - 1) # 从上一索引前一位开始（防止倒退）
        best_index = self.current_index
        best_dist = float("inf")
        for index in range(start, len(self.path)):
            point = self.path[index]
            dist = math.hypot(point[0] - px, point[1] - py)
            if dist < best_dist:
                best_dist = dist
                best_index = index
        # 若前方路点已足够近（waypoint_reached_distance），继续推进进度
        while (
            best_index + 1 < len(self.path)
            and math.hypot(self.path[best_index][0] - px, self.path[best_index][1] - py)
            < self.waypoint_reached_distance
        ):
            best_index += 1
        return best_index

    def find_lookahead_index(self, px, py, start_index):
        # 函数：沿路径累计弧长，找到距当前位置约 lookahead_distance 的前视点
        # 参数：px, py - 当前位置；start_index - 起点索引；返回：前视点索引
        distance = math.hypot(self.path[start_index][0] - px, self.path[start_index][1] - py)
        last = self.path[start_index]
        for index in range(start_index + 1, len(self.path)):
            point = self.path[index]
            distance += math.hypot(point[0] - last[0], point[1] - last[1]) # 累计相邻路点间距
            if distance >= self.lookahead_distance:
                return index # 达到前视距离
            last = point
        return len(self.path) - 1 # 路径太短：返回最后一点


def main():
    rclpy.init()
    node = SupportPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop() # 退出前确保停车
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
