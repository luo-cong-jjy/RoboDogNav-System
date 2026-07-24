#!/usr/bin/env python3
"""Follow a traversability support path directly with /cmd_vel.

This is a Gazebo validation controller for legged M20 Building tests. It bypasses
SCAN-Planner's aerial-style B-spline collision optimizer and tracks /initial_path
as a ground/support-surface reference.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SupportPathFollower(Node):
    def __init__(self):
        super().__init__("m20_support_path_follower")
        self.declare_parameter("use_sim_time", False)
        self.lookahead_distance = self.declare_parameter("lookahead_distance", 0.70).value
        self.waypoint_reached_distance = self.declare_parameter(
            "waypoint_reached_distance", 0.35
        ).value
        self.finish_distance = self.declare_parameter("finish_distance", 0.40).value
        self.rotate_in_place_yaw_error = self.declare_parameter(
            "rotate_in_place_yaw_error", 0.75
        ).value
        self.kp_xy = self.declare_parameter("kp_xy", 0.85).value
        self.kp_yaw = self.declare_parameter("kp_yaw", 1.6).value
        self.max_vx = self.declare_parameter("max_vx", 0.45).value
        self.max_vy = self.declare_parameter("max_vy", 0.08).value
        self.max_vyaw = self.declare_parameter("max_vyaw", 0.8).value
        self.forward_only = self.declare_parameter("forward_only", True).value

        self.path = []
        self.current_index = 0
        self.odom = None
        self.active = False

        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(
            Path, "initial_path", self.path_callback, path_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 20)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Support path follower ready")

    def path_callback(self, msg):
        self.path = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in msg.poses
        ]
        self.current_index = 0
        self.active = len(self.path) >= 2
        if self.active:
            self.get_logger().info(
                "Received support path: %d waypoints, start_z=%.2f, goal_z=%.2f"
                % (len(self.path), self.path[0][2], self.path[-1][2])
            )
        else:
            self.get_logger().warning("Ignoring support path with fewer than 2 waypoints")

    def odom_callback(self, msg):
        self.odom = msg

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def timer_callback(self):
        if not self.active or self.odom is None:
            self.publish_stop()
            return

        pos = self.odom.pose.pose.position
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        px, py = pos.x, pos.y

        final = self.path[-1]
        if math.hypot(final[0] - px, final[1] - py) <= self.finish_distance:
            self.active = False
            self.publish_stop()
            self.get_logger().info("Support path goal reached")
            return

        self.current_index = self.find_progress_index(px, py)
        target_index = self.find_lookahead_index(px, py, self.current_index)
        tx, ty, _ = self.path[target_index]
        dx = tx - px
        dy = ty - py

        desired_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-4 else yaw
        yaw_error = normalize_angle(desired_yaw - yaw)
        yaw_cmd = clamp(self.kp_yaw * yaw_error, -self.max_vyaw, self.max_vyaw)

        cmd = Twist()
        if abs(yaw_error) > self.rotate_in_place_yaw_error:
            cmd.angular.z = yaw_cmd
            self.cmd_pub.publish(cmd)
            return

        c = math.cos(yaw)
        s = math.sin(yaw)
        x_body = c * dx + s * dy
        y_body = -s * dx + c * dy

        if self.forward_only:
            cmd.linear.x = clamp(self.kp_xy * x_body, 0.0, self.max_vx)
        else:
            cmd.linear.x = clamp(self.kp_xy * x_body, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(self.kp_xy * y_body, -self.max_vy, self.max_vy)
        cmd.angular.z = yaw_cmd
        self.cmd_pub.publish(cmd)

    def find_progress_index(self, px, py):
        start = max(0, self.current_index - 1)
        best_index = self.current_index
        best_dist = float("inf")
        for index in range(start, len(self.path)):
            point = self.path[index]
            dist = math.hypot(point[0] - px, point[1] - py)
            if dist < best_dist:
                best_dist = dist
                best_index = index
        while (
            best_index + 1 < len(self.path)
            and math.hypot(self.path[best_index][0] - px, self.path[best_index][1] - py)
            < self.waypoint_reached_distance
        ):
            best_index += 1
        return best_index

    def find_lookahead_index(self, px, py, start_index):
        distance = math.hypot(self.path[start_index][0] - px, self.path[start_index][1] - py)
        last = self.path[start_index]
        for index in range(start_index + 1, len(self.path)):
            point = self.path[index]
            distance += math.hypot(point[0] - last[0], point[1] - last[1])
            if distance >= self.lookahead_distance:
                return index
            last = point
        return len(self.path) - 1


def main():
    rclpy.init()
    node = SupportPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
