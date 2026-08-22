#!/usr/bin/env python3
"""Interactively record odometry positions as ROS 2 SCAN-Planner parameters."""
# ============================================================
# 文件职责：交互式路点录制工具（keypoint_recorder）。
#   订阅里程计话题，读取最新位姿，把当前机器人位置记录为路点，
#   最终保存为 SCAN-Planner 可直接加载的 ROS 2 参数 YAML
#   （scan_planner_node/ros__parameters/fsm.waypoints）。
#   键盘控制：Enter/Space/a 添加、r 替换、d 删除、u 撤销、
#   l 列表、s 保存、q 保存并退出。
# ============================================================

import argparse
import math
import os
import select
import sys
import termios
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args


DEFAULT_OUTPUT = os.path.join(os.getcwd(), "keypoints.yaml") # 默认输出文件：当前目录 keypoints.yaml


def format_float(value):
    # 函数：把浮点数格式化为 YAML 中的紧凑字符串（去掉多余尾零与小数点）
    # 参数：value - 待格式化的数值；返回：格式化后的字符串
    text = f"{float(value):.6f}".rstrip("0").rstrip(".") # 保留 6 位小数后去尾零
    return "0" if text in ("", "-0") else text # 处理 "0" 与 "-0" 的边界情况


def atomic_write(path, content):
    # 函数：原子写文件——先写临时文件再 os.replace，避免写一半损坏原文件
    # 参数：path - 目标路径；content - 要写入的内容
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True) # 确保目录存在
    temporary = path + ".tmp" # 临时文件路径
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(temporary, path) # 原子替换（同目录下 rename）


class KeypointRecorder(Node):
    def __init__(self, odom_topic, output_path):
        # 构造函数：创建 ROS 2 节点并订阅里程计
        # 参数：odom_topic - 里程计话题名；output_path - 输出 YAML 路径
        super().__init__("keypoint_recorder")
        self.odom_topic = odom_topic
        self.output_path = os.path.abspath(output_path) # 归一化为绝对路径
        self.latest_odom = None # 最新一帧里程计
        self.waypoints = []     # 已录制的路点列表 [(x, y, z), ...]
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)

    def odom_callback(self, message):
        # 回调：仅缓存最新里程计消息
        self.latest_odom = message

    def current_point(self):
        # 函数：取最新里程计的位置作为当前点（校验数值有限）
        # 返回：None（无里程计或含 NaN/Inf）或 (x, y, z) 元组
        if self.latest_odom is None:
            return None
        position = self.latest_odom.pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        return point if all(math.isfinite(value) for value in point) else None

    def record_current(self):
        # 函数：把当前位置追加为路点
        point = self.current_point()
        if point is None:
            self.get_logger().warning("No valid odometry received yet") # 无有效里程计
            return
        self.waypoints.append(point)
        self.get_logger().info(f"Recorded waypoint {len(self.waypoints)}: {point}")

    def replace_current(self, index):
        # 函数：用当前位置替换指定序号（1 起始）的路点
        point = self.current_point()
        if point is None or not 1 <= index <= len(self.waypoints):
            self.get_logger().warning("Invalid waypoint index or odometry")
            return
        self.waypoints[index - 1] = point

    def delete_waypoint(self, index):
        # 函数：删除指定序号（1 起始）的路点
        if 1 <= index <= len(self.waypoints):
            self.waypoints.pop(index - 1)
        else:
            self.get_logger().warning("Invalid waypoint index")

    def build_yaml(self):
        # 函数：把路点列表序列化为 SCAN-Planner 参数 YAML 文本
        # 返回：YAML 字符串（fsm.waypoints 为扁平化的 x,y,z 数组）
        values = [format_float(value) for point in self.waypoints for value in point] # 展平为数值列表
        return (
            "scan_planner_node:\n"
            "  ros__parameters:\n"
            f"    fsm.waypoints: [{', '.join(values)}]\n" # 路点数组写入参数
        )

    def save(self):
        # 函数：把路点原子写入输出文件
        atomic_write(self.output_path, self.build_yaml())
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoint(s) to {self.output_path}")

    def prompt_index(self, settings, action):
        # 函数：在终端提示用户输入路点序号（临时恢复行缓冲以支持 input）
        # 参数：settings - 原终端设置；action - 提示动作名（Replace/Delete）
        # 返回：用户输入的序号；输入非法时返回 -1
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # 恢复标准行缓冲
        try:
            return int(input(f"{action} waypoint index (1-{len(self.waypoints)}): ").strip())
        except ValueError:
            return -1 # 非整数输入
        finally:
            tty.setcbreak(sys.stdin.fileno()) # 重新进入 cbreak 模式（单字符按键）

    def run(self):
        # 函数：主循环——在单字符键盘模式下处理按键与 ROS 回调
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard control requires a TTY") # 需要终端
        print("Enter/Space/a: add, r: replace, d: delete, u: undo, l: list, s: save, q: save+quit")
        settings = termios.tcgetattr(sys.stdin) # 保存原终端设置
        tty.setcbreak(sys.stdin.fileno())       # 进入 cbreak 模式（无需回车即可读取按键）
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05) # 非阻塞处理 ROS 回调
                ready, _, _ = select.select([sys.stdin], [], [], 0.0) # 检查是否有按键输入
                if not ready:
                    continue
                key = sys.stdin.read(1).lower() # 读取单个按键
                if key in ("\n", "\r", " ", "a"):
                    self.record_current() # 添加当前点
                elif key == "r":
                    self.replace_current(self.prompt_index(settings, "Replace")) # 替换指定点
                elif key == "d":
                    self.delete_waypoint(self.prompt_index(settings, "Delete")) # 删除指定点
                elif key == "u" and self.waypoints:
                    self.waypoints.pop() # 撤销最后一点
                elif key == "l":
                    print("\n".join(f"{i}: {point}" for i, point in enumerate(self.waypoints, 1))) # 列出全部路点
                elif key == "s":
                    self.save() # 保存
                elif key in ("q", "\x03"): # q 或 Ctrl-C
                    self.save() # 保存并退出
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # 恢复终端设置


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom", default="/LIO/odom_vehicle") # 里程计话题
    parser.add_argument("--output", default=DEFAULT_OUTPUT)    # 输出文件
    arguments = parser.parse_args(remove_ros_args(args=sys.argv)[1:]) # 剥离 ROS 参数后解析
    rclpy.init(args=sys.argv) # 初始化 ROS 2
    recorder = KeypointRecorder(arguments.odom, arguments.output)
    try:
        recorder.run()
    except (KeyboardInterrupt, RuntimeError) as error:
        recorder.get_logger().error(str(error))
    finally:
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
