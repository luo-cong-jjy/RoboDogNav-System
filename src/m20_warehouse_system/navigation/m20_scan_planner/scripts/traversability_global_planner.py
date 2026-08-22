#!/usr/bin/env python3
"""Lightweight traversability-aware global planner for Building PCD maps.

This node converts a static surface PCD into a support-surface graph, snaps
RViz goals onto reachable support surfaces, and publishes /initial_path for
SCAN-Planner navi_mode=3.
"""
# ============================================================
# 文件职责：轻量级"可通行性感知"全局规划器（供楼宇 PCD 地图使用）。
#   1. 把静态表面 PCD 地图转成"支撑面图"（SupportGraph）：
#      按 XY 栅格 + 高度分箱统计点云，筛选出满足支撑条件的可站立层；
#   2. 把 RViz 目标点吸附到可达的支撑面上（按 XY 半径与 Z 容差搜索）；
#   3. 在支撑面图上做 A* 搜索，输出下采样后的路径点，发布 /initial_path
#      供 SCAN-Planner 的 navi_mode=3（楼宇地面导航）使用。
#   支持 --no-ros 命令行模式（纯离线计算）与 ROS 2 节点模式。
# ============================================================

import argparse
import heapq
import math
import os
import time
from collections import defaultdict
from dataclasses import dataclass

import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


@dataclass(frozen=True)
class SupportNode:
    # 数据类：支撑图节点（一个可站立的栅格-高度组合）
    ix: int          # 栅格 x 索引
    iy: int          # 栅格 y 索引
    z: float         # 支撑面高度
    standability: float # 可站立性（支撑邻域比例，0~1）


@dataclass
class PlannerConfig:
    # 数据类：规划器配置（全部带默认值，可在节点参数/命令行中覆盖）
    grid_resolution: float = 0.20 # 栅格分辨率（米）
    height_bin_resolution: float = 0.08 # 高度分箱分辨率（米）
    min_points_per_height_bin: int = 2 # 每个高度箱最少点数（过滤噪点）
    standable_radius: float = 0.35 # 支撑判定半径（米）
    standable_min_ratio: float = 0.20 # 支撑邻域最小比例
    standable_height_tolerance: float = 0.12 # 支撑高度容差（米）
    step_max: float = 0.30 # 相邻节点最大高度差（可跨越步高）
    max_edge_slope: float = 1.20 # 最大边坡度（高度差/水平距离）
    snap_xy_radius: float = 0.75 # 目标吸附的 XY 搜索半径
    snap_z_tolerance: float = 0.75 # 目标吸附的 Z 容差
    body_height: float = 0.40 # 机身高度（用于由机身 z 反推支撑面 z）
    explicit_goal_z_threshold: float = 0.20 # 显式 Z 目标判定阈值
    goal_layer_policy: str = "nearest_to_start_height" # 目标层选择策略
    waypoint_spacing: float = 0.35 # 输出路点水平间距
    vertical_waypoint_spacing: float = 0.25 # 输出路点垂直间距
    max_output_waypoints: int = 28 # 输出路点数量上限
    max_goal_candidates: int = 16 # 目标候选数上限
    vertical_weight: float = 1.40 # 代价函数中高度差的权重


class SupportGraph:
    def __init__(self, points: np.ndarray, cfg: PlannerConfig):
        # 构造函数：从点云构建支撑面图
        # 参数：points - Nx3 点云（x y z）；cfg - 规划配置
        self.cfg = cfg
        self.points_min = points.min(axis=0) # 点云包围盒最小值
        self.points_max = points.max(axis=0) # 点云包围盒最大值
        self.origin_xy = self.points_min[:2] - cfg.grid_resolution # 栅格原点（留一个栅格余量）
        self.nodes = [] # 支撑节点列表
        self.cell_to_nodes = defaultdict(list) # 栅格坐标 -> 节点 ID 列表
        self._neighbor_cache = {} # 邻居缓存（避免重复计算）
        self._build(points) # 构建支撑图

    @classmethod
    def from_pcd(cls, pcd_path: str, cfg: PlannerConfig):
        # 类方法：直接从 ASCII PCD 文件构建支撑图
        return cls(load_ascii_pcd_xyz(pcd_path), cfg)

    def _cell_index(self, x: float, y: float):
        # 函数：由世界坐标计算栅格索引
        # 返回：(ix, iy) 整数栅格索引
        res = self.cfg.grid_resolution
        return (
            int(math.floor((x - self.origin_xy[0]) / res)),
            int(math.floor((y - self.origin_xy[1]) / res)),
        )

    def _cell_center(self, ix: int, iy: int):
        # 函数：由栅格索引计算栅格中心的 XY 坐标
        res = self.cfg.grid_resolution
        return np.array(
            [
                self.origin_xy[0] + (ix + 0.5) * res,
                self.origin_xy[1] + (iy + 0.5) * res,
            ],
            dtype=float,
        )

    def node_position(self, node_id: int):
        # 函数：返回节点 ID 对应的三维位置（栅格中心 XY + 支撑高度 Z）
        node = self.nodes[node_id]
        xy = self._cell_center(node.ix, node.iy)
        return np.array([xy[0], xy[1], node.z], dtype=float)

    def _build(self, points: np.ndarray):
        # 函数：从点云构建支撑面图（核心建图逻辑）
        cfg = self.cfg
        raw_bins = defaultdict(lambda: defaultdict(lambda: [0, 0.0])) # (ix,iy) -> zbin -> [点数, 高度和]

        # ---- 1. 统计每个栅格内各高度分箱的点数与高度和 ----
        for x, y, z in points:
            ix, iy = self._cell_index(float(x), float(y)) # 栅格索引
            zbin = int(round(float(z) / cfg.height_bin_resolution)) # 高度分箱
            bucket = raw_bins[(ix, iy)][zbin]
            bucket[0] += 1 # 点数累加
            bucket[1] += float(z) # 高度累加

        # ---- 2. 合并各分箱为候选高度（按平均高度，剔除点数不足的分箱） ----
        raw_heights = {}
        for cell, bins in raw_bins.items():
            heights = []
            for count, zsum in bins.values():
                if count >= cfg.min_points_per_height_bin: # 点数足够才保留
                    heights.append(zsum / count) # 平均高度
            if heights:
                # 合并相近高度（同一平面被分成多个分箱时合并）
                raw_heights[cell] = sorted(merge_close_values(heights, cfg.height_bin_resolution))

        # ---- 3. 支撑判定核：统计邻域内高度接近的栅格数量 ----
        kernel_radius = max(1, int(math.ceil(cfg.standable_radius / cfg.grid_resolution)))
        kernel_offsets = [
            (dx, dy)
            for dx in range(-kernel_radius, kernel_radius + 1)
            for dy in range(-kernel_radius, kernel_radius + 1)
            if math.hypot(dx * cfg.grid_resolution, dy * cfg.grid_resolution)
            <= cfg.standable_radius + 1e-6 # 只在支撑半径内的邻域
        ]
        min_support = max(3, int(math.ceil(len(kernel_offsets) * cfg.standable_min_ratio))) # 最小支撑栅格数

        # ---- 4. 生成节点：满足支撑条件的 (栅格, 高度) 才成为节点 ----
        for (ix, iy), heights in raw_heights.items():
            for z in heights:
                support = 0 # 邻域内支撑栅格计数
                for dx, dy in kernel_offsets:
                    neighbor_heights = raw_heights.get((ix + dx, iy + dy))
                    if not neighbor_heights:
                        continue
                    # 邻域内有高度接近的候选层，视为对该高度的支撑
                    if any(abs(nz - z) <= cfg.standable_height_tolerance for nz in neighbor_heights):
                        support += 1

                if support < min_support:
                    continue # 支撑不足：该层不可站立

                node_id = len(self.nodes)
                # 可站立性 = 支撑邻域比例（用于代价惩罚）
                self.nodes.append(SupportNode(ix, iy, float(z), support / len(kernel_offsets)))
                self.cell_to_nodes[(ix, iy)].append(node_id) # 登记栅格索引

    def nearest_candidates(self, x: float, y: float, z_hint=None, radius=None, max_count=40):
        # 函数：在指定 XY 位置附近搜索最近的节点候选（按距离/高度差/可站立性打分）
        # 参数：x, y - 查询位置；z_hint - 期望高度（None 则忽略高度过滤）；
        #       radius - 搜索半径（默认用配置值）；max_count - 返回候选数上限
        # 返回：按分数升序排列的节点 ID 列表
        cfg = self.cfg
        radius = cfg.snap_xy_radius if radius is None else radius
        cell_radius = max(1, int(math.ceil(radius / cfg.grid_resolution))) # 搜索的栅格半径
        ix, iy = self._cell_index(x, y)
        candidates = []

        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                for node_id in self.cell_to_nodes.get((ix + dx, iy + dy), []):
                    pos = self.node_position(node_id)
                    dxy = float(np.linalg.norm(pos[:2] - np.array([x, y]))) # XY 距离
                    if dxy > radius:
                        continue
                    dz = 0.0 if z_hint is None else abs(pos[2] - z_hint) # Z 偏差
                    if z_hint is not None and dz > cfg.snap_z_tolerance:
                        continue # 超出 Z 容差
                    # 打分：XY 距离为主，高度偏差次之，可站立性加分
                    score = dxy + 0.35 * dz - 0.05 * self.nodes[node_id].standability
                    candidates.append((score, node_id))

        candidates.sort() # 按分数排序
        return [node_id for _, node_id in candidates[:max_count]]

    def neighbors(self, node_id: int):
        # 函数：返回节点在 8 邻域内的可行邻居及其边代价（带缓存）
        # 返回：[(邻居节点 ID, 边代价), ...]
        if node_id in self._neighbor_cache:
            return self._neighbor_cache[node_id] # 命中缓存

        cfg = self.cfg
        src = self.nodes[node_id]
        result = []

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue # 跳过自身
                horiz = math.hypot(dx * cfg.grid_resolution, dy * cfg.grid_resolution) # 水平距离
                if horiz <= 1e-8:
                    continue
                for dst_id in self.cell_to_nodes.get((src.ix + dx, src.iy + dy), []):
                    dst = self.nodes[dst_id]
                    dz = abs(dst.z - src.z) # 高度差
                    if dz > cfg.step_max:
                        continue # 超过可跨越步高
                    if dz / horiz > cfg.max_edge_slope:
                        continue # 坡度过大
                    cost = horiz + cfg.vertical_weight * dz # 距离代价 + 垂直代价
                    cost += 0.05 * (2.0 - src.standability - dst.standability) # 可站立性惩罚
                    result.append((dst_id, cost))

        self._neighbor_cache[node_id] = result # 写入缓存
        return result

    def astar(self, start_id: int, goal_id: int):
        # 函数：A* 搜索从 start_id 到 goal_id 的最短路径
        # 参数：start_id, goal_id - 起终点节点 ID
        # 返回：(路径节点 ID 列表, 总代价)；不可达时返回 (None, inf)
        goal_pos = self.node_position(goal_id)
        open_heap = [(0.0, 0.0, start_id)] # 优先队列：(f, g, node)
        parent = {start_id: None} # 父节点记录（用于回溯路径）
        g_score = {start_id: 0.0} # 起点到各节点的实际代价

        while open_heap:
            _, current_g, current = heapq.heappop(open_heap) # 弹出 f 最小的节点
            if current == goal_id:
                return reconstruct_path(parent, current), current_g # 到达终点
            if current_g > g_score.get(current, float("inf")) + 1e-9:
                continue # 该记录已过期（g 值被更新过）

            for neighbor, step_cost in self.neighbors(current):
                tentative = current_g + step_cost # 候选新 g 值
                if tentative >= g_score.get(neighbor, float("inf")):
                    continue # 不是更优路径
                g_score[neighbor] = tentative
                parent[neighbor] = current
                heuristic = float(np.linalg.norm(self.node_position(neighbor) - goal_pos)) # 欧氏距离启发
                heapq.heappush(open_heap, (tentative + heuristic, tentative, neighbor)) # f = g + h

        return None, float("inf") # 开放集耗尽：不可达

    def plan(self, start_xyz, goal_xyz, goal_has_explicit_z=False):
        # 函数：完整规划流程——找起终点候选、选目标层、A* 搜索、下采样
        # 参数：start_xyz - 起点 (x, y, z)；goal_xyz - 目标点 (x, y[, z])；
        #       goal_has_explicit_z - 目标是否显式指定了 Z
        # 返回：(下采样后的路径点列表, 信息字典)；失败时抛 RuntimeError
        cfg = self.cfg
        start_support_z = float(start_xyz[2]) - cfg.body_height # 由机身高度反推支撑面高度
        start_candidates = self.nearest_candidates(
            float(start_xyz[0]), float(start_xyz[1]), start_support_z
        )
        if not start_candidates:
            # 高度提示无候选：放宽为任意高度再搜
            start_candidates = self.nearest_candidates(float(start_xyz[0]), float(start_xyz[1]), None)
        if not start_candidates:
            raise RuntimeError("No support-surface candidate near robot start pose")

        goal_z_hint = float(goal_xyz[2]) if goal_has_explicit_z else None # 目标高度提示
        max_goal_candidates = max(1, cfg.max_goal_candidates)
        goal_candidates = self.nearest_candidates(
            float(goal_xyz[0]), float(goal_xyz[1]), goal_z_hint, max_count=max_goal_candidates
        )
        if not goal_candidates:
            # 目标高度无候选：放宽
            goal_candidates = self.nearest_candidates(
                float(goal_xyz[0]), float(goal_xyz[1]), None, max_count=max_goal_candidates
            )
        if not goal_candidates:
            raise RuntimeError("No support-surface candidate near requested goal")

        start_id = start_candidates[0] # 取最近起点候选
        policy = cfg.goal_layer_policy

        # ---- 按策略对目标候选排序（决定优先尝试哪一层） ----
        if goal_has_explicit_z:
            ordered_goals = goal_candidates # 显式 Z：保持距离排序
        elif policy == "highest":
            ordered_goals = sorted(goal_candidates, key=lambda node_id: self.nodes[node_id].z, reverse=True) # 最高层优先
        elif policy == "lowest":
            ordered_goals = sorted(goal_candidates, key=lambda node_id: self.nodes[node_id].z) # 最低层优先
        elif policy == "nearest_to_start_height":
            start_z = self.nodes[start_id].z
            ordered_goals = sorted(goal_candidates, key=lambda node_id: abs(self.nodes[node_id].z - start_z)) # 最接近起点高度优先
        else:
            ordered_goals = goal_candidates

        best_path = None
        best_cost = float("inf")
        best_goal = None

        if policy == "min_path_cost" and not goal_has_explicit_z:
            candidates_to_try = ordered_goals[:max_goal_candidates]
        else:
            candidates_to_try = ordered_goals[:max_goal_candidates]

        # ---- 依次尝试目标候选，保留代价最小的路径 ----
        for goal_id in candidates_to_try:
            path, cost = self.astar(start_id, goal_id) # A* 搜索
            if path is not None and cost < best_cost:
                best_path = path
                best_cost = cost
                best_goal = goal_id
                if policy != "min_path_cost" or goal_has_explicit_z:
                    break # 非 min_path_cost 策略：第一个可达目标即可

        if best_path is None:
            raise RuntimeError("No connected traversable support path to requested goal")

        # ---- 下采样输出路径点并返回 ----
        return downsample_path(
            [self.node_position(node_id) for node_id in best_path], # 路径节点转三维坐标
            cfg.waypoint_spacing,
            cfg.vertical_waypoint_spacing,
            cfg.max_output_waypoints,
        ), {
            "start_id": start_id,
            "goal_id": best_goal,
            "cost": best_cost,
            "raw_nodes": len(best_path), # 原始节点数（未下采样）
        }


class TraversabilityGlobalPlannerNode(Node):
    def __init__(self):
        # 构造函数：声明参数、构建支撑图、创建订阅/发布器
        super().__init__("m20_traversability_global_planner")
        # ---- 声明全部参数（与 PlannerConfig 对应） ----
        self.declare_parameter("pcd_map_file", "") # 静态表面 PCD 地图路径
        self.declare_parameter("frame_id", "world") # 输出路径坐标系
        self.declare_parameter("grid_resolution", 0.20)
        self.declare_parameter("height_bin_resolution", 0.08)
        self.declare_parameter("min_points_per_height_bin", 2)
        self.declare_parameter("standable_radius", 0.35)
        self.declare_parameter("standable_min_ratio", 0.20)
        self.declare_parameter("standable_height_tolerance", 0.12)
        self.declare_parameter("step_max", 0.30)
        self.declare_parameter("max_edge_slope", 1.20)
        self.declare_parameter("snap_xy_radius", 0.75)
        self.declare_parameter("snap_z_tolerance", 0.75)
        self.declare_parameter("body_height", 0.40)
        self.declare_parameter("explicit_goal_z_threshold", 0.20)
        self.declare_parameter("goal_layer_policy", "nearest_to_start_height")
        self.declare_parameter("waypoint_spacing", 0.35)
        self.declare_parameter("vertical_waypoint_spacing", 0.25)
        self.declare_parameter("max_output_waypoints", 28)
        self.declare_parameter("max_goal_candidates", 16)
        self.declare_parameter("vertical_weight", 1.40)

        self.frame_id = self.get_parameter("frame_id").value # 坐标系
        # ---- 把参数组装成 PlannerConfig ----
        self.cfg = PlannerConfig(
            grid_resolution=float(self.get_parameter("grid_resolution").value),
            height_bin_resolution=float(self.get_parameter("height_bin_resolution").value),
            min_points_per_height_bin=int(self.get_parameter("min_points_per_height_bin").value),
            standable_radius=float(self.get_parameter("standable_radius").value),
            standable_min_ratio=float(self.get_parameter("standable_min_ratio").value),
            standable_height_tolerance=float(self.get_parameter("standable_height_tolerance").value),
            step_max=float(self.get_parameter("step_max").value),
            max_edge_slope=float(self.get_parameter("max_edge_slope").value),
            snap_xy_radius=float(self.get_parameter("snap_xy_radius").value),
            snap_z_tolerance=float(self.get_parameter("snap_z_tolerance").value),
            body_height=float(self.get_parameter("body_height").value),
            explicit_goal_z_threshold=float(self.get_parameter("explicit_goal_z_threshold").value),
            goal_layer_policy=str(self.get_parameter("goal_layer_policy").value),
            waypoint_spacing=float(self.get_parameter("waypoint_spacing").value),
            vertical_waypoint_spacing=float(
                self.get_parameter("vertical_waypoint_spacing").value
            ),
            max_output_waypoints=int(self.get_parameter("max_output_waypoints").value),
            max_goal_candidates=int(self.get_parameter("max_goal_candidates").value),
            vertical_weight=float(self.get_parameter("vertical_weight").value),
        )

        # ---- 加载 PCD 地图并构建支撑图 ----
        pcd_map_file = os.path.expanduser(str(self.get_parameter("pcd_map_file").value)) # 展开 ~
        if not pcd_map_file or not os.path.isfile(pcd_map_file):
            raise RuntimeError(f"pcd_map_file does not exist: {pcd_map_file}") # 地图文件不存在

        self.get_logger().info(f"Building traversability support graph from {pcd_map_file}")
        self.graph = SupportGraph.from_pcd(pcd_map_file, self.cfg) # 构建支撑图
        self.get_logger().info(
            "Support graph ready: %d nodes, %d occupied XY cells, bounds min=%s max=%s"
            % (
                len(self.graph.nodes),
                len(self.graph.cell_to_nodes),
                np.array2string(self.graph.points_min, precision=2),
                np.array2string(self.graph.points_max, precision=2),
            )
        )

        self.odom = None # 最新里程计
        # 路径使用 TRANSIENT_LOCAL 持久化 QoS（晚订阅的消费者也能拿到最新路径）
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "initial_path", latching_qos) # 发布规划路径
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 10 # 订阅里程计（获取起点）
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "move_base_simple/goal", self.goal_callback, 10 # 2D 导航目标（RViz）
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped, "clicked_point", self.clicked_point_callback, 10 # 3D 点击点目标
        )

    def odom_callback(self, msg: Odometry):
        # 回调：缓存最新里程计
        self.odom = msg

    def goal_callback(self, msg: PoseStamped):
        # 回调：收到 2D 导航目标（move_base_simple/goal）
        goal = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        has_z = goal[2] > self.cfg.explicit_goal_z_threshold # Z 高于阈值视为显式指定高度
        self.plan_and_publish(goal, has_z)

    def clicked_point_callback(self, msg: PointStamped):
        # 回调：收到 3D 点击点（clicked_point，RViz 中显式点击带高度）
        goal = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.plan_and_publish(goal, True) # 点击点始终视为显式 Z

    def plan_and_publish(self, goal_xyz, goal_has_explicit_z):
        # 函数：执行规划并发布路径（供两个目标回调共用）
        # 参数：goal_xyz - 目标点；goal_has_explicit_z - 是否显式指定 Z
        if self.odom is None:
            self.get_logger().warning("Ignore goal before receiving body_pose") # 尚无里程计
            return

        p = self.odom.pose.pose.position
        start = np.array([p.x, p.y, p.z], dtype=float) # 起点 = 当前里程计位置
        t0 = time.perf_counter() # 计时开始
        try:
            path_points, info = self.graph.plan(start, goal_xyz, goal_has_explicit_z) # 规划
        except RuntimeError as exc:
            self.get_logger().error(str(exc)) # 规划失败：记录错误
            return
        elapsed = time.perf_counter() - t0 # 规划耗时

        # ---- 把路径点封装为 Path 消息 ----
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0 # 姿态默认单位四元数（朝向由跟踪器处理）
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg) # 发布路径
        # 打印规划结果摘要
        start_node = self.graph.nodes[info["start_id"]]
        goal_node = self.graph.nodes[info["goal_id"]]
        self.get_logger().info(
            "Published traversability initial_path: %d waypoints, raw_nodes=%d, cost=%.2f, plan_time=%.3fs, "
            "start_support_z=%.2f, goal_support_z=%.2f"
            % (
                len(path_msg.poses),
                info["raw_nodes"],
                info["cost"],
                elapsed,
                start_node.z,
                goal_node.z,
            )
        )


def load_ascii_pcd_xyz(path: str):
    # 函数：读取 ASCII XYZ PCD 文件的前三列（x y z）
    # 参数：path - PCD 文件路径；返回：Nx3 float32 数组
    # 要求：FIELDS 行必须以 x y z 开头，DATA 必须是 ascii
    points = []
    data = False # 是否已进入数据段
    fields = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue # 跳过空行
            lower = stripped.lower()
            if data:
                # 数据段：取前三个数值作为 x y z
                values = stripped.split()
                if len(values) < 3:
                    continue
                points.append((float(values[0]), float(values[1]), float(values[2])))
            elif lower.startswith("fields"):
                fields = stripped.split()[1:] # 字段名列表
                if fields[:3] != ["x", "y", "z"]:
                    raise RuntimeError(f"Only PCD fields starting with x y z are supported: {fields}")
            elif lower.startswith("data"):
                if "ascii" not in lower:
                    raise RuntimeError("Only ASCII PCD files are supported by this lightweight planner")
                data = True # 进入数据段

    if not points:
        raise RuntimeError(f"No XYZ points loaded from {path}") # 无点数据
    return np.asarray(points, dtype=np.float32)


def merge_close_values(values, tolerance):
    # 函数：把彼此间距在 tolerance 以内的数值合并（取平均值）
    # 参数：values - 升序数值列表；tolerance - 合并容差
    # 返回：合并后的数值列表（仍升序）
    if not values:
        return values
    merged = []
    current = [values[0]] # 当前簇
    for value in values[1:]:
        if abs(value - current[-1]) <= tolerance:
            current.append(value) # 属于当前簇
        else:
            merged.append(float(sum(current) / len(current))) # 结束一个簇（取均值）
            current = [value]
    merged.append(float(sum(current) / len(current))) # 处理最后一个簇
    return merged


def reconstruct_path(parent, current):
    # 函数：从 parent 字典回溯得到从起点到 current 的路径
    # 参数：parent - 节点 -> 父节点 映射；current - 终点节点
    # 返回：路径节点列表（起点 -> 终点）
    path = [current]
    while parent[current] is not None:
        current = parent[current]
        path.append(current)
    path.reverse() # 反转得到正向路径
    return path


def downsample_path(points, spacing, vertical_spacing, max_points):
    # 函数：对路径点做下采样——保留满足水平/垂直间距的路点
    # 参数：points - 路径点列表；spacing - 水平间距；vertical_spacing - 垂直间距
    #       max_points - 最终点数上限
    # 返回：下采样（并截断）后的路径点列表
    if not points:
        return []
    result = [points[0]] # 始终保留起点
    last = points[0]
    for point in points[1:-1]: # 中间点按间距筛选
        if (
            np.linalg.norm(point[:2] - last[:2]) >= spacing # 水平移动达到间距
            or abs(point[2] - last[2]) >= vertical_spacing # 或高度变化达到间距
        ):
            result.append(point)
            last = point
    if len(points) > 1:
        result.append(points[-1]) # 始终保留终点
    return limit_polyline_points(result, max_points) # 若仍超限，按弧长均匀取点


def limit_polyline_points(points, max_points):
    # 函数：把折线均匀重采样到不超过 max_points 个点
    # 参数：points - 路径点列表；max_points - 点数上限
    # 返回：重采样后的点列表
    if max_points <= 1 or len(points) <= max_points:
        return points # 无需处理

    # 计算累计弧长
    cumulative = [0.0]
    for prev, current in zip(points[:-1], points[1:]):
        cumulative.append(cumulative[-1] + float(np.linalg.norm(current - prev)))

    total = cumulative[-1] # 总弧长
    if total <= 1e-6:
        return [points[0], points[-1]] # 路径退化：只保留起终点

    targets = np.linspace(0.0, total, max_points) # 目标弧长（等间隔）
    result = []
    seg = 0
    for target in targets:
        while seg + 1 < len(cumulative) and cumulative[seg + 1] < target:
            seg += 1 # 找到目标弧长所在的线段
        if seg + 1 >= len(points):
            result.append(points[-1])
            continue
        # 在线段内按比例线性插值
        denom = cumulative[seg + 1] - cumulative[seg]
        ratio = 0.0 if denom <= 1e-9 else (target - cumulative[seg]) / denom
        result.append(points[seg] * (1.0 - ratio) + points[seg + 1] * ratio)

    result[0] = points[0] # 修正端点
    result[-1] = points[-1]
    return result


def run_cli(args):
    # 函数：命令行（--no-ros）离线规划入口
    # 参数：args - 解析后的命令行参数
    cfg = PlannerConfig(
        grid_resolution=args.grid_resolution,
        goal_layer_policy=args.goal_layer_policy,
        step_max=args.step_max,
        max_edge_slope=args.max_edge_slope,
        waypoint_spacing=args.waypoint_spacing,
        vertical_waypoint_spacing=args.vertical_waypoint_spacing,
        max_output_waypoints=args.max_output_waypoints,
        max_goal_candidates=args.max_goal_candidates,
    )
    graph = SupportGraph.from_pcd(args.pcd, cfg) # 加载 PCD 构建支撑图
    goal = np.array(args.goal, dtype=float) # 目标点
    goal_has_z = len(args.goal) == 3 # 目标是否含 Z
    start = np.array(args.start, dtype=float) # 起点
    points, info = graph.plan(start, goal, goal_has_z) # 规划
    # 打印规划结果
    print(f"support_nodes: {len(graph.nodes)}")
    print(f"xy_cells: {len(graph.cell_to_nodes)}")
    print(f"raw_nodes: {info['raw_nodes']}")
    print(f"waypoints: {len(points)}")
    print(f"cost: {info['cost']:.3f}")
    print(f"start_support_z: {graph.nodes[info['start_id']].z:.3f}")
    print(f"goal_support_z: {graph.nodes[info['goal_id']].z:.3f}")
    for point in points[:20]: # 打印前 20 个路点
        print(f"{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}")
    if len(points) > 20:
        print("...")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-ros", action="store_true") # 离线模式（无需 ROS）
    parser.add_argument("--pcd") # PCD 地图路径（离线模式必填）
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "BODY_Z")) # 起点（机身 z）
    parser.add_argument("--goal", nargs="+", type=float, metavar="VALUE") # 目标（2 或 3 个数值）
    parser.add_argument("--grid-resolution", type=float, default=0.20)
    parser.add_argument("--goal-layer-policy", default="nearest_to_start_height")
    parser.add_argument("--step-max", type=float, default=0.30)
    parser.add_argument("--max-edge-slope", type=float, default=1.20)
    parser.add_argument("--waypoint-spacing", type=float, default=0.35)
    parser.add_argument("--vertical-waypoint-spacing", type=float, default=0.25)
    parser.add_argument("--max-output-waypoints", type=int, default=28)
    parser.add_argument("--max-goal-candidates", type=int, default=16)
    args, ros_args = parser.parse_known_args() # 未知参数留给 ROS

    if args.no_ros:
        # 离线模式：校验参数后直接规划
        if not args.pcd or not args.start or not args.goal or len(args.goal) not in (2, 3):
            parser.error("--no-ros requires --pcd, --start X Y BODY_Z, and --goal X Y [Z]")
        run_cli(args)
        return

    # ROS 2 节点模式
    rclpy.init(args=ros_args)
    node = TraversabilityGlobalPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
