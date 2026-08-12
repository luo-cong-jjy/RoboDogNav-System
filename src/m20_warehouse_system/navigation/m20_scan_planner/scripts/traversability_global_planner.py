#!/usr/bin/env python3
"""Lightweight traversability-aware global planner for Building PCD maps.

This node converts a static surface PCD into a support-surface graph, snaps
RViz goals onto reachable support surfaces, and publishes /initial_path for
SCAN-Planner navi_mode=3.
"""

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
    ix: int
    iy: int
    z: float
    standability: float


@dataclass
class PlannerConfig:
    grid_resolution: float = 0.20
    height_bin_resolution: float = 0.08
    min_points_per_height_bin: int = 2
    standable_radius: float = 0.35
    standable_min_ratio: float = 0.20
    standable_height_tolerance: float = 0.12
    step_max: float = 0.30
    max_edge_slope: float = 1.20
    snap_xy_radius: float = 0.75
    snap_z_tolerance: float = 0.75
    body_height: float = 0.40
    explicit_goal_z_threshold: float = 0.20
    goal_layer_policy: str = "nearest_to_start_height"
    waypoint_spacing: float = 0.35
    vertical_waypoint_spacing: float = 0.25
    max_output_waypoints: int = 28
    max_goal_candidates: int = 16
    vertical_weight: float = 1.40


class SupportGraph:
    def __init__(self, points: np.ndarray, cfg: PlannerConfig):
        self.cfg = cfg
        self.points_min = points.min(axis=0)
        self.points_max = points.max(axis=0)
        self.origin_xy = self.points_min[:2] - cfg.grid_resolution
        self.nodes = []
        self.cell_to_nodes = defaultdict(list)
        self._neighbor_cache = {}
        self._build(points)

    @classmethod
    def from_pcd(cls, pcd_path: str, cfg: PlannerConfig):
        return cls(load_ascii_pcd_xyz(pcd_path), cfg)

    def _cell_index(self, x: float, y: float):
        res = self.cfg.grid_resolution
        return (
            int(math.floor((x - self.origin_xy[0]) / res)),
            int(math.floor((y - self.origin_xy[1]) / res)),
        )

    def _cell_center(self, ix: int, iy: int):
        res = self.cfg.grid_resolution
        return np.array(
            [
                self.origin_xy[0] + (ix + 0.5) * res,
                self.origin_xy[1] + (iy + 0.5) * res,
            ],
            dtype=float,
        )

    def node_position(self, node_id: int):
        node = self.nodes[node_id]
        xy = self._cell_center(node.ix, node.iy)
        return np.array([xy[0], xy[1], node.z], dtype=float)

    def _build(self, points: np.ndarray):
        cfg = self.cfg
        raw_bins = defaultdict(lambda: defaultdict(lambda: [0, 0.0]))

        for x, y, z in points:
            ix, iy = self._cell_index(float(x), float(y))
            zbin = int(round(float(z) / cfg.height_bin_resolution))
            bucket = raw_bins[(ix, iy)][zbin]
            bucket[0] += 1
            bucket[1] += float(z)

        raw_heights = {}
        for cell, bins in raw_bins.items():
            heights = []
            for count, zsum in bins.values():
                if count >= cfg.min_points_per_height_bin:
                    heights.append(zsum / count)
            if heights:
                raw_heights[cell] = sorted(merge_close_values(heights, cfg.height_bin_resolution))

        kernel_radius = max(1, int(math.ceil(cfg.standable_radius / cfg.grid_resolution)))
        kernel_offsets = [
            (dx, dy)
            for dx in range(-kernel_radius, kernel_radius + 1)
            for dy in range(-kernel_radius, kernel_radius + 1)
            if math.hypot(dx * cfg.grid_resolution, dy * cfg.grid_resolution)
            <= cfg.standable_radius + 1e-6
        ]
        min_support = max(3, int(math.ceil(len(kernel_offsets) * cfg.standable_min_ratio)))

        for (ix, iy), heights in raw_heights.items():
            for z in heights:
                support = 0
                for dx, dy in kernel_offsets:
                    neighbor_heights = raw_heights.get((ix + dx, iy + dy))
                    if not neighbor_heights:
                        continue
                    if any(abs(nz - z) <= cfg.standable_height_tolerance for nz in neighbor_heights):
                        support += 1

                if support < min_support:
                    continue

                node_id = len(self.nodes)
                self.nodes.append(SupportNode(ix, iy, float(z), support / len(kernel_offsets)))
                self.cell_to_nodes[(ix, iy)].append(node_id)

    def nearest_candidates(self, x: float, y: float, z_hint=None, radius=None, max_count=40):
        cfg = self.cfg
        radius = cfg.snap_xy_radius if radius is None else radius
        cell_radius = max(1, int(math.ceil(radius / cfg.grid_resolution)))
        ix, iy = self._cell_index(x, y)
        candidates = []

        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                for node_id in self.cell_to_nodes.get((ix + dx, iy + dy), []):
                    pos = self.node_position(node_id)
                    dxy = float(np.linalg.norm(pos[:2] - np.array([x, y])))
                    if dxy > radius:
                        continue
                    dz = 0.0 if z_hint is None else abs(pos[2] - z_hint)
                    if z_hint is not None and dz > cfg.snap_z_tolerance:
                        continue
                    score = dxy + 0.35 * dz - 0.05 * self.nodes[node_id].standability
                    candidates.append((score, node_id))

        candidates.sort()
        return [node_id for _, node_id in candidates[:max_count]]

    def neighbors(self, node_id: int):
        if node_id in self._neighbor_cache:
            return self._neighbor_cache[node_id]

        cfg = self.cfg
        src = self.nodes[node_id]
        result = []

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                horiz = math.hypot(dx * cfg.grid_resolution, dy * cfg.grid_resolution)
                if horiz <= 1e-8:
                    continue
                for dst_id in self.cell_to_nodes.get((src.ix + dx, src.iy + dy), []):
                    dst = self.nodes[dst_id]
                    dz = abs(dst.z - src.z)
                    if dz > cfg.step_max:
                        continue
                    if dz / horiz > cfg.max_edge_slope:
                        continue
                    cost = horiz + cfg.vertical_weight * dz
                    cost += 0.05 * (2.0 - src.standability - dst.standability)
                    result.append((dst_id, cost))

        self._neighbor_cache[node_id] = result
        return result

    def astar(self, start_id: int, goal_id: int):
        goal_pos = self.node_position(goal_id)
        open_heap = [(0.0, 0.0, start_id)]
        parent = {start_id: None}
        g_score = {start_id: 0.0}

        while open_heap:
            _, current_g, current = heapq.heappop(open_heap)
            if current == goal_id:
                return reconstruct_path(parent, current), current_g
            if current_g > g_score.get(current, float("inf")) + 1e-9:
                continue

            for neighbor, step_cost in self.neighbors(current):
                tentative = current_g + step_cost
                if tentative >= g_score.get(neighbor, float("inf")):
                    continue
                g_score[neighbor] = tentative
                parent[neighbor] = current
                heuristic = float(np.linalg.norm(self.node_position(neighbor) - goal_pos))
                heapq.heappush(open_heap, (tentative + heuristic, tentative, neighbor))

        return None, float("inf")

    def plan(self, start_xyz, goal_xyz, goal_has_explicit_z=False):
        cfg = self.cfg
        start_support_z = float(start_xyz[2]) - cfg.body_height
        start_candidates = self.nearest_candidates(
            float(start_xyz[0]), float(start_xyz[1]), start_support_z
        )
        if not start_candidates:
            start_candidates = self.nearest_candidates(float(start_xyz[0]), float(start_xyz[1]), None)
        if not start_candidates:
            raise RuntimeError("No support-surface candidate near robot start pose")

        goal_z_hint = float(goal_xyz[2]) if goal_has_explicit_z else None
        max_goal_candidates = max(1, cfg.max_goal_candidates)
        goal_candidates = self.nearest_candidates(
            float(goal_xyz[0]), float(goal_xyz[1]), goal_z_hint, max_count=max_goal_candidates
        )
        if not goal_candidates:
            goal_candidates = self.nearest_candidates(
                float(goal_xyz[0]), float(goal_xyz[1]), None, max_count=max_goal_candidates
            )
        if not goal_candidates:
            raise RuntimeError("No support-surface candidate near requested goal")

        start_id = start_candidates[0]
        policy = cfg.goal_layer_policy

        if goal_has_explicit_z:
            ordered_goals = goal_candidates
        elif policy == "highest":
            ordered_goals = sorted(goal_candidates, key=lambda node_id: self.nodes[node_id].z, reverse=True)
        elif policy == "lowest":
            ordered_goals = sorted(goal_candidates, key=lambda node_id: self.nodes[node_id].z)
        elif policy == "nearest_to_start_height":
            start_z = self.nodes[start_id].z
            ordered_goals = sorted(goal_candidates, key=lambda node_id: abs(self.nodes[node_id].z - start_z))
        else:
            ordered_goals = goal_candidates

        best_path = None
        best_cost = float("inf")
        best_goal = None

        if policy == "min_path_cost" and not goal_has_explicit_z:
            candidates_to_try = ordered_goals[:max_goal_candidates]
        else:
            candidates_to_try = ordered_goals[:max_goal_candidates]

        for goal_id in candidates_to_try:
            path, cost = self.astar(start_id, goal_id)
            if path is not None and cost < best_cost:
                best_path = path
                best_cost = cost
                best_goal = goal_id
                if policy != "min_path_cost" or goal_has_explicit_z:
                    break

        if best_path is None:
            raise RuntimeError("No connected traversable support path to requested goal")

        return downsample_path(
            [self.node_position(node_id) for node_id in best_path],
            cfg.waypoint_spacing,
            cfg.vertical_waypoint_spacing,
            cfg.max_output_waypoints,
        ), {
            "start_id": start_id,
            "goal_id": best_goal,
            "cost": best_cost,
            "raw_nodes": len(best_path),
        }


class TraversabilityGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("m20_traversability_global_planner")
        self.declare_parameter("pcd_map_file", "")
        self.declare_parameter("frame_id", "world")
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

        self.frame_id = self.get_parameter("frame_id").value
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

        pcd_map_file = os.path.expanduser(str(self.get_parameter("pcd_map_file").value))
        if not pcd_map_file or not os.path.isfile(pcd_map_file):
            raise RuntimeError(f"pcd_map_file does not exist: {pcd_map_file}")

        self.get_logger().info(f"Building traversability support graph from {pcd_map_file}")
        self.graph = SupportGraph.from_pcd(pcd_map_file, self.cfg)
        self.get_logger().info(
            "Support graph ready: %d nodes, %d occupied XY cells, bounds min=%s max=%s"
            % (
                len(self.graph.nodes),
                len(self.graph.cell_to_nodes),
                np.array2string(self.graph.points_min, precision=2),
                np.array2string(self.graph.points_max, precision=2),
            )
        )

        self.odom = None
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "initial_path", latching_qos)
        self.odom_sub = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "move_base_simple/goal", self.goal_callback, 10
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped, "clicked_point", self.clicked_point_callback, 10
        )

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def goal_callback(self, msg: PoseStamped):
        goal = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        has_z = goal[2] > self.cfg.explicit_goal_z_threshold
        self.plan_and_publish(goal, has_z)

    def clicked_point_callback(self, msg: PointStamped):
        goal = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.plan_and_publish(goal, True)

    def plan_and_publish(self, goal_xyz, goal_has_explicit_z):
        if self.odom is None:
            self.get_logger().warning("Ignore goal before receiving body_pose")
            return

        p = self.odom.pose.pose.position
        start = np.array([p.x, p.y, p.z], dtype=float)
        t0 = time.perf_counter()
        try:
            path_points, info = self.graph.plan(start, goal_xyz, goal_has_explicit_z)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        elapsed = time.perf_counter() - t0

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
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
    points = []
    data = False
    fields = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            lower = stripped.lower()
            if data:
                values = stripped.split()
                if len(values) < 3:
                    continue
                points.append((float(values[0]), float(values[1]), float(values[2])))
            elif lower.startswith("fields"):
                fields = stripped.split()[1:]
                if fields[:3] != ["x", "y", "z"]:
                    raise RuntimeError(f"Only PCD fields starting with x y z are supported: {fields}")
            elif lower.startswith("data"):
                if "ascii" not in lower:
                    raise RuntimeError("Only ASCII PCD files are supported by this lightweight planner")
                data = True

    if not points:
        raise RuntimeError(f"No XYZ points loaded from {path}")
    return np.asarray(points, dtype=np.float32)


def merge_close_values(values, tolerance):
    if not values:
        return values
    merged = []
    current = [values[0]]
    for value in values[1:]:
        if abs(value - current[-1]) <= tolerance:
            current.append(value)
        else:
            merged.append(float(sum(current) / len(current)))
            current = [value]
    merged.append(float(sum(current) / len(current)))
    return merged


def reconstruct_path(parent, current):
    path = [current]
    while parent[current] is not None:
        current = parent[current]
        path.append(current)
    path.reverse()
    return path


def downsample_path(points, spacing, vertical_spacing, max_points):
    if not points:
        return []
    result = [points[0]]
    last = points[0]
    for point in points[1:-1]:
        if (
            np.linalg.norm(point[:2] - last[:2]) >= spacing
            or abs(point[2] - last[2]) >= vertical_spacing
        ):
            result.append(point)
            last = point
    if len(points) > 1:
        result.append(points[-1])
    return limit_polyline_points(result, max_points)


def limit_polyline_points(points, max_points):
    if max_points <= 1 or len(points) <= max_points:
        return points

    cumulative = [0.0]
    for prev, current in zip(points[:-1], points[1:]):
        cumulative.append(cumulative[-1] + float(np.linalg.norm(current - prev)))

    total = cumulative[-1]
    if total <= 1e-6:
        return [points[0], points[-1]]

    targets = np.linspace(0.0, total, max_points)
    result = []
    seg = 0
    for target in targets:
        while seg + 1 < len(cumulative) and cumulative[seg + 1] < target:
            seg += 1
        if seg + 1 >= len(points):
            result.append(points[-1])
            continue
        denom = cumulative[seg + 1] - cumulative[seg]
        ratio = 0.0 if denom <= 1e-9 else (target - cumulative[seg]) / denom
        result.append(points[seg] * (1.0 - ratio) + points[seg + 1] * ratio)

    result[0] = points[0]
    result[-1] = points[-1]
    return result


def run_cli(args):
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
    graph = SupportGraph.from_pcd(args.pcd, cfg)
    goal = np.array(args.goal, dtype=float)
    goal_has_z = len(args.goal) == 3
    start = np.array(args.start, dtype=float)
    points, info = graph.plan(start, goal, goal_has_z)
    print(f"support_nodes: {len(graph.nodes)}")
    print(f"xy_cells: {len(graph.cell_to_nodes)}")
    print(f"raw_nodes: {info['raw_nodes']}")
    print(f"waypoints: {len(points)}")
    print(f"cost: {info['cost']:.3f}")
    print(f"start_support_z: {graph.nodes[info['start_id']].z:.3f}")
    print(f"goal_support_z: {graph.nodes[info['goal_id']].z:.3f}")
    for point in points[:20]:
        print(f"{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}")
    if len(points) > 20:
        print("...")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--pcd")
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "BODY_Z"))
    parser.add_argument("--goal", nargs="+", type=float, metavar="VALUE")
    parser.add_argument("--grid-resolution", type=float, default=0.20)
    parser.add_argument("--goal-layer-policy", default="nearest_to_start_height")
    parser.add_argument("--step-max", type=float, default=0.30)
    parser.add_argument("--max-edge-slope", type=float, default=1.20)
    parser.add_argument("--waypoint-spacing", type=float, default=0.35)
    parser.add_argument("--vertical-waypoint-spacing", type=float, default=0.25)
    parser.add_argument("--max-output-waypoints", type=int, default=28)
    parser.add_argument("--max-goal-candidates", type=int, default=16)
    args, ros_args = parser.parse_known_args()

    if args.no_ros:
        if not args.pcd or not args.start or not args.goal or len(args.goal) not in (2, 3):
            parser.error("--no-ros requires --pcd, --start X Y BODY_Z, and --goal X Y [Z]")
        run_cli(args)
        return

    rclpy.init(args=ros_args)
    node = TraversabilityGlobalPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
