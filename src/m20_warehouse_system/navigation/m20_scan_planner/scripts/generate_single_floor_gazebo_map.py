#!/usr/bin/env python3
"""Generate a synchronized single-floor Gazebo world and PCD map."""
# ============================================================
# 文件职责：生成"同步"的单层 Gazebo 仿真世界与对应 PCD 点云地图。
#   随机在指定区域内放置矩形障碍物（避开起点与中心区域），然后：
#     1. 生成 .world 文件（SDF 格式：每个障碍物一个静态 box 模型）；
#     2. 按固定分辨率对障碍物表面（四个侧面 + 顶面）均匀采样，
#        生成 ASCII XYZ PCD 点云（作为激光/遍历地图的 ground truth）；
#     3. 生成 .json 元数据（种子、地图尺寸、障碍物列表等），
#        保证 Gazebo 世界与 PCD 地图一一对应（用于验证导航）。
# ============================================================

import argparse
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class BoxObstacle:
    # 数据类：一个长方体障碍物的描述（名称、中心坐标、尺寸）
    name: str
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float


def generate_obstacles(args):
    # 函数：随机生成障碍物列表（带放置约束与碰撞检查）
    # 参数：args - 命令行参数（地图尺寸、障碍物数量/尺寸、清除区域等）
    # 返回：BoxObstacle 列表；放置数量不足时抛出 RuntimeError
    rng = random.Random(args.seed) # 按种子创建随机数生成器
    obstacles = []
    margin = args.width_max + 0.5 # 边缘留白（保证障碍物不越界）
    min_x = -0.5 * args.x_length + margin # x 方向可放置范围
    max_x = 0.5 * args.x_length - margin
    min_y = -0.5 * args.y_length + margin # y 方向可放置范围
    max_y = 0.5 * args.y_length - margin
    attempts = 0
    max_attempts = args.obstacle_count * 80 # 最大尝试次数（防止死循环）

    while len(obstacles) < args.obstacle_count and attempts < max_attempts:
        attempts += 1
        sx = rng.uniform(args.width_min, args.width_max) # 随机 x 尺寸
        sy = rng.uniform(args.width_min, args.width_max) # 随机 y 尺寸
        sz = rng.uniform(args.height_min, args.height_max) # 随机 z 尺寸
        x = rng.uniform(min_x, max_x) # 随机中心 x
        y = rng.uniform(min_y, max_y) # 随机中心 y
        if math.hypot(x - args.start_x, y - args.start_y) < args.start_clearance:
            continue # 离起点太近：丢弃
        if args.keep_center_clear and abs(x) < args.center_clear_x and abs(y) < args.center_clear_y:
            continue # 中心清除区域内：丢弃
        if any(overlaps(x, y, sx, sy, other, args.min_gap) for other in obstacles):
            continue # 与已有障碍物重叠（含最小间隙）：丢弃
        obstacles.append(
            BoxObstacle(
                name=f"mockamap_box_{len(obstacles):03d}", # 按序号命名
                x=x,
                y=y,
                z=0.5 * sz, # 中心高度 = 半高（底面在 z=0）
                size_x=sx,
                size_y=sy,
                size_z=sz,
            )
        )

    if len(obstacles) < args.obstacle_count:
        # 尝试次数耗尽仍未放满：报错提示调整参数
        raise RuntimeError(
            f"Only placed {len(obstacles)} of {args.obstacle_count} obstacles; "
            "reduce obstacle_count/min_gap or increase map size."
        )
    return obstacles


def overlaps(x, y, sx, sy, other, gap):
    # 函数：判断候选障碍物（x,y,sx,sy）是否与已有障碍物 other 在 XY 平面重叠
    # 参数：x,y - 候选中心；sx,sy - 候选尺寸；other - 已有障碍物；gap - 最小间隙
    # 返回：True 表示重叠
    return (
        abs(x - other.x) < 0.5 * (sx + other.size_x) + gap # x 方向相交
        and abs(y - other.y) < 0.5 * (sy + other.size_y) + gap # y 方向相交
    )


def sample_box_surfaces(obstacles, resolution):
    # 函数：对每个障碍物的外表面（四侧面 + 顶面）按分辨率均匀采样
    # 参数：obstacles - 障碍物列表；resolution - 采样分辨率（米）
    # 返回：排序后的采样点列表（含重复剔除）
    points = set() # 用集合自动去重
    for obstacle in obstacles:
        x0 = obstacle.x # 中心坐标
        y0 = obstacle.y
        z0 = obstacle.z
        hx = 0.5 * obstacle.size_x # 半尺寸
        hy = 0.5 * obstacle.size_y
        hz = 0.5 * obstacle.size_z
        xs = axis_samples(-hx, hx, resolution) # 各轴采样位置
        ys = axis_samples(-hy, hy, resolution)
        zs = axis_samples(-hz, hz, resolution)

        for y in ys:
            for z in zs:
                add_point(points, x0 - hx, y0 + y, z0 + z) # x 负侧面
                add_point(points, x0 + hx, y0 + y, z0 + z) # x 正侧面
        for x in xs:
            for z in zs:
                add_point(points, x0 + x, y0 - hy, z0 + z) # y 负侧面
                add_point(points, x0 + x, y0 + hy, z0 + z) # y 正侧面
        for x in xs:
            for y in ys:
                add_point(points, x0 + x, y0 + y, z0 + hz) # 顶面

    return sorted(points) # 排序后返回（保证输出确定性）


def axis_samples(low, high, resolution):
    # 函数：在一维区间 [low, high] 上按分辨率生成采样位置（含端点）
    # 返回：采样位置列表
    if high < low:
        low, high = high, low # 保证 low <= high
    count = max(2, int(math.ceil((high - low) / resolution)) + 1)
    if count == 1:
        return [low]
    return [low + (high - low) * i / (count - 1) for i in range(count)] # 等间距插值


def add_point(points, x, y, z):
    # 函数：把坐标四舍五入到 4 位小数后加入点集（消除浮点误差带来的重复）
    points.add((round(x, 4), round(y, 4), round(z, 4)))


def write_pcd(path, points):
    # 函数：把点集写为 ASCII XYZ PCD 文件
    # 参数：path - 输出路径（Path）；points - 点集合（(x,y,z) 元组）
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z\n")
        handle.write("SIZE 4 4 4\n")
        handle.write("TYPE F F F\n")
        handle.write("COUNT 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(points)}\n")
        handle.write("DATA ascii\n")
        for x, y, z in points:
            handle.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def write_world(path, obstacles, args):
    # 函数：生成 SDF 格式的 Gazebo .world 文件（每个障碍物一个静态 box）
    # 参数：path - 输出路径；obstacles - 障碍物列表；args - 命令行参数
    with path.open("w", encoding="utf-8") as handle:
        handle.write("<?xml version='1.0'?>\n")
        handle.write("<sdf version='1.6'>\n")
        handle.write("  <world name='mockamap_single_floor'>\n")
        handle.write("    <gravity>0 0 -9.8</gravity>\n") # 重力
        handle.write("    <include><uri>model://sun</uri></include>\n") # 阳光
        handle.write("    <include><uri>model://ground_plane</uri></include>\n") # 地面
        handle.write("    <gui>\n")
        handle.write("      <camera name='user_camera'>\n") # 默认相机视角
        handle.write("        <pose>-18 -18 18 0 0.75 0.78</pose>\n")
        handle.write("      </camera>\n")
        handle.write("    </gui>\n")
        for index, obstacle in enumerate(obstacles):
            # 按序号生成伪随机颜色（保证每个障碍物颜色不同且稳定）
            color = (
                0.35 + 0.45 * ((index * 37) % 100) / 100.0,
                0.40 + 0.35 * ((index * 61) % 100) / 100.0,
                0.45 + 0.30 * ((index * 19) % 100) / 100.0,
                1.0,
            )
            handle.write(f"    <model name='{obstacle.name}'>\n")
            handle.write("      <static>true</static>\n") # 静态模型（不参与物理）
            handle.write(f"      <pose>{obstacle.x:.4f} {obstacle.y:.4f} {obstacle.z:.4f} 0 0 0</pose>\n") # 位姿
            handle.write("      <link name='link'>\n")
            handle.write("        <collision name='collision'>\n") # 碰撞体
            handle.write("          <geometry><box>")
            handle.write(
                f"<size>{obstacle.size_x:.4f} {obstacle.size_y:.4f} {obstacle.size_z:.4f}</size>"
            )
            handle.write("</box></geometry>\n")
            handle.write("        </collision>\n")
            handle.write("        <visual name='visual'>\n") # 可视化体
            handle.write("          <geometry><box>")
            handle.write(
                f"<size>{obstacle.size_x:.4f} {obstacle.size_y:.4f} {obstacle.size_z:.4f}</size>"
            )
            handle.write("</box></geometry>\n")
            handle.write("          <material>\n")
            handle.write(
                "            <ambient>%.3f %.3f %.3f %.3f</ambient>\n" % color # 环境光颜色
            )
            handle.write(
                "            <diffuse>%.3f %.3f %.3f %.3f</diffuse>\n" % color # 漫反射颜色
            )
            handle.write("          </material>\n")
            handle.write("        </visual>\n")
            handle.write("      </link>\n")
            handle.write("    </model>\n")
        handle.write("  </world>\n")
        handle.write("</sdf>\n")


def write_metadata(path, obstacles, args, pcd_path, world_path):
    # 函数：生成 JSON 元数据文件（记录生成参数与障碍物列表）
    # 参数：path - 输出路径；其余为生成结果与输入参数
    metadata = {
        "seed": args.seed, # 随机种子
        "x_length": args.x_length, # 地图尺寸
        "y_length": args.y_length,
        "z_length": args.z_length,
        "obstacle_count": len(obstacles), # 障碍物数量
        "surface_resolution": args.surface_resolution, # 采样分辨率
        "pcd_file": pcd_path.name,   # 对应 PCD 文件名
        "world_file": world_path.name, # 对应 world 文件名
        "obstacles": [asdict(obstacle) for obstacle in obstacles], # 障碍物明细
    }
    path.write_text(json.dumps(metadata, indent=2), encoding="utf-8") # 写为美化 JSON


def parse_args():
    # 函数：解析命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", # 输出目录（默认指向包内的 models 目录）
        default=(
            "src/m20_warehouse_system/navigation/m20_scan_planner/"
            "models/mockamap_single_floor"
        ),
    )
    parser.add_argument("--seed", type=int, default=127) # 随机种子
    parser.add_argument("--x-length", type=float, default=40.0) # 地图 x 长度
    parser.add_argument("--y-length", type=float, default=40.0) # 地图 y 长度
    parser.add_argument("--z-length", type=float, default=5.0)  # 地图 z 高度
    parser.add_argument("--obstacle-count", type=int, default=180) # 障碍物数量
    parser.add_argument("--width-min", type=float, default=0.2)  # 障碍物最小宽
    parser.add_argument("--width-max", type=float, default=0.8)  # 障碍物最大宽
    parser.add_argument("--height-min", type=float, default=2.0) # 障碍物最小高
    parser.add_argument("--height-max", type=float, default=2.0) # 障碍物最大高
    parser.add_argument("--surface-resolution", type=float, default=0.10) # 表面采样分辨率
    parser.add_argument("--start-x", type=float, default=-19.0) # 起点 x
    parser.add_argument("--start-y", type=float, default=1.0)   # 起点 y
    parser.add_argument("--start-clearance", type=float, default=2.0) # 起点清除半径
    parser.add_argument("--min-gap", type=float, default=0.20)  # 障碍物最小间隙
    parser.add_argument("--keep-center-clear", dest="keep_center_clear", action="store_true", default=True) # 默认保持中心清除
    parser.add_argument("--no-center-clear", dest="keep_center_clear", action="store_false") # 取消中心清除
    parser.add_argument("--center-clear-x", type=float, default=1.5) # 中心清除区域 x 半宽
    parser.add_argument("--center-clear-y", type=float, default=1.5) # 中心清除区域 y 半宽
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True) # 创建输出目录
    world_path = output_dir / "mockamap_single_floor.world" # world 文件
    pcd_path = output_dir / "mockamap_single_floor.pcd"    # PCD 文件
    metadata_path = output_dir / "mockamap_single_floor.json" # 元数据文件

    obstacles = generate_obstacles(args) # 生成障碍物
    points = sample_box_surfaces(obstacles, args.surface_resolution) # 表面采样
    write_world(world_path, obstacles, args) # 写 world
    write_pcd(pcd_path, points)              # 写 PCD
    write_metadata(metadata_path, obstacles, args, pcd_path, world_path) # 写元数据
    # 打印输出摘要
    print(f"world: {world_path}")
    print(f"pcd: {pcd_path}")
    print(f"metadata: {metadata_path}")
    print(f"obstacles: {len(obstacles)}")
    print(f"points: {len(points)}")


if __name__ == "__main__":
    main()
