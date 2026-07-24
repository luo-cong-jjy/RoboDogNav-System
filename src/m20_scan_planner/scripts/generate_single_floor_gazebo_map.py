#!/usr/bin/env python3
"""Generate a synchronized single-floor Gazebo world and PCD map."""

import argparse
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass
class BoxObstacle:
    name: str
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float


def generate_obstacles(args):
    rng = random.Random(args.seed)
    obstacles = []
    margin = args.width_max + 0.5
    min_x = -0.5 * args.x_length + margin
    max_x = 0.5 * args.x_length - margin
    min_y = -0.5 * args.y_length + margin
    max_y = 0.5 * args.y_length - margin
    attempts = 0
    max_attempts = args.obstacle_count * 80

    while len(obstacles) < args.obstacle_count and attempts < max_attempts:
        attempts += 1
        sx = rng.uniform(args.width_min, args.width_max)
        sy = rng.uniform(args.width_min, args.width_max)
        sz = rng.uniform(args.height_min, args.height_max)
        x = rng.uniform(min_x, max_x)
        y = rng.uniform(min_y, max_y)
        if math.hypot(x - args.start_x, y - args.start_y) < args.start_clearance:
            continue
        if args.keep_center_clear and abs(x) < args.center_clear_x and abs(y) < args.center_clear_y:
            continue
        if any(overlaps(x, y, sx, sy, other, args.min_gap) for other in obstacles):
            continue
        obstacles.append(
            BoxObstacle(
                name=f"mockamap_box_{len(obstacles):03d}",
                x=x,
                y=y,
                z=0.5 * sz,
                size_x=sx,
                size_y=sy,
                size_z=sz,
            )
        )

    if len(obstacles) < args.obstacle_count:
        raise RuntimeError(
            f"Only placed {len(obstacles)} of {args.obstacle_count} obstacles; "
            "reduce obstacle_count/min_gap or increase map size."
        )
    return obstacles


def overlaps(x, y, sx, sy, other, gap):
    return (
        abs(x - other.x) < 0.5 * (sx + other.size_x) + gap
        and abs(y - other.y) < 0.5 * (sy + other.size_y) + gap
    )


def sample_box_surfaces(obstacles, resolution):
    points = set()
    for obstacle in obstacles:
        x0 = obstacle.x
        y0 = obstacle.y
        z0 = obstacle.z
        hx = 0.5 * obstacle.size_x
        hy = 0.5 * obstacle.size_y
        hz = 0.5 * obstacle.size_z
        xs = axis_samples(-hx, hx, resolution)
        ys = axis_samples(-hy, hy, resolution)
        zs = axis_samples(-hz, hz, resolution)

        for y in ys:
            for z in zs:
                add_point(points, x0 - hx, y0 + y, z0 + z)
                add_point(points, x0 + hx, y0 + y, z0 + z)
        for x in xs:
            for z in zs:
                add_point(points, x0 + x, y0 - hy, z0 + z)
                add_point(points, x0 + x, y0 + hy, z0 + z)
        for x in xs:
            for y in ys:
                add_point(points, x0 + x, y0 + y, z0 + hz)

    return sorted(points)


def axis_samples(low, high, resolution):
    if high < low:
        low, high = high, low
    count = max(2, int(math.ceil((high - low) / resolution)) + 1)
    if count == 1:
        return [low]
    return [low + (high - low) * i / (count - 1) for i in range(count)]


def add_point(points, x, y, z):
    points.add((round(x, 4), round(y, 4), round(z, 4)))


def write_pcd(path, points):
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
    with path.open("w", encoding="utf-8") as handle:
        handle.write("<?xml version='1.0'?>\n")
        handle.write("<sdf version='1.6'>\n")
        handle.write("  <world name='mockamap_single_floor'>\n")
        handle.write("    <gravity>0 0 -9.8</gravity>\n")
        handle.write("    <include><uri>model://sun</uri></include>\n")
        handle.write("    <include><uri>model://ground_plane</uri></include>\n")
        handle.write("    <gui>\n")
        handle.write("      <camera name='user_camera'>\n")
        handle.write("        <pose>-18 -18 18 0 0.75 0.78</pose>\n")
        handle.write("      </camera>\n")
        handle.write("    </gui>\n")
        for index, obstacle in enumerate(obstacles):
            color = (
                0.35 + 0.45 * ((index * 37) % 100) / 100.0,
                0.40 + 0.35 * ((index * 61) % 100) / 100.0,
                0.45 + 0.30 * ((index * 19) % 100) / 100.0,
                1.0,
            )
            handle.write(f"    <model name='{obstacle.name}'>\n")
            handle.write("      <static>true</static>\n")
            handle.write(f"      <pose>{obstacle.x:.4f} {obstacle.y:.4f} {obstacle.z:.4f} 0 0 0</pose>\n")
            handle.write("      <link name='link'>\n")
            handle.write("        <collision name='collision'>\n")
            handle.write("          <geometry><box>")
            handle.write(
                f"<size>{obstacle.size_x:.4f} {obstacle.size_y:.4f} {obstacle.size_z:.4f}</size>"
            )
            handle.write("</box></geometry>\n")
            handle.write("        </collision>\n")
            handle.write("        <visual name='visual'>\n")
            handle.write("          <geometry><box>")
            handle.write(
                f"<size>{obstacle.size_x:.4f} {obstacle.size_y:.4f} {obstacle.size_z:.4f}</size>"
            )
            handle.write("</box></geometry>\n")
            handle.write("          <material>\n")
            handle.write(
                "            <ambient>%.3f %.3f %.3f %.3f</ambient>\n" % color
            )
            handle.write(
                "            <diffuse>%.3f %.3f %.3f %.3f</diffuse>\n" % color
            )
            handle.write("          </material>\n")
            handle.write("        </visual>\n")
            handle.write("      </link>\n")
            handle.write("    </model>\n")
        handle.write("  </world>\n")
        handle.write("</sdf>\n")


def write_metadata(path, obstacles, args, pcd_path, world_path):
    metadata = {
        "seed": args.seed,
        "x_length": args.x_length,
        "y_length": args.y_length,
        "z_length": args.z_length,
        "obstacle_count": len(obstacles),
        "surface_resolution": args.surface_resolution,
        "pcd_file": pcd_path.name,
        "world_file": world_path.name,
        "obstacles": [asdict(obstacle) for obstacle in obstacles],
    }
    path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="src/m20_scan_planner/models/mockamap_single_floor")
    parser.add_argument("--seed", type=int, default=127)
    parser.add_argument("--x-length", type=float, default=40.0)
    parser.add_argument("--y-length", type=float, default=40.0)
    parser.add_argument("--z-length", type=float, default=5.0)
    parser.add_argument("--obstacle-count", type=int, default=180)
    parser.add_argument("--width-min", type=float, default=0.2)
    parser.add_argument("--width-max", type=float, default=0.8)
    parser.add_argument("--height-min", type=float, default=2.0)
    parser.add_argument("--height-max", type=float, default=2.0)
    parser.add_argument("--surface-resolution", type=float, default=0.10)
    parser.add_argument("--start-x", type=float, default=-19.0)
    parser.add_argument("--start-y", type=float, default=1.0)
    parser.add_argument("--start-clearance", type=float, default=2.0)
    parser.add_argument("--min-gap", type=float, default=0.20)
    parser.add_argument("--keep-center-clear", dest="keep_center_clear", action="store_true", default=True)
    parser.add_argument("--no-center-clear", dest="keep_center_clear", action="store_false")
    parser.add_argument("--center-clear-x", type=float, default=1.5)
    parser.add_argument("--center-clear-y", type=float, default=1.5)
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    world_path = output_dir / "mockamap_single_floor.world"
    pcd_path = output_dir / "mockamap_single_floor.pcd"
    metadata_path = output_dir / "mockamap_single_floor.json"

    obstacles = generate_obstacles(args)
    points = sample_box_surfaces(obstacles, args.surface_resolution)
    write_world(world_path, obstacles, args)
    write_pcd(pcd_path, points)
    write_metadata(metadata_path, obstacles, args, pcd_path, world_path)
    print(f"world: {world_path}")
    print(f"pcd: {pcd_path}")
    print(f"metadata: {metadata_path}")
    print(f"obstacles: {len(obstacles)}")
    print(f"points: {len(points)}")


if __name__ == "__main__":
    main()
