#!/usr/bin/env python3
"""Sample a Collada triangle mesh into an ASCII XYZ PCD file."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def parse_floats(text):
    return np.fromstring(text or "", sep=" ", dtype=np.float64)


def local_matrix(node):
    matrix = np.eye(4)
    for child in list(node):
        tag = child.tag.rsplit("}", 1)[-1]
        values = parse_floats(child.text)
        if tag == "matrix" and values.size == 16:
            matrix = matrix @ values.reshape((4, 4))
    return matrix


def load_geometries(root):
    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", NS):
        geometry_id = geometry.get("id")
        mesh = geometry.find("c:mesh", NS)
        if geometry_id is None or mesh is None:
            continue

        sources = {}
        for source in mesh.findall("c:source", NS):
            source_id = source.get("id")
            array = source.find("c:float_array", NS)
            accessor = source.find("c:technique_common/c:accessor", NS)
            if source_id is None or array is None or accessor is None:
                continue

            stride = int(accessor.get("stride", "1"))
            values = parse_floats(array.text)
            if stride <= 0 or values.size < stride:
                continue
            sources[source_id] = values.reshape((-1, stride))[:, :3]

        vertex_sources = {}
        for vertices in mesh.findall("c:vertices", NS):
            vertices_id = vertices.get("id")
            position = vertices.find("c:input[@semantic='POSITION']", NS)
            if vertices_id is not None and position is not None:
                vertex_sources[vertices_id] = position.get("source", "").lstrip("#")

        triangles = []
        for tri_node in mesh.findall("c:triangles", NS):
            inputs = tri_node.findall("c:input", NS)
            if not inputs:
                continue

            vertex_input = None
            for input_node in inputs:
                if input_node.get("semantic") == "VERTEX":
                    vertex_input = input_node
                    break
            if vertex_input is None:
                continue

            source_id = vertex_sources.get(vertex_input.get("source", "").lstrip("#"))
            if source_id not in sources:
                continue

            stride = max(int(input_node.get("offset", "0")) for input_node in inputs) + 1
            vertex_offset = int(vertex_input.get("offset", "0"))
            index_text = tri_node.findtext("c:p", default="", namespaces=NS)
            indices = np.fromstring(index_text, sep=" ", dtype=np.int64)
            if indices.size == 0:
                continue

            vertices = indices.reshape((-1, stride))[:, vertex_offset].reshape((-1, 3))
            triangles.append(sources[source_id][vertices])

        if triangles:
            geometries[geometry_id] = np.vstack(triangles)
    return geometries


def instanced_triangles(root, geometries):
    def walk(node, parent_matrix):
        matrix = parent_matrix @ local_matrix(node)
        instance = node.find("c:instance_geometry", NS)
        if instance is not None:
            geometry_id = instance.get("url", "").lstrip("#")
            triangles = geometries.get(geometry_id)
            if triangles is not None:
                flat = triangles.reshape((-1, 3))
                homogeneous = np.concatenate([flat, np.ones((flat.shape[0], 1))], axis=1)
                transformed = (matrix @ homogeneous.T).T[:, :3]
                yield transformed.reshape((-1, 3, 3))

        for child in node.findall("c:node", NS):
            yield from walk(child, matrix)

    for scene in root.findall(".//c:library_visual_scenes/c:visual_scene", NS):
        for node in scene.findall("c:node", NS):
            yield from walk(node, np.eye(4))


def sample_triangles(triangles, spacing, rng):
    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]
    areas = np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1) * 0.5
    keep = areas > 1e-10
    if not np.any(keep):
        return np.empty((0, 3), dtype=np.float64)

    v0 = v0[keep]
    v1 = v1[keep]
    v2 = v2[keep]
    areas = areas[keep]
    counts = np.maximum(1, np.ceil(areas / (spacing * spacing)).astype(np.int64))
    total = int(counts.sum())

    tri_index = np.repeat(np.arange(len(counts)), counts)
    r1 = np.sqrt(rng.random(total))
    r2 = rng.random(total)
    weights0 = 1.0 - r1
    weights1 = r1 * (1.0 - r2)
    weights2 = r1 * r2
    return (
        v0[tri_index] * weights0[:, None]
        + v1[tri_index] * weights1[:, None]
        + v2[tri_index] * weights2[:, None]
    )


def voxel_downsample(points, voxel_size):
    if voxel_size <= 0.0 or points.size == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
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
            handle.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--spacing", type=float, default=0.10)
    parser.add_argument("--voxel-size", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=127)
    parser.add_argument(
        "--translation",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    args = parser.parse_args()

    if args.spacing <= 0.0:
        raise ValueError("--spacing must be positive")
    if not args.input.is_file():
        raise FileNotFoundError(args.input)

    root = ET.parse(args.input).getroot()
    geometries = load_geometries(root)
    rng = np.random.default_rng(args.seed)

    chunks = []
    triangle_count = 0
    for triangles in instanced_triangles(root, geometries):
        triangle_count += len(triangles)
        sampled = sample_triangles(triangles, args.spacing, rng)
        if sampled.size:
            chunks.append(sampled)

    if not chunks:
        raise RuntimeError("No triangle geometry was found in the Collada visual scene")

    points = np.vstack(chunks)
    points += np.array(args.translation, dtype=np.float64)
    points = voxel_downsample(points, args.voxel_size)
    write_pcd(args.output, points)

    bounds_min = points.min(axis=0)
    bounds_max = points.max(axis=0)
    print(f"triangles: {triangle_count}")
    print(f"points: {len(points)}")
    print(
        "bounds: "
        f"({bounds_min[0]:.3f}, {bounds_min[1]:.3f}, {bounds_min[2]:.3f}) "
        f"({bounds_max[0]:.3f}, {bounds_max[1]:.3f}, {bounds_max[2]:.3f})"
    )
    print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
