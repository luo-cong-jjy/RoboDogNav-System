#!/usr/bin/env python3
"""Sample a Collada triangle mesh into an ASCII XYZ PCD file."""
# ============================================================
# 文件职责：把 Collada（.dae）三角形网格均匀采样为 ASCII XYZ 格式的
#   PCD 点云文件（用于生成仿真环境的地面点云地图）。
#   处理流程：
#     1. 解析 Collada XML：读取几何（library_geometries）中的三角面片
#        顶点数据与实例化（library_visual_scenes）的节点变换矩阵；
#     2. 按每个三角形的面积比例确定采样点数，用重心坐标均匀撒点；
#     3. 可选平移（--translation）与体素下采样（--voxel-size）；
#     4. 输出标准 ASCII PCD（x y z 三个字段）。
# ============================================================

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"} # Collada XML 命名空间前缀


def parse_floats(text):
    # 函数：把空格分隔的字符串解析为 float64 数组
    # 参数：text - 原始文本；返回：np.ndarray
    return np.fromstring(text or "", sep=" ", dtype=np.float64)


def local_matrix(node):
    # 函数：计算单个 Collada 节点的本地变换矩阵（累积其子 matrix 元素）
    # 参数：node - XML 节点；返回：4x4 齐次变换矩阵
    matrix = np.eye(4) # 初始为单位阵
    for child in list(node):
        tag = child.tag.rsplit("}", 1)[-1] # 去掉命名空间前缀取标签名
        values = parse_floats(child.text)
        if tag == "matrix" and values.size == 16: # 只有 16 元素的 matrix 才生效
            matrix = matrix @ values.reshape((4, 4)) # 右乘该子矩阵
    return matrix


def load_geometries(root):
    # 函数：解析 Collada 几何库，提取所有网格的三角形顶点数据
    # 参数：root - XML 根元素
    # 返回：字典 {geometry_id: Nx3x3 数组}（每个元素为一个三角形的三个顶点）
    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", NS):
        geometry_id = geometry.get("id") # 几何 ID
        mesh = geometry.find("c:mesh", NS)
        if geometry_id is None or mesh is None:
            continue

        # ---- 解析所有 source（顶点坐标数组等） ----
        sources = {}
        for source in mesh.findall("c:source", NS):
            source_id = source.get("id")
            array = source.find("c:float_array", NS)          # 原始数值数组
            accessor = source.find("c:technique_common/c:accessor", NS) # 访问器（描述步长）
            if source_id is None or array is None or accessor is None:
                continue

            stride = int(accessor.get("stride", "1")) # 每个顶点的元素数
            values = parse_floats(array.text)
            if stride <= 0 or values.size < stride:
                continue
            sources[source_id] = values.reshape((-1, stride))[:, :3] # 只取前 3 维（x y z）

        # ---- 解析 vertices 到 POSITION source 的映射 ----
        vertex_sources = {}
        for vertices in mesh.findall("c:vertices", NS):
            vertices_id = vertices.get("id")
            position = vertices.find("c:input[@semantic='POSITION']", NS) # 位置输入
            if vertices_id is not None and position is not None:
                vertex_sources[vertices_id] = position.get("source", "").lstrip("#") # 去掉 # 前缀

        # ---- 解析三角形索引并取顶点坐标 ----
        triangles = []
        for tri_node in mesh.findall("c:triangles", NS):
            inputs = tri_node.findall("c:input", NS)
            if not inputs:
                continue

            vertex_input = None # 找到语义为 VERTEX 的输入
            for input_node in inputs:
                if input_node.get("semantic") == "VERTEX":
                    vertex_input = input_node
                    break
            if vertex_input is None:
                continue

            source_id = vertex_sources.get(vertex_input.get("source", "").lstrip("#")) # 对应的 POSITION source
            if source_id not in sources:
                continue

            # 索引数组的步长 = 所有输入的最大 offset + 1
            stride = max(int(input_node.get("offset", "0")) for input_node in inputs) + 1
            vertex_offset = int(vertex_input.get("offset", "0")) # VERTEX 在索引中的偏移
            index_text = tri_node.findtext("c:p", default="", namespaces=NS) # 索引文本（空格分隔）
            indices = np.fromstring(index_text, sep=" ", dtype=np.int64)
            if indices.size == 0:
                continue

            # 取出每个三角形的三个顶点索引，再索引到坐标数组
            vertices = indices.reshape((-1, stride))[:, vertex_offset].reshape((-1, 3))
            triangles.append(sources[source_id][vertices])

        if triangles:
            geometries[geometry_id] = np.vstack(triangles) # 合并该几何的所有三角形
    return geometries


def instanced_triangles(root, geometries):
    # 生成器函数：遍历视觉场景中的节点，应用变换矩阵后产出所有三角形
    # 参数：root - XML 根元素；geometries - load_geometries 的结果
    # 产出：Nx3x3 数组（变换到世界系后的三角形）
    def walk(node, parent_matrix):
        matrix = parent_matrix @ local_matrix(node) # 累积父级变换
        instance = node.find("c:instance_geometry", NS) # 节点实例化的几何
        if instance is not None:
            geometry_id = instance.get("url", "").lstrip("#")
            triangles = geometries.get(geometry_id)
            if triangles is not None:
                flat = triangles.reshape((-1, 3)) # 展平为顶点列表
                homogeneous = np.concatenate([flat, np.ones((flat.shape[0], 1))], axis=1) # 补齐次坐标
                transformed = (matrix @ homogeneous.T).T[:, :3] # 变换并去掉齐次维
                yield transformed.reshape((-1, 3, 3)) # 还原为三角形结构

        for child in node.findall("c:node", NS): # 递归遍历子节点
            yield from walk(child, matrix)

    for scene in root.findall(".//c:library_visual_scenes/c:visual_scene", NS):
        for node in scene.findall("c:node", NS):
            yield from walk(node, np.eye(4)) # 场景根节点使用单位矩阵


def sample_triangles(triangles, spacing, rng):
    # 函数：在三角形内按面积比例均匀采样点
    # 参数：triangles - Nx3x3 三角形数组；spacing - 采样间距（米）；
    #       rng - numpy 随机数生成器
    # 返回：Mx3 采样点数组
    v0 = triangles[:, 0, :] # 顶点 0
    v1 = triangles[:, 1, :] # 顶点 1
    v2 = triangles[:, 2, :] # 顶点 2
    areas = np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1) * 0.5 # 三角形面积（叉积模/2）
    keep = areas > 1e-10 # 剔除退化三角形（面积近似为 0）
    if not np.any(keep):
        return np.empty((0, 3), dtype=np.float64) # 没有有效三角形

    v0 = v0[keep]
    v1 = v1[keep]
    v2 = v2[keep]
    areas = areas[keep]
    counts = np.maximum(1, np.ceil(areas / (spacing * spacing)).astype(np.int64)) # 每三角形采样点数（面积/间距平方）
    total = int(counts.sum())

    tri_index = np.repeat(np.arange(len(counts)), counts) # 按计数展开三角形索引
    r1 = np.sqrt(rng.random(total)) # 均匀采样的两个随机数（sqrt 保证面积均匀）
    r2 = rng.random(total)
    weights0 = 1.0 - r1 # 重心坐标权重 w0
    weights1 = r1 * (1.0 - r2) # 权重 w1
    weights2 = r1 * r2 # 权重 w2
    return (
        v0[tri_index] * weights0[:, None] # 重心坐标插值：p = w0*v0 + w1*v1 + w2*v2
        + v1[tri_index] * weights1[:, None]
        + v2[tri_index] * weights2[:, None]
    )


def voxel_downsample(points, voxel_size):
    # 函数：体素下采样——每个体素网格只保留第一个点
    # 参数：points - Nx3 点云；voxel_size - 体素边长（<=0 则不做）
    # 返回：下采样后的点云
    if voxel_size <= 0.0 or points.size == 0:
        return points # 无效参数直接返回
    keys = np.floor(points / voxel_size).astype(np.int64) # 计算每个点所在的体素网格坐标
    _, indices = np.unique(keys, axis=0, return_index=True) # 每个体素取首个索引
    return points[np.sort(indices)] # 按原顺序输出


def write_pcd(path, points):
    # 函数：把点云写为 ASCII XYZ PCD 文件
    # 参数：path - 输出路径（Path）；points - Nx3 点云
    path.parent.mkdir(parents=True, exist_ok=True) # 确保父目录存在
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z\n") # 字段：x y z
        handle.write("SIZE 4 4 4\n")   # 每字段字节数（float32）
        handle.write("TYPE F F F\n")   # 字段类型均为浮点
        handle.write("COUNT 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\n") # 点数
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(points)}\n")
        handle.write("DATA ascii\n") # ASCII 数据段
        for x, y, z in points:
            handle.write(f"{x:.5f} {y:.5f} {z:.5f}\n") # 逐点写坐标


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)  # 输入 Collada 文件
    parser.add_argument("output", type=Path) # 输出 PCD 文件
    parser.add_argument("--spacing", type=float, default=0.10)     # 采样间距（米）
    parser.add_argument("--voxel-size", type=float, default=0.10)  # 体素下采样尺寸
    parser.add_argument("--seed", type=int, default=127)           # 随机种子
    parser.add_argument(
        "--translation", # 输出点云的平移量
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    args = parser.parse_args()

    if args.spacing <= 0.0:
        raise ValueError("--spacing must be positive") # 间距必须为正
    if not args.input.is_file():
        raise FileNotFoundError(args.input) # 输入文件不存在

    root = ET.parse(args.input).getroot() # 解析 Collada XML
    geometries = load_geometries(root)    # 提取几何三角形
    rng = np.random.default_rng(args.seed) # 按种子创建随机数生成器

    chunks = []
    triangle_count = 0
    for triangles in instanced_triangles(root, geometries):
        triangle_count += len(triangles)
        sampled = sample_triangles(triangles, args.spacing, rng) # 逐实例采样
        if sampled.size:
            chunks.append(sampled)

    if not chunks:
        raise RuntimeError("No triangle geometry was found in the Collada visual scene") # 无几何

    points = np.vstack(chunks) # 合并所有采样点
    points += np.array(args.translation, dtype=np.float64) # 应用平移
    points = voxel_downsample(points, args.voxel_size) # 体素下采样
    write_pcd(args.output, points) # 写 PCD

    # 打印统计信息（三角形数、点数、包围盒）
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
