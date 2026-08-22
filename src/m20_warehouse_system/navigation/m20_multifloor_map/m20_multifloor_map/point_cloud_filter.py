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
# point_cloud_filter.py —— 点云筛选工具函数（中文注释版）
# 作用：为动态局部感知节点提供两个纯 NumPy 工具函数：
#   1) select_local_points：按水平圆形窗口裁剪出局部点（过滤非有限值）
#   2) rotating_point_slice：按相位对点集做旋转分片采样，使每帧点数不超过
#      DDS 传输上限，同时保证在连续多帧内覆盖全部点
# 输入点云约定：numpy 数组，形状 (N, 4)，列为 x / y / z / intensity
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Pure NumPy helpers used by the dynamic local sensing node."""

# 数学库：向上取整 ceil
import math

# NumPy：数组运算
import numpy as np


# 返回位于水平圆形感知窗口内的有限点（以 (center_x, center_y) 为圆心、radius 为半径）
def select_local_points(
    points: np.ndarray,
    center_x: float,
    center_y: float,
    radius: float,
) -> np.ndarray:
    """Return finite points inside a horizontal circular sensing window."""
    # 转成 float32 数组
    values = np.asarray(points, dtype=np.float32)
    # 校验形状必须是 (N, 4)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError('points must have shape (N, 4)')
    # 校验半径必须为正
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    # 空输入直接返回副本
    if values.size == 0:
        return values.copy()
    # 逐点检查 x/y/z/intensity 是否全部有限（排除 NaN/Inf）
    finite = np.isfinite(values).all(axis=1)
    # 计算点相对圆心的水平偏移
    dx = values[:, 0] - float(center_x)
    dy = values[:, 1] - float(center_y)
    # 筛选：有限 且 水平距离平方 <= 半径平方（圆内）
    selected = finite & ((dx * dx + dy * dy) <= float(radius) ** 2)
    # 返回连续内存的筛选结果
    return np.ascontiguousarray(values[selected], dtype=np.float32)


# 对点集做旋转分片：每帧只发布一个"同余类"（residue class）的点
def rotating_point_slice(
    points: np.ndarray,
    maximum_count: int,
    phase: int,
) -> np.ndarray:
    """Limit one DDS sample while covering every point across later phases."""
    # 转成 float32 数组
    values = np.asarray(points, dtype=np.float32)
    # 校验形状必须是 (N, 4)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError('points must have shape (N, 4)')
    # 校验最大点数必须为正
    if maximum_count <= 0:
        raise ValueError('maximum_count must be positive')
    # 点数本就不超上限：直接返回全部点
    if values.shape[0] <= maximum_count:
        return np.ascontiguousarray(values, dtype=np.float32)

    # Each phase publishes one residue class. This keeps every sample below
    # the DDS transport threshold and still reconstructs the full local map
    # in the accumulating SCAN occupancy grid over successive frames.
    # 注释（原文）：每个相位发布一个同余类。这既让每个 DDS 样本保持在传输上限
    # 之下，又能在后续帧中通过累加的 SCAN 占据栅格重建完整的局部地图。
    # 步长 = 点数 / 上限，向上取整（分片数量）
    stride = int(math.ceil(values.shape[0] / maximum_count))
    # 起始索引 = 相位对步长取模（轮流旋转起点）
    start = int(phase) % stride
    # 按等间隔抽取索引（0, stride, 2*stride, ... 的循环移位）
    indices = np.arange(start, values.shape[0], stride, dtype=np.int64)
    # 返回连续内存的采样结果
    return np.ascontiguousarray(values[indices], dtype=np.float32)
