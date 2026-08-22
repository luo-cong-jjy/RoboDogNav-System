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
# test_point_cloud_filter.py —— 点云筛选工具的单元测试（中文注释版）
# 测试对象：m20_multifloor_map.point_cloud_filter 模块的
#           select_local_points（局部点选择）与 rotating_point_slice（旋转分片）
# 运行方式：colcon test 或 pytest test/test_point_cloud_filter.py
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Unit tests for generation-independent point selection."""

# NumPy：构造测试数据与断言比较
import numpy as np
# pytest：测试框架（参数化断言、异常断言）
import pytest

# 导入被测函数
from m20_multifloor_map.point_cloud_filter import (
    rotating_point_slice,
    select_local_points,
)


# 测试1：只保留半径内的有限点（排除 NaN 与圆外点）
def test_selects_only_finite_points_inside_radius():
    # 构造测试点集：
    #   点1 (0,0)     —— 圆心处，应保留
    #   点2 (3,4)     —— 距离 5，恰好在半径 5 边界内，应保留
    #   点3 (5.01,0)  —— 距离 5.01 > 5，应剔除
    #   点4 (nan,0)   —— 含 NaN 非有限值，应剔除
    points = np.asarray(
        [
            [0.0, 0.0, 1.0, 1.0],
            [3.0, 4.0, 2.0, 2.0],
            [5.01, 0.0, 3.0, 3.0],
            [np.nan, 0.0, 4.0, 4.0],
        ],
        dtype=np.float32,
    )

    # 以 (0,0) 为圆心、半径 5 筛选
    selected = select_local_points(points, 0.0, 0.0, 5.0)

    # 断言：应恰好保留 2 个点（形状 (2,4)）
    assert selected.shape == (2, 4)
    # 断言：保留点的 xy 坐标分别为 (0,0) 与 (3,4)
    np.testing.assert_allclose(selected[:, :2], [[0.0, 0.0], [3.0, 4.0]])


# 测试2：非法输入（形状错误 / 半径非正）应抛出 ValueError
def test_rejects_invalid_shape_and_radius():
    # 形状 (2,3) 不符合 (N,4)：应抛异常
    with pytest.raises(ValueError):
        select_local_points(np.zeros((2, 3)), 0.0, 0.0, 1.0)
    # 半径为 0（非正）：应抛异常
    with pytest.raises(ValueError):
        select_local_points(np.zeros((2, 4)), 0.0, 0.0, 0.0)


# 测试3：旋转分片每帧不超过上限，且多帧组合能覆盖全部点
def test_rotating_slices_stay_bounded_and_cover_all_points():
    # 构造 10 个点：x = 0..9，其余列全 0
    points = np.column_stack(
        [
            np.arange(10, dtype=np.float32),
            np.zeros((10, 3), dtype=np.float32),
        ]
    )

    # 以 maximum_count=3 为上限，取 4 个不同相位分别分片
    slices = [
        rotating_point_slice(points, maximum_count=3, phase=phase)
        for phase in range(4)
    ]
    # 把所有分片的 x 值收集起来并排序
    recovered = sorted(
        int(value)
        for point_slice in slices
        for value in point_slice[:, 0]
    )

    # 断言：每个分片点数都不超过 3
    assert all(point_slice.shape[0] <= 3 for point_slice in slices)
    # 断言：所有分片合起来恰好覆盖 0..9 全部 10 个点
    assert recovered == list(range(10))


# 测试4：旋转分片不应把不相邻楼层区域的点混在一起
def test_rotating_slice_does_not_mix_disjoint_floor_regions():
    # 构造楼层一的两个点：x 均为负值（-10, -5）
    floor_one = np.asarray(
        [[-10.0, 0.0, 0.0, 1.0], [-5.0, 1.0, 0.0, 1.0]],
        dtype=np.float32,
    )

    # 上限 1 点、相位 0 分片
    selected = rotating_point_slice(floor_one, maximum_count=1, phase=0)

    # 断言：选中的点 x 仍为负（没有混入其它楼层/区域的正 x 点）
    assert selected[:, 0].max() < 0.0
