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
# test_kinematics.py —— 平面运动学函数的单元测试（中文注释版）
# 测试对象：m20_warehouse_sim.kinematics 模块的
#           PlanarState / clamp_planar_command / integrate_planar / normalize_angle
# 运行方式：colcon test 或 pytest test/test_kinematics.py
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Tests for pure planar kinematics."""

# 数学库：pi、近似比较
import math
# 路径库：定位包根目录
from pathlib import Path

# pytest：测试框架（近似断言）
import pytest

# 导入被测模块
from m20_warehouse_sim.kinematics import (
    PlanarState,             # 机身平面状态
    clamp_planar_command,    # 速度指令限幅
    integrate_planar,        # 平面运动学积分
    normalize_angle,         # 角度归一化
)


# 包根目录（测试文件的上上级目录）
ROOT = Path(__file__).parents[1]


# 测试1：前向运动应沿机身朝向方向行进
def test_forward_motion_follows_heading() -> None:
    # 初始状态：位置 (1, 2, 0.59)，航向 90 度（朝 +y）
    state = PlanarState(1.0, 2.0, 0.59, math.pi / 2.0)
    # 以机身系 vx=0.4 前进 2 秒
    result = integrate_planar(state, (0.4, 0.0, 0.0), 2.0)
    # 断言：x 不变（航向朝 +y，前进不改变 x）
    assert result.x == pytest.approx(1.0)
    # 断言：y 增加 0.4 * 2 = 0.8 -> 2.8
    assert result.y == pytest.approx(2.8)
    # 断言：z 高度不变
    assert result.z == 0.59
    # 断言：机身系前向速度保持 0.4
    assert result.vx_body == pytest.approx(0.4)
    # 断言：机身系侧向速度为 0
    assert result.vy_body == pytest.approx(0.0)
    # 断言：世界系 x 速度约等于 0（旋转后投影为 0）
    assert result.vx_world == pytest.approx(0.0, abs=1.0e-12)
    # 断言：世界系 y 速度为 0.4
    assert result.vy_world == pytest.approx(0.4)


# 测试2：平面旋转后航向角应归一化到 [-pi, pi]
def test_planar_rotation_is_normalized() -> None:
    # 初始航向 pi - 0.1（接近 pi）
    state = PlanarState(0.0, 0.0, 0.59, math.pi - 0.1)
    # 以角速度 0.5 旋转 1 秒：航向变为 pi + 0.4（超过 pi）
    result = integrate_planar(state, (0.0, 0.0, 0.5), 1.0)
    # 断言：航向角始终落在 [-pi, pi]
    assert -math.pi <= result.yaw <= math.pi
    # 断言：归一化结果等于 normalize_angle(pi + 0.4)
    assert result.yaw == pytest.approx(normalize_angle(math.pi + 0.4))


# 测试3：后端速度上限应具有防御性（超限指令被截断）
def test_backend_limits_are_defensive() -> None:
    # 输入 (2.0, -2.0, 3.0) 超过上限 (0.45, 0.2, 0.65)，应被截断为各轴上限
    assert clamp_planar_command(2.0, -2.0, 3.0, (0.45, 0.2, 0.65)) == (
        0.45,
        -0.2,
        0.65,
    )


# 测试4：里程计发布的线速度应表达在机身坐标系（child_frame）中
def test_odometry_publishes_body_velocity_in_child_frame() -> None:
    # 读取后端节点源码（静态检查，避免引入 ROS2 运行依赖）
    source = (
        ROOT
        / 'm20_warehouse_sim'
        / 'kinematic_backend_node.py'
    ).read_text(encoding='utf-8')

    # 断言：源码中 x 方向线速度取自已保存的机身系速度 vx_body
    assert (
        'odometry.twist.twist.linear.x = self._state.vx_body'
        in source
    )
    # 断言：源码中 y 方向线速度取自机身系速度 vy_body
    assert (
        'odometry.twist.twist.linear.y = self._state.vy_body'
        in source
    )
