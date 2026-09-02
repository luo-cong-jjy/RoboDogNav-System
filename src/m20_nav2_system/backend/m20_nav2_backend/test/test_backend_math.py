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

# ============================================================
# 文件：test_backend_math.py
# 用途：M20 关节坐标换算与底层 PD 律的单元测试（pytest）。
#       覆盖：
#       - SDK ↔ 原始坐标往返一致性（sdk_to_raw / raw_to_sdk）；
#       - PD 力矩的非有限值保护与执行器限幅（pd_torque）；
#       - 轮子驻车制动只改轮子、不改输入（apply_wheel_brake）；
#       - 世界系→机体系向量旋转（0°/90°偏航、三维姿态、非法输入）；
#       - 仓库接触统计排除普通地面接触（warehouse_contact_summary）。
#       部分用例需要本机安装 mujoco Python 包。
# ============================================================

"""Unit tests for M20 joint conversion and the low-level PD law."""

import mujoco
import numpy as np

from m20_nav2_backend.backend_node import warehouse_contact_summary
from m20_nav2_backend.dynamics import (
    LEG_INDICES,
    WHEEL_INDICES,
    apply_wheel_brake,
    pd_torque,
    raw_to_sdk,
    sdk_to_raw,
    world_vector_to_body,
    yaw_quaternion,
)


def test_sdk_raw_coordinate_round_trip():
    # 坐标往返测试：任意 SDK 位置/速度/力矩经 sdk_to_raw 再经 raw_to_sdk
    # 应恢复原值（腿部与轮子索引都需一致）。
    position = np.linspace(-0.4, 0.4, 16)
    velocity = np.linspace(-2.0, 2.0, 16)
    torque = np.linspace(-4.0, 4.0, 16)
    raw = sdk_to_raw(position, velocity, torque)
    recovered = raw_to_sdk(*raw)
    assert np.allclose(recovered[0][LEG_INDICES], position[LEG_INDICES])
    assert np.allclose(recovered[0][WHEEL_INDICES], position[WHEEL_INDICES])
    assert np.allclose(recovered[1], velocity)
    assert np.allclose(recovered[2], torque)


def test_pd_torque_rejects_nonfinite_and_clamps_limits():
    # PD 律测试：期望位置含 NaN 时该通道力矩应为 0（非有限保护），
    # 其余通道按 PD 计算并截断到 [low, high] 限幅：
    #   10*kp=100 → 截断到 5；-10*kp=-100 → 截断到 -6；NaN → 0。
    result = pd_torque(
        position=np.zeros(3),
        velocity=np.zeros(3),
        desired_position=np.asarray([10.0, -10.0, np.nan]),
        desired_velocity=np.zeros(3),
        kp=np.ones(3) * 10.0,
        kd=np.ones(3),
        feedforward=np.zeros(3),
        low=np.asarray([-5.0, -6.0, -7.0]),
        high=np.asarray([5.0, 6.0, 7.0]),
    )
    assert np.allclose(result, [5.0, -6.0, 0.0])


def test_wheel_brake_zeros_only_wheel_targets_without_mutating_input():
    # 驻车制动测试：只对 4 个轮子清零速度/前馈并把 kd 抬到 brake_kd，
    # 腿部目标不变，且输入数组不能被修改（返回的是副本）。
    velocity = np.arange(16, dtype=float)
    kd = np.ones(16)
    feedforward = np.ones(16) * 2.0
    stopped_velocity, stopped_kd, stopped_feedforward = apply_wheel_brake(
        velocity,
        kd,
        feedforward,
        4.0,
    )
    assert np.allclose(stopped_velocity[WHEEL_INDICES], 0.0)
    assert np.allclose(stopped_kd[WHEEL_INDICES], 4.0)
    assert np.allclose(stopped_feedforward[WHEEL_INDICES], 0.0)
    assert np.allclose(stopped_velocity[LEG_INDICES], velocity[LEG_INDICES])
    assert np.allclose(velocity, np.arange(16, dtype=float))


def test_world_vector_to_body_is_identity_at_zero_yaw():
    # 零偏航时世界系→机体系应为恒等变换。
    result = world_vector_to_body(
        [1.0, -2.0, 0.5],
        yaw_quaternion(0.0),
    )
    assert np.allclose(result, [1.0, -2.0, 0.5])


def test_world_vector_to_body_rotates_ninety_degree_yaw():
    # 偏航 90° 时，世界系 (0, 2, 0.5) 应旋转为机体系 (2, 0, 0.5)。
    result = world_vector_to_body(
        [0.0, 2.0, 0.5],
        yaw_quaternion(np.pi / 2.0),
    )
    assert np.allclose(result, [2.0, 0.0, 0.5], atol=1.0e-12)


def test_world_vector_to_body_uses_full_three_dimensional_orientation():
    # 绕 X 轴 90° 的四元数：世界系 (0,0,3) 应变为机体系 (0,3,0)。
    half_sqrt = np.sqrt(0.5)
    result = world_vector_to_body(
        [0.0, 0.0, 3.0],
        [half_sqrt, half_sqrt, 0.0, 0.0],
    )
    assert np.allclose(result, [0.0, 3.0, 0.0], atol=1.0e-12)


def test_world_vector_to_body_rejects_nonfinite_vector():
    # 输入含 NaN 时应返回零向量（安全回退）。
    result = world_vector_to_body(
        [np.nan, 1.0, 2.0],
        yaw_quaternion(0.0),
    )
    assert np.allclose(result, np.zeros(3))


def test_warehouse_contact_summary_excludes_normal_ground_contact():
    # 接触统计测试：构造一个微型 MuJoCo 场景（地面 + 仓库墙 + 机器人体），
    # 让机器人体下落与墙接触；统计结果应包含 warehouse_wall_000 接触对，
    # 但普通地面（warehouse_ground）接触不应出现在 pairs 中。
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <geom name="warehouse_ground" type="plane" size="5 5 0.1"/>
            <geom name="warehouse_wall_000" type="box"
                  pos="0 0 0.5" size="0.5 0.5 0.5"/>
            <body name="robot" pos="0 0 1.1">
              <freejoint/>
              <geom name="robot_collision" type="sphere" size="0.2"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    summary = warehouse_contact_summary(model, data)
    assert summary['count'] >= 1
    assert any(
        'warehouse_wall_000' in pair for pair in summary['pairs']
    )
    assert all('warehouse_ground' not in pair for pair in summary['pairs'])
