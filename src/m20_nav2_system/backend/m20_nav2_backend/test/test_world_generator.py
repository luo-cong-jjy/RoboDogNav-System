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
# 文件：test_world_generator.py
# 用途：JSON 地图元数据 → MJCF 世界「确定性生成」的集成测试（pytest）。
#       依赖 bringup/m20_warehouse_inspection 的 dense_four_corner 系统
#       配置与 m20_nav2_description 的网格，因此属于集成测试。
#       覆盖：
#       - 稠密配置生成两个碰撞区域（F1/F2 楼层）且数量断言
#         （2 层 / 492 障碍物 / 8 墙段 / 16 执行器）；
#       - 生成结果字节级确定（同一输入两次生成完全一致）。
# ============================================================

"""Tests for deterministic JSON-to-MJCF world generation."""

from pathlib import Path

import mujoco

from m20_nav2_backend.world_generator import generate_world

# 包根、系统根与相关依赖包路径。
PACKAGE = Path(__file__).parents[1]
SYSTEM_ROOT = PACKAGE.parents[1]
INTEGRATION = SYSTEM_ROOT / 'bringup' / 'm20_warehouse_inspection'
DESCRIPTION = SYSTEM_ROOT / 'common' / 'm20_nav2_description'


def test_dense_profile_generates_both_collision_regions(tmp_path):
    # 用 dense_four_corner_system.yaml 生成世界并校验：
    #   - 报告数量：2 层、492 个障碍物、8 段墙、16 个执行器；
    #   - MuJoCo 能加载生成的模型（nu==16）；
    #   - 两层的首个障碍物 geom（warehouse_obstacle_F1_0000 /
    #     warehouse_obstacle_F2_0000）都存在。
    output = tmp_path / 'dense.xml'
    report = generate_world(
        system_config=(
            INTEGRATION / 'config' / 'dense_four_corner_system.yaml'
        ),
        package_root=INTEGRATION,
        robot_template=PACKAGE / 'models' / 'm20_robot.xml',
        mesh_directory=DESCRIPTION / 'meshes',
        output_path=output,
        validate_model=True,
    )
    assert report.floor_count == 2
    assert report.obstacle_count == 492
    assert report.wall_count == 8
    assert report.actuator_count == 16

    model = mujoco.MjModel.from_xml_path(str(output))
    assert model.nu == 16
    assert mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        'warehouse_obstacle_F1_0000',
    ) >= 0
    assert mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        'warehouse_obstacle_F2_0000',
    ) >= 0


def test_generation_is_byte_deterministic(tmp_path):
    # 确定性测试：同一输入生成两次，输出文件字节必须完全一致
    # （关闭 MuJoCo 加载校验，只比较生成器的确定性）。
    arguments = {
        'system_config': (
            INTEGRATION / 'config' / 'dense_four_corner_system.yaml'
        ),
        'package_root': INTEGRATION,
        'robot_template': PACKAGE / 'models' / 'm20_robot.xml',
        'mesh_directory': DESCRIPTION / 'meshes',
        'validate_model': False,
    }
    first = tmp_path / 'first.xml'
    second = tmp_path / 'second.xml'
    generate_world(output_path=first, **arguments)
    generate_world(output_path=second, **arguments)
    assert first.read_bytes() == second.read_bytes()
