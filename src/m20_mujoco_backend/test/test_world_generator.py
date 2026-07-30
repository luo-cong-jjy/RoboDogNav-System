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

"""Tests for deterministic JSON-to-MJCF world generation."""

from pathlib import Path

import mujoco

from m20_mujoco_backend.world_generator import generate_world


WORKSPACE_SRC = Path(__file__).parents[2]
PACKAGE = Path(__file__).parents[1]
INTEGRATION = WORKSPACE_SRC / 'm20_warehouse_inspection'
DESCRIPTION = WORKSPACE_SRC / 'm20_official_description'


def test_dense_profile_generates_both_collision_regions(tmp_path):
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
