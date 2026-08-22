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
# 文件：mujoco_backend.launch.py
# 用途：M20 MuJoCo 仿真后端的 ROS2 launch 入口。
#       - 运行时（OpaqueFunction）读取系统配置 YAML，调用
#         world_generator.generate_world 生成仓库碰撞世界（含障碍物/
#         围墙），输出到 /tmp 缓存路径；
#       - 启动 m20_mujoco_backend 物理节点（无头，use_viewer=False，
#         参数叠加 config/mujoco_backend.yaml 与启动参数覆盖）；
#       - 可选启动 m20_mujoco_viewer 只读显示节点（IfCondition
#         use_viewer），并以 nice -n 10 低优先级运行，避免抢占
#         官方策略与 1kHz 物理进程的调度；
#       - 支持 initial_x/y/yaw 冷启动覆盖（留空则用 F1 initial_pose）。
# ============================================================

"""Generate the selected warehouse world and start its MuJoCo backend."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from m20_mujoco_backend.world_generator import (
    cached_world_path,
    generate_world,
)


def _runtime_actions(context):
    # ---------- 运行时解析路径与生成世界 ----------
    # system_config：仓库系统 YAML（含 floors 配置）；
    # package_root：地图元数据相对根（为空时从配置安装位置推断）。
    system_config = Path(
        LaunchConfiguration('system_config').perform(context)
    ).expanduser().resolve()
    configured_root = LaunchConfiguration('package_root').perform(context)
    package_root = (
        Path(configured_root).expanduser().resolve()
        if configured_root
        else system_config.parent.parent
    )
    # 包共享目录：本包（模型模板/配置）与 m20_official_description（网格）。
    backend_share = Path(
        get_package_share_directory('m20_mujoco_backend')
    )
    description_share = Path(
        get_package_share_directory('m20_official_description')
    )
    # 生成世界（原子写、字节确定、带 MuJoCo 加载校验）。
    output_path = cached_world_path(system_config)
    report = generate_world(
        system_config=system_config,
        package_root=package_root,
        robot_template=backend_share / 'models' / 'm20_robot.xml',
        mesh_directory=description_share / 'meshes',
        output_path=output_path,
        validate_model=True,
    )
    # 读取系统配置中的 F1 初始位姿，作为冷启动默认值。
    with system_config.open('r', encoding='utf-8') as stream:
        system = yaml.safe_load(stream)
    initial_pose = system['floors']['F1']['initial_pose']

    def initial_component(argument: str, index: int) -> float:
        """Resolve an optional simulation-only cold-start pose override."""
        # 启动参数覆盖优先：参数非空则用参数值，否则用 F1 initial_pose。
        configured = LaunchConfiguration(argument).perform(context).strip()
        return float(configured) if configured else float(initial_pose[index])

    return [
        # ---------- 1) 物理后端节点（无头） ----------
        Node(
            package='m20_mujoco_backend',
            executable='m20_mujoco_backend',
            name='m20_mujoco_backend',
            output='screen',
            parameters=[
                # 先加载包内默认参数文件，再用字典覆盖关键参数。
                str(backend_share / 'config' / 'mujoco_backend.yaml'),
                {
                    'model_xml_path': report.output_path,
                    # Keep GUI synchronization outside the 1 kHz physics and
                    # official-policy feedback process.
                    # 保持 GUI 同步在 1kHz 物理与官方策略反馈进程之外
                    # （显示由独立的 viewer 进程负责）。
                    'use_viewer': False,
                    'viewer_distance': LaunchConfiguration(
                        'viewer_distance'
                    ),
                    'viewer_azimuth': LaunchConfiguration(
                        'viewer_azimuth'
                    ),
                    'viewer_elevation': LaunchConfiguration(
                        'viewer_elevation'
                    ),
                    'viewer_max_fps': LaunchConfiguration(
                        'viewer_max_fps'
                    ),
                    'initial_x': initial_component('initial_x', 0),
                    'initial_y': initial_component('initial_y', 1),
                    'initial_yaw': initial_component('initial_yaw', 2),
                    'real_time_factor': LaunchConfiguration(
                        'real_time_factor'
                    ),
                },
            ],
        ),
        # ---------- 2) 只读显示节点（可选） ----------
        Node(
            package='m20_mujoco_backend',
            executable='m20_mujoco_viewer',
            name='m20_mujoco_viewer',
            output='screen',
            # Rendering is diagnostic-only.  Keep it below the official
            # policy and 1 kHz physics processes in the Linux scheduler.
            # 渲染只是诊断用途：用 nice -n 10 降低其调度优先级，
            # 不抢占官方策略与 1kHz 物理进程。
            prefix='nice -n 10',
            # 仅当 use_viewer=true 时启动。
            condition=IfCondition(LaunchConfiguration('use_viewer')),
            parameters=[
                {
                    'model_xml_path': report.output_path,
                    'distance': LaunchConfiguration('viewer_distance'),
                    'azimuth': LaunchConfiguration('viewer_azimuth'),
                    'elevation': LaunchConfiguration('viewer_elevation'),
                    'max_fps': LaunchConfiguration('viewer_max_fps'),
                    'initial_x': initial_component('initial_x', 0),
                    'initial_y': initial_component('initial_y', 1),
                    'initial_yaw': initial_component('initial_yaw', 2),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the backend's profile and rendering controls."""
    # 声明全部 launch 参数：
    #   system_config：仓库系统 YAML 绝对路径（必填）
    #   package_root：地图路径根（空则自动推断）
    #   use_viewer：是否打开原生 3D viewer（无头跑设 false）
    #   viewer_*：跟随相机参数；real_time_factor：实时倍率
    #   initial_x/y/yaw：冷启动位姿覆盖（空用 F1 initial_pose）
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'system_config',
                description='Absolute warehouse system YAML path.',
            ),
            DeclareLaunchArgument(
                'package_root',
                default_value='',
                description=(
                    'Root containing map paths in system_config; inferred '
                    'from the configuration install location when empty.'
                ),
            ),
            DeclareLaunchArgument(
                'use_viewer',
                default_value='true',
                description=(
                    'Open the native MuJoCo 3D viewer; set false for '
                    'headless runs.'
                ),
            ),
            DeclareLaunchArgument(
                'viewer_distance',
                default_value='4.0',
                description=(
                    'Robot-centered direct-follow distance in metres.'
                ),
            ),
            DeclareLaunchArgument(
                'viewer_azimuth',
                default_value='90.0',
                description='Initial direct-follow camera azimuth in degrees.',
            ),
            DeclareLaunchArgument(
                'viewer_elevation',
                default_value='-89.0',
                description='Initial near-vertical camera elevation in degrees.',
            ),
            DeclareLaunchArgument(
                'viewer_max_fps',
                default_value='30.0',
                description='Maximum isolated viewer wall-clock frame rate.',
            ),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),
            DeclareLaunchArgument(
                'initial_x',
                default_value='',
                description=(
                    'Optional MuJoCo cold-start x override. Empty uses the '
                    'F1 initial_pose from system_config.'
                ),
            ),
            DeclareLaunchArgument(
                'initial_y',
                default_value='',
                description=(
                    'Optional MuJoCo cold-start y override. Empty uses the '
                    'F1 initial_pose from system_config.'
                ),
            ),
            DeclareLaunchArgument(
                'initial_yaw',
                default_value='',
                description=(
                    'Optional MuJoCo cold-start yaw override. Empty uses the '
                    'F1 initial_pose from system_config.'
                ),
            ),
            # OpaqueFunction 在 launch 运行时执行世界生成与节点实例化。
            OpaqueFunction(function=_runtime_actions),
        ]
    )
