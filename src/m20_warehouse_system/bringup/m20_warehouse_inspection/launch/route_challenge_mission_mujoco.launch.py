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

# ============================================================================
# 文件：route_challenge_mission_mujoco.launch.py
# 功能：通过 MuJoCo 物理后端运行「路线障碍挑战（obstacle-interruption）」
#       专项配置的完整巡检任务。本 launch 是轻量入口：只负责选定挑战场景
#       的系统配置文件（route_challenge_system.yaml），并把 use_rviz /
#       use_planner / use_mujoco_viewer / mujoco_viewer_distance 等参数
#       透传给完整的 MuJoCo 巡检任务 launch
#       （inspection_mission_mujoco.launch.py），复用其完整的 MuJoCo 图。
# ============================================================================

"""Run the obstacle-interruption profile through the MuJoCo backend."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值


def generate_launch_description() -> LaunchDescription:
    """Select route-challenge assets while reusing the full MuJoCo graph."""
    # —— 路径准备：解析本包 share 目录并拼接挑战配置文件路径 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    system_config = (  # 路线障碍挑战场景的系统配置文件
        integration / 'config' / 'route_challenge_system.yaml'
    )
    # —— launch 参数（可被命令行覆盖）——
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    use_planner = LaunchConfiguration('use_planner')  # 是否启动规划器
    use_mujoco_viewer = LaunchConfiguration('use_mujoco_viewer')  # 是否打开 MuJoCo 3D 查看器
    mujoco_viewer_distance = LaunchConfiguration(  # MuJoCo 相机跟随距离（米）
        'mujoco_viewer_distance'
    )
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument(
                'use_mujoco_viewer',
                default_value='true',  # 默认打开 MuJoCo 查看器
            ),
            DeclareLaunchArgument(
                'mujoco_viewer_distance',
                default_value='4.0',  # 默认相机距离 4 米
            ),
            # —— 包含完整 MuJoCo 巡检任务 launch ——
            IncludeLaunchDescription(  # 包含 inspection_mission_mujoco.launch.py，复用完整 MuJoCo 图
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_mujoco.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'system_config': str(system_config),  # 挑战场景系统配置文件（绝对路径）
                    'use_rviz': use_rviz,  # 透传：是否启动 RViz
                    'use_planner': use_planner,  # 透传：是否启动规划器
                    'use_mujoco_viewer': use_mujoco_viewer,  # 透传：是否打开 MuJoCo 查看器
                    'mujoco_viewer_distance': mujoco_viewer_distance,  # 透传：相机距离
                }.items(),
            ),
        ]
    )
