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
# 文件：dense_four_corner_mission_rviz.launch.py
# 功能：启动「高密度四角巡逻（high-density four-corner patrol）」专项
#       配置的 RViz 仿真巡检任务。本 launch 是轻量入口：只负责选定
#       高密度四角场景的系统配置文件（dense_four_corner_system.yaml），
#       并把 use_rviz / use_planner / run_acceptance / acceptance_mode
#       等参数透传给完整的巡检任务 launch
#       （inspection_mission_rviz.launch.py），复用其完整的任务图。
# ============================================================================

"""Launch the independent high-density four-corner patrol profile."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值


def generate_launch_description() -> LaunchDescription:
    """Select the dense profile while reusing the complete mission graph."""
    # —— 路径准备：解析本包 share 目录并拼接高密度四角配置文件路径 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    system_config = (  # 高密度四角场景的系统配置文件
        integration / 'config' / 'dense_four_corner_system.yaml'
    )
    # —— launch 参数（可被命令行覆盖）——
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    use_planner = LaunchConfiguration('use_planner')  # 是否启动规划器
    run_acceptance = LaunchConfiguration('run_acceptance')  # 是否运行验收测试
    acceptance_mode = LaunchConfiguration('acceptance_mode')  # 验收模式（如 quick）
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument('run_acceptance', default_value='false'),  # 默认不运行验收
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),  # 默认验收模式 quick
            # —— 包含完整巡检任务 launch ——
            IncludeLaunchDescription(  # 包含 inspection_mission_rviz.launch.py，复用完整的任务图
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_rviz.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'system_config': str(system_config),  # 高密度四角场景系统配置文件（绝对路径）
                    'use_rviz': use_rviz,  # 透传：是否启动 RViz
                    'use_planner': use_planner,  # 透传：是否启动规划器
                    'run_acceptance': run_acceptance,  # 透传：是否运行验收
                    'acceptance_mode': acceptance_mode,  # 透传：验收模式
                }.items(),
            ),
        ]
    )
