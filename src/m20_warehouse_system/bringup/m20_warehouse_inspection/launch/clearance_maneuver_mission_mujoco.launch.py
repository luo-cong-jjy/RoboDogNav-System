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
# 文件：clearance_maneuver_mission_mujoco.launch.py
# 功能：启动「单障碍物双向绕行（single-obstacle double-bypass）」MuJoCo
#       基准测试。本 launch 是轻量入口：选定 clearance_maneuver_system.yaml
#       场景配置，并使用生产级保守间隙配置（clearance_conservative.yaml），
#       把参数透传给完整的 MuJoCo 巡检任务 launch
#       （inspection_mission_mujoco.launch.py）。
# ============================================================================

"""Launch the single-obstacle double-bypass MuJoCo benchmark."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值


def generate_launch_description() -> LaunchDescription:
    """Compose the benchmark with production conservative clearance."""
    # —— 路径准备 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    navigation = Path(  # m20_scan_navigation 包的 share 目录（存放间隙配置）
        get_package_share_directory('m20_scan_navigation')
    )
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 声明基准场景系统配置参数
                'system_config',
                default_value=str(  # 默认使用双向绕行基准场景配置
                    integration
                    / 'config'
                    / 'clearance_maneuver_system.yaml'
                ),
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument(  # 默认打开 MuJoCo 查看器
                'use_mujoco_viewer',
                default_value='true',
            ),
            DeclareLaunchArgument(  # 单步导航超时（秒）
                'navigation_timeout_sec',
                default_value='120.0',
            ),
            DeclareLaunchArgument(  # 默认不启用速度反馈
                'velocity_feedback_enabled',
                default_value='false',
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
                    'system_config': LaunchConfiguration('system_config'),  # 基准场景系统配置
                    'clearance_config': str(  # 强制使用生产级保守间隙配置
                        navigation
                        / 'config'
                        / 'clearance_conservative.yaml'
                    ),
                    'use_rviz': LaunchConfiguration('use_rviz'),  # 透传：是否启动 RViz
                    'use_mujoco_viewer': LaunchConfiguration(  # 透传：是否打开 MuJoCo 查看器
                        'use_mujoco_viewer'
                    ),
                    'use_planner': 'true',  # 强制启用规划器
                    'navigation_timeout_sec': LaunchConfiguration(  # 透传：单步导航超时
                        'navigation_timeout_sec'
                    ),
                    'velocity_feedback_enabled': LaunchConfiguration(  # 透传：速度反馈开关
                        'velocity_feedback_enabled'
                    ),
                }.items(),
            ),
        ]
    )
