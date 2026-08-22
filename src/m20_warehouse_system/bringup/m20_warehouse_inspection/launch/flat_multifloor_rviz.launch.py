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
# 文件：flat_multifloor_rviz.launch.py
# 功能：阶段 1（phase-1）的平面多楼层（flat F1/F2）地图基线 launch。
#       只启动平面多楼层地图服务器（m20_flat_map_server），把指定楼层
#       （默认 F1）的地图发布到活动地图话题，并可选启动 RViz 展示
#       双区域总览（flat_multifloor.rviz 配置）。
#       本文件是 phase-1 的最小化入口，不含导航/传感等其他节点。
# ============================================================================

"""Launch the phase-1 flat F1/F2 map baseline and optional RViz."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument  # 启动动作：声明 launch 参数
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作（如 use_rviz 为 true 时启动 RViz）
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点


def generate_launch_description() -> LaunchDescription:
    """Build the phase-1 launch graph."""
    # —— 路径准备 ——
    package_root = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = package_root / 'config' / 'flat_multifloor_system.yaml'  # 默认平面多楼层系统配置
    default_rviz = package_root / 'rviz' / 'flat_multifloor.rviz'  # 默认 RViz 配置文件（双区域总览）

    # —— launch 参数（可被命令行覆盖）——
    config_path = LaunchConfiguration('config_path')  # 系统配置文件路径
    initial_floor = LaunchConfiguration('initial_floor')  # 初始发布的楼层
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    rviz_config = LaunchConfiguration('rviz_config')  # RViz 配置文件路径

    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(
                'config_path',
                default_value=str(default_config),  # 默认系统级平面多楼层配置
                description='System-level flat multi-floor configuration',
            ),
            DeclareLaunchArgument(
                'initial_floor',
                default_value='F1',  # 默认只发布 F1 楼层
                description='Only this floor is published to active-map topics',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',  # 默认启动 RViz
                description='Start RViz for the two-region overview',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=str(default_rviz),  # 默认 RViz 配置
                description='RViz configuration file',
            ),
            # —— 启动平面多楼层地图服务器 ——
            Node(  # m20_flat_map_server：加载多楼层地图并发布指定楼层到活动地图话题
                package='m20_warehouse_inspection',  # 所属功能包
                executable='m20_flat_map_server',  # 可执行文件名（本包脚本）
                name='flat_multifloor_map_server',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'config_path': config_path,  # 系统配置（定义楼层/地图偏移）
                        'package_root': str(package_root),  # 包根目录（定位地图资源）
                        'initial_floor': initial_floor,  # 初始楼层
                    }
                ],
            ),
            # —— 可选启动 RViz ——
            Node(  # 启动 RViz 展示双区域总览
                package='rviz2',  # 所属功能包
                executable='rviz2',  # 可执行文件名
                name='flat_multifloor_rviz',  # 节点名
                arguments=['-d', rviz_config],  # 加载指定 RViz 配置文件
                output='screen',  # 日志输出到屏幕
                condition=IfCondition(use_rviz),  # 仅当 use_rviz=true 时才启动
            ),
        ]
    )
