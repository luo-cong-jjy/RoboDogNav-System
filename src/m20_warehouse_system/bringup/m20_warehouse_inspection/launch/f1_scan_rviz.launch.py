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
# 文件：f1_scan_rviz.launch.py
# 功能：启动完整的阶段 2（phase-2）F1 楼层 RViz 导航系统。
#       本 launch 把 F1 楼层地图、M20 官方模型 TF、SCAN 仿真传感、
#       SCAN 导航栈（f1_scan.launch.py）、安全监督/碰撞防护节点
#       以及 RViz 连接在一起，作为 F1 单楼层仿真的完整入口。
#       与上游 SCAN 栈共用同一份 RViz 契约（default.rviz）。
# ============================================================================

"""Launch the complete phase-2 F1 RViz navigation system."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

import yaml  # YAML 解析库：读取系统配置中的初始位姿

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点


def generate_launch_description() -> LaunchDescription:
    """Connect map, M20 model, sensing, SCAN, safety, and RViz."""
    # —— 路径准备：解析各依赖包的 share 目录 ——
    integration = Path(  # 本包 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    description = Path(  # M20 官方描述包 share 目录（URDF 模型）
        get_package_share_directory('m20_official_description')
    )
    scan = Path(get_package_share_directory('m20_scan_navigation'))  # SCAN 导航包 share 目录
    scan_vendor = Path(get_package_share_directory('m20_scan_planner'))  # SCAN 规划器（vendor）包 share 目录
    simulation = Path(get_package_share_directory('m20_warehouse_sim'))  # 仓库仿真包 share 目录
    core = Path(get_package_share_directory('m20_inspection_core'))  # 巡检核心包 share 目录
    locomotion = Path(  # M20 运动控制包 share 目录
        get_package_share_directory('m20_locomotion_control')
    )

    system_config = integration / 'config' / 'flat_multifloor_system.yaml'  # 系统配置（楼层/初始位姿）
    with system_config.open('r', encoding='utf-8') as stream:
        initial_pose = yaml.safe_load(stream)['floors']['F1']['initial_pose']  # 从配置读取 F1 楼层初始位姿 [x, y, yaw]
    model = description / 'urdf' / 'm20_official.urdf'  # M20 官方 URDF 模型
    # Keep the lightweight F1 entry on the same RViz contract as the native
    # SCAN stack.  The former phase-2 file still referenced retired /m20/*
    # aliases, so its goal tool and most planner displays had no subscribers.
    # （与上游 SCAN 栈保持同一 RViz 契约：直接加载 vendor 包的 default.rviz；
    #   旧版 phase-2 文件仍引用已废弃的 /m20/* 别名，导致目标工具与多数规划器
    #   显示没有订阅者，故此处改用官方 RViz 配置）
    rviz = scan_vendor / 'rviz' / 'default.rviz'
    # —— launch 参数 ——
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    use_local_sensing = LaunchConfiguration('use_local_sensing')  # 是否使用 SCAN 本地传感仿真（PCD 射线投射）
    use_planner = LaunchConfiguration('use_planner')  # 是否启动规划器

    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_local_sensing', default_value='true'),  # 默认启用本地传感仿真
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            # —— 启动平面多楼层地图服务器 ——
            Node(  # m20_flat_map_server：加载多楼层地图并发布 F1 楼层
                package='m20_warehouse_inspection',  # 所属功能包
                executable='m20_flat_map_server',  # 可执行文件名（本包脚本）
                name='flat_multifloor_map_server',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'config_path': str(system_config),  # 系统配置
                        'package_root': str(integration),  # 包根目录
                        'initial_floor': 'F1',  # 初始楼层 F1
                        # Transient-local consumers receive one snapshot.
                        # Avoid rebuilding inflated maps every ten seconds.
                        # （瞬态本地订阅者只需一份快照；避免每 10 秒重建膨胀地图）
                        'republish_period_sec': 0.0,  # 重发布周期 0：只发布一次
                    }
                ],
            ),
            # —— 启动 TF 发布节点 ——
            Node(  # robot_state_publisher：发布 M20 官方模型 TF
                package='robot_state_publisher',  # 所属功能包
                executable='robot_state_publisher',  # 可执行文件名
                name='m20_robot_state_publisher',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        # Direct official M20 geometry/kinematic tree. The
                        # simulated lidar remains a data source, not a model.
                        # （直接使用官方 M20 几何/运动学树；仿真雷达只是数据源，不参与模型）
                        'robot_description': model.read_text(encoding='utf-8')  # 读取官方 URDF 内容
                    }
                ],
            ),
            # SCAN's simulator publishes local sensing in "world". During the
            # flat F1 phase, world and map intentionally share one origin.
            # （SCAN 仿真器在 "world" 系发布本地传感；平面 F1 阶段 world 与 map 故意共原点）
            Node(  # 发布 map→world 静态变换（单位变换，共原点）
                package='tf2_ros',  # 所属功能包
                executable='static_transform_publisher',  # 静态变换发布可执行文件
                name='m20_map_to_scan_world',  # 节点名
                # Positional arguments work on both Foxy and Humble.  Foxy
                # does not implement Humble's named-argument CLI.
                # （位置参数在 Foxy 与 Humble 上都可用；Foxy 未实现 Humble 的命名参数 CLI）
                arguments=[
                    '0', '0', '0', '0', '0', '0',  # 平移/旋转均为 0
                    'map', 'world',  # map → world
                ],
                output='screen',  # 日志输出到屏幕
            ),
            # —— 启动 RViz 运动学仿真后端 ——
            Node(  # m20_rviz_kinematic_backend：在 RViz 中做平面运动学仿真
                package='m20_warehouse_sim',  # 所属功能包
                executable='m20_rviz_kinematic_backend',  # 可执行文件名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(simulation / 'config' / 'rviz_kinematic.yaml'),  # 运动学后端配置
                    {
                        'initial_x': float(initial_pose[0]),  # 初始 X（来自配置）
                        'initial_y': float(initial_pose[1]),  # 初始 Y（来自配置）
                        'initial_yaw': float(initial_pose[2]),  # 初始偏航角（来自配置）
                    },
                ],
            ),
            # —— 启动导航适配器 ——
            Node(  # m20_navigation_adapter：把导航命令适配为 M20 运动命令
                package='m20_locomotion_control',  # 所属功能包
                executable='m20_navigation_adapter',  # 可执行文件名
                name='m20_navigation_adapter',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(locomotion / 'config' / 'sdk_locomotion.yaml')  # 运动 SDK 配置
                ],
            ),
            # —— 启动安全监督节点 ——
            Node(  # m20_safety_supervisor：安全监督（急停/楼层切换门控等）
                package='m20_inspection_core',  # 所属功能包
                executable='m20_safety_supervisor',  # 可执行文件名
                output='screen',  # 日志输出到屏幕
                parameters=[str(core / 'config' / 'safety.yaml')],  # 安全配置
            ),
            # —— 启动碰撞防护节点 ——
            Node(  # m20_collision_guard：独立碰撞防护（基于 SCAN 本地传感）
                package='m20_inspection_core',  # 所属功能包
                executable='m20_collision_guard',  # 可执行文件名
                name='m20_collision_guard',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(  # SCAN 原生碰撞防护配置
                        core
                        / 'config'
                        / 'collision_guard_scan_native.yaml'
                    )
                ],
            ),
            # —— 包含 SCAN 导航栈 ——
            IncludeLaunchDescription(  # 包含 m20_scan_navigation 的 f1_scan.launch.py（规划/控制/传感）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(scan / 'launch' / 'f1_scan.launch.py')
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'use_local_sensing': use_local_sensing,  # 透传：本地传感开关
                    'use_planner': use_planner,  # 透传：规划器开关
                }.items(),
            ),
            # —— 可选启动 RViz ——
            Node(  # 启动 RViz（使用 vendor 的默认配置）
                package='rviz2',  # 所属功能包
                executable='rviz2',  # 可执行文件名
                name='m20_phase2_rviz',  # 节点名
                arguments=['-d', str(rviz)],  # 加载指定 RViz 配置文件
                output='screen',  # 日志输出到屏幕
                condition=IfCondition(use_rviz),  # 仅当 use_rviz=true 时才启动
            ),
        ]
    )
