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
# 文件：inspection_mission_hardware.launch.py
# 功能：把 M20 仓库巡检任务接到工厂真机运动层（real-robot motion layer）。
#       本 launch 组合：
#         - Elevator-LIO 定位（lio 包的 start_ros2.launch.py，可选启动）
#         - 硬件定位适配器（m20_hardware_localization_adapter：把 LIO
#           位姿 + 工厂实测 Twist 融合为规范导航 Odometry）
#         - 完整 RViz 巡检任务图（inspection_mission_rviz.launch.py，
#           motion_backend=external、execution_profile=m20_safe 等）
#         - 官方运动层（official_locomotion.launch.py，连接真机运输层）
#       注意：硬件站点配置（site profile）必须已安装，否则报错。
# ============================================================================

"""M20 warehouse inspection wired to the factory real-robot motion layer."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 参数替换、路径拼接替换
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.substitutions import FindPackageShare  # 运行时查找包的 share 目录


def generate_launch_description() -> LaunchDescription:
    """Compose Elevator-LIO, navigation, and guarded factory locomotion."""
    # —— 路径准备 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    locomotion = Path(  # M20 运动控制包 share 目录
        get_package_share_directory('m20_locomotion_control')
    )
    default_system = (  # 默认硬件站点配置（PAO F1 模板）
        integration / 'config' / 'sites' / 'm20_pao_f1_template.yaml'
    )
    if not default_system.is_file():  # 站点配置未安装则报错
        raise RuntimeError(
            'hardware site profile is not installed; pass system_config '
            'only after installing a surveyed-site configuration'
        )
    default_capability = (  # 默认工厂敏捷平层能力档案
        locomotion
        / 'config'
        / 'm20_factory_agile_flat_capabilities.yaml'
    )
    # —— launch 参数（可被命令行覆盖）——
    system_config = LaunchConfiguration('system_config')  # 系统配置文件路径
    capability = LaunchConfiguration('locomotion_capability_config')  # 运动能力档案

    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 系统配置文件
                'system_config', default_value=str(default_system)  # 默认 PAO F1 站点模板
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument(  # 是否启动 Elevator-LIO
                'start_elevator_lio',
                default_value='true',  # 默认由本 launch 启动 LIO
                description=(
                    'Start the validated Elevator-LIO package in this '
                    'bring-up. Set false when LIO is managed separately.'
                ),
            ),
            DeclareLaunchArgument(  # LIO 根配置文件名
                'lio_config_path',
                default_value='root_config_m20_navigation_relocation.yaml',  # 默认导航+重定位根配置
                description=(
                    'Elevator-LIO root YAML filename under lio/yaml. The '
                    'hardware default loads the commissioned F1 map and '
                    'keeps its TF isolated as lio_world -> lio_base_link; '
                    'mapping is a separate commissioning command.'
                ),
            ),
            DeclareLaunchArgument(  # 是否启动定位适配器
                'start_localization_adapter',
                default_value='true',  # 默认启动
                description=(
                    'Fuse Elevator-LIO body pose with factory measured '
                    'Twist into the canonical navigation Odometry.'
                ),
            ),
            DeclareLaunchArgument(  # LIO 车体位姿话题
                'lio_body_pose_topic',
                default_value='/LIO/odom_vehicle',  # LIO 输出的里程计话题
            ),
            DeclareLaunchArgument(  # 规范导航位姿话题
                'body_pose_topic',
                default_value='/m20/localization/body_pose',  # 活动地图系中的真实定位里程计
                description=(
                    'Real localization Odometry in the active map frame.'
                ),
            ),
            DeclareLaunchArgument(  # 导航点云话题
                'navigation_cloud_topic',
                default_value='/m20/localization/cloud',  # LIO 去畸变点云（别名到 SCAN 世界系）
                description=(
                    'Elevator-LIO deskewed live cloud message-aliased to the '
                    'SCAN world frame; raw lidar points are not sufficient.'
                ),
            ),
            DeclareLaunchArgument(  # 传感器位姿话题
                'sensor_pose_topic',
                default_value='/m20/localization/sensor_pose',  # LIO 传感器位姿
            ),
            DeclareLaunchArgument(  # 重定位服务
                'relocation_service',
                default_value='/m20/localization/set_pose',  # 逐楼层重定位服务
                description=(
                    'Per-floor localization reinitialization service. It '
                    'must implement '
                    'm20_warehouse_interfaces/SetSimulationPose; the '
                    'simulation-oriented type name is retained only for '
                    'wire compatibility.'
                ),
            ),
            DeclareLaunchArgument(  # 运动能力档案
                'locomotion_capability_config',
                default_value=str(default_capability),  # 默认工厂敏捷平层档案
            ),
            DeclareLaunchArgument(  # 真机主机地址
                'robot_host', default_value='10.21.31.103'
            ),
            DeclareLaunchArgument(  # 工厂运输层类型
                'factory_transport', default_value='basic_server'
            ),
            DeclareLaunchArgument(  # 命令所有权确认
                'command_ownership_confirmed', default_value='false'
            ),
            DeclareLaunchArgument(  # 是否自动使能运动
                'auto_enable_motion', default_value='false'
            ),
            # —— 可选包含 Elevator-LIO 定位栈 ——
            IncludeLaunchDescription(  # 包含 lio 包的 start_ros2.launch.py（Elevator-LIO 定位）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    PathJoinSubstitution(  # 运行时拼接 launch 路径
                        [
                            FindPackageShare('lio'),  # 查找 lio 包
                            'launch',
                            'start_ros2.launch.py',
                        ]
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'config_path': LaunchConfiguration('lio_config_path'),  # LIO 根配置
                    # The integrated RViz already contains SCAN and robot
                    # displays; do not create a second RViz process.
                    # （集成 RViz 已含 SCAN 与机器人显示，不重复创建第二个 RViz 进程）
                    'use_rviz': 'false',  # 强制不启动 LIO 自带 RViz
                }.items(),
                condition=IfCondition(  # 仅当 start_elevator_lio=true 时才启动 LIO
                    LaunchConfiguration('start_elevator_lio')
                ),
            ),
            # —— 可选启动硬件定位适配器 ——
            Node(  # m20_hardware_localization_adapter：LIO 位姿 + 实测 Twist → 规范导航 Odometry
                package='m20_warehouse_inspection',  # 所属功能包
                executable='m20_hardware_localization_adapter',  # 可执行文件名（本包脚本）
                name='m20_hardware_localization_adapter',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'input_odometry_topic': LaunchConfiguration(  # 输入：LIO 车体位姿话题
                            'lio_body_pose_topic'
                        ),
                        'output_odometry_topic': LaunchConfiguration(  # 输出：规范导航位姿话题
                            'body_pose_topic'
                        ),
                        'measured_twist_topic': (  # 输入：工厂实测 Twist
                            '/m20/locomotion/measured_twist'
                        ),
                        'expected_world_frame': 'lio_world',  # 期望 LIO 世界系
                        'expected_body_frame': 'lio_base_link',  # 期望 LIO 机体系
                        'expected_sensor_frame': 'lio_imu',  # 期望 LIO 传感器系
                        'output_world_frame': 'world',  # 输出世界系
                        'output_body_frame': 'base_link',  # 输出机体系
                        'output_sensor_frame': 'm20_lio_sensor',  # 输出传感器系
                    }
                ],
                condition=IfCondition(  # 仅当 start_localization_adapter=true 时启动
                    LaunchConfiguration('start_localization_adapter')
                ),
            ),
            # —— 包含完整 RViz 巡检任务图（外部运动后端）——
            IncludeLaunchDescription(  # 包含 inspection_mission_rviz.launch.py
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_rviz.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'system_config': system_config,  # 系统配置
                    'use_rviz': LaunchConfiguration('use_rviz'),  # 透传：是否启动 RViz
                    'use_planner': LaunchConfiguration('use_planner'),  # 透传：是否启动规划器
                    'use_local_sensing': 'false',  # 真机不仿真本地传感（使用实时点云）
                    'motion_backend': 'external',  # 运动后端为外部（真机运动层）
                    # Real hardware remains fail-closed until a dedicated
                    # native-SCAN/M20 physical acceptance has been completed.
                    # （真机在完成专项原生 SCAN/M20 物理验收前保持失效关闭）
                    'execution_profile': 'm20_safe',  # 真机强制使用 m20_safe 安全档案
                    'velocity_feedback_enabled': 'true',  # 真机启用速度反馈
                    'velocity_feedback_source': 'twist',  # 速度反馈来源为实测 Twist
                    'velocity_feedback_twist_topic': (  # 实测 Twist 话题
                        '/m20/locomotion/measured_twist'
                    ),
                    'body_pose_topic': LaunchConfiguration(  # 透传：规范位姿话题
                        'body_pose_topic'
                    ),
                    'navigation_cloud_topic': LaunchConfiguration(  # 透传：导航点云话题
                        'navigation_cloud_topic'
                    ),
                    'sensor_pose_topic': LaunchConfiguration(  # 透传：传感器位姿话题
                        'sensor_pose_topic'
                    ),
                    'relocation_service': LaunchConfiguration(  # 透传：重定位服务
                        'relocation_service'
                    ),
                    'locomotion_capability_config': capability,  # 透传：运动能力档案
                    'navigation_timeout_sec': '300.0',  # 真机单步导航超时 300 秒
                }.items(),
            ),
            # —— 包含官方运动层 ——
            IncludeLaunchDescription(  # 包含 official_locomotion.launch.py（真机运动层）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        locomotion
                        / 'launch'
                        / 'official_locomotion.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'factory_transport': LaunchConfiguration(  # 透传：工厂运输层类型
                        'factory_transport'
                    ),
                    'locomotion_capability_config': capability,  # 运动能力档案
                    'robot_host': LaunchConfiguration('robot_host'),  # 透传：真机主机地址
                    'command_ownership_confirmed': LaunchConfiguration(  # 透传：命令所有权确认
                        'command_ownership_confirmed'
                    ),
                    'auto_enable_motion': LaunchConfiguration(  # 透传：自动使能运动
                        'auto_enable_motion'
                    ),
                }.items(),
            ),
        ]
    )
