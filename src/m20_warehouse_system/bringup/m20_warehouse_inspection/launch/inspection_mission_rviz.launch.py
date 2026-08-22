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
# 文件：inspection_mission_rviz.launch.py
# 功能：阶段 4（phase-4）自动平面多楼层仓库巡检的完整任务 launch。
#       本 launch 组合：
#         - 完整的平面多楼层 SCAN 仿真图（multifloor_scan_rviz.launch.py：
#           地图服务器、M20 模型 TF、RViz 运动学后端或外部后端、
#           安全监督/碰撞防护、SCAN 导航栈、楼层切换管理）
#         - 类型化任务执行器（m20_mission_executor：按系统配置依次
#           发布目标点驱动巡检）
#         - 可选验收测试节点（m20_phase4_acceptance）
#       支持系统配置、执行档案、速度反馈、能力档案等大量参数透传。
# ============================================================================

"""Launch phase 4: automatic flat multi-floor warehouse inspection."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作（如 run_acceptance 为 true 时启动验收节点）
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.parameter_descriptions import ParameterValue  # 把 launch 参数显式转换为指定 ROS2 参数类型


def generate_launch_description() -> LaunchDescription:
    """Compose phase-3 navigation with the typed mission executor."""
    # —— 路径准备 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = (  # 默认系统配置（平面多楼层）
        integration / 'config' / 'flat_multifloor_system.yaml'
    )
    # —— launch 参数（可被命令行覆盖，以下逐一说明）——
    system_config = LaunchConfiguration('system_config')  # 系统配置文件路径
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    use_planner = LaunchConfiguration('use_planner')  # 是否启动规划器
    use_local_sensing = LaunchConfiguration('use_local_sensing')  # 是否使用 SCAN 本地传感仿真
    clearance_config = LaunchConfiguration('clearance_config')  # 间隙配置（M20 几何覆盖）
    planner_config = LaunchConfiguration('planner_config')  # 规划器配置（速度型 M20 覆盖）
    controller_config = LaunchConfiguration('controller_config')  # 控制器配置（速度型 M20 覆盖）
    motion_backend = LaunchConfiguration('motion_backend')  # 运动后端（rviz 平面后端或外部后端）
    execution_profile = LaunchConfiguration('execution_profile')  # 执行档案（scan_native/m20_safe/m20_progress）
    body_pose_topic = LaunchConfiguration('body_pose_topic')  # 机器人位姿话题
    navigation_cloud_topic = LaunchConfiguration(  # 导航点云话题
        'navigation_cloud_topic'
    )
    sensor_pose_topic = LaunchConfiguration('sensor_pose_topic')  # 传感器位姿话题
    relocation_service = LaunchConfiguration('relocation_service')  # 重定位（传送）服务
    velocity_feedback_enabled = LaunchConfiguration(  # 速度反馈开关
        'velocity_feedback_enabled'
    )
    velocity_feedback_source = LaunchConfiguration(  # 速度反馈来源
        'velocity_feedback_source'
    )
    velocity_feedback_twist_topic = LaunchConfiguration(  # 实测 Twist 话题
        'velocity_feedback_twist_topic'
    )
    locomotion_capability_config = LaunchConfiguration(  # M20 平台能力档案
        'locomotion_capability_config'
    )
    navigation_timeout_sec = LaunchConfiguration(  # 单步导航超时（秒）
        'navigation_timeout_sec'
    )
    run_acceptance = LaunchConfiguration('run_acceptance')  # 是否运行验收测试
    acceptance_mode = LaunchConfiguration('acceptance_mode')  # 验收模式
    start_mission_executor = LaunchConfiguration('start_mission_executor')
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 系统配置文件
                'system_config',
                default_value=str(default_config),  # 默认平面多楼层系统配置（绝对路径）
                description='Absolute system profile YAML path.',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument('use_local_sensing', default_value='true'),  # 默认启用本地传感仿真
            DeclareLaunchArgument(  # 机器人位姿话题
                'body_pose_topic', default_value='/m20/sim/body_pose'  # 默认仿真位姿话题
            ),
            DeclareLaunchArgument(  # 导航点云话题
                'navigation_cloud_topic',
                default_value='/quad_0/cloud',  # 默认四足机器人点云话题
            ),
            DeclareLaunchArgument(  # 传感器位姿话题
                'sensor_pose_topic',
                default_value='/quad_0/lidar_pose',  # 默认雷达位姿话题
            ),
            DeclareLaunchArgument(  # 重定位（传送）服务
                'relocation_service',
                default_value='/m20/sim/set_pose',  # 默认仿真传送服务
            ),
            DeclareLaunchArgument(  # 间隙配置
                'clearance_config',
                default_value=str(  # 默认 M20 几何间隙覆盖
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'clearance_m20.yaml'
                ),
                description=(
                    'M20 geometry override for native SCAN planning and the '
                    'optional independent command guard.'
                ),
            ),
            DeclareLaunchArgument(  # 规划器配置
                'planner_config',
                default_value=str(  # 默认速度型 M20 规划器覆盖
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'scan_m20_velocity_planner.yaml'
                ),
                description=(
                    'Velocity-only M20 overlay layered after the upstream '
                    'SCAN planner profile.'
                ),
            ),
            DeclareLaunchArgument(  # 控制器配置
                'controller_config',
                default_value=str(  # 默认速度型 M20 控制器覆盖
                    Path(
                        get_package_share_directory(
                            'm20_scan_navigation'
                        )
                    )
                    / 'config'
                    / 'scan_m20_velocity_controller.yaml'
                ),
                description=(
                    'Velocity-only M20 overlay layered after the upstream '
                    'SCAN closed-loop controller profile.'
                ),
            ),
            DeclareLaunchArgument(  # 运动后端
                'motion_backend',
                default_value='rviz',  # 默认 RViz 平面后端
                description='rviz planar backend or an external backend.',
            ),
            DeclareLaunchArgument(  # 执行档案
                'execution_profile',
                default_value='scan_native',  # 默认上游兼容 SCAN 执行
                description=(
                    'Default upstream-compatible SCAN execution; use '
                    'm20_safe for the Twist guard/adapter or m20_progress '
                    'for measured-progress B-spline execution.'
                ),
            ),
            DeclareLaunchArgument(  # 速度反馈开关
                'velocity_feedback_enabled',
                default_value='false',  # 默认不启用
                description=(
                    'Enable bounded measured body-velocity PI compensation.'
                ),
            ),
            DeclareLaunchArgument(  # 速度反馈来源
                'velocity_feedback_source',
                default_value='odometry',  # 默认里程计来源
            ),
            DeclareLaunchArgument(  # 实测 Twist 话题
                'velocity_feedback_twist_topic',
                default_value='/m20/locomotion/measured_twist',  # 默认实测 Twist 话题
            ),
            DeclareLaunchArgument(  # M20 平台能力档案
                'locomotion_capability_config',
                default_value=str(  # 默认 v1 策略能力档案
                    Path(
                        get_package_share_directory(
                            'm20_locomotion_control'
                        )
                    )
                    / 'config'
                    / 'm20_policy_v1_capabilities.yaml'
                ),
                description='Versioned M20 platform capability profile.',
            ),
            DeclareLaunchArgument(  # 单步导航超时
                'navigation_timeout_sec',
                default_value='180.0',  # 默认 180 秒
                description='Per-navigation-step mission timeout.',
            ),
            DeclareLaunchArgument('run_acceptance', default_value='false'),  # 默认不运行验收
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),  # 默认验收模式 quick
            DeclareLaunchArgument(
                'start_mission_executor',
                default_value='true',
                description='Start the mission executor in this launch.',
            ),
            # —— 包含完整平面多楼层 SCAN 仿真图 ——
            IncludeLaunchDescription(  # 包含 multifloor_scan_rviz.launch.py（地图/TF/传感/导航/安全/楼层切换）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        integration
                        / 'launch'
                        / 'multifloor_scan_rviz.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'use_rviz': use_rviz,  # 透传：是否启动 RViz
                    'use_planner': use_planner,  # 透传：是否启动规划器
                    'use_local_sensing': use_local_sensing,  # 透传：本地传感开关
                    'body_pose_topic': body_pose_topic,  # 透传：位姿话题
                    'navigation_cloud_topic': navigation_cloud_topic,  # 透传：点云话题
                    'sensor_pose_topic': sensor_pose_topic,  # 透传：传感器位姿话题
                    'relocation_service': relocation_service,  # 透传：重定位服务
                    'clearance_config': clearance_config,  # 透传：间隙配置
                    'planner_config': planner_config,  # 透传：规划器配置
                    'controller_config': controller_config,  # 透传：控制器配置
                    'motion_backend': motion_backend,  # 透传：运动后端
                    'execution_profile': execution_profile,  # 透传：执行档案
                    'velocity_feedback_enabled': velocity_feedback_enabled,  # 透传：速度反馈开关
                    'velocity_feedback_source': velocity_feedback_source,  # 透传：速度反馈来源
                    'velocity_feedback_twist_topic': (  # 透传：实测 Twist 话题
                        velocity_feedback_twist_topic
                    ),
                    'locomotion_capability_config': (  # 透传：能力档案
                        locomotion_capability_config
                    ),
                    'system_config': system_config,  # 透传：系统配置
                }.items(),
            ),
            # —— 启动类型化任务执行器 ——
            Node(  # m20_mission_executor：按系统配置依次发布目标点驱动巡检
                package='m20_inspection_core',  # 所属功能包
                executable='m20_mission_executor',  # 可执行文件名
                name='m20_mission_executor',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'config_path': system_config,  # 系统配置（途经点/楼层顺序）
                        'navigation_timeout_sec': ParameterValue(  # 单步导航超时（显式浮点类型）
                            navigation_timeout_sec,
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(start_mission_executor),
            ),
            # —— 可选启动验收测试节点 ——
            Node(  # m20_phase4_acceptance：阶段 4 验收测试
                package='m20_warehouse_inspection',  # 所属功能包
                executable='m20_phase4_acceptance',  # 可执行文件名（本包脚本）
                name='m20_phase4_acceptance',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'mode': acceptance_mode,  # 验收模式
                        'timeout_sec': 900.0,  # 验收总超时 900 秒
                        'config_path': system_config,  # 系统配置
                    }
                ],
                condition=IfCondition(run_acceptance),  # 仅当 run_acceptance=true 时才启动
            ),
        ]
    )
