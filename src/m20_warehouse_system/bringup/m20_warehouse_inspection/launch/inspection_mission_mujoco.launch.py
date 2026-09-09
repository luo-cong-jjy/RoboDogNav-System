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
# 文件：inspection_mission_mujoco.launch.py
# 功能：以 MuJoCo 作为执行后端的完整 M20 仓库巡检任务 launch。
#       本 launch 组合：
#         - 完整的 RViz 巡检任务图（inspection_mission_rviz.launch.py，
#           motion_backend=external，即由 MuJoCo 提供位姿/关节/TF）
#         - MuJoCo 物理后端（mujoco_backend.launch.py，含官方 RL 控制
#           与 M20 物理/传感仿真）
#         - 官方运动 SDK（sdk_locomotion.launch.py）
#       支持执行档案（execution_profile：scan_native/m20_safe/m20_progress）、
#       MuJoCo 查看器参数、冷启动位姿覆盖与验收测试透传。
# ============================================================================

"""Complete M20 warehouse inspection with MuJoCo as execution backend."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 启动动作：声明 launch 参数、包含其他 launch 文件
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值


def generate_launch_description() -> LaunchDescription:
    """Compose SCAN, maps, tasks, official RL control, and M20 physics."""
    # —— 路径准备 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    mujoco_backend = Path(  # MuJoCo 后端包 share 目录
        get_package_share_directory('m20_mujoco_backend')
    )
    locomotion = Path(  # M20 运动控制包 share 目录
        get_package_share_directory('m20_locomotion_control')
    )
    default_config = (  # 默认系统配置：0.90 米高密度四角场景
        integration / 'config' / 'dense_four_corner_system.yaml'
    )
    # —— launch 参数（可被命令行覆盖）——
    system_config = LaunchConfiguration('system_config')  # 系统配置文件路径
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    use_planner = LaunchConfiguration('use_planner')  # 是否启动规划器
    execution_profile = LaunchConfiguration('execution_profile')  # 执行档案（scan_native/m20_safe/m20_progress）
    clearance_config = LaunchConfiguration('clearance_config')  # 间隙配置
    planner_config = LaunchConfiguration('planner_config')  # 规划器配置
    controller_config = LaunchConfiguration('controller_config')  # 控制器配置
    use_mujoco_viewer = LaunchConfiguration('use_mujoco_viewer')  # 是否打开 MuJoCo 3D 查看器
    mujoco_viewer_distance = LaunchConfiguration(  # 相机跟随距离（米）
        'mujoco_viewer_distance'
    )
    mujoco_viewer_azimuth = LaunchConfiguration(  # 相机初始方位角（度）
        'mujoco_viewer_azimuth'
    )
    mujoco_viewer_elevation = LaunchConfiguration(  # 相机初始俯仰角（度）
        'mujoco_viewer_elevation'
    )
    mujoco_viewer_max_fps = LaunchConfiguration(  # 查看器最大 FPS
        'mujoco_viewer_max_fps'
    )
    real_time_factor = LaunchConfiguration('real_time_factor')  # 实时因子
    initial_x = LaunchConfiguration('initial_x')  # 可选冷启动 X 覆盖
    initial_y = LaunchConfiguration('initial_y')  # 可选冷启动 Y 覆盖
    initial_yaw = LaunchConfiguration('initial_yaw')  # 可选冷启动偏航角覆盖
    velocity_feedback_enabled = LaunchConfiguration(  # 是否启用速度反馈（PI 补偿）
        'velocity_feedback_enabled'
    )
    locomotion_capability_config = LaunchConfiguration(  # M20 平台能力档案
        'locomotion_capability_config'
    )
    navigation_timeout_sec = LaunchConfiguration(  # 单步导航超时（秒）
        'navigation_timeout_sec'
    )
    run_acceptance = LaunchConfiguration('run_acceptance')  # 是否运行验收测试
    acceptance_mode = LaunchConfiguration('acceptance_mode')  # 验收模式
    model_xml_override = LaunchConfiguration('model_xml_override')
    world_source = LaunchConfiguration('world_source')
    world_file = LaunchConfiguration('world_file')

    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 系统配置文件
                'system_config',
                default_value=str(default_config),  # 默认 0.90 米高密度完整系统场景
                description=(
                    'Warehouse profile. The 0.90 m dense profile is the '
                    'default complete-system scene.'
                ),
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('model_xml_override', default_value=''),
            DeclareLaunchArgument('world_source', default_value='warehouse'),
            DeclareLaunchArgument('world_file', default_value=''),
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument(  # 执行档案
                'execution_profile',
                default_value='m20_safe',  # 规划基线叠加 M20 安全运动适配
                description=(
                    'Default upstream-compatible SCAN command path; '
                    'm20_safe restores the Twist adapter/guard chain; '
                    'm20_progress enables measured-progress B-spline '
                    'execution for an explicit A/B run.'
                ),
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
            DeclareLaunchArgument(  # 是否打开 MuJoCo 查看器
                'use_mujoco_viewer',
                default_value='true',  # 默认打开原生 3D 查看器
                description=(
                    'Open the native MuJoCo 3D viewer by default; set false '
                    'for headless runs.'
                ),
            ),
            DeclareLaunchArgument(  # 查看器相机距离
                'mujoco_viewer_distance',
                default_value='4.0',  # 默认 4 米
                description=(
                    'MuJoCo direct-follow camera distance in metres.'
                ),
            ),
            DeclareLaunchArgument(  # 查看器相机方位角
                'mujoco_viewer_azimuth',
                default_value='90.0',  # 默认 90 度
                description='Initial MuJoCo camera azimuth in degrees.',
            ),
            DeclareLaunchArgument(  # 查看器相机俯仰角
                'mujoco_viewer_elevation',
                default_value='-89.0',  # 默认接近垂直俯视
                description='Near-vertical MuJoCo camera elevation.',
            ),
            DeclareLaunchArgument(  # 查看器最大 FPS
                'mujoco_viewer_max_fps',
                default_value='30.0',  # 默认 30 FPS
                description='Maximum wall-clock MuJoCo viewer FPS.',
            ),
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),  # 默认实时因子 1.0
            DeclareLaunchArgument(  # 可选冷启动 X 覆盖
                'initial_x',
                default_value='',  # 默认空：使用配置中的初始位姿
                description='Optional simulation-only cold-start x override.',
            ),
            DeclareLaunchArgument(  # 可选冷启动 Y 覆盖
                'initial_y',
                default_value='',  # 默认空
                description='Optional simulation-only cold-start y override.',
            ),
            DeclareLaunchArgument(  # 可选冷启动偏航角覆盖
                'initial_yaw',
                default_value='',  # 默认空
                description=(
                    'Optional simulation-only cold-start yaw override.'
                ),
            ),
            DeclareLaunchArgument(  # 速度反馈开关
                'velocity_feedback_enabled',
                default_value='false',  # 默认不启用（A/B 对照）
                description=(
                    'Enable bounded measured body-velocity PI compensation '
                    'for MuJoCo A/B validation.'
                ),
            ),
            DeclareLaunchArgument(  # M20 平台能力档案
                'locomotion_capability_config',
                default_value=str(  # 仿真使用支持原地四足转向的平层能力档案
                    locomotion
                    / 'config'
                    / 'm20_factory_agile_flat_capabilities.yaml'
                ),
                description='Versioned M20 platform capability profile.',
            ),
            DeclareLaunchArgument(  # 单步导航超时
                'navigation_timeout_sec',
                default_value='300.0',  # 默认 300 秒（物理运动比运动学慢）
                description=(
                    'Per-step timeout for physical MuJoCo motion; longer '
                    'than the RViz kinematic profile.'
                ),
            ),
            DeclareLaunchArgument('run_acceptance', default_value='false'),  # 默认不运行验收
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),  # 默认验收模式 quick
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
                    'model_xml_override': model_xml_override,
                    'world_source': world_source,
                    'world_file': world_file,
                    'use_rviz': use_rviz,  # 透传：是否启动 RViz
                    'use_planner': use_planner,  # 透传：是否启动规划器
                    'clearance_config': clearance_config,  # 透传：间隙配置
                    'planner_config': planner_config,  # 透传：规划器配置
                    'controller_config': controller_config,  # 透传：控制器配置
                    'motion_backend': 'external',  # 运动后端为外部（MuJoCo 提供位姿/关节/TF）
                    'execution_profile': execution_profile,  # 透传：执行档案
                    'velocity_feedback_enabled': velocity_feedback_enabled,  # 透传：速度反馈开关
                    'locomotion_capability_config': (  # 透传：能力档案
                        locomotion_capability_config
                    ),
                    'navigation_timeout_sec': navigation_timeout_sec,  # 透传：单步超时
                    'run_acceptance': run_acceptance,  # 透传：是否运行验收
                    'acceptance_mode': acceptance_mode,  # 透传：验收模式
                    'start_mission_executor': 'false',
                }.items(),
            ),
            # —— 包含 MuJoCo 物理后端 ——
            IncludeLaunchDescription(  # 包含 MuJoCo 后端 launch（官方 RL 控制 + M20 物理/传感）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        mujoco_backend
                        / 'launch'
                        / 'mujoco_backend.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'system_config': system_config,  # 系统配置
                    'package_root': str(integration),  # 包根目录
                    'use_viewer': use_mujoco_viewer,  # 是否打开查看器
                    'viewer_distance': mujoco_viewer_distance,  # 相机距离
                    'viewer_azimuth': mujoco_viewer_azimuth,  # 相机方位角
                    'viewer_elevation': mujoco_viewer_elevation,  # 相机俯仰角
                    'viewer_max_fps': mujoco_viewer_max_fps,  # 相机最大 FPS
                    'real_time_factor': real_time_factor,  # 实时因子
                    'initial_x': initial_x,  # 冷启动 X 覆盖
                    'initial_y': initial_y,  # 冷启动 Y 覆盖
                    'initial_yaw': initial_yaw,  # 冷启动偏航角覆盖
                }.items(),
            ),
            # —— 包含官方运动 SDK ——
            IncludeLaunchDescription(  # 包含运动 SDK launch（官方 RL 控制下发）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        locomotion
                        / 'launch'
                        / 'sdk_locomotion.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'start_sdk': 'true',  # 启动 SDK
                    'require_backend_ready': 'true',  # 要求后端就绪后才开始
                    'locomotion_capability_config': (  # 能力档案
                        locomotion_capability_config
                    ),
                    # MuJoCo has no hardware gait safety envelope, so use a
                    # quicker command ramp for responsive physical tracking.
                    'output_linear_accel': '1.8',
                    'output_yaw_accel': '2.2',
                    'execution_profile': execution_profile,  # 执行档案
                }.items(),
            ),
        ]
    )
