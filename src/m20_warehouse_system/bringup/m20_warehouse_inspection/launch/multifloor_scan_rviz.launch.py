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
# 文件：multifloor_scan_rviz.launch.py
# 功能：阶段 3（phase-3）双向平面多楼层 SCAN 仿真 launch。
#       本 launch 通过 OpaqueFunction 在运行期解析系统配置后，构建：
#         - 平面多楼层地图服务器（m20_flat_map_server）
#         - M20 官方模型 TF（robot_state_publisher）与 map→world 静态变换
#         - RViz 运动学后端（motion_backend=rviz 时）或外部后端
#         - 传感状态节点（m20_vendor_sensing_state）
#         - 导航适配器（m20_safe 档案时）、轨迹进度跟踪器（m20_progress 时）
#         - 安全监督（m20_safety_supervisor）与可选碰撞防护（m20_collision_guard）
#         - SCAN 导航栈（f1_scan.launch.py）
#         - 楼层切换管理器（m20_floor_switch_manager）
#         - 可选 RViz
#       支持执行档案（scan_native/m20_safe/m20_progress）与速度反馈等参数。
# ============================================================================

"""Launch phase 3: bidirectional flat multi-floor SCAN simulation."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

import yaml  # YAML 解析库：读取系统配置中的楼层初始位姿

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import (  # 启动动作
    DeclareLaunchArgument,  # 声明 launch 参数
    IncludeLaunchDescription,  # 包含其他 launch 文件
    OpaqueFunction,  # 不透明函数动作：运行期执行回调以动态构建动作列表
)
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration, PythonExpression  # 参数替换、Python 表达式（用于字符串比较）
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.parameter_descriptions import ParameterValue  # 把 launch 参数显式转换为指定 ROS2 参数类型

from m20_locomotion_control.capability_profile import (  # M20 能力档案加载器（提供命令/控制/安全参数）
    load_capability_profile,
)


def _runtime_actions(context):
    """Build nodes after resolving the selected system configuration."""
    # —— 路径准备：解析各依赖包的 share 目录 ——
    integration = Path(  # 本包 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    description = Path(  # M20 官方描述包 share 目录（URDF 模型）
        get_package_share_directory('m20_official_description')
    )
    scan = Path(get_package_share_directory('m20_scan_navigation'))  # SCAN 导航包 share 目录
    simulation = Path(get_package_share_directory('m20_warehouse_sim'))  # 仓库仿真包 share 目录
    core = Path(get_package_share_directory('m20_inspection_core'))  # 巡检核心包 share 目录
    locomotion = Path(  # M20 运动控制包 share 目录
        get_package_share_directory('m20_locomotion_control')
    )
    scan_vendor = Path(get_package_share_directory('m20_scan_planner'))  # SCAN 规划器（vendor）包 share 目录

    system_config = Path(  # 解析系统配置参数为绝对路径
        LaunchConfiguration('system_config').perform(context)
    ).expanduser().resolve()
    with system_config.open('r', encoding='utf-8') as stream:
        initial_pose = yaml.safe_load(stream)['floors']['F1']['initial_pose']  # 读取 F1 楼层初始位姿 [x, y, yaw]
    model = description / 'urdf' / 'm20_official.urdf'  # M20 官方 URDF 模型
    # Load the exact RViz profile installed by the vendored SCAN package.
    # （加载 vendor 版 SCAN 包安装的官方 RViz 配置）
    rviz = scan_vendor / 'rviz' / 'default.rviz'
    # —— launch 参数（可被命令行覆盖）——
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    use_planner = LaunchConfiguration('use_planner')  # 是否启动规划器
    use_local_sensing = LaunchConfiguration('use_local_sensing')  # 是否使用本地传感仿真
    clearance_config = LaunchConfiguration('clearance_config')  # 间隙配置
    controller_config = LaunchConfiguration('controller_config')  # 控制器配置
    motion_backend = LaunchConfiguration('motion_backend')  # 运动后端
    execution_profile = LaunchConfiguration(  # 执行档案（运行期解析为字符串）
        'execution_profile'
    ).perform(context)
    if execution_profile not in {  # 校验执行档案取值
        'scan_native',
        'm20_safe',
        'm20_progress',
    }:
        raise RuntimeError(  # 非法取值直接报错
            'execution_profile must be scan_native, m20_safe, or '
            'm20_progress'
        )
    scan_native = execution_profile == 'scan_native'  # 上游原生 SCAN 命令路径
    m20_safe = execution_profile == 'm20_safe'  # Twist 适配器/防护链
    m20_progress = execution_profile == 'm20_progress'  # 实测进度 B 样条执行
    velocity_feedback_enabled = LaunchConfiguration(  # 速度反馈开关
        'velocity_feedback_enabled'
    )
    velocity_feedback_source = LaunchConfiguration(  # 速度反馈来源
        'velocity_feedback_source'
    )
    velocity_feedback_twist_topic = LaunchConfiguration(  # 实测 Twist 话题
        'velocity_feedback_twist_topic'
    )
    body_pose_topic = LaunchConfiguration('body_pose_topic')  # 机器人位姿话题
    navigation_cloud_topic = LaunchConfiguration(  # 导航点云话题
        'navigation_cloud_topic'
    )
    sensor_pose_topic = LaunchConfiguration('sensor_pose_topic')  # 传感器位姿话题
    relocation_service = LaunchConfiguration('relocation_service')  # 重定位（传送）服务
    planner_config = LaunchConfiguration('planner_config')  # 规划器配置
    capability_profile = load_capability_profile(  # 加载 M20 平台能力档案
        LaunchConfiguration(
            'locomotion_capability_config'
        ).perform(context)
    )
    intent_parameters = capability_profile.intent_parameters()  # 运动意图参数
    controller_parameters = capability_profile.controller_parameters()  # 控制器参数
    guard_parameters = capability_profile.collision_guard_parameters()  # 碰撞防护参数
    safety_parameters = capability_profile.safety_parameters()  # 安全参数
    native_safety_parameters = (  # 原生 SCAN 档案下的安全监督参数
        {
            # Preserve SCAN's upstream command semantics while applying the
            # validated M20 numeric envelope loaded above.
            # This supervisor is only the indispensable multi-floor/e-stop
            # gate in the native profile; it performs no obstacle veto.
            # （保留 SCAN 上游命令语义，同时应用上面加载的经校验 M20 数值包络；
            #   该监督器在原生档案下只承担多楼层/急停门控，不做障碍否决）
            'navigation_topic': '/m20/navigation/cmd_vel_raw',  # 导航命令话题（原生路径）
            'collision_guard_enabled': False,  # 原生档案关闭独立碰撞防护
            'max_linear_x': capability_profile.max_forward,  # 最大前进速度（来自能力档案）
            'max_linear_y': capability_profile.max_side,  # 最大侧向速度（来自能力档案）
            'max_angular_z': capability_profile.max_yaw,  # 最大偏航角速度（来自能力档案）
            'max_linear_accel': 50.0,  # 最大线加速度（宽松限值）
            'max_angular_accel': 50.0,  # 最大角加速度（宽松限值）
        }
        if scan_native  # 仅原生档案使用
        else {}
    )

    return [
            # —— 启动平面多楼层地图服务器 ——
            Node(  # m20_flat_map_server：加载多楼层地图并发布当前楼层
                package='m20_warehouse_inspection',  # 所属功能包
                executable='m20_flat_map_server',  # 可执行文件名（本包脚本）
                name='flat_multifloor_map_server',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'config_path': str(system_config),  # 系统配置（楼层/地图偏移）
                        'package_root': str(integration),  # 包根目录
                        'initial_floor': 'F1',  # 初始楼层 F1
                        'republish_period_sec': 0.0,  # 重发布周期 0：只发布一次
                        'switch_commit_delay_sec': 0.05,  # 楼层切换提交延迟 0.05 秒
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
                        # Keep the visible robot byte-for-byte equivalent in
                        # structure to the official M20 URDF.
                        # （保持可见机器人与官方 M20 URDF 结构逐字节一致）
                        'robot_description': model.read_text(encoding='utf-8')  # 读取官方 URDF 内容
                    }
                ],
            ),
            # —— 发布 map→world 静态变换 ——
            Node(  # 发布 map→world 静态变换（单位变换，共原点）
                package='tf2_ros',  # 所属功能包
                executable='static_transform_publisher',  # 静态变换发布可执行文件
                name='m20_map_to_scan_world',  # 节点名
                # Foxy only accepts the legacy positional CLI.  Humble keeps
                # this form for compatibility, so use one launch contract on
                # both the development and M20-PRO target systems.
                # （Foxy 只接受旧式位置参数 CLI；Humble 为兼容保留该形式，
                #   因此在开发机与 M20-PRO 目标机上使用同一 launch 契约）
                arguments=[
                    '0', '0', '0', '0', '0', '0',  # 平移/旋转均为 0
                    'map', 'world',  # map → world
                ],
                output='screen',  # 日志输出到屏幕
            ),
            # —— 可选启动 RViz 运动学后端 ——
            Node(  # m20_rviz_kinematic_backend：RViz 平面运动学仿真后端
                package='m20_warehouse_sim',  # 所属功能包
                executable='m20_rviz_kinematic_backend',  # 可执行文件名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(simulation / 'config' / 'rviz_scan_vendor.yaml'),  # 运动学后端配置
                    {
                        # The system configuration is the single source of
                        # truth when map offsets or initial poses change.
                        # （地图偏移或初始位姿变化时，系统配置是唯一事实来源）
                        'initial_x': float(initial_pose[0]),  # 初始 X（来自配置）
                        'initial_y': float(initial_pose[1]),  # 初始 Y（来自配置）
                        'initial_yaw': float(initial_pose[2]),  # 初始偏航角（来自配置）
                    },
                ],
                condition=IfCondition(  # 仅当 motion_backend=rviz 时启动
                    PythonExpression(
                        ["'", motion_backend, "' == 'rviz'"]
                    )
                ),
            ),
            # —— 启动传感状态节点 ——
            Node(  # m20_vendor_sensing_state：发布 vendor 传感状态（点云等）
                package='m20_multifloor_map',  # 所属功能包
                executable='m20_vendor_sensing_state',  # 可执行文件名
                output='screen',  # 日志输出到屏幕
                parameters=[{'cloud_topic': navigation_cloud_topic}],  # 点云发布话题
            ),
            # —— 可选启动导航适配器（m20_safe 档案）——
            Node(  # m20_navigation_adapter：导航命令 → M20 运动命令适配器
                package='m20_locomotion_control',  # 所属功能包
                executable='m20_navigation_adapter',  # 可执行文件名
                name='m20_navigation_adapter',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(locomotion / 'config' / 'sdk_locomotion.yaml'),  # 运动 SDK 配置
                    intent_parameters,  # 运动意图参数（来自能力档案）
                    {
                        'velocity_feedback_enabled': ParameterValue(  # 速度反馈开关（显式布尔类型）
                            velocity_feedback_enabled,
                            value_type=bool,
                        ),
                        'velocity_feedback_odometry_topic': body_pose_topic,  # 速度反馈里程计话题
                        'velocity_feedback_source': (  # 速度反馈来源
                            velocity_feedback_source
                        ),
                        'velocity_feedback_twist_topic': (  # 实测 Twist 话题
                            velocity_feedback_twist_topic
                        ),
                    },
                ],
                condition=IfCondition(str(m20_safe).lower()),  # 仅 m20_safe 档案时启动
            ),
            # —— 可选启动轨迹进度跟踪器（m20_progress 档案）——
            Node(  # m20_trajectory_progress_tracker：按实测空间进度跟踪 B 样条轨迹
                package='m20_locomotion_control',  # 所属功能包
                executable='m20_trajectory_progress_tracker',  # 可执行文件名
                name='m20_trajectory_progress_tracker',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(locomotion / 'config' / 'sdk_locomotion.yaml'),  # 运动 SDK 配置
                    intent_parameters,  # 运动意图参数
                    {
                        'odometry_topic': body_pose_topic,  # 里程计话题
                        'reverse_tracking_enabled': (  # 反向跟踪使能（来自控制器参数）
                            controller_parameters[
                                'bidirectional_tracking_enabled'
                            ]
                        ),
                        'reverse_tracking_enter_angle': (  # 反向跟踪进入角（来自控制器参数）
                            controller_parameters[
                                'reverse_tracking_enter_angle'
                            ]
                        ),
                        'reverse_tracking_exit_angle': (  # 反向跟踪退出角（来自控制器参数）
                            controller_parameters[
                                'reverse_tracking_exit_angle'
                            ]
                        ),
                    },
                ],
                condition=IfCondition(str(m20_progress).lower()),  # 仅 m20_progress 档案时启动
            ),
            # —— 启动安全监督节点 ——
            Node(  # m20_safety_supervisor：安全监督（多楼层/急停门控）
                package='m20_inspection_core',  # 所属功能包
                executable='m20_safety_supervisor',  # 可执行文件名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(core / 'config' / 'safety_scan_vendor.yaml'),  # 安全配置
                    safety_parameters,  # 安全参数（来自能力档案）
                    native_safety_parameters,  # 原生档案安全参数（非原生时为空字典）
                    {'odom_topic': body_pose_topic},  # 里程计话题
                ],
            ),
            # —— 可选启动碰撞防护节点（非原生档案时）——
            Node(  # m20_collision_guard：独立碰撞防护
                package='m20_inspection_core',  # 所属功能包
                executable='m20_collision_guard',  # 可执行文件名
                name='m20_collision_guard',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(core / 'config' / 'collision_guard_scan_native.yaml'),  # 碰撞防护配置
                    clearance_config,  # 间隙配置（M20 几何）
                    guard_parameters,  # 防护参数（来自能力档案）
                    {'odom_topic': body_pose_topic},  # 里程计话题
                ],
                condition=IfCondition(str(not scan_native).lower()),  # 非原生档案时才启动
            ),
            # —— 包含 SCAN 导航栈 ——
            IncludeLaunchDescription(  # 包含 m20_scan_navigation 的 f1_scan.launch.py
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(scan / 'launch' / 'f1_scan.launch.py')
                ),
                launch_arguments={
                    # This is SCAN's native CPU ray-casting renderer. A small
                    # opt-in reload seam lets it rebuild after a floor switch.
                    # （SCAN 原生 CPU 光线投射渲染器；内置小型可选重载缝隙，
                    #   使其在楼层切换后可重建渲染）
                    'use_local_sensing': use_local_sensing,  # 本地传感开关
                    'use_planner': use_planner,  # 规划器开关
                    'body_pose_topic': body_pose_topic,  # 位姿话题
                    'navigation_cloud_topic': navigation_cloud_topic,  # 点云话题
                    'sensor_pose_topic': sensor_pose_topic,  # 传感器位姿话题
                    'clearance_config': clearance_config,  # 间隙配置
                    'planner_config': planner_config,  # 规划器配置
                    'controller_config': controller_config,  # 控制器配置
                    'require_external_execution_hold': str(  # 是否要求外部执行保持（非原生档案）
                        not scan_native
                    ).lower(),
                    'collision_guard_required': str(  # 是否要求碰撞防护（非原生档案）
                        not scan_native
                    ).lower(),
                    'bidirectional_tracking_enabled': str(  # 双向跟踪使能
                        False  # 原生档案强制关闭，否则用控制器参数
                        if scan_native
                        else controller_parameters[
                            'bidirectional_tracking_enabled'
                        ]
                    ).lower(),
                    'reverse_tracking_enter_angle': str(  # 反向跟踪进入角
                        controller_parameters[
                            'reverse_tracking_enter_angle'
                        ]
                    ),
                    'reverse_tracking_exit_angle': str(  # 反向跟踪退出角
                        controller_parameters[
                            'reverse_tracking_exit_angle'
                        ]
                    ),
                    'reverse_tracking_min_hold_sec': str(  # 反向跟踪最小保持时间
                        controller_parameters[
                            'reverse_tracking_min_hold_sec'
                        ]
                    ),
                    'reverse_tracking_entry_alignment': str(  # 反向跟踪入口对齐
                        controller_parameters[
                            'reverse_tracking_entry_alignment'
                        ]
                    ),
                    'reverse_tracking_exit_alignment': str(  # 反向跟踪出口对齐
                        controller_parameters[
                            'reverse_tracking_exit_alignment'
                        ]
                    ),
                }.items(),
            ),
            # —— 启动楼层切换管理器 ——
            Node(  # m20_floor_switch_manager：管理楼层切换与机器人传送
                package='m20_inspection_core',  # 所属功能包
                executable='m20_floor_switch_manager',  # 可执行文件名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    {
                        'config_path': str(system_config),  # 系统配置（楼层定义）
                        'body_pose_topic': body_pose_topic,  # 位姿话题
                        'set_pose_service': relocation_service,  # 传送/重定位服务
                    }
                ],
            ),
            # —— 可选启动 RViz ——
            Node(  # 启动 RViz（使用 vendor 的默认配置）
                package='rviz2',  # 所属功能包
                executable='rviz2',  # 可执行文件名
                name='m20_phase3_multifloor_rviz',  # 节点名
                arguments=['-d', str(rviz)],  # 加载指定 RViz 配置文件
                output='screen',  # 日志输出到屏幕
                condition=IfCondition(use_rviz),  # 仅当 use_rviz=true 时才启动
            ),
        ]


def generate_launch_description() -> LaunchDescription:
    """Connect the selected profile to the atomic floor-switch graph."""
    # —— 路径准备：解析本包 share 目录并拼接默认系统配置 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = (  # 默认系统配置（平面多楼层）
        integration / 'config' / 'flat_multifloor_system.yaml'
    )
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 系统配置文件
                'system_config',
                default_value=str(default_config),  # 默认平面多楼层配置（绝对路径）
                description='Absolute system profile YAML path.',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument(  # 本地传感开关
                'use_local_sensing',
                default_value='true',  # 默认仿真中使用 SCAN PCD 光线投射
                description=(
                    'Use SCAN PCD ray casting in simulation; hardware '
                    'launches disable it and provide a live cloud.'
                ),
            ),
            DeclareLaunchArgument(  # 机器人位姿话题
                'body_pose_topic',
                default_value='/m20/sim/body_pose',  # 默认仿真位姿话题
            ),
            DeclareLaunchArgument(  # 导航点云话题
                'navigation_cloud_topic',
                default_value='/quad_0/cloud',  # 默认点云话题
            ),
            DeclareLaunchArgument(  # 传感器位姿话题
                'sensor_pose_topic',
                default_value='/quad_0/lidar_pose',  # 默认雷达位姿话题
            ),
            DeclareLaunchArgument(  # 重定位（传送）服务
                'relocation_service',
                default_value='/m20/sim/set_pose',  # 默认仿真传送服务
                description=(
                    'Simulation teleport service or a real localization '
                    'reinitialization service with the same contract.'
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
                    'M20 body-geometry override layered on the otherwise '
                    'native SCAN planner. Use clearance_vendor.yaml for an '
                    'exact upstream geometry A/B run.'
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
                description=(
                    'rviz starts the planar backend; external reserves the '
                    'pose, joint-state and TF contract for MuJoCo or hardware.'
                ),
            ),
            DeclareLaunchArgument(  # 执行档案
                'execution_profile',
                default_value='scan_native',  # 默认上游原生 SCAN 命令路径
                description=(
                    'scan_native preserves the upstream SCAN command path; '
                    'm20_safe enables the experimental rolling adapter, '
                    'footprint guard and trajectory hold chain; m20_progress '
                    'tracks the unchanged B-spline by measured M20 spatial '
                    'progress before the same guard.'
                ),
            ),
            DeclareLaunchArgument(  # 速度反馈开关
                'velocity_feedback_enabled',
                default_value='false',  # 默认不启用
                description=(
                    'Enable bounded measured body-velocity PI compensation '
                    'before collision prediction.'
                ),
            ),
            DeclareLaunchArgument(  # 速度反馈来源
                'velocity_feedback_source',
                default_value='odometry',  # 默认里程计来源
                description='Measured velocity source: odometry or twist.',
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
                description=(
                    'Versioned M20 command, turn and recovery capability '
                    'profile shared by motion adaptation and collision guard.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),  # 运行期执行配置解析并动态构建节点图
        ]
    )
