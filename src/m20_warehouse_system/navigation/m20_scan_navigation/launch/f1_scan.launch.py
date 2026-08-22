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

# =============================================================================
# f1_scan.launch.py —— F1 场景 SCAN 导航 launch 文件
# 所属模块：m20_scan_navigation / launch
# 职责：围绕 M20 接口启动"引入的 SCAN"第一场景话题图：
#   1. local_sensing_node/pcl_render_node：把全局地图渲染为局部感知点云；
#   2. m20_scan_planner/scan_planner_node：SCAN 规划器（算法即上游源码镜像）；
#   3. m20_scan_planner/closed_loop_controller：闭环轨迹跟踪控制器；
#   4. m20_scan_navigation/m20_navigation_gateway：类型化导航网关。
# 支持两个 M20 集成边界重映射：机身位姿话题（body_pose）与 SCAN 复位服务
# （reset -> /m20/navigation/reset）。
# =============================================================================

"""Launch the vendored SCAN first-scene graph around the M20 interfaces."""

from pathlib import Path      # 跨平台路径处理

from ament_index_python.packages import get_package_share_directory   # 获取包安装目录
from launch import LaunchDescription                                  # launch 描述对象
from launch.actions import DeclareLaunchArgument                      # 声明 launch 参数
from launch.conditions import IfCondition                             # 条件（是否启用节点）
from launch.substitutions import LaunchConfiguration                  # 读取 launch 参数值
from launch_ros.actions import Node                                   # ROS2 节点动作
from launch_ros.parameter_descriptions import ParameterValue           # 带类型的参数值


def generate_launch_description() -> LaunchDescription:
    """创建 SCAN 的规范话题图，并带两个 M20 边界重映射。"""
    # 本包安装后的共享目录（config/ 等资源所在处）
    share = Path(get_package_share_directory('m20_scan_navigation'))
    # ---- 声明并读取全部 launch 参数 ----
    use_local_sensing = LaunchConfiguration('use_local_sensing')      # 是否启动局部感知
    use_planner = LaunchConfiguration('use_planner')                  # 是否启动规划器+控制器
    body_pose_topic = LaunchConfiguration('body_pose_topic')          # 机身位姿话题
    navigation_cloud_topic = LaunchConfiguration(
        'navigation_cloud_topic'                                      # SCAN 输入点云话题
    )
    sensor_pose_topic = LaunchConfiguration('sensor_pose_topic')      # 传感器位姿话题
    clearance_config = LaunchConfiguration('clearance_config')        # 净空参数覆盖文件
    planner_config = LaunchConfiguration('planner_config')            # 规划器参数覆盖文件
    controller_config = LaunchConfiguration('controller_config')      # 控制器参数覆盖文件
    require_external_execution_hold = LaunchConfiguration(
        'require_external_execution_hold'                             # 是否要求外部执行保持
    )
    collision_guard_required = LaunchConfiguration(
        'collision_guard_required'                                    # 是否要求碰撞守卫
    )
    bidirectional_tracking_enabled = LaunchConfiguration(
        'bidirectional_tracking_enabled'                              # 是否启用双向跟踪
    )
    reverse_tracking_enter_angle = LaunchConfiguration(
        'reverse_tracking_enter_angle'                                # 进入倒车跟踪的航向差角
    )
    reverse_tracking_exit_angle = LaunchConfiguration(
        'reverse_tracking_exit_angle'                                 # 退出倒车跟踪的航向差角
    )
    reverse_tracking_min_hold_sec = LaunchConfiguration(
        'reverse_tracking_min_hold_sec'                               # 倒车跟踪最短保持时间
    )
    reverse_tracking_entry_alignment = LaunchConfiguration(
        'reverse_tracking_entry_alignment'                            # 进入倒车的对齐阈值
    )
    reverse_tracking_exit_alignment = LaunchConfiguration(
        'reverse_tracking_exit_alignment'                             # 退出倒车的对齐阈值
    )

    # ---- 节点 1：局部感知（点云渲染），use_local_sensing=true 时启动 ----
    local_sensing = Node(
        package='local_sensing_node',       # 感知包
        executable='pcl_render_node',       # 点云渲染可执行
        name='pcl_render_node',
        output='screen',                    # 日志输出到屏幕
        parameters=[
            str(share / 'config' / 'scan_vendor_local_sensing.yaml'),  # 厂商感知参数
            {'body_pose_topic': body_pose_topic},                      # 覆盖机身位姿话题
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),  # 全局地图输入重映射
            ('cloud', navigation_cloud_topic),              # 输出点云 -> SCAN 输入
            ('sensor_cloud', '/quad_0/sensor_cloud'),
            ('dyn_cloud', '/quad_0/dyn_cloud'),
            ('uav_cloud', '/quad_0/uav_cloud'),
        ],
        condition=IfCondition(use_local_sensing),   # 按参数决定是否启动
    )
    # ---- 节点 2：SCAN 规划器，use_planner=true 时启动 ----
    planner = Node(
        # The algorithm implementation is the direct SCAN source mirror.
        # Only body odometry and reset cross the M20 integration boundary.
        # （算法实现是 SCAN 源码的直接镜像；只有机身里程计与复位跨越 M20
        #   集成边界。）
        package='m20_scan_planner',         # SCAN 规划器包（算法镜像）
        executable='scan_planner_node',     # 规划器节点
        name='scan_planner_node',
        output='screen',
        parameters=[
            # 基准：厂商规划器参数；之后叠加可选覆盖与净空覆盖
            str(share / 'config' / 'scan_vendor_planner.yaml'),
            planner_config,
            clearance_config,
        ],
        remappings=[
            ('body_pose', body_pose_topic),         # 机身位姿输入重映射
            ('sensor_pose', sensor_pose_topic),     # 传感器位姿重映射
            ('cloud', navigation_cloud_topic),      # 点云输入重映射
            ('reset', '/m20/navigation/reset'),     # 复位服务 -> M20 集成边界
            ('move_base_simple/goal', '/move_base_simple/goal'),  # 目标话题保持公共名
        ],
        condition=IfCondition(use_planner),
    )
    # ---- 节点 3：闭环控制器，use_planner=true 时启动 ----
    controller = Node(
        package='m20_scan_planner',         # 控制器与规划器同包
        executable='closed_loop_controller',# 闭环跟踪控制器
        name='closed_loop_controller',
        output='screen',
        parameters=[
            # 基准：厂商控制器参数；之后叠加可选覆盖与 M20 执行参数
            str(share / 'config' / 'scan_vendor_controller.yaml'),
            controller_config,
            {
                # 将各 launch 参数以正确类型注入控制器参数
                'require_external_execution_hold': ParameterValue(
                    require_external_execution_hold,
                    value_type=bool,
                ),
                'bidirectional_tracking_enabled': ParameterValue(
                    bidirectional_tracking_enabled,
                    value_type=bool,
                ),
                'reverse_tracking_enter_angle': ParameterValue(
                    reverse_tracking_enter_angle,
                    value_type=float,
                ),
                'reverse_tracking_exit_angle': ParameterValue(
                    reverse_tracking_exit_angle,
                    value_type=float,
                ),
                'reverse_tracking_min_hold_sec': ParameterValue(
                    reverse_tracking_min_hold_sec,
                    value_type=float,
                ),
                'reverse_tracking_entry_alignment': ParameterValue(
                    reverse_tracking_entry_alignment,
                    value_type=float,
                ),
                'reverse_tracking_exit_alignment': ParameterValue(
                    reverse_tracking_exit_alignment,
                    value_type=float,
                ),
            },
        ],
        remappings=[
            ('body_pose', body_pose_topic),                 # 机身位姿重映射
            ('cmd_vel', '/m20/navigation/cmd_vel_raw'),     # 速度输出 -> 原始指令话题
        ],
        condition=IfCondition(use_planner),
    )
    # ---- 节点 4：导航网关（类型化 action + 受管 RViz 目标）----
    navigation_gateway = Node(
        package='m20_scan_navigation',      # 本包
        executable='m20_navigation_gateway',# 网关可执行（入口脚本）
        name='m20_navigation_gateway',
        output='screen',
        parameters=[
            str(share / 'config' / 'navigation_gateway.yaml'),  # 网关参数文件
            {
                # 集成 launch 中关闭手点监控：RViz 目标直接走公共话题
                'manual_goal_monitor_enabled': False,
                'direct_goal_topic': '/move_base_simple/goal',  # 目标直发话题
                'odom_topic': body_pose_topic,                  # 里程计用机身位姿
                'collision_guard_required': ParameterValue(
                    collision_guard_required,
                    value_type=bool,
                ),
            },
        ],
        condition=IfCondition(use_planner),
    )
    # ---- 组装 launch 描述：声明参数 + 各节点 ----
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_local_sensing', default_value='true'),  # 是否启动局部感知
            DeclareLaunchArgument(
                'body_pose_topic',
                default_value='/m20/sim/body_pose',
                description=(
                    'Odometry source shared by SCAN, safety, and hardware '
                    'localization adapters.'
                    # 机身位姿/里程计来源：SCAN、安全、硬件定位适配器共享
                ),
            ),
            DeclareLaunchArgument(
                'navigation_cloud_topic',
                default_value='/quad_0/cloud',
                description='Map/world-frame cloud consumed by SCAN.',  # SCAN 消费的世界系点云
            ),
            DeclareLaunchArgument(
                'sensor_pose_topic',
                default_value='/quad_0/lidar_pose',
                description='Odometry-form sensor pose consumed by SCAN.',  # SCAN 消费的里程计形式传感器位姿
            ),
            DeclareLaunchArgument('use_planner', default_value='true'),  # 是否启动规划器/控制器
            DeclareLaunchArgument(
                'clearance_config',
                default_value=str(
                    share / 'config' / 'clearance_vendor.yaml'
                ),
                description=(
                    'ROS parameter override for native SCAN and the '
                    'independent online-point-cloud collision guard.'
                    # 原生 SCAN 与独立在线点云碰撞守卫的 ROS 参数覆盖
                ),
            ),
            DeclareLaunchArgument(
                'planner_config',
                default_value=str(
                    share / 'config' / 'scan_vendor_planner.yaml'
                ),
                description=(
                    'Optional planner parameter override layered after the '
                    'exact vendor profile. Standalone SCAN remains vendor-like.'
                    # 可选规划器参数覆盖，叠加在精确厂商档之后；独立 SCAN 保持厂商形态
                ),
            ),
            DeclareLaunchArgument(
                'controller_config',
                default_value=str(
                    share / 'config' / 'scan_vendor_controller.yaml'
                ),
                description=(
                    'Optional controller override layered after the exact '
                    'vendor parameters. Standalone SCAN remains vendor-like.'
                    # 可选控制器覆盖，叠加在精确厂商参数之后；独立 SCAN 保持厂商形态
                ),
            ),
            DeclareLaunchArgument(
                'bidirectional_tracking_enabled',
                default_value='false',
                description=(
                    'Opt-in M20 execution adaptation; standalone SCAN keeps '
                    'the vendor forward-only controller behavior.'
                    # 可选 M20 执行适配；独立 SCAN 保持厂商仅前进控制器行为
                ),
            ),
            DeclareLaunchArgument(
                'require_external_execution_hold',
                default_value='false',
                description=(
                    'Optional M20 safety integration. Native SCAN advances '
                    'its trajectory clock exactly as upstream.'
                    # 可选 M20 安全集成；原生 SCAN 与上游完全一致地推进轨迹时钟
                ),
            ),
            DeclareLaunchArgument(
                'collision_guard_required',
                default_value='true',
                description=(
                    'Require the independent online-cloud guard before '
                    'typed/managed goals are relayed to native SCAN.'
                    # 类型化/受管目标转发给原生 SCAN 前，是否要求独立在线点云守卫
                ),
            ),
            DeclareLaunchArgument(
                'reverse_tracking_enter_angle',
                default_value='2.10',      # 进入倒车跟踪的航向差角（弧度）≈120°
            ),
            DeclareLaunchArgument(
                'reverse_tracking_exit_angle',
                default_value='1.75',      # 退出倒车跟踪的航向差角（弧度）≈100°
            ),
            DeclareLaunchArgument(
                'reverse_tracking_min_hold_sec',
                default_value='0.80',      # 倒车跟踪最短保持时间（秒）
            ),
            DeclareLaunchArgument(
                'reverse_tracking_entry_alignment',
                default_value='0.20',      # 进入倒车的对齐阈值（米）
            ),
            DeclareLaunchArgument(
                'reverse_tracking_exit_alignment',
                default_value='0.35',      # 退出倒车的对齐阈值（米）
            ),
            local_sensing,        # 局部感知节点
            navigation_gateway,   # 导航网关节点
            planner,              # SCAN 规划器节点
            controller,           # 闭环控制器节点
        ]
    )
