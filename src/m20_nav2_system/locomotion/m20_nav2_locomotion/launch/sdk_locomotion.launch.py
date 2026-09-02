# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：sdk_locomotion.launch.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：把安全的仓库指令流连接到官方 M20 RL SDK。
#   - 启动 m20_locomotion_manager（意图适配器）+ 可选的厂商控制器
#     m20_sdk_deploy/rl_deploy_cmdvel（start_sdk 默认 false，需后端在线时开启）；
#   - 加载版本化能力配置文件（默认 m20_policy_v1_capabilities.yaml），
#     注入 intent 参数与 SDK 限幅参数；
#   - execution_profile 三种执行方式：scan_native（直通）/ m20_safe（Twist 适配）/
#     m20_progress（实测空间路径进度）；
#   - 后端就绪/故障话题可覆盖（仿真用 MuJoCo 话题）。
# ============================================================================
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

"""Connect the safe warehouse command stream to the official M20 RL SDK."""
# 【中文注释】模块说明：把安全的仓库指令流连接到官方 M20 RL SDK。

from pathlib import Path             # 跨平台路径对象

from ament_index_python.packages import get_package_share_directory  # 获取包共享目录
from launch import LaunchDescription  # 启动描述
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # 启动动作
from launch.conditions import IfCondition  # 条件动作（按参数决定是否启动节点）
from launch.substitutions import LaunchConfiguration  # 启动参数取值
from launch_ros.actions import Node   # ROS 节点动作

from m20_nav2_locomotion.capability_profile import (  # 能力配置文件加载
    load_capability_profile,
)


def _runtime_actions(context):
    """Resolve and validate one capability profile before starting nodes."""
    # 【中文注释】在启动节点前解析并校验一个能力配置文件。
    share = Path(get_package_share_directory('m20_nav2_locomotion'))
    config = str(share / 'config' / 'sdk_locomotion.yaml')  # 基础配置
    profile_path = LaunchConfiguration(  # 能力配置文件路径
        'locomotion_capability_config'
    ).perform(context)
    profile = load_capability_profile(profile_path)  # 加载并校验
    execution_profile = LaunchConfiguration(  # 执行方式
        'execution_profile'
    ).perform(context)
    if execution_profile not in {  # 校验执行方式取值
        'scan_native',
        'm20_safe',
        'm20_progress',
    }:
        raise RuntimeError(
            'execution_profile must be scan_native, m20_safe, or '
            'm20_progress'
        )
    native_command_parameters = (  # scan_native 模式：直通 + 仅应用数值包络
        {
            # Keep the upstream SCAN Twist semantics, then apply the validated
            # M20 numeric envelope at this downstream motion boundary. Backend
            # readiness and timeout handling stay active.
            # 【中文注释】保持上游 SCAN Twist 语义，仅在该下游运动边界应用校验过的
            # M20 数值包络；后端就绪与超时处理仍然生效。
            'rolling_navigation_enabled': False,  # 不再重复适配
            'max_forward': profile.max_forward,
            'max_side': profile.max_side,
            'max_yaw': profile.max_yaw,
            'deadband_linear': 0.0,   # 直通模式不设死区
            'deadband_yaw': 0.0,
            'turn_max_forward': profile.max_forward,
            'lateral_max_forward': profile.max_forward,
            'suppress_side_in_cruise': False,  # 不抑制横向
            'suppress_side_in_turn': False,
        }
        if execution_profile == 'scan_native'
        else {}  # 其他模式：由能力配置注入
    )
    require_backend_ready = LaunchConfiguration('require_backend_ready')
    backend_ready_topic = LaunchConfiguration('backend_ready_topic')
    backend_fault_topic = LaunchConfiguration('backend_fault_topic')
    return [
        Node(  # 运动管理器节点
            package='m20_nav2_locomotion',
            executable='m20_locomotion_manager',
            name='m20_locomotion_manager',
            output='screen',
            parameters=[
                config,
                profile.intent_parameters(),  # 注入能力配置的意图参数
                native_command_parameters,    # scan_native 覆盖参数（可能为空）
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'require_backend_ready': require_backend_ready,
                    'backend_ready_topic': backend_ready_topic,
                    'backend_fault_topic': backend_fault_topic,
                },
            ],
        ),
        Node(  # 厂商 SDK 控制器节点（可选）
            package='m20_sdk_deploy',
            executable='rl_deploy_cmdvel',
            output='screen',
            parameters=[config, profile.sdk_parameters(), {
                'cmd_vel_topic': LaunchConfiguration('sdk_cmd_vel_topic'),
                'debug_print': LaunchConfiguration('sdk_debug_print'),
            }],  # 注入 SDK 限幅与最终 Twist 入口
            condition=IfCondition(LaunchConfiguration('start_sdk')),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Start the project-owned intent adapter and optional vendor controller."""
    # 【中文注释】启动项目自有的意图适配器与可选的厂商控制器。
    share = Path(get_package_share_directory('m20_nav2_locomotion'))
    default_capability = (  # 默认能力配置文件
        share / 'config' / 'm20_policy_v1_capabilities.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(  # 是否启动 SDK 控制器
                'start_sdk',
                default_value='false',
                description=(
                    'Start m20_sdk_deploy/rl_deploy_cmdvel. Enable only when '
                    'a MuJoCo, Gazebo joint bridge, or real M20 backend is live.'
                ),
            ),
            DeclareLaunchArgument(
                'input_topic', default_value='/m20/control/cmd_vel_safe',
                description='Twist input consumed by the locomotion manager.',
            ),
            DeclareLaunchArgument(
                'sdk_cmd_vel_topic',
                default_value='/m20/locomotion/cmd_vel_sdk',
                description='Final Twist topic consumed by the official SDK.',
            ),
            DeclareLaunchArgument(
                'sdk_debug_print', default_value='false',
                description='Print the SDK command/state gate once per second.',
            ),
            DeclareLaunchArgument(  # 是否要求后端就绪
                'require_backend_ready',
                default_value='false',
                description=(
                    'Gate SDK velocity commands on the MuJoCo ready/fault '
                    'contract.'
                ),
            ),
            DeclareLaunchArgument(  # 后端就绪话题覆盖
                'backend_ready_topic',
                default_value='/m20/sim/backend_ready',
                description='Backend-specific ready source for SDK mode.',
            ),
            DeclareLaunchArgument(  # 后端故障话题覆盖
                'backend_fault_topic',
                default_value='/m20/sim/backend_fault',
                description='Backend-specific fault source for SDK mode.',
            ),
            DeclareLaunchArgument(  # 能力配置文件路径
                'locomotion_capability_config',
                default_value=str(default_capability),
                description=(
                    'Versioned M20 command, turn and recovery capability '
                    'profile.'
                ),
            ),
            DeclareLaunchArgument(  # 执行方式
                'execution_profile',
                default_value='scan_native',
                description=(
                    'scan_native relays SCAN Twist without project-side '
                    'motion projection; m20_safe uses the Twist adapter; '
                    'm20_progress uses measured spatial path progress.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),  # 惰性执行运行时动作
        ]
    )
