# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：official_locomotion.launch.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：启动"真实 M20 工厂运动后端"（位于公共安全 Twist 之后）。
#   - 通过 factory_transport 参数选择传输方式：basic_server（默认）或 direct_ros；
#   - 加载版本化能力配置文件（m20_factory_agile_flat_capabilities.yaml），
#     把其中的 intent 参数注入 m20_locomotion_manager；
#   - 启动 m20_locomotion_manager + 所选后端节点；
#   - 刻意保守的"真实硬件启动契约"：command_ownership_confirmed 与
#     auto_enable_motion 默认均为 false，需人工确认后才能运动。
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

"""Start the real M20 factory motion backend behind the common safe Twist."""
# 【中文注释】模块说明：在公共安全 Twist 之后启动真实 M20 工厂运动后端。

from pathlib import Path             # 跨平台路径对象

from ament_index_python.packages import get_package_share_directory  # 获取包共享目录
from launch import LaunchDescription  # 启动描述
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # 启动动作：参数声明/惰性函数
from launch.substitutions import LaunchConfiguration  # 启动参数取值
from launch_ros.actions import Node   # ROS 节点动作
from launch_ros.parameter_descriptions import ParameterValue  # 参数值（带类型）

from m20_nav2_locomotion.capability_profile import (  # 能力配置文件加载
    load_capability_profile,
)


def _runtime_actions(context):
    # 【中文注释】运行时动作：解析传输方式/配置路径/能力配置，生成要启动的节点列表。
    transport = LaunchConfiguration('factory_transport').perform(context)
    if transport not in {'basic_server', 'direct_ros'}:  # 校验传输方式
        raise RuntimeError(
            'factory_transport must be basic_server or direct_ros'
        )
    config_path = LaunchConfiguration('factory_motion_config').perform(
        context
    )
    if not config_path:  # 未显式指定配置 → 使用随包分发的传输相关配置文件
        share = Path(get_package_share_directory('m20_nav2_locomotion'))
        filename = (
            'm20_factory_basic_server.yaml'
            if transport == 'basic_server'
            else 'm20_factory_direct_ros.yaml'
        )
        config_path = str(share / 'config' / filename)
    profile = load_capability_profile(  # 加载并校验能力配置文件
        LaunchConfiguration('locomotion_capability_config').perform(context)
    )
    overrides = {  # 从启动参数注入的两个布尔覆盖项（带类型）
        'command_ownership_confirmed': ParameterValue(
            LaunchConfiguration('command_ownership_confirmed'),
            value_type=bool,
        ),
        'auto_enable_motion': ParameterValue(
            LaunchConfiguration('auto_enable_motion'), value_type=bool
        ),
    }
    executable = (  # 按传输方式选择后端可执行文件
        'm20_basic_server_backend'
        if transport == 'basic_server'
        else 'm20_direct_ros_backend'
    )
    if transport == 'basic_server':
        overrides['robot_host'] = LaunchConfiguration('robot_host')  # 机器人地址覆盖
    return [
        Node(  # 运动管理器节点：安全 Twist → SDK Twist
            package='m20_nav2_locomotion',
            executable='m20_locomotion_manager',
            name='m20_locomotion_manager',
            output='screen',
            parameters=[
                config_path,
                profile.intent_parameters(),  # 注入能力配置的意图参数
                {'require_backend_ready': True},  # 真实硬件要求后端就绪才放行
            ],
        ),
        Node(  # 所选工厂后端节点
            package='m20_nav2_locomotion',
            executable=executable,
            name=executable,
            output='screen',
            parameters=[config_path, overrides],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the intentionally guarded real-hardware launch contract."""
    # 【中文注释】声明"刻意保守"的真实硬件启动契约（默认全部禁用运动）。
    share = Path(get_package_share_directory('m20_nav2_locomotion'))
    return LaunchDescription(
        [
            DeclareLaunchArgument(  # 传输方式参数
                'factory_transport',
                default_value='basic_server',
                description=(
                    'Factory transport selected behind the common safe '
                    'Twist contract: basic_server or direct_ros.'
                ),
            ),
            DeclareLaunchArgument(  # 后端配置覆盖参数
                'factory_motion_config',
                default_value='',
                description=(
                    'Optional backend YAML override. Empty selects the '
                    'transport-specific packaged profile.'
                ),
            ),
            DeclareLaunchArgument(  # 能力配置文件路径参数
                'locomotion_capability_config',
                default_value=str(
                    share
                    / 'config'
                    / 'm20_factory_agile_flat_capabilities.yaml'
                ),
                description='M20 factory-gait navigation capability profile.',
            ),
            DeclareLaunchArgument(  # 机器人地址参数
                'robot_host',
                default_value='10.21.31.103',
                description='M20 AOS basic_server address.',
            ),
            DeclareLaunchArgument(  # 指令所有权确认参数
                'command_ownership_confirmed',
                default_value='false',
                description=(
                    'Set true only after the onboard planner and autonomous '
                    'charging motion sources have been stopped.'
                ),
            ),
            DeclareLaunchArgument(  # 自动启用运动参数
                'auto_enable_motion',
                default_value='false',
                description=(
                    'Normally leave false and enable through '
                    '/m20/hardware/enable_motion after the preflight check.'
                ),
            ),
            OpaqueFunction(function=_runtime_actions),  # 惰性执行运行时动作
        ]
    )
