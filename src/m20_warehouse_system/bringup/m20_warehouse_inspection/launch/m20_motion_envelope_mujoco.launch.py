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
# 文件：m20_motion_envelope_mujoco.launch.py
# 功能：在无障碍的 MuJoCo 测试场中运行官方 M20 SDK 运动包络测试。
#       本 launch 只组合：MuJoCo 物理后端、官方 SDK 策略（rl_deploy_cmdvel
#       命令接口，绕过 SCAN 控制器与项目运动适配器）、以及脚本化测试探针
#       （m20_command_envelope_probe）。探针按 suite/case 发布精确的 Twist
#       命令测量 M20 实际运动包络；探针完成后自动关闭整个 launch。
# ============================================================================

"""Run the official M20 SDK in an obstacle-free MuJoCo test field."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import (  # 启动动作
    DeclareLaunchArgument,  # 声明 launch 参数
    IncludeLaunchDescription,  # 包含其他 launch 文件
    Shutdown,  # 关闭动作：结束整个 launch
)
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点


def generate_launch_description() -> LaunchDescription:
    """Compose only physics, the official SDK policy, and the test probe."""
    # —— 路径准备 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    backend = Path(get_package_share_directory('m20_mujoco_backend'))  # MuJoCo 后端包 share 目录
    locomotion = Path(  # M20 运动控制包 share 目录（SDK 配置）
        get_package_share_directory('m20_locomotion_control')
    )

    # —— launch 参数（可被命令行覆盖）——
    use_viewer = LaunchConfiguration('use_viewer')  # 是否打开 MuJoCo 查看器
    run_probe = LaunchConfiguration('run_probe')  # 是否运行测试探针
    suite = LaunchConfiguration('suite')  # 探针测试套件（smoke/axial/navigation 等）
    case_name = LaunchConfiguration('case_name')  # 可选单用例名（覆盖 suite）
    output_directory = LaunchConfiguration('output_directory')  # 结果输出目录

    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 运动包络测试系统配置
                'system_config',
                default_value=str(
                    integration / 'config' / 'motion_envelope_system.yaml'  # 默认运动包络场景配置
                ),
            ),
            DeclareLaunchArgument('use_viewer', default_value='false'),  # 默认不打开查看器（无头）
            DeclareLaunchArgument('real_time_factor', default_value='1.0'),  # 默认实时因子 1.0
            DeclareLaunchArgument(  # 是否运行探针
                'run_probe',
                default_value='true',  # 默认运行脚本化探针
                description=(
                    'Run the scripted probe and shut down when it completes.'
                ),
            ),
            DeclareLaunchArgument(  # 探针测试套件
                'suite',
                default_value='smoke',  # 默认 smoke 冒烟套件
                description=(
                    'Probe suite: smoke, axial, navigation, reverse, '
                    'turn_matrix, or full.'
                ),
            ),
            DeclareLaunchArgument(  # 可选单用例名
                'case_name',
                default_value='',  # 默认空：按 suite 运行
                description='Optional single case name, overriding suite.',
            ),
            DeclareLaunchArgument(  # 结果输出目录
                'output_directory',
                default_value='/tmp/m20_motion_envelope',  # 默认输出到 /tmp
            ),
            # —— 包含 MuJoCo 后端 ——
            IncludeLaunchDescription(  # 包含 MuJoCo 物理后端 launch
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(backend / 'launch' / 'mujoco_backend.launch.py')
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'system_config': LaunchConfiguration('system_config'),  # 系统配置
                    'package_root': str(integration),  # 包根目录
                    'use_viewer': use_viewer,  # 透传：是否打开查看器
                    'real_time_factor': LaunchConfiguration(  # 透传：实时因子
                        'real_time_factor'
                    ),
                }.items(),
            ),
            # Bypass both the SCAN controller and project motion adapter.
            # The probe publishes exact Twist commands to this SDK input.
            # （绕过 SCAN 控制器与项目运动适配器；探针直接向该 SDK 输入发布精确 Twist 命令）
            Node(  # m20_cmd_vel_interface：官方 SDK 策略的 cmd_vel 接口
                package='m20_sdk_deploy',  # 所属功能包
                executable='rl_deploy_cmdvel',  # 可执行文件名（SDK 部署）
                name='m20_cmd_vel_interface',  # 节点名
                output='screen',  # 日志输出到屏幕
                parameters=[
                    str(  # SDK 运动配置
                        locomotion
                        / 'config'
                        / 'sdk_locomotion.yaml'
                    ),
                ],
            ),
            # —— 启动命令包络测试探针 ——
            Node(  # m20_command_envelope_probe：脚本化测试探针（测量运动包络）
                package='m20_warehouse_inspection',  # 所属功能包
                executable='m20_command_envelope_probe',  # 可执行文件名（本包脚本）
                name='m20_command_envelope_probe',  # 节点名
                output='screen',  # 日志输出到屏幕
                arguments=[  # 命令行参数
                    '--suite',  # 测试套件
                    suite,
                    '--case',  # 单用例名
                    case_name,
                    '--output-directory',  # 输出目录
                    output_directory,
                ],
                condition=IfCondition(run_probe),  # 仅当 run_probe=true 时才启动
                on_exit=[  # 探针退出后的动作
                    Shutdown(  # 关闭整个 launch
                        reason='M20 command-envelope probe completed'
                    )
                ],
            ),
        ]
    )
