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
# 文件：phase5_regression.launch.py
# 功能：在完整系统图中运行阶段 5（phase-5）稳定性/故障回归测试。
#       以生产速度组合完整的巡检任务 launch（inspection_mission_rviz
#       .launch.py），同时启动一个回归控制器节点（m20_phase5_regression）
#       执行楼层切换压力（switch_stress）等回归场景，并对内存增长/
#       斜率设置阈值监控；回归节点退出时自动触发整个 launch 关闭。
# ============================================================================

"""Launch phase-5 stability or fault regression inside the system graph."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import (  # 启动动作
    DeclareLaunchArgument,  # 声明 launch 参数
    EmitEvent,  # 发出事件（如 Shutdown 关闭事件）
    IncludeLaunchDescription,  # 包含其他 launch 文件
    RegisterEventHandler,  # 注册事件处理器
)
from launch.event_handlers import OnProcessExit  # 进程退出事件处理器
from launch.events import Shutdown  # launch 关闭事件
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点


def generate_launch_description() -> LaunchDescription:
    """Compose production-speed phase 4 with one regression controller."""
    # —— 路径准备 ——
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    default_config = (  # 默认系统配置（平面多楼层）
        integration / 'config' / 'flat_multifloor_system.yaml'
    )
    # —— launch 参数（可被命令行覆盖）——
    system_config = LaunchConfiguration('system_config')  # 系统配置文件路径
    use_rviz = LaunchConfiguration('use_rviz')  # 是否启动 RViz
    mode = LaunchConfiguration('mode')  # 回归模式（如 switch_stress 楼层切换压力）
    switch_iterations = LaunchConfiguration('switch_iterations')  # 楼层切换迭代次数
    mission_runs = LaunchConfiguration('mission_runs')  # 任务运行次数
    mission_timeout = LaunchConfiguration('mission_timeout_sec')  # 单次任务超时（秒）
    memory_growth_limit = LaunchConfiguration('memory_growth_limit_mib')  # 内存增长上限（MiB）
    memory_slope_limit = LaunchConfiguration(  # 内存增长斜率上限（MiB）
        'memory_slope_limit_mib'
    )
    # —— 构建回归控制器节点 ——
    regression = Node(  # m20_phase5_regression：回归测试控制器
        package='m20_warehouse_inspection',  # 所属功能包
        executable='m20_phase5_regression',  # 可执行文件名（本包脚本）
        name='m20_phase5_regression',  # 节点名
        output='screen',  # 日志输出到屏幕
        parameters=[
            {
                'config_path': system_config,  # 系统配置
                'mode': mode,  # 回归模式
                'switch_iterations': switch_iterations,  # 楼层切换迭代次数
                'mission_runs': mission_runs,  # 任务运行次数
                'mission_timeout_sec': mission_timeout,  # 任务超时
                'memory_growth_limit_mib': memory_growth_limit,  # 内存增长上限
                'memory_slope_limit_mib': memory_slope_limit,  # 内存斜率上限
            }
        ],
    )
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument('use_rviz', default_value='false'),  # 回归测试默认不启动 RViz
            DeclareLaunchArgument(  # 系统配置文件
                'system_config',
                default_value=str(default_config),  # 默认平面多楼层配置
                description='Absolute system profile YAML path.',
            ),
            DeclareLaunchArgument(  # 回归模式
                'mode', default_value='switch_stress'  # 默认楼层切换压力测试
            ),
            DeclareLaunchArgument(  # 默认 20 次楼层切换迭代
                'switch_iterations', default_value='20'
            ),
            DeclareLaunchArgument('mission_runs', default_value='10'),  # 默认 10 次任务运行
            DeclareLaunchArgument(  # 默认任务超时 900 秒
                'mission_timeout_sec', default_value='900.0'
            ),
            DeclareLaunchArgument(  # 默认内存增长上限 128 MiB
                'memory_growth_limit_mib', default_value='128.0'
            ),
            DeclareLaunchArgument(  # 默认内存斜率上限 8 MiB
                'memory_slope_limit_mib', default_value='8.0'
            ),
            # —— 包含完整巡检任务 launch ——
            IncludeLaunchDescription(  # 包含 inspection_mission_rviz.launch.py（生产速度完整系统）
                PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                    str(
                        integration
                        / 'launch'
                        / 'inspection_mission_rviz.launch.py'
                    )
                ),
                launch_arguments={  # 向被包含 launch 传入的参数
                    'use_rviz': use_rviz,  # 透传：是否启动 RViz
                    'use_planner': 'true',  # 强制启用规划器
                    'run_acceptance': 'false',  # 回归测试不运行验收
                    'system_config': system_config,  # 系统配置
                }.items(),
            ),
            # —— 注册回归节点退出事件处理器 ——
            RegisterEventHandler(  # 监听回归节点退出事件
                OnProcessExit(
                    target_action=regression,  # 目标动作：回归节点
                    on_exit=[  # 退出后执行的动作
                        EmitEvent(  # 发出关闭事件
                            event=Shutdown(  # 关闭整个 launch
                                reason='phase-5 regression finished'
                            )
                        )
                    ],
                )
            ),
            regression,  # 启动回归控制器节点
        ]
    )
