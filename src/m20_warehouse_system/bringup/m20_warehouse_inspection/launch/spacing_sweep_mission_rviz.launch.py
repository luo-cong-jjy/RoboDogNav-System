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
# 文件：spacing_sweep_mission_rviz.launch.py
# 功能：启动「障碍间距扫描（obstacle-spacing sweep）」单组实验配置的
#       RViz 仿真巡检任务。通过 spacing 参数选择障碍物间距（0.70/0.80/
#       0.90/1.00 米），由 OpaqueFunction 在运行期解析出对应的系统配置文件
#       （spacing_sweep_<tag>_system.yaml），再包含完整的巡检任务 launch
#       （inspection_mission_rviz.launch.py），保持生产节点图不变。
# ============================================================================

"""Launch one controlled obstacle-spacing experiment profile."""

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import (  # 启动动作
    DeclareLaunchArgument,  # 声明 launch 参数
    IncludeLaunchDescription,  # 包含其他 launch 文件
    OpaqueFunction,  # 不透明函数动作：运行期执行回调以动态构建动作列表
)
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数的值


PROFILE_TAGS = {  # 间距（米）→ 配置文件名标签 的映射表
    '0.70': '070',
    '0.80': '080',
    '0.90': '090',
    '1.00': '100',
}


def _include_profile(context):
    # 运行期回调：根据 spacing 参数解析对应系统配置并包含完整任务 launch
    integration = Path(  # 本包安装后的 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    spacing = LaunchConfiguration('spacing').perform(context)  # 读取并解析 spacing 参数值
    tag = PROFILE_TAGS.get(spacing)  # 查表得到配置文件标签
    if tag is None:  # 非法间距值则报错
        choices = ', '.join(PROFILE_TAGS)
        raise RuntimeError(
            f'unsupported spacing {spacing!r}; choose one of: {choices}'
        )
    config = integration / 'config' / (  # 拼接对应间距的配置文件路径
        f'spacing_sweep_{tag}_system.yaml'
    )
    if not config.is_file():  # 配置文件缺失则提示先运行生成工具
        raise RuntimeError(
            f'missing generated spacing profile: {config}; regenerate it '
            'with tools/generate_spacing_sweep.py'
        )
    return [
        # —— 包含完整巡检任务 launch ——
        IncludeLaunchDescription(  # 包含 inspection_mission_rviz.launch.py，复用生产节点图
            PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                str(
                    integration
                    / 'launch'
                    / 'inspection_mission_rviz.launch.py'
                )
            ),
            launch_arguments={  # 向被包含 launch 传入的参数
                'system_config': str(config),  # 本次实验的系统配置文件（绝对路径）
                'use_rviz': LaunchConfiguration('use_rviz'),  # 透传：是否启动 RViz
                'use_planner': LaunchConfiguration('use_planner'),  # 透传：是否启动规划器
                'run_acceptance': LaunchConfiguration('run_acceptance'),  # 透传：是否运行验收
                'acceptance_mode': LaunchConfiguration('acceptance_mode'),  # 透传：验收模式
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Select one spacing while preserving the production node graph."""
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(
                'spacing',
                default_value='0.70',  # 默认障碍物间距 0.70 米
                choices=list(PROFILE_TAGS),  # 可选值受映射表约束
                description='Obstacle body-to-body minimum spacing in metres.',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument('use_planner', default_value='true'),  # 默认启动规划器
            DeclareLaunchArgument('run_acceptance', default_value='false'),  # 默认不运行验收
            DeclareLaunchArgument('acceptance_mode', default_value='quick'),  # 默认验收模式 quick
            OpaqueFunction(function=_include_profile),  # 运行期执行配置解析与 launch 包含
        ]
    )
