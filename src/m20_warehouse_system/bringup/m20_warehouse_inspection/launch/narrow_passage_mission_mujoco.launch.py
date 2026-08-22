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
# 文件：narrow_passage_mission_mujoco.launch.py
# 功能：在完整 MuJoCo 图中运行一次受控的 M20 门洞宽度测试。
#       通过 width 参数选择门洞宽度（0.60~0.90 米），通过
#       clearance_profile 选择间隙配置（vendor/m20/tight/balanced/
#       conservative），由 OpaqueFunction 在运行期解析对应的系统配置
#       与间隙配置（narrow_passage_<tag>_system.yaml、clearance_<profile>.yaml），
#       再包含完整的 MuJoCo 巡检任务 launch，保持生产节点图不变。
# ============================================================================

"""Launch one controlled M20 doorway-width test in the full MuJoCo graph."""

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


WIDTH_TAGS = {  # 门洞宽度（米）→ 配置文件名标签 的映射表
    '0.60': '060',
    '0.65': '065',
    '0.70': '070',
    '0.75': '075',
    '0.80': '080',
    '0.90': '090',
}
CLEARANCE_PROFILES = (  # 可选的间隙配置档位
    'vendor',  # 上游 vendor 原生间隙
    'm20',  # M20 官方几何间隙
    'tight',  # 紧间隙
    'balanced',  # 平衡间隙
    'conservative',  # 保守间隙
)


def _include_profile(context):
    # 运行期回调：根据 width/clearance_profile 解析配置并包含完整 MuJoCo 任务 launch
    integration = Path(  # 本包 share 目录
        get_package_share_directory('m20_warehouse_inspection')
    )
    navigation = Path(  # SCAN 导航包 share 目录（存放间隙配置）
        get_package_share_directory('m20_scan_navigation')
    )
    width = LaunchConfiguration('width').perform(context)  # 读取门洞宽度参数
    tag = WIDTH_TAGS.get(width)  # 查表得到配置标签
    if tag is None:  # 非法宽度则报错
        raise RuntimeError(
            f'unsupported width {width!r}; choose one of '
            f'{", ".join(WIDTH_TAGS)}'
        )
    clearance = LaunchConfiguration('clearance_profile').perform(context)  # 读取间隙档位参数
    if clearance not in CLEARANCE_PROFILES:  # 非法档位则报错
        raise RuntimeError(
            f'unsupported clearance profile {clearance!r}; choose one of '
            f'{", ".join(CLEARANCE_PROFILES)}'
        )
    system_config = (  # 对应宽度的窄通道系统配置
        integration / 'config' / f'narrow_passage_{tag}_system.yaml'
    )
    clearance_config = (  # 对应档位的间隙配置文件
        navigation / 'config' / f'clearance_{clearance}.yaml'
    )
    for path in (system_config, clearance_config):  # 校验两个配置文件都存在
        if not path.is_file():
            raise RuntimeError(f'missing narrow-passage test input: {path}')
    return [
        # —— 包含完整 MuJoCo 巡检任务 launch ——
        IncludeLaunchDescription(  # 包含 inspection_mission_mujoco.launch.py
            PythonLaunchDescriptionSource(  # 源文件为 Python 格式的 launch
                str(
                    integration
                    / 'launch'
                    / 'inspection_mission_mujoco.launch.py'
                )
            ),
            launch_arguments={  # 向被包含 launch 传入的参数
                'system_config': str(system_config),  # 本次测试的系统配置（绝对路径）
                'clearance_config': str(clearance_config),  # 本次测试的间隙配置（绝对路径）
                'use_rviz': LaunchConfiguration('use_rviz'),  # 透传：是否启动 RViz
                'use_planner': 'true',  # 强制启用规划器
                'use_mujoco_viewer': LaunchConfiguration(  # 透传：是否打开 MuJoCo 查看器
                    'use_mujoco_viewer'
                ),
                'mujoco_viewer_distance': LaunchConfiguration(  # 透传：相机距离
                    'mujoco_viewer_distance'
                ),
                'navigation_timeout_sec': LaunchConfiguration(  # 透传：单步导航超时
                    'navigation_timeout_sec'
                ),
                'velocity_feedback_enabled': LaunchConfiguration(  # 透传：速度反馈开关
                    'velocity_feedback_enabled'
                ),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Select width and clearance without changing the production graph."""
    return LaunchDescription(
        [
            # —— 声明参数默认值 ——
            DeclareLaunchArgument(  # 门洞宽度参数
                'width',
                default_value='0.80',  # 默认 0.80 米
                choices=list(WIDTH_TAGS),  # 可选值受映射表约束
                description='Nominal obstacle-surface doorway width.',
            ),
            DeclareLaunchArgument(  # 间隙档位参数
                'clearance_profile',
                default_value='m20',  # 默认 M20 官方几何间隙
                choices=list(CLEARANCE_PROFILES),  # 可选值受档位表约束
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 默认启动 RViz
            DeclareLaunchArgument(  # 默认打开 MuJoCo 查看器
                'use_mujoco_viewer',
                default_value='true',
            ),
            DeclareLaunchArgument(  # 默认相机距离 4 米
                'mujoco_viewer_distance',
                default_value='4.0',
            ),
            DeclareLaunchArgument(  # 默认单步导航超时 120 秒
                'navigation_timeout_sec',
                default_value='120.0',
            ),
            DeclareLaunchArgument(  # 默认不启用速度反馈
                'velocity_feedback_enabled',
                default_value='false',
            ),
            OpaqueFunction(function=_include_profile),  # 运行期执行配置解析与 launch 包含
        ]
    )
