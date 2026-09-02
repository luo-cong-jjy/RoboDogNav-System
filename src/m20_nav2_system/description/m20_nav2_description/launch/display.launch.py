# Copyright (c) 2026 Virdyn Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Display the wrapped official M20 model in RViz."""
# 文件用途：在 RViz 中显示封装的官方 Deep Robotics M20 机器人模型
# 启动内容：robot_state_publisher（加载 URDF）+ joint_state_publisher（关节状态）+ rviz2

from pathlib import Path  # 路径处理：拼接包内文件路径

from ament_index_python.packages import get_package_share_directory  # 获取 ROS2 包共享目录
from launch import LaunchDescription          # 启动描述（launch 文件核心结构）
from launch.actions import DeclareLaunchArgument  # 声明可配置的启动参数
from launch.conditions import IfCondition     # 条件判断（按参数决定是否启动节点）
from launch.substitutions import LaunchConfiguration  # 读取启动参数值
from launch_ros.actions import Node           # ROS2 节点启动动作


def generate_launch_description() -> LaunchDescription:
    """Create the standalone M20 model display graph."""
    # 生成启动描述：创建独立的 M20 模型显示节点图
    share = Path(get_package_share_directory('m20_nav2_description'))  # 本包共享目录
    model = share / 'urdf' / 'm20_official.urdf'  # 官方模型 URDF 文件路径
    rviz = share / 'rviz' / 'model.rviz'          # RViz 显示配置文件路径
    use_rviz = LaunchConfiguration('use_rviz')   # 是否启动 RViz 的参数（默认 true）

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),  # 声明参数：use_rviz
            Node(
                package='robot_state_publisher',      # 机器人状态发布包
                executable='robot_state_publisher',   # 发布 TF 树（基于 URDF）
                name='m20_robot_state_publisher',     # 节点名
                output='screen',                      # 日志输出到屏幕
                parameters=[
                    {
                        # Load the ROS-facing copy of the official URDF
                        # directly; no sensor or project geometry is injected.
                        # （直接加载官方 URDF 的 ROS 侧副本，不注入任何传感器或项目几何）
                        'robot_description': model.read_text(encoding='utf-8')  # 读取 URDF 文本作为 robot_description 参数
                    }
                ],
            ),
            Node(
                package='joint_state_publisher',   # 关节状态发布包
                executable='joint_state_publisher',# 发布各关节角度（可 GUI 拖动/读取）
                name='m20_joint_state_publisher',
                output='screen',
            ),
            Node(
                package='rviz2',                   # RViz2 可视化
                executable='rviz2',
                name='m20_model_rviz',
                arguments=['-d', str(rviz)],       # 加载模型显示配置
                condition=IfCondition(use_rviz),   # 仅当 use_rviz=true 时启动
                output='screen',
            ),
        ]
    )
