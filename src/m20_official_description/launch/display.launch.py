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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the standalone M20 model display graph."""
    share = Path(get_package_share_directory('m20_official_description'))
    model = share / 'urdf' / 'm20_official.urdf'
    rviz = share / 'rviz' / 'model.rviz'
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='m20_robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        # Load the ROS-facing copy of the official URDF
                        # directly; no sensor or project geometry is injected.
                        'robot_description': model.read_text(encoding='utf-8')
                    }
                ],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='m20_joint_state_publisher',
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='m20_model_rviz',
                arguments=['-d', str(rviz)],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]
    )
