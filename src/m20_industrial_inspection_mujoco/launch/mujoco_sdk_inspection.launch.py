from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_launch = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'launch',
        'mujoco_sdk_bringup.launch.py',
    ])

    start_sim = LaunchConfiguration('start_sim')
    start_sdk_deploy = LaunchConfiguration('start_sdk_deploy')
    start_control_core = LaunchConfiguration('start_control_core')
    use_viewer = LaunchConfiguration('use_viewer')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_model = LaunchConfiguration('use_robot_model')
    freeze_wheel_joints = LaunchConfiguration('freeze_wheel_joints')

    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('start_sdk_deploy', default_value='true'),
        DeclareLaunchArgument('start_control_core', default_value='true'),
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_robot_model', default_value='true'),
        DeclareLaunchArgument('freeze_wheel_joints', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'start_sim': start_sim,
                'start_sdk_deploy': start_sdk_deploy,
                'start_control_core': start_control_core,
                'use_viewer': use_viewer,
                'use_rviz': use_rviz,
                'use_robot_model': use_robot_model,
                'freeze_wheel_joints': freeze_wheel_joints,
            }.items(),
        ),
    ])
