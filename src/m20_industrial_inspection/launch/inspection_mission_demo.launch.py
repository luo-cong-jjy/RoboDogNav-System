from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = Path(get_package_share_directory('m20_industrial_inspection'))
    robot_description = (
        package_share / 'models' / 'urdf' / 'M20_visual.urdf'
    ).read_text()

    safety_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'safety.yaml',
    ])
    sim_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'sim.yaml',
    ])
    tracker_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'odom_tracker.yaml',
    ])
    mission_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'inspection_mission.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'rviz',
        'odom_patrol.rviz',
    ])

    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_model = LaunchConfiguration('use_robot_model')
    freeze_wheel_joints = LaunchConfiguration('freeze_wheel_joints')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否随巡检任务演示一起启动 RViz。',
        ),
        DeclareLaunchArgument(
            'use_robot_model',
            default_value='true',
            description='是否启动 M20 URDF 模型显示链路。',
        ),
        DeclareLaunchArgument(
            'freeze_wheel_joints',
            default_value='true',
            description='RViz 模型显示时是否固定四个轮子关节。',
        ),
        Node(
            package='m20_industrial_inspection',
            executable='cmd_vel_safety_mux_node',
            name='cmd_vel_safety_mux',
            output='screen',
            parameters=[safety_config],
        ),
        Node(
            package='m20_industrial_inspection',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[sim_config],
        ),
        Node(
            package='m20_industrial_inspection',
            executable='odom_waypoint_patrol_node',
            name='odom_waypoint_patrol_node',
            output='screen',
            parameters=[tracker_config],
        ),
        Node(
            package='m20_industrial_inspection',
            executable='inspection_mission_node',
            name='inspection_mission_node',
            output='screen',
            parameters=[mission_config],
        ),
        Node(
            package='m20_industrial_inspection',
            executable='m20_joint_state_bridge',
            name='m20_joint_state_bridge',
            output='screen',
            parameters=[{
                'freeze_wheel_joints': ParameterValue(freeze_wheel_joints, value_type=bool),
                'use_current_time_stamp': True,
            }],
            condition=IfCondition(use_robot_model),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='m20_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(use_robot_model),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='m20_inspection_mission_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
