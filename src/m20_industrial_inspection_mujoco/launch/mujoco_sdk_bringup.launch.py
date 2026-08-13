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
    package_share = Path(get_package_share_directory('m20_industrial_inspection_mujoco'))
    robot_description = (
        package_share / 'models' / 'urdf' / 'M20_visual.urdf'
    ).read_text()

    safety_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'config',
        'safety.yaml',
    ])
    sim_adapter_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'config',
        'sim.yaml',
    ])
    factory_sim_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'config',
        'factory_sim.yaml',
    ])
    sdk_deploy_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'config',
        'sdk_deploy_cmdvel.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection_mujoco'),
        'rviz',
        'mujoco_control.rviz',
    ])

    start_sim = LaunchConfiguration('start_sim')
    start_sdk_deploy = LaunchConfiguration('start_sdk_deploy')
    start_control_core = LaunchConfiguration('start_control_core')
    use_viewer = LaunchConfiguration('use_viewer')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_model = LaunchConfiguration('use_robot_model')
    freeze_wheel_joints = LaunchConfiguration('freeze_wheel_joints')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_sim',
            default_value='true',
            description='是否启动项目 MuJoCo 16 自由度仿真节点。',
        ),
        DeclareLaunchArgument(
            'start_sdk_deploy',
            default_value='true',
            description='是否启动官方 m20_sdk_deploy/rl_deploy_cmdvel 控制器。',
        ),
        DeclareLaunchArgument(
            'start_control_core',
            default_value='true',
            description='是否启动安全仲裁和运动适配层。',
        ),
        DeclareLaunchArgument(
            'use_viewer',
            default_value='true',
            description='是否打开 MuJoCo viewer。',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz；底层 bringup 默认打开。',
        ),
        DeclareLaunchArgument(
            'use_robot_model',
            default_value='true',
            description='是否启动 /joint_states 到 M20 URDF 的可视化链路。',
        ),
        DeclareLaunchArgument(
            'freeze_wheel_joints',
            default_value='true',
            description='RViz 模型显示时是否固定四个轮子关节。',
        ),
        Node(
            package='m20_industrial_inspection_mujoco',
            executable='m20_factory_simulation',
            name='m20_factory_simulation',
            output='screen',
            parameters=[
                factory_sim_config,
                {
                    'use_viewer': ParameterValue(use_viewer, value_type=bool),
                    'enable_drdds_bridge': True,
                    'publish_ground_truth_odom': True,
                    'publish_tf': True,
                },
            ],
            condition=IfCondition(start_sim),
        ),
        Node(
            package='m20_sdk_deploy',
            executable='rl_deploy_cmdvel',
            output='screen',
            parameters=[sdk_deploy_config],
            condition=IfCondition(start_sdk_deploy),
        ),
        Node(
            package='m20_industrial_inspection_mujoco',
            executable='cmd_vel_safety_mux_node',
            name='cmd_vel_safety_mux',
            output='screen',
            parameters=[safety_config],
            condition=IfCondition(start_control_core),
        ),
        Node(
            package='m20_industrial_inspection_mujoco',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[sim_adapter_config],
            condition=IfCondition(start_control_core),
        ),
        Node(
            package='m20_industrial_inspection_mujoco',
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
            name='m20_mujoco_sdk_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
