from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
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
    sim_adapter_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'sim.yaml',
    ])
    factory_sim_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'factory_sim.yaml',
    ])
    sdk_deploy_config = PathJoinSubstitution([
        FindPackageShare('m20_industrial_inspection'),
        'config',
        'sdk_deploy_cmdvel.yaml',
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

    start_sim = LaunchConfiguration('start_sim')
    start_sdk_deploy = LaunchConfiguration('start_sdk_deploy')
    start_inspection_core = LaunchConfiguration('start_inspection_core')
    start_mission = LaunchConfiguration('start_mission')
    force_bringup_with_mission = LaunchConfiguration('force_bringup_with_mission')
    use_viewer = LaunchConfiguration('use_viewer')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_model = LaunchConfiguration('use_robot_model')
    freeze_wheel_joints = LaunchConfiguration('freeze_wheel_joints')
    bringup_guard = [
        "'",
        start_mission,
        "' != 'true' or '",
        force_bringup_with_mission,
        "' == 'true'",
    ]

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
            'start_inspection_core',
            default_value='true',
            description='是否启动巡检包的安全仲裁和运动适配层。',
        ),
        DeclareLaunchArgument(
            'start_mission',
            default_value='false',
            description='是否启动 odom 目标跟踪器和巡检任务管理器。二次启动任务时只开任务层，避免重复启动仿真和 RViz。',
        ),
        DeclareLaunchArgument(
            'force_bringup_with_mission',
            default_value='false',
            description='与 start_mission:=true 同时使用时，是否仍强制启动仿真、官方控制器、安全层和可视化；一般不要打开。',
        ),
        DeclareLaunchArgument(
            'use_viewer',
            default_value='true',
            description='是否打开 MuJoCo viewer。',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz；start_mission:=true 且未强制 bringup 时不会重复打开。',
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
        LogInfo(
            msg='start_mission:=true 只启动巡检任务层；如果已经有 MuJoCo/SDK/RViz 在运行，不会再重复启动它们。',
            condition=IfCondition(start_mission),
        ),
        Node(
            package='m20_industrial_inspection',
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
            condition=IfCondition(PythonExpression([
                "'",
                start_sim,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
        Node(
            package='m20_sdk_deploy',
            executable='rl_deploy_cmdvel',
            output='screen',
            parameters=[sdk_deploy_config],
            condition=IfCondition(PythonExpression([
                "'",
                start_sdk_deploy,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
        Node(
            package='m20_industrial_inspection',
            executable='cmd_vel_safety_mux_node',
            name='cmd_vel_safety_mux',
            output='screen',
            parameters=[safety_config],
            condition=IfCondition(PythonExpression([
                "'",
                start_inspection_core,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
        Node(
            package='m20_industrial_inspection',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[sim_adapter_config],
            condition=IfCondition(PythonExpression([
                "'",
                start_inspection_core,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
        Node(
            package='m20_industrial_inspection',
            executable='odom_waypoint_patrol_node',
            name='odom_waypoint_patrol_node',
            output='screen',
            parameters=[tracker_config],
            condition=IfCondition(start_mission),
        ),
        Node(
            package='m20_industrial_inspection',
            executable='inspection_mission_node',
            name='inspection_mission_node',
            output='screen',
            parameters=[mission_config],
            condition=IfCondition(start_mission),
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
            condition=IfCondition(PythonExpression([
                "'",
                use_robot_model,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='m20_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(PythonExpression([
                "'",
                use_robot_model,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='m20_mujoco_sdk_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(PythonExpression([
                "'",
                use_rviz,
                "' == 'true' and (",
                *bringup_guard,
                ")",
            ])),
        ),
    ])
