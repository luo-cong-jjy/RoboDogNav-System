from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_factory_defaults(config_file):
    defaults = {
        'use_viewer': 'true',
        'initial_x': '0.0',
        'initial_y': '0.0',
        'initial_z': '0.2',
        'initial_yaw': '0.0',
        'render_interval': '50',
        'enable_drdds_bridge': 'true',
        'publish_ground_truth_odom': 'true',
        'publish_tf': 'true',
        'odom_topic': '/odom',
        'odom_frame_id': 'odom',
        'base_frame_id': 'base_link',
    }

    for line in Path(config_file).read_text(encoding='utf-8').splitlines():
        stripped = line.strip()
        if ':' not in stripped or stripped.startswith('#'):
            continue
        key, value = stripped.split(':', 1)
        key = key.strip()
        if key in defaults:
            defaults[key] = value.strip().strip('"\'')

    return defaults


def generate_launch_description():
    package_share = Path(get_package_share_directory('m20_industrial_inspection'))
    config_file = str(package_share / 'config' / 'factory_sim.yaml')
    defaults = _load_factory_defaults(config_file)

    return LaunchDescription([
        DeclareLaunchArgument('use_viewer', default_value=defaults['use_viewer']),
        DeclareLaunchArgument('initial_x', default_value=defaults['initial_x']),
        DeclareLaunchArgument('initial_y', default_value=defaults['initial_y']),
        DeclareLaunchArgument('initial_z', default_value=defaults['initial_z']),
        DeclareLaunchArgument('initial_yaw', default_value=defaults['initial_yaw']),
        DeclareLaunchArgument('render_interval', default_value=defaults['render_interval']),
        DeclareLaunchArgument('enable_drdds_bridge', default_value=defaults['enable_drdds_bridge']),
        DeclareLaunchArgument(
            'publish_ground_truth_odom',
            default_value=defaults['publish_ground_truth_odom'],
        ),
        DeclareLaunchArgument('publish_tf', default_value=defaults['publish_tf']),
        DeclareLaunchArgument('odom_topic', default_value=defaults['odom_topic']),
        DeclareLaunchArgument('odom_frame_id', default_value=defaults['odom_frame_id']),
        DeclareLaunchArgument('base_frame_id', default_value=defaults['base_frame_id']),
        Node(
            package='m20_industrial_inspection',
            executable='m20_factory_simulation',
            name='m20_factory_simulation',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_viewer': ParameterValue(LaunchConfiguration('use_viewer'), value_type=bool),
                    'initial_x': ParameterValue(LaunchConfiguration('initial_x'), value_type=float),
                    'initial_y': ParameterValue(LaunchConfiguration('initial_y'), value_type=float),
                    'initial_z': ParameterValue(LaunchConfiguration('initial_z'), value_type=float),
                    'initial_yaw': ParameterValue(LaunchConfiguration('initial_yaw'), value_type=float),
                    'render_interval': ParameterValue(LaunchConfiguration('render_interval'), value_type=int),
                    'enable_drdds_bridge': ParameterValue(
                        LaunchConfiguration('enable_drdds_bridge'),
                        value_type=bool,
                    ),
                    'publish_ground_truth_odom': ParameterValue(
                        LaunchConfiguration('publish_ground_truth_odom'),
                        value_type=bool,
                    ),
                    'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                    'base_frame_id': LaunchConfiguration('base_frame_id'),
                },
            ],
        ),
    ])
