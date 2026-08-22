# ============================================================================
# 文件：pcd_nav2_planner.launch.py
# 功能：PCD 地图规划器验证 launch：
#       基于由 PCD 生成的栅格地图（occupancy grid，t100ipro_grid.yaml）
#       启动 Nav2 map_server 与 planner_server，并逐步激活其生命周期，
#       验证纯 2D 全局路径规划（无代价地图、无控制器）在离线地图上的可行性。
#       同时发布 map→base_link 静态 TF 供规划测试使用。
# ============================================================================

from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, TimerAction  # 声明参数、定时延迟动作
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 参数替换、路径拼接替换
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.parameter_descriptions import ParameterValue  # 把 launch 参数显式转换为指定 ROS2 参数类型
from launch_ros.substitutions import FindPackageShare  # 运行时查找包的 share 目录


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")  # 栅格地图 YAML 文件路径（可被命令行覆盖）
    package_share = FindPackageShare("m20_nav2_system")  # 运行时查找本包的 share 目录
    planner_params = PathJoinSubstitution([  # 规划器参数文件路径
        package_share,
        "config",
        "pcd_nav2_planner.yaml",
    ])
    default_map_yaml = PathJoinSubstitution([  # 默认 PCD 栅格地图路径
        package_share,
        "maps",
        "pcd", "grids",
        "t100ipro_grid.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),  # 声明地图文件参数，默认使用 PCD 生成的栅格地图

        # —— 启动地图服务器 ——
        Node(  # 启动 map_server：加载栅格地图并发布 /map 话题
            package="nav2_map_server",  # 所属功能包
            executable="map_server",  # 可执行文件名
            name="map_server",  # 节点名
            output="screen",  # 日志输出到屏幕
            parameters=[  # 节点参数
                planner_params,  # 规划器公共参数文件
                {
                    "use_sim_time": False,  # 离线模式不使用仿真时间
                    "yaml_filename": ParameterValue(map_yaml, value_type=str),  # 地图 YAML 文件路径（字符串型）
                },
            ],
        ),

        # —— 启动全局规划器 ——
        Node(  # 启动 planner_server：提供全局路径规划服务
            package="nav2_planner",  # 所属功能包
            executable="planner_server",  # 可执行文件名
            name="planner_server",  # 节点名
            output="screen",  # 日志输出到屏幕
            parameters=[planner_params],  # 加载规划器参数
        ),

        # —— 静态 TF 发布（规划测试占位）——
        Node(  # 发布 map→base_link 静态变换（测试占位：假设机器人位于地图原点）
            package="tf2_ros",  # 所属功能包
            executable="static_transform_publisher",  # 静态变换发布可执行文件
            name="static_map_to_base_link_for_planner_test",  # 节点名
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],  # 单位变换：map → base_link
        ),

        # —— 延迟激活 map_server 生命周期 ——
        TimerAction(  # 定时动作：1 秒后激活 map_server
            period=1.0,  # 延迟 1 秒（等 map_server 节点启动）
            actions=[
                Node(  # 把 map_server 从 unconfigured 推进到 active
                    package="m20_nav2_system",  # 所属功能包
                    executable="lifecycle_configure_activate",  # 生命周期 configure+activate 脚本
                    name="activate_pcd_map_server",  # 节点名
                    output="screen",  # 日志输出到屏幕
                    parameters=[{  # 激活参数
                        "target_node": "/map_server",  # 目标生命周期节点名
                        "service_timeout_sec": 15.0,  # 服务调用超时 15 秒
                        "transition_timeout_sec": 20.0,  # 状态迁移超时 20 秒
                        "activate_delay_sec": 0.5,  # 激活前额外延迟 0.5 秒
                    }],
                ),
            ],
        ),

        # —— 延迟激活 planner_server 生命周期 ——
        TimerAction(  # 定时动作：3 秒后激活 planner_server
            period=3.0,  # 延迟 3 秒（等 map_server 先激活并发布地图）
            actions=[
                Node(  # 把 planner_server 从 unconfigured 推进到 active
                    package="m20_nav2_system",  # 所属功能包
                    executable="lifecycle_configure_activate",  # 生命周期 configure+activate 脚本
                    name="activate_pcd_planner_server",  # 节点名
                    output="screen",  # 日志输出到屏幕
                    parameters=[{  # 激活参数
                        "target_node": "/planner_server",  # 目标生命周期节点名
                        "service_timeout_sec": 30.0,  # 服务调用超时 30 秒
                        "transition_timeout_sec": 60.0,  # 状态迁移超时 60 秒
                        "activate_delay_sec": 0.5,  # 激活前额外延迟 0.5 秒
                    }],
                ),
            ],
        ),
    ])
