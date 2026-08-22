# ============================================================================
# 文件：factory_navigation.launch.py
# 功能：一键启动「工厂仿真 + 自由导航」的总入口 launch。
#       按 lidar_mode 参数选择 2D 扫描（默认）或 3D 雷达 的 Gazebo 仿真，
#       延迟启动 Nav2 导航栈（基于已有地图导航，不建图），
#       再延迟启动 RViz 可视化。
#       注意：巡检任务（factory_inspection_mission.launch.py）不在此启动，
#       需在 Nav2 导航栈就绪后单独运行。
# ============================================================================

"""Start the factory simulation and free navigation stack.

The inspection mission is intentionally not included. Start it separately with
factory_inspection_mission.launch.py after Nav2 is active.
"""

from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction  # 声明参数、包含其他 launch、设置环境变量、定时延迟动作
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression  # 参数替换、路径拼接替换、Python 表达式（用于字符串比较）
from launch_ros.substitutions import FindPackageShare  # 运行时查找包的 share 目录
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
import os  # 操作系统接口：读取环境变量（ROS_DOMAIN_ID、RMW_IMPLEMENTATION）


def generate_launch_description():
    # —— 拼接各子 launch / 资源文件路径（运行时解析）——
    pkg = FindPackageShare("m20_nav2_system")  # 查找本包 share 目录
    gazebo_2d = PathJoinSubstitution([pkg, "launch", "gazebo_sensor_m20_factory_2d_scan.launch.py"])  # 2D 扫描模式 Gazebo launch 路径
    gazebo_3d = PathJoinSubstitution([pkg, "launch", "gazebo_sensor_m20_factory_3d_rslidar.launch.py"])  # 3D 雷达模式 Gazebo launch 路径
    nav2 = PathJoinSubstitution([pkg, "launch", "nav2_map_navigation_m20_factory.launch.py"])  # Nav2 导航栈 launch 路径
    rviz_config = PathJoinSubstitution([pkg, "rviz", "nav2_sandbox.rviz"])  # RViz 配置文件路径
    default_map = PathJoinSubstitution([pkg, "maps", "factory", "factory_slam_map.yaml"])  # 默认地图文件
    default_world = PathJoinSubstitution([pkg, "worlds", "factory_environment_v3_dynamic_obstacles.world"])  # 默认世界文件（v3 含动态障碍物）

    mode = LaunchConfiguration("lidar_mode")  # 雷达模式选择：2d / 3d
    args = {name: LaunchConfiguration(name) for name in  # 批量生成要透传给 Gazebo launch 的参数
            ("use_gazebo_gui", "gazebo_gui_delay", "spawn_delay",
             "rviz_delay", "x", "y", "z", "yaw", "world")}

    common = {key: value for key, value in args.items()}  # 复制一份公共参数表
    # RViz is started by the Nav2 phase, after Gazebo and navigation are ready.
    # RViz 由 Nav2 阶段统一启动（在 Gazebo 和导航就绪之后），这里先关闭 Gazebo 阶段的 RViz
    common["use_rviz"] = "false"
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),  # 是否最终启动 RViz
        # Gazebo Classic GUI can black-screen under WSL/remote OpenGL. Keep
        # the simulation headless by default; opt in explicitly when needed.
        # （WSL/远程 OpenGL 环境下 Gazebo GUI 可能黑屏，默认关闭，需要时显式开启）
        DeclareLaunchArgument("use_gazebo_gui", default_value="true"),  # 是否启动 Gazebo GUI（gzclient）
        DeclareLaunchArgument("gazebo_gui_delay", default_value="3.0"),  # gzclient 延迟 3 秒启动
        DeclareLaunchArgument("spawn_delay", default_value="5.0"),  # 机器人生成延迟 5 秒（等 gzserver 就绪）
        DeclareLaunchArgument("rviz_delay", default_value="18.0"),  # RViz 延迟 18 秒启动（等导航栈完全就绪）
        DeclareLaunchArgument("nav2_delay", default_value="10.0"),  # Nav2 延迟 10 秒启动（等仿真环境就绪）
        DeclareLaunchArgument("lidar_mode", default_value="2d"),  # 雷达模式：默认 2d（轻量 2D 扫描）
        DeclareLaunchArgument("x", default_value="0.0"),  # 机器人出生位置 X
        DeclareLaunchArgument("y", default_value="0.0"),  # 机器人出生位置 Y
        DeclareLaunchArgument("z", default_value="0.59"),  # 机器人出生高度 Z（底盘离地）
        DeclareLaunchArgument("yaw", default_value="0.0"),  # 机器人出生朝向偏航角
        DeclareLaunchArgument("world", default_value=default_world),  # 世界文件默认值（v3 动态障碍物场景）
        DeclareLaunchArgument("map", default_value=default_map),  # 地图文件默认值（工厂 SLAM 地图）
        DeclareLaunchArgument("ros_domain_id", default_value=os.environ.get("ROS_DOMAIN_ID", "0")),  # ROS_DOMAIN_ID：默认取环境变量，否则用 0
        DeclareLaunchArgument("rmw_implementation", default_value=os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")),  # RMW 实现：默认取环境变量，否则用 Fast-DDS
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),  # 设置 ROS_DOMAIN_ID 环境变量（隔离不同 ROS 域）
        SetEnvironmentVariable("RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")),  # 设置 RMW 中间件实现环境变量
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_2d), launch_arguments=common.items(), condition=IfCondition(PythonExpression(["'", mode, "' == '2d'"]))),  # lidar_mode=2d 时包含 2D 扫描 Gazebo launch
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_3d), launch_arguments=common.items(), condition=IfCondition(PythonExpression(["'", mode, "' == '3d'"]))),  # lidar_mode=3d 时包含 3D 雷达 Gazebo launch
        TimerAction(  # 定时动作：延迟 nav2_delay 秒后启动 Nav2 导航栈
            period=LaunchConfiguration("nav2_delay"),  # Nav2 延迟时间（秒）
            actions=[IncludeLaunchDescription(  # 包含 Nav2 导航栈 launch
                PythonLaunchDescriptionSource(nav2),  # 源文件为 Python 格式的 launch
                launch_arguments={  # 向 Nav2 launch 传入参数
                    "use_rviz": "false",  # 由本 launch 统一管理 RViz，Nav2 阶段不重复启动
                    "map": LaunchConfiguration("map"),  # 地图文件
                }.items(),
            )],
        ),
        TimerAction(  # 定时动作：延迟 rviz_delay 秒后启动 RViz
            # Delay is measured from launch start. Keep RViz independent from
            # nested launch argument scopes and start exactly one instance.
            # （延迟从 launch 启动时刻起算；RViz 独立于嵌套 launch 的参数作用域，保证只启动一个实例）
            period=LaunchConfiguration("rviz_delay"),  # RViz 延迟时间（秒）
            actions=[Node(  # 启动 RViz 可视化
                package="rviz2",  # 所属功能包
                executable="rviz2",  # 可执行文件名
                name="rviz2",  # 节点名
                output="screen",  # 日志输出到屏幕
                arguments=["-d", rviz_config],  # 加载指定 RViz 配置文件
                parameters=[{"use_sim_time": True}],  # 使用仿真时间
                condition=IfCondition(LaunchConfiguration("launch_rviz")),  # 仅当 launch_rviz=true 时才启动
            )],
        ),
    ])
