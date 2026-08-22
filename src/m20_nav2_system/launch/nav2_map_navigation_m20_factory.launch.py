# ============================================================================
# 文件：nav2_map_navigation_m20_factory.launch.py
# 功能：在工厂场景中启动 Nav2 导航栈（基于已有地图的导航，不做 SLAM）。
#       通过包含 nav2_bringup 的 bringup_launch.py 启动 map_server、planner、
#       controller、recovery 等全部 Nav2 组件（组件容器名 m20_nav2_container，
#       与仓库巡检栈的 Nav2 实例隔离），并可选延迟启动 RViz 可视化。
# ============================================================================

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # 声明参数、包含其他 launch、定时延迟动作
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 参数替换、路径拼接替换
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.substitutions import FindPackageShare  # 运行时查找包的 share 目录


def generate_launch_description():
    # —— 路径准备 ——
    pkg_share = Path(get_package_share_directory("m20_nav2_system"))  # 本包安装后的 share 目录
    nav2_params = str(pkg_share / "config" / "nav2_params.yaml")  # Nav2 全局参数文件（planner/controller/costmap 等）
    default_map = str(pkg_share / "maps" / "factory" / "factory_slam_map.yaml")  # 默认工厂 SLAM 地图文件
    rviz_config = str(pkg_share / "rviz" / "nav2_sandbox.rviz")  # RViz 配置文件

    # —— launch 参数 ——
    use_sim_time = LaunchConfiguration("use_sim_time")  # 是否使用仿真时间
    use_rviz = LaunchConfiguration("use_rviz")  # 是否启动 RViz
    rviz_delay = LaunchConfiguration("rviz_delay")  # RViz 延迟启动时间（秒）
    params_file = LaunchConfiguration("params_file")  # Nav2 参数文件路径
    map_file = LaunchConfiguration("map")  # 地图文件路径（yaml）

    nav2_bringup_launch = PathJoinSubstitution([  # 拼接 nav2_bringup 包中 bringup_launch.py 的完整路径
        FindPackageShare("nav2_bringup"),  # 运行时查找 nav2_bringup 包的 share 目录
        "launch",
        "bringup_launch.py",
    ])

    return LaunchDescription([
        # —— 声明参数默认值 ——
        DeclareLaunchArgument("use_sim_time", default_value="true"),  # 默认使用仿真时间
        DeclareLaunchArgument("use_rviz", default_value="true"),  # 默认启动 RViz
        DeclareLaunchArgument("rviz_delay", default_value="8.0"),  # RViz 延迟 8 秒启动（等 Nav2 全部就绪）
        DeclareLaunchArgument("params_file", default_value=nav2_params),  # Nav2 参数文件默认值
        DeclareLaunchArgument("map", default_value=default_map),  # 地图文件默认值

        # —— 包含 nav2_bringup 完整导航栈 ——
        IncludeLaunchDescription(  # 包含 nav2_bringup 的 bringup_launch.py，启动整套 Nav2 导航栈
            PythonLaunchDescriptionSource(nav2_bringup_launch),  # 源文件为 Python 格式的 launch
            launch_arguments={  # 向 bringup launch 传入的参数
                "slam": "False",  # 关闭 SLAM（使用已有地图做定位导航）
                "map": map_file,  # 要加载的地图文件
                "use_sim_time": use_sim_time,  # 使用仿真时间
                "params_file": params_file,  # Nav2 参数文件
                "autostart": "True",  # 自动启动 Nav2 各生命周期节点（无需手动 configure/activate）
                # Keep this stack isolated from the warehouse inspection
                # stack, which may also run Nav2 in the same ROS domain.
                # （使用独立组件容器名，避免与同 ROS 域内可能并存的仓库巡检 Nav2 实例冲突）
                "container_name": "m20_nav2_container",  # 组件容器名（Nav2 节点共用容器，便于隔离与管理）
            }.items(),
        ),

        # —— 延迟启动 RViz ——
        TimerAction(  # 定时动作：延迟 rviz_delay 秒后启动 RViz
            period=rviz_delay,  # 延迟时间（秒）
            actions=[
                Node(  # 启动 RViz 可视化
                    package="rviz2",  # 所属功能包
                    executable="rviz2",  # 可执行文件名
                    name="rviz2",  # 节点名
                    output="screen",  # 日志输出到屏幕
                    arguments=["-d", rviz_config],  # 加载指定 RViz 配置文件
                    parameters=[{"use_sim_time": use_sim_time}],  # 使用仿真时间
                    condition=IfCondition(use_rviz),  # 仅当 use_rviz=true 时才启动
                ),
            ],
        ),
    ])
