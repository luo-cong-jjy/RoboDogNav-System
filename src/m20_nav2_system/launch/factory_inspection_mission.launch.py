# ============================================================================
# 文件：factory_inspection_mission.launch.py
# 功能：启动 M20 工厂巡检任务节点（factory_inspection_nav2_mission）。
#       该节点读取巡检途经点参数文件（factory_inspection_midpoints.yaml），
#       在 Nav2 导航栈就绪后，依次向 /m20_nav2/goal_pose_raw 发布目标点，
#       驱动 M20 机器人按途经点顺序完成工厂巡检。
#       注意：本 launch 只启动任务节点，不含仿真/Nav2，
#       需在 Gazebo 仿真与 Nav2 导航栈启动之后单独运行。
# ============================================================================

from pathlib import Path  # 面向对象的路径操作类，用于拼接文件路径

from ament_index_python.packages import get_package_share_directory  # 查询已安装 ROS2 包的 share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument  # 声明可配置的 launch 参数
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("m20_nav2_system"))  # 本包安装后的 share 目录
    default_params = str(pkg_share / "config" / "factory_inspection_midpoints.yaml")  # 默认巡检途经点参数文件

    params_file = LaunchConfiguration("params_file")  # 参数文件路径（可被命令行覆盖）

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),  # 声明巡检参数文件参数，默认使用出厂配置

        Node(  # 启动工厂巡检任务节点
            package="m20_nav2_system",  # 所属功能包
            executable="factory_inspection_nav2_mission",  # 可执行文件名（本包脚本）
            name="factory_inspection_nav2_mission",  # 节点名
            output="screen",  # 日志输出到屏幕
            parameters=[params_file],  # 加载途经点、等待时间等任务参数
        ),
    ])
