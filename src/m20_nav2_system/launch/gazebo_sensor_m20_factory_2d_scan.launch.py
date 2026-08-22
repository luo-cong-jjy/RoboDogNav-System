# ============================================================================
# 文件：gazebo_sensor_m20_factory_2d_scan.launch.py
# 功能：启动 M20 机器人在工厂场景（factory_environment_v2.world）中的
#       Gazebo Classic 仿真环境，机器人装配轻量二维激光雷达（2D scan）。
#       本 launch 负责：
#         - 配置 Gazebo 插件/模型路径环境变量（GAZEBO_PLUGIN_PATH / GAZEBO_MODEL_PATH）
#         - 启动 gzserver 服务器（可选 gzclient GUI）
#         - 通过 spawn_entity 在指定位姿生成 M20 机器人实体
#         - 发布机器人 TF（robot_state_publisher）与站立形态关节状态
#         - 可选将 3D 点云转换为 2D /scan（默认关闭：2D 模式下雷达直接输出 /scan）
#         - 重写目标点时间戳（goal_pose_restamper，供 Nav2 使用）
#         - 可选启动 RViz 可视化
# ============================================================================

import os  # 操作系统接口：读取环境变量（GAZEBO_PLUGIN_PATH）、获取进程号（os.getpid）等
from pathlib import Path  # 面向对象的路径操作类，用于拼接/判断文件路径

from ament_index_python.packages import get_package_prefix, get_package_share_directory  # 查询已安装 ROS2 包的 prefix/share 目录
from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction  # 启动动作：声明参数、执行进程、包含其他 launch、设置环境变量、定时延迟动作
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作（如 use_rviz 为 true 时启动 RViz）
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点


def _make_gazebo_model_path(pkg_share: Path) -> str:
    """Expose only this package as a Gazebo model path for model:// mesh URIs."""
    # 在 /tmp 下创建以进程号命名的临时目录，作为 Gazebo 模型搜索路径的根目录
    model_path_root = Path("/tmp") / f"m20_nav2_gazebo_model_path_{os.getpid()}"
    model_path_root.mkdir(parents=True, exist_ok=True)  # 递归创建目录；已存在则静默跳过
    model_link = model_path_root / "m20_nav2_system"  # 定义指向本包 share 目录的软链接路径
    if not model_link.exists():  # 仅当软链接尚不存在时才创建，避免重复链接报错
        model_link.symlink_to(pkg_share, target_is_directory=True)  # 建立指向包目录的符号链接（支持 model:// 引用本包资源）
    return str(model_path_root)  # 返回模型路径根目录字符串


def generate_launch_description():
    # —— 路径准备：解析本包 share/prefix 目录并拼接各资源默认路径 ——
    pkg_share = Path(get_package_share_directory("m20_nav2_system"))  # 本包安装后的 share 目录（存放 launch/config/models/worlds 等）
    pkg_prefix = Path(get_package_prefix("m20_nav2_system"))  # 本包安装前缀目录（动态库 lib 目录在其下）
    default_world = str(pkg_share / "worlds" / "factory_environment_v2.world")  # 默认 Gazebo 世界文件：工厂环境 v2
    gazebo_robot_urdf = pkg_share / "models" / "m20_gazebo_combined_2d_scan.urdf"  # Gazebo 仿真用 URDF：四轮站立形态 + 轻量 2D 雷达
    visual_robot_urdf = pkg_share / "models" / "urdf" / "M20_nav_visual.urdf"  # RViz 可视化用 URDF（带雷达的官方外观模型）
    pointcloud_to_scan_launch = pkg_share / "launch" / "rslidar_pointcloud_to_scan.launch.py"  # 点云转激光扫描的 launch 文件路径（本模式下默认不使用）
    rviz_config = str(pkg_share / "rviz" / "nav2_sandbox.rviz")  # RViz 配置文件路径
    plugin_path = str(pkg_prefix / "lib")  # 本包动态库（Gazebo 插件）所在目录
    existing_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")  # 读取系统已有的 Gazebo 插件路径（可能为空）
    gazebo_plugin_path = (  # 合并插件路径：本包 lib 优先，后面追加已有路径
        plugin_path
        if not existing_plugin_path
        else f"{plugin_path}:{existing_plugin_path}"
    )
    gazebo_model_path = _make_gazebo_model_path(pkg_share)  # 生成 Gazebo 模型搜索路径（软链接到本包）

    # —— launch 参数（可被命令行 --ros-args 或 launch 参数覆盖）——
    use_sim_time = LaunchConfiguration("use_sim_time")  # 是否使用仿真时间（订阅 /clock）
    use_rviz = LaunchConfiguration("use_rviz")  # 是否启动 RViz
    use_pointcloud_to_scan = LaunchConfiguration("use_pointcloud_to_scan")  # 是否启用点云转 2D 扫描（2D 模式下默认关闭）
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")  # 是否启动 Gazebo GUI（gzclient）
    gazebo_gui_delay = LaunchConfiguration("gazebo_gui_delay")  # gzclient 延迟启动时间（秒）
    spawn_delay = LaunchConfiguration("spawn_delay")  # 机器人生成（spawn）延迟时间（秒）
    rviz_delay = LaunchConfiguration("rviz_delay")  # RViz 延迟启动时间（秒）
    world = LaunchConfiguration("world")  # 世界文件路径
    x = LaunchConfiguration("x")  # 机器人出生位置 X（米）
    y = LaunchConfiguration("y")  # 机器人出生位置 Y（米）
    z = LaunchConfiguration("z")  # 机器人出生位置 Z（米，抬高到轮子离地高度）
    yaw = LaunchConfiguration("yaw")  # 机器人出生朝向偏航角（弧度）
    cloud_topic = LaunchConfiguration("cloud_topic")  # 3D 点云话题名（仅在启用转换时使用）
    scan_topic = LaunchConfiguration("scan_topic")  # 转换后的 2D 扫描话题名

    # Gazebo 使用四轮站立形态 M20 和轻量二维 /scan 雷达；RViz 使用带雷达的官方外观模型。
    robot_description = visual_robot_urdf.read_text(encoding="utf-8")  # 读取可视化 URDF 内容，作为 robot_description 参数

    return LaunchDescription([
        # —— 声明可配置参数及其默认值 ——
        DeclareLaunchArgument("use_sim_time", default_value="true"),  # 默认使用仿真时间（仿真环境必须为 true）
        DeclareLaunchArgument("use_rviz", default_value="false"),  # 默认不启动 RViz（由上层 launch 统一管理）
        DeclareLaunchArgument("use_pointcloud_to_scan", default_value="false"),  # 2D 模式下雷达直接输出 /scan，默认关闭点云转换
        DeclareLaunchArgument("use_gazebo_gui", default_value="false"),  # 默认无头模式（不启动 GUI，避免 WSL/远程黑屏）
        DeclareLaunchArgument("gazebo_gui_delay", default_value="1.0"),  # gzclient 延迟 1 秒启动，等待服务端就绪
        DeclareLaunchArgument("spawn_delay", default_value="4.0"),  # 延迟 4 秒生成机器人（等 gzserver 启动完成）
        DeclareLaunchArgument("rviz_delay", default_value="5.0"),  # 延迟 5 秒启动 RViz（等 TF/扫描话题就绪）
        DeclareLaunchArgument("world", default_value=default_world),  # 世界文件默认值（工厂环境 v2）
        DeclareLaunchArgument("x", default_value="0.0"),  # 出生位置 X = 0
        DeclareLaunchArgument("y", default_value="0.0"),  # 出生位置 Y = 0
        DeclareLaunchArgument("z", default_value="0.59"),  # 出生高度 Z = 0.59（底盘离地高度）
        DeclareLaunchArgument("yaw", default_value="0.0"),  # 出生朝向偏航角 = 0
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR/POINTS"),  # 雷达点云话题（预留，本模式不使用）
        DeclareLaunchArgument("scan_topic", default_value="/scan"),  # Nav2 使用的 2D 激光话题

        # —— 设置 Gazebo 运行时环境变量 ——
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),  # 指定 Gazebo 插件搜索路径（含本包 lib 目录）
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),  # 指定 Gazebo 模型搜索路径（支持 model:// 引用本包模型）

        # —— 包含点云转扫描 launch（按条件启用，2D 模式下默认不启用）——
        IncludeLaunchDescription(  # 包含 rslidar_pointcloud_to_scan.launch.py：把 3D 点云裁剪为 2D 扫描
            PythonLaunchDescriptionSource(str(pointcloud_to_scan_launch)),  # 源文件为 Python 格式的 launch
            launch_arguments={  # 向被包含 launch 传入的参数
                "use_sim_time": use_sim_time,  # 使用仿真时间
                "cloud_topic": cloud_topic,  # 输入点云话题
                "scan_topic": scan_topic,  # 输出扫描话题
                "range_max": "30.0",  # 最大量程 30 米
            }.items(),
            condition=IfCondition(use_pointcloud_to_scan),  # 仅当 use_pointcloud_to_scan=true 时才包含该 launch
        ),

        # —— 启动 TF 发布节点 ——
        Node(  # robot_state_publisher：根据 robot_description 发布各连杆间 TF（base_link 等）
            package="robot_state_publisher",  # 所属功能包
            executable="robot_state_publisher",  # 可执行文件名
            name="robot_state_publisher",  # 节点名
            output="screen",  # 日志输出到屏幕
            parameters=[
                {"use_sim_time": use_sim_time},  # 使用仿真时间
                {"robot_description": robot_description},  # 机器人 URDF 描述内容
            ],
        ),

        # —— 启动关节状态发布节点 ——
        Node(  # m20_standing_joint_state_publisher：发布站立形态 M20 的关节状态到 /joint_states
            package="m20_nav2_system",  # 所属功能包
            executable="m20_standing_joint_state_publisher",  # 可执行文件名（本包脚本）
            name="m20_standing_joint_state_publisher",  # 节点名
            output="screen",  # 日志输出到屏幕
            parameters=[
                {"use_sim_time": use_sim_time},  # 使用仿真时间
                {"output_topic": "/joint_states"},  # 关节状态发布话题
            ],
        ),

        # —— 启动目标点时间戳重写节点 ——
        Node(  # goal_pose_restamper：把上游目标点话题的时间戳改写为当前时间，供 Nav2 使用
            package="m20_nav2_system",  # 所属功能包
            executable="goal_pose_restamper",  # 可执行文件名（本包脚本）
            name="goal_pose_restamper",  # 节点名
            output="screen",  # 日志输出到屏幕
            parameters=[
                {"use_sim_time": use_sim_time},  # 使用仿真时间
                {"input_topic": "/m20_nav2/goal_pose_raw"},  # 输入：原始目标点话题（如巡检任务发布）
                {"output_topic": "/goal_pose"},  # 输出：重写时间戳后的目标点话题（Nav2 订阅）
                {"use_zero_stamp": True},  # 是否把时间戳清零（配合仿真时钟使用）
            ],
        ),

        # —— 启动 Gazebo 服务器 ——
        ExecuteProcess(  # 启动 Gazebo Classic 服务器 gzserver
            cmd=[
                "gzserver",  # Gazebo 服务器可执行文件
                "--verbose",  # 输出详细日志
                world,  # 世界文件路径
                "-s", "libgazebo_ros_init.so",  # 加载 ROS 初始化插件（提供 /clock 仿真时钟）
                "-s", "libgazebo_ros_factory.so",  # 加载 ROS 工厂插件（支持 spawn 实体服务）
            ],
            output="screen",  # 日志输出到屏幕
        ),

        # —— 延迟启动 Gazebo GUI（可选）——
        TimerAction(  # 定时动作：延迟 gazebo_gui_delay 秒后执行内部动作
            period=gazebo_gui_delay,  # 延迟时间（秒）
            actions=[
                ExecuteProcess(  # 启动 gzclient GUI 客户端
                    cmd=[
                        "gzclient",  # Gazebo 客户端可执行文件
                        "--verbose",  # 输出详细日志
                    ],
                    output="screen",  # 日志输出到屏幕
                    condition=IfCondition(use_gazebo_gui),  # 仅当 use_gazebo_gui=true 时才启动 GUI
                ),
            ],
        ),

        # —— 延迟生成机器人实体 ——
        TimerAction(  # 定时动作：延迟 spawn_delay 秒后生成机器人
            period=spawn_delay,  # 延迟时间（秒）
            actions=[
                ExecuteProcess(  # 调用 gazebo_ros 的 spawn_entity 脚本在仿真中生成机器人
                    cmd=[
                        "/usr/bin/python3",  # Python 解释器路径
                        "/opt/ros/humble/lib/gazebo_ros/spawn_entity.py",  # spawn 实体脚本路径
                        "-file", str(gazebo_robot_urdf),  # 要生成的机器人 URDF 文件
                        "-entity", "m20_nav_proxy",  # 生成的实体名称
                        "-x", x,  # X 坐标
                        "-y", y,  # Y 坐标
                        "-z", z,  # Z 坐标
                        "-Y", yaw,  # 偏航角（大写 Y 表示绕 Z 轴）
                    ],
                    name="spawn_m20_nav_proxy",  # 进程名（便于日志区分）
                    output="screen",  # 日志输出到屏幕
                ),
            ],
        ),

        # —— 延迟启动 RViz（可选）——
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
